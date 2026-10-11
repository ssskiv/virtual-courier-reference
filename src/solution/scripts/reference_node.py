#!/usr/bin/env python3
"""Эталонное решение «Виртуального курьера» (ТОЛЬКО ДЛЯ ОРГАНИЗАТОРОВ).

Использует только разрешённые данные: /odom, /imu, /scan, /camera, /courier/cargo_count.
Никаких топиков судьи.

Идея:
  * локализация: путь по энкодерам (/odom) + курс с IMU (не дрейфует) — точнее, чем
    slam_toolbox, который «проскальзывает» вдоль длинных одинаковых коридоров;
  * карта занятости строится самим узлом по /scan (log-odds), кадр «map» = стартовая поза;
    старт — центр ячейки, курс кратен 90° ⇒ центры ячеек лежат в точках (2i, 2j) карты;
  * поиск станций — камерой по цветному полу (A — жёлтый, B — голубой, C — пурпурный):
    пиксели пятна проецируются на плоскость пола и привязываются к центру ячейки;
  * исследование — к ближайшей по графу ячейке, которую камера ещё не видела;
  * движение — свой A* по раздутой карте занятости + pure pursuit + защита по лидару;
  * на каждой найденной станции стоим, пока не придёт груз (или 6 с — станция неактивна);
  * все грузы везём одним рейсом, у Старта встаём кормой к стене с ArUco №0 и наклоняем лоток.
"""

import heapq
import math
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy import ndimage
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Float64, UInt8
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

CELL = 2.0
GRID_N = 8                      # полигон 8×8 ячеек
ROBOT_R = 0.175
PLAN_RES = 0.1                  # м/пиксель плановой сетки
LETHAL = ROBOT_R + 0.12         # ближе к препятствию центр робота не планируем
SOFT = 0.65                     # до этой дистанции — штраф (держимся середины коридора)
MAX_V = 0.55
MAX_W = 1.2                     # быстрее — сканы смазываются, SLAM теряет согласованность
LOOKAHEAD = 0.5
TIME_LIMIT = 600.0              # с — лимит заезда
HOME_SPEED = 0.3                # м/с — средняя скорость по пути домой (с поворотами), с запасом
HOME_MARGIN = 35.0              # с — на разворот, задний ход и выгрузку
DWELL_TIMEOUT = 6.0             # с ожидания груза на станции
UNLOAD_DIST = 0.60              # м от линии стены до центра робота при выгрузке
TRAY_UNLOAD = 1.0

# HSV (OpenCV: H 0..180, S/V 0..255) пола станций — откалибровано по кадрам камеры в Webots:
# A ≈ (24, 160), B ≈ (98, 155), C ≈ (149, 122). Паллеты (H≈16, S≈150) отсекаются по H.
STATION_HSV = {'A': ((20, 120, 90), (30, 255, 255)),
               'B': ((90, 110, 90), (106, 255, 255)),
               'C': ((140, 90, 90), (160, 255, 255))}
ARUCO = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
MARKER_SIZE = 0.18              # м: чёрный квадрат маркера (табличка 0.24 м с белой рамкой)
MARKER_STATION = {1: 'A', 2: 'B', 3: 'C'}
CONFIRM = 4.0                   # суммарный вес голосов, при котором станция считается найденной

MAP_RES = 0.05
MAP_HALF = 18.0                 # карта ±18 м от старта: полигон 16 м может лежать в любую сторону
LASER_X = 0.09                  # лидар впереди центра робота
CAMERA_X, CAMERA_Z = 0.178, 0.07
L_OCC, L_FREE, L_MAX = 0.9, -0.35, 4.0
L_STICKY = 1.5
# Поворот base_link → camera_optical_frame: z_opt = x_base, x_opt = -y_base, y_opt = -z_base.
R_BASE_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def cell_of(x, y):
    return (int(round(x / CELL)), int(round(y / CELL)))


def cell_center(c):
    return (c[0] * CELL, c[1] * CELL)


class Planner:
    """A* по раздутой карте занятости (неизвестное считается свободным)."""

    def __init__(self):
        self.cost = None
        self.occ = None          # исходная карта (-1/0/100) в разрешении PLAN_RES
        self.ox = self.oy = 0.0
        self.w = self.h = 0

    def update(self, logodds, origin, res):
        k = max(1, int(round(PLAN_RES / res)))
        h, w = logodds.shape[0] // k * k, logodds.shape[1] // k * k
        raw = logodds[:h, :w].reshape(h // k, k, w // k, k)
        occupied = (raw > 0.6).any(axis=(1, 3))
        known = occupied | (raw < -0.6).any(axis=(1, 3))
        self.occ = np.where(occupied, 100, np.where(known, 0, -1)).astype(np.int16)
        self.h, self.w = self.occ.shape
        self.ox, self.oy = origin
        dist = ndimage.distance_transform_edt(~occupied) * PLAN_RES
        cost = 1.0 + 6.0 * np.clip((SOFT - dist) / (SOFT - LETHAL), 0.0, 1.0)
        cost[~known] += 0.3
        relaxed = cost.copy()
        relaxed[dist < ROBOT_R + 0.03] = np.inf
        cost[dist < LETHAL] = np.inf
        self.cost = cost
        self.cost_relaxed = relaxed       # запасной вариант для узких мест

    def to_px(self, x, y):
        return int((x - self.ox) / PLAN_RES), int((y - self.oy) / PLAN_RES)

    def to_xy(self, i, j):
        return self.ox + (i + 0.5) * PLAN_RES, self.oy + (j + 0.5) * PLAN_RES

    def inside(self, i, j):
        return 0 <= i < self.w and 0 <= j < self.h

    def occupied(self, x, y):
        i, j = self.to_px(x, y)
        return self.inside(i, j) and self.occ[j, i] == 100

    def known_free(self, x, y):
        i, j = self.to_px(x, y)
        return self.inside(i, j) and self.occ[j, i] == 0

    def line_clear(self, x0, y0, x1, y1):
        n = int(math.hypot(x1 - x0, y1 - y0) / (PLAN_RES / 2)) + 1
        for t in np.linspace(0, 1, n):
            if self.occupied(x0 + t * (x1 - x0), y0 + t * (y1 - y0)):
                return False
        return True

    def free_near(self, x, y, radius=0.7):
        """Ближайшая к (x, y) проходимая точка (цель может попасть в паллету)."""
        i0, j0 = self.to_px(x, y)
        best, bd = None, 1e9
        r = int(radius / PLAN_RES)
        for j in range(j0 - r, j0 + r + 1):
            for i in range(i0 - r, i0 + r + 1):
                if self.inside(i, j) and np.isfinite(self.cost[j, i]):
                    d = (i - i0) ** 2 + (j - j0) ** 2
                    if d < bd:
                        best, bd = (i, j), d
        if best is None and not self.inside(i0, j0):
            return x, y           # вне карты — неизвестность, пусть A* разбирается
        return None if best is None else self.to_xy(*best)

    def plan(self, start, goal):
        path = self._plan(start, goal, self.cost)
        if path is None and self.cost is not None:
            path = self._plan(start, goal, self.cost_relaxed)
        return path

    def _plan(self, start, goal, cost):
        if cost is None:
            return None
        si, sj = self.to_px(*start)
        gi, gj = self.to_px(*goal)
        # Цель вне карты: расширяем рамку неизвестным пространством.
        pad = 0
        for i, j in ((si, sj), (gi, gj)):
            pad = max(pad, -i, -j, i - self.w + 1, j - self.h + 1)
        if pad > 0:
            pad += 5
            cost = np.pad(cost, pad, constant_values=1.3)
            si, sj, gi, gj = si + pad, sj + pad, gi + pad, gj + pad
        H, W = cost.shape
        cost = cost.copy()
        # Старт может оказаться у стены (выгрузка) — освобождаем пятно вокруг робота.
        r = int(LETHAL / PLAN_RES) + 1
        cost[max(0, sj - r):sj + r + 1, max(0, si - r):si + r + 1] = np.where(
            np.isinf(cost[max(0, sj - r):sj + r + 1, max(0, si - r):si + r + 1]), 3.0,
            cost[max(0, sj - r):sj + r + 1, max(0, si - r):si + r + 1])
        if not (0 <= gi < W and 0 <= gj < H) or not np.isfinite(cost[gj, gi]):
            return None
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        g = {(si, sj): 0.0}
        parent = {}
        openq = [(0.0, si, sj)]
        closed = set()
        while openq:
            _, i, j = heapq.heappop(openq)
            if (i, j) in closed:
                continue
            if (i, j) == (gi, gj):
                break
            closed.add((i, j))
            gc = g[(i, j)]
            for di, dj, dc in moves:
                ni, nj = i + di, j + dj
                if not (0 <= ni < W and 0 <= nj < H):
                    continue
                c = cost[nj, ni]
                if not np.isfinite(c):
                    continue
                ng = gc + dc * c
                if ng < g.get((ni, nj), 1e18):
                    g[(ni, nj)] = ng
                    parent[(ni, nj)] = (i, j)
                    heapq.heappush(openq, (ng + math.hypot(gi - ni, gj - nj), ni, nj))
        else:
            return None
        if (gi, gj) not in g:
            return None
        path = [(gi, gj)]
        while path[-1] != (si, sj):
            path.append(parent[path[-1]])
        path.reverse()
        ox, oy = self.ox - pad * PLAN_RES, self.oy - pad * PLAN_RES
        return [(ox + (i + 0.5) * PLAN_RES, oy + (j + 0.5) * PLAN_RES) for i, j in path]


class Reference(Node):
    def __init__(self):
        super().__init__('reference')
        self.declare_parameter('gui', False)
        self.gui = self.get_parameter('gui').value
        self.declare_parameter('debug_dir', '')
        self.debug_dir = self.get_parameter('debug_dir').value

        self.bridge = CvBridge()
        self.planner = Planner()

        self.scan = None
        self.K = None
        self.cargo = 0
        self.pose = None                 # (x, y, yaw) в map
        self.history = deque(maxlen=200) # (t, x, y, yaw) — поза на момент кадра/скана
        self.prev_odom = None
        self.imu_yaw = None
        self.imu_yaw0 = None
        n = int(2 * MAP_HALF / MAP_RES)
        self.logodds = np.zeros((n, n), np.float32)
        self.map_origin = (-MAP_HALF, -MAP_HALF)
        self.last_map_update = -1e9
        self.start_truth = None
        self.t0 = None

        # Знания о полигоне
        self.unload_dir = None           # единичный вектор от центра старта к стене выгрузки
        self.spin_votes = []             # курсы, при которых ArUco 0 был по центру кадра
        self.station_votes = {}          # (sid, cell) → суммарный вес голосов
        self.stations = {}               # sid → cell
        self.handled = {}                # sid → 'loaded' | 'empty'
        self.seen = {(0, 0)}             # ячейки, пол которых видела камера
        self.unreachable = {}            # cell → время, до которого не пробовать

        # Движение
        self.path = None
        self.goal = None
        self.goal_tol = 0.2
        self.last_plan_t = -1e9
        self.progress = (0.0, None)      # (время, позиция) последнего заметного продвижения
        self.backup_until = 0.0

        self.home_eta = 0.0              # оценка времени до выгрузки, с
        self.last_eta_t = -1e9

        self.mode = 'INIT'
        self.mode_t = 0.0
        self.target_station = None
        self.cargo_before = 0

        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(UInt8, '/courier/cargo_count', self.on_cargo, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tray_pub = self.create_publisher(Float64, '/courier/tray_cmd', 10)
        self.create_timer(0.1, self.loop)
        if self.gui:
            # Визуализация для RViz: своя карта, TF map→odom, путь, станции, ячейки.
            self.tf_pub = TransformBroadcaster(self)
            self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
            self.path_pub = self.create_publisher(Path, '/reference/path', 1)
            self.marker_pub = self.create_publisher(MarkerArray, '/reference/markers', 1)
            self.create_timer(1.0, self.publish_viz)
        if self.debug_dir:
            # ТОЛЬКО ДЛЯ ОТЛАДКИ: истинная поза от судьи сравнивается с оценкой в логе.
            # В управлении не используется; на зачётном прогоне debug_dir пуст.
            import json
            from std_msgs.msg import String
            self.truth = None
            def on_truth(m):
                self.truth = json.loads(m.data)['robot']
                if self.t0 is None:                 # поза до первого движения = начало map
                    self.start_truth = self.truth
            self.create_subscription(String, '/referee/state', on_truth, 10)
            self.create_timer(5.0, self.debug_dump)
        self.log('эталонное решение запущено')

    # ================================================================ утилиты

    def log(self, text):
        self.get_logger().info(text)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self):
        return 0.0 if self.t0 is None else self.now() - self.t0

    def drive(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.cmd_pub.publish(m)

    def set_mode(self, mode):
        if mode != self.mode:
            self.log(f'[{self.elapsed():5.1f} с] {self.mode} → {mode}')
        self.mode = mode
        self.mode_t = self.now()

    def pose_at(self, t):
        """Поза на момент t (ближайшая из истории)."""
        if not self.history:
            return self.pose
        best = min(self.history, key=lambda h: abs(h[0] - t))
        return best[1:]

    def sector_min(self, a0, a1):
        s = self.scan
        if s is None:
            return float('inf')
        n = len(s.ranges)
        best = float('inf')
        for deg in range(a0, a1 + 1):
            i = int(round((math.radians(deg) - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[i]
            if s.range_min <= r < best:
                best = r
        return best

    def debug_dump(self):
        """Отладка: состояние в лог и картинка карты с ячейками в debug_dir."""
        if self.pose is None or self.planner.cost is None:
            return
        votes = {f'{k[0]}{k[1]}': v for k, v in self.station_votes.items()}
        tr = getattr(self, 'truth', None)
        if tr and self.start_truth:
            st = self.start_truth
            c, s_ = math.cos(-st['yaw']), math.sin(-st['yaw'])
            dx, dy = tr['x'] - st['x'], tr['y'] - st['y']
            self.log(f'DBG truth(map)=({c * dx - s_ * dy:.2f},{s_ * dx + c * dy:.2f}) est=({self.pose[0]:.2f},{self.pose[1]:.2f})')
        self.log(f'DBG t={self.elapsed():.0f} mode={self.mode} pose=({self.pose[0]:.2f},{self.pose[1]:.2f},'
                 f'{math.degrees(self.pose[2]):.0f}°) cell={cell_of(*self.pose[:2])} '
                 f'target={getattr(self, "explore_cell", None)} seen={len(self.seen)} '
                 f'path={len(self.path) if self.path else None} votes={votes}')
        p = self.planner
        img = np.full((p.h, p.w, 3), 128, np.uint8)
        img[p.occ == 0] = (255, 255, 255)
        img[p.occ == 100] = (0, 0, 0)
        img[np.isinf(p.cost) & (p.occ != 100)] = (180, 180, 255)
        for (i, j) in self.seen:
            x, y = cell_center((i, j))
            u, v = p.to_px(x, y)
            cv2.circle(img, (u, v), 3, (0, 160, 0), -1)
        for c, t in self.unreachable.items():
            u, v = p.to_px(*cell_center(c))
            cv2.circle(img, (u, v), 4, (0, 0, 255), 1)
        if self.path:
            for x, y in self.path:
                u, v = p.to_px(x, y)
                if p.inside(u, v):
                    img[v, u] = (255, 0, 0)
        u, v = p.to_px(*self.pose[:2])
        cv2.circle(img, (u, v), 2, (255, 0, 255), -1)
        cv2.imwrite(f'{self.debug_dir}/map.png', cv2.resize(np.flipud(img), None, fx=3, fy=3,
                                                         interpolation=cv2.INTER_NEAREST))

    # ================================================================ колбэки

    def on_imu(self, msg):
        self.imu_yaw = yaw_of(msg.orientation)
        if self.imu_yaw0 is None:
            self.imu_yaw0 = self.imu_yaw

    def on_odom(self, msg):
        """Путь — по энкодерам (/odom), курс — с IMU (без дрейфа)."""
        if self.imu_yaw is None:
            return
        p = msg.pose.pose.position
        if self.prev_odom is None:
            self.prev_odom = (p.x, p.y)
            self.pose = (0.0, 0.0, 0.0)
        ds = math.hypot(p.x - self.prev_odom[0], p.y - self.prev_odom[1])
        if msg.twist.twist.linear.x < 0:
            ds = -ds
        self.prev_odom = (p.x, p.y)
        x, y, _ = self.pose
        yaw = wrap(self.imu_yaw - self.imu_yaw0)
        self.pose = (x + ds * math.cos(yaw), y + ds * math.sin(yaw), yaw)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.history.append((t,) + self.pose)
        if self.gui:
            self.publish_map_odom_tf(msg)

    def on_scan(self, msg):
        self.scan = msg
        if self.pose is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x, y, yaw = self.pose_at(t)
        lx, ly = x + LASER_X * math.cos(yaw), y + LASER_X * math.sin(yaw)
        r = np.asarray(msg.ranges, np.float32)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment + yaw
        # Ближе 0.3 м сзади — сам робот с грузом на лотке, в карту не пишем.
        hit = np.isfinite(r) & (r < msg.range_max - 0.05) & (r > 0.3)
        r = np.where(np.isfinite(r), np.minimum(r, msg.range_max), msg.range_max)
        # Свободные клетки вдоль лучей.
        steps = np.arange(0.0, msg.range_max, MAP_RES)
        rr = steps[None, :]
        mask = rr < (r[:, None] - 0.08)
        fx = (lx + rr * np.cos(a)[:, None])[mask]
        fy = (ly + rr * np.sin(a)[:, None])[mask]
        n = self.logodds.shape[0]
        fi = ((fx - self.map_origin[0]) / MAP_RES).astype(np.int32)
        fj = ((fy - self.map_origin[1]) / MAP_RES).astype(np.int32)
        ok = (fi >= 0) & (fi < n) & (fj >= 0) & (fj < n)
        free = np.zeros_like(self.logodds, dtype=bool)
        free[fj[ok], fi[ok]] = True
        # Полигон статичен: уверенно занятые клетки не «стираем» (лучи вскользь проходят сквозь
        # клетки тонких стен и иначе постепенно делают их свободными).
        free &= self.logodds < L_STICKY
        self.logodds[free] += L_FREE
        # Занятые — концы лучей.
        hx = lx + r[hit] * np.cos(a[hit])
        hy = ly + r[hit] * np.sin(a[hit])
        hi = ((hx - self.map_origin[0]) / MAP_RES).astype(np.int32)
        hj = ((hy - self.map_origin[1]) / MAP_RES).astype(np.int32)
        ok = (hi >= 0) & (hi < n) & (hj >= 0) & (hj < n)
        occ = np.zeros_like(free)
        occ[hj[ok], hi[ok]] = True
        self.logodds[occ] += L_OCC - L_FREE * free[occ]
        np.clip(self.logodds, -L_MAX, L_MAX, out=self.logodds)
        if t - self.last_map_update > 1.0:
            self.last_map_update = t
            self.planner.update(self.logodds, self.map_origin, MAP_RES)

    def on_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def on_cargo(self, msg):
        if msg.data != self.cargo:
            self.log(f'[{self.elapsed():5.1f} с] грузов на лотке: {msg.data}')
        self.cargo = msg.data

    def on_image(self, msg):
        if self.K is None or self.pose is None:
            return
        x, y, yaw = self.pose_at(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        c, s_ = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]]) @ R_BASE_OPT
        t = np.array([x + CAMERA_X * c, y + CAMERA_X * s_, CAMERA_Z])
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), ARUCO)
        markers = [] if ids is None else list(zip(corners, ids.ravel().tolist()))
        self.find_unload_marker(markers)
        self.find_station_markers(markers, R, t)
        self.find_stations(img, R, t)
        self.mark_seen(R, t)
        if self.gui:
            for c, i in markers:
                pts = c.reshape(4, 2).astype(int)
                cv2.polylines(img, [pts], True, (0, 255, 0), 2)
                cv2.putText(img, {0: 'FINISH', 1: 'A', 2: 'B', 3: 'C'}.get(i, str(i)), tuple(pts[0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(img, f'{self.mode}  cargo={self.cargo}  stations={sorted(self.stations)}', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow('reference camera', img)
            cv2.waitKey(1)

    # ================================================================ визуализация

    def publish_map_odom_tf(self, odom):
        """map→odom = (map→base) · (odom→base)⁻¹, чтобы RViz показывал робота на нашей карте."""
        q = odom.pose.pose.orientation
        oyaw = yaw_of(q)
        ox, oy = odom.pose.pose.position.x, odom.pose.pose.position.y
        x, y, yaw = self.pose
        d = yaw - oyaw
        tf = TransformStamped()
        tf.header.stamp = odom.header.stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = x - (math.cos(d) * ox - math.sin(d) * oy)
        tf.transform.translation.y = y - (math.sin(d) * ox + math.cos(d) * oy)
        tf.transform.rotation.z = math.sin(d / 2)
        tf.transform.rotation.w = math.cos(d / 2)
        self.tf_pub.sendTransform(tf)

    def marker(self, mid, ns, mtype, x, y, z, sx, sy, sz, rgba, text=''):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.text = text
        return m

    def publish_viz(self):
        p = self.planner
        if p.occ is None:
            return
        stamp = self.get_clock().now().to_msg()
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = stamp
        grid.info.resolution = PLAN_RES
        grid.info.width, grid.info.height = p.w, p.h
        grid.info.origin.position.x, grid.info.origin.position.y = p.ox, p.oy
        grid.info.origin.orientation.w = 1.0
        grid.data = p.occ.astype(np.int8).ravel().tolist()
        self.map_pub.publish(grid)

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for x, y in (self.path or []):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

        colors = {'A': (0.95, 0.75, 0.1, 0.9), 'B': (0.1, 0.7, 0.9, 0.9), 'C': (0.85, 0.3, 0.85, 0.9)}
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        k = 0
        for (i, j) in self.seen:
            x, y = cell_center((i, j))
            arr.markers.append(self.marker(k, 'seen', Marker.CYLINDER, x, y, 0.01, 0.25, 0.25, 0.02, (0.1, 0.8, 0.2, 0.6)))
            k += 1
        for sid, cell in self.stations.items():
            x, y = cell_center(cell)
            state = self.handled.get(sid, 'найдена')
            state = {'loaded': 'груз взят', 'empty': 'неактивна'}.get(state, state)
            arr.markers.append(self.marker(k, 'stations', Marker.CUBE, x, y, 0.02, 1.4, 1.4, 0.04, colors[sid]))
            arr.markers.append(self.marker(k + 1, 'stations', Marker.TEXT_VIEW_FACING, x, y, 0.8, 0, 0, 0.45,
                                           (1.0, 1.0, 1.0, 1.0), f'{sid}: {state}'))
            k += 2
        target = None
        if self.mode == 'EXPLORE':
            target = cell_center(self.explore_cell)
        elif self.mode in ('TO_STATION', 'DWELL') and self.target_station in self.stations:
            target = cell_center(self.stations[self.target_station])
        elif self.mode in ('HOME', 'FACE', 'BACKUP', 'UNLOAD'):
            target = (0.0, 0.0)
        if target:
            arr.markers.append(self.marker(k, 'target', Marker.SPHERE, target[0], target[1], 0.3, 0.35, 0.35, 0.35,
                                           (1.0, 0.2, 0.2, 0.9)))
            k += 1
        if self.unload_dir:
            ux, uy = self.unload_dir
            arr.markers.append(self.marker(k, 'unload', Marker.CUBE, ux * 0.7, uy * 0.7, 0.01,
                                           0.6 if ux else 1.0, 1.0 if ux else 0.6, 0.02, (1.0, 1.0, 1.0, 0.8)))
            k += 1
        if self.pose:
            arr.markers.append(self.marker(k, 'status', Marker.TEXT_VIEW_FACING, self.pose[0], self.pose[1], 1.2,
                                           0, 0, 0.4, (1.0, 1.0, 0.3, 1.0),
                                           f'{self.mode}  t={self.elapsed():.0f} с  груз={self.cargo}'))
        self.marker_pub.publish(arr)

    # ================================================================ восприятие

    def find_unload_marker(self, markers):
        if self.unload_dir is not None or self.mode != 'SPIN':
            return
        for c, i in markers:
            if i == 0 and abs(c.reshape(4, 2)[:, 0].mean() - self.K[0, 2]) < 80:
                self.spin_votes.append(self.pose[2])

    def vote(self, sid, cell, weight):
        if sid in self.handled or cell == (0, 0):
            return
        key = (sid, cell)
        if self.debug_dir and self.station_votes.get(key, 0.0) < 3:
            self.log(f'DBG vote {sid} {cell} w={weight} {self.vote_src} pose=({self.pose[0]:.2f},{self.pose[1]:.2f})')
        self.station_votes[key] = self.station_votes.get(key, 0.0) + weight
        best = max(((w, c) for (s, c), w in self.station_votes.items() if s == sid))
        if best[0] >= CONFIRM and self.stations.get(sid) != best[1]:
            what = 'найдена' if sid not in self.stations else 'уточнена'
            self.stations[sid] = best[1]
            self.log(f'[{self.elapsed():5.1f} с] {what} станция {sid}: ячейка {best[1]}')

    def find_station_markers(self, markers, R, t):
        """ArUco 1–3 на внешней стене: центр станции — в 1 м от маркера вглубь полигона."""
        for c, i in markers:
            sid = MARKER_STATION.get(i)
            if sid is None:
                continue
            _, tvec, _ = cv2.aruco.estimatePoseSingleMarkers([c], MARKER_SIZE, self.K, np.zeros(5))
            pm = R @ tvec.reshape(3) + t
            dx, dy = t[0] - pm[0], t[1] - pm[1]
            dist = math.hypot(dx, dy)
            if dist > 5.0:
                continue
            # Нормаль стены — ось сетки, ближайшая к направлению «маркер → камера».
            if abs(dx) > abs(dy):
                nx, ny = math.copysign(1.0, dx), 0.0
            else:
                nx, ny = 0.0, math.copysign(1.0, dy)
            cx, cy = pm[0] + nx * CELL / 2, pm[1] + ny * CELL / 2
            cell = cell_of(cx, cy)
            ccx, ccy = cell_center(cell)
            if math.hypot(cx - ccx, cy - ccy) < 0.6:
                self.vote_src = f'aruco d={dist:.2f} pm=({pm[0]:.2f},{pm[1]:.2f}) tvec={np.round(tvec.ravel(), 2)}'
                self.vote(sid, cell, 2.0 if dist < 3.0 else 1.0)

    def floor_point(self, u, v, R, t):
        """Пересечение луча пикселя с полом (z = 0) в кадре map."""
        ray = R @ np.array([(u - self.K[0, 2]) / self.K[0, 0], (v - self.K[1, 2]) / self.K[1, 1], 1.0])
        if ray[2] >= -1e-3:
            return None
        s = -t[2] / ray[2]
        return t[0] + s * ray[0], t[1] + s * ray[1], s

    def find_stations(self, img, R, t):
        horizon = int(self.K[1, 2]) + 3
        hsv = cv2.cvtColor(img[horizon:], cv2.COLOR_BGR2HSV)
        for sid, (lo, hi) in STATION_HSV.items():
            if sid in self.handled:
                continue
            mask = cv2.inRange(hsv, lo, hi)
            n, _, stats, cent = cv2.connectedComponentsWithStats(mask)
            for k in range(1, n):
                if stats[k, cv2.CC_STAT_AREA] < 60:
                    continue
                # Ближняя к роботу (нижняя) кромка пятна проецируется точнее всего.
                u = cent[k][0]
                v = stats[k, cv2.CC_STAT_TOP] + stats[k, cv2.CC_STAT_HEIGHT] - 1 + horizon
                p = self.floor_point(u, v, R, t)
                # Камера в 7 см от пола: дальняя проекция очень чувствительна к наклону корпуса.
                if p is None or p[2] > 1.5:
                    continue
                # Кромка квадрата 1.4 м — сдвигаем точку на 0.5 м дальше вдоль луча к центру квадрата.
                dx, dy = p[0] - t[0], p[1] - t[1]
                d = math.hypot(dx, dy)
                cx, cy = p[0] + 0.5 * dx / d, p[1] + 0.5 * dy / d
                cell = cell_of(cx, cy)
                ccx, ccy = cell_center(cell)
                if math.hypot(cx - ccx, cy - ccy) > 0.8:
                    continue
                self.vote_src = f'floor d={p[2]:.2f} p=({p[0]:.2f},{p[1]:.2f}) uv=({u:.0f},{v})'
                self.vote(sid, cell, 0.5)

    def mark_seen(self, R, t):
        if self.planner.cost is None:
            return
        Rt = R.T
        ci, cj = cell_of(t[0], t[1])
        for i in range(ci - 3, ci + 4):
            for j in range(cj - 3, cj + 4):
                if (i, j) in self.seen:
                    continue
                x, y = cell_center((i, j))
                pc = Rt @ (np.array([x, y, 0.0]) - t)
                if pc[2] < 0.4 or pc[2] > 4.5 or abs(pc[0] / pc[2]) > 0.5:
                    continue
                known = any(self.planner.known_free(x + ox, y + oy)
                            for ox in (-0.3, 0.0, 0.3) for oy in (-0.3, 0.0, 0.3))
                if known and self.planner.line_clear(t[0], t[1], x, y):
                    self.seen.add((i, j))

    # ================================================================ граф ячеек

    def edge_state(self, a, b):
        """'wall' | 'open' | 'unknown' между соседними ячейками по карте."""
        ax, ay = cell_center(a)
        bx, by = cell_center(b)
        mx, my = (ax + bx) / 2, (ay + by) / 2
        horiz = a[1] == b[1]          # соседи по x — стена вертикальная
        occ = free = 0
        for s in np.arange(-0.6, 0.61, 0.1):
            for d in (-0.1, 0.0, 0.1):
                x, y = (mx + d, my + s) if horiz else (mx + s, my + d)
                if self.planner.occupied(x, y):
                    occ += 1
                elif self.planner.known_free(x, y):
                    free += 1
        if occ >= 4:
            return 'wall'
        return 'open' if free >= 20 else 'unknown'

    def bounds(self):
        """Допустимый диапазон индексов ячеек (полигон 8×8, старт у стены выгрузки)."""
        cells = list(self.seen)
        lo_i = min(c[0] for c in cells)
        hi_i = max(c[0] for c in cells)
        lo_j = min(c[1] for c in cells)
        hi_j = max(c[1] for c in cells)
        bi = [hi_i - (GRID_N - 1), lo_i + (GRID_N - 1)]
        bj = [hi_j - (GRID_N - 1), lo_j + (GRID_N - 1)]
        if self.unload_dir is not None:
            ux, uy = self.unload_dir
            if ux > 0.5:
                bi[1] = 0
            elif ux < -0.5:
                bi[0] = 0
            elif uy > 0.5:
                bj[1] = 0
            elif uy < -0.5:
                bj[0] = 0
        return bi, bj

    def explore_target(self):
        """Ближайшая по графу ячейка, которую камера ещё не видела."""
        start = cell_of(self.pose[0], self.pose[1])
        bi, bj = self.bounds()
        now = self.now()
        dist = {start: 0}
        q = [(0, start)]
        while q:
            d, c = heapq.heappop(q)
            if d > dist[c]:
                continue
            if c not in self.seen and self.unreachable.get(c, 0) < now:
                return c
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (c[0] + di, c[1] + dj)
                if not (bi[0] <= n[0] <= bi[1] and bj[0] <= n[1] <= bj[1]):
                    continue
                if self.edge_state(c, n) == 'wall':
                    continue
                if d + 1 < dist.get(n, 1e9):
                    dist[n] = d + 1
                    heapq.heappush(q, (d + 1, n))
        return None

    # ================================================================ движение

    def go_to(self, x, y, tol=0.2):
        """Шаг движения к точке. True — приехали."""
        px, py, yaw = self.pose
        if math.hypot(x - px, y - py) < tol:
            self.drive(0, 0)
            return True
        now = self.now()
        if self.goal is None or math.hypot(self.goal[0] - x, self.goal[1] - y) > 0.05:
            self.goal, self.path, self.last_plan_t = (x, y), None, -1e9
            self.progress = (now, (px, py))
        if self.path is None or now - self.last_plan_t > 1.5:
            self.last_plan_t = now
            target = self.planner.free_near(x, y)
            path = self.planner.plan((px, py), target) if target else None
            if path is None:
                self.path = None
                self.drive(0, 0)
                if self.debug_dir and now - getattr(self, 'last_fail_dump', -1e9) > 10:
                    self.last_fail_dump = now
                    np.savez(f'{self.debug_dir}/fail_{int(self.elapsed())}.npz', cost=self.planner.cost,
                             occ=self.planner.occ, origin=(self.planner.ox, self.planner.oy),
                             start=(px, py), goal=(x, y), target=target if target else (np.nan, np.nan))
                return None              # недостижимо
            self.path = path

        # Застревание: нет продвижения 6 с → отъезд назад.
        if math.hypot(px - self.progress[1][0], py - self.progress[1][1]) > 0.3:
            self.progress = (now, (px, py))
        elif now - self.progress[0] > 6.0:
            self.backup_until = now + 1.2
            self.progress = (now, (px, py))
            self.path = None
        if now < self.backup_until:
            if self.sector_min(150, 210) > 0.3:
                self.drive(-0.15, 0.0)
            else:
                self.drive(0.0, 0.6)      # сзади тоже тесно — разворачиваемся на месте
            return False

        # pure pursuit
        pts = self.path
        k = min(range(len(pts)), key=lambda i: (pts[i][0] - px) ** 2 + (pts[i][1] - py) ** 2)
        look = pts[-1]
        for p in pts[k:]:
            if math.hypot(p[0] - px, p[1] - py) >= LOOKAHEAD:
                look = p
                break
        err = wrap(math.atan2(look[1] - py, look[0] - px) - yaw)
        w = max(-MAX_W, min(MAX_W, 2.5 * err))
        if abs(err) > 0.9:
            v = 0.0
        else:
            v = MAX_V * math.cos(err) ** 2
        dgoal = math.hypot(x - px, y - py)
        v = min(v, 0.15 + 0.6 * dgoal)
        if self.cargo:
            v = min(v, 0.45)
        if self.sector_min(-20, 20) < 0.22 and v > 0:
            v = 0.0
            self.path = None
        self.drive(v, w)
        return False

    def turn_to(self, yaw_goal):
        err = wrap(yaw_goal - self.pose[2])
        if abs(err) < 0.03:
            self.drive(0, 0)
            return True
        self.drive(0, max(-1.2, min(1.2, 2.5 * err)))
        return False

    # ================================================================ миссия

    def time_to_go_home(self):
        """Пора домой: груз есть, а времени осталось только на дорогу и выгрузку."""
        if not self.cargo:
            return False
        now = self.now()
        if now - self.last_eta_t > 5.0:
            self.last_eta_t = now
            path = self.planner.plan(self.pose[:2], (0.0, 0.0))
            if path:
                length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
                self.home_eta = length / HOME_SPEED + HOME_MARGIN
        return self.elapsed() + self.home_eta > TIME_LIMIT - 10.0

    def next_station(self):
        best, bd = None, 1e9
        for sid, cell in self.stations.items():
            if sid in self.handled:
                continue
            x, y = cell_center(cell)
            d = math.hypot(x - self.pose[0], y - self.pose[1])
            if d < bd:
                best, bd = sid, d
        return best

    def loop(self):
        if self.pose is None or self.scan is None or self.planner.cost is None:
            return
        if self.t0 is None:
            self.t0 = self.now()
            self.tray_pub.publish(Float64(data=0.0))
            self.set_mode('SPIN')
            self.spin_start_yaw = self.pose[2]
            self.spin_acc, self.spin_prev = 0.0, self.pose[2]

        if self.mode == 'SPIN':
            # Полный оборот: ищем стену выгрузки (ArUco 0) и заодно станции рядом.
            self.spin_acc += abs(wrap(self.pose[2] - self.spin_prev))
            self.spin_prev = self.pose[2]
            if self.spin_acc < 2 * math.pi + 0.2:
                self.drive(0, 1.0)
                return
            self.drive(0, 0)
            if self.spin_votes:
                a = math.atan2(sum(math.sin(v) for v in self.spin_votes), sum(math.cos(v) for v in self.spin_votes))
                a = round(a / (math.pi / 2)) * (math.pi / 2)
                self.unload_dir = (round(math.cos(a)), round(math.sin(a)))
                self.log(f'стена выгрузки: направление {self.unload_dir}')
            else:
                self.log('ArUco 0 не найден при развороте — определю по ходу')
            self.set_mode('DECIDE')
            return

        if self.mode == 'DECIDE':
            if self.time_to_go_home():
                self.log(f'пора домой: путь ≈ {self.home_eta:.0f} с')
                self.set_mode('HOME')
                return
            sid = self.next_station()
            if sid:
                self.target_station = sid
                self.dwell_tries = 0
                self.set_mode('TO_STATION')
                return
            if len(self.stations) < 3:
                target = self.explore_target()
                if target is not None:
                    self.explore_cell = target
                    self.set_mode('EXPLORE')
                    return
            self.set_mode('HOME' if self.cargo else 'DONE')
            return

        if self.mode == 'EXPLORE':
            if self.explore_cell in self.seen or (self.next_station() and self.now() - self.mode_t > 0.5) \
                    or self.time_to_go_home():
                self.set_mode('DECIDE')
                return
            r = self.go_to(*cell_center(self.explore_cell), tol=0.6)
            if r is None or self.now() - self.mode_t > 40:
                self.unreachable[self.explore_cell] = self.now() + 30
                self.set_mode('DECIDE')
            elif r:
                self.seen.add(self.explore_cell)
                self.set_mode('DECIDE')
            return

        if self.mode == 'TO_STATION':
            sid = self.target_station
            r = self.go_to(*cell_center(self.stations[sid]), tol=0.25)
            if r is None:
                if self.now() - self.mode_t > 20:
                    self.log(f'станция {sid} недостижима')
                    self.handled[sid] = 'empty'
                    self.set_mode('DECIDE')
            elif r:
                self.cargo_before = self.cargo
                self.dwell_cell = self.stations[sid]
                self.set_mode('DWELL')
            return

        if self.mode == 'DWELL':
            self.drive(0, 0)
            if self.cargo > self.cargo_before:
                self.handled[self.target_station] = 'loaded'
                self.set_mode('DECIDE')
            elif self.now() - self.mode_t > DWELL_TIMEOUT:
                sid = self.target_station
                if self.stations[sid] != self.dwell_cell and self.dwell_tries < 3:
                    self.log(f'станция {sid}: оценка сместилась → {self.stations[sid]}, пробую ещё раз')
                    self.dwell_tries += 1
                    self.set_mode('TO_STATION')
                    return
                self.log(f'станция {sid}: груза нет (неактивна)')
                self.handled[sid] = 'empty'
                self.set_mode('DECIDE')
            return

        if self.mode == 'HOME':
            if self.next_station() and not self.time_to_go_home():
                self.set_mode('DECIDE')       # по дороге нашлась необработанная станция
                return
            r = self.go_to(0.0, 0.0, tol=0.15)
            if r:
                self.set_mode('FACE')
            elif r is None:
                self.drive(0.0, 0.4)      # путь не строится — медленно осматриваемся, карта обновится
            return

        if self.mode == 'FACE':
            if self.unload_dir is None:
                # Запасной вариант: ближайшая стена по лидару.
                best = min(((self.sector_min(a - 5, a + 5), a) for a in (0, 90, 180, -90)))[1]
                a = wrap(self.pose[2] + math.radians(best))
                a = round(a / (math.pi / 2)) * (math.pi / 2)
                self.unload_dir = (round(math.cos(a)), round(math.sin(a)))
            ux, uy = self.unload_dir
            if self.turn_to(math.atan2(-uy, -ux)):     # кормой к стене
                self.set_mode('BACKUP')
            return

        if self.mode == 'BACKUP':
            ux, uy = self.unload_dir
            heading = math.atan2(-uy, -ux)
            # Стена выгрузки — в CELL/2 от центра старта; пятимся, пока до неё не останется UNLOAD_DIST.
            to_wall = CELL / 2 - (self.pose[0] * ux + self.pose[1] * uy)
            if to_wall > UNLOAD_DIST and self.now() - self.mode_t < 15:
                self.drive(-0.1, 2.0 * wrap(heading - self.pose[2]))
            else:
                self.drive(0, 0)
                self.tray_pub.publish(Float64(data=TRAY_UNLOAD))
                self.set_mode('UNLOAD')
            return

        if self.mode == 'UNLOAD':
            self.drive(0, 0)
            if self.now() - self.mode_t > 3.5:
                self.tray_pub.publish(Float64(data=0.0))
                if self.cargo and self.now() - self.mode_t < 8:
                    self.tray_pub.publish(Float64(data=TRAY_UNLOAD))   # ещё раз
                    return
                self.set_mode('AFTER_UNLOAD')
            return

        if self.mode == 'AFTER_UNLOAD':
            self.tray_pub.publish(Float64(data=0.0))
            unhandled = [s for s in ('A', 'B', 'C') if s not in self.handled]
            if unhandled and self.elapsed() < TIME_LIMIT - 60:
                # отъезжаем от зоны выгрузки и продолжаем поиск
                if self.go_to(0.0, 0.0, tol=0.2):
                    self.set_mode('DECIDE')
            else:
                self.set_mode('DONE')
            return

        if self.mode == 'DONE':
            self.drive(0, 0)


def main():
    rclpy.init()
    node = Reference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.drive(0, 0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
