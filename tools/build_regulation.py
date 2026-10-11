#!/usr/bin/env python3
"""Оформление регламента как у РобоКросса: титул, оглавление, колонтитул, A4.

build_reg.py <src.docx> <out.docx> [pages.json]
pages.json — {"заголовок": страница} для оглавления (второй проход).
"""
import json
import os
import re
import shutil
import sys
import tempfile
import zipfile
from xml.sax.saxutils import escape

src, out = sys.argv[1], sys.argv[2]
pages = json.load(open(sys.argv[3])) if len(sys.argv) > 3 else {}

BLUE = '2F5496'
RED = 'C00000'
TITLE = 'ВИРТУАЛЬНЫЙ КУРЬЕР-2026'

tmp = tempfile.mkdtemp()
with zipfile.ZipFile(src) as z:
    z.extractall(tmp)
W = lambda p: os.path.join(tmp, p)

doc = open(W('word/document.xml'), encoding='utf-8').read()

# ---------- убрать заголовок, строку автора, вводный абзац и абзац версии
paras = re.findall(r'<w:p>.*?</w:p>', doc[:6000], re.S)
for p in paras[:4]:
    doc = doc.replace(p, '', 1)


def run(text, size=None, color=None, bold=False, font=None):
    rpr = ''
    if font:
        rpr += f'<w:rFonts w:ascii="{font}" w:hAnsi="{font}" w:cs="{font}"/>'
    if bold:
        rpr += '<w:b/>'
    if color:
        rpr += f'<w:color w:val="{color}"/>'
    if size:
        rpr += f'<w:sz w:val="{size}"/><w:szCs w:val="{size}"/>'
    return f'<w:r><w:rPr>{rpr}</w:rPr><w:t xml:space="preserve">{escape(text)}</w:t></w:r>'


def para(runs, jc='center', before=0, after=0, extra=''):
    return (f'<w:p><w:pPr>{extra}<w:spacing w:before="{before}" w:after="{after}"/>'
            f'<w:jc w:val="{jc}"/></w:pPr>{runs}</w:p>')


# ---------- титульный лист
title = ''.join([
    para(run('РЕГЛАМЕНТ', 36, BLUE, True), before=2600),
    para(run('СОРЕВНОВАНИЙ В СРЕДЕ СИМУЛЯЦИИ', 30, BLUE, True)),
    para(run(f'«{TITLE}»', 40, BLUE, True), after=60),
    para(run('(вер. 1.0 от 11.10.2026 г.)', 26, RED, True)),
    para(run('Черновик: пункты с пометкой «[уточнить]» заполняются Оргкомитетом до публикации.',
             20, '7F7F7F'), before=600),
    '<w:p><w:r><w:br w:type="page"/></w:r></w:p>',
])

# ---------- оглавление (поле TOC с готовым результатом)
heads = re.findall(r'<w:p><w:pPr><w:pStyle w:val="Heading2"/></w:pPr><w:r><w:t xml:space="preserve">(.*?)</w:t></w:r></w:p>', doc)
toc = [para(run('Оглавление', 32, None, False), jc='left', after=120)]
tab = '<w:tabs><w:tab w:val="right" w:leader="dot" w:pos="9638"/></w:tabs>'
for i, h in enumerate(heads):
    plain = re.sub(r'&[a-z]+;', lambda m: {'&quot;': '"', '&amp;': '&', '&lt;': '<', '&gt;': '>'}.get(m.group(0), ''), h)
    num = str(pages.get(plain, ''))
    fld_begin = ''
    if i == 0:
        fld_begin = ('<w:r><w:fldChar w:fldCharType="begin"/></w:r>'
                     '<w:r><w:instrText xml:space="preserve"> TOC \\o "2-2" \\h \\z \\u </w:instrText></w:r>'
                     '<w:r><w:fldChar w:fldCharType="separate"/></w:r>')
    fld_end = '<w:r><w:fldChar w:fldCharType="end"/></w:r>' if i == len(heads) - 1 else ''
    m = re.match(r'(\d+)\.\s+(.*)', plain)
    if m:
        text_runs = run(f'{m.group(1)}.') + '<w:r><w:tab/></w:r>' + run(m.group(2))
        ind = '<w:ind w:left="567" w:hanging="567"/>'
    else:
        text_runs = run(plain)
        ind = ''
    toc.append(f'<w:p><w:pPr>{tab}{ind}<w:spacing w:before="0" w:after="60"/><w:jc w:val="left"/></w:pPr>'
               f'{fld_begin}{text_runs}<w:r><w:tab/></w:r>{run(num)}{fld_end}</w:p>')
toc.append('<w:p><w:r><w:br w:type="page"/></w:r></w:p>')

doc = doc.replace('<w:body>', '<w:body>' + title + ''.join(toc), 1)

# ---------- приложения: с новой страницы, заголовок справа
def appendix(m):
    num, name = m.group(1), m.group(2)
    return (f'<w:p><w:pPr><w:pageBreakBefore/><w:spacing w:before="0" w:after="0"/><w:jc w:val="right"/></w:pPr>'
            f'{run(f"Приложение №{num}", 24, None, True)}</w:p>'
            f'<w:p><w:pPr><w:pStyle w:val="Heading2"/><w:spacing w:before="0" w:after="240"/><w:jc w:val="right"/></w:pPr>'
            f'{run(name, 24, None, True)}</w:p>')

doc = re.sub(r'<w:p><w:pPr><w:pStyle w:val="Heading2"/></w:pPr><w:r><w:t xml:space="preserve">'
             r'Приложение №(\d)\. (.*?)</w:t></w:r></w:p>', appendix, doc)

# картинки — по центру
doc = doc.replace('<w:p><w:pPr></w:pPr><w:r><w:drawing>', '<w:p><w:pPr><w:jc w:val="center"/></w:pPr><w:r><w:drawing>')

# текст в таблицах — по левому краю (иначе выравнивание по ширине растягивает пробелы)
doc = re.sub(r'<w:tbl>.*?</w:tbl>', lambda m: m.group(0).replace('<w:pPr></w:pPr>', '<w:pPr><w:jc w:val="left"/></w:pPr>'), doc, flags=re.S)

# ---------- A4, поля 2 см, колонтитул, без колонтитула на титуле
sect = ('<w:sectPr><w:footerReference w:type="default" r:id="rIdFooter1"/>'
        '<w:pgSz w:w="11906" w:h="16838"/>'
        '<w:pgMar w:top="1134" w:right="851" w:bottom="1134" w:left="1418" w:header="567" w:footer="567" w:gutter="0"/>'
        '<w:titlePg/></w:sectPr>')
doc = re.sub(r'<w:sectPr>.*?</w:sectPr>', sect, doc, flags=re.S)
open(W('word/document.xml'), 'w', encoding='utf-8').write(doc)

# ---------- footer
TNR = 'Times New Roman'
def fld(code):
    return (f'<w:r><w:rPr><w:rFonts w:ascii="{TNR}" w:hAnsi="{TNR}"/><w:b/><w:sz w:val="24"/></w:rPr><w:fldChar w:fldCharType="begin"/></w:r>'
            f'<w:r><w:rPr><w:rFonts w:ascii="{TNR}" w:hAnsi="{TNR}"/><w:b/><w:sz w:val="24"/></w:rPr><w:instrText xml:space="preserve"> {code} </w:instrText></w:r>'
            f'<w:r><w:rPr><w:rFonts w:ascii="{TNR}" w:hAnsi="{TNR}"/><w:b/><w:sz w:val="24"/></w:rPr><w:fldChar w:fldCharType="separate"/></w:r>'
            f'<w:r><w:rPr><w:rFonts w:ascii="{TNR}" w:hAnsi="{TNR}"/><w:b/><w:sz w:val="24"/></w:rPr><w:t>1</w:t></w:r>'
            f'<w:r><w:rPr><w:rFonts w:ascii="{TNR}" w:hAnsi="{TNR}"/><w:b/><w:sz w:val="24"/></w:rPr><w:fldChar w:fldCharType="end"/></w:r>')
footer = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
          '<w:ftr xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main" '
          'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
          '<w:p><w:pPr><w:tabs><w:tab w:val="right" w:pos="9637"/></w:tabs><w:spacing w:after="0"/></w:pPr>'
          + run(TITLE, 24, None, False, TNR) + '<w:r><w:tab/></w:r>' + run('стр. ', 24, None, False, TNR)
          + fld('PAGE') + run(' из ', 24, None, False, TNR) + fld('NUMPAGES')
          + '</w:p></w:ftr>')
open(W('word/footer1.xml'), 'w', encoding='utf-8').write(footer)

rels = open(W('word/_rels/document.xml.rels'), encoding='utf-8').read()
if 'footer1.xml' not in rels:
    rels = rels.replace('</Relationships>',
                        '<Relationship Id="rIdFooter1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/footer" Target="footer1.xml"/></Relationships>')
    open(W('word/_rels/document.xml.rels'), 'w', encoding='utf-8').write(rels)
ct = open(W('[Content_Types].xml'), encoding='utf-8').read()
if 'footer1.xml' not in ct:
    ct = ct.replace('</Types>', '<Override PartName="/word/footer1.xml" '
                    'ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.footer+xml"/></Types>')
    open(W('[Content_Types].xml'), 'w', encoding='utf-8').write(ct)

# ---------- стили: синие заголовки разделов, русский язык, текст по ширине
st = open(W('word/styles.xml'), encoding='utf-8').read()
st = st.replace('<w:lang w:val="en-US"/>', '<w:lang w:val="ru-RU" w:eastAsia="ru-RU"/>')
st = st.replace('<w:style w:type="paragraph" w:styleId="Heading2"><w:name w:val="heading 2"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:qFormat/><w:pPr><w:keepNext/><w:spacing w:before="280" w:after="100"/><w:outlineLvl w:val="1"/></w:pPr><w:rPr><w:b/><w:sz w:val="26"/></w:rPr></w:style>',
                f'<w:style w:type="paragraph" w:styleId="Heading2"><w:name w:val="heading 2"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:qFormat/><w:pPr><w:keepNext/><w:spacing w:before="360" w:after="120"/><w:jc w:val="left"/><w:outlineLvl w:val="1"/></w:pPr><w:rPr><w:b/><w:color w:val="{BLUE}"/><w:sz w:val="28"/></w:rPr></w:style>')
st = st.replace('<w:style w:type="paragraph" w:default="1" w:styleId="Normal"><w:name w:val="Normal"/><w:qFormat/></w:style>',
                '<w:style w:type="paragraph" w:default="1" w:styleId="Normal"><w:name w:val="Normal"/><w:qFormat/><w:pPr><w:spacing w:after="100"/><w:jc w:val="both"/></w:pPr></w:style>')
open(W('word/styles.xml'), 'w', encoding='utf-8').write(st)

# ---------- упаковка
if os.path.exists(out):
    os.remove(out)
with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED) as z:
    for root, _, files in os.walk(tmp):
        for f in files:
            full = os.path.join(root, f)
            z.write(full, os.path.relpath(full, tmp))
shutil.rmtree(tmp)
print('ok', out, len(heads), 'заголовков')
