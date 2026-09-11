#!/usr/bin/env python3
"""Export the generated guide: pip install playwright pypdf; requires Chrome."""

from pathlib import Path
import shutil
from tempfile import TemporaryDirectory
from urllib.parse import unquote, urlparse

from playwright.sync_api import sync_playwright
from pypdf import PdfReader, PdfWriter
from pypdf.generic import TextStringObject, NameObject


ROOT = Path(__file__).resolve().parent
PRINT_STYLE = """
@page { size: A4; margin: 16mm 16mm 20mm; }
* { print-color-adjust: exact; -webkit-print-color-adjust: exact; }
html { scroll-behavior: auto; }
body { background: white; font-size: 10pt; line-height: 1.45; }
.masthead { break-inside: avoid; }
.masthead-inner { width: auto; padding: 10mm 8mm; }
.masthead h1 { font-size: 29pt; }
.lede { font-size: 11pt; }
.layout { display: block; width: auto; margin: 0; padding: 0; }
nav { position: static; margin: 8mm 0; break-inside: avoid; }
nav strong { font-size: 9pt; }
.toc > ul > li > ul { display: block; columns: 2; column-gap: 8mm; }
nav li { break-inside: avoid; }
nav a { font-size: 9pt; padding: 2mm 3mm; }
main > h2 { break-before: page; margin-top: 0; font-size: 21pt; }
h2, h3 { break-after: avoid; }
h3 { font-size: 13pt; margin-top: 6mm; }
p, li { orphans: 3; widows: 3; }
pre { white-space: pre-wrap; overflow-wrap: anywhere; overflow: visible;
      font-size: 8pt; padding: 4mm; break-inside: avoid; }
blockquote, tr { break-inside: avoid; }
thead { display: table-header-group; }
th, td { padding: 2.5mm; font-size: 9pt; }
img { max-width: 100%; max-height: 125mm; width: auto; object-fit: contain;
      margin: 5mm auto; break-inside: avoid; background: white; border: 0; }
a { text-decoration: underline; }
footer { display: none; }
"""


def main():
    chrome = shutil.which("google-chrome") or shutil.which("chromium")
    if chrome is None:
        raise SystemExit("Install Chrome or Chromium before exporting the PDF")
    output = ROOT / "lwir-calibration.pdf"
    with TemporaryDirectory(prefix="lwir-pdf-") as temporary:
        rendered = Path(temporary) / "rendered.pdf"
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(executable_path=chrome, headless=True)
            page = browser.new_page()
            page.goto((ROOT / "lwir-calibration.html").as_uri(), wait_until="load")
            page.evaluate("""() => {
                for (const image of document.images) {
                    image.src = image.src.replace(/\\.png$/, '.svg');
                }
            }""")
            page.wait_for_function("[...document.images].every(image => image.complete && image.naturalWidth > 0)")
            page.evaluate("document.fonts.ready")
            page.add_style_tag(content=PRINT_STYLE)
            page.pdf(path=str(rendered), prefer_css_page_size=True, print_background=True,
                     tagged=True, outline=True, display_header_footer=True,
                     header_template="<span></span>",
                     footer_template='''<div style="width:100%;margin:0 16mm;font-family:sans-serif;
                         font-size:8px;color:#53627a;display:flex;justify-content:space-between">
                         <span>CATIA / Tiny1-C / Workshop Calibration</span>
                         <span><span class="pageNumber"></span> / <span class="totalPages"></span></span></div>''')
            browser.close()
        reader = PdfReader(rendered)
        writer = PdfWriter(clone_from=reader)
        for page in writer.pages:
            for reference in page.get("/Annots", []):
                annotation = reference.get_object()
                action = annotation.get("/A")
                if action is None or action.get("/S") != "/URI":
                    continue
                uri = urlparse(str(action.get("/URI", "")))
                if uri.scheme == "file":
                    target = Path(unquote(uri.path))
                    if target.parent.resolve() == ROOT:
                        relative = target.name + ("#" + uri.fragment if uri.fragment else "")
                        action[NameObject("/URI")] = TextStringObject(relative)
        writer.add_metadata({"/Title": "LWIR Camera Calibration", "/Author": "CATIA",
                             "/Subject": "Tiny1-C lens, mounting and temperature workshop guide"})
        temporary_output = ROOT / "lwir-calibration.pdf.tmp"
        try:
            with temporary_output.open("wb") as file:
                writer.write(file)
            temporary_output.replace(output)
        finally:
            temporary_output.unlink(missing_ok=True)
        print(f"Generated {output.name}: {len(reader.pages)} pages")


if __name__ == "__main__":
    main()