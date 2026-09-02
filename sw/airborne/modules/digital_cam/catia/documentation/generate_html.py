#!/usr/bin/env python3

from __future__ import annotations

import html
from pathlib import Path
import sys

try:
    import markdown
except ImportError as error:
    print(
        "generate_html.py: Python package 'Markdown' is required; "
        "install it with: python3 -m pip install Markdown",
        file=sys.stderr,
    )
    raise SystemExit(1) from error


DOCUMENTATION_DIR = Path(__file__).resolve().parent
SOURCE = DOCUMENTATION_DIR / "README.md"
OUTPUT = DOCUMENTATION_DIR / "index.html"

STYLE = """
:root {
  color-scheme: light;
  --ink: #14213d;
  --muted: #53627a;
  --paper: #f7f9fc;
  --surface: #ffffff;
  --line: #d8e0ea;
  --cyan: #007f8b;
  --green: #237a45;
  --amber: #a85c00;
  --code: #101827;
  --code-ink: #dce8f5;
}
* { box-sizing: border-box; }
html { scroll-behavior: smooth; }
body {
  margin: 0;
  color: var(--ink);
  background:
    linear-gradient(90deg, rgba(0, 127, 139, 0.045) 1px, transparent 1px),
    linear-gradient(rgba(0, 127, 139, 0.045) 1px, transparent 1px),
    var(--paper);
  background-size: 28px 28px;
  font-family: "IBM Plex Sans", "Segoe UI", sans-serif;
  line-height: 1.65;
}
a { color: var(--cyan); }
code, pre { font-family: "IBM Plex Mono", "Liberation Mono", monospace; }
.masthead { color: white; background: #11243c; border-bottom: 5px solid #20b6bb; }
.masthead-inner {
  width: min(1180px, calc(100% - 40px));
  margin: 0 auto;
  padding: 48px 0 38px;
}
.eyebrow {
  margin: 0 0 8px;
  color: #78e3df;
  font-size: 0.8rem;
  font-weight: 800;
  letter-spacing: 0;
  text-transform: uppercase;
}
.masthead h1 { margin: 0; color: white; font-size: clamp(2.25rem, 7vw, 4.7rem); }
.lede { max-width: 760px; margin: 14px 0 0; color: #cfdae8; font-size: 1.12rem; }
.layout {
  display: grid;
  grid-template-columns: 250px minmax(0, 850px);
  gap: 48px;
  width: min(1180px, calc(100% - 40px));
  margin: 0 auto;
  padding: 38px 0 80px;
}
nav { position: sticky; top: 24px; align-self: start; }
nav strong {
  display: block;
  margin-bottom: 12px;
  color: var(--muted);
  font-size: 0.78rem;
  text-transform: uppercase;
}
nav ul { margin: 0; padding: 0; list-style: none; }
.toc > ul > li > a { display: none; }
.toc ul ul { display: block; margin: 0; padding: 0; list-style: none; }
nav a {
  display: block;
  padding: 8px 12px;
  border-left: 2px solid var(--line);
  color: var(--ink);
  text-decoration: none;
}
nav a:hover { border-color: var(--cyan); color: var(--cyan); background: white; }
main { min-width: 0; }
main > h1 { display: none; }
h1, h2, h3 { line-height: 1.18; letter-spacing: 0; }
h2 {
  margin: 54px 0 16px;
  padding-bottom: 9px;
  border-bottom: 2px solid var(--line);
  font-size: 1.75rem;
  scroll-margin-top: 24px;
}
main > h2:first-of-type { margin-top: 0; }
h3 { margin: 30px 0 10px; font-size: 1.18rem; scroll-margin-top: 24px; }
p { margin: 10px 0 16px; }
blockquote {
  margin: 18px 0;
  padding: 14px 18px;
  border-left: 4px solid var(--amber);
  background: #fff7e8;
}
blockquote:first-of-type { border-color: var(--cyan); background: #edf8f8; }
img {
  display: block;
  max-width: 100%;
  height: auto;
  margin: 28px auto 48px;
  border: 1px solid #1a334f;
  background: #07111f;
}
pre {
  overflow-x: auto;
  margin: 14px 0 20px;
  padding: 16px 18px;
  border-left: 4px solid #20b6bb;
  color: var(--code-ink);
  background: var(--code);
  font-size: 0.9rem;
  line-height: 1.55;
}
code:not(pre code) { padding: 0.12em 0.34em; color: #8c3142; background: #f0e9ec; }
table { width: 100%; border-collapse: collapse; margin: 18px 0 26px; background: white; }
th, td { padding: 11px 13px; border: 1px solid var(--line); text-align: left; vertical-align: top; }
th { color: white; background: #233a56; }
tbody tr:nth-child(even) { background: #f4f7fa; }
ul, ol { padding-left: 24px; }
hr { border: 0; border-top: 2px solid var(--line); }
footer { padding: 25px 20px; color: #ccd8e6; background: #11243c; text-align: center; }
@media (max-width: 820px) {
  .layout { grid-template-columns: 1fr; gap: 24px; }
  nav { position: static; }
  .toc > ul > li > ul { display: grid; grid-template-columns: repeat(2, 1fr); }
  .masthead-inner { padding-top: 34px; }
}
@media (max-width: 520px) {
  .layout, .masthead-inner { width: min(100% - 24px, 1180px); }
  .toc > ul > li > ul { grid-template-columns: 1fr; }
  .masthead h1 { font-size: 2.25rem; }
  th, td { padding: 8px; }
}
"""


def main() -> int:
    source_text = SOURCE.read_text(encoding="ascii")
    renderer = markdown.Markdown(
        extensions=["fenced_code", "tables", "toc", "sane_lists"],
        extension_configs={"toc": {"permalink": False, "toc_depth": "2"}},
        output_format="html5",
    )
    body = renderer.convert(source_text)
    title = "CATIA Camera Pipeline"
    page = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="description" content="CATIA camera pipeline setup, testing, capture, EXIF, and SODA guide">
  <title>{html.escape(title)} Guide</title>
  <style>{STYLE}</style>
</head>
<body>
  <header class="masthead">
    <div class="masthead-inner">
      <p class="eyebrow">Paparazzi UAV / MORA / SODA</p>
      <h1>{html.escape(title)}</h1>
      <p class="lede">From a flight-triggered shot to a georeferenced JPEG and image-analysis result. Start locally, then move to CHDK or Raspberry Pi camera hardware.</p>
    </div>
  </header>
  <div class="layout">
    <nav aria-label="Guide sections">
      <strong>On this page</strong>
      {renderer.toc}
    </nav>
    <main>{body}</main>
  </div>
  <footer>CATIA documentation / Paparazzi UAV</footer>
</body>
</html>
"""
    OUTPUT.write_text(page, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
