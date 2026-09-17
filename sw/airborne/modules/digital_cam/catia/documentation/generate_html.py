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
HTML_DIR = DOCUMENTATION_DIR / "html"
HTML_DIR.mkdir(parents=True, exist_ok=True)

# (source markdown, output html, title, nav_label, description, lede)
DOCUMENTS = (
    (
        "index.md",
        "index.html",
        "CATIA Documentation Hub",
        "Documentation Hub",
        "Overview and portal to the CATIA camera pipeline, MORA companion computer deployment, AI & thermal camera drivers, acoustic localization, and mission operations.",
        "Documentation portal for CATIA, MORA companion systems, and onboard sensor payloads.",
    ),
    (
        "catia_camera_pipeline.md",
        "catia_camera_pipeline.html",
        "CATIA Camera Pipeline",
        "Camera Pipeline",
        "CATIA camera pipeline setup, testing, capture, EXIF, and SODA guide",
        "From a flight-triggered shot to a georeferenced JPEG and image-analysis "
        "result. Complete technical guide to MORA deployment, camera backends, EXIF geotagging, and SODA.",
    ),
    (
        "raspberry_pi_ai_camera.md",
        "raspberry_pi_ai_camera.html",
        "Raspberry Pi AI Camera",
        "AI Camera (IMX500)",
        "IMX500 camera setup, rpicam commands, inference demos and CATIA integration",
        "Connect and test the Raspberry Pi AI Camera, then integrate optical "
        "capture with CATIA on MORA.",
    ),
      (
        "setup_os_rpi_zero_2w.md",
        "setup_os_rpi_zero_2w.html",
        "Raspberry Pi Zero 2 W OS Setup",
        "Pi Zero 2 W Setup",
        "Headless Raspberry Pi Zero 2 W imaging, networking, UART, cloning, and power-management setup",
        "Prepare a Raspberry Pi Zero 2 W for CATIA and camera work before deploying the MORA software stack.",
      ),
    (
        "lwir-calibration.md",
        "lwir-calibration.html",
        "LWIR Camera Calibration",
        "LWIR Calibration",
        "Practical Tiny1-C lens and mounting calibration using an ordinary workshop target",
        "A measured lens, a known mounting direction, and evidence you can keep. "
        "Work through the Tiny1-C calibration one clear checkpoint at a time.",
    ),
    (
        "earcam-loudest-spot-explained.md",
        "earcam-loudest-spot-explained.html",
        "EARcam Explained",
        "EARcam Guide",
        "Why one USB microphone, how far it hears a motionSCOUT alarm, and how "
        "the loudest spot is computed",
        "A plain-language guide to everything involved in finding the loudest "
        "spot on the ground from a small aircraft with a single microphone.",
    ),
    (
        "earcam-dataflow.md",
        "earcam-dataflow.html",
        "EARcam Position and Sound Data Flow",
        "EARcam Data Flow",
        "How acoustic measurements and flight-controller coordinates are synchronized, "
        "buffered, and solved on the MORA companion computer",
        "Trace the synchronization from flight-controller trigger to "
        "microphone loudness window, geotagged ring buffer, and loudest-spot solve.",
    ),
    (
        "mission2-score-first.md",
        "mission2-score-first.html",
        "Mission 2: Maximize Points, Keep The Airframe",
        "Mission 2 Plan",
        "Fixed-hardware IMAV 2026 Mission 2 scoring, airframe preservation and validation plan",
        "Maximize our chance of winning with the existing aircraft. Improve "
        "the onboard software and bring the Talon back with its Mission 2 results.",
    ),
      (
        "precision-landing-flight-test.md",
        "precision-landing-flight-test.html",
        "Adam and EasyStar 3 Precision Landing Flight Test",
        "Precision Landing",
        "Staged validation and tuning plan for Adam and EasyStar 3 autonomous IMAV2026 fixed-wing precision landing",
        "Tune the final standstill spot near TD, preferably with first touch also inside the 20 by 3 metre box; review contact quality separately.",
      ),
)

STYLE = """
:root {
  color-scheme: light;
  --ink: #111111;
  --muted: #595959;
  --paper: #ffffff;
  --surface: #ffffff;
  --line: #dedede;
  --yellow: #ffdd00;
  --link: #806600;
  --soft: #f5f5f5;
  --black: #000000;
  --code: #111111;
  --code-ink: #ffffff;
}
* { box-sizing: border-box; }
html { scroll-behavior: smooth; }
body {
  margin: 0;
  color: var(--ink);
  background:
    linear-gradient(90deg, rgba(0, 0, 0, 0.018) 1px, transparent 1px),
    linear-gradient(rgba(0, 0, 0, 0.018) 1px, transparent 1px),
    var(--paper);
  background-size: 28px 28px;
  font-family: "IBM Plex Sans", "Segoe UI", sans-serif;
  line-height: 1.65;
}
a { color: var(--link); text-underline-offset: 0.2em; }
a:hover { color: var(--ink); text-decoration-thickness: 2px; }
a:focus-visible { outline: 2px solid var(--link); outline-offset: 3px; }
::selection { color: var(--black); background: var(--yellow); }
code, pre { font-family: "IBM Plex Mono", "Liberation Mono", monospace; }
.masthead { color: var(--surface); background: var(--black); border-bottom: 5px solid var(--yellow); }
.masthead-inner {
  width: min(1180px, calc(100% - 40px));
  margin: 0 auto;
  padding: 32px 0 34px;
}
.site-nav {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
  margin-bottom: 24px;
  padding-bottom: 18px;
  border-bottom: 1px solid rgba(255, 255, 255, 0.15);
}
.site-nav-link {
  display: inline-block;
  padding: 6px 12px;
  border-radius: 4px;
  background: rgba(255, 255, 255, 0.08);
  color: #ffffff;
  font-size: 0.85rem;
  font-weight: 600;
  text-decoration: none;
  transition: background 0.15s, color 0.15s;
}
.site-nav-link:hover, .site-nav-link:focus-visible {
  background: var(--yellow);
  color: var(--black);
}
.site-nav-link.active {
  background: var(--yellow);
  color: var(--black);
}
.eyebrow {
  margin: 0 0 8px;
  color: var(--yellow);
  font-size: 0.8rem;
  font-weight: 800;
  letter-spacing: 0;
  text-transform: uppercase;
}
.masthead h1 { margin: 0; color: var(--surface); font-size: 2.75rem; overflow-wrap: anywhere; }
.lede { max-width: 760px; margin: 12px 0 0; color: #dedede; font-size: 1.08rem; }
.layout {
  display: grid;
  grid-template-columns: 260px minmax(0, 850px);
  gap: 48px;
  width: min(1180px, calc(100% - 40px));
  margin: 0 auto;
  padding: 38px 0 80px;
}
nav { position: sticky; top: 24px; align-self: start; max-height: calc(100vh - 48px); overflow-y: auto; }
.nav-section { margin-bottom: 26px; }
.nav-section strong {
  display: block;
  margin-bottom: 10px;
  color: var(--muted);
  font-size: 0.78rem;
  text-transform: uppercase;
  letter-spacing: 0.04em;
}
.guide-nav, .toc ul { margin: 0; padding: 0; list-style: none; }
.guide-nav li, .toc li { margin: 0; padding: 0; }
.guide-nav a, .toc a {
  display: block;
  padding: 6px 12px;
  border-left: 2px solid var(--line);
  color: var(--ink);
  font-size: 0.88rem;
  text-decoration: none;
}
.guide-nav a:hover, .toc a:hover, .guide-nav a:focus-visible, .toc a:focus-visible {
  border-color: var(--yellow);
  color: var(--link);
  background: var(--soft);
}
.guide-nav a.active {
  border-left: 3px solid var(--yellow);
  color: var(--black);
  background: var(--soft);
  font-weight: 700;
}
.toc > ul > li > a { display: none; }
.toc ul ul { display: block; margin: 0; padding: 0; list-style: none; }
main { min-width: 0; overflow-wrap: anywhere; }
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
  border-left: 4px solid var(--yellow);
  background: var(--soft);
}
img {
  display: block;
  max-width: 100%;
  height: auto;
  margin: 28px auto 48px;
  border: 1px solid var(--line);
  background: var(--surface);
}
pre {
  overflow-x: auto;
  margin: 14px 0 20px;
  padding: 16px 18px;
  border-left: 4px solid var(--yellow);
  color: var(--code-ink);
  background: var(--code);
  font-size: 0.9rem;
  line-height: 1.55;
}
code:not(pre code) { padding: 0.12em 0.34em; color: var(--ink); background: var(--soft); }
a code:not(pre code) { color: inherit; }
table { width: 100%; border-collapse: collapse; margin: 18px 0 26px; background: var(--surface); }
th, td { padding: 11px 13px; border: 1px solid var(--line); text-align: left; vertical-align: top; }
th { color: var(--black); background: var(--yellow); }
tbody tr:nth-child(even) { background: var(--soft); }
ul, ol { padding-left: 24px; }
hr { border: 0; border-top: 2px solid var(--line); margin: 32px 0; }
footer { padding: 25px 20px; color: var(--surface); background: var(--black); border-top: 3px solid var(--yellow); text-align: center; }
@media (max-width: 820px) {
  .layout { grid-template-columns: 1fr; gap: 24px; }
  nav { position: static; max-height: none; }
  .toc > ul > li > ul { display: grid; grid-template-columns: repeat(2, 1fr); }
  .masthead-inner { padding-top: 28px; }
}
@media (max-width: 520px) {
  .layout, .masthead-inner { width: min(100% - 24px, 1180px); }
  .toc > ul > li > ul { grid-template-columns: 1fr; }
  .masthead h1 { font-size: 2.25rem; }
  th, td { padding: 8px; }
}
"""


def render(source_name: str, output_name: str, title: str, nav_label: str,
           description: str, lede: str) -> None:
    source_text = (DOCUMENTATION_DIR / source_name).read_text(encoding="utf-8")
    renderer = markdown.Markdown(
        extensions=["fenced_code", "tables", "toc", "sane_lists"],
        extension_configs={"toc": {"permalink": False, "toc_depth": "2"}},
        output_format="html5",
    )
    body = renderer.convert(source_text)

    top_nav_links = []
    sidebar_guide_links = []
    for s_name, out_name, doc_title, doc_nav_label, _, _ in DOCUMENTS:
        is_active = (out_name == output_name)
        active_class = " active" if is_active else ""
        top_nav_links.append(
            f'<a href="{html.escape(out_name)}" class="site-nav-link{active_class}">{html.escape(doc_nav_label)}</a>'
        )
        sidebar_guide_links.append(
            f'<li><a href="{html.escape(out_name)}" class="{active_class.strip()}">{html.escape(doc_nav_label)}</a></li>'
        )

    top_nav_html = "\n      ".join(top_nav_links)
    sidebar_guides_html = "\n        ".join(sidebar_guide_links)

    page = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="description" content="{html.escape(description)}">
  <title>{html.escape(title)}</title>
  <style>{STYLE}</style>
</head>
<body>
  <header class="masthead">
    <div class="masthead-inner">
      <nav class="site-nav" aria-label="Main Navigation">
        {top_nav_html}
      </nav>
      <p class="eyebrow">Paparazzi UAV / MORA / SODA</p>
      <h1>{html.escape(title)}</h1>
      <p class="lede">{html.escape(lede)}</p>
    </div>
  </header>
  <div class="layout">
    <nav aria-label="Guide navigation">
      <div class="nav-section">
        <strong>All Guides</strong>
        <ul class="guide-nav">
          {sidebar_guides_html}
        </ul>
      </div>
      <div class="nav-section">
        <strong>On this page</strong>
        {renderer.toc}
      </div>
    </nav>
    <main>{body}</main>
  </div>
  <footer>CATIA documentation / Paparazzi UAV</footer>
</body>
</html>
"""
    (HTML_DIR / output_name).write_text(page, encoding="utf-8")


def main() -> int:
    for document in DOCUMENTS:
        render(*document)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
