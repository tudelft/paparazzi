#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2026 The Paparazzi Team
#
# This file is part of paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
"""Build the two mesh research papers as styled, optimized PDFs."""

from __future__ import annotations

import argparse
import html
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
from typing import Final

import markdown
from markdown.extensions.toc import slugify_unicode


ROOT: Final = Path(__file__).absolute().parents[3]
DOC_DIR: Final = ROOT / "doc" / "mesh"
CSS_PATH: Final = DOC_DIR / "mesh_publication.css"
PAPERS: Final = (
    ("mesh_network_design.md", "mesh_network_design.pdf", "Flighted reference design"),
    ("asynchronous_coded_mesh.md", "asynchronous_coded_mesh.pdf", "Phase 1 feasibility study"),
)
MERMAID_MODULE: Final = Path(
    "/usr/share/code/resources/app/extensions/markdown-language-features/"
    "markdown-editor-out/mermaid.core-VZVANW2P.js"
)
KATEX_DIR: Final = Path(
    "/usr/share/code/resources/app/extensions/markdown-math/notebook-out"
)


def require_tool(name: str) -> str:
    executable = shutil.which(name)
    if executable is None:
        raise RuntimeError(f"required executable not found: {name}")
    return executable


def preprocess(source: str) -> str:
    """Protect Mermaid and TeX blocks before Markdown conversion."""
    source = re.sub(
        r"```mermaid\n(.*?)\n```",
        lambda match: (
            '\n<div class="mermaid" data-source="'
            + html.escape(match.group(1), quote=True)
            + '"></div>\n'
        ),
        source,
        flags=re.DOTALL,
    )
    source = re.sub(
        r"\$\$(.*?)\$\$",
        lambda match: (
            '\n<div class="math-display" data-math="'
            + html.escape(match.group(1).strip(), quote=True)
            + '"></div>\n'
        ),
        source,
        flags=re.DOTALL,
    )
    source = re.sub(
        r"(?<!\$)\$([^$\n]+?)\$(?!\$)",
        lambda match: (
            '<span class="math-inline" data-math="'
            + html.escape(match.group(1), quote=True)
            + '"></span>'
        ),
        source,
    )
    return source


def extract_metadata(source: str) -> tuple[str, str, str, str]:
    title_match = re.search(r"^# (.+)$", source, re.MULTILINE)
    author_match = re.search(r"^\*\*Author:\*\* (.+)$", source, re.MULTILINE)
    project_match = re.search(r"^\*\*Project:\*\* (.+)$", source, re.MULTILINE)
    status_match = re.search(r"^\*\*Document status:\*\* (.+)$", source, re.MULTILINE)
    if not all((title_match, author_match, project_match, status_match)):
        raise ValueError("paper title, author, project, or status metadata is missing")
    return tuple(match.group(1) for match in
                 (title_match, author_match, project_match, status_match))


def cover(title: str, author: str, project: str, status: str) -> str:
    return f"""
<section class="cover">
  <div class="cover-inner">
    <p class="cover-kicker">Paparazzi UAV / Mesh Research</p>
    <h1>{html.escape(title)}</h1>
    <div class="cover-rule"></div>
    <p class="cover-subtitle">{html.escape(project)}</p>
    <dl class="cover-meta">
      <dt>Author</dt><dd>{html.escape(author)}</dd>
      <dt>Status</dt><dd>{html.escape(status)}</dd>
      <dt>Edition</dt><dd>2026 research edition</dd>
    </dl>
  </div>
</section>
"""


def render_markdown(source: str) -> tuple[str, str]:
    converter = markdown.Markdown(
        extensions=["extra", "fenced_code", "tables", "toc", "sane_lists"],
        extension_configs={
            "toc": {"permalink": False, "slugify": slugify_unicode},
        },
        output_format="html5",
    )
    body = converter.convert(preprocess(source))
    toc = converter.toc
    body = re.sub(r"<table>", '<div class="table-wrap"><table>', body)
    body = re.sub(r"</table>", "</table></div>", body)
    body = body.replace(".md\"", ".pdf\"")
    toc = toc.replace('<div class="toc">', '<nav class="toc"><h2>Contents</h2>')
    toc = toc.replace("</div>", "</nav>")
    toc = re.sub(r'<li><a href="([^"]+)">', r'<li class="toc-level-2"><a href="\1">', toc)
    return body, toc


def html_document(source_path: Path, css_path: Path, work_dir: Path) -> tuple[str, str, str]:
    source = source_path.read_text(encoding="utf-8")
    title, author, project, status = extract_metadata(source)
    body, toc = render_markdown(source)
    css_uri = css_path.as_uri()
    katex_css = (KATEX_DIR / "katex.min.css").as_uri()
    katex_js = (KATEX_DIR / "katex.js").as_uri()
    mermaid_uri = MERMAID_MODULE.as_uri()
    html_text = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="author" content="{html.escape(author, quote=True)}">
  <meta name="document-status" content="{html.escape(status, quote=True)}">
  <title>{html.escape(title)}</title>
  <base href="{source_path.parent.as_uri()}/">
  <link rel="stylesheet" href="{katex_css}">
  <link rel="stylesheet" href="{css_uri}">
</head>
<body>
{cover(title, author, project, status)}
<main>
    <section class="publication-front">
        {toc}
    </section>
  {body}
</main>
<script src="{katex_js}"></script>
<script type="module">
  import mermaid from "{mermaid_uri}";
  const theme = {{
    theme: "base",
    startOnLoad: false,
    securityLevel: "strict",
    fontFamily: "Lato, Noto Sans, sans-serif",
    themeVariables: {{
      primaryColor: "#fff4ad",
      primaryTextColor: "#111111",
      primaryBorderColor: "#111111",
      secondaryColor: "#ffd400",
      secondaryTextColor: "#111111",
      secondaryBorderColor: "#111111",
      tertiaryColor: "#ffffff",
      tertiaryTextColor: "#111111",
      tertiaryBorderColor: "#b8b8ae",
      lineColor: "#111111",
      textColor: "#111111",
      mainBkg: "#fff4ad",
      nodeBorder: "#111111",
      clusterBkg: "#f5f5f1",
      clusterBorder: "#b8b8ae",
      edgeLabelBackground: "#ffffff"
    }},
    flowchart: {{ curve: "basis", htmlLabels: true, useMaxWidth: true }}
  }};
  mermaid.initialize(theme);
  let diagramIndex = 0;
  for (const element of document.querySelectorAll(".mermaid")) {{
    const source = element.dataset.source;
    const result = await mermaid.render(`mesh-diagram-${{diagramIndex++}}`, source);
    element.innerHTML = result.svg;
  }}
  for (const element of document.querySelectorAll("[data-math]")) {{
    katex.render(element.dataset.math, element, {{
      displayMode: element.classList.contains("math-display"),
      throwOnError: true,
      strict: "error"
    }});
  }}
  document.documentElement.dataset.rendered = "true";
</script>
</body>
</html>
"""
    output = work_dir / f"{source_path.stem}.html"
    output.write_text(html_text, encoding="utf-8")
    return title, author, output.as_uri()


def print_pdf(chrome: str, html_uri: str, output: Path, profile: Path) -> None:
    command = [
        chrome,
        "--headless=new",
        "--disable-gpu",
        "--no-sandbox",
        "--allow-file-access-from-files",
        "--disable-extensions",
        "--disable-background-networking",
        "--disable-default-apps",
        "--disable-sync",
        "--hide-scrollbars",
        "--no-pdf-header-footer",
        "--run-all-compositor-stages-before-draw",
        "--virtual-time-budget=15000",
        f"--user-data-dir={profile}",
        f"--print-to-pdf={output}",
        html_uri,
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode != 0 or not output.exists():
        raise RuntimeError(
            f"Chrome PDF generation failed ({completed.returncode}):\n"
            f"{completed.stdout}\n{completed.stderr}"
        )


def optimize_pdf(gs: str, source: Path, destination: Path,
                 title: str, author: str) -> None:
    pdfmark = source.parent / f"{source.stem}-metadata.pdfmark"
    escaped_title = title.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
    escaped_author = author.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
    pdfmark.write_text(
        f"[ /Title ({escaped_title}) /Author ({escaped_author}) "
        "/Subject () /Keywords () /Creator () /Producer () "
        "/CreationDate () /ModDate () /DOCINFO pdfmark\n",
        encoding="ascii",
    )
    command = [
        gs, "-q", "-dBATCH", "-dNOPAUSE", "-dSAFER",
        "-sDEVICE=pdfwrite", "-dCompatibilityLevel=1.7",
        "-dDetectDuplicateImages=true", "-dCompressFonts=true",
        "-dSubsetFonts=true", "-dEmbedAllFonts=true",
        "-dAutoRotatePages=/None",
        f"-sOutputFile={destination}", str(source), str(pdfmark),
    ]
    subprocess.run(command, check=True)


def build(output_dir: Path, keep_html: bool) -> None:
    chrome = require_tool("google-chrome")
    gs = require_tool("ghostscript")
    if not CSS_PATH.exists() or not MERMAID_MODULE.exists() or not KATEX_DIR.exists():
        raise RuntimeError("publication CSS or bundled VS Code render assets are missing")
    output_dir.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="mesh-papers-") as temporary:
        work_dir = Path(temporary)
        for source_name, pdf_name, _label in PAPERS:
            source_path = DOC_DIR / source_name
            title, author, html_uri = html_document(source_path, CSS_PATH, work_dir)
            raw_pdf = work_dir / f"{source_path.stem}-raw.pdf"
            print_pdf(chrome, html_uri, raw_pdf, work_dir / "chrome-profile")
            destination = output_dir / pdf_name
            optimize_pdf(gs, raw_pdf, destination, title, author)
            if keep_html:
                shutil.copy2(Path(html_uri.removeprefix("file://")),
                             output_dir / f"{source_path.stem}.html")
            print(f"wrote {destination} ({destination.stat().st_size / 1024:.0f} KiB)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DOC_DIR,
                        help="PDF destination (default: doc/mesh)")
    parser.add_argument("--keep-html", action="store_true",
                        help="retain intermediate styled HTML beside the PDFs")
    args = parser.parse_args()
    try:
        build(args.output_dir.absolute(), args.keep_html)
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"error: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())