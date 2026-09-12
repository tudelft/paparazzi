#!/usr/bin/env python3
"""Generate original, explicitly schematic workshop illustrations."""

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parent
HTML_DIR = ROOT / "html"
HTML_DIR.mkdir(parents=True, exist_ok=True)
INK = "#111111"
YELLOW = "#ca8a04"
AMBER = "#d97706"
DARK = "#222222"
LIGHT_BG = "#ffffff"
CARD_BORDER = "#dedede"


def element(parent, tag, **attributes):
    return ET.SubElement(parent, tag, {key.replace("_", "-"): str(value)
                                     for key, value in attributes.items()})


def text(parent, horizontal, vertical, content, size=20, color=INK, weight="normal"):
    element(parent, "text", x=horizontal, y=vertical, fill=color, font_size=size,
            font_family="DejaVu Sans", font_weight=weight).text = content


def rectangle(parent, horizontal, vertical, width, height, fill, stroke="none", stroke_width=2):
    element(parent, "rect", x=horizontal, y=vertical, width=width, height=height,
            fill=fill, stroke=stroke, stroke_width=stroke_width)


def line(parent, start, end, color=DARK, arrow=False, width=3):
    attributes = dict(x1=start[0], y1=start[1], x2=end[0], y2=end[1],
                      stroke=color, stroke_width=width)
    if arrow:
        attributes["marker_end"] = "url(#arrow)"
    element(parent, "line", **attributes)


def canvas(title, subtitle, height=620):
    root = ET.Element("svg", {"xmlns": "http://www.w3.org/2000/svg",
                              "viewBox": f"0 0 880 {height}", "width": "880", "height": str(height)})
    element(root, "title").text = title
    definitions = element(root, "defs")
    marker = element(definitions, "marker", id="arrow", markerWidth=8, markerHeight=8,
                     refX=7, refY=3, orient="auto", markerUnits="strokeWidth")
    element(marker, "path", d="M0,0 L0,6 L7,3 Z", fill=DARK)
    rectangle(root, 0, 0, 880, height, LIGHT_BG, CARD_BORDER, 2)
    rectangle(root, 32, 32, 6, 54, YELLOW)
    text(root, 54, 54, title, 26, weight="bold")
    text(root, 54, 83, subtitle, 16, color="#555555")
    return root


def board(parent, left, top, pitch, corners=False):
    rectangle(parent, left - pitch / 2, top - pitch / 2, pitch * 9, pitch * 8, "#d1d5db", INK, 2)
    for row in range(7):
        for column in range(8):
            rectangle(parent, left + column * pitch, top + row * pitch, pitch, pitch,
                      "#f3f4f6" if (row + column) % 2 else "#1f2937")
    if corners:
        for row in range(1, 7):
            for column in range(1, 8):
                element(parent, "circle", cx=left + column * pitch, cy=top + row * pitch,
                        r=5, fill="#facc15", stroke="#854d0e", stroke_width=1.5)


def save(root, name):
    ET.indent(root)
    ET.ElementTree(root).write(HTML_DIR / f"lwir-calibration-{name}.svg", encoding="utf-8", xml_declaration=True)


def target():
    root = canvas("Build squares. Count inner corners.", "Construction schematic / dimensions come from your ruler")
    board(root, 80, 150, 48, corners=True)
    text(root, 530, 170, "8 x 7 squares", 26, weight="bold")
    text(root, 530, 205, "7 x 6 inner corners", 23, AMBER, "bold")
    rectangle(root, 530, 237, 24, 24, "#1f2937")
    text(root, 569, 256, "Thin matte tape", 19)
    rectangle(root, 530, 277, 24, 24, "#f3f4f6", INK, 1.5)
    text(root, 569, 296, "Smooth foil", 19)
    text(root, 530, 347, "Dots show crossings.", 18)
    text(root, 530, 375, "Do not add dots to the target.", 17, color="#555555")
    text(root, 530, 420, "Flat backing, clean edges,", 18)
    text(root, 530, 447, "uniform border, no overlaps.", 18)
    line(root, (80, 530), (320, 530), color=DARK, width=3)
    line(root, (80, 518), (80, 542), color=DARK, width=3)
    line(root, (320, 518), (320, 542), color=DARK, width=3)
    text(root, 80, 570, "5 pitches = 250 mm gives 50 mm per square", 19, weight="bold")
    text(root, 80, 599, "Example only. Measure both directions on the finished board.", 16, color="#555555")
    save(root, "target")


def coverage():
    root = canvas("Let the corners visit the whole image.", "Keep the complete border visible / keep flight focus unchanged", 740)
    labels = ["01  Center", "02  Upper left", "03  Lower right", "04  Left/right tilt", "05  Forward/back tilt", "06  Change distance"]
    shapes = ["70,25 160,25 160,105 70,105", "15,8 105,8 105,88 15,88",
              "120,50 210,50 210,130 120,130", "60,20 155,35 155,105 60,120",
              "60,35 170,35 150,115 80,115", "95,50 145,50 145,95 95,95"]
    for index, (label, points) in enumerate(zip(labels, shapes)):
        left = 38 + (index % 3) * 279
        top = 130 + (index // 3) * 234
        text(root, left, top, label, 18, weight="bold")
        group = element(root, "g", transform=f"translate({left} {top + 20})")
        rectangle(group, 0, 0, 240, 145, "#f9fafb", "#9ca3af", 2)
        line(group, (120, 0), (120, 145), "#d1d5db", width=1.5)
        line(group, (0, 72), (240, 72), "#d1d5db", width=1.5)
        element(group, "polygon", points=points, fill="#fef08a", stroke=YELLOW, stroke_width=3)
        text(root, left, top + 187, "Target footprint in sensor", 14, color="#555555")
    text(root, 40, 634, "25-35 varied fitting views", 24, YELLOW, "bold")
    text(root, 40, 673, "+ 5-8 new views kept only for checking", 23, AMBER, "bold")
    text(root, 40, 714, "Include all edges and corners; these six poses are examples, not the full set.", 17, color="#555555")
    save(root, "coverage")


def mount():
    root = canvas("Give the board the aircraft's axes.", "Top view / board and aircraft body reference must be level", 680)
    board(root, 110, 220, 42, corners=True)
    line(root, (480, 420), (480, 158), color=DARK, arrow=True, width=3.5)
    text(root, 505, 172, "FORWARD / nose", 22, DARK, "bold")
    line(root, (450, 530), (745, 530), color=DARK, arrow=True, width=3.5)
    text(root, 545, 567, "BODY RIGHT", 22, DARK, "bold")
    for label, horizontal, vertical in (("0", 152, 262), ("1", 194, 262), ("7", 152, 304)):
        element(root, "circle", cx=horizontal, cy=vertical, r=15, fill="#fef08a", stroke=YELLOW, stroke_width=2.5)
        text(root, horizontal - 6, vertical + 6, label, 18, INK, "bold")
    text(root, 90, 162, "Near nose", 18, weight="bold")
    text(root, 92, 564, "Rows increase toward tail", 18, weight="bold")
    text(root, 535, 245, "0: nose-side, left", 21, weight="bold")
    text(root, 535, 285, "0 to 1: body right", 20)
    text(root, 535, 325, "0 to 7: toward tail", 20)
    text(root, 535, 386, "DOWN goes into this page.", 17, color="#555555")
    text(root, 535, 425, "Mark zero on the border,", 17, color="#555555")
    text(root, 535, 452, "not on a crossing.", 17, color="#555555")
    text(root, 40, 623, "Confirm labels on the saved preview before trusting the rotation.", 21, AMBER, "bold")
    text(root, 40, 658, "This ordering applies to the 7-column inner-corner pattern shown here.", 17, color="#555555")
    save(root, "mount")


if __name__ == "__main__":
    target()
    coverage()
    mount()