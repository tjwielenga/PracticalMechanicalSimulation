#!/usr/bin/env python3
"""Generate the vector figures for the numerical-stiffness paper."""

from __future__ import annotations

import cmath
import html
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
OUTPUT = ROOT / "docs/common/assets/numerical-stiffness"
OUTPUT.mkdir(parents=True, exist_ok=True)

INK = "#20252b"
MUTED = "#5f6b76"
ACCENT = "#236a9a"
FILL = "#dcebf4"
PALE = "#f5f8fa"
WHITE = "#ffffff"


def document(width: int, height: int, title: str, body: str, extra_defs: str = "") -> str:
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">
  <title id="title">{html.escape(title)}</title>
  <desc id="desc">Clean vector redraw for the numerical-stiffness paper.</desc>
  <defs>
    <marker id="arrow" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="{INK}"/>
    </marker>
    <marker id="blue-arrow" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="{ACCENT}"/>
    </marker>
    {extra_defs}
  </defs>
  <style>
    text {{ font-family: Arial, Helvetica, sans-serif; fill: {INK}; font-size: 17px; }}
    .small {{ font-size: 14px; }}
    .label {{ font-size: 18px; font-weight: 600; }}
    .panel-title {{ font-size: 17px; font-weight: 600; }}
    .axis {{ stroke: {INK}; stroke-width: 2.2; fill: none; }}
    .tick {{ stroke: {INK}; stroke-width: 1.2; }}
    .guide {{ stroke: {MUTED}; stroke-width: 1.4; stroke-dasharray: 6 5; fill: none; }}
    .curve {{ stroke: {ACCENT}; stroke-width: 2.5; fill: none; }}
    .region {{ fill: {FILL}; stroke: {ACCENT}; stroke-width: 2.2; }}
    .point {{ stroke: {INK}; stroke-width: 3; }}
  </style>
  {body}
</svg>
'''


def line(x1, y1, x2, y2, cls="axis", marker="") -> str:
    marker_attr = f' marker-end="url(#{marker})"' if marker else ""
    return f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" class="{cls}"{marker_attr}/>'


def text(x, y, value, cls="", anchor="start", rotate=None) -> str:
    transform = f' transform="rotate({rotate} {x} {y})"' if rotate is not None else ""
    class_attr = f' class="{cls}"' if cls else ""
    return f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}"{class_attr}{transform}>{html.escape(value)}</text>'


def cross(x, y, size=7, color=INK) -> str:
    return (
        f'<path d="M {x-size:.2f} {y-size:.2f} L {x+size:.2f} {y+size:.2f} '
        f'M {x-size:.2f} {y+size:.2f} L {x+size:.2f} {y-size:.2f}" '
        f'stroke="{color}" stroke-width="3" fill="none"/>'
    )


def circle_label(x, y, label) -> str:
    return (
        f'<circle cx="{x}" cy="{y}" r="13" fill="{WHITE}" stroke="{INK}" stroke-width="2"/>'
        + text(x, y + 5, str(label), "small", "middle")
    )


def save(number: int, title: str, width: int, height: int, body: str, defs: str = "") -> None:
    (OUTPUT / f"figure-{number}.svg").write_text(
        document(width, height, title, body, defs), encoding="utf-8"
    )


def figure_1() -> None:
    width, height = 720, 470
    ox, oy = 390, 245
    body = [line(85, oy, 655, oy), line(ox, 410, ox, 55)]
    body += [text(650, oy - 14, "Real λ", "label", "end"), text(ox + 16, 72, "Imaginary λ", "label")]

    # Eigenvalue locations.
    locations = [
        (150, oy, 1, 150, oy + 38),
        (ox, 105, 2, ox + 56, 94),
        (ox, 385, 2, ox + 56, 396),
        (245, 105, 3, 212, 83),
        (245, 385, 3, 212, 407),
    ]
    for x, y, label, lx, ly in locations:
        body.append(cross(x, y))
        body.append(circle_label(lx, ly, label))

    px, py = 245, 105
    body += [line(px, py, px, oy, "guide"), line(px, py, ox, py, "guide")]
    body.append(f'<path d="M {ox-60} {oy} A 60 60 0 0 1 {ox-43} {oy-43}" class="curve"/>')
    body += [
        line(ox, oy, px, py, "curve"),
        text(ox - 81, oy - 19, "α", "label", "middle"),
        text(px + 57, oy + 25, "−σ", "label", "middle"),
        text(ox + 18, py + 40, "ωd", "label"),
        text(306, 160, "ωₙ", "label", "middle", -44),
    ]
    save(1, "Typical eigenvalues on the complex plane", width, height, "\n".join(body))


def figure_2() -> None:
    width, height = 620, 500
    ox, oy, radius = 310, 250, 155
    body = [
        f'<circle cx="{ox}" cy="{oy}" r="{radius}" class="region"/>',
        line(65, oy, 560, oy),
        line(ox, 455, ox, 45),
        text(555, oy - 15, "Real hλ", "label", "end"),
        text(ox + 16, 65, "Imaginary hλ", "label"),
    ]
    theta = math.radians(138)
    bx, by = ox + radius * math.cos(theta), oy - radius * math.sin(theta)
    body += [line(ox, oy, bx, by, "curve", "blue-arrow"), text((ox + bx) / 2 - 8, (oy + by) / 2 - 10, "Rₖ", "label", "middle")]
    body += [cross(ox - 92, oy - 58, 6), cross(ox - 88, oy + 66, 6)]
    save(2, "Accuracy region", width, height, "\n".join(body))


def figure_3() -> None:
    width, height = 650, 540
    ox, oy = 325, 270
    values = [0.014, 0.076, 0.17, 0.26, 0.34, 0.41]
    scale = 480
    body = [line(55, oy, 595, oy), line(ox, 500, ox, 45)]
    body += [text(590, oy - 14, "Real hλ", "label", "end"), text(ox + 16, 62, "Imaginary hλ", "label")]
    for order, value in reversed(list(enumerate(values, start=1))):
        radius = value * scale
        body.append(f'<circle cx="{ox}" cy="{oy}" r="{radius:.2f}" fill="none" stroke="{ACCENT}" stroke-width="2.2"/>')
        angle = math.radians(30)
        x = ox + radius * math.cos(angle)
        y = oy - radius * math.sin(angle)
        body.append(text(x + 7, y - 3, str(order), "small"))
    for value in [-0.4, -0.3, -0.2, -0.1]:
        x = ox + value * scale
        body += [line(x, oy - 5, x, oy + 5, "tick"), text(x, oy + 24, f"{value:.1f}", "small", "middle")]
    body.append(text(500, 95, "order k", "label"))
    save(3, "Gear accuracy limits", width, height, "\n".join(body))


AM = {
    1: ([1, -1], [1, 0]),
    2: ([1, -1], [1 / 2, 1 / 2]),
    3: ([1, -1, 0], [5 / 12, 8 / 12, -1 / 12]),
    4: ([1, -1, 0, 0], [9 / 24, 19 / 24, -5 / 24, 1 / 24]),
    5: ([1, -1, 0, 0, 0], [251 / 720, 646 / 720, -264 / 720, 106 / 720, -19 / 720]),
    6: ([1, -1, 0, 0, 0, 0], [475 / 1440, 1427 / 1440, -798 / 1440, 482 / 1440, -173 / 1440, 27 / 1440]),
}

BDF = {
    1: ([1, -1], [1, 0]),
    2: ([3 / 2, -2, 1 / 2], [1, 0, 0]),
    3: ([11 / 6, -3, 3 / 2, -1 / 3], [1, 0, 0, 0]),
    4: ([25 / 12, -4, 3, -4 / 3, 1 / 4], [1, 0, 0, 0, 0]),
    5: ([137 / 60, -5, 5, -10 / 3, 5 / 4, -1 / 5], [1, 0, 0, 0, 0, 0]),
    6: ([147 / 60, -6, 15 / 2, -20 / 3, 15 / 4, -6 / 5, 1 / 6], [1, 0, 0, 0, 0, 0, 0]),
}


def polynomial(coefficients, z):
    degree = len(coefficients) - 1
    return sum(value * z ** (degree - index) for index, value in enumerate(coefficients))


def boundary(coefficients, samples=720):
    alpha, beta = coefficients
    points = []
    for index in range(samples + 1):
        theta = 2 * math.pi * index / samples
        z = cmath.exp(1j * theta)
        denominator = polynomial(beta, z)
        if abs(denominator) < 1e-8:
            points.append(None)
        else:
            points.append(polynomial(alpha, z) / denominator)
    return points


def mapped_path(points, x, y, width, height, xlim, ylim, close=False):
    xmin, xmax = xlim
    ymin, ymax = ylim

    def mapped(point):
        px = x + (point.real - xmin) / (xmax - xmin) * width
        py = y + height - (point.imag - ymin) / (ymax - ymin) * height
        return px, py

    commands = []
    drawing = False
    for point in points:
        if point is None or abs(point.real) > 1e4 or abs(point.imag) > 1e4:
            drawing = False
            continue
        px, py = mapped(point)
        commands.append(f'{"L" if drawing else "M"} {px:.2f} {py:.2f}')
        drawing = True
    if close:
        commands.append("Z")
    return " ".join(commands)


def stability_panel(method, order, x, y, width, height, xlim, ylim, clip_id):
    xmin, xmax = xlim
    ymin, ymax = ylim

    def mx(value):
        return x + (value - xmin) / (xmax - xmin) * width

    def my(value):
        return y + height - (value - ymin) / (ymax - ymin) * height

    coefficients = AM[order] if method == "am" else BDF[order]
    points = boundary(coefficients)
    path = mapped_path(points, x, y, width, height, xlim, ylim, close=True)
    pieces = [f'<g clip-path="url(#{clip_id})">']
    if method == "bdf" or order == 1:
        pieces.append(f'<rect x="{x}" y="{y}" width="{width}" height="{height}" fill="{FILL}"/>')
        pieces.append(f'<path d="{path}" fill="{WHITE}" stroke="{ACCENT}" stroke-width="2"/>')
    elif order == 2:
        pieces.append(f'<rect x="{x}" y="{y}" width="{max(0, mx(0)-x)}" height="{height}" fill="{FILL}"/>')
        pieces.append(line(mx(0), y, mx(0), y + height, "curve"))
    else:
        pieces.append(f'<path d="{path}" class="region"/>')
    pieces.append("</g>")
    if xmin <= 0 <= xmax:
        pieces.append(line(mx(0), y, mx(0), y + height, "axis"))
    if ymin <= 0 <= ymax:
        pieces.append(line(x, my(0), x + width, my(0), "axis"))
    pieces.append(f'<rect x="{x}" y="{y}" width="{width}" height="{height}" fill="none" stroke="{MUTED}" stroke-width="1"/>')
    for value in range(math.ceil(xmin / 2) * 2, math.floor(xmax / 2) * 2 + 1, 2):
        if value == 0:
            continue
        px = mx(value)
        pieces += [line(px, my(0) - 4, px, my(0) + 4, "tick"), text(px, my(0) + 18, str(value), "small", "middle")]
    pieces += [
        text(x + 8, y + 21, f"{order}{'st' if order == 1 else 'nd' if order == 2 else 'rd' if order == 3 else 'th'} order", "panel-title"),
        text(x + width - 6, my(0) - 8, "Real hλ", "small", "end"),
        text(mx(0) + 7, y + 16, "Imaginary hλ", "small"),
    ]
    return "\n".join(pieces)


def stability_figure(number, method, title, xlim, ylim):
    # Equal horizontal and vertical scales are essential in the hλ plane:
    # angles, circles, and stability wedges otherwise appear distorted.
    panel_height = 310
    panel_width = round(panel_height * (xlim[1] - xlim[0]) / (ylim[1] - ylim[0]))
    column_gap = 50
    row_gap = 28
    left = 45
    top = 38
    x_positions = [left, left + panel_width + column_gap]
    y_positions = [top + row * (panel_height + row_gap) for row in range(3)]
    width = left * 2 + panel_width * 2 + column_gap
    height = top * 2 + panel_height * 3 + row_gap * 2
    defs = []
    body = []
    order = 1
    for row, y in enumerate(y_positions):
        for column, x in enumerate(x_positions):
            clip_id = f"clip-{number}-{order}"
            defs.append(f'<clipPath id="{clip_id}"><rect x="{x}" y="{y}" width="{panel_width}" height="{panel_height}"/></clipPath>')
            body.append(stability_panel(method, order, x, y, panel_width, panel_height, xlim, ylim, clip_id))
            order += 1
    save(number, title, width, height, "\n".join(body), "\n".join(defs))


def figure_6() -> None:
    width, height = 640, 600
    x, y, ph = 170, 50, 480
    xlim, ylim = (-4.0, 1.0), (-4.0, 4.0)
    pw = ph * (xlim[1] - xlim[0]) / (ylim[1] - ylim[0])
    clip_id = "clip-combined"
    defs = f'<clipPath id="{clip_id}"><rect x="{x}" y="{y}" width="{pw}" height="{ph}"/></clipPath>'

    def mx(value):
        return x + (value - xlim[0]) / (xlim[1] - xlim[0]) * pw

    def my(value):
        return y + ph - (value - ylim[0]) / (ylim[1] - ylim[0]) * ph

    path = mapped_path(boundary(BDF[5]), x, y, pw, ph, xlim, ylim, close=True)
    accuracy_radius = 0.34 * pw / (xlim[1] - xlim[0])
    body = [
        f'<g clip-path="url(#{clip_id})">',
        f'<rect x="{x}" y="{y}" width="{pw}" height="{ph}" fill="{FILL}"/>',
        f'<path d="{path}" fill="{WHITE}" stroke="{ACCENT}" stroke-width="2.5"/>',
        f'<circle cx="{mx(0)}" cy="{my(0)}" r="{accuracy_radius:.2f}" fill="{ACCENT}" opacity="0.95"/>',
        '</g>',
        line(mx(0), y, mx(0), y + ph),
        line(x, my(0), x + pw, my(0)),
        f'<rect x="{x}" y="{y}" width="{pw}" height="{ph}" fill="none" stroke="{MUTED}" stroke-width="1"/>',
        text(215, 145, "stable", "label"),
        line(460, 155, mx(0) + 14, my(0) - 9, "curve", "blue-arrow"),
        text(470, 145, "accurate", "label", "end"),
        text(x + pw + 72, my(0) - 12, "Real hλ", "label", "end"),
        text(mx(0) + 14, y + 28, "Imaginary hλ", "label"),
    ]
    save(6, "Combined accuracy and stability regions for fifth-order Gear", width, height, "\n".join(body), defs)


def figure_7() -> None:
    width, height = 720, 460
    table_y = 345
    body = [line(80, table_y, 650, table_y), line(335, table_y, 335, 255, "axis", "arrow")]
    body += [text(650, table_y - 13, "x", "label", "end"), text(351, 270, "y", "label")]
    body.append(f'<circle cx="335" cy="135" r="62" fill="{PALE}" stroke="{INK}" stroke-width="2.5"/>')
    body += [line(335, 135, 335, 215, "axis", "arrow"), text(349, 175, "mg", "label")]
    body += [line(400, 150, 480, 195, "curve", "blue-arrow"), text(488, 205, "v", "label")]
    save(7, "Ball above table", width, height, "\n".join(body))


def figure_8() -> None:
    width, height = 720, 470
    table_y = 285
    cx, cy, radius = 470, 230, 72
    body = [line(70, table_y, 655, table_y), line(175, table_y, 175, 190, "axis", "arrow")]
    body += [text(650, table_y - 13, "x", "label", "end"), text(191, 205, "y", "label")]
    body.append(f'<circle cx="{cx}" cy="{cy}" r="{radius}" fill="{PALE}" stroke="{INK}" stroke-width="2.5"/>')
    body += [line(cx, cy, cx, table_y, "guide"), line(cx, cy, cx + radius - 7, cy, "guide")]
    body += [text(cx + 37, cy - 12, "r", "label"), text(cx + 18, table_y + 34, "d = y − r", "label")]
    body += [line(cx + 20, cy, cx + 20, table_y - 8, "axis", "arrow"), text(cx + 34, cy + 34, "mg", "label")]
    body += [line(cx - 10, 390, cx - 10, table_y + 6, "curve", "blue-arrow"), text(cx - 25, 407, "F", "label", "middle")]
    save(8, "Ball contacting table", width, height, "\n".join(body))


def figure_9() -> None:
    width, height = 720, 500
    ox, oy, radius = 430, 250, 120
    body = [
        f'<circle cx="{ox}" cy="{oy}" r="{radius}" fill="none" stroke="{ACCENT}" stroke-width="2.5"/>',
        line(65, oy, 655, oy),
        line(ox, 440, ox, 60),
        text(650, oy - 14, "Real hλ", "label", "end"),
        text(ox + 14, 78, "Imaginary hλ", "label"),
    ]
    start_points = [(115, 175), (115, 335)]
    for sx, sy in start_points:
        body.append(cross(sx, sy, 8))
        body.append(line(sx + 12, sy, ox - 8, oy, "curve", "blue-arrow"))
    body.append(cross(ox, oy, 8, ACCENT))
    body.append(text(180, 418, "decreasing h moves hλ toward the origin", "small"))
    save(9, "Eigenvalue movement", width, height, "\n".join(body))


def main() -> None:
    figure_1()
    figure_2()
    figure_3()
    stability_figure(4, "am", "Adams-Moulton stability regions", (-6, 2), (-4, 4))
    stability_figure(5, "bdf", "Gear stability regions", (-8, 8), (-8, 8))
    figure_6()
    figure_7()
    figure_8()
    figure_9()


if __name__ == "__main__":
    main()
