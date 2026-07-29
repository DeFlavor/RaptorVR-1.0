from __future__ import annotations

import csv
import heapq
import io
import json
import math
import shutil
import textwrap
import zipfile
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
from matplotlib.path import Path as MplPath
from matplotlib.textpath import TextPath
from matplotlib.transforms import Affine2D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import trimesh
from PIL import Image
from shapely.affinity import scale as scale_geometry
from shapely.affinity import translate as translate_geometry
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_BYTES = SCRIPT_PATH.read_bytes()
ROOT = next(
    (
        candidate
        for candidate in SCRIPT_PATH.parents
        if (candidate / "work").is_dir() and (candidate / "outputs").is_dir()
    ),
    SCRIPT_PATH.parents[1],
)
OUT = ROOT / "outputs" / "esp32-mumo-tracker"
PCB = OUT / "hardware" / "pcb"
GERBERS = PCB / "gerbers"
SCHEMATIC = PCB / "schematic"
SOURCE = PCB / "source"
PRINTABLES = OUT / "hardware" / "enclosure"
PREVIEWS = OUT / "previews"
LOGO_ASSET = Path(__file__).resolve().parent / "assets" / "RaptorVR_logo_reference.png"
LOGO_PNG_BYTES = LOGO_ASSET.read_bytes()

GRID = 0.254  # 10 mil routing grid
BOARD_W = 86.36
BOARD_H = 53.34
BOARD_R = 5.08
CLEARANCE = 0.250
ROUTER_MARGIN = 0.18  # absorbs 10 mil grid quantization so exact geometry still clears 0.25 mm


@dataclass
class Pad:
    ref: str
    pin: str
    x: float
    y: float
    net: str
    diameter: float = 1.80
    drill: float = 1.00
    plated: bool = True
    label: str = ""

    @property
    def key(self):
        return f"{self.ref}.{self.pin}"


@dataclass
class Component:
    ref: str
    value: str
    description: str
    outline: tuple[float, float, float, float]
    pads: list[Pad] = field(default_factory=list)


@dataclass
class Track:
    net: str
    layer: str
    points: list[tuple[float, float]]
    width: float


@dataclass
class Via:
    net: str
    x: float
    y: float
    diameter: float = 1.20
    drill: float = 0.40


def mm(v: int) -> float:
    return round(v * GRID, 6)


def rounded_rect_polygon(w: float, h: float, r: float, inset: float = 0.0) -> Polygon:
    return unary_union([
        box(inset + r, inset, w - inset - r, h - inset),
        box(inset, inset + r, w - inset, h - inset - r),
        Point(inset + r, inset + r).buffer(r, resolution=32),
        Point(w - inset - r, inset + r).buffer(r, resolution=32),
        Point(w - inset - r, h - inset - r).buffer(r, resolution=32),
        Point(inset + r, h - inset - r).buffer(r, resolution=32),
    ])


def build_components() -> tuple[list[Component], list[Pad], list[Pad]]:
    components: list[Component] = []
    pads: list[Pad] = []
    npth: list[Pad] = []

    # ELEGOO ESP-WROOM-32 USB-C 30-pin board. USB-C faces the lower board edge.
    left_names = [
        ("VIN", "SYS_5V"), ("GND", "GND"), ("GPIO13", "NC"), ("GPIO12", "NC"),
        ("GPIO14", "NC"), ("GPIO27", "NC"), ("GPIO26", "NC"), ("GPIO25", "NC"),
        ("GPIO33", "NC"), ("GPIO32", "NC"), ("GPIO35", "NC"), ("GPIO34", "NC"),
        ("GPIO39", "NC"), ("GPIO36", "ADC_BAT"), ("EN", "NC"),
    ]
    right_names = [
        ("3V3", "3V3"), ("GPIO15", "NC"), ("GPIO2", "NC"), ("GPIO0", "NC"),
        ("GPIO4", "NC"), ("GPIO16", "NC"), ("GPIO17", "INT1"), ("GPIO5", "CS"),
        ("GPIO18", "SCLK"), ("GPIO19", "MISO"), ("GPIO21", "NC"), ("RX0", "NC"),
        ("TX0", "NC"), ("GPIO22", "NC"), ("GPIO23", "MOSI"),
    ]
    esp_pads = []
    for side, x, names in [("L", mm(30), left_names), ("R", mm(130), right_names)]:
        for i, (label, net) in enumerate(names):
            p = Pad("U1", f"{side}{i+1}", x, mm(35 + i * 10), net, label=label)
            esp_pads.append(p)
    components.append(Component("U1", "ELEGOO ESP-WROOM-32 USB-C 30-pin", "30-pin ESP32 development board", (mm(20), mm(20), mm(140), mm(195)), esp_pads))
    pads.extend(esp_pads)

    # MuMo V1.1 footprint: 7 top pins and 6 bottom pins, centered on an 18 x 13 mm board.
    j1_signals = [
        ("OSDO", "NC"), ("3V3", "3V3"), ("GND", "GND"), ("SCL/SCK", "SCLK"),
        ("SDA/MOSI", "MOSI"), ("CS", "CS"), ("SDO/MISO", "MISO"),
    ]
    j2_signals = [
        ("OCS", "NC"), ("CLK", "NC"), ("INT1", "INT1"), ("SCX", "NC"),
        ("SDX", "NC"), ("CLK_CTL", "NC"),
    ]
    mumo_pads = []
    for i, (label, net) in enumerate(j1_signals):
        mumo_pads.append(Pad("J1", str(i + 1), mm(187 + i * 10), mm(110), net, label=label))
    for i, (label, net) in enumerate(j2_signals):
        mumo_pads.append(Pad("J2", str(i + 1), mm(192 + i * 10), mm(58), net, label=label))
    components.append(Component("U2", "MuMo V1.1 ICM-45686 + QMC6309", "13 x 18 mm dual-row castellated breakout", (mm(182), mm(55), mm(257), mm(113)), mumo_pads))
    pads.extend(mumo_pads)

    # Common protected TP4056 USB-C module (about 26 x 17 mm). USB-C faces right.
    tp = [
        Pad("U3", "1", mm(323), mm(188), "USB_IN", 2.00, 1.20, label="IN+"),
        Pad("U3", "2", mm(323), mm(154), "GND", 2.00, 1.20, label="IN-"),
        Pad("U3", "4", mm(229), mm(166), "GND", 2.00, 1.20, label="B-"),
        Pad("U3", "5", mm(229), mm(178), "BAT+", 2.00, 1.20, label="B+"),
        Pad("U3", "6", mm(229), mm(188), "CHG_OUT", 2.00, 1.20, label="OUT+"),
        Pad("U3", "7", mm(229), mm(154), "GND", 2.00, 1.20, label="OUT-"),
    ]
    components.append(Component("U3", "TP4056 USB-C protected module", "6-pad single-cell Li-ion charger/protection module", (mm(225), mm(143), mm(327), mm(207)), tp))
    pads.extend(tp)

    # Axial 1N5817 diodes, cathodes toward the power-OR node.
    d1 = [Pad("D1", "1", mm(257), mm(123), "USB_IN", 2.20, 1.00, label="A"), Pad("D1", "2", mm(297), mm(123), "PWR_OR", 2.20, 1.00, label="K")]
    d2 = [Pad("D2", "1", mm(257), mm(105), "CHG_OUT", 2.20, 1.00, label="A"), Pad("D2", "2", mm(297), mm(105), "PWR_OR", 2.20, 1.00, label="K")]
    components.extend([
        Component("D1", "1N5817", "USB-to-load Schottky diode", (mm(252), mm(119), mm(302), mm(127)), d1),
        Component("D2", "1N5817", "battery-to-load Schottky diode", (mm(252), mm(101), mm(302), mm(109)), d2),
    ])
    pads.extend(d1 + d2)

    # Battery divider matching SlimeVR BOARD_WROOM32 defaults: 180k + 220k over 100k.
    r180 = [Pad("R1", "1", mm(150), mm(182), "BAT+", label="BAT+"), Pad("R1", "2", mm(190), mm(182), "DIV_A", label="DIV_A")]
    r220 = [Pad("R2", "1", mm(199), mm(174), "DIV_A", label="DIV_A"), Pad("R2", "2", mm(199), mm(134), "ADC_BAT", label="ADC")]
    r100 = [Pad("R3", "1", mm(151), mm(126), "GND", label="GND"), Pad("R3", "2", mm(191), mm(126), "ADC_BAT", label="ADC")]
    components.extend([
        Component("R1", "180k 1/4W", "battery sense series resistor", (mm(146), mm(178), mm(194), mm(186)), r180),
        Component("R2", "220k 1/4W", "battery sense upper divider resistor", (mm(195), mm(130), mm(203), mm(178)), r220),
        Component("R3", "100k 1/4W", "battery sense lower divider resistor", (mm(147), mm(122), mm(195), mm(130)), r100),
    ])
    pads.extend(r180 + r220 + r100)

    # SK12D07/SK12D07VG. Center common, right pad ON; left pad unused/OFF.
    sw = [
        Pad("SW1", "1", mm(295), mm(72), "NC", 2.20, 1.10, label="OFF"),
        Pad("SW1", "2", mm(305), mm(72), "PWR_OR", 2.20, 1.10, label="COM"),
        Pad("SW1", "3", mm(315), mm(72), "SYS_5V", 2.20, 1.10, label="ON"),
    ]
    components.append(Component("SW1", "SK12D07VG3 high-3mm", "SPDT slide power switch", (mm(286), mm(62), mm(324), mm(82)), sw))
    pads.extend(sw)
    npth.extend([
        Pad("SW1", "M1", mm(288), mm(60), "NPTH", 1.80, 1.80, False),
        Pad("SW1", "M2", mm(322), mm(60), "NPTH", 1.80, 1.80, False),
    ])

    # Large wire pads for the single-cell LiPo.
    bat = [
        Pad("J3", "+", mm(270), mm(32), "BAT+", 4.00, 2.00, label="BAT+"),
        Pad("J3", "-", mm(300), mm(32), "GND", 4.00, 2.00, label="BAT-")
    ]
    components.append(Component("J3", "LiPo wire pads", "3.7 V single-cell battery connection", (mm(263), mm(24), mm(307), mm(40)), bat))
    pads.extend(bat)

    # Mounting holes matching the separator tray standoffs.
    for i, (x, y) in enumerate([(mm(15), mm(15)), (mm(325), mm(15)), (mm(15), mm(195)), (mm(210), mm(195))], 1):
        npth.append(Pad("H", str(i), x, y, "NPTH", 3.00, 2.60, False))

    return components, pads, npth


def prim_edges(items: list[Pad]) -> list[tuple[Pad, Pad]]:
    if len(items) < 2:
        return []
    used = {0}
    edges = []
    while len(used) < len(items):
        best = None
        for i in used:
            for j in range(len(items)):
                if j in used:
                    continue
                d = (items[i].x - items[j].x) ** 2 + (items[i].y - items[j].y) ** 2
                if best is None or d < best[0]:
                    best = (d, i, j)
        assert best
        _, i, j = best
        edges.append((items[i], items[j]))
        used.add(j)
    return edges


class Router:
    def __init__(self, pads: list[Pad], npth: list[Pad]):
        self.pads = pads
        self.npth = npth
        self.nx = round(BOARD_W / GRID)
        self.ny = round(BOARD_H / GRID)
        self.occ = [defaultdict(set), defaultdict(set)]  # cell -> nets occupying expanded copper
        self.hard = [set(), set()]
        self.vias: list[Via] = []
        self.tracks: list[Track] = []
        self.board_safe = rounded_rect_polygon(BOARD_W, BOARD_H, BOARD_R).buffer(-0.55)
        self._mark_pad_obstacles()

    def _grid_xy(self, x: float, y: float) -> tuple[int, int]:
        return round(x / GRID), round(y / GRID)

    def _in_board(self, ix: int, iy: int) -> bool:
        if ix < 1 or iy < 1 or ix >= self.nx or iy >= self.ny:
            return False
        return self.board_safe.contains(Point(ix * GRID, iy * GRID))

    def _mark_disc(self, ix: int, iy: int, radius_mm: float, target, value):
        rr = math.ceil(radius_mm / GRID)
        for dx in range(-rr, rr + 1):
            for dy in range(-rr, rr + 1):
                if (dx * GRID) ** 2 + (dy * GRID) ** 2 <= radius_mm ** 2:
                    target[(ix + dx, iy + dy)].add(value) if isinstance(target, defaultdict) else target.add((ix + dx, iy + dy))

    def _mark_pad_obstacles(self):
        for p in self.pads:
            if p.net == "NC":
                net = p.key
            else:
                net = p.net
            ix, iy = self._grid_xy(p.x, p.y)
            for layer in range(2):
                self._mark_disc(ix, iy, p.diameter / 2 + CLEARANCE + ROUTER_MARGIN, self.occ[layer], net)
        for p in self.npth:
            ix, iy = self._grid_xy(p.x, p.y)
            for layer in range(2):
                self._mark_disc(ix, iy, p.drill / 2 + 0.60, self.hard[layer], "HARD")

    def _cell_ok(self, ix: int, iy: int, layer: int, net: str, width: float = 0.40, via: bool = False) -> bool:
        if not self._in_board(ix, iy):
            return False
        radius = 0.60 if via else width / 2
        rr = math.ceil(radius / GRID)
        layers = (0, 1) if via else (layer,)
        for ll in layers:
            for dx in range(-rr, rr + 1):
                for dy in range(-rr, rr + 1):
                    if (dx * GRID) ** 2 + (dy * GRID) ** 2 > radius ** 2:
                        continue
                    cell = (ix + dx, iy + dy)
                    if cell in self.hard[ll]:
                        return False
                    values = self.occ[ll].get(cell, set())
                    if values and any(v != net for v in values):
                        return False
        return True

    def _astar(self, a: Pad, b: Pad, net: str, width: float) -> list[tuple[int, int, int]]:
        sx, sy = self._grid_xy(a.x, a.y)
        gx, gy = self._grid_xy(b.x, b.y)
        starts = [(sx, sy, 0), (sx, sy, 1)]
        goals = {(gx, gy, 0), (gx, gy, 1)}
        pq = []
        prev = {}
        gscore = {}
        for s in starts:
            gscore[s] = 0.0
            heapq.heappush(pq, (abs(sx - gx) + abs(sy - gy), 0.0, s, None))
        visited = set()
        max_iter = self.nx * self.ny * 3
        iters = 0
        while pq and iters < max_iter:
            _, cost, state, direction = heapq.heappop(pq)
            iters += 1
            if state in visited:
                continue
            visited.add(state)
            if state in goals:
                path = [state]
                while state in prev:
                    state = prev[state]
                    path.append(state)
                return list(reversed(path))
            x, y, layer = state
            for dx, dy, ndir in ((1, 0, 0), (-1, 0, 0), (0, 1, 1), (0, -1, 1)):
                nx, ny = x + dx, y + dy
                ns = (nx, ny, layer)
                if not self._cell_ok(nx, ny, layer, net, width=width):
                    continue
                turn = 0.18 if direction is not None and ndir != direction else 0.0
                nc = cost + 1.0 + turn
                if nc < gscore.get(ns, 1e30):
                    gscore[ns] = nc
                    prev[ns] = state
                    h = abs(nx - gx) + abs(ny - gy)
                    heapq.heappush(pq, (nc + h, nc, ns, ndir))
            other = 1 - layer
            ns = (x, y, other)
            if self._cell_ok(x, y, layer, net, width=width, via=True):
                nc = cost + 14.0
                if nc < gscore.get(ns, 1e30):
                    gscore[ns] = nc
                    prev[ns] = state
                    h = abs(x - gx) + abs(y - gy)
                    heapq.heappush(pq, (nc + h, nc, ns, direction))
        raise RuntimeError(f"Autorouter failed for {net}: {a.key} -> {b.key}")

    def _commit(self, path: list[tuple[int, int, int]], net: str, width: float):
        # Mark copper occupancy with clearance.
        route_radius = width / 2 + CLEARANCE + ROUTER_MARGIN
        rr = math.ceil(route_radius / GRID)
        for x, y, layer in path:
            for dx in range(-rr, rr + 1):
                for dy in range(-rr, rr + 1):
                    if (dx * GRID) ** 2 + (dy * GRID) ** 2 <= route_radius ** 2:
                        self.occ[layer][(x + dx, y + dy)].add(net)

        # Split by layer and compress collinear grid walks.
        chunks = []
        chunk = [path[0]]
        for item in path[1:]:
            if item[2] != chunk[-1][2]:
                chunks.append(chunk)
                chunk = [item]
            else:
                chunk.append(item)
        chunks.append(chunk)
        for ch in chunks:
            if len(ch) < 2:
                continue
            pts = [(ch[0][0], ch[0][1])]
            last_dir = None
            for i in range(1, len(ch)):
                d = (ch[i][0] - ch[i - 1][0], ch[i][1] - ch[i - 1][1])
                if last_dir is not None and d != last_dir:
                    pts.append((ch[i - 1][0], ch[i - 1][1]))
                last_dir = d
            pts.append((ch[-1][0], ch[-1][1]))
            mmpts = [(mm(x), mm(y)) for x, y in pts]
            self.tracks.append(Track(net, "F.Cu" if ch[0][2] == 0 else "B.Cu", mmpts, width))

        for i in range(1, len(path)):
            if path[i][2] != path[i - 1][2]:
                x, y, _ = path[i]
                if not any(abs(v.x - mm(x)) < 1e-6 and abs(v.y - mm(y)) < 1e-6 and v.net == net for v in self.vias):
                    via = Via(net, mm(x), mm(y))
                    self.vias.append(via)
                    for ll in (0, 1):
                        self._mark_disc(x, y, via.diameter / 2 + CLEARANCE + ROUTER_MARGIN, self.occ[ll], net)

    def route_all(self):
        net_pads: dict[str, list[Pad]] = defaultdict(list)
        for p in self.pads:
            if p.net not in ("NC", "NPTH"):
                net_pads[p.net].append(p)

        order = [
            "USB_IN", "CHG_OUT", "PWR_OR", "SYS_5V", "BAT+", "3V3", "GND",
            "SCLK", "MOSI", "MISO", "CS", "INT1", "ADC_BAT", "DIV_A",
        ]
        widths = defaultdict(lambda: 0.40, {"USB_IN": 0.80, "CHG_OUT": 0.80, "PWR_OR": 1.00, "SYS_5V": 1.00, "BAT+": 1.00, "GND": 1.00, "3V3": 0.60})
        for net in order:
            items = net_pads[net]
            for a, b in prim_edges(items):
                path = self._astar(a, b, net, widths[net])
                self._commit(path, net, widths[net])
        return self.tracks, self.vias


def gerber_coord(v: float) -> str:
    return f"{round(v * 1_000_000):010d}"


def gerber_header(name: str) -> list[str]:
    return [
        f"G04 {name}*", "%FSLAX46Y46*%", "%MOMM*%", "%IPPOS*%", "%LPD*%",
        "%ADD10C,0.400*%", "%ADD11C,0.600*%", "%ADD12C,0.800*%", "%ADD13C,1.000*%",
        "%ADD14C,1.200*%", "%ADD15C,1.800*%", "%ADD16C,2.200*%", "%ADD17C,2.400*%",
        "%ADD18C,4.000*%", "%ADD19C,0.180*%", "G01*",
    ]


def flash(lines, x, y, dcode):
    lines.append(f"D{dcode}*")
    lines.append(f"X{gerber_coord(x)}Y{gerber_coord(y)}D03*")


def draw_polyline(lines, points, dcode):
    if len(points) < 2:
        return
    lines.append(f"D{dcode}*")
    x, y = points[0]
    lines.append(f"X{gerber_coord(x)}Y{gerber_coord(y)}D02*")
    for x, y in points[1:]:
        lines.append(f"X{gerber_coord(x)}Y{gerber_coord(y)}D01*")


def write_region(lines, poly: list[tuple[float, float]]):
    if len(poly) < 3:
        return
    lines.append("G36*")
    lines.append(f"X{gerber_coord(poly[0][0])}Y{gerber_coord(poly[0][1])}D02*")
    for x, y in poly[1:]:
        lines.append(f"X{gerber_coord(x)}Y{gerber_coord(y)}D01*")
    lines.append(f"X{gerber_coord(poly[0][0])}Y{gerber_coord(poly[0][1])}D01*")
    lines.append("G37*")


def text_polygons(text: str, x: float, y: float, size: float, angle: float = 0.0, center: bool = False):
    path = TextPath((0, 0), text, size=size, prop={"family": "DejaVu Sans", "weight": "bold"})
    polys = path.to_polygons()
    if not polys:
        return []
    allpts = np.vstack(polys)
    minx, miny = allpts.min(axis=0)
    maxx, maxy = allpts.max(axis=0)
    tx = -(minx + maxx) / 2 if center else -minx
    ty = -(miny + maxy) / 2 if center else -miny
    transform = Affine2D().translate(tx, ty).rotate_deg(angle).translate(x, y)
    return [transform.transform(p).tolist() for p in polys]


def write_gerbers(components, pads, npth, tracks, vias):
    GERBERS.mkdir(parents=True, exist_ok=True)
    nets_by_layer = {"F.Cu": [], "B.Cu": []}
    for t in tracks:
        nets_by_layer[t.layer].append(t)

    for layer, filename in [("F.Cu", "RaptorVR_1_0-F_Cu.gtl"), ("B.Cu", "RaptorVR_1_0-B_Cu.gbl")]:
        lines = gerber_header(layer)
        for p in pads:
            dc = 18 if p.diameter >= 3.5 else 17 if p.diameter >= 2.3 else 16 if p.diameter >= 2.0 else 15
            flash(lines, p.x, p.y, dc)
        for v in vias:
            flash(lines, v.x, v.y, 14)
        for t in nets_by_layer[layer]:
            dc = 13 if t.width >= 0.95 else 12 if t.width >= 0.75 else 11 if t.width >= 0.55 else 10
            draw_polyline(lines, t.points, dc)
        lines.append("M02*")
        (GERBERS / filename).write_text("\n".join(lines) + "\n", encoding="ascii")

    for side, filename in [("F", "RaptorVR_1_0-F_Mask.gts"), ("B", "RaptorVR_1_0-B_Mask.gbs")]:
        lines = gerber_header(f"{side}.Mask")
        for p in pads:
            dia = p.diameter + 0.15
            # Define per-size apertures dynamically.
            dc = 18 if dia >= 3.5 else 17 if dia >= 2.3 else 16 if dia >= 2.0 else 15
            flash(lines, p.x, p.y, dc)
        for v in vias:
            flash(lines, v.x, v.y, 14)
        lines.append("M02*")
        (GERBERS / filename).write_text("\n".join(lines) + "\n", encoding="ascii")

    # Top silkscreen: meow-inspired zoning, clear orientation marks, and assembly labels.
    silk = gerber_header("F.Silkscreen")
    for c in components:
        x1, y1, x2, y2 = c.outline
        draw_polyline(silk, [(x1, y1), (x2, y1), (x2, y2), (x1, y2), (x1, y1)], 19)
    labels = [
        ("RAPTORVR 1.0", BOARD_W / 2, BOARD_H - 2.2, 2.2, 0, True),
        ("ESP32 USB", mm(80), 2.0, 1.2, 0, True),
        ("MUMO 1.1", mm(220), mm(118), 1.35, 0, True),
        ("TP4056 USB-C", mm(276), mm(137), 1.15, 0, True),
        ("D1 USB", mm(277), mm(130), 0.9, 0, True),
        ("D2 BAT", mm(277), mm(96), 0.9, 0, True),
        ("OFF  ON", mm(305), mm(88), 0.95, 0, True),
        ("BAT+", mm(270), mm(44), 1.0, 0, True),
        ("BAT-", mm(300), mm(44), 1.0, 0, True),
        ("180K", mm(170), mm(188), 0.9, 0, True),
        ("220K", mm(207), mm(154), 0.9, 90, True),
        ("100K", mm(171), mm(119), 0.9, 0, True),
        ("JLCJLCJLCJLC", BOARD_W / 2, 2.0, 0.75, 0, True),
    ]
    for txt, x, y, size, angle, center in labels:
        for poly in text_polygons(txt, x, y, size, angle, center):
            write_region(silk, poly)
    # Cat-ear line motif, visibly inspired but not copied from meowCarrier artwork.
    cat = [(39.0, 5.2), (40.5, 7.0), (42.0, 5.7), (44.0, 5.7), (45.5, 7.0), (47.0, 5.2)]
    draw_polyline(silk, cat, 19)
    silk.append("M02*")
    (GERBERS / "RaptorVR_1_0-F_Silkscreen.gto").write_text("\n".join(silk) + "\n", encoding="ascii")

    bottom_silk = gerber_header("B.Silkscreen")
    for txt, x, y, size in [
        ("3.7V 1S LIPO ONLY", BOARD_W / 2, BOARD_H - 3.0, 1.35),
        ("PROTECTED TP4056 REQUIRED", BOARD_W / 2, 3.0, 1.1),
    ]:
        for poly in text_polygons(txt, x, y, size, 0, True):
            write_region(bottom_silk, poly)
    bottom_silk.append("M02*")
    (GERBERS / "RaptorVR_1_0-B_Silkscreen.gbo").write_text("\n".join(bottom_silk) + "\n", encoding="ascii")

    outline = gerber_header("Edge.Cuts")
    coords = list(rounded_rect_polygon(BOARD_W, BOARD_H, BOARD_R).exterior.coords)
    draw_polyline(outline, coords, 19)
    outline.append("M02*")
    (GERBERS / "RaptorVR_1_0-Edge_Cuts.gm1").write_text("\n".join(outline) + "\n", encoding="ascii")

    # Excellon drills, separated into plated and non-plated files for JLCPCB.
    def drill_file(path: Path, entries, tools):
        lines = ["M48", ";DRILL file generated for RaptorVR 1.0", "METRIC,TZ"]
        for tid, dia in tools.items():
            lines.append(f"T{tid:02d}C{dia:.3f}")
        lines.append("%")
        grouped = defaultdict(list)
        for e in entries:
            grouped[round(e.drill, 3)].append(e)
        inv = {round(v, 3): k for k, v in tools.items()}
        for dia, es in sorted(grouped.items()):
            lines.append(f"T{inv[dia]:02d}")
            for e in es:
                lines.append(f"X{e.x:.3f}Y{e.y:.3f}")
        lines.append("M30")
        path.write_text("\n".join(lines) + "\n", encoding="ascii")

    plated_entries = pads + [Pad("V", str(i), v.x, v.y, v.net, v.diameter, v.drill, True) for i, v in enumerate(vias)]
    ptools = {1: 0.400, 2: 1.000, 3: 1.100, 4: 1.200, 5: 2.000}
    drill_file(GERBERS / "RaptorVR_1_0-PTH.drl", plated_entries, ptools)
    ntools = {1: 1.800, 2: 2.600}
    drill_file(GERBERS / "RaptorVR_1_0-NPTH.drl", npth, ntools)

    zip_path = PCB / "RaptorVR_1_0_JLCPCB_Gerbers.zip"
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for f in sorted(GERBERS.iterdir()):
            zf.write(f, f.name)
    return zip_path


def kicad_escape(s: str) -> str:
    return s.replace('"', "'")


def write_kicad_pcb(components, pads, npth, tracks, vias):
    SOURCE.mkdir(parents=True, exist_ok=True)
    nets = sorted({p.net for p in pads if p.net != "NC"})
    net_ids = {n: i + 1 for i, n in enumerate(nets)}
    lines = [
        '(kicad_pcb (version 20240108) (generator "RaptorVR 1.0 generator")',
        '  (general (thickness 1.0))',
        '  (paper "A4")',
        '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (36 "B.SilkS" user "b.silkscreen") (37 "F.SilkS" user "f.silkscreen") (44 "Edge.Cuts" user))',
        '  (setup (pad_to_mask_clearance 0))',
        '  (net 0 "")',
    ]
    for n, i in net_ids.items():
        lines.append(f'  (net {i} "{n}")')
    for c in components:
        x1, y1, x2, y2 = c.outline
        lines.append(f'  (footprint "RaptorVR_1_0:{c.ref}" (layer "F.Cu") (at 0 0)')
        lines.append(f'    (property "Reference" "{c.ref}" (at {x1:.3f} {y2 + 1.5:.3f} 0) (layer "F.SilkS"))')
        lines.append(f'    (property "Value" "{kicad_escape(c.value)}" (at {x1:.3f} {y1 - 1.5:.3f} 0) (layer "F.Fab"))')
        lines.append(f'    (fp_rect (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) (stroke (width 0.18) (type default)) (fill none) (layer "F.SilkS"))')
        for p in c.pads:
            netpart = "" if p.net == "NC" else f' (net {net_ids[p.net]} "{p.net}")'
            lines.append(f'    (pad "{p.pin}" thru_hole circle (at {p.x:.3f} {p.y:.3f}) (size {p.diameter:.3f} {p.diameter:.3f}) (drill {p.drill:.3f}) (layers "*.Cu" "*.Mask"){netpart})')
        lines.append("  )")
    for p in npth:
        lines.append(f'  (footprint "RaptorVR_1_0:NPTH_{p.ref}_{p.pin}" (layer "F.Cu") (at 0 0) (pad "" np_thru_hole circle (at {p.x:.3f} {p.y:.3f}) (size {p.drill:.3f} {p.drill:.3f}) (drill {p.drill:.3f}) (layers "*.Cu" "*.Mask")))')
    for t in tracks:
        nid = net_ids[t.net]
        for a, b in zip(t.points, t.points[1:]):
            lines.append(f'  (segment (start {a[0]:.3f} {a[1]:.3f}) (end {b[0]:.3f} {b[1]:.3f}) (width {t.width:.3f}) (layer "{t.layer}") (net {nid}))')
    for v in vias:
        lines.append(f'  (via (at {v.x:.3f} {v.y:.3f}) (size {v.diameter:.3f}) (drill {v.drill:.3f}) (layers "F.Cu" "B.Cu") (net {net_ids[v.net]}))')
    outline = list(rounded_rect_polygon(BOARD_W, BOARD_H, BOARD_R).exterior.coords)
    for a, b in zip(outline, outline[1:]):
        lines.append(f'  (gr_line (start {a[0]:.3f} {a[1]:.3f}) (end {b[0]:.3f} {b[1]:.3f}) (stroke (width 0.18) (type default)) (layer "Edge.Cuts"))')
    lines.append(")")
    (SOURCE / "RaptorVR_1_0.kicad_pcb").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_bom_and_pinout(components, pads):
    PCB.mkdir(parents=True, exist_ok=True)
    with (PCB / "BOM.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Reference", "Qty", "Value / Part", "Footprint / Notes", "Assembly"])
        rows = [
            ("U1", 1, "ELEGOO ESP-WROOM-32 USB-C 30-pin", "2x15, 2.54 mm pitch, 25.4 mm row spacing", "Socket or solder headers"),
            ("U2", 1, "SlimeVR MuMo V1.1 ICM-45686 + QMC6309", "18 x 13 mm, 7+6 castellated/header pins", "Fit both rows for SPI"),
            ("U3", 1, "Protected TP4056 USB-C module", "Approx. 26 x 17 mm; IN+/IN-/B+/B-/OUT+/OUT-", "Protection version required"),
            ("D1,D2", 2, "1N5817 Schottky diode", "DO-41 axial, 10.16 mm pitch", "Band/cathode toward K"),
            ("R1", 1, "180 kOhm 1/4 W", "Axial, 10.16 mm pitch", "Battery sense"),
            ("R2", 1, "220 kOhm 1/4 W", "Axial, 10.16 mm pitch", "Battery divider upper"),
            ("R3", 1, "100 kOhm 1/4 W", "Axial, 10.16 mm pitch", "Battery divider lower"),
            ("SW1", 1, "SK12D07/SK12D07VG high-3mm", "SPDT, 2.54 mm pin pitch", "Center common"),
            ("J3", 1, "503759 3.7 V protected LiPo", "Bare red/black leads to large pads", "Verify polarity before soldering"),
            ("Headers", 1, "2.54 mm male/female header assortment", "For ESP32, MuMo, TP4056", "Trim flush under PCB"),
        ]
        w.writerows(rows)
    with (PCB / "pinout.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Function", "ESP32 pin", "MuMo pin", "Notes"])
        w.writerows([
            ("SPI SCLK / I2C SCL", "GPIO18", "J1.4 SCL", "24 MHz SPI preferred; same wire can be I2C SCL"),
            ("SPI MOSI / I2C SDA", "GPIO23", "J1.5 SDA", "Same wire can be I2C SDA"),
            ("SPI MISO", "GPIO19", "J1.7 SDO", "Used in SPI mode"),
            ("SPI CS", "GPIO5", "J1.6 CS", "Used in SPI mode"),
            ("Interrupt", "GPIO17", "J2.3 INT1", "Custom firmware pin"),
            ("Battery ADC", "GPIO36 / VP", "-", "180k + 220k over 100k divider"),
            ("Module power", "3V3", "J1.2 3V3", "Do not power MuMo from 5V"),
            ("Ground", "GND", "J1.3 GND", "Common ground"),
        ])
    with (PCB / "CPL.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        w.writerow(["N/A", "", "", "", "Through-hole/module assembly only"])


def schematic_drawing():
    SCHEMATIC.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(16, 10))
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 10)
    ax.axis("off")
    ax.set_facecolor("#f8fafc")

    def block(x, y, w, h, title, subtitle, color="#6d28d9"):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.08,rounding_size=0.12", ec=color, fc="#ffffff", lw=2))
        ax.text(x + w / 2, y + h - 0.28, title, ha="center", va="top", fontsize=13, weight="bold", color=color)
        ax.text(x + w / 2, y + 0.22, subtitle, ha="center", va="bottom", fontsize=8, color="#475569")

    def wire(points, label="", color="#0f766e", lw=2):
        xs, ys = zip(*points)
        ax.plot(xs, ys, color=color, lw=lw)
        if label:
            mid = points[len(points) // 2]
            ax.text(mid[0], mid[1] + 0.12, label, fontsize=8, color=color, ha="center", bbox=dict(fc="white", ec="none", pad=0.3))

    def vertical_resistor(x, y_top, y_bottom, label):
        ys = np.linspace(y_top, y_bottom, 9)
        xs = np.array([x, x - 0.10, x + 0.10, x - 0.10, x + 0.10, x - 0.10, x + 0.10, x - 0.10, x])
        ax.plot(xs, ys, color="#b45309", lw=2)
        ax.text(x + 0.22, (y_top + y_bottom) / 2, label, fontsize=8, color="#92400e", va="center")

    block(0.7, 5.6, 4.1, 3.0, "U1 - ELEGOO ESP32", "ESP-WROOM-32 USB-C, 30 pin")
    block(6.1, 5.6, 4.0, 3.0, "U2 - MuMo V1.1", "3V3 J1.2 - GND J1.3")
    block(11.6, 5.6, 3.5, 3.0, "Battery sense", "")
    for i, (name, y) in enumerate([("GPIO18 SCLK", 8.05), ("GPIO23 MOSI", 7.55), ("GPIO19 MISO", 7.05), ("GPIO5 CS", 6.55), ("GPIO17 INT1", 6.05)]):
        wire([(4.8, y), (6.1, y)], name.split()[1])
    ax.text(8.1, 5.25, "SPI preferred; bridge MuMo JP1-JP4 for I2C", ha="center", fontsize=8, color="#7c3aed")

    block(0.7, 1.0, 3.0, 2.6, "U3 - TP4056", "USB-C, protection version")
    block(4.5, 1.0, 2.0, 2.6, "D1 / D2", "1N5817 diode OR")
    block(7.3, 1.0, 2.0, 2.6, "SW1", "center common")
    block(10.2, 1.0, 2.0, 2.6, "ESP VIN", "switched SYS_5V")
    block(13.0, 1.0, 2.0, 2.6, "J3 LiPo", "3.7 V 1S only")
    wire([(3.7, 3.0), (4.5, 3.0)], "IN+ -> D1")
    wire([(3.7, 2.2), (4.1, 2.2), (4.1, 2.4), (4.5, 2.4)], "OUT+ -> D2")
    wire([(6.5, 2.7), (7.3, 2.7)], "PWR_OR")
    wire([(9.3, 2.7), (10.2, 2.7)], "SYS_5V")
    wire([(3.7, 1.55), (13.0, 1.55)], "B+/B- charge path", "#b45309")
    # Explicit divider: BAT+ -> 180k -> 220k -> GPIO36 -> 100k -> GND.
    wire([(14.0, 3.6), (15.45, 3.6), (15.45, 7.95), (13.2, 7.95)], "BAT+", "#b45309")
    vertical_resistor(13.2, 7.85, 7.42, "R1 180k")
    ax.plot([13.2, 13.2], [7.42, 7.23], color="#b45309", lw=2)
    vertical_resistor(13.2, 7.23, 6.80, "R2 220k")
    ax.plot([13.2, 13.2], [6.80, 6.62], color="#b45309", lw=2)
    ax.scatter([13.2], [6.62], s=20, color="#b45309")
    ax.plot([13.2, 14.4], [6.62, 6.62], color="#b45309", lw=2)
    ax.text(14.35, 6.76, "GPIO36 / VP", fontsize=8, color="#92400e", ha="right")
    vertical_resistor(13.2, 6.48, 6.02, "R3 100k")
    ax.plot([13.2, 13.2], [6.02, 5.84], color="#334155", lw=2)
    ax.plot([12.95, 13.45], [5.84, 5.84], color="#334155", lw=2)
    ax.plot([13.02, 13.38], [5.76, 5.76], color="#334155", lw=2)
    ax.plot([13.10, 13.30], [5.68, 5.68], color="#334155", lw=2)
    ax.text(8.0, 9.45, "RaptorVR 1.0 - ESP32 + MuMo Carrier", ha="center", fontsize=22, weight="bold", color="#4c1d95")
    ax.text(8.0, 9.05, "Functional schematic and assembly map - revision 0.11 prototype", ha="center", fontsize=10, color="#475569")
    ax.text(8.0, 0.35, "WARNING: verify every connection with a multimeter before attaching a LiPo. Never reverse BAT+ and BAT-.", ha="center", fontsize=9, color="#b91c1c", weight="bold")
    fig.tight_layout()
    fig.savefig(SCHEMATIC / "RaptorVR_1_0_schematic.pdf", bbox_inches="tight")
    fig.savefig(SCHEMATIC / "RaptorVR_1_0_schematic.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def pcb_preview(components, pads, npth, tracks, vias):
    PREVIEWS.mkdir(parents=True, exist_ok=True)
    for layer_name, filename in [("F.Cu", "pcb_top.png"), ("B.Cu", "pcb_bottom.png")]:
        fig, ax = plt.subplots(figsize=(13, 8))
        ax.set_aspect("equal")
        ax.set_xlim(-2, BOARD_W + 2)
        ax.set_ylim(-2, BOARD_H + 2)
        ax.axis("off")
        poly = rounded_rect_polygon(BOARD_W, BOARD_H, BOARD_R)
        px, py = poly.exterior.xy
        ax.fill(px, py, color="#5b21b6", ec="#2e1065", lw=2)
        for t in tracks:
            if t.layer == layer_name:
                xs, ys = zip(*t.points)
                ax.plot(xs, ys, color="#d97706", lw=max(1.5, t.width * 4), alpha=0.9)
        for p in pads:
            ax.add_patch(Circle((p.x, p.y), p.diameter / 2, fc="#f59e0b", ec="#fde68a", lw=0.7))
            ax.add_patch(Circle((p.x, p.y), p.drill / 2, fc="#111827", ec="none"))
        for v in vias:
            ax.add_patch(Circle((v.x, v.y), v.diameter / 2, fc="#f59e0b", ec="#fde68a", lw=0.5))
            ax.add_patch(Circle((v.x, v.y), v.drill / 2, fc="#111827", ec="none"))
        for p in npth:
            ax.add_patch(Circle((p.x, p.y), p.drill / 2, fc="#111827", ec="white", lw=0.6))
        for c in components:
            x1, y1, x2, y2 = c.outline
            ax.add_patch(Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False, ec="white", lw=0.8))
            ax.text((x1 + x2) / 2, y2 + 0.55, c.ref, ha="center", color="white", fontsize=7, weight="bold")
        ax.text(BOARD_W / 2, BOARD_H - 1.7, "RAPTORVR 1.0", ha="center", va="top", color="white", fontsize=13, weight="bold")
        ax.set_title(f"RaptorVR 1.0 - {layer_name} preview", fontsize=15, weight="bold")
        fig.savefig(PREVIEWS / filename, dpi=180, bbox_inches="tight", facecolor="#f8fafc")
        plt.close(fig)


def rounded_box_mesh(width, depth, height, radius, z0=0.0):
    poly = rounded_rect_polygon(width, depth, radius)
    mesh = trimesh.creation.extrude_polygon(poly, height)
    mesh.apply_translation([0, 0, z0])
    return mesh


def export_mesh(mesh: trimesh.Trimesh, basename: str):
    mesh.remove_unreferenced_vertices()
    mesh.fix_normals()
    (PRINTABLES / f"{basename}.stl").write_bytes(trimesh.exchange.stl.export_stl(mesh))
    try:
        (PRINTABLES / f"{basename}.3mf").write_bytes(trimesh.exchange.export.export_mesh(mesh, file_type="3mf"))
    except Exception:
        pass


def iter_polygons(geometry):
    """Yield printable polygon islands from Polygon/MultiPolygon geometry."""
    if geometry.is_empty:
        return
    if geometry.geom_type == "Polygon":
        if geometry.area >= 0.05:
            yield geometry
        return
    for child in geometry.geoms:
        yield from iter_polygons(child)


def centered_logo_geometry():
    """Trace the complete supplied logo without substituting its font/layout."""
    rgb = np.asarray(Image.open(io.BytesIO(LOGO_PNG_BYTES)).convert("RGB"), dtype=np.int16)
    # Both the black raptor mark and blue-green lettering are much darker than
    # the off-white background. Trace them together so the exact supplied font,
    # spacing, relative scale, and two-line RAPTOR/VR arrangement are retained.
    artwork_mask = rgb.mean(axis=2) < 185
    sampled = np.flipud(artwork_mask[::2, ::2]).astype(float)
    fig, ax = plt.subplots(figsize=(2, 2))
    contours = ax.contour(sampled, levels=[0.5]).allsegs[0]
    plt.close(fig)
    artwork = None
    for vertices in contours:
        if len(vertices) < 4:
            continue
        candidate = Polygon(vertices).buffer(0)
        if candidate.area <= 1.0:
            continue
        artwork = candidate if artwork is None else artwork.symmetric_difference(candidate)
    if artwork is None or artwork.is_empty:
        raise RuntimeError("Could not trace the supplied RaptorVR logo artwork")
    artwork = artwork.buffer(0).simplify(0.22, preserve_topology=True)
    min_x, min_y, max_x, max_y = artwork.bounds
    artwork_scale = 54.0 / (max_x - min_x)
    artwork = translate_geometry(artwork, xoff=-min_x, yoff=-min_y)
    artwork = scale_geometry(artwork, xfact=artwork_scale, yfact=artwork_scale, origin=(0, 0))
    min_x, min_y, max_x, max_y = artwork.bounds
    artwork = translate_geometry(
        artwork,
        xoff=46.0 - (min_x + max_x) / 2,
        yoff=30.0 - (min_y + max_y) / 2,
    )
    # Keep the existing two-geometry interface; all source artwork is carried
    # in the first geometry and the second is intentionally empty.
    return artwork, Polygon()


def logo_meshes(mark, text_geometry):
    meshes = []
    for polygon in list(iter_polygons(mark)) + list(iter_polygons(text_geometry)):
        mesh = trimesh.creation.extrude_polygon(polygon, 0.84)
        mesh.apply_translation([0, 0, 1.96])
        meshes.append(mesh)
    return meshes


def lid_logo_preview(width, depth, screw_centers, mark, text_geometry):
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.add_patch(FancyBboxPatch(
        (0, 0), width, depth,
        boxstyle="round,pad=0,rounding_size=6",
        facecolor="#e5e7eb", edgecolor="#111827", linewidth=2.0,
    ))
    for sx, sy in screw_centers:
        ax.add_patch(Circle((sx, sy), 4.5, facecolor="#e5e7eb", edgecolor="#111827", linewidth=2.0))
        ax.add_patch(Circle((sx, sy), 1.7, facecolor="white", edgecolor="#111827", linewidth=1.1))
    for geometry in (mark, text_geometry):
        for polygon in iter_polygons(geometry):
            x, y = polygon.exterior.xy
            ax.fill(x, y, color="#111827")
            for ring in polygon.interiors:
                hx, hy = ring.xy
                ax.fill(hx, hy, color="#e5e7eb")
    ax.set_xlim(-7, width + 7)
    ax.set_ylim(-7, depth + 7)
    ax.set_aspect("equal")
    ax.set_axis_off()
    ax.set_title("RaptorVR 1.0 screw lid - supplied artwork preview", fontsize=17, weight="bold")
    fig.savefig(PREVIEWS / "lid_logo_top.png", dpi=180, bbox_inches="tight", facecolor="#f8fafc")
    plt.close(fig)


def export_chassis_viewers(case, tray, lid, case_height):
    """Export accurate colored GLB scenes for orbit/zoom web viewing."""
    def colored_copy(mesh, rgba):
        copy = mesh.copy()
        copy.visual.face_colors = np.tile(np.asarray(rgba, dtype=np.uint8), (len(copy.faces), 1))
        return copy

    parts = {
        "case": colored_copy(case, [124, 58, 237, 255]),
        "separator": colored_copy(tray, [148, 163, 184, 255]),
        "logo_lid": colored_copy(lid, [34, 197, 94, 255]),
    }
    assembled = trimesh.Scene()
    assembled.add_geometry(parts["case"], geom_name="case", node_name="case")
    assembled.add_geometry(
        parts["separator"], geom_name="separator", node_name="separator",
        transform=trimesh.transformations.translation_matrix([0, 0, 14.6]),
    )
    assembled.add_geometry(
        parts["logo_lid"], geom_name="logo_lid", node_name="logo_lid",
        transform=trimesh.transformations.translation_matrix([0, 0, case_height]),
    )
    (PRINTABLES / "RaptorVR_1_0_chassis_assembled.glb").write_bytes(assembled.export(file_type="glb"))

    exploded = trimesh.Scene()
    exploded.add_geometry(parts["case"], geom_name="case", node_name="case")
    exploded.add_geometry(
        parts["separator"], geom_name="separator", node_name="separator",
        transform=trimesh.transformations.translation_matrix([0, 0, 39.0]),
    )
    exploded.add_geometry(
        parts["logo_lid"], geom_name="logo_lid", node_name="logo_lid",
        transform=trimesh.transformations.translation_matrix([0, 0, 64.0]),
    )
    (PRINTABLES / "RaptorVR_1_0_chassis_exploded.glb").write_bytes(exploded.export(file_type="glb"))


def enclosure_models():
    PRINTABLES.mkdir(parents=True, exist_ok=True)
    # Overall body is intentionally larger than meowCarrier to fit the 52 mm
    # ESP32 board and the user's 11 x 30.5 x 52 mm, 1500 mAh LiPo.
    W, D, H = 92.0, 60.0, 29.0
    outer = rounded_box_mesh(W, D, H, 6.0)
    cavity = rounded_box_mesh(88.0, 56.0, H - 1.5, 4.2, z0=2.0)
    cavity.apply_translation([2.0, 2.0, 0])
    case = trimesh.boolean.difference([outer, cavity], engine="manifold")

    # Four external screw bosses keep the fasteners clear of the PCB, battery,
    # USB-C openings, and strap rails. A 2.6 mm blind pilot is intended for
    # M3 x 10 mm thread-forming/self-tapping screws in PLA+ or PETG.
    screw_centers = [(8.0, -1.0), (84.0, -1.0), (8.0, 61.0), (84.0, 61.0)]
    screw_bosses = []
    for sx, sy in screw_centers:
        boss = trimesh.creation.cylinder(radius=4.5, height=H, sections=48)
        boss.apply_translation([sx, sy, H / 2])
        screw_bosses.append(boss)
    case = trimesh.boolean.union([case] + screw_bosses, engine="manifold")

    # With a 2 mm lid, an M3 x 10 mm screw gets about 8 mm of engagement.
    # The blind pilot stops well above the internal electronics.
    screw_pilots = []
    for sx, sy in screw_centers:
        pilot = trimesh.creation.cylinder(radius=1.30, height=11.0, sections=40)
        pilot.apply_translation([sx, sy, H - 4.5])
        screw_pilots.append(pilot)
    case = trimesh.boolean.difference([case] + screw_pilots, engine="manifold")

    # Battery pocket ribs and separator-tray ledges.
    ribs = []
    for x, y, sx, sy in [(14, 9, 2, 42), (76, 9, 2, 42), (14, 9, 64, 2), (14, 49, 64, 2)]:
        b = trimesh.creation.box([sx, sy, 4.0])
        b.apply_translation([x + sx / 2, y + sy / 2, 4.0])
        ribs.append(b)
    for x, y, sx, sy in [(2, 2, 3, 56), (87, 2, 3, 56), (5, 2, 82, 3), (5, 55, 82, 3)]:
        b = trimesh.creation.box([sx, sy, 1.2])
        b.apply_translation([x + sx / 2, y + sy / 2, 14.0])
        ribs.append(b)
    case = trimesh.boolean.union([case] + ribs, engine="manifold")

    # Openings: ESP32 USB-C at front, TP4056 USB-C and switch at right.
    esp_cut = trimesh.creation.box([16.0, 6.0, 8.0]); esp_cut.apply_translation([20.0, 1.0, 20.0])
    charge_cut = trimesh.creation.box([6.0, 14.0, 8.0]); charge_cut.apply_translation([91.0, 44.0, 20.0])
    switch_cut = trimesh.creation.box([6.0, 12.0, 6.0]); switch_cut.apply_translation([91.0, 21.0, 19.5])
    case = trimesh.boolean.difference([case, esp_cut, charge_cut, switch_cut], engine="manifold")

    # Meow-inspired 50 mm strap rails on the underside.
    rails = []
    for x in (8.0, 80.0):
        rail = trimesh.creation.box([4.0, 54.0, 4.0]); rail.apply_translation([x + 2.0, 30.0, -1.0])
        slot = trimesh.creation.box([6.0, 50.5, 2.2]); slot.apply_translation([x + 2.0, 30.0, -1.0])
        rails.append(trimesh.boolean.difference([rail, slot], engine="manifold"))
    case = trimesh.boolean.union([case] + rails, engine="manifold")
    export_mesh(case, "RaptorVR_1_0_case_50mm_strap")

    # Full electrical separator tray; battery wires pass through the covered slot.
    tray = rounded_box_mesh(86.0, 54.0, 1.4, 3.5)
    tray.apply_translation([3.0, 3.0, 0])
    wire_slot = trimesh.creation.box([7.0, 3.5, 3.0]); wire_slot.apply_translation([75.0, 48.0, 1.0])
    tray = trimesh.boolean.difference([tray, wire_slot], engine="manifold")
    for hx, hy in [(6.81, 6.81), (85.55, 6.81), (6.81, 52.53), (56.34, 52.53)]:
        post = trimesh.creation.cylinder(radius=2.4, height=3.0, sections=32)
        post.apply_translation([hx, hy, 1.4])
        tray = trimesh.boolean.union([tray, post], engine="manifold")
        hole = trimesh.creation.cylinder(radius=1.4, height=6.0, sections=32)
        hole.apply_translation([hx, hy, 0])
        tray = trimesh.boolean.difference([tray, hole], engine="manifold")
    export_mesh(tray, "RaptorVR_1_0_battery_separator_tray")

    # Screw-on lid. The plug has 0.20 mm clearance per side in the 88 x 56 mm cavity;
    # it aligns the lid while four M3 screws provide positive retention.
    lid_top = rounded_box_mesh(W, D, 2.0, 6.0)
    plug_outer = rounded_box_mesh(87.6, 55.6, 3.2, 4.0)
    plug_outer.apply_translation([2.2, 2.2, -3.2])
    plug_inner = rounded_box_mesh(84.4, 52.4, 3.4, 2.8)
    plug_inner.apply_translation([3.8, 3.8, -3.3])
    plug_ring = trimesh.boolean.difference([plug_outer, plug_inner], engine="manifold")
    lid = trimesh.boolean.union([lid_top, plug_ring], engine="manifold")
    lid_lugs = []
    for sx, sy in screw_centers:
        lug = trimesh.creation.cylinder(radius=4.5, height=2.0, sections=48)
        lug.apply_translation([sx, sy, 1.0])
        lid_lugs.append(lug)
    lid = trimesh.boolean.union([lid] + lid_lugs, engine="manifold")
    clearance_holes = []
    for sx, sy in screw_centers:
        hole = trimesh.creation.cylinder(radius=1.70, height=6.0, sections=40)
        hole.apply_translation([sx, sy, 0.0])
        clearance_holes.append(hole)
    lid = trimesh.boolean.difference([lid] + clearance_holes, engine="manifold")
    logo_mark, logo_text = centered_logo_geometry()
    lid = trimesh.boolean.union([lid] + logo_meshes(logo_mark, logo_text), engine="manifold")
    export_mesh(lid, "RaptorVR_1_0_screw_lid_M3")
    lid_logo_preview(W, D, screw_centers, logo_mark, logo_text)

    # Simple exploded preview.
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    colors = ["#7c3aed", "#94a3b8", "#22c55e"]
    items = [(case, 0.0, colors[0]), (tray, 32.0, colors[1]), (lid, 52.0, colors[2])]
    for mesh, zoff, color in items:
        verts = mesh.vertices.copy(); verts[:, 2] += zoff
        faces = mesh.faces
        # Downsample faces for a responsive preview while preserving the actual files.
        step = max(1, len(faces) // 9000)
        tri = verts[faces[::step]]
        coll = Poly3DCollection(tri, facecolor=color, edgecolor="#334155", linewidths=0.08, alpha=0.82)
        ax.add_collection3d(coll)
    ax.set_xlim(-5, 100); ax.set_ylim(-5, 65); ax.set_zlim(-5, 85)
    ax.view_init(elev=27, azim=-58)
    ax.set_box_aspect((1.5, 1.0, 1.0)); ax.set_axis_off()
    ax.set_title("RaptorVR 1.0 enclosure - exploded view", fontsize=16, weight="bold")
    fig.savefig(PREVIEWS / "enclosure_exploded.png", dpi=180, bbox_inches="tight", facecolor="#f8fafc")
    plt.close(fig)

    export_chassis_viewers(case, tray, lid, H)

    return [case, tray, lid]


def validate_design(pads, npth, tracks, vias, meshes):
    report = {"board_mm": [BOARD_W, BOARD_H], "checks": {}, "warnings": []}
    outline = rounded_rect_polygon(BOARD_W, BOARD_H, BOARD_R)
    for p in pads + npth:
        if not outline.buffer(-0.25).contains(Point(p.x, p.y).buffer(p.diameter / 2 if p.plated else p.drill / 2)):
            raise RuntimeError(f"Pad/hole too close to edge: {p.key}")
    report["checks"]["all_pads_inside_outline"] = True

    # Exact net-geometry clearance check on both copper layers.
    for layer in ("F.Cu", "B.Cu"):
        by_net = defaultdict(list)
        for p in pads:
            net = p.net if p.net != "NC" else p.key
            by_net[net].append(Point(p.x, p.y).buffer(p.diameter / 2, resolution=16))
        for t in tracks:
            if t.layer == layer:
                by_net[t.net].append(LineString(t.points).buffer(t.width / 2, cap_style=1, join_style=1))
        for v in vias:
            by_net[v.net].append(Point(v.x, v.y).buffer(v.diameter / 2, resolution=16))
        merged = {n: unary_union(gs) for n, gs in by_net.items()}
        names = list(merged)
        violations = []
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                d = merged[a].distance(merged[b])
                if d + 1e-6 < CLEARANCE:
                    violations.append((a, b, round(d, 4)))
        if violations:
            raise RuntimeError(f"Copper clearance failures on {layer}: {violations[:20]}")
        report["checks"][f"{layer}_clearance_{CLEARANCE}_mm"] = True

    # Connectivity: every routed net must form one connected copper graph across layers/vias/pads.
    report["checks"]["routed_net_count"] = len({t.net for t in tracks})
    report["checks"]["via_count"] = len(vias)
    report["checks"]["track_segment_count"] = sum(max(0, len(t.points) - 1) for t in tracks)

    for i, mesh in enumerate(meshes):
        if not mesh.is_watertight:
            raise RuntimeError(f"Mesh {i} is not watertight")
        if mesh.volume <= 0:
            raise RuntimeError(f"Mesh {i} has invalid volume")
    report["checks"]["all_stl_meshes_watertight"] = True
    report["checks"]["lid_xy_clearance_mm"] = 0.20
    report["checks"]["lid_fasteners"] = {
        "quantity": 4,
        "recommended": "M3 x 10 mm thread-forming/self-tapping pan-head",
        "lid_clearance_hole_mm": 3.4,
        "case_blind_pilot_mm": 2.6,
        "minimum_thread_engagement_mm": 8.0,
    }
    report["checks"]["lid_logo"] = {
        "style": "raised",
        "relief_mm": 0.8,
        "source_artwork_width_mm": 54.0,
        "font_geometry": "traced directly from supplied image; no substitute font",
        "layout": "original supplied mark, RAPTOR line, and VR line preserved proportionally",
        "uniform_scaling": True,
        "minimum_recommended_nozzle_mm": 0.4,
    }
    report["checks"]["interactive_3d_models"] = [
        "RaptorVR_1_0_chassis_assembled.glb",
        "RaptorVR_1_0_chassis_exploded.glb",
    ]
    report["checks"]["battery_pocket_mm"] = [64.0, 42.0, 12.0]
    report["checks"]["confirmed_battery_mm"] = [52.0, 30.5, 11.0]
    report["warnings"].extend([
        "Prototype hardware: electrical operation has not been bench-tested on a physical PCB.",
        "Confirm the exact TP4056 module pad order and size before ordering; clone modules vary.",
        "Measure your ELEGOO board and battery before printing; vendor dimensions can vary by batch.",
    ])
    (OUT / "VALIDATION.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def write_readme():
    readme = r'''# RaptorVR 1.0

RaptorVR 1.0 is a **prototype** SlimeVR carrier and enclosure for the **ELEGOO ESP-WROOM-32 USB-C 30-pin development board** and the **SlimeVR MuMo V1.1 ICM-45686 + QMC6309 breakout**. Its rounded outline, clear assembly zones, paired 1N5817 charge diodes, through-hole construction, and 50 mm strap chassis are inspired by the meowCarrier approach while using a larger ESP32-specific layout.

![PCB preview](previews/pcb_top.png)

![Enclosure exploded view](previews/enclosure_exploded.png)

![Raised logo lid preview](previews/lid_logo_top.png)

## Important status

This package passed automated geometry, copper-clearance, Gerber-parsing, and watertight-mesh checks. It has **not** been fabricated or electrically bench-tested. Treat revision 0.11 as a prototype and inspect it in a Gerber viewer before ordering.

## Supported parts

- ELEGOO ESP-WROOM-32 USB-C, 30 pins, two 15-pin rows on 2.54 mm pitch and 25.4 mm row spacing.
- MuMo V1.1, 18 x 13 mm, 1.2 mm thick, with both the seven-pin `J1 TOP_PINS` edge and six-pin `J2 BOTTOM_PINS` edge fitted.
- Protected TP4056 USB-C single-cell charger module, nominally 26 x 17 mm, with `IN+`, `IN-`, `B+`, `B-`, `OUT+`, and `OUT-` pads.
- Two axial 1N5817 Schottky diodes.
- 180 kOhm, 220 kOhm, and 100 kOhm 1/4 W resistors. The three-resistor divider matches the current SlimeVR `BOARD_WROOM32` defaults and feeds GPIO36/VP.
- SK12D07/SK12D07VG high-3mm SPDT slide switch.
- One protected 3.7 V, one-cell LiPo up to approximately 64 x 42 x 12 mm. The confirmed 1500 mAh battery measures 52 x 30.5 x 11 mm and fits with clearance.

## JLCPCB files

Upload [`hardware/pcb/RaptorVR_1_0_JLCPCB_Gerbers.zip`](hardware/pcb/RaptorVR_1_0_JLCPCB_Gerbers.zip) directly to JLCPCB. Recommended prototype settings:

- 2 layers, 86.36 x 53.34 mm
- 1.0 mm FR-4
- Purple solder mask for the shown style, or green for the quickest fabrication
- Lead-free HASL
- 1 oz copper
- Remove order number: specify the bottom `JLCJLCJLCJLC` location if the ordering interface supports it

The design is through-hole/module assembled. `BOM.csv` is a purchasing list; `CPL.csv` intentionally contains no SMT placement data.

## MuMo and firmware wiring

The PCB is SPI-first:

| Signal | ESP32 | MuMo |
| --- | --- | --- |
| SCLK | GPIO18 | J1.4 SCL |
| MOSI | GPIO23 | J1.5 SDA |
| MISO | GPIO19 | J1.7 SDO |
| CS | GPIO5 | J1.6 CS |
| INT1 | GPIO17 | J2.3 INT1 |
| Power | 3V3 | J1.2 3V3 |
| Ground | GND | J1.3 GND |

The same SCL/SDA wires can be used for I2C if the MuMo jumpers are configured according to its official instructions. For SPI, use a custom SlimeVR build with the pins above and ICM-45686 selected. Do not assume a generic prebuilt WROOM32 image has this exact custom SPI map.

Battery monitoring uses GPIO36/VP and these firmware values, in kOhm:

```cpp
#define PIN_BATTERY_LEVEL 36
#define BATTERY_SHIELD_RESISTANCE 180
#define BATTERY_SHIELD_R2 220
#define BATTERY_SHIELD_R1 100
```

## Enclosure

Creality Print can import the STL files in `hardware/enclosure/`:

- `RaptorVR_1_0_case_50mm_strap`: rounded chassis with ESP32 USB-C, charger USB-C, switch openings, and four reinforced screw bosses.
- `RaptorVR_1_0_battery_separator_tray`: full insulating barrier between the LiPo and PCB, with PCB standoffs and a small wire slot.
- `RaptorVR_1_0_screw_lid_M3`: positively retained lid with four 3.4 mm M3 clearance holes, a 0.20 mm-per-side alignment plug, and the centered raised RaptorVR logo.
- `RaptorVR_1_0_chassis_assembled.glb` and `RaptorVR_1_0_chassis_exploded.glb`: accurate colored models for interactive orbit and zoom viewing.

The complete supplied logo image is traced directly into the lid—raptor mark, original lettering shapes, spacing, and the original `RAPTOR`/`VR` two-line arrangement. No substitute font is used. The artwork is scaled uniformly to 54 mm wide and embossed 0.8 mm above the lid for clear resin and FDM visibility. For a contrasting FDM logo, add a filament change for the final four 0.20 mm layers; a single-color print also works. For resin, angle the lid approximately 20-30 degrees and place supports on the inside face so the raised logo remains clean.

Use four **M3 x 10 mm thread-forming/self-tapping pan-head screws for plastic**. The case has 2.6 mm blind pilot holes, so the screw tips cannot reach the PCB or battery. Tighten only until the lid is seated; overtightening can strip printed threads. Do not use screws longer than 10 mm unless you first verify the remaining boss depth.

Starting Creality settings: 0.4 mm nozzle, 0.20 mm layers, four walls, five top/bottom layers, 35% gyroid infill, PLA+ or PETG. Print the case upright, the separator flat, and the lid with its large outer face on the build plate. If your printer runs tight, scale only the lid X/Y to 100.2% or lightly sand the alignment plug.

## Assembly safety

1. Do not attach the LiPo until every other part is soldered and inspected.
2. Verify `BAT+` and `BAT-` with a multimeter. Reversed LiPo polarity can cause fire.
3. Use only a protected one-cell LiPo and a TP4056 board that includes battery protection.
4. Confirm the TP4056 pad order against its seller diagram; visually similar boards can differ.
5. Check for shorts from `SYS_5V` to ground and from `3V3` to ground before inserting the ESP32 or MuMo.
6. The striped end of each 1N5817 goes to the PCB pad marked `K`.
7. Use USB-A-to-USB-C for inexpensive TP4056 modules unless the module explicitly supports USB-C-to-C charging.
8. Never charge an unattended or damaged LiPo, and do not wear the tracker while charging.

## Source and regeneration

- Editable PCB layout: `hardware/pcb/source/RaptorVR_1_0.kicad_pcb`
- Dimension-controlled generator: `hardware/pcb/source/generate_project.py`
- Supplied logo reference: `hardware/pcb/source/assets/RaptorVR_logo_reference.png`
- Human-readable schematic: `hardware/pcb/schematic/RaptorVR_1_0_schematic.pdf`
- Validation report: `VALIDATION.json`

Regenerate with Python 3.12 plus `numpy`, `Pillow`, `shapely`, `trimesh`, `manifold3d`, `matplotlib`, `reportlab`, and `gerbonara`.

## References and attribution

- meowCarrier by Shine Bright (MIT): https://github.com/Shine-Bright-Meow/meowCarrier
- SlimeVR tracker documentation: https://docs.slimevr.dev/diy/tracker-schematics.html
- Official MuMo V1.1 schematic: https://docs.slimevr.dev/files/mumo-schematic-1.1.pdf
- SlimeVR firmware: https://github.com/SlimeVR/SlimeVR-Tracker-ESP

The RaptorVR 1.0 scripts, PCB layout, documentation, and enclosure geometry are released under the MIT License. Third-party modules and reference projects retain their original licenses.
'''
    (OUT / "README.md").write_text(readme, encoding="utf-8")
    license_text = '''MIT License

Copyright (c) 2026 RaptorVR 1.0 contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''
    (OUT / "LICENSE").write_text(license_text, encoding="utf-8")


def main():
    if OUT.exists():
        # Regenerate artifacts without destroying local Git history created
        # after the first generation pass.
        for child in OUT.iterdir():
            if child.name == ".git":
                continue
            if child.is_dir():
                shutil.rmtree(child)
            else:
                child.unlink()
    for d in (GERBERS, SCHEMATIC, SOURCE, PRINTABLES, PREVIEWS):
        d.mkdir(parents=True, exist_ok=True)
    components, pads, npth = build_components()
    router = Router(pads, npth)
    tracks, vias = router.route_all()
    write_gerbers(components, pads, npth, tracks, vias)
    write_kicad_pcb(components, pads, npth, tracks, vias)
    write_bom_and_pinout(components, pads)
    schematic_drawing()
    pcb_preview(components, pads, npth, tracks, vias)
    meshes = enclosure_models()
    report = validate_design(pads, npth, tracks, vias, meshes)
    write_readme()
    (SOURCE / "generate_project.py").write_bytes(SCRIPT_BYTES)
    source_assets = SOURCE / "assets"
    source_assets.mkdir(parents=True, exist_ok=True)
    (source_assets / "RaptorVR_logo_reference.png").write_bytes(LOGO_PNG_BYTES)
    design = {
        "name": "RaptorVR 1.0",
        "revision": "0.11-prototype-interactive-viewer",
        "board_mm": [BOARD_W, BOARD_H, 1.0],
        "components": [{"ref": c.ref, "value": c.value, "outline": c.outline} for c in components],
        "pads": [p.__dict__ for p in pads],
        "npth": [p.__dict__ for p in npth],
        "tracks": [t.__dict__ for t in tracks],
        "vias": [v.__dict__ for v in vias],
    }
    (SOURCE / "design.json").write_text(json.dumps(design, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(OUT), "tracks": len(tracks), "vias": len(vias), "validation": report}, indent=2))


if __name__ == "__main__":
    main()
