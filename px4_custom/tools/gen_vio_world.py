#!/usr/bin/env python3
"""Generates the `vio_rich` Gazebo world and its textures (feature-rich scene for visual odometry).

    /usr/bin/python3 px4_custom/tools/gen_vio_world.py

Writes:
  px4_custom/models/vio_textures/   (model.config, model.sdf, materials/*.png)
  px4_custom/worlds/vio_rich.sdf

Everything is seeded, so re-running gives the same world. Edit the constants below and rerun.
The original `walls` layout stays at the centre so existing coordinates keep working.
"""
import math
import random
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
TEX_DIR = ROOT / "models" / "vio_textures" / "materials"
WORLD_PATH = ROOT / "worlds" / "vio_rich.sdf"

SEED = 7
SIZE = 1024                 # pixels per texture
WORLD_HALF = 100.0          # world is 200 m x 200 m
TILE = 25.0                 # ground tile edge in metres
PANEL = 5.0                 # wall panel edge in metres (one texture image per panel)

# Things that already exist in `walls` (x0, x1, y0, y1) and must not be built over.
WALLS = {
    "box1": (5, 0, 1, 20, 15),      # name -> (cx, cy, sx, sy, height)
    "box2": (-3, 5, 10, 1, 15),
    "box3": (13, -10, 17, 1, 15),
    "box4": (12, 0, 1, 20, 15),
}
TAGS = [(-4, -4), (16, -6), (-6, 11), (9, 6)]
SPAWN = (0, 0)


# ----------------------------------------------------------------------------- textures
def value_noise(rng, first_cells=4, octaves=6, size=SIZE):
    total = np.zeros((size, size), dtype=np.float32)
    amp = 1.0
    for o in range(octaves):
        cells = first_cells * 2 ** o
        grid = rng.random((cells, cells)).astype(np.float32)
        img = Image.fromarray(grid, "F").resize((size, size), Image.BICUBIC)
        total += amp * np.asarray(img)
        amp *= 0.55
    total -= total.min()
    return total / total.max()


def ramp(values, stops):
    """Map 0..1 values to RGB through [(position, (r, g, b)), ...]."""
    pos = [p for p, _ in stops]
    out = np.stack([np.interp(values, pos, [c[i] for _, c in stops]) for i in range(3)], axis=-1)
    return out.astype(np.float32)


def speckle(rng, rgb, amount):
    noise = (rng.random(rgb.shape[:2]).astype(np.float32) - 0.5) * 2 * amount
    return rgb + noise[..., None]


def to_image(rgb):
    return Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8), "RGB")


def gravel(rng):
    n = value_noise(rng, 8, 6)
    rgb = ramp(n, [(0, (70, 66, 60)), (0.5, (140, 132, 120)), (1, (205, 198, 185))])
    return to_image(speckle(rng, rgb, 40))


def asphalt(rng):
    n = value_noise(rng, 6, 6)
    rgb = speckle(rng, ramp(n, [(0, (35, 35, 38)), (1, (95, 95, 98))]), 30)
    img = to_image(rgb)
    d = ImageDraw.Draw(img)
    for _ in range(14):                                  # cracks
        x, y = rng.integers(0, SIZE, 2)
        pts = [(int(x), int(y))]
        for _ in range(12):
            x += rng.integers(-60, 60)
            y += rng.integers(-60, 60)
            pts.append((int(x), int(y)))
        d.line(pts, fill=(15, 15, 15), width=int(rng.integers(2, 5)))
    for _ in range(3):                                   # paint marks
        x, y = rng.integers(0, SIZE - 200, 2)
        d.rectangle([int(x), int(y), int(x) + int(rng.integers(20, 200)), int(y) + 14], fill=(200, 190, 60))
    return img


def grass(rng):
    n = value_noise(rng, 10, 6)
    rgb = ramp(n, [(0, (35, 70, 28)), (0.5, (70, 120, 45)), (1, (130, 160, 70))])
    return to_image(speckle(rng, rgb, 45))


def dirt(rng):
    n = value_noise(rng, 5, 6)
    img = to_image(speckle(rng, ramp(n, [(0, (85, 60, 40)), (1, (170, 130, 90))]), 25))
    d = ImageDraw.Draw(img)
    for _ in range(500):                                 # pebbles
        x, y = rng.integers(0, SIZE, 2)
        r = int(rng.integers(2, 9))
        g = int(rng.integers(90, 200))
        d.ellipse([int(x), int(y), int(x) + r, int(y) + r], fill=(g, g - 10, g - 25))
    return img


def brick(rng):
    img = Image.new("RGB", (SIZE, SIZE), (175, 170, 160))
    d = ImageDraw.Draw(img)
    bw, bh, mortar = 61, 24, 3
    for row, y in enumerate(range(0, SIZE, bh)):
        offset = (bw // 2) if row % 2 else 0
        for x in range(-bw, SIZE + bw, bw):
            base = int(rng.integers(0, 3))
            color = [(150, 65, 45), (125, 50, 38), (170, 90, 60)][base]
            tint = int(rng.integers(-18, 18))
            c = tuple(max(0, min(255, v + tint)) for v in color)
            d.rectangle([x + offset + mortar, y + mortar, x + offset + bw, y + bh], fill=c)
    arr = np.asarray(img).astype(np.float32)
    arr *= (0.8 + 0.4 * value_noise(rng, 6, 5))[..., None]
    return to_image(speckle(rng, arr, 14))


def concrete(rng):
    n = value_noise(rng, 5, 7)
    img = to_image(speckle(rng, ramp(n, [(0, (95, 95, 92)), (1, (190, 188, 182))]), 22))
    d = ImageDraw.Draw(img)
    for x in range(0, SIZE, 256):                        # panel joints
        d.line([(x, 0), (x, SIZE)], fill=(60, 60, 60), width=3)
    for y in range(0, SIZE, 256):
        d.line([(0, y), (SIZE, y)], fill=(60, 60, 60), width=3)
    for _ in range(60):                                  # holes and stains
        x, y = rng.integers(0, SIZE, 2)
        r = int(rng.integers(3, 25))
        g = int(rng.integers(40, 90))
        d.ellipse([int(x), int(y), int(x) + r, int(y) + r], fill=(g, g, g))
    return img


def colorblocks(rng):
    img = to_image(ramp(value_noise(rng, 4, 4), [(0, (150, 150, 150)), (1, (210, 210, 210))]))
    d = ImageDraw.Draw(img)
    for _ in range(90):
        x, y = rng.integers(0, SIZE - 40, 2)
        w, h = rng.integers(30, 260, 2)
        color = tuple(int(v) for v in rng.integers(20, 255, 3))
        if rng.random() < 0.5:
            d.rectangle([int(x), int(y), int(x + w), int(y + h)], fill=color)
        else:
            d.ellipse([int(x), int(y), int(x + w), int(y + h)], fill=color)
    return img


def tiles(rng):
    img = Image.new("RGB", (SIZE, SIZE))
    d = ImageDraw.Draw(img)
    palette = [(30, 60, 120), (200, 200, 200), (180, 60, 50), (240, 200, 60), (40, 40, 40)]
    step = 32
    for x in range(0, SIZE, step):
        for y in range(0, SIZE, step):
            d.rectangle([x, y, x + step - 2, y + step - 2], fill=palette[int(rng.integers(0, len(palette)))])
    return to_image(speckle(rng, np.asarray(img).astype(np.float32), 10))


def wood(rng):
    img = Image.new("RGB", (SIZE, SIZE))
    d = ImageDraw.Draw(img)
    plank = 128
    grain = value_noise(rng, 3, 5)
    for i, x in enumerate(range(0, SIZE, plank)):
        base = int(rng.integers(110, 170))
        d.rectangle([x, 0, x + plank - 4, SIZE], fill=(base, int(base * 0.7), int(base * 0.42)))
    arr = np.asarray(img).astype(np.float32) * (0.75 + 0.5 * grain)[..., None]
    for _ in range(40):                                  # knots
        x, y = rng.integers(0, SIZE, 2)
        r = int(rng.integers(6, 20))
        arr[max(0, y - r):y + r, max(0, x - r):x + r] *= 0.6
    return to_image(arr)


def poster_checker(rng):
    img = Image.new("RGB", (SIZE, SIZE), (255, 255, 255))
    d = ImageDraw.Draw(img)
    n = 10
    cell = (SIZE - 128) // n
    for i in range(n):
        for j in range(n):
            if (i + j) % 2 == 0:
                x, y = 64 + i * cell, 64 + j * cell
                d.rectangle([x, y, x + cell, y + cell], fill=(0, 0, 0))
    return img


def poster_dots(rng):
    img = Image.new("RGB", (SIZE, SIZE), (245, 245, 240))
    d = ImageDraw.Draw(img)
    for _ in range(700):
        x, y = rng.integers(0, SIZE, 2)
        r = int(rng.integers(4, 30))
        color = tuple(int(v) for v in rng.integers(0, 120, 3))
        shape = rng.integers(0, 3)
        box = [int(x), int(y), int(x) + r, int(y) + r]
        if shape == 0:
            d.ellipse(box, fill=color)
        elif shape == 1:
            d.rectangle(box, fill=color)
        else:
            d.polygon([(box[0], box[3]), (box[2], box[3]), ((box[0] + box[2]) // 2, box[1])], fill=color)
    return img


TEXTURES = {fn.__name__: fn for fn in (gravel, asphalt, grass, dirt, brick, concrete, colorblocks, tiles,
                                       wood, poster_checker, poster_dots)}
GROUND = ["gravel", "asphalt", "grass", "dirt"]
WALL_TEX = ["brick", "concrete", "colorblocks", "tiles"]


def write_textures():
    TEX_DIR.mkdir(parents=True, exist_ok=True)
    for i, (name, fn) in enumerate(TEXTURES.items()):
        fn(np.random.default_rng(SEED + i)).save(TEX_DIR / f"{name}.png", optimize=True)
    model = TEX_DIR.parent
    (model / "model.config").write_text(
        '<?xml version="1.0"?>\n<model>\n  <name>vio_textures</name>\n  <version>1.0</version>\n'
        '  <sdf version="1.9">model.sdf</sdf>\n  <author><name>generated</name><email>none@example.com</email></author>\n'
        '  <description>Generated textures for the vio_rich world (not a physical model).</description>\n</model>\n')
    (model / "model.sdf").write_text(
        '<?xml version="1.0"?>\n<sdf version="1.9">\n  <model name="vio_textures"><static>true</static></model>\n</sdf>\n')


# ----------------------------------------------------------------------------- SDF helpers
def material(tex):
    return (f"<material><pbr><metal><albedo_map>model://vio_textures/materials/{tex}.png</albedo_map>"
            f"<metalness>0.0</metalness><roughness>1.0</roughness></metal></pbr>"
            f"<ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular></material>")


def box_geo(sx, sy, sz):
    return f"<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"


def pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    return f"<pose>{x:.3f} {y:.3f} {z:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>"


def visual(name, geo, tex, p=""):
    return f'<visual name="{name}">{p}{geo}{material(tex)}</visual>'


def collision(name, geo, p=""):
    return f'<collision name="{name}">{p}{geo}</collision>'


def model(name, links, p=""):
    return f'<model name="{name}"><static>true</static>{p}<link name="link">{links}</link></model>'


def simple(name, geo, tex, p):
    return model(name, collision("c", geo) + visual("v", geo, tex), p)


def paneled_wall(name, cx, cy, sx, sy, height, tex):
    """A wall with one collision box and visuals cut into ~PANEL-sized panels, so a texture
    image covers about 5 m and keeps its detail."""
    horizontal = sx >= sy
    length, thick = (sx, sy) if horizontal else (sy, sx)
    nl, nz = max(1, round(length / PANEL)), max(1, round(height / PANEL))
    pl, pz = length / nl, height / nz
    visuals = []
    for i in range(nl):
        for k in range(nz):
            along = -length / 2 + pl * (i + 0.5)
            z = -height / 2 + pz * (k + 0.5)
            size = (pl, thick, pz) if horizontal else (thick, pl, pz)
            off = (along, 0, z) if horizontal else (0, along, z)
            visuals.append(visual(f"v{i}_{k}", box_geo(*size), tex, pose(*off)))
    whole = box_geo(sx, sy, height)
    return model(name, collision("c", whole) + "".join(visuals), pose(cx, cy, height / 2))


# ----------------------------------------------------------------------------- world
def keep_out(x, y, radius):
    if math.hypot(x - SPAWN[0], y - SPAWN[1]) < 4 + radius:
        return True
    if any(math.hypot(x - tx, y - ty) < 2 + radius for tx, ty in TAGS):
        return True
    for cx, cy, sx, sy, _ in WALLS.values():
        if abs(x - cx) < sx / 2 + 3 + radius and abs(y - cy) < sy / 2 + 3 + radius:
            return True
    return False


def build_world():
    rnd = random.Random(SEED)
    parts = []

    # ground: visual-only tiles over an invisible collision plane
    n = int(2 * WORLD_HALF / TILE)
    for i in range(n):
        for j in range(n):
            x = -WORLD_HALF + TILE * (i + 0.5)
            y = -WORLD_HALF + TILE * (j + 0.5)
            tex = rnd.choice(GROUND)
            yaw = rnd.choice([0, math.pi / 2, math.pi, 3 * math.pi / 2])
            parts.append(model(f"ground_{i}_{j}", visual("v", box_geo(TILE, TILE, 0.1), tex),
                               pose(x, y, -0.05, yaw=yaw)))

    # original walls, now textured and panelled
    for (name, (cx, cy, sx, sy, h)), tex in zip(WALLS.items(), WALL_TEX):
        parts.append(paneled_wall(name, cx, cy, sx, sy, h, tex))

    placed = []

    def free(x, y, r):
        return not keep_out(x, y, r) and all(math.hypot(x - px, y - py) > r + pr + 1.0 for px, py, pr in placed)

    def spot(r):
        for _ in range(200):
            x = rnd.uniform(-WORLD_HALF + 6, WORLD_HALF - 6)
            y = rnd.uniform(-WORLD_HALF + 6, WORLD_HALF - 6)
            if free(x, y, r):
                placed.append((x, y, r))
                return x, y
        return None

    def many(count, radius, make):
        for k in range(count):
            s = spot(radius)
            if s:
                parts.append(make(k, *s))

    def building(k, x, y):
        sx, sy, sz = rnd.uniform(3, 7), rnd.uniform(3, 7), rnd.uniform(3, 10)
        return simple(f"building{k}", box_geo(sx, sy, sz), rnd.choice(WALL_TEX),
                      pose(x, y, sz / 2, yaw=rnd.uniform(0, math.pi)))

    def crate(k, x, y):
        s = rnd.uniform(0.8, 1.6)
        top = ""
        if rnd.random() < 0.4:
            top = (collision("c2", box_geo(s * 0.8, s * 0.8, s * 0.8), pose(0.1, 0, s * 0.9)) +
                   visual("v2", box_geo(s * 0.8, s * 0.8, s * 0.8), "wood", pose(0.1, 0, s * 0.9)))
        return model(f"crate{k}", collision("c", box_geo(s, s, s), pose(0, 0, s / 2)) +
                     visual("v", box_geo(s, s, s), "wood", pose(0, 0, s / 2)) + top,
                     pose(x, y, 0, yaw=rnd.uniform(0, math.pi)))

    def barrel(k, x, y):
        geo = f"<geometry><cylinder><radius>0.3</radius><length>0.9</length></cylinder></geometry>"
        return simple(f"barrel{k}", geo, rnd.choice(["colorblocks", "tiles"]), pose(x, y, 0.45))

    def pillar(k, x, y):
        r, h = rnd.uniform(0.3, 0.6), rnd.uniform(3, 10)
        geo = f"<geometry><cylinder><radius>{r:.2f}</radius><length>{h:.2f}</length></cylinder></geometry>"
        return simple(f"pillar{k}", geo, rnd.choice(["concrete", "brick"]), pose(x, y, h / 2))

    def poster(k, x, y):
        tex = "poster_checker" if k % 2 == 0 else "poster_dots"
        geo = box_geo(3.0, 0.08, 2.2)
        return simple(f"poster{k}", geo, tex, pose(x, y, 1.3, yaw=rnd.uniform(0, math.pi)))

    def rock(k, x, y):
        r = rnd.uniform(0.5, 1.5)
        geo = f"<geometry><sphere><radius>{r:.2f}</radius></sphere></geometry>"
        return simple(f"rock{k}", geo, rnd.choice(["gravel", "dirt", "concrete"]), pose(x, y, r * 0.4))

    def ramp(k, x, y):
        geo = box_geo(6.0, 3.0, 0.4)
        return simple(f"ramp{k}", geo, rnd.choice(["asphalt", "wood"]),
                      pose(x, y, 0.3, pitch=0.18, yaw=rnd.uniform(0, math.pi)))

    many(25, 6, building)
    many(40, 1.4, crate)
    many(30, 0.5, barrel)
    many(20, 0.8, pillar)
    many(25, 1.6, poster)
    many(25, 1.6, rock)
    many(10, 3.5, ramp)
    return parts


HEADER = """<?xml version="1.0" encoding="UTF-8"?>
<!-- GENERATED by px4_custom/tools/gen_vio_world.py - edit the script, not this file. -->
<sdf version="1.9">
  <world name="vio_rich">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.6 0.75 0.9 1</background>
      <shadows>true</shadows>
    </scene>
    <!-- collision only: the visible ground is made of textured tiles -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>
        </collision>
      </link>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.3 0.4 -0.8</direction>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg> 8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
    <include><uri>model://arucotag</uri></include>
"""


def main():
    write_textures()
    parts = build_world()
    WORLD_PATH.write_text(HEADER + "\n".join(parts) + "\n  </world>\n</sdf>\n")
    print(f"wrote {len(TEXTURES)} textures to {TEX_DIR}")
    print(f"wrote {WORLD_PATH} with {len(parts)} models")


if __name__ == "__main__":
    main()
