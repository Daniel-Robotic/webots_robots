import math
import json
import random

from controller import Supervisor


def _clamp01(v):
    return max(0.0, min(1.0, float(v)))

def _pick_color(rng, palette):
    if not palette:
        return [_clamp01(rng.random()), _clamp01(rng.random()), _clamp01(rng.random())]
    return [float(c) for c in random.choice(palette)]

def _rand_in_range(rng, lo, hi):
    lo, hi = float(lo), float(hi)
    if hi < lo: lo, hi = hi, lo
    return float(rng.uniform(lo, hi))

def _pick_position(rng, limits):
    x = _rand_in_range(rng, limits["x"][0], limits["x"][1])
    y = _rand_in_range(rng, limits["y"][0], limits["y"][1])
    z = _rand_in_range(rng, limits["z"][0], limits["z"][1])
    return [x, y, z]

def _pick_rpy(rng, limits):
    r = _rand_in_range(rng, limits["r"][0], limits["r"][1])
    p = _rand_in_range(rng, limits["p"][0], limits["p"][1])
    y = _rand_in_range(rng, limits["y"][0], limits["y"][1])
    return [r, p, y]

def _rpy_to_axis_angle(r, p, y):
    """
    RPY (Zyx convention) -> quaternion -> axis-angle
    Webots rotation: [ax, ay, az, angle]
    """
    cr = math.cos(r * 0.5); sr = math.sin(r * 0.5)
    cp = math.cos(p * 0.5); sp = math.sin(p * 0.5)
    cy = math.cos(y * 0.5); sy = math.sin(y * 0.5)

    qw = cy*cp*cr + sy*sp*sr
    qx = cy*cp*sr - sy*sp*cr
    qy = cy*sp*cr + sy*cp*sr
    qz = sy*cp*cr - cy*sp*sr

    angle = 2.0 * math.acos(max(-1.0, min(1.0, qw)))
    s = math.sqrt(1.0 - qw*qw)
    if s < 1e-8:
        return [1.0, 0.0, 0.0, 0.0]
    
    ax = qx / s
    ay = qy / s
    az = qz / s

    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm > 0:
        ax /= norm; ay /= norm; az /= norm
    return [ax, ay, az, angle]

def load_objects_config(path):
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)

    seed = raw.get("seed", None)
    rng = random.Random(seed) if seed is not None else random.Random()

    defaults = raw.get("defaults", {})
    def_pos_limits = defaults.get("position_limits", None)
    def_rpy_limits = defaults.get("rpy_limits", None)
    def_palette    = defaults.get("color_palette", None)

    items = raw.get("objects", [])
    resolved = []

    for it in items:
        name = it["def"]

        color = it.get("color", "random")
        if isinstance(color, str) and color.lower() == "random":
            palette = it.get("color_palette", def_palette) or []
            color = _pick_color(rng, palette)
        else:
            color = [_clamp01(color[0]), _clamp01(color[1]), _clamp01(color[2])]

        start_pos = it.get("start_position", "random")
        if isinstance(start_pos, str) and start_pos.lower() == "random":
            limits = it.get("position_limits", def_pos_limits)
            if not limits:
                raise ValueError(f"Для '{name}' random позиция, но нет position_limits ни в объекте, ни в defaults")
            start_pos = _pick_position(rng, limits)
        else:
            start_pos = [float(start_pos[0]), float(start_pos[1]), float(start_pos[2])]

        # RPY
        start_rpy = it.get("start_rpy", [0.0, 0.0, 0.0])
        if isinstance(start_rpy, str) and start_rpy.lower() == "random":
            rpy_limits = it.get("rpy_limits", def_rpy_limits)
            if not rpy_limits:
                rpy_limits = {
                    "r": [-math.pi, math.pi],
                    "p": [-math.pi/2, math.pi/2],
                    "y": [-math.pi, math.pi]
                }
            start_rpy = _pick_rpy(rng, rpy_limits)
        else:
            start_rpy = [float(start_rpy[0]), float(start_rpy[1]), float(start_rpy[2])]

        resolved.append({
            "def": name,
            "color": [float(color[0]), float(color[1]), float(color[2])],
            "start_position": [float(start_pos[0]), float(start_pos[1]), float(start_pos[2])],
            "start_rpy": [float(start_rpy[0]), float(start_rpy[1]), float(start_rpy[2])]
        })
    return resolved

def apply_objects_config(robot: Supervisor, resolved):
    """
    Применяем translation (XYZ), rotation (из RPY) и baseColor для каждого DEF.
    """
    for it in resolved:
        def_name = it["def"]
        node = robot.getFromDef(def_name)
        if node is None:
            print(f"[WARN] DEF '{def_name}' не найден — пропускаю.")
            continue

        try:
            tr = node.getField("translation")
            tr.setSFVec3f(it["start_position"])
        except Exception as e:
            print(f"[WARN] Не удалось установить translation для '{def_name}': {e}")

        try:
            r, p, y = it["start_rpy"]
            ax, ay, az, ang = _rpy_to_axis_angle(r, p, y)
            rot_field = node.getField("rotation")
            rot_field.setSFRotation([ax, ay, az, ang])
        except Exception as e:
            print(f"[WARN] Не удалось установить rotation (RPY) для '{def_name}': {e}")

        try:
            children = node.getField("children")
            if children is None or children.getCount() == 0:
                print(f"[WARN] '{def_name}': нет children для изменения цвета.")
                continue
            shape = children.getMFNode(0)
            app_field = shape.getField("appearance")
            app_node = app_field.getSFNode() if app_field else None
            base_color_field = app_node.getField("baseColor") if app_node else None
            if base_color_field:
                base_color_field.setSFColor(it["color"])
            else:
                print(f"[WARN] '{def_name}': не найден baseColor.")
        except Exception as e:
            print(f"[WARN] Не удалось установить цвет для '{def_name}': {e}")
