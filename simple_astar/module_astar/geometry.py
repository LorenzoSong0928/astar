import gdsfactory as gf

from shapely.geometry import Polygon, LineString
from shapely.affinity import translate, scale, rotate

from config import (
    WAVEGUIDE_WIDTH,
    MINIMUM_SPACING,
    WAVEGUIDE_LAYER,
    BEND_RADIUS,
    BEND_SPAN,
    STRAIGHT_LENGTH,
)


def create_base_bend():
    """
    创建标准 R -> U 的 90° Euler bend。
    Returns
    -------
    bend_component:
        GDSFactory Component

    bend_polygon:
        Shapely Polygon，用于 DRC
    """
    bend_component = gf.components.bend_euler(
        radius=BEND_RADIUS,
        width=WAVEGUIDE_WIDTH,
        angle=90,
        layer=WAVEGUIDE_LAYER,
    )

    polygons = bend_component.get_polygons_points(
        by="tuple"
    )

    wg_polygons = polygons[WAVEGUIDE_LAYER]

    bend_polygon = Polygon(
        wg_polygons[0]
    )

    return bend_component, bend_polygon

def is_geometry_legal(
    waveguide_polygon,
    obstacle_polygons,
    minimum_spacing=MINIMUM_SPACING,
):
    """
    检查真实波导 geometry 是否满足：
    1. 不与障碍物碰撞
    2. minimum spacing
    """

    for i, obstacle in enumerate(obstacle_polygons):

        # 直接碰撞
        if waveguide_polygon.intersects(obstacle):

            return (
                False,
                f"Collision with obstacle {i}"
            )

        # 最小间距
        distance = waveguide_polygon.distance(
            obstacle
        )

        if distance < minimum_spacing:

            return (
                False,
                f"Spacing violation with obstacle {i}: "
                f"{distance:.3f} < {minimum_spacing}"
            )

    return True, "Legal"

def get_bend_geometry(
    base_bend_polygon,
    old_direction,
    new_direction,
):
    """
    根据 old_direction -> new_direction，
    将标准 R -> U bend 旋转/镜像为对应方向。
    """

    direction_pair = (
        old_direction,
        new_direction
    )

    # -------------------------
    # 左转
    # -------------------------

    left_turn_angles = {
        ("R", "U"): 0,
        ("U", "L"): 90,
        ("L", "D"): 180,
        ("D", "R"): 270,
    }

    if direction_pair in left_turn_angles:

        angle = left_turn_angles[
            direction_pair
        ]

        return rotate(
            base_bend_polygon,
            angle=angle,
            origin=(0, 0)
        )

    # -------------------------
    # 右转
    # -------------------------

    right_turn_angles = {
        ("R", "D"): 0,
        ("U", "R"): 90,
        ("L", "U"): 180,
        ("D", "L"): 270,
    }

    if direction_pair in right_turn_angles:

        # R -> U 镜像为 R -> D
        mirrored_bend = scale(
            base_bend_polygon,
            xfact=1,
            yfact=-1,
            origin=(0, 0)
        )

        angle = right_turn_angles[
            direction_pair
        ]

        return rotate(
            mirrored_bend,
            angle=angle,
            origin=(0, 0)
        )

    raise ValueError(
        f"不支持的转向: "
        f"{old_direction} -> {new_direction}"
    )

def place_bend_geometry(
    base_bend_polygon,
    old_direction,
    new_direction,
    position,
):
    """
    生成正确方向的 Euler bend，
    并将入口移动到指定 physical position。
    """

    geometry = get_bend_geometry(
        base_bend_polygon,
        old_direction,
        new_direction
    )

    x, y = position

    geometry = translate(
        geometry,
        xoff=x,
        yoff=y
    )

    return geometry


def get_bend_endpoint(
    position,
    old_direction,
    new_direction,
    bend_span=BEND_SPAN,
):
    """
    给定 bend 入口，计算走完整个 Euler bend 后的出口。
    """
    offsets = {
        ("R", "U"): (+bend_span, +bend_span),
        ("R", "D"): (+bend_span, -bend_span),

        ("U", "L"): (-bend_span, +bend_span),
        ("U", "R"): (+bend_span, +bend_span),

        ("L", "D"): (-bend_span, -bend_span),
        ("L", "U"): (-bend_span, +bend_span),

        ("D", "R"): (+bend_span, -bend_span),
        ("D", "L"): (-bend_span, -bend_span),
    }

    direction_pair = (
        old_direction,
        new_direction
    )

    if direction_pair not in offsets:

        raise ValueError(
            f"不支持的转向: "
            f"{old_direction} -> {new_direction}"
        )

    dx, dy = offsets[direction_pair]

    x, y = position

    return (
        x + dx,
        y + dy
    )

# straight_geometry
def get_straight_geometry(
    position,
    direction,
    length=STRAIGHT_LENGTH,
    width=WAVEGUIDE_WIDTH,
):
    """
    生成一段具有真实宽度的直波导。
    """

    offsets = {
        "R": (+length, 0),
        "L": (-length, 0),
        "U": (0, +length),
        "D": (0, -length),
    }

    x, y = position

    dx, dy = offsets[direction]

    endpoint = (
        x + dx,
        y + dy
    )

    # 波导中心线
    centerline = LineString([
        position,
        endpoint
    ])

    # 中心线扩展成真实二维波导
    geometry = centerline.buffer(
        width / 2,
        cap_style="flat"
    )

    return geometry, endpoint

# ============================================================
# Candidate Evaluation
# ============================================================

def evaluate_bend_candidate(
    base_bend_polygon,
    position,
    old_direction,
    new_direction,
    obstacles,
    minimum_spacing=MINIMUM_SPACING,
    bend_span=BEND_SPAN,
):
    """
    生成 Bend candidate，并进行 Geometry DRC。
    """

    geometry = place_bend_geometry(
        base_bend_polygon,
        old_direction,
        new_direction,
        position
    )

    endpoint = get_bend_endpoint(
        position,
        old_direction,
        new_direction,
        bend_span
    )

    legal, reason = is_geometry_legal(
        geometry,
        obstacles,
        minimum_spacing
    )

    return legal, endpoint, reason, geometry


def evaluate_straight_candidate(
    position,
    direction,
    obstacles,
    minimum_spacing=MINIMUM_SPACING,
    length=STRAIGHT_LENGTH,
    width=WAVEGUIDE_WIDTH,
):
    """
    生成 Straight candidate，并进行 Geometry DRC。
    """

    geometry, endpoint = get_straight_geometry(
        position,
        direction,
        length,
        width
    )

    legal, reason = is_geometry_legal(
        geometry,
        obstacles,
        minimum_spacing
    )

    return legal, endpoint, reason, geometry
