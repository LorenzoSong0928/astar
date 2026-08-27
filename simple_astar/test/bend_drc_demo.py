import gdsfactory as gf
import heapq
import itertools

from shapely.geometry import Polygon, box, LineString
from shapely.affinity import translate, scale, rotate


# ============================================================
# 0. Parameters
# ============================================================

WAVEGUIDE_WIDTH = 0.5
MINIMUM_SPACING = 0.5

BEND_RADIUS = 10
BEND_SPAN = 10

STRAIGHT_LENGTH = 10

WAVEGUIDE_LAYER = (1, 0)


# ============================================================
# 1. Create Base Euler Bend
# ============================================================

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


# ============================================================
# 2. Geometry DRC
# ============================================================

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


# ============================================================
# 3. Bend Geometry
# ============================================================

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

# ============================================================
# 4-1. sharp bend Geometry
# ============================================================
def get_sharp_bend_geometry(
    position,
    old_direction,
    new_direction,
    bend_span=BEND_SPAN,
    width=WAVEGUIDE_WIDTH,
):
    """
    用两段正交直线模拟旧式 Manhattan 尖角路径。
    仅用于和真实 Euler bend 做对照实验。
    """

    offsets = {
        "R": (1, 0),
        "L": (-1, 0),
        "U": (0, 1),
        "D": (0, -1),
    }

    x, y = position

    # 第一段沿 old_direction
    dx1, dy1 = offsets[old_direction]

    corner = (
        x + dx1 * bend_span,
        y + dy1 * bend_span
    )

    # 第二段沿 new_direction
    dx2, dy2 = offsets[new_direction]

    endpoint = (
        corner[0] + dx2 * bend_span,
        corner[1] + dy2 * bend_span
    )

    centerline = LineString([
        position,
        corner,
        endpoint
    ])

    geometry = centerline.buffer(
        width / 2,
        cap_style="flat",
        join_style="mitre"
    )

    return geometry, endpoint

# ============================================================
# 4. Straight Geometry
# ============================================================

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
# 5. Candidate Evaluation
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


# ============================================================
# 6. Curvy-aware Neighbors
# ============================================================

def get_curvy_neighbors(
    position,
    direction,
    base_bend_polygon,
    obstacles,
    bend_length,
    minimum_spacing=MINIMUM_SPACING,
    straight_length=STRAIGHT_LENGTH,
    bend_span=BEND_SPAN,
    width=WAVEGUIDE_WIDTH,
):
    """
    从当前 position + direction 出发：

    1. 尝试 straight
    2. 尝试两个 90° Euler bend
    3. Geometry DRC
    4. 只返回合法 neighbors
    """

    neighbors = []

    turn_directions = {
        "R": ["U", "D"],
        "L": ["U", "D"],
        "U": ["L", "R"],
        "D": ["L", "R"],
    }

    # -------------------------
    # Straight
    # -------------------------

    legal, endpoint, reason, geometry = (
        evaluate_straight_candidate(
            position=position,
            direction=direction,
            obstacles=obstacles,
            minimum_spacing=minimum_spacing,
            length=straight_length,
            width=width,
        )
    )

    if legal:

        neighbors.append({
            "type": "straight",
            "position": endpoint,
            "direction": direction,
            "geometry": geometry,
            "cost": straight_length,
        })

    # -------------------------
    # Euler bends
    # -------------------------

    for new_direction in turn_directions[
        direction
    ]:

        legal, endpoint, reason, geometry = (
            evaluate_bend_candidate(
                base_bend_polygon=
                base_bend_polygon,

                position=position,

                old_direction=direction,

                new_direction=
                new_direction,

                obstacles=obstacles,

                minimum_spacing=
                minimum_spacing,

                bend_span=bend_span,
            )
        )

        if legal:

            neighbors.append({
                "type": "bend",
                "position": endpoint,
                "direction": new_direction,
                "geometry": geometry,
                "cost": bend_length,
            })

    return neighbors


# ============================================================
# 7. Curvy-aware Heuristic
# ============================================================

def curvy_heuristic(
    position,
    goal,
    bend_span=BEND_SPAN,
    bend_length=16.637,
):
    """
    Manhattan distance 修正：

    两段正交 straight
    ↓
    一段 Euler bend

    h =
        Manhattan
        - 被 bend 替代的 straight length
        + Euler bend length
    """

    x, y = position
    gx, gy = goal

    dx = abs(gx - x)
    dy = abs(gy - y)

    manhattan = dx + dy

    n_bends = (
        min(dx, dy)
        / bend_span
    )

    h = (
        manhattan
        - n_bends * 2 * bend_span
        + n_bends * bend_length
    )

    return h

def reconstruct_path(
    came_from,
    transition_from,
    goal_state,
):
    """
    根据 came_from 回溯完整路径。
    """

    states = [goal_state]
    transitions = []

    current = goal_state

    while current in came_from:

        # current 是通过哪种 geometry 到达的
        transitions.append(
            transition_from[current]
        )

        current = came_from[current]

        states.append(current)

    states.reverse()
    transitions.reverse()

    return states, transitions


def curvy_astar(
    start_position,
    start_direction,
    goal,
    base_bend_polygon,
    obstacles,
    bend_length,
    minimum_spacing=MINIMUM_SPACING,
):
    """
    Geometry-aware Curvy A*

    State:
        (x, y, direction)

    Neighbor:
        straight / 90° Euler bend

    每个 candidate 在进入 open_set 前
    都必须通过真实 geometry DRC。
    """

    start_state = (
        start_position[0],
        start_position[1],
        start_direction
    )

    # =====================================
    # A* Data
    # =====================================

    open_set = []

    counter = itertools.count()

    g_score = {
        start_state: 0
    }

    came_from = {}

    # 记录从 parent -> current
    # 使用的是 straight 还是 bend
    transition_from = {}

    # =====================================
    # Start
    # =====================================

    start_h = curvy_heuristic(
        start_position,
        goal,
        bend_length=bend_length
    )

    heapq.heappush(
        open_set,
        (
            start_h,
            next(counter),
            start_state,
            0
        )
    )

    expanded_nodes = 0

    # =====================================
    # Main A* Loop
    # =====================================

    while open_set:

        (
            current_f,
            _,
            current_state,
            pushed_g
        ) = heapq.heappop(open_set)

        # -------------------------
        # 跳过旧的 heap entry
        # -------------------------

        current_best_g = g_score.get(
            current_state,
            float("inf")
        )

        if pushed_g > current_best_g:
            continue

        expanded_nodes += 1

        x, y, current_direction = (
            current_state
        )

        current_position = (x, y)

        # =================================
        # Goal
        # =================================

        if current_position == goal:

            states, transitions = (
                reconstruct_path(
                    came_from,
                    transition_from,
                    current_state
                )
            )

            return {
                "success": True,
                "cost": current_best_g,
                "states": states,
                "transitions": transitions,
                "expanded_nodes": expanded_nodes,
            }

        # =================================
        # Geometry-aware Neighbors
        # =================================

        neighbors = get_curvy_neighbors(
            position=current_position,
            direction=current_direction,
            base_bend_polygon=
            base_bend_polygon,
            obstacles=obstacles,
            bend_length=bend_length,
            minimum_spacing=minimum_spacing,
        )

        # =================================
        # Relaxation
        # =================================

        for neighbor in neighbors:

            nx, ny = neighbor["position"]

            new_direction = (
                neighbor["direction"]
            )

            new_state = (
                nx,
                ny,
                new_direction
            )

            new_g = (
                current_best_g
                + neighbor["cost"]
            )

            old_g = g_score.get(
                new_state,
                float("inf")
            )

            if new_g < old_g:

                # -------------------------
                # 更新最优路径
                # -------------------------

                g_score[new_state] = new_g

                came_from[new_state] = (
                    current_state
                )

                transition_from[new_state] = (
                    neighbor
                )

                # -------------------------
                # h
                # -------------------------

                h = curvy_heuristic(
                    neighbor["position"],
                    goal,
                    bend_length=bend_length
                )

                # -------------------------
                # f = g + h
                # -------------------------

                f = new_g + h

                heapq.heappush(
                    open_set,
                    (
                        f,
                        next(counter),
                        new_state,
                        new_g
                    )
                )

    # =====================================
    # Failed
    # =====================================

    return {
        "success": False,
        "cost": None,
        "states": [],
        "transitions": [],
        "expanded_nodes": expanded_nodes,
    }

# ============================================================
# 8. GDS
# ============================================================
def export_result_gds(
    result,
    obstacles,
    filename="curvy_astar_result.gds"
):
    top = gf.Component("curvy_astar_result")

    # -------------------------
    # Route geometry
    # -------------------------
    for transition in result["transitions"]:

        geometry = transition["geometry"]

        points = list(
            geometry.exterior.coords
        )

        top.add_polygon(
            points,
            layer=(1, 0)
        )

    # -------------------------
    # Obstacles
    # -------------------------
    for obstacle in obstacles:

        points = list(
            obstacle.exterior.coords
        )

        top.add_polygon(
            points,
            layer=(2, 0)
        )

    top.write_gds(filename)

    print(
        "\nGDS exported:",
        filename
    )


# ============================================================
# 9. Demo
# ============================================================

def main():

    # -------------------------
    # Base bend
    # -------------------------

    bend_component, bend_polygon = (
        create_base_bend()
    )

    bend_length = bend_component.info[
        "length"
    ]

    print(
        "Euler bend length:",
        bend_length
    )

    # -------------------------
    # Scene
    # -------------------------

    position = (20, 30)
    direction = "R"

    goal = (40, 10)

    obstacles = [
        box(
            28.0,
            24.5,
            29.0,
            25.5
        )
    ]

    # -------------------------
    # Generate neighbors
    # -------------------------

    neighbors = get_curvy_neighbors(
        position=position,
        direction=direction,
        base_bend_polygon=bend_polygon,
        obstacles=obstacles,
        bend_length=bend_length,
    )

    # -------------------------
    # Print neighbors
    # -------------------------

    print("\n--- Legal Neighbors ---")

    for neighbor in neighbors:

        print(
            neighbor["type"],
            neighbor["position"],
            neighbor["direction"],
            "cost =",
            neighbor["cost"]
        )

# 对照测试
    print("\n--- Sharp vs Euler DRC Test ---")

    position = (20, 30)

    # =====================================
    # 旧：尖锐 Manhattan bend
    # =====================================

    sharp_geometry, sharp_endpoint = (
        get_sharp_bend_geometry(
            position=position,
            old_direction="R",
            new_direction="D",
        )
    )

    sharp_legal, sharp_reason = (
        is_geometry_legal(
            sharp_geometry,
            obstacles,
            minimum_spacing=MINIMUM_SPACING
        )
    )

    # =====================================
    # 新：真实 Euler bend
    # =====================================

    euler_legal, euler_endpoint, euler_reason, euler_geometry = (
        evaluate_bend_candidate(
            base_bend_polygon=bend_polygon,
            position=position,
            old_direction="R",
            new_direction="D",
            obstacles=obstacles,
            minimum_spacing=MINIMUM_SPACING,
        )
    )

    print("Sharp Manhattan:")
    print("  endpoint:", sharp_endpoint)
    print("  legal:", sharp_legal)
    print("  reason:", sharp_reason)

    print()

    print("Real Euler:")
    print("  endpoint:", euler_endpoint)
    print("  legal:", euler_legal)
    print("  reason:", euler_reason)

    # -------------------------
    # --- Curvy-aware A* ---
    # -------------------------

    current_g = 20
    print("\n--- Curvy-aware A* ---")

    result = curvy_astar(
        start_position=position,
        start_direction=direction,
        goal=goal,
        base_bend_polygon=bend_polygon,
        obstacles=obstacles,
        bend_length=bend_length,
    )

    print("搜索成功:", result["success"])
    print(
        "总代价:",
        round(result["cost"], 3)
        if result["cost"] is not None
        else None
    )

    print(
        "扩展节点数:",
        result["expanded_nodes"]
    )

    print("\n--- Route ---")

    for i, state in enumerate(
            result["states"]
    ):
        print(
            f"State {i}:",
            state
        )

    print("\n--- Transitions ---")

    for i, transition in enumerate(
            result["transitions"]
    ):
        print(
            f"Step {i + 1}:",
            transition["type"],
            "→",
            transition["position"],
            transition["direction"],
            "cost =",
            transition["cost"]
        )

    if result["success"]:
        export_result_gds(
            result,
            obstacles,
            filename="curvy_astar_result.gds"
        )

if __name__ == "__main__":
    main()