import gdsfactory as gf

from shapely.geometry import box, LineString

from geometry import (
    BEND_SPAN,
    WAVEGUIDE_WIDTH,
    MINIMUM_SPACING,

    create_base_bend,
    is_geometry_legal,
    evaluate_bend_candidate,
    get_straight_geometry,
    evaluate_straight_candidate,
)

from astar import (
    get_curvy_neighbors,
    curvy_astar,
)

# ============================================================
# 0. Parameters
# ============================================================


# ============================================================
# 1. Create Base Euler Bend
# ============================================================


# ============================================================
# 2. Geometry DRC
# ============================================================




# ============================================================
# 3. Bend Geometry
# ============================================================


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




# ============================================================
# 5. Candidate Evaluation
# ============================================================



# ============================================================
# 6. Curvy-aware Neighbors
# ============================================================




# ============================================================
# 7. Curvy-aware Heuristic
# ============================================================

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