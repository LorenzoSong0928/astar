import gdsfactory as gf

from shapely.geometry import box, LineString

from simple_astar.photonic_router.geometry import (
    BEND_SPAN,
    WAVEGUIDE_WIDTH,
    MINIMUM_SPACING,

    create_base_bend,
    is_geometry_legal,
    evaluate_bend_candidate,
    get_straight_geometry,
    evaluate_straight_candidate,
    merge_route_geometry,
    find_routed_conflicts
)

from simple_astar.photonic_router.astar import (
    get_curvy_neighbors,
    curvy_astar,
)

from simple_astar.photonic_router.router import (
    route_grid_curvy,
    register_route_geometry,
    route_nets_curvy_sequential,
    route_grid_curvy_with_fallback,
    rip_up_route_geometry,
    route_net_curvy_with_rr
)

from simple_astar.photonic_router.gds_export import export_routed_geometries
# ============================================================
# 0. Parameters
# 1. Create Base Euler Bend
# 2. Geometry DRC
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
# 5. Candidate Evaluation
# 6. Curvy-aware Neighbors
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

    # =====================================
    # Activate generic PDK
    # =====================================

    gf.gpdk.PDK.activate()

    # -------------------------
    # Base bend
    # -------------------------

    bend_component, bend_polygon = (
        create_base_bend()
    )

    bend_length = bend_component.info[
        "length"
    ]

#==========================================
    # -------------------------
    # --- Curvy-aware A* ---
    # -------------------------

    collision_candidate = box(
        19,
        29,
        21,
        31
    )

    print("\n==============================")
    print(" Multi-Net Geometry DRC Test ")
    print("==============================")
    # 空地图
    multi_grid = [
        [0] * 8
        for _ in range(8)
    ]
    # 横纵网络
    nets = {
        0: {
            "start": (0, 4),
            "goal": (7, 4),
            "start_direction": "R",
            "goal_entry_direction": "R",
        },

        1: {
            "start": (3, 6),
            "goal": (3, 1),
            "start_direction": "U",
            "goal_entry_direction": "U",
        },
    }

    results, routed_geometries = (
        route_nets_curvy_sequential(
            grid=multi_grid,
            nets=nets,
            bend_polygon=bend_polygon,
            bend_length=bend_length,
            grid_size=10,
        )
    )

    # 测试碰撞
    conflicts = find_routed_conflicts(
        collision_candidate,
        routed_geometries,
    )

    print("\n--- Routed Conflict Detection Test ---")
    print(
        "Collision candidate conflicts:",
        conflicts
    )
    # 远处
    far_candidate = box(
        70,
        60,
        71,
        61
    )

    conflicts = find_routed_conflicts(
        far_candidate,
        routed_geometries,
    )

    print(
        "Far candidate conflicts:",
        conflicts
    )

    test_routed_geometries = {
        0: routed_geometries[0]
    }

    print("\n--- Strict vs Non-strict Candidate Test ---")

    strict_legal, endpoint, strict_reason, geometry = (
        evaluate_straight_candidate(
            position=(30, 20),
            direction="U",
            obstacles=[],
            routed_geometries=test_routed_geometries,
            strict=True,
        )
    )

    nonstrict_legal, endpoint, nonstrict_reason, geometry = (
        evaluate_straight_candidate(
            position=(30, 20),
            direction="U",
            obstacles=[],
            routed_geometries=test_routed_geometries,
            strict=False,
        )
    )

    conflicts = find_routed_conflicts(
        geometry,
        test_routed_geometries,
    )

    print("Strict:")
    print("  legal:", strict_legal)
    print("  reason:", strict_reason)

    print("Non-strict:")
    print("  legal:", nonstrict_legal)
    print("  reason:", nonstrict_reason)
    print("  conflicts:", conflicts)

    print("\n--- Net 1 Non-strict A* ---")

    # 这个实验只让 Net 1 与 Net 0 比较
    net0_only_geometries = {
        0: routed_geometries[0]
    }

    result1_nonstrict = route_grid_curvy(
        grid=multi_grid,
        start_grid=nets[1]["start"],
        goal_grid=nets[1]["goal"],
        start_direction=nets[1]["start_direction"],
        goal_entry_direction=nets[1]["goal_entry_direction"],
        bend_polygon=bend_polygon,
        bend_length=bend_length,
        grid_size=10,
        routed_geometries=net0_only_geometries,

        strict=False,
        conflict_penalty=1,
    )
    print(
        "success:",
        result1_nonstrict["success"]
    )

    print(
        "cost:",
        result1_nonstrict["cost"]
    )

    print(
        "conflict nets:",
        result1_nonstrict["conflict_net_ids"]
    )

    print("\nRoute:")

    for state in result1_nonstrict["states"]:
        print(" ", state)

    print("\nTransitions:")

    for i, transition in enumerate(
            result1_nonstrict["transitions"],
            start=1
    ):
        print(
            f"Step {i}:",
            transition["type"],
            "→",
            transition["position"],
            transition["direction"],
            "| geometry cost =",
            transition["cost"],
            "| conflict cost =",
            transition["conflict_cost"],
            "| conflicts =",
            transition["conflict_net_ids"],
        )

    # 横向封死场景
    multi_grid = [
        [0] * 8
        for _ in range(8)
    ]
    # 横纵网络
    nets = {
        0: {
            "start": (0, 4),
            "goal": (7, 4),
            "start_direction": "R",
            "goal_entry_direction": "R",
        },

        1: {
            "start": (3, 6),
            "goal": (3, 1),
            "start_direction": "U",
            "goal_entry_direction": "U",
        },
    }
    blocked_geometries = {}
    # 先布net0
    result0_blocked = route_grid_curvy(
        grid=multi_grid,
        start_grid=nets[0]["start"],
        goal_grid=nets[0]["goal"],
        start_direction=nets[0]["start_direction"],
        goal_entry_direction=nets[0]["goal_entry_direction"],
        bend_polygon=bend_polygon,
        bend_length=bend_length,
        grid_size=10,
        routed_geometries=blocked_geometries,
        strict=True,
    )

    if result0_blocked["success"]:
        register_route_geometry(
            blocked_geometries,
            net_id=0,
            result=result0_blocked,
        )




    fallback_result = (
        route_grid_curvy_with_fallback(
            grid=multi_grid,
            start_grid=nets[1]["start"],
            goal_grid=nets[1]["goal"],
            start_direction=nets[1]["start_direction"],
            goal_entry_direction=nets[1]["goal_entry_direction"],
            bend_polygon=bend_polygon,
            bend_length=bend_length,
            grid_size=10,
            routed_geometries=blocked_geometries,
            conflict_penalty=1,
        )
    )

    print(
        "\n--- Automatic Strict → Non-strict Test ---"
    )

    print(
        "success:",
        fallback_result["success"]
    )

    print(
        "routing mode:",
        fallback_result["routing_mode"]
    )

    print(
        "cost:",
        fallback_result["cost"]
    )

    print(
        "conflict nets:",
        fallback_result["conflict_net_ids"]
    )

    for state in fallback_result["states"]:
        print(" ", state)

    # 复制了之前的字典
    rr_geometries = dict(
        blocked_geometries
    )

    print("\n--- Geometry Rip-up Test ---")

    print(
        "Before rip-up:",
        list(rr_geometries.keys())
    )

    ripped_geometries = {}

    for net_id in fallback_result[
        "conflict_net_ids"
    ]:
        geometry = rip_up_route_geometry(
            rr_geometries,
            net_id,
        )

        if geometry is not None:
            ripped_geometries[net_id] = (
                geometry
            )

    print(
        "Ripped net ids:",
        list(ripped_geometries.keys())
    )

    print(
        "After rip-up:",
        list(rr_geometries.keys())
    )

    print("\n--- Reroute Current Net Strictly ---")

    reroute_current_result = route_grid_curvy(
        grid=multi_grid,
        start_grid=nets[1]["start"],
        goal_grid=nets[1]["goal"],
        start_direction=nets[1]["start_direction"],
        goal_entry_direction=nets[1]["goal_entry_direction"],
        bend_polygon=bend_polygon,
        bend_length=bend_length,
        grid_size=10,

        # 注意：
        # 此时 Net 0 已经被 rip-up
        routed_geometries=rr_geometries,

        strict=True,
    )

    print(
        "success:",
        reroute_current_result["success"]
    )

    print(
        "cost:",
        reroute_current_result["cost"]
    )

    print(
        "conflict nets:",
        reroute_current_result[
            "conflict_net_ids"
        ]
    )

    if reroute_current_result["success"]:
        for state in reroute_current_result["states"]:
            print(" ", state)

    if reroute_current_result["success"]:
        registered = register_route_geometry(
            rr_geometries,
            net_id=1,
            result=reroute_current_result,
        )

        print(
            "Current net registered:",
            registered
        )

        print(
            "Registered net ids:",
            list(rr_geometries.keys())
        )

    print("\n--- Reroute Ripped Net 0 ---")

    reroute_ripped_result = route_grid_curvy(
        grid=multi_grid,
        start_grid=nets[0]["start"],
        goal_grid=nets[0]["goal"],
        start_direction=nets[0]["start_direction"],
        goal_entry_direction=nets[0]["goal_entry_direction"],
        bend_polygon=bend_polygon,
        bend_length=bend_length,
        grid_size=10,
        routed_geometries=rr_geometries,
        strict=True,
    )

    print(
        "success:",
        reroute_ripped_result["success"]
    )

    print(
        "cost:",
        reroute_ripped_result["cost"]
    )

    print(
        "conflict nets:",
        reroute_ripped_result[
            "conflict_net_ids"
        ]
    )

    if reroute_ripped_result["success"]:
        for state in reroute_ripped_result["states"]:
            print(" ", state)

    if reroute_ripped_result["success"]:
        registered = register_route_geometry(
            rr_geometries,
            net_id=0,
            result=reroute_ripped_result,
        )

        print(
            "Ripped net registered:",
            registered
        )

        print(
            "Final registered net ids:",
            sorted(rr_geometries.keys())
        )
        #最后重新DRC验证
        net0_final_geometry = rr_geometries[0]
        net1_final_geometry = rr_geometries[1]

        print("\n--- Final RR Geometry Check ---")

        print(
            "intersects:",
            net0_final_geometry.intersects(
                net1_final_geometry
            )
        )

        print(
            "distance:",
            net0_final_geometry.distance(
                net1_final_geometry
            )
        )

        print(
            "minimum spacing:",
            MINIMUM_SPACING
        )

    rr_test_geometries = {}

    initial_net0_result = route_grid_curvy(
        grid=multi_grid,
        start_grid=nets[0]["start"],
        goal_grid=nets[0]["goal"],
        start_direction=nets[0]["start_direction"],
        goal_entry_direction=nets[0]["goal_entry_direction"],
        bend_polygon=bend_polygon,
        bend_length=bend_length,
        grid_size=10,
        routed_geometries=rr_test_geometries,
        strict=True,
    )

    register_route_geometry(
        rr_test_geometries,
        net_id=0,
        result=initial_net0_result,
    )

    rr_result = route_net_curvy_with_rr(
        grid=multi_grid,
        nets=nets,
        current_net_id=1,
        bend_polygon=bend_polygon,
        bend_length=bend_length,
        routed_geometries=rr_test_geometries,
        grid_size=10,
        conflict_penalty=1,
    )

    print("\n--- Automatic Geometry RR ---")

    print(
        "success:",
        rr_result["success"]
    )

    print(
        "routing mode:",
        rr_result["routing_mode"]
    )

    print(
        "ripped net ids:",
        rr_result["ripped_net_ids"]
    )

    print(
        "current net cost:",
        rr_result[
            "current_result"
        ]["cost"]
    )

    for net_id, result in (
            rr_result[
                "rerouted_results"
            ].items()
    ):
        print(
            f"rerouted net {net_id} cost:",
            result["cost"]
        )

    print(
        "final registered net ids:",
        sorted(
            rr_test_geometries.keys()
        )
    )

    filepath = export_routed_geometries(
        rr_test_geometries,
        filename="geometry_rr_result.gds",
    )

    print(
        "\nGeometry GDS exported:",
        filepath
    )

    print("\n==============================")
    print(" RR Failure Rollback Test ")
    print("==============================")

    rollback_grid = [
        [0] * 8
        for _ in range(8)
    ]

    rollback_nets = {
        0: {
            # 横向贯穿整个区域
            "start": (0, 4),
            "goal": (7, 4),
            "start_direction": "R",
            "goal_entry_direction": "R",
        },

        1: {
            # 纵向贯穿整个区域
            "start": (3, 7),
            "goal": (3, 0),
            "start_direction": "U",
            "goal_entry_direction": "U",
        },
    }

    rollback_geometries = {}

    initial_net0 = route_grid_curvy(
        grid=rollback_grid,
        start_grid=rollback_nets[0]["start"],
        goal_grid=rollback_nets[0]["goal"],
        start_direction=rollback_nets[0]["start_direction"],
        goal_entry_direction=rollback_nets[0]["goal_entry_direction"],
        bend_polygon=bend_polygon,
        bend_length=bend_length,
        grid_size=10,
        routed_geometries=rollback_geometries,
        strict=True,
    )

    register_route_geometry(
        rollback_geometries,
        net_id=0,
        result=initial_net0,
    )

    original_net0_geometry = rollback_geometries[0]

    print(
        "Before RR:",
        sorted(rollback_geometries.keys())
    )

    rollback_result = route_net_curvy_with_rr(
        grid=rollback_grid,
        nets=rollback_nets,
        current_net_id=1,
        bend_polygon=bend_polygon,
        bend_length=bend_length,
        routed_geometries=rollback_geometries,
        grid_size=10,
        conflict_penalty=1,
    )

    print(
        "RR success:",
        rollback_result["success"]
    )

    print(
        "routing mode:",
        rollback_result["routing_mode"]
    )

    print(
        "ripped net ids:",
        rollback_result["ripped_net_ids"]
    )

    for net_id, result in (
            rollback_result["rerouted_results"].items()
    ):
        print(
            f"rerouted net {net_id} success:",
            result["success"]
        )

    print(
        "After RR:",
        sorted(rollback_geometries.keys())
    )

    net0_restored = (
            0 in rollback_geometries
            and rollback_geometries[0].equals(
        original_net0_geometry
    )
    )

    print(
        "Net 0 restored:",
        net0_restored
    )

    print(
        "Net 1 present:",
        1 in rollback_geometries
    )

    print("\n==============================")
    print(" Final Regression Check ")
    print("==============================")

    # =====================================
    # 1. Non-strict conflict
    # =====================================

    assert result1_nonstrict["success"]
    assert result1_nonstrict["conflict_net_ids"] == {0}

    # =====================================
    # 2. Automatic RR
    # =====================================

    assert rr_result["success"]
    assert rr_result["routing_mode"] == "rip-up-reroute"
    assert rr_result["ripped_net_ids"] == {0}

    # =====================================
    # 3. Final routed database
    # =====================================

    assert sorted(
        rr_test_geometries.keys()
    ) == [0, 1]

    # =====================================
    # 4. Final Geometry DRC
    # =====================================

    net0_geometry = rr_test_geometries[0]
    net1_geometry = rr_test_geometries[1]

    assert not net0_geometry.intersects(
        net1_geometry
    )

    assert (
            net0_geometry.distance(net1_geometry)
            >= MINIMUM_SPACING
    )

    # =====================================
    # 5. Goal Entry Direction
    # =====================================

    assert (
            rr_result["current_result"]["states"][-1][2]
            == nets[1]["goal_entry_direction"]
    )

    assert (
            rr_result["rerouted_results"][0]["states"][-1][2]
            == nets[0]["goal_entry_direction"]
    )

    # =====================================
    # 6. Rollback
    # =====================================

    assert not rollback_result["success"]

    assert sorted(
        rollback_geometries.keys()
    ) == [0]

    assert rollback_geometries[0].equals(
        original_net0_geometry
    )

    assert 1 not in rollback_geometries

    print("All regression checks passed.")

if __name__ == "__main__":
    main()