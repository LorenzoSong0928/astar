"""负责多网络调度、冲突处理和 rip-up & reroute。"""

from astar import (
    directional_astar,
    curvy_astar,
    backtrack_directional,
    states_to_positions,
    count_bends,
)

from grid import (
    register_path,
    remove_path,
    find_conflicts,
    find_conflicting_net_ids,
    print_grid,
    grid_to_physical,
    grid_obstacles_to_polygons,
    grid_to_boundary_polygon,
)

from geometry import merge_route_geometry

# ==================== 7. 单网络布线封装 ====================
def route_net(
        grid,
        net_id,
        net,
        strict=True,
        conflict_penalty=50,
        bend_cost=5,
        clearance=0
):
    start = net["start"]
    goal = net["goal"]

    parent, cost, visited, goal_state = directional_astar(
        grid=grid,
        start=start,
        goal=goal,
        start_direction=net["start_direction"],
        goal_entry_direction=net["goal_entry_direction"],
        bend_cost=bend_cost,
        clearance=clearance,
        strict=strict,
        conflict_penalty=conflict_penalty
    )

    state_path = backtrack_directional(parent, goal_state)
    path = states_to_positions(state_path)
    conflicts = find_conflicts(path, grid)

    result = {
        "net_id": net_id,
        "success": goal_state is not None,
        "path": path,
        "state_path": state_path,
        "goal_state": goal_state,
        "conflicts": conflicts,
        "visited_count": len(visited),
        "cost": cost.get(goal_state),
    }

    return result

def route_all_nets(
    grid,
    owner_grid,
    nets,
    routing_order=None,
    conflict_penalty=50,
    bend_cost=5,
    clearance=0
):
    """按指定顺序完成多网络布线，并处理冲突与重布线。"""

    if routing_order is None:
        routing_order = list(range(len(nets)))

    routed_paths = {}

    for net_id in routing_order:
        net = nets[net_id]

        # ---------- 第一次尝试：严格模式 ----------
        result = route_net(
            grid,
            net_id,
            net,
            strict=True,
            conflict_penalty=conflict_penalty,
            bend_cost=bend_cost,
            clearance=clearance
        )

        print(f"\n正在严格布线：net{net_id}")
        print("是否成功：", result["success"])

        if result["success"]:
            print_route_result(result)

            print_grid(
                grid,
                result["path"],
                net["start"],
                net["goal"]
            )

            register_path(
                grid,
                owner_grid,
                result["path"],
                net_id
            )

            routed_paths[net_id] = result["path"]

            continue

        # ---------- 严格模式失败 ----------
        print("严格模式下未找到路径")

        print_grid(
            grid,
            None,
            net["start"],
            net["goal"]
        )

        # ---------- 第二次尝试：非严格模式 ----------
        loose_result = route_net(
            grid,
            net_id,
            net,
            strict=False,
            conflict_penalty=conflict_penalty,
            bend_cost=bend_cost,
            clearance=clearance
        )

        print(f"\n正在非严格布线：net {net_id}")
        print("是否成功：", loose_result["success"])

        if not loose_result["success"]:
            print("非严格模式下也未找到路径")
            continue

        print_route_result(loose_result)

        print_grid(
            grid,
            loose_result["path"],
            net["start"],
            net["goal"]
        )

        # ---------- 找出冲突网络 ----------
        conflicting_net_ids = find_conflicting_net_ids(
            loose_result["path"],
            owner_grid
        )
        print("冲突网络编号：", conflicting_net_ids)

        # ---------- 拆除冲突网络 ----------
        ripped_net_ids = list(conflicting_net_ids)

        for ripped_id in ripped_net_ids:
            remove_path(
                grid,
                owner_grid,
                routed_paths[ripped_id],
                ripped_id
            )

            routed_paths.pop(ripped_id)

        print("\n拆除冲突网络后：")
        print_grid(grid)

        # ---------- 拆线后重新严格布当前网络 ----------
        reroute_current = route_net(
            grid,
            net_id,
            net,
            strict=True,
            conflict_penalty=conflict_penalty,
            bend_cost=bend_cost,
            clearance=clearance
        )

        print(f"\n拆线后重新严格布线：net {net_id}")
        print("是否成功：", reroute_current["success"])

        if reroute_current["success"]:
            print_route_result(reroute_current)
            print_grid(
                grid,
                reroute_current["path"],
                net["start"],
                net["goal"]
            )

            # 正式注册当前网络
            register_path(
                grid,
                owner_grid,
                reroute_current["path"],
                net_id
            )

            routed_paths[net_id] = reroute_current["path"]
        else:
            print(
                f"net {net_id} 拆除冲突网络后仍然无法布通"
            )

        # ---------- 重新布被拆除的网络 ----------
        for ripped_id in ripped_net_ids:

            ripped_net = nets[ripped_id]

            reroute_ripped = route_net(
                grid,
                ripped_id,
                ripped_net,
                strict=True,
                conflict_penalty=conflict_penalty,
                bend_cost=bend_cost,
                clearance=clearance
            )

            print(f"\n重新布线被拆网络：net {ripped_id}")
            print("是否成功：", reroute_ripped["success"])

            if reroute_ripped["success"]:

                print_route_result(reroute_ripped)

                print_grid(
                    grid,
                    reroute_ripped["path"],
                    ripped_net["start"],
                    ripped_net["goal"]
                )

                register_path(
                    grid,
                    owner_grid,
                    reroute_ripped["path"],
                    ripped_id
                )

                routed_paths[ripped_id] = reroute_ripped["path"]

            else:
                print(f"net {ripped_id} 重新布线失败")

    return routed_paths

def route_grid_curvy(
    grid,
    start_grid,
    goal_grid,
    start_direction,
    goal_entry_direction,
    bend_polygon,
    bend_length,
    grid_size=10,
    routed_geometries=None,
    strict=True,
    conflict_penalty=0,
):
    grid_height = len(grid)

    # Grid → Physical
    start_position = grid_to_physical(
        start_grid[0],
        start_grid[1],
        grid_height,
        grid_size
    )

    goal_position = grid_to_physical(
        goal_grid[0],
        goal_grid[1],
        grid_height,
        grid_size
    )

    # Grid → Geometry
    obstacles = grid_obstacles_to_polygons(
        grid,
        grid_size
    )

    boundary = grid_to_boundary_polygon(
        grid,
        grid_size
    )

    # Curvy-aware A*
    result = curvy_astar(
        start_position=start_position,
        start_direction=start_direction,
        goal=goal_position,
        goal_entry_direction=goal_entry_direction,
        base_bend_polygon=bend_polygon,
        obstacles=obstacles,
        bend_length=bend_length,
        boundary=boundary,
        routed_geometries=routed_geometries,
        strict=strict,
        conflict_penalty=conflict_penalty,
    )

    return result

def register_route_geometry(
    routed_geometries,
    net_id,
    result,
):
    """
    将一条已经成功布通的网络，
    注册为完整的真实 waveguide geometry。
    """

    if not result["success"]:
        return False

    route_geometry = merge_route_geometry(
        result["transitions"]
    )

    routed_geometries[net_id] = route_geometry

    return True

def route_nets_curvy_sequential(
    grid,
    nets,
    bend_polygon,
    bend_length,
    grid_size=10,
):
    routed_geometries = {}
    results = {}

    for net_id, net in nets.items():

        result = route_grid_curvy(
            grid=grid,
            start_grid=net["start"],
            goal_grid=net["goal"],
            start_direction=net["start_direction"],
            goal_entry_direction=net["goal_entry_direction"],
            bend_polygon=bend_polygon,
            bend_length=bend_length,
            grid_size=grid_size,
            routed_geometries=routed_geometries,
        )

        results[net_id] = result

        if result["success"]:
            register_route_geometry(
                routed_geometries,
                net_id=net_id,
                result=result,
            )

    return results, routed_geometries

def route_grid_curvy_with_fallback(
    grid,
    start_grid,
    goal_grid,
    start_direction,
    goal_entry_direction,
    bend_polygon,
    bend_length,
    grid_size=10,
    routed_geometries=None,
    conflict_penalty=1,
):
    """
    先尝试 strict routing。

    strict 失败后，
    再尝试 non-strict routing。
    """

    # =====================================
    # 1. Strict Routing
    # =====================================

    strict_result = route_grid_curvy(
        grid=grid,
        start_grid=start_grid,
        goal_grid=goal_grid,
        start_direction=start_direction,
        goal_entry_direction=goal_entry_direction,
        bend_polygon=bend_polygon,
        bend_length=bend_length,
        grid_size=grid_size,
        routed_geometries=routed_geometries,
        strict=True,
        conflict_penalty=0,
    )

    if strict_result["success"]:

        strict_result["routing_mode"] = (
            "strict"
        )

        return strict_result

    # =====================================
    # 2. Non-strict Routing
    # =====================================

    nonstrict_result = route_grid_curvy(
        grid=grid,
        start_grid=start_grid,
        goal_grid=goal_grid,
        start_direction=start_direction,
        goal_entry_direction=goal_entry_direction,
        bend_polygon=bend_polygon,
        bend_length=bend_length,
        grid_size=grid_size,
        routed_geometries=routed_geometries,
        strict=False,
        conflict_penalty=conflict_penalty,
    )

    nonstrict_result["routing_mode"] = (
        "non-strict"
    )

    return nonstrict_result

def rip_up_route_geometry(
    routed_geometries,
    net_id,
):
    """
    从已布网络中删除指定 net 的真实 geometry。

    返回被删除的 geometry。
    如果 net_id 不存在，则返回 None。
    """

    if net_id not in routed_geometries:
        return None

    return routed_geometries.pop(net_id)

def route_net_curvy_with_rr(
    grid,
    nets,
    current_net_id,
    bend_polygon,
    bend_length,
    routed_geometries,
    grid_size=10,
    conflict_penalty=1,
):
    """
    对 current net 执行：

    1. Strict routing
    2. Strict fail → Non-strict routing
    3. 找到 conflicting nets
    4. Rip-up conflicting nets
    5. Strict reroute current net
    6. Reroute ripped nets
    """

    # 快照当前几何
    original_routed_geometries = dict(
        routed_geometries
    )
    current_net = nets[current_net_id]

    # =====================================
    # 1. Strict Routing
    # =====================================

    strict_result = route_grid_curvy(
        grid=grid,
        start_grid=current_net["start"],
        goal_grid=current_net["goal"],
        start_direction=current_net["start_direction"],
        goal_entry_direction=current_net["goal_entry_direction"],
        bend_polygon=bend_polygon,
        bend_length=bend_length,
        grid_size=grid_size,
        routed_geometries=routed_geometries,
        strict=True,
    )

    # Strict 直接成功
    if strict_result["success"]:

        register_route_geometry(
            routed_geometries,
            net_id=current_net_id,
            result=strict_result,
        )

        return {
            "success": True,
            "routing_mode": "strict",
            "current_result": strict_result,
            "ripped_net_ids": set(),
            "rerouted_results": {},
        }

    # =====================================
    # 2. Non-strict Routing
    # =====================================

    nonstrict_result = route_grid_curvy(
        grid=grid,
        start_grid=current_net["start"],
        goal_grid=current_net["goal"],
        start_direction=current_net["start_direction"],
        goal_entry_direction=current_net["goal_entry_direction"],
        bend_polygon=bend_polygon,
        bend_length=bend_length,
        grid_size=grid_size,
        routed_geometries=routed_geometries,
        strict=False,
        conflict_penalty=conflict_penalty,
    )

    # Non-strict 也失败
    if not nonstrict_result["success"]:

        return {
            "success": False,
            "routing_mode": "failed",
            "current_result": nonstrict_result,
            "ripped_net_ids": set(),
            "rerouted_results": {},
        }

    # =====================================
    # 3. 找出冲突网络
    # =====================================

    conflict_net_ids = (
        nonstrict_result[
            "conflict_net_ids"
        ]
    )

    if not conflict_net_ids:

        return {
            "success": False,
            "routing_mode": "failed",
            "current_result": nonstrict_result,
            "ripped_net_ids": set(),
            "rerouted_results": {},
        }

    # =====================================
    # 4. Rip-up
    # =====================================

    ripped_geometries = {}

    for net_id in conflict_net_ids:

        geometry = rip_up_route_geometry(
            routed_geometries,
            net_id,
        )

        if geometry is not None:
            ripped_geometries[net_id] = (
                geometry
            )

    # =====================================
    # 5. Strict reroute current net
    # =====================================

    current_result = route_grid_curvy(
        grid=grid,
        start_grid=current_net["start"],
        goal_grid=current_net["goal"],
        start_direction=current_net["start_direction"],
        goal_entry_direction=current_net["goal_entry_direction"],
        bend_polygon=bend_polygon,
        bend_length=bend_length,
        grid_size=grid_size,
        routed_geometries=routed_geometries,
        strict=True,
    )

    if not current_result["success"]:
        # Rollback:clear和update在传进来的字典基础上修改
        routed_geometries.clear()
        routed_geometries.update(original_routed_geometries)

        return {
            "success": False,
            "routing_mode": "rr-failed",
            "current_result": current_result,
            "ripped_net_ids": set(
                ripped_geometries.keys()
            ),
            "rerouted_results": {},
        }

    register_route_geometry(
        routed_geometries,
        net_id=current_net_id,
        result=current_result,
    )

    # =====================================
    # 6. Reroute ripped nets
    # =====================================

    rerouted_results = {}

    for ripped_net_id in (
        ripped_geometries.keys()
    ):

        ripped_net = nets[
            ripped_net_id
        ]

        reroute_result = route_grid_curvy(
            grid=grid,
            start_grid=ripped_net["start"],
            goal_grid=ripped_net["goal"],
            start_direction=ripped_net["start_direction"],
            goal_entry_direction=ripped_net["goal_entry_direction"],
            bend_polygon=bend_polygon,
            bend_length=bend_length,
            grid_size=grid_size,
            routed_geometries=routed_geometries,
            strict=True,
        )

        rerouted_results[
            ripped_net_id
        ] = reroute_result

        if not reroute_result["success"]:
            # Rollback
            routed_geometries.clear()
            routed_geometries.update(original_routed_geometries)

            return {
                "success": False,
                "routing_mode": "rr-failed",
                "current_result": current_result,
                "ripped_net_ids": set(
                    ripped_geometries.keys()
                ),
                "rerouted_results": rerouted_results,
            }

        register_route_geometry(
            routed_geometries,
            net_id=ripped_net_id,
            result=reroute_result,
        )

    return {
        "success": True,
        "routing_mode": "rip-up-reroute",
        "current_result": current_result,
        "ripped_net_ids": set(
            ripped_geometries.keys()
        ),
        "rerouted_results": rerouted_results,
    }

def print_route_result(result):
    """打印布线结果统计信息。"""
    print("总代价：", result["cost"])
    print("移动步数：", len(result["path"]) - 1)
    print(
        "转弯次数：",
        count_bends(result["state_path"])
    )
    print(
        "冲突数量：",
        len(result["conflicts"])
    )