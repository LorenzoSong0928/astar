"""负责多网络调度、冲突处理和 rip-up & reroute。"""

from astar import (
    directional_astar,
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
)

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