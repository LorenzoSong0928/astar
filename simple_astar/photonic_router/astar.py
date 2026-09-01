"""负责 Geometry-aware 单网络 A* 搜索。"""

import heapq
import itertools

from .config import (
    WAVEGUIDE_WIDTH,
    MINIMUM_SPACING,
    BEND_SPAN,
    STRAIGHT_LENGTH,
)

from .geometry import (
    evaluate_bend_candidate,
    evaluate_straight_candidate,
    find_routed_conflicts,
)

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
    boundary=None,
    routed_geometries=None,
    strict=True,
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
            boundary=boundary,
            routed_geometries=routed_geometries,
            strict=strict,
        )
    )

    if legal:
        conflict_net_ids = find_routed_conflicts(
            geometry,
            routed_geometries,
            minimum_spacing=minimum_spacing,
        )

        neighbors.append({
            "type": "straight",
            "position": endpoint,
            "direction": direction,
            "geometry": geometry,
            "cost": straight_length,
            "conflict_net_ids": conflict_net_ids,
        })

    # -------------------------
    # Euler bends
    # -------------------------

    for new_direction in turn_directions[
        direction
    ]:

        legal, endpoint, reason, geometry = (
            evaluate_bend_candidate(
                base_bend_polygon=base_bend_polygon,
                position=position,
                old_direction=direction,
                new_direction=new_direction,
                obstacles=obstacles,
                minimum_spacing=minimum_spacing,
                bend_span=bend_span,
                boundary=boundary,
                routed_geometries=routed_geometries,
                strict=strict,
            )
        )

        if legal:
            conflict_net_ids = find_routed_conflicts(
                geometry,
                routed_geometries,
                minimum_spacing=minimum_spacing,
            )

            neighbors.append({
                "type": "bend",
                "position": endpoint,
                "direction": new_direction,
                "geometry": geometry,
                "cost": bend_length,
                "conflict_net_ids": conflict_net_ids,
            })

    return neighbors


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
    goal_entry_direction,
    base_bend_polygon,
    obstacles,
    bend_length,
    minimum_spacing=MINIMUM_SPACING,
    boundary=None,
    routed_geometries=None,
    strict=True,
    conflict_penalty=0,
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

            # 如果指定了终点进入方向，
            # 当前方向必须满足要求
            if (
                    goal_entry_direction is not None
                    and current_direction
                    != goal_entry_direction
            ):
                continue

            states, transitions = (
                reconstruct_path(
                    came_from,
                    transition_from,
                    current_state
                )
            )

            # 汇总整条路径的冲突网络
            conflict_net_ids = set()

            for transition in transitions:
                conflict_net_ids.update(
                    transition.get(
                        "conflict_net_ids",
                        set()
                    )
                )

            return {
                "success": True,
                "cost": current_best_g,
                "states": states,
                "transitions": transitions,
                "expanded_nodes": expanded_nodes,
                "conflict_net_ids": conflict_net_ids,
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
            boundary=boundary,
            routed_geometries=routed_geometries,
            strict=strict,
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

            # Step Cost
            conflict_net_ids = neighbor.get(
                "conflict_net_ids",
                set()
            )

            conflict_cost = 0

            if not strict:
                conflict_cost = (
                        conflict_penalty
                        * len(conflict_net_ids)
                )

            step_cost = (
                    neighbor["cost"]
                    + conflict_cost
            )

            new_g = (
                    current_best_g
                    + step_cost
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

                transition = neighbor.copy()

                transition["conflict_cost"] = (
                    conflict_cost
                )

                transition["step_cost"] = (
                    step_cost
                )

                transition_from[new_state] = (
                    transition
                )

                # h
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
        "conflict_net_ids": set(),
    }
