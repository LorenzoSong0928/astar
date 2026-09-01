"""负责：单条布线相关"""

import heapq
import itertools
from grid import get_directional_neighbours

from config import (
    WAVEGUIDE_WIDTH,
    MINIMUM_SPACING,
    BEND_SPAN,
    STRAIGHT_LENGTH,
)

from geometry import (
    evaluate_bend_candidate,
    evaluate_straight_candidate,
    find_routed_conflicts,
)

def heuristic(pos, goal):
    x1, y1 = pos
    x2, y2 = goal

    return abs(x1 - x2) + abs(y1 - y2)

def directional_astar(
    grid,
    start,
    goal,
    start_direction=None,
    goal_entry_direction=None,
    bend_cost=5,
    clearance=0,
    strict=True,
    conflict_penalty=50
):
    open_set = []

    # 起点没有进入方向
    start_state = (start[0], start[1], None)

    start_priority = heuristic(start, goal)
    heapq.heappush(open_set, (start_priority, start_state))

    cost = {start_state: 0}
    parent = {start_state: None}
    visited = set()

    goal_state = None

    while open_set:
        current_priority, current_state = heapq.heappop(open_set)

        if current_state in visited:
            continue

        visited.add(current_state)

        current_x, current_y, current_direction = current_state

        # 到达终点坐标后，还要检查进入方向
        if (current_x, current_y) == goal:
            if (
                goal_entry_direction is None
                or current_direction == goal_entry_direction
            ):
                goal_state = current_state
                break

            # 方向错误，本次到达不算成功
            continue

        for neighbour_state in get_directional_neighbours(
                current_state,
                grid,
                clearance,
                strict
        ):
            new_x, new_y, new_direction = neighbour_state

            if (
                    current_state == start_state
                    and start_direction is not None
                    and new_direction != start_direction
            ):
                continue

            step_cost = get_step_cost(
                current_direction,
                new_direction,
                bend_cost
            )

            if not strict and grid[new_y][new_x] == 2:
                step_cost += conflict_penalty

            new_cost = cost[current_state] + step_cost

            if (
                    neighbour_state not in cost
                    or new_cost < cost[neighbour_state]
            ):
                cost[neighbour_state] = new_cost
                parent[neighbour_state] = current_state

                priority = (
                        new_cost
                        + heuristic((new_x, new_y), goal)
                )

                heapq.heappush(
                    open_set,
                    (priority, neighbour_state)
                )

    return parent, cost, visited, goal_state

def backtrack_directional(parent, goal_state):
    if goal_state is None:
        return None

    state_path = []
    current_state = goal_state

    while current_state is not None:
        state_path.append(current_state)
        current_state = parent[current_state]

    state_path.reverse()

    return state_path

# 判断转弯cost
def get_step_cost(current_direction, new_direction, bend_cost=5):
    move_cost = 1

    if current_direction is None:
        return move_cost

    if current_direction == new_direction:
        return move_cost

    return move_cost + bend_cost

# 计算弯曲数量
def count_bends(state_path):
    if not state_path:
        return 0

    bend_count = 0
    previous_direction = None

    for _, _, direction in state_path:
        if direction is None:
            continue

        if (
            previous_direction is not None
            and direction != previous_direction
        ):
            bend_count += 1

        previous_direction = direction

    return bend_count

# 去掉状态中的方向，使print_grid正常可用于方向A*
def states_to_positions(state_path):
    if state_path is None:
        return None

    return [
        (x, y)
        for x, y, direction in state_path
    ]

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
