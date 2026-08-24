"""负责：单条布线相关"""

import heapq
from grid import get_directional_neighbours

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