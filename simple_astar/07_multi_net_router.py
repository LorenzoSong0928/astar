import heapq

# 候选的方向状态
DIRECTIONS = [
    (-1, 0, "L"),  # 左
    (1, 0, "R"),  # 右
    (0, -1, "U"),  # 上
    (0, 1, "D"),  # 下
]

# ==================== 1. 地图与网络配置 ====================
WIDTH = 25
HEIGHT = 15

grid = [[0] * WIDTH for _ in range(HEIGHT)]

nets = [
    {
        "name": "horizontal",
        "start": (0, 7),
        "goal": (24, 7),
        "start_direction": "R",
        "goal_entry_direction": "R",
    },
    {
        "name": "vertical",
        "start": (12, 0),
        "goal": (12, 14),
        "start_direction": "D",
        "goal_entry_direction": "D",
    },
]

# ==================== 2. 合法性检查与邻居生成 ====================
def get_directional_neighbours(
    state,
    grid,
    clearance=0,
    strict=True
):
    x, y, current_direction = state
    neighbours = []

    for dx, dy, new_direction in DIRECTIONS:
        new_x = x + dx
        new_y = y + dy

        if is_available(
            new_x,
            new_y,
            grid,
            clearance,
            strict
        ):
            neighbours.append(
                (new_x, new_y, new_direction)
            )

    return neighbours

# 合法性判断：clearance表示线与障碍物之间需要空出的间隙
def is_available(x, y, grid, clearance=0, strict=True):
    height = len(grid)
    width = len(grid[0])

    if not (0 <= x < width and 0 <= y < height):
        return False

    for check_y in range(y - clearance, y + clearance + 1):
        for check_x in range(x - clearance, x + clearance + 1):

            if not (
                0 <= check_x < width
                and 0 <= check_y < height
            ):
                continue

            cell = grid[check_y][check_x]

            # 固定障碍始终禁止通过
            if cell == 1:
                return False

            # 严格模式下，已有线路也禁止通过
            if strict and cell == 2:
                return False

    return True

# ==================== 3. 代价函数与启发函数 ====================
def heuristic(pos, goal):
    x1, y1 = pos
    x2, y2 = goal

    return abs(x1 - x2) + abs(y1 - y2)

# ==================== 4. 带方向约束的 A* 搜索 ====================
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

# ==================== 5. 路径处理与统计 ====================
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

# 记录冲突位置
def find_conflicts(path, grid):
    if path is None:
        return []

    conflicts = []

    for x, y in path:
        if grid[y][x] == 2:
            conflicts.append((x, y))

    return conflicts

# ==================== 6. 网格显示与线路注册 ====================
def print_grid(grid, path=None, start=None, goal=None):
    path_set = set(path) if path else set()

    width = len(grid[0])
    display_width = width * 2 - 1
    title = "print_grid"

    left_dash = (display_width - len(title)) // 2
    right_dash = display_width - len(title) - left_dash

    print(
        "\n|"
        + "-" * left_dash
        + title
        + "-" * right_dash
        + "|"
    )

    for y in range(len(grid)):
        row = []

        for x in range(width):
            pos = (x, y)
            if pos == start:
                row.append("S")
            elif pos == goal:
                row.append("G")
            elif pos in path_set and grid[y][x] == 2:
                row.append("X")  # 当前路径与已有线路冲突
            elif pos in path_set:
                row.append("*")
            elif grid[y][x] == 1:
                row.append("#")
            elif grid[y][x] == 2:
                row.append("=")
            else:
                row.append(".")
        print(" ".join(row))
    print("|" + "-" * display_width + "|\n")

# 去掉状态中的方向，使print_grid正常可用于方向A*
def states_to_positions(state_path):
    if state_path is None:
        return None

    return [
        (x, y)
        for x, y, direction in state_path
    ]

# 登记网络
def register_path(grid, path, start=None, goal=None, value=2):
    if path is None:
        return

    for x, y in path:
        """pos = (x, y)

        if pos == start or pos == goal:
            continue
        """
        grid[y][x] = value

# ==================== 7. 单网络布线封装 ====================
def route_net(
    grid,
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
        "name": net["name"],
        "success": goal_state is not None,
        "path": path,
        "state_path": state_path,
        "goal_state": goal_state,
        "conflicts": conflicts,
        "visited_count": len(visited),
        "cost": cost.get(goal_state),
    }

    return result

# ==================== 8. 多网络依次布线 ====================
results = []

for net in nets:
    result = route_net(
        grid,
        net,
        strict=True,
        conflict_penalty=50,
        bend_cost=5,
        clearance=0
    )

    print(f"\n正在布线：{result['name']}")
    print("是否成功：", result["success"])

    if result["success"]:
        print("总代价：", result["cost"])
        print("移动步数：", len(result["path"]) - 1)
        print("转弯次数：", count_bends(result["state_path"]))
        print("冲突数量：", len(result["conflicts"]))

        # 先显示：此时 grid 中只有之前的线路
        print_grid(
            grid,
            result["path"],
            net["start"],
            net["goal"]
        )

        # 再注册：供后面的网络使用
        register_path(
            grid,
            result["path"],
            net["start"],
            net["goal"]
        )
    else:
        print("严格模式下未找到路径")
        print_grid(
            grid,
            None,
            net["start"],
            net["goal"]
        )