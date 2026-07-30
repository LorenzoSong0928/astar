import heapq
WIDTH = 15
HEIGHT = 11

grid = [[0] * WIDTH for _ in range(HEIGHT)]

start = (0, 5)
goal = (14, 5)

# 四面竖墙，中央缺口上下交替
wall_settings = [
    (3, 4),
    (6, 6),
    (9, 4),
    (12, 6),
]

for wall_x, gap_y in wall_settings:
    for y in range(1, HEIGHT - 1):
        if y != gap_y:
            grid[y][wall_x] = 1

# 2. 生成合法邻居
def get_neighbours(pos, grid):
    x, y = pos
    height = len(grid)
    width = len(grid[0])

    directions = [
        (-1, 0),  # 左
        (1, 0),   # 右
        (0, -1),  # 上
        (0, 1),   # 下
    ] # 后续8方向只需增加4个即可

    neighbours = []

    for dx, dy in directions:
        new_x = x + dx
        new_y = y + dy

        # 后续可以单独改成if_available函数判断：返回true false
        if (
            0 <= new_x < width
            and 0 <= new_y < height
            and grid[new_y][new_x] != 1
        ):
            neighbours.append((new_x, new_y))

    return neighbours

# print(get_neighbours((0, 0), grid))
# print(get_neighbours((1, 0), grid))

# 加上方向
def get_directional_neighbours(state, grid):
    x, y, current_direction = state

    height = len(grid)
    width = len(grid[0])

    DIRECTIONS = [
        (-1, 0, "L"),  # 左
        (1, 0, "R"),  # 右
        (0, -1, "U"),  # 上
        (0, 1, "D"),  # 下
    ]
    neighbours = []

    for dx, dy, new_direction in DIRECTIONS:
        new_x = x + dx
        new_y = y + dy

        if (
            0 <= new_x < width
            and 0 <= new_y < height
            and grid[new_y][new_x] != 1
        ):
            neighbours.append(
                (new_x, new_y, new_direction)
            )

    return neighbours

# 3. 曼哈顿距离
def heuristic(pos, goal):
    x1, y1 = pos
    x2, y2 = goal

    return abs(x1 - x2) + abs(y1 - y2)

# 4.1 Dijkstra 搜索
def dijkstra(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))

    cost = {start: 0}
    parent = {start: None}
    visited = set()

    while open_set:
        current_cost, current = heapq.heappop(open_set)

        if current in visited:
            continue

        visited.add(current)

        if current == goal:
            break

        for neighbour in get_neighbours(current,grid):
            step_cost = 1 # 还没有设置每个的权重

            new_cost = current_cost + step_cost

            if neighbour not in cost or new_cost < cost[neighbour]:
                cost[neighbour] = new_cost
                parent[neighbour] = current

                heapq.heappush(open_set,(cost[neighbour],neighbour))
    # 暂时先不回溯路径
    return parent, cost, visited

# 4.2 A*搜索
# 实际上，priority = cost + heuristic，保持原本cost不变，将加上曼哈顿距离进行排列优先级即可
def astar(grid, start, goal):
    open_set = []

    start_priority = heuristic(start, goal)
    heapq.heappush(open_set, (start_priority, start))

    cost = {start: 0}
    parent = {start: None}
    visited = set()

    while open_set:
        current_priority, current = heapq.heappop(open_set)

        if current in visited:
            continue

        visited.add(current)

        if current == goal:
            break

        for neighbour in get_neighbours(current, grid):
            new_cost = cost[current] + 1

            if neighbour not in cost or new_cost < cost[neighbour]:
                cost[neighbour] = new_cost
                parent[neighbour] = current

                priority = new_cost + heuristic(neighbour, goal)
                heapq.heappush(open_set, (priority, neighbour))

    return parent, cost, visited
# 方向A*
def directional_astar(grid, start, goal, bend_cost=5):
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

        # 只要位置到达目标即可
        if (current_x, current_y) == goal:
            goal_state = current_state
            break

        for neighbour_state in get_directional_neighbours(
            current_state,
            grid
        ):
            new_x, new_y, new_direction = neighbour_state

            step_cost = get_step_cost(
                current_direction,
                new_direction,
                bend_cost
            )

            new_cost = cost[current_state] + step_cost

            if (
                neighbour_state not in cost
                or new_cost < cost[neighbour_state]
            ):
                cost[neighbour_state] = new_cost
                parent[neighbour_state] = current_state

                h = heuristic((new_x, new_y), goal)
                priority = new_cost + h

                heapq.heappush(
                    open_set,
                    (priority, neighbour_state)
                )

    return parent, cost, visited, goal_state

# 5. 回溯路径
def backtrack(parent, start, goal):
    # goal 不在 parent 中，说明没有找到路径
    if goal not in parent:
        return None

    path = []
    current = goal

    # 从终点不断寻找父节点
    while current is not None:
        path.append(current)
        current = parent[current]

    # 当前得到的是 goal -> start，需要反转
    path.reverse()
    return path

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

# 6.判断转弯cost
def get_step_cost(current_direction, new_direction, bend_cost=5):
    move_cost = 1

    if current_direction is None:
        return move_cost

    if current_direction == new_direction:
        return move_cost

    return move_cost + bend_cost

# 展示函数
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
            elif pos in path_set:
                row.append("*")
            elif grid[y][x] == 1:
                row.append("#")
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

#计算弯曲数量
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

# 7. 测试
for bend_cost in [0, 1, 5, 20]:
    parent, cost, visited, goal_state = directional_astar(
        grid,
        start,
        goal,
        bend_cost=bend_cost
    )

    state_path = backtrack_directional(parent, goal_state)
    path = states_to_positions(state_path)

    print(f"\n转弯惩罚：{bend_cost}")
    print("移动步数：", len(state_path) - 1)
    print("转弯次数：", count_bends(state_path))
    print("总代价：", cost.get(goal_state))

    print_grid(grid, path, start, goal)