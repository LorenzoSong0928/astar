import heapq

MAX_RANGE = 10

# 1. 创建网格
grid = [[0] * MAX_RANGE for _ in range(MAX_RANGE)]

# 2. 设置障碍、起点和终点
grid[1][1] = 1
grid[1][2] = 1
grid[1][3] = 1

start = (0, 0)
goal = (5, 3)
# for row in grid: print(*row)

# 3. 生成合法邻居
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

print(get_neighbours((0, 0), grid))
print(get_neighbours((1, 0), grid))

# 4. Dijkstra 搜索
def dijkstra(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))

    cost = {start: 0}
    parent = {start: None}
    visited = set()

    while open_set:
        current_cost, current = heapq.heappop(open_set)

        # TODO 1：如果 current 已处理过，跳过
        if current in visited:
            continue
        # TODO 2：将 current 加入 visited
        visited.add(current)
        # TODO 3：如果 current 是 goal，结束循环
        if current == goal:
            break
        # TODO 4：遍历 get_neighbours(current, grid)
        for neighbour in get_neighbours(current,grid):
            step_cost = 1 # 还没有设置每个的权重
        # TODO 5：计算 new_cost
            new_cost = current_cost + step_cost
        # TODO 6：判断是否需要更新 cost 和 parent
            if neighbour not in cost or new_cost < cost[neighbour]:
                cost[neighbour] = new_cost
                parent[neighbour] = current
        # TODO 7：把更新后的邻居放入 open_set
                heapq.heappush(open_set,(cost[neighbour],neighbour))
    # 暂时先不回溯路径
    return parent, cost

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

# 展示函数
def print_grid(grid, path=None, start=None, goal=None):
    path_set = set(path) if path else set()
    print("\n-----print_grid-----")
    for y in range(len(grid)):
        row = []

        for x in range(len(grid[0])):
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
    print("-------------------\n")

# 6. 测试
parent, cost = dijkstra(grid, start, goal)
path = backtrack(parent, start, goal)
print_grid(grid, path, start, goal)

print("终点代价：", cost.get(goal))
print("路径：", path)
