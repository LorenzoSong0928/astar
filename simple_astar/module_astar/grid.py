
# 候选的方向状态
DIRECTIONS = [
    (-1, 0, "L"),  # 左
    (1, 0, "R"),  # 右
    (0, -1, "U"),  # 上
    (0, 1, "D"),  # 下
]

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

# 记录冲突位置
def find_conflicts(path, grid):
    if path is None:
        return []

    conflicts = []

    for x, y in path:
        if grid[y][x] == 2:
            conflicts.append((x, y))

    return conflicts

# 返回当前路径穿过的已有网络编号。
def find_conflicting_net_ids(path, owner_grid):
    conflicting_net_ids = set()

    if path is None:
        return conflicting_net_ids

    for x, y in path:
        net_id = owner_grid[y][x]

        if net_id is not None:
            conflicting_net_ids.add(net_id)

    return conflicting_net_ids

# 登记网格
def register_path(grid, owner_grid, path, net_id):
    """注册路径，并记录每个格点所属的网络编号。"""
    if path is None:
        return

    for x, y in path:
        grid[y][x] = 2
        owner_grid[y][x] = net_id

# 移除网络
def remove_path(grid,owner_grid,path,net_id):
    """拆除指定编号网络已经注册的路径。"""
    if path is None:
        return

    for x, y in path:
        if owner_grid[y][x] == net_id:
            grid[y][x] = 0
            owner_grid[y][x] = None

# 打印网格
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