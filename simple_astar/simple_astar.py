MAX_RANGE = 10

grid = [[0] * MAX_RANGE for _ in range(MAX_RANGE)]

for row in grid: print(*row)

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