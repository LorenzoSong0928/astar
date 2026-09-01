from shapely.geometry import box

def grid_to_physical(
    x,
    y,
    grid_height,
    grid_size=10,
):
    """
    将 grid 坐标转换成 physical 坐标。
    physical 坐标对应 grid cell 的中心。
    """

    x_phys = x * grid_size

    y_phys = (
        grid_height - 1 - y
    ) * grid_size

    return x_phys, y_phys

def grid_obstacles_to_polygons(
    grid,
    grid_size=10,
):
    """
    将 grid[y][x] == 1 的障碍格
    转换成 Shapely rectangle polygons。
    """

    grid_height = len(grid)

    obstacles = []

    half_size = grid_size / 2

    for y in range(len(grid)):

        for x in range(len(grid[0])):

            if grid[y][x] != 1:
                continue

            x_phys, y_phys = grid_to_physical(
                x,
                y,
                grid_height,
                grid_size
            )

            obstacle = box(
                x_phys - half_size,
                y_phys - half_size,
                x_phys + half_size,
                y_phys + half_size,
            )

            obstacles.append(obstacle)

    return obstacles
# 边界转换
def grid_to_boundary_polygon(
    grid,
    grid_size=10,
):
    """
    将整个 grid 的有效区域转换成
    Shapely routing boundary。
    """

    grid_height = len(grid)
    grid_width = len(grid[0])

    half_size = grid_size / 2

    min_x = -half_size
    min_y = -half_size

    max_x = (
        (grid_width - 1) * grid_size
        + half_size
    )

    max_y = (
        (grid_height - 1) * grid_size
        + half_size
    )

    return box(
        min_x,
        min_y,
        max_x,
        max_y
    )

