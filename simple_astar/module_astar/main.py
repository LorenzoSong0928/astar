from router import route_all_nets
# ==================== 1. 地图配置 ====================

WIDTH = 25
HEIGHT = 15

grid = [
    [0] * WIDTH
    for _ in range(HEIGHT)
]

owner_grid = [
    [None] * WIDTH
    for _ in range(HEIGHT)
]


# ==================== 2. 网络配置 ====================

nets = [
    {
        "start": (0, 7),
        "goal": (24, 7),
        "start_direction": "R",
        "goal_entry_direction": "R",
    },
    {
        "start": (12, 2),
        "goal": (12, 12),
        "start_direction": "D",
        "goal_entry_direction": "D",
    },
]


# ==================== 3. 启动布线 ====================

routed_paths = route_all_nets(
    grid,
    owner_grid,
    nets,
    conflict_penalty=50,
    bend_cost=5,
    clearance=0
)

print("\n最终布线路径：")
print(routed_paths)
