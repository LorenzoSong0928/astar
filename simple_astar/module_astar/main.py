from router import route_all_nets
from gds_export import export_routed_paths
from simple_astar.module_astar.grid import print_grid

# ============================================================
# 综合实验地图
#
# 左上 A：A* / bend_cost
# 右上 B：clearance / simplified DRC
# 左下 C：multi-net / rip-up & reroute
# 右下 D：multi-waveguide / GDS
# ============================================================

WIDTH = 56
HEIGHT = 34


def add_rect(grid, x1, y1, x2, y2):
    """添加矩形固定障碍。"""
    for y in range(y1, y2 + 1):
        for x in range(x1, x2 + 1):
            grid[y][x] = 1


def build_demo_map():
    # 1. 初始化空地图
    grid = [
        [0] * WIDTH
        for _ in range(HEIGHT)
    ]

    # 2. 外边界
    for x in range(WIDTH):
        grid[0][x] = 1
        grid[HEIGHT - 1][x] = 1

    for y in range(HEIGHT):
        grid[y][0] = 1
        grid[y][WIDTH - 1] = 1

    # 3. 将地图划分成四个独立实验区
    #
    #       A        |        B
    #                |
    #  --------------+--------------
    #                |
    #       C        |        D

    # 竖向隔离墙
    for y in range(HEIGHT):
        grid[y][27] = 1

    # 横向隔离墙
    for x in range(WIDTH):
        grid[16][x] = 1

    # Zone A：A* + bend_cost 实验
    # ========================================================
    #
    # 设计：
    # 中间存在一条较短但转弯很多的“蛇形路径”
    # 上下存在稍长但转弯较少的绕行路径
    #
    # bend_cost 小：
    #     倾向短路径 + 多转弯
    #
    # bend_cost 大：
    #     倾向长路径 + 少转弯
    #
    # ========================================================

    bend_walls = [
        (7, 7),
        (11, 9),
        (15, 7),
        (19, 9),
        (23, 7),
    ]

    for wall_x, gap_y in bend_walls:

        for y in range(3, 14):

            if y != gap_y:
                grid[y][wall_x] = 1

    # ========================================================
    # Zone B：clearance / simplified DRC 实验
    # ========================================================
    #
    # 每面墙有：
    #
    # y = 8：
    #     单格窄通道
    #
    # y = 3~5：
    #     三格宽通道
    #
    # clearance = 0：
    #     直接穿过中间窄通道
    #
    # clearance = 1：
    #     窄通道非法，被迫走上面的宽通道
    #
    # clearance = 2：
    #     无法通过
    #
    # ========================================================

    for wall_x in [37, 45]:

        for y in range(2, 15):

            # 上部宽通道
            if y in [3, 4, 5]:
                continue

            # 中间窄通道
            if y == 8:
                continue

            grid[y][wall_x] = 1

    # ========================================================
    # Zone C：Multi-Net + Rip-up & Reroute
    # ========================================================
    #
    # 这里不放固定障碍。
    #
    # net C0 首先横向占满整个区域。
    #
    # 随后 C1 从上向下：
    #     strict routing 失败
    #
    # non-strict：
    #     穿过 C0
    #     → 找出 conflict net
    #
    # rip-up C0
    #
    # C1 先成功
    #
    # C0 再从 C1 上方/下方绕行
    #
    # 最后再加入 C2，形成更多网络。
    #
    # ========================================================

    # Zone C 本身保持为空
    # 冲突完全由 routed nets 制造

    # ========================================================
    # Zone D：GDS 展示区
    # ========================================================
    #
    # 三条平行 net
    # 每条分别遇到一个固定矩形障碍
    #
    # 最终 GDS 会产生三条不同位置的 Euler 绕行波导
    #
    # ========================================================

    # 上层障碍
    add_rect(
        grid,
        35, 18,
        39, 20
    )

    # 中层障碍
    add_rect(
        grid,
        42, 23,
        46, 25
    )

    # 下层障碍
    add_rect(
        grid,
        35, 28,
        39, 30
    )

    # ========================================================
    # 4. Nets
    # ========================================================

    nets = [

        # ----------------------------------------------------
        # Zone A
        # bend_cost experiment
        # ----------------------------------------------------
        {
            "name": "A_bend",
            "start": (2, 8),
            "goal": (25, 8),
            "start_direction": "R",
            "goal_entry_direction": "R",
        },

        # ----------------------------------------------------
        # Zone B
        # clearance experiment
        # ----------------------------------------------------
        {
            "name": "B_clearance",
            "start": (29, 8),
            "goal": (53, 8),
            "start_direction": "R",
            "goal_entry_direction": "R",
        },

        # ----------------------------------------------------
        # Zone C
        # Rip-up & Reroute
        #
        # 注意顺序：
        # C_barrier 必须先布
        # ----------------------------------------------------

        {
            "name": "C_barrier",
            "start": (1, 24),
            "goal": (26, 24),
            "start_direction": "R",
            "goal_entry_direction": "R",
        },

        {
            "name": "C_vertical_1",
            "start": (10, 19),
            "goal": (10, 29),
            "start_direction": "D",
            "goal_entry_direction": "D",
        },

        {
            "name": "C_vertical_2",
            "start": (18, 19),
            "goal": (18, 29),
            "start_direction": "D",
            "goal_entry_direction": "D",
        },

        # ----------------------------------------------------
        # Zone D
        # GDS showcase
        # ----------------------------------------------------

        {
            "name": "D_upper",
            "start": (29, 19),
            "goal": (53, 19),
            "start_direction": "R",
            "goal_entry_direction": "R",
        },

        {
            "name": "D_middle",
            "start": (29, 24),
            "goal": (53, 24),
            "start_direction": "R",
            "goal_entry_direction": "R",
        },

        {
            "name": "D_lower",
            "start": (29, 29),
            "goal": (53, 29),
            "start_direction": "R",
            "goal_entry_direction": "R",
        },
    ]

    return grid, nets

# 初始化
grid, nets = build_demo_map()

owner_grid = [
    [None] * WIDTH
    for _ in range(HEIGHT)
]

# 实验设计
EXPERIMENTS = {

    # Part 1
    "astar": [
        0
    ],

    # Part 1 中的 simplified DRC
    "clearance": [
        1
    ],

    # Part 2
    "ripup": [
        2,
        3,
        4
    ],

    # Part 3
    "gds": [
        5,
        6,
        7
    ],

    # 最终综合展示
    "all": list(range(8))
}
# 实验启动函数
def load_experiment(name):

    grid, all_nets = build_demo_map()

    net_ids = EXPERIMENTS[name]

    nets = [
        all_nets[i]
        for i in net_ids
    ]

    owner_grid = [
        [None] * WIDTH
        for _ in range(HEIGHT)
    ]

    return grid, owner_grid, nets
# ==================== 5. 启动实验 ====================
# 1.A* bend cost
"""grid, owner_grid, nets = load_experiment("astar")

routed_paths = route_all_nets(
    grid,
    owner_grid,
    nets,
    conflict_penalty=50,
    bend_cost=5,
    clearance=0
)
'''比较bend_cost=0和1'''

# 2.Clearance / DRC
grid, owner_grid, nets = load_experiment("clearance")

routed_paths = route_all_nets(
    grid,
    owner_grid,
    nets,
    conflict_penalty=50,
    bend_cost=5,
    clearance=2
)
'''分别跑clearance=012'''

# 3. Multi-net + Rip-up & Reroute
grid, owner_grid, nets = load_experiment("ripup")

routed_paths = route_all_nets(
    grid,
    owner_grid,
    nets,
    conflict_penalty=50,
    bend_cost=5,
    clearance=0
)
"""
"""
# 4.GDS
grid, owner_grid, nets = load_experiment("gds")

routed_paths = route_all_nets(
    grid,
    owner_grid,
    nets,
    conflict_penalty=50,
    bend_cost=5,
    clearance=0
)

export_routed_paths(
    routed_paths,
    filename="gds_demo.gds",
    grid_size=10,
    width=0.5,
    bend_radius=5,
    grid_height=HEIGHT
)
"""
# 5. 综合全部运行
grid, owner_grid, nets = load_experiment("all")

routed_paths = route_all_nets(
    grid,
    owner_grid,
    nets,
    conflict_penalty=50,
    bend_cost=5,
    clearance=0
)
