import gdsfactory as gf

# =====================================
# Legacy GDS Export
# =====================================
def simplify_path(path):
    """删除连续共线的中间点，只保留起点、拐点和终点。"""

    if path is None or len(path) <= 2:
        return path

    simplified = [path[0]]

    for i in range(1, len(path) - 1):
        prev_x, prev_y = path[i - 1]
        cur_x, cur_y = path[i]
        next_x, next_y = path[i + 1]

        direction1 = (
            cur_x - prev_x,
            cur_y - prev_y
        )

        direction2 = (
            next_x - cur_x,
            next_y - cur_y
        )

        if direction1 != direction2:
            simplified.append(path[i])

    simplified.append(path[-1])

    return simplified

def grid_to_um(path, grid_size=10, grid_height=None):
    if grid_height is None:
        raise ValueError("grid_height must be provided")

    return [
        (
            x * grid_size,
            (grid_height - 1 - y) * grid_size
        )
        for x, y in path
    ]

def path_to_waveguide(
    points,
    width=0.5,
    layer=(1, 0),
    bend_radius=5
):
    """将物理坐标关键点转换为平滑波导 Component。"""

    path = gf.path.smooth(
        points=points,
        radius=bend_radius,
        bend=gf.path.euler,
        use_eff=False,
    )

    waveguide = gf.path.extrude(
        path,
        width=width,
        layer=layer,
    )

    return waveguide

def export_routed_paths(
    routed_paths,
    filename="astar_router.gds",
    grid_size=10,
    width=0.5,
    layer=(1, 0),
    bend_radius=5,
    grid_height=None
):
    """将所有已布网络导出到同一个 GDS。"""

    top = gf.Component("astar_router")

    for net_id, path in routed_paths.items():

        simplified = simplify_path(path)

        physical_path = grid_to_um(
            simplified,
            grid_size=grid_size,
            grid_height=grid_height
        )

        waveguide = path_to_waveguide(
            physical_path,
            width=width,
            layer=layer,
            bend_radius=bend_radius
        )

        top.add_ref(waveguide)

    filepath = top.write_gds(filename)

    return filepath

# =====================================
# Geometry-aware GDS Export
# =====================================
def export_routed_geometries(
    routed_geometries,
    filename="curvy_router.gds",
    layer=(1, 0),
):
    """
    直接将已经通过 DRC 并注册的真实 Shapely geometry
    输出到 GDS。

    不再重新 smooth，也不重新生成 Euler bend。
    """

    top = gf.Component("curvy_router")

    for net_id, geometry in routed_geometries.items():

        if geometry is None:
            continue

        # -------------------------
        # Polygon
        # -------------------------

        if geometry.geom_type == "Polygon":

            points = list(
                geometry.exterior.coords
            )

            top.add_polygon(
                points,
                layer=layer,
            )

        # -------------------------
        # MultiPolygon
        # -------------------------

        elif geometry.geom_type == "MultiPolygon":

            for polygon in geometry.geoms:

                points = list(
                    polygon.exterior.coords
                )

                top.add_polygon(
                    points,
                    layer=layer,
                )

        else:
            raise ValueError(
                f"Unsupported geometry type: "
                f"{geometry.geom_type}"
            )

    filepath = top.write_gds(filename)

    return filepath