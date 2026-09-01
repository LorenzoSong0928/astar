import gdsfactory as gf
from pathlib import Path

OUTPUT_DIR = (
    Path(__file__).resolve().parent
    / "output"
)

OUTPUT_DIR.mkdir(
    parents=True,
    exist_ok=True
)

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

    output_path = OUTPUT_DIR / filename
    top.write_gds(output_path)

    return str(output_path)