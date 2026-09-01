import gdsfactory as gf

from .geometry import create_base_bend
from .router import route_net_curvy_with_rr
from .gds_export import export_routed_geometries


# ============================================================
# Parameters
# ============================================================

GRID_SIZE = 10
CONFLICT_PENALTY = 1

WIDTH = 8
HEIGHT = 8


# ============================================================
# Build Routing Problem
# ============================================================

def build_routing_problem():

    # -------------------------------------
    # Grid
    # 0 = free
    # 1 = fixed obstacle
    # -------------------------------------

    grid = [
        [0] * WIDTH
        for _ in range(HEIGHT)
    ]

    # -------------------------------------
    # Nets
    # -------------------------------------

    nets = {

        0: {
            "name": "net_0",
            "start": (0, 4),
            "goal": (7, 4),
            "start_direction": "R",
            "goal_entry_direction": "R",
        },

        1: {
            "name": "net_1",
            "start": (3, 6),
            "goal": (3, 1),
            "start_direction": "U",
            "goal_entry_direction": "U",
        },
    }

    return grid, nets


# ============================================================
# Main Routing Flow
# ============================================================

def main():

    # -------------------------------------
    # 0. Activate generic PDK
    # -------------------------------------

    gf.gpdk.PDK.activate()

    # -------------------------------------
    # 1. Build routing problem
    # -------------------------------------

    grid, nets = build_routing_problem()

    # -------------------------------------
    # 2. Create base Euler bend
    # -------------------------------------

    bend_component, bend_polygon = (
        create_base_bend()
    )

    bend_length = bend_component.info[
        "length"
    ]

    print(
        "Euler bend length:",
        bend_length
    )

    # -------------------------------------
    # 3. Routing database
    # -------------------------------------

    routed_geometries = {}

    results = {}

    # -------------------------------------
    # 4. Route nets sequentially
    # -------------------------------------

    for net_id, net in nets.items():

        print("\n==============================")
        print(
            f" Routing {net['name']} "
        )
        print("==============================")

        result = route_net_curvy_with_rr(
            grid=grid,
            nets=nets,
            current_net_id=net_id,
            bend_polygon=bend_polygon,
            bend_length=bend_length,
            routed_geometries=routed_geometries,
            grid_size=GRID_SIZE,
            conflict_penalty=CONFLICT_PENALTY,
        )

        results[net_id] = result

        print(
            "success:",
            result["success"]
        )

        print(
            "routing mode:",
            result["routing_mode"]
        )

        if result["success"]:

            print(
                "cost:",
                result[
                    "current_result"
                ]["cost"]
            )

            print(
                "registered net ids:",
                sorted(
                    routed_geometries.keys()
                )
            )

        else:

            print(
                f"Routing failed: net {net_id}"
            )

            break

    # -------------------------------------
    # 5. Export final geometry
    # -------------------------------------

    if routed_geometries:

        filepath = export_routed_geometries(
            routed_geometries,
            filename="curvy_router_result.gds",
        )

        print("\n==============================")
        print(" Routing Finished ")
        print("==============================")

        print(
            "Final routed net ids:",
            sorted(
                routed_geometries.keys()
            )
        )

        print(
            "GDS exported:",
            filepath
        )


if __name__ == "__main__":
    main()