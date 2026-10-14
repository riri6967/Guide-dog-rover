#!/usr/bin/env python3

import math


WAYPOINT_TOLERANCE = 0.35

# Known geometry of the simulated environment
ROOM1_DOORWAY = (-5.0, 1.5)
ROOM2_DOORWAY = (5.0, -1.5)

ROOM1_STAGING = (-5.0, 0.6)
ROOM1_CROSSING = (-5.0, 2.2)

ROOM2_STAGING = (5.0, -0.6)
ROOM2_CROSSING = (5.0, -2.2)


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def generate_room_route(room, goal):
    """
    Generate a safe hierarchical route from the corridor
    into a room and finally to the requested goal.

    Parameters
    ----------
    room : str
        "room1" or "room2"
    goal : tuple[float, float]
        Final (x, y) goal in odom coordinates.

    Returns
    -------
    list[tuple[float, float]]
        Generated waypoint sequence.
    """

    goal = (float(goal[0]), float(goal[1]))

    if room.lower() == "room1":
        staging = ROOM1_STAGING
        crossing = ROOM1_CROSSING

    elif room.lower() == "room2":
        staging = ROOM2_STAGING
        crossing = ROOM2_CROSSING

    else:
        raise ValueError(f"Unknown room: {room}")

    route = []

    # Approach doorway
    route.append(staging)

    # Cross doorway
    route.append(crossing)

    # Final room goal
    if distance(crossing, goal) > WAYPOINT_TOLERANCE:
        route.append(goal)

    return route


if __name__ == "__main__":
    route = generate_room_route("room1", (-5.0, 3.0))

    print("Generated Room 1 route:")
    for i, waypoint in enumerate(route, start=1):
        print(f"  {i}: ({waypoint[0]:.2f}, {waypoint[1]:.2f})")
