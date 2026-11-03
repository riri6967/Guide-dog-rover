#!/usr/bin/env python3
"""V9 global occupancy-map planner.

The planner consumes collision geometry only.  It is deliberately blind to
semantic labels at the SAC-policy interface: the output is an occupancy grid
and a geometric path/local subgoal.

Important SDF detail:
world-model pose -> link pose -> collision pose.
The previous version only composed model+collision poses, which can collapse
many SDF objects onto the world origin when the SDF stores placement on the
link.  V9.1 composes the full 2-D transform chain.
"""

import heapq
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

WORLD_PATH = (
    Path(__file__).resolve().parent.parent / "worlds" / "assistive_world.sdf"
)


def _pose_xyyaw(element):
    if element is None:
        return 0.0, 0.0, 0.0
    pose = element.find("pose")
    if pose is None or not pose.text:
        return 0.0, 0.0, 0.0
    vals = [float(v) for v in pose.text.split()[:6]]
    vals += [0.0] * (6 - len(vals))
    return vals[0], vals[1], vals[5]


def _compose_pose(a, b):
    """Compose 2-D pose a * b."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c = math.cos(ayaw)
    s = math.sin(ayaw)
    return (
        ax + c * bx - s * by,
        ay + s * bx + c * by,
        ayaw + byaw,
    )


class StructuralOccupancyMap:
    """2-D occupancy map derived from static SDF collision geometry.

    Static furniture is allowed in the map when it is actually part of the
    world. Dynamic objects are not magically known to this planner; they are
    handled by LiDAR/SAC and the safety shield.
    """

    def __init__(
        self,
        world_path=WORLD_PATH,
        resolution=0.15,
        inflation_radius=0.55,
        bounds=(-13.0, 13.0, -8.0, 8.0),
    ):
        self.world_path = Path(world_path) if world_path is not None else WORLD_PATH
        self.resolution = float(resolution)
        self.inflation_radius = float(inflation_radius)
        self.xmin, self.xmax, self.ymin, self.ymax = map(float, bounds)

        self.nx = int(math.ceil((self.xmax - self.xmin) / self.resolution)) + 1
        self.ny = int(math.ceil((self.ymax - self.ymin) / self.resolution)) + 1
        self.grid = np.zeros((self.ny, self.nx), dtype=np.uint8)
        self.raw_boxes = []
        self.ignored_unsupported = 0
        self.static_models = 0
        self.collision_primitives = 0
        self._build()

    @staticmethod
    def _is_floor_like(model_name, size):
        """Reject obvious support-surface collisions without rejecting long walls."""
        n = model_name.lower()
        if "floor" in n or "ground" in n:
            return True
        sx, sy, sz = size
        # A broad, thin slab is usually a support surface.  Do not classify a
        # long/thin wall as a floor merely because its XY area is large.
        return (
            sz <= 0.5
            and sx >= 15.0
            and sy >= 8.0
        )

    @staticmethod
    def _world_box_aabb(world_pose, size):
        wx, wy, yaw = world_pose
        sx, sy, _ = size
        corners = np.array(
            [
                [-sx / 2, -sy / 2],
                [-sx / 2,  sy / 2],
                [ sx / 2, -sy / 2],
                [ sx / 2,  sy / 2],
            ],
            dtype=np.float64,
        )
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s], [s, c]], dtype=np.float64)
        wc = corners @ rot.T
        wc[:, 0] += wx
        wc[:, 1] += wy
        return (
            float(wc[:, 0].min()),
            float(wc[:, 0].max()),
            float(wc[:, 1].min()),
            float(wc[:, 1].max()),
        )

    @staticmethod
    def _circle_aabb(world_pose, radius):
        wx, wy, _ = world_pose
        r = float(radius)
        return wx - r, wx + r, wy - r, wy + r

    def _add_collision(self, model_name, world_pose, geometry):
        box = geometry.find("box")
        cyl = geometry.find("cylinder")
        sph = geometry.find("sphere")

        if box is not None and box.find("size") is not None:
            size = tuple(float(v) for v in box.find("size").text.split()[:3])
            if self._is_floor_like(model_name, size):
                return
            self.raw_boxes.append(self._world_box_aabb(world_pose, size))
            self.collision_primitives += 1
            return

        if cyl is not None and cyl.find("radius") is not None:
            if "floor" in model_name.lower() or "ground" in model_name.lower():
                return
            radius = float(cyl.findtext("radius"))
            self.raw_boxes.append(self._circle_aabb(world_pose, radius))
            self.collision_primitives += 1
            return

        if sph is not None and sph.find("radius") is not None:
            if "floor" in model_name.lower() or "ground" in model_name.lower():
                return
            radius = float(sph.findtext("radius"))
            self.raw_boxes.append(self._circle_aabb(world_pose, radius))
            self.collision_primitives += 1
            return

        self.ignored_unsupported += 1

    def _build(self):
        if not self.world_path.exists():
            raise FileNotFoundError(self.world_path)

        root = ET.parse(self.world_path).getroot()
        world = root.find("world")
        if world is None:
            raise RuntimeError("SDF <world> element not found.")

        # Static world models.  Compose model -> link -> collision poses.
        for model in world.findall("model"):
            static = (model.findtext("static") or "").strip().lower()
            if static not in ("true", "1"):
                continue

            self.static_models += 1
            model_pose = _pose_xyyaw(model)
            links = model.findall("link")

            if links:
                for link in links:
                    link_pose = _pose_xyyaw(link)
                    model_link_pose = _compose_pose(model_pose, link_pose)
                    for collision in link.findall("collision"):
                        collision_pose = _pose_xyyaw(collision)
                        world_collision_pose = _compose_pose(
                            model_link_pose,
                            collision_pose,
                        )
                        geometry = collision.find("geometry")
                        if geometry is not None:
                            self._add_collision(
                                model.get("name", ""),
                                world_collision_pose,
                                geometry,
                            )
            else:
                # Be tolerant of compact SDFs that put collision directly
                # under the model.
                for collision in model.findall("collision"):
                    collision_pose = _pose_xyyaw(collision)
                    world_collision_pose = _compose_pose(
                        model_pose,
                        collision_pose,
                    )
                    geometry = collision.find("geometry")
                    if geometry is not None:
                        self._add_collision(
                            model.get("name", ""),
                            world_collision_pose,
                            geometry,
                        )

        self._rasterize()

        print(
            f"[V9 MAP] resolution={self.resolution:.2f}m "
            f"grid={self.nx}x{self.ny} "
            f"bounds=({self.xmin:.1f},{self.xmax:.1f},"
            f"{self.ymin:.1f},{self.ymax:.1f})"
        )
        print(
            f"[V9 MAP] static models={self.static_models} "
            f"collision primitives={self.collision_primitives}"
        )
        print(
            f"[V9 MAP] occupied cells={int(self.grid.sum())} "
            f"ignored unsupported collisions={self.ignored_unsupported}"
        )

        # Critical sanity check: the known Gazebo training start must not be
        # swallowed by the map. Print the first boxes that contain it before
        # inflation so a malformed SDF transform is immediately obvious.
        offenders = []
        for i, (xmin, xmax, ymin, ymax) in enumerate(self.raw_boxes):
            if xmin <= 0.0 <= xmax and ymin <= 0.0 <= ymax:
                offenders.append((i, xmin, xmax, ymin, ymax))
        if offenders:
            print(f"[V9 MAP WARNING] raw collision boxes containing (0,0): {len(offenders)}")
            for item in offenders[:8]:
                i, xmin, xmax, ymin, ymax = item
                print(
                    f"  box#{i}: x=[{xmin:.2f},{xmax:.2f}] "
                    f"y=[{ymin:.2f},{ymax:.2f}]"
                )
        else:
            print("[V9 MAP] start sanity: (0,0) is not inside any raw collision box.")

    def _rasterize(self):
        pad = self.inflation_radius
        for xmin, xmax, ymin, ymax in self.raw_boxes:
            xmin -= pad
            xmax += pad
            ymin -= pad
            ymax += pad

            ix0 = max(0, int(math.floor((xmin - self.xmin) / self.resolution)))
            ix1 = min(self.nx - 1, int(math.ceil((xmax - self.xmin) / self.resolution)))
            iy0 = max(0, int(math.floor((ymin - self.ymin) / self.resolution)))
            iy1 = min(self.ny - 1, int(math.ceil((ymax - self.ymin) / self.resolution)))

            if ix0 <= ix1 and iy0 <= iy1:
                self.grid[iy0:iy1 + 1, ix0:ix1 + 1] = 1

    def world_to_grid(self, x, y):
        return (
            int(round((x - self.xmin) / self.resolution)),
            int(round((y - self.ymin) / self.resolution)),
        )

    def grid_to_world(self, ix, iy):
        return (
            self.xmin + ix * self.resolution,
            self.ymin + iy * self.resolution,
        )

    def in_bounds(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def is_free_world(self, x, y):
        ix, iy = self.world_to_grid(x, y)
        return self.in_bounds(ix, iy) and self.grid[iy, ix] == 0

    def nearest_free(self, x, y, max_radius=2.0):
        gx, gy = self.world_to_grid(x, y)
        max_cells = int(max_radius / self.resolution)
        if self.in_bounds(gx, gy) and self.grid[gy, gx] == 0:
            return gx, gy

        best = None
        best_d2 = float("inf")
        for dy in range(-max_cells, max_cells + 1):
            for dx in range(-max_cells, max_cells + 1):
                ix, iy = gx + dx, gy + dy
                if not self.in_bounds(ix, iy) or self.grid[iy, ix] != 0:
                    continue
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2 = d2
                    best = (ix, iy)
        if best is None:
            raise RuntimeError(f"No free cell near ({x:.2f}, {y:.2f}).")
        return best

    def segment_is_free(self, a, b):
        ax, ay = a
        bx, by = b
        dist = math.hypot(bx - ax, by - ay)
        steps = max(2, int(math.ceil(dist / (self.resolution * 0.5))))
        for t in np.linspace(0.0, 1.0, steps):
            x = ax + t * (bx - ax)
            y = ay + t * (by - ay)
            if not self.is_free_world(x, y):
                return False
        return True

    def plan(self, start, goal):
        sx, sy = self.nearest_free(*start)
        gx, gy = self.nearest_free(*goal)
        start_idx = (sx, sy)
        goal_idx = (gx, gy)

        pq = [(0.0, start_idx)]
        came_from = {start_idx: None}
        gscore = {start_idx: 0.0}
        neighbors = [
            (-1, -1, math.sqrt(2.0)), (-1, 0, 1.0), (-1, 1, math.sqrt(2.0)),
            (0, -1, 1.0), (0, 1, 1.0),
            (1, -1, math.sqrt(2.0)), (1, 0, 1.0), (1, 1, math.sqrt(2.0)),
        ]

        found = False
        while pq:
            _, cur = heapq.heappop(pq)
            if cur == goal_idx:
                found = True
                break

            cx, cy = cur
            for dx, dy, cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if not self.in_bounds(nx, ny) or self.grid[ny, nx]:
                    continue
                if dx != 0 and dy != 0:
                    if self.grid[cy, nx] or self.grid[ny, cx]:
                        continue

                nxt = (nx, ny)
                tentative = gscore[cur] + cost
                if tentative >= gscore.get(nxt, float("inf")):
                    continue

                came_from[nxt] = cur
                gscore[nxt] = tentative
                h = math.hypot(goal_idx[0] - nx, goal_idx[1] - ny)
                heapq.heappush(pq, (tentative + h, nxt))

        if not found:
            raise RuntimeError(f"No A* path from {start} to {goal}.")

        cells = []
        cur = goal_idx
        while cur is not None:
            cells.append(cur)
            cur = came_from[cur]
        cells.reverse()
        raw_path = [self.grid_to_world(ix, iy) for ix, iy in cells]

        simplified = [raw_path[0]]
        i = 0
        while i < len(raw_path) - 1:
            j = len(raw_path) - 1
            while j > i + 1 and not self.segment_is_free(raw_path[i], raw_path[j]):
                j -= 1
            simplified.append(raw_path[j])
            i = j

        total = sum(
            math.dist(simplified[k], simplified[k + 1])
            for k in range(len(simplified) - 1)
        )
        return simplified, total

    @staticmethod
    def _project_point_to_segment(px, py, ax, ay, bx, by):
        vx, vy = bx - ax, by - ay
        denom = vx * vx + vy * vy
        if denom < 1e-12:
            return 0.0, ax, ay
        t = ((px - ax) * vx + (py - ay) * vy) / denom
        t = max(0.0, min(1.0, t))
        return t, ax + t * vx, ay + t * vy

    def local_subgoal(self, path, robot_xy, lookahead=1.5, start_segment=0):
        if len(path) == 1:
            return path[0], 0.0, 0.0, 0

        px, py = robot_xy
        cumulative = [0.0]
        for i in range(len(path) - 1):
            cumulative.append(cumulative[-1] + math.dist(path[i], path[i + 1]))

        best_dist = float("inf")
        best_seg = max(0, min(start_segment, len(path) - 2))
        best_t = 0.0
        best_xy = path[best_seg]

        for i in range(best_seg, len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i + 1]
            t, qx, qy = self._project_point_to_segment(px, py, ax, ay, bx, by)
            d = math.hypot(px - qx, py - qy)
            if d < best_dist:
                best_dist = d
                best_seg = i
                best_t = t
                best_xy = (qx, qy)

        progress = cumulative[best_seg] + best_t * math.dist(path[best_seg], path[best_seg + 1])
        target_s = min(cumulative[-1], progress + float(lookahead))
        target = path[-1]
        for i in range(len(path) - 1):
            if cumulative[i] <= target_s <= cumulative[i + 1]:
                seg = cumulative[i + 1] - cumulative[i]
                t = 0.0 if seg < 1e-9 else (target_s - cumulative[i]) / seg
                target = (
                    path[i][0] + t * (path[i + 1][0] - path[i][0]),
                    path[i][1] + t * (path[i + 1][1] - path[i][1]),
                )
                break

        remaining = max(0.0, cumulative[-1] - progress)
        return target, progress, remaining, best_seg


def demo():
    planner = StructuralOccupancyMap()
    tests = [
        ((0.0, 0.0), (-5.0, 4.0)),
        ((0.0, 0.0), (5.0, -4.0)),
        ((0.0, 0.0), (-3.0, 0.0)),
        ((0.0, 0.0), (3.0, 4.0)),
    ]

    for start, goal in tests:
        print("\n" + "=" * 70)
        print(f"Planning {start} -> {goal}")
        try:
            path, length = planner.plan(start, goal)
            print(
                f"[V9 PLAN] points={len(path)} length={length:.2f}m"
            )
            for point in path[:8]:
                print(f"  {point}")
            if len(path) > 8:
                print("  ...")
            print("  Final:", path[-1])
        except Exception as exc:
            print("[V9 PLAN ERROR]", exc)


if __name__ == "__main__":
    demo()
