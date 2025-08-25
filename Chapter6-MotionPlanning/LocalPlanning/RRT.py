import math
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from matplotlib.collections import LineCollection

# ---------------------------- Grid Utilities ---------------------------- #

def create_occupancy_grid(width, height, obstacle_prob, seed):
    """
    Returns a (height x width) uint8 grid where 0 = free, 1 = obstacle.
    """
    rng = np.random.default_rng(seed)
    grid = (rng.random((height, width)) < obstacle_prob).astype(np.uint8)
    return grid

def in_bounds(p, h, w):
    y, x = p
    return 0 <= y < h and 0 <= x < w

def cell_is_free(grid, p):
    y, x = int(round(p[0])), int(round(p[1]))
    h, w = grid.shape
    if 0 <= y < h and 0 <= x < w:
        return grid[y, x] == 0
    return False

# ---------------------------- RRT Core ---------------------------- #

def sample_free(grid):
    """Uniform sample a free point in continuous grid coordinates (y, x)."""
    h, w = grid.shape
    # Try a few times to land in free space; fallback to random until we do.
    for _ in range(1000):
        y = random.uniform(0, h - 1)
        x = random.uniform(0, w - 1)
        if cell_is_free(grid, (y, x)):
            return (y, x)
    # Extremely unlikely fallback: return center
    return (h/2.0, w/2.0)

def nearest_node(nodes, p_rand):
    """Return index of nearest node in nodes to p_rand (Euclidean)."""
    arr = np.asarray(nodes)
    dy = arr[:, 0] - p_rand[0]
    dx = arr[:, 1] - p_rand[1]
    d2 = dy*dy + dx*dx
    return int(np.argmin(d2))

def steer(p_near, p_rand, step_size):
    """Step from p_near towards p_rand by at most step_size."""
    y0, x0 = p_near
    y1, x1 = p_rand
    dy, dx = (y1 - y0), (x1 - x0)
    dist = math.hypot(dy, dx)
    if dist <= 1e-9:
        return p_near
    scale = min(1.0, step_size / dist)
    return (y0 + dy * scale, x0 + dx * scale)

def collision_free(grid, p0, p1, resolution=0.25):
    """
    Check segment p0->p1 for collision by sampling points every 'resolution' units.
    Coordinates are (y, x) in continuous space, mapped to nearest cell indices.
    """
    y0, x0 = p0
    y1, x1 = p1
    dist = math.hypot(y1 - y0, x1 - x0)
    n = max(1, int(dist / resolution))
    for i in range(n + 1):
        t = i / n
        y = y0 * (1 - t) + y1 * t
        x = x0 * (1 - t) + x1 * t
        if not cell_is_free(grid, (y, x)):
            return False
    return True

def reconstruct_path(nodes, parents, goal_idx):
    """Return list of (row, col) = (y, x) from start to goal_idx."""
    path = []
    cur = goal_idx
    while cur != -1:
        path.append((nodes[cur][0], nodes[cur][1]))  # (y, x) floats
        cur = parents[cur]
    path.reverse()
    return path

# ---------------------------- RRT Generator ---------------------------- #

def rrt_search_generator(
    grid,
    start, goal,
    step_size=1.0,
    goal_radius=1.0,
    goal_sample_rate=0.05,
    max_iters=6000
):
    """
    Step-by-step RRT generator.

    Emits:
      ("extend", nodes, edges, sample_point, near_point, new_point)
      ("found", path_list[(y,x)], nodes, edges)
      ("nopath", nodes, edges)
    nodes: list of (y, x)
    edges: list of ((y0, x0), (y1, x1))
    """
    h, w = grid.shape
    sy, sx = start
    gy, gx = goal
    if grid[sy, sx] == 1 or grid[gy, gx] == 1:
        raise ValueError("Start or goal inside an obstacle.")

    # Nodes/parents store continuous (y, x). Start at cell centers.
    start_c = (float(sy), float(sx))
    goal_c  = (float(gy), float(gx))

    nodes = [start_c]
    parents = [-1]  # parent index for each node
    edges = []

    for it in range(max_iters):
        # Goal bias sampling
        if random.random() < goal_sample_rate:
            p_rand = goal_c
        else:
            p_rand = sample_free(grid)

        # Find nearest
        idx_near = nearest_node(nodes, p_rand)
        p_near = nodes[idx_near]

        # Steer
        p_new = steer(p_near, p_rand, step_size)

        # Only add if collision-free and inside bounds and free
        if not in_bounds(p_new, h, w) or not cell_is_free(grid, p_new):
            # Still yield an attempt for visualization
            yield ("extend", nodes, edges, p_rand, p_near, None)
            continue

        if collision_free(grid, p_near, p_new):
            # Accept new node
            nodes.append(p_new)
            parents.append(idx_near)
            edges.append((p_near, p_new))

            # Visualize successful extension
            yield ("extend", nodes, edges, p_rand, p_near, p_new)

            # Check connect to goal
            if math.hypot(p_new[0] - goal_c[0], p_new[1] - goal_c[1]) <= goal_radius:
                # Ensure final connection is collision-free
                if collision_free(grid, p_new, goal_c):
                    # Add goal node
                    nodes.append(goal_c)
                    parents.append(len(nodes) - 2)  # parent is p_new
                    edges.append((p_new, goal_c))
                    path = reconstruct_path(nodes, parents, len(nodes) - 1)
                    yield ("found", path, nodes, edges)
                    return
        else:
            # Extension blocked; show attempt
            yield ("extend", nodes, edges, p_rand, p_near, None)

    # Not found
    yield ("nopath", nodes, edges)

# ---------------------------- Visualization Helpers ---------------------------- #

def make_base_grid_image(grid, start=None, goal=None):
    """
    Base categorical image for grid with start/goal overlays:
      0 = free, 1 = obstacle, 5 = start, 6 = goal
    (Other layers are drawn with artists, not inside the image.)
    """
    base = np.zeros_like(grid, dtype=np.uint8)  # 0 free
    base[grid == 1] = 1  # obstacle
    if start is not None:
        sy, sx = start
        base[sy, sx] = 5
    if goal is not None:
        gy, gx = goal
        base[gy, gx] = 6

    cmap = ListedColormap([
        "#FFFFFF",  # 0 free (white)
        "#000000",  # 1 obstacle (black)
        "#B0C4DE",  # 2 (unused here, reserved for "visited" in earlier demos)
        "#FFC107",  # 3 (unused here, we draw samples via markers)
        "#DC143C",  # 4 (unused here, new node via marker)
        "#1E90FF",  # 5 start (dodger blue)
        "#32CD32",  # 6 goal (lime green)
    ])
    return base, cmap

def interpolate_points(points, frames_per_cell=5):
    """Interpolate between (y, x) points for smoother follow animation."""
    if len(points) < 2:
        return np.array([(p[1], p[0]) for p in points], dtype=float)  # return (x,y)
    out = []
    pts = np.array(points, dtype=float)  # (y,x)
    for i in range(len(pts) - 1):
        p0, p1 = pts[i], pts[i + 1]
        length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        steps = max(1, int(frames_per_cell * max(1.0, length)))
        for t in np.linspace(0, 1, steps, endpoint=False):
            y = p0[0] * (1 - t) + p1[0] * t
            x = p0[1] * (1 - t) + p1[1] * t
            out.append((x, y))  # (x,y)
    out.append((pts[-1, 1], pts[-1, 0]))
    return np.array(out)

# ---------------------------- Animation ---------------------------- #

def animate_rrt(
    grid, start, goal,
    step_size=1.0,
    goal_radius=1.0,
    goal_sample_rate=0.05,
    search_interval=30,
    follow_interval=20,
    max_iters=6000
):
    """
    Runs RRT step-by-step and animates:
      1) tree growth (samples / new nodes / edges)
      2) path following once found
    """
    h, w = grid.shape
    gen = rrt_search_generator(
        grid, start, goal,
        step_size=step_size,
        goal_radius=goal_radius,
        goal_sample_rate=goal_sample_rate,
        max_iters=max_iters
    )

    # Figure setup
    fig, ax = plt.subplots(figsize=(w/4, h/4))
    ax.set_title("RRT (step-by-step) → Path Follow")
    ax.set_xticks(np.arange(-0.5, w, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, h, 1), minor=True)
    ax.grid(which='minor', linestyle=':', linewidth=0.5)
    ax.set_xlim(-0.5, w - 0.5)
    ax.set_ylim(h - 0.5, -0.5)

    # Base grid image with start/goal
    base_img, cmap = make_base_grid_image(grid, start=start, goal=goal)
    im = ax.imshow(base_img, cmap=cmap, origin='upper', vmin=0, vmax=6)

    # Tree artists
    tree_lines = LineCollection([], linewidths=1.2, colors="#4682B4", alpha=0.9)  # steel blue
    ax.add_collection(tree_lines)
    nodes_plot, = ax.plot([], [], linestyle='None', marker='.', markersize=3, color="#4682B4", alpha=0.9)

    # Sample + new node markers
    sample_marker, = ax.plot([], [], marker='x', markersize=6, linestyle='None', color="#FFC107")  # amber
    newnode_marker, = ax.plot([], [], marker='o', markersize=6, linestyle='None', color="#DC143C")  # crimson

    # Path + car artists
    path_line, = ax.plot([], [], lw=2, alpha=0.9, color="#8A2BE2")  # blueviolet
    car_marker, = ax.plot([], [], marker='o', markersize=8, color="#8A2BE2")

    # Legend (consistent with previous demos + RRT elements)
    legend_elements = [
        Patch(facecolor="#FFFFFF", edgecolor="k", label="Free"),
        Patch(facecolor="#000000", edgecolor="k", label="Obstacle"),
        Patch(facecolor="#1E90FF", edgecolor="k", label="Start"),
        Patch(facecolor="#32CD32", edgecolor="k", label="Goal"),
        Line2D([0], [0], color="#4682B4", lw=2, label="Tree"),
        Line2D([0], [0], marker='x', color="#FFC107", lw=0, label="Sample"),
        Line2D([0], [0], marker='o', color="#DC143C", lw=0, label="New node"),
        Line2D([0], [0], color="#8A2BE2", lw=2, label="Path / Car"),
    ]
    ax.legend(handles=legend_elements, bbox_to_anchor=(1.05, 1), loc='upper left')

    # Internal state for follow phase
    phase = {"mode": "search", "follow_points": None, "follow_idx": 0}

    # Buffers for artists
    segs = []        # list of ((x0,y0),(x1,y1)) in display coords
    node_x, node_y = [], []

    def update(_frame):
        nonlocal segs, node_x, node_y, phase
        if phase["mode"] == "search":
            try:
                item = next(gen)
            except StopIteration:
                return (im, tree_lines, nodes_plot, sample_marker, newnode_marker, path_line, car_marker)

            tag = item[0]
            if tag == "extend":
                _, nodes, edges, p_rand, p_near, p_new = item

                # Update tree lines/nodes
                # Convert edges from (y,x) to (x,y) tuples
                segs = [((e[0][1], e[0][0]), (e[1][1], e[1][0])) for e in edges]
                tree_lines.set_segments(segs)

                # Update nodes scatter
                node_x = [n[1] for n in nodes]
                node_y = [n[0] for n in nodes]
                nodes_plot.set_data(node_x, node_y)

                # Sample marker
                if p_rand is not None:
                    sample_marker.set_data([p_rand[1]], [p_rand[0]])
                else:
                    sample_marker.set_data([], [])

                # New node marker
                if p_new is not None:
                    newnode_marker.set_data([p_new[1]], [p_new[0]])
                else:
                    newnode_marker.set_data([], [])

                return (im, tree_lines, nodes_plot, sample_marker, newnode_marker)

            elif tag == "found":
                _, path, nodes, edges = item
                # Draw final tree state
                segs = [((e[0][1], e[0][0]), (e[1][1], e[1][0])) for e in edges]
                tree_lines.set_segments(segs)
                node_x = [n[1] for n in nodes]
                node_y = [n[0] for n in nodes]
                nodes_plot.set_data(node_x, node_y)

                # Draw path line (convert (y,x)->(x,y))
                py, px = zip(*path)
                path_line.set_data(px, py)

                # Prepare follow animation
                follow_pts = interpolate_points(path, frames_per_cell=5)  # (x,y)
                phase["mode"] = "follow"
                phase["follow_points"] = follow_pts
                phase["follow_idx"] = 0

                # Place car at start
                car_marker.set_data([follow_pts[0, 0]], [follow_pts[0, 1]])
                ax.set_title("RRT: Path found → Following")
                return (im, tree_lines, nodes_plot, sample_marker, newnode_marker, path_line, car_marker)

            elif tag == "nopath":
                _, nodes, edges = item
                segs = [((e[1], e[0]),) for e in []]  # no-op, keep as-is
                ax.set_title("RRT: No path found")
                return (im, tree_lines, nodes_plot, sample_marker, newnode_marker)

        elif phase["mode"] == "follow":
            pts = phase["follow_points"]
            i = phase["follow_idx"]
            if i < len(pts):
                x, y = pts[i]
                car_marker.set_data([x], [y])
                phase["follow_idx"] += 1
            return (im, tree_lines, nodes_plot, sample_marker, newnode_marker, path_line, car_marker)

    ani = FuncAnimation(fig, update, frames=10**9, interval=search_interval, blit=True)

    def on_draw(event):
        if phase["mode"] == "follow":
            ani.event_source.interval = follow_interval
    cid = fig.canvas.mpl_connect('draw_event', on_draw)

    plt.show()

# ---------------------------- Main ---------------------------- #

def main():
    # Same scenario as before
    W, H = 20, 20
    start = (H - 2, 1)   # (row, col) i.e., (y, x)
    goal  = (1, W - 2)

    grid = create_occupancy_grid(W, H, obstacle_prob=0.22, seed=7)
    grid[start] = 0
    grid[goal] = 0

    # RRT parameters: tweak as desired
    animate_rrt(
        grid, start, goal,
        step_size=1.2,          # growth per extension (cells)
        goal_radius=1.0,        # connect when within this distance
        goal_sample_rate=0.07,  # % of samples that are exactly the goal
        search_interval=25,
        follow_interval=18,
        max_iters=6000
    )

if __name__ == "__main__":
    main()
