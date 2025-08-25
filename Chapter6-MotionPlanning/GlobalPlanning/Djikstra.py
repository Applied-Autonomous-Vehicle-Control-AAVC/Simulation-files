import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.animation import FuncAnimation
from matplotlib.colors import ListedColormap

# ---------------------------- Grid Utilities ---------------------------- #

def create_occupancy_grid(width, height, obstacle_prob, seed):
    """
    Returns a (height x width) uint8 grid where 0 = free, 1 = obstacle.
    """
    rng = np.random.default_rng(seed)
    grid = (rng.random((height, width)) < obstacle_prob).astype(np.uint8)
    return grid

def neighbors(y, x, h, w, allow_diagonal=True):
    steps4 = [(-1,0),(1,0),(0,-1),(0,1)]
    steps8 = steps4 + [(-1,-1),(-1,1),(1,-1),(1,1)]
    steps = steps8 if allow_diagonal else steps4
    for dy, dx in steps:
        ny, nx = y+dy, x+dx
        if 0 <= ny < h and 0 <= nx < w:
            yield ny, nx, math.hypot(dy, dx)  # 1 or sqrt(2)

# ---------------------------- Search Generator ---------------------------- #

def dijkstra_search_generator(grid, start, goal, allow_diagonal=True):
    """
    A step-by-step Dijkstra that yields the search state at each node expansion.
    Yields tuples:
      ("search", visited_bool_array, frontier_list[(y,x)], current_node(y,x))
    and finally:
      ("found", path_list[(y,x)], visited_bool_array)   if a path exists
      OR
      ("nopath", visited_bool_array)                    if no path exists
    """
    h, w = grid.shape
    sy, sx = start
    gy, gx = goal
    if grid[sy, sx] == 1 or grid[gy, gx] == 1:
        raise ValueError("Start or goal inside an obstacle.")

    dist = np.full((h, w), np.inf)
    prev = np.full((h, w, 2), -1, dtype=int)
    visited = np.zeros((h, w), dtype=bool)
    pq = []  # (dist, (y,x))
    dist[sy, sx] = 0.0
    heapq.heappush(pq, (0.0, (sy, sx)))

    while pq:
        d, (y, x) = heapq.heappop(pq)
        if visited[y, x]:
            # skip stale queue entries
            continue
        visited[y, x] = True

        # Snapshot frontier nodes currently in the heap (not yet visited)
        # (Make a shallow copy of coords only for display)
        frontier_nodes = [(coord[0], coord[1]) for _, coord in pq if not visited[coord]]

        # Yield the current search step
        yield ("search", visited.copy(), frontier_nodes, (y, x))

        if (y, x) == (gy, gx):
            # Reconstruct path
            path = []
            cy, cx = gy, gx
            while (cy, cx) != (sy, sx):
                path.append((cy, cx))
                cy, cx = prev[cy, cx]
            path.append((sy, sx))
            path.reverse()
            yield ("found", path, visited.copy())
            return

        # Relax neighbors
        for ny, nx, cost in neighbors(y, x, h, w, allow_diagonal):
            if visited[ny, nx] or grid[ny, nx] == 1:
                continue
            nd = d + cost
            if nd < dist[ny, nx]:
                dist[ny, nx] = nd
                prev[ny, nx] = [y, x]
                heapq.heappush(pq, (nd, (ny, nx)))

    # If we exhaust the frontier: no path
    yield ("nopath", visited.copy())

# ---------------------------- Animation ---------------------------- #

def make_color_layers_image(grid, visited=None, frontier=None, current=None, start=None, goal=None):
    """
    Compose a simple categorical image:
      0 = free, 1 = obstacle, 2 = visited, 3 = frontier, 4 = current, 5 = start, 6 = goal
    Priority: goal > start > current > frontier > visited > obstacle > free
    """
    base = np.zeros_like(grid, dtype=np.uint8)  # 0 free
    base[grid == 1] = 1  # obstacle

    # Overlay layers with priority using masks:
    if visited is not None:
        base[visited & (grid == 0)] = 2
    if frontier:
        fy, fx = zip(*frontier) if len(frontier) else ([], [])
        if fy:
            base[(np.array(fy), np.array(fx))] = 3
    if current is not None:
        cy, cx = current
        base[cy, cx] = 4
    if start is not None:
        sy, sx = start
        base[sy, sx] = 5
    if goal is not None:
        gy, gx = goal
        base[gy, gx] = 6

    # Define a clean, high-contrast palette
    cmap = ListedColormap([
        "#FFFFFF",  # 0 free (white)
        "#000000",  # 1 obstacle (black)
        "#B0C4DE",  # 2 visited (light steel blue)
        "#FFC107",  # 3 frontier (amber)
        "#DC143C",  # 4 current (crimson)
        "#1E90FF",  # 5 start (dodger blue)
        "#32CD32",  # 6 goal (lime green)
    ])
    return base, cmap

def interpolate_points(points, frames_per_cell=5):
    """Create intermediate points between grid centers for a smoother follow animation."""
    if len(points) < 2:
        return np.array(points, dtype=float)
    out = []
    pts = np.array(points, dtype=float)
    for i in range(len(pts) - 1):
        p0, p1 = pts[i], pts[i+1]
        length = np.linalg.norm(p1 - p0)
        steps = max(1, int(frames_per_cell * max(1.0, length)))
        for t in np.linspace(0, 1, steps, endpoint=False):
            out.append(p0 * (1 - t) + p1 * t)
    out.append(pts[-1])
    return np.array(out)

def animate_dijkstra(grid, start, goal, allow_diagonal=True, search_interval=40, follow_interval=25):
    """
    Runs Dijkstra step-by-step and animates:
      1) search expansion (visited/frontier/current)
      2) path following once found
    """
    h, w = grid.shape
    gen = dijkstra_search_generator(grid, start, goal, allow_diagonal=allow_diagonal)

    # Figure setup
    fig, ax = plt.subplots(figsize=(w/4, h/4))
    ax.set_title("Dijkstra Search (step-by-step) → Path Follow")
    ax.set_xticks(np.arange(-0.5, w, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, h, 1), minor=True)
    ax.grid(which='minor', linestyle=':', linewidth=0.5)
    ax.set_xlim(-0.5, w-0.5)
    ax.set_ylim(h-0.5, -0.5)

    # Initial image
    img_data, cmap = make_color_layers_image(grid, visited=np.zeros_like(grid, bool),
                                             frontier=[], current=None,
                                             start=start, goal=goal)
    im = ax.imshow(img_data, cmap=cmap, origin='upper', vmin=0, vmax=6)

    # Path + car artists (added later when path found)
    path_line, = ax.plot([], [], lw=2, alpha=0.9)
    car_marker, = ax.plot([], [], marker='o', markersize=8)

    # ---------------- Legend ---------------- #
    legend_elements = [
        Patch(facecolor="#FFFFFF", edgecolor="k", label="Free"),
        Patch(facecolor="#000000", edgecolor="k", label="Obstacle"),
        Patch(facecolor="#B0C4DE", edgecolor="k", label="Visited"),
        Patch(facecolor="#FFC107", edgecolor="k", label="Frontier"),
        Patch(facecolor="#DC143C", edgecolor="k", label="Current"),
        Patch(facecolor="#1E90FF", edgecolor="k", label="Start"),
        Patch(facecolor="#32CD32", edgecolor="k", label="Goal"),
    ]
    ax.legend(handles=legend_elements, bbox_to_anchor=(1.05, 1), loc='upper left')

    # Internal state to switch phases
    phase = {"mode": "search", "follow_points": None, "follow_idx": 0}

    def update(_frame):
        nonlocal phase
        if phase["mode"] == "search":
            try:
                item = next(gen)
            except StopIteration:
                return (im, path_line, car_marker)

            tag = item[0]
            if tag == "search":
                _tag, visited, frontier, current = item
                img_data, _ = make_color_layers_image(
                    grid, visited=visited, frontier=frontier, current=current,
                    start=start, goal=goal
                )
                im.set_data(img_data)
                return (im,)

            elif tag == "found":
                _tag, path, visited = item
                img_data, _ = make_color_layers_image(
                    grid, visited=visited, frontier=[], current=None,
                    start=start, goal=goal
                )
                im.set_data(img_data)

                py, px = zip(*path)
                path_line.set_data(px, py)

                pts = [(x, y) for y, x in path]
                follow_pts = interpolate_points(pts, frames_per_cell=5)
                phase["mode"] = "follow"
                phase["follow_points"] = follow_pts
                phase["follow_idx"] = 0
                car_marker.set_data([follow_pts[0, 0]], [follow_pts[0, 1]])
                return (im, path_line, car_marker)

            elif tag == "nopath":
                _tag, visited = item
                img_data, _ = make_color_layers_image(
                    grid, visited=visited, frontier=[], current=None,
                    start=start, goal=goal
                )
                im.set_data(img_data)
                ax.set_title("Dijkstra: No path found")
                return (im,)

        elif phase["mode"] == "follow":
            pts = phase["follow_points"]
            i = phase["follow_idx"]
            if i < len(pts):
                x, y = pts[i]
                car_marker.set_data([x], [y])
                phase["follow_idx"] += 1
            return (im, path_line, car_marker)

    ani = FuncAnimation(fig, update, frames=10**9, interval=search_interval, blit=True)

    def on_draw(event):
        if phase["mode"] == "follow":
            ani.event_source.interval = follow_interval
    cid = fig.canvas.mpl_connect('draw_event', on_draw)

    plt.show()

# ---------------------------- Main ---------------------------- #

def main():
    # Scenario
    W, H = 60, 35
    start = (H - 2, 1)   # (row, col)
    goal  = (1, W - 2)

    grid = create_occupancy_grid(W, H, obstacle_prob=0.22, seed=7)
    grid[start] = 0
    grid[goal] = 0

    animate_dijkstra(grid, start, goal, allow_diagonal=True,
                     search_interval=40, follow_interval=25)

if __name__ == "__main__":
    main()
