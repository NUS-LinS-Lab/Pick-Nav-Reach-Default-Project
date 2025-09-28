import numpy as np
import random
import matplotlib.pyplot as plt
import os
from pathlib import Path

def generate_maze_map(
    n_rows: int,
    n_cols: int,
    in_pos: tuple,          # (r, c)
    out_pos: tuple,         # (r, c)
    grid_density: float = 0.4,
    add_x_min_wall: bool = False,
    add_x_max_wall: bool = False,
    add_y_min_wall: bool = True,
    add_y_max_wall: bool = True,
    p_greedy: float = 0.5,  # probability to choose the best (goal-directed) step
    seed: int | None = None,
) -> np.ndarray:
    """
    Return an (n_rows, n_cols) int array with 0 (free) and 1 (wall).
    Guaranteed open path from in_pos to out_pos.

    Notes:
    - Carving is restricted to the *interior region* if the corresponding border walls are enabled.
      Only the entrance/exit cells may sit on a walled border; they are kept open.
    - grid_density applies to leftover *interior* cells (excluding the carved path).
      Final global wall fraction may be > grid_density if border walls are enabled.
    """
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    # --- Basic validations ---
    def inside_grid(r, c):
        return 0 <= r < n_rows and 0 <= c < n_cols

    if not (inside_grid(*in_pos) and inside_grid(*out_pos)):
        raise ValueError("in_pos and out_pos must be inside the grid.")

    # --- Define carving bounds (interior) ---
    r_min = 1 if add_y_min_wall else 0
    r_max = (n_rows - 2) if add_y_max_wall else (n_rows - 1)
    c_min = 1 if add_x_min_wall else 0
    c_max = (n_cols - 2) if add_x_max_wall else (n_cols - 1)

    if r_min > r_max or c_min > c_max:
        raise ValueError("Interior is empty; relax border walls or increase grid size.")

    # --- Helper: is a cell allowed for carving (interior or the terminals) ---
    def allowed(r, c):
        if (r, c) == in_pos or (r, c) == out_pos:
            return True
        return (r_min <= r <= r_max) and (c_min <= c <= c_max)

    if not (allowed(*in_pos) and allowed(*out_pos)):
        raise ValueError(
            "in_pos/out_pos are incompatible with border walls; "
            "they must lie either inside the interior or exactly on the border to be opened."
        )

    # --- Start empty (all free) ---
    maze = np.zeros((n_rows, n_cols), dtype=np.int8)

    # --- Carve a guaranteed path via bounded, goal-biased random walk ---
    path = set()
    cur = tuple(in_pos)
    path.add(cur)

    # Manhattan distance
    def md(a, b): return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # all 4 neighbors within *grid*, then filter by 'allowed' (plus permit stepping into out_pos)
    dirs = [(1,0), (-1,0), (0,1), (0,-1)]
    steps_limit = n_rows * n_cols * 10  # safety cap

    while cur != tuple(out_pos) and steps_limit > 0:
        steps_limit -= 1
        r, c = cur
        nbrs = []
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if 0 <= nr < n_rows and 0 <= nc < n_cols:
                if allowed(nr, nc):  # allowed interior or terminals
                    nbrs.append((nr, nc))

        if not nbrs:
            break  # dead end (shouldn’t happen often with sane inputs)

        d0 = md(cur, out_pos)
        best = [p for p in nbrs if md(p, out_pos) < d0]
        others = [p for p in nbrs if p not in best]

        if best and random.random() < p_greedy:
            cur = random.choice(best)  # goal-directed
        else:
            cur = random.choice(others if others else nbrs)  # exploration

        path.add(cur)

    # Ensure terminals included
    path.add(tuple(in_pos))
    path.add(tuple(out_pos))

    # --- Randomly place walls on remaining *interior* cells (keep path free) ---
    interior_cells = [
        (r, c)
        for r in range(r_min, r_max + 1)
        for c in range(c_min, c_max + 1)
        if (r, c) not in path
    ]
    k = int(round(grid_density * len(interior_cells)))
    k = max(0, min(k, len(interior_cells)))
    walls = set(random.sample(interior_cells, k))
    for r, c in walls:
        maze[r, c] = 1

    # --- Add border walls last (then punch gates for in/out) ---
    if add_y_min_wall:  maze[0, :] = 1
    if add_y_max_wall:  maze[-1, :] = 1
    if add_x_min_wall:  maze[:, 0] = 1
    if add_x_max_wall:  maze[:, -1] = 1

    # keep the carved path and terminals open
    for r, c in path:
        maze[r, c] = 0
    maze[in_pos] = 0
    maze[out_pos] = 0

    return maze

def visualize_maze(maze: np.ndarray, in_pos=None, out_pos=None):
    """
    Visualize a maze using matplotlib.
    - Walls = black squares
    - Free space = white
    - Entrance (in_pos) = green
    - Exit (out_pos) = red
    """
    plt.figure(figsize=(maze.shape[1] / 2, maze.shape[0] / 2))
    plt.imshow(maze, cmap="gray_r", origin="upper")

    if in_pos is not None:
        plt.scatter(in_pos[1], in_pos[0], c="green", s=150, marker="o", label="Entrance")
    if out_pos is not None:
        plt.scatter(out_pos[1], out_pos[0], c="red", s=150, marker="*", label="Exit")

    plt.xticks([]); plt.yticks([])
    plt.legend()
    plt.show()

def add_left_room_to_maze(maze: np.ndarray, room_width: int = 3):
    """
    Add a rectangle room to the left side of the maze.
    The room is connected to the maze by removing a wall.

    Args:
        maze: 2D numpy array representing the maze
        room_width: Width of the room
    """
    room_height = maze.shape[0]
    room = np.zeros((room_height, room_width), dtype=bool)

    room[0, :] = 1
    room[-1, :] = 1
    room[:, 0] = 1

    maze = np.concatenate((room, maze), axis=1)
    return maze

# def create_maze_urdf(
#     maze_map,
#     urdf_path: str = "maze.urdf",
#     grid_xy: float = 1.0,
#     grid_z: float = 3.0,
#     maze_center_x: float = 0.0,
#     maze_center_y: float = 0.0,
#     maze_center_z: float = 0.0,
#     box_color=(1, 1, 1, 1),
# ):
#     """
#     Generate a URDF file describing a maze as static cubes for PyBullet.

#     Args:
#         maze_map: 2D list or array (1=wall, 0=empty)
#         urdf_path: path to save URDF file
#         grid_xy: grid cell size in XY
#         grid_z: wall height
#         maze_center_x, maze_center_y, maze_center_z: maze world center offset
#         box_color: RGBA for walls
#     """

#     n_rows = len(maze_map)
#     n_cols = len(maze_map[0]) if n_rows > 0 else 0
#     half_x, half_y, half_z = grid_xy / 2, grid_xy / 2, grid_z / 2

#     # XML header
#     urdf_parts = [
#         '<?xml version="1.0" ?>',
#         '<robot name="maze">'
#     ]

#     # each cube is a <link> + <joint> anchored to world
#     for i in range(n_rows):
#         for j in range(n_cols):
#             if maze_map[i][j] == 1:
#                 world_x = maze_center_x + (j - n_cols / 2 + 0.5) * grid_xy
#                 world_y = maze_center_y + (n_rows / 2 - i - 0.5) * grid_xy
#                 world_z = maze_center_z + half_z  # sit on ground

#                 name = f"block_{i}_{j}"

#                 link = f"""
#   <link name="{name}">
#     <visual>
#       <origin xyz="{world_x} {world_y} {world_z}" rpy="0 0 0"/>
#       <geometry>
#         <box size="{grid_xy} {grid_xy} {grid_z}"/>
#       </geometry>
#       <material name="wall">
#         <color rgba="{box_color[0]} {box_color[1]} {box_color[2]} {box_color[3]}"/>
#       </material>
#     </visual>
#     <collision>
#       <origin xyz="{world_x} {world_y} {world_z}" rpy="0 0 0"/>
#       <geometry>
#         <box size="{grid_xy} {grid_xy} {grid_z}"/>
#       </geometry>
#     </collision>
#     <inertial>
#       <mass value="0"/>  <!-- static -->
#       <inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/>
#     </inertial>
#   </link>
# """
#                 urdf_parts.append(link)

#     urdf_parts.append("</robot>")
#     urdf_str = "\n".join(urdf_parts)

#     # Save to file
#     Path(os.path.dirname(urdf_path) or ".").mkdir(parents=True, exist_ok=True)
#     with open(urdf_path, "w") as f:
#         f.write(urdf_str)

#     return urdf_path


def create_maze_urdf(
    maze_map,
    urdf_path: str = "maze.urdf",
    grid_xy: float = 1.0,
    grid_z: float = 3.0,
    maze_center_x: float = 0.0,
    maze_center_y: float = 0.0,
    maze_center_z: float = 0.0,
    box_color=(1, 1, 1, 1),
):
    """
    Generate a URDF file for a maze (walls as cubes) that can be loaded in PyBullet.
    - Single root link 'maze_base'
    - Each block attached with a fixed joint
    """

    n_rows = len(maze_map)
    n_cols = len(maze_map[0]) if n_rows > 0 else 0
    half_x, half_y, half_z = grid_xy / 2, grid_xy / 2, grid_z / 2

    urdf_parts = [
        '<?xml version="1.0" ?>',
        '<robot name="maze">',
        '  <link name="maze_base"/>',   # single root link
    ]

    cube_positions = []

    for i in range(n_rows):
        for j in range(n_cols):
            if maze_map[i][j] == 1:
                world_x = maze_center_x + (j - n_cols / 2 + 0.5) * grid_xy
                world_y = maze_center_y + (n_rows / 2 - i - 0.5) * grid_xy
                world_z = maze_center_z + half_z

                name = f"block_{i}_{j}"

                # child link
                link = f"""
  <link name="{name}">
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="{grid_xy} {grid_xy} {grid_z}"/>
      </geometry>
      <material name="wall">
        <color rgba="{box_color[0]} {box_color[1]} {box_color[2]} {box_color[3]}"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="{grid_xy} {grid_xy} {grid_z}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0"/> <!-- static -->
      <inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
"""

                joint = f"""
  <joint name="joint_{name}" type="fixed">
    <parent link="maze_base"/>
    <child link="{name}"/>
    <origin xyz="{world_x} {world_y} {world_z}" rpy="0 0 0"/>
  </joint>
"""

                urdf_parts.append(link)
                urdf_parts.append(joint)
                cube_positions.append((world_x, world_y, world_z))

    urdf_parts.append("</robot>")
    urdf_str = "\n".join(urdf_parts)

    Path(os.path.dirname(urdf_path) or ".").mkdir(parents=True, exist_ok=True)
    with open(urdf_path, "w") as f:
        f.write(urdf_str)

    cube_positions = np.array(cube_positions) if len(cube_positions) else None
    return cube_positions


if __name__ == "__main__":
    # maze hyperparameters
    n_rows = 12
    n_cols = 16
    room_width = 6
    maze_offset = 2.0
    grid_xy = 1.0
    grid_z = 3.0
    # need to randomize
    random_row_in = np.random.randint(1, n_rows-1)
    random_row_out = np.random.randint(1, n_rows-1) 
    in_pos = (random_row_in, 0)
    out_pos = (random_row_out, n_cols-1)
    

    # Test maze generation and visualization
    maze = generate_maze_map(
        n_rows=n_rows, n_cols=n_cols,
        in_pos=in_pos, out_pos=out_pos,
        grid_density=0.8,
        add_x_min_wall=True, add_x_max_wall=True,
        add_y_min_wall=True, add_y_max_wall=True,
        p_greedy=0.6,
        seed=5,
    )

    print(maze)
    maze = add_left_room_to_maze(maze, room_width=room_width)
    print(maze)
    visualize_maze(maze, in_pos=(in_pos[0], in_pos[1] + room_width), out_pos=(out_pos[0], out_pos[1] + room_width))

    create_maze_urdf(
        maze, 
        urdf_path="maze.urdf",
        grid_xy=grid_xy, grid_z=grid_z,
        maze_center_x=(n_cols-room_width)/2*grid_xy + maze_offset, 
        maze_center_y=0.0, 
        maze_center_z=0.0,
        box_color=(0.8, 0.8, 0.8, 1.0),
    )

