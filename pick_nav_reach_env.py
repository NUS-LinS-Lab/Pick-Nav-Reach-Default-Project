from time import time
import pybullet as p
import pybullet_data
import numpy as np
import random 
from pathlib import Path
from maze_utils import generate_maze_map, add_left_room_to_maze, create_maze_urdf

class PickNavReachEnv:

    def __init__(self, 
                 seed=0,
                 object_idx=0,
                 ):
        self.set_seed(seed)

        self.pb_physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
        p.setGravity(0,0,-9.8)

        p.configureDebugVisualizer(lightPosition=[0, 0, 10])
        p.configureDebugVisualizer(rgbBackground=[1, 1, 1])        # white background brightens perception

        p.loadURDF("plane.urdf")

        self.seed = seed
        self.object_idx = object_idx

        self._load_scene()

    def set_seed(self, seed):
        np.random.seed(seed)
        random.seed(seed)
    
    def _load_scene(self):
        self._load_agent()
        self._load_object_table()
        self._load_maze()

    def _load_agent(self):
        pass 

    def _load_object_table(self):

        p.loadURDF("table/table.urdf", baseOrientation=p.getQuaternionFromEuler([0,0,np.pi/2]), useFixedBase=1)

        # hyperparameters
        ycb_object_dir_path = "./assets/ycb_objects/"
        ycb_objects_paths = sorted(list(Path(ycb_object_dir_path).glob("*")))
        assert 0 <= self.object_idx and self.object_idx < len(ycb_objects_paths), f"object_idx should be in [0, {len(ycb_objects_paths)-1}]"
        object_urdf_path = (ycb_objects_paths[self.object_idx] / "coacd_decomposed_object_one_link.urdf").absolute()
        p.setAdditionalSearchPath(str(object_urdf_path.parent) )
        p.loadURDF(str(object_urdf_path), basePosition=[0, 0, 5.0], useFixedBase=0)


    def _load_maze(self):
        # maze hyperparameters
        n_rows = 12
        n_cols = 16
        room_width = 6
        maze_offset = 2.0
        grid_xy = 1.0
        grid_z = 2.0
        # need to randomize
        random_row_in = np.random.randint(1, n_rows-1)
        random_row_out = np.random.randint(1, n_rows-1) 
        in_pos = (random_row_in, 0)
        out_pos = (random_row_out, n_cols-1)

        maze = generate_maze_map(
            n_rows=n_rows, n_cols=n_cols,
            in_pos=in_pos, out_pos=out_pos,
            grid_density=0.8,
            add_x_min_wall=True, add_x_max_wall=True,
            add_y_min_wall=True, add_y_max_wall=True,
            p_greedy=0.6,
            seed=5,
        )

        maze = add_left_room_to_maze(maze, room_width=room_width)

        create_maze_urdf(
            maze, 
            urdf_path="./assets/maze.urdf",
            grid_xy=grid_xy, grid_z=grid_z,
            maze_center_x=(n_cols-room_width)/2*grid_xy + maze_offset, 
            maze_center_y=0.0, 
            maze_center_z=0.0,
            box_color=(0.95, 0.95, 0.98, 1.0),
        )

        p.loadURDF("./assets/maze.urdf", useFixedBase=1, flags=p.URDF_MERGE_FIXED_LINKS)


if __name__ == "__main__":
    env = PickNavReachEnv(seed=42)

    # for i in range (10000):
    import time
    while True:
        p.stepSimulation()
        time.sleep(1./240.)


