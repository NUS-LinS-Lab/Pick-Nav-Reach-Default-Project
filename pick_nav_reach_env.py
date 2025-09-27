from time import time
import pybullet as p
import pybullet_data
import numpy as np
import random 
from pathlib import Path
from maze_utils import generate_maze_map, add_left_room_to_maze, create_maze_urdf
from keyboard_control import KeyBoardController

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
        
        self.action_scale = 0.05
        self.max_force = 2000000
        self.substeps = 20

        self._load_scene()

    def set_seed(self, seed):
        np.random.seed(seed)
        random.seed(seed)
    
    def _load_scene(self):
        # self._load_agent()
        self._load_object_table()
        self._load_maze()

    def _load_agent(self):
        # Place the Fetch base near the table
        base_pos = [0.0, 0.0, 0.0]
        base_ori = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = p.loadURDF(
            "fetch/fetch.urdf",
            base_pos,
            base_ori,
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION,
        )

        # Collect joint info (skip fixed)
        n_joints = p.getNumJoints(self.robot_id)
        indices = []
        lowers, uppers, ranges, rest = [], [], [], []

        for j in range(n_joints):
            info = p.getJointInfo(self.robot_id, j)
            joint_type = info[2]
            if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                indices.append(j)
                lowers.append(info[8])
                uppers.append(info[9])
                ranges.append(info[9] - info[8])
                rest.append(info[10])  # joint damping? (PyBullet packs different things; we keep a placeholder)

                # Disable default motors to allow POSITION_CONTROL commands
                p.setJointMotorControl2(
                    self.robot_id, j, p.VELOCITY_CONTROL, force=0.0
                )

        self.joint_indices = indices
        self.joint_lower = np.array(lowers, dtype=np.float32)
        self.joint_upper = np.array(uppers, dtype=np.float32)
        self.joint_ranges = np.array(ranges, dtype=np.float32)
        self.rest_poses = np.zeros_like(self.joint_lower, dtype=np.float32)
        
        # self.joint_lower[self.joint_upper==-1] = -np.inf
        # self.joint_upper[self.joint_upper==-1] = np.inf
        # self.joint_ranges[self.joint_upper==np.inf] = np.inf
        
        print("Controllable joints:", len(self.joint_indices))
        print("Joint Indices:", self.joint_indices)
        print("Joint Lower:", self.joint_lower)
        print("Joint Upper:", self.joint_upper)

        # # Try to set a sane initial configuration (elbow up, etc.) if limits known
        # q0 = np.clip(
        #     np.array([
        #         0.0, -1.2, 1.2, -1.0, 0.0, 1.0, 0.5  # a typical 7-DoF arm seed (adjust to your Fetch URDF)
        #     ] + [0.0] * (len(self.joint_indices) - 7),
        #     self.joint_lower,
        #     self.joint_upper,
        # ))
        # self._set_qpos(q0)

    def _load_object_table(self):

        p.loadURDF("table/table.urdf", baseOrientation=p.getQuaternionFromEuler([0,0,np.pi/2]), useFixedBase=1)

        # hyperparameters
        ycb_object_dir_path = "./assets/ycb_objects/"
        ycb_objects_paths = sorted(list(Path(ycb_object_dir_path).glob("*")))
        assert 0 <= self.object_idx and self.object_idx < len(ycb_objects_paths), f"object_idx should be in [0, {len(ycb_objects_paths)-1}]"
        object_urdf_path = (ycb_objects_paths[self.object_idx] / "coacd_decomposed_object_one_link.urdf").absolute()
        p.setAdditionalSearchPath(str(object_urdf_path.parent) )
        p.loadURDF(str(object_urdf_path), basePosition=[0, 0, 5.0], useFixedBase=0)


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
        
    def step(self, action):
        """
        Apply action (delta q) then simulate for `substeps`.
        action: shape (n_dofs,) in [-1, 1] -> scaled by action_scale
        Returns: obs, reward, terminated, truncated, info
        """
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        n = len(self.joint_indices)
        if action.size != n:
            raise ValueError(f"Action size {action.size} != controllable dofs {n}")

        action = np.clip(action, -1.0, 1.0)
        qpos, _ = self._get_state()
        print("current qpos: ", qpos)
        target = qpos + self.action_scale * action
        target = np.clip(target, self.joint_lower, self.joint_upper)
        print("target: ", target)

        # Position control for all controllable joints
        p.setJointMotorControlArray(
            bodyUniqueId=self.robot_id,
            jointIndices=self.joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=target.tolist(),
            forces=[self.max_force] * len(self.joint_indices),
            positionGains=[0.3] * len(self.joint_indices),
        )

        for _ in range(self.substeps):
            p.stepSimulation()
            time.sleep(1./240.)

        obs = self._get_obs()

        reward = 0.0
        terminated = False
        truncated = False
        info = {}

        return obs, reward, terminated, truncated, info
    
    def _get_obs(self):
        qpos, qvel = self._get_state()
        return np.concatenate([qpos, qvel], axis=0).astype(np.float32)

    def _get_state(self):
        states = p.getJointStates(self.robot_id, self.joint_indices)
        qpos = np.array([s[0] for s in states], dtype=np.float32)
        qvel = np.array([s[1] for s in states], dtype=np.float32)
        return qpos, qvel

    def _set_qpos(self, qpos):
        qpos = np.asarray(qpos, dtype=np.float32).reshape(-1)
        if qpos.size != len(self.joint_indices):
            raise ValueError("qpos size mismatch.")
        for idx, q in zip(self.joint_indices, qpos):
            p.resetJointState(self.robot_id, idx, q)

    @property
    def action_size(self):
        return len(self.joint_indices)

    @property
    def obs_size(self):
        # qpos + qvel
        return 2 * len(self.joint_indices)


if __name__ == "__main__":
    env = PickNavReachEnv(seed=42)
    # print(f"Action size: {env.action_size}, Obs size: {env.obs_size}")
    
    # keyboard_controller = KeyBoardController(env)

    # for i in range (10000):
    import time
    while True:
        # random_action = np.random.uniform(-1.0, 1.0, size=(env.action_size,))
        # action = keyboard_controller.get_action()
        # print("action: ", action)
        # obs, reward, terminated, truncated, info = env.step(action)
        p.stepSimulation()
        time.sleep(1./240.)


