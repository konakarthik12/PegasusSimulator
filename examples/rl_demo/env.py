# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import math

import carb
import gymnasium
import numpy as np
from gymnasium import spaces


class DroneEnv(gymnasium.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(
        self,
        skip_frame=1,
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 60.0,
        max_episode_length=256,
        seed=0,
        headless=True,
    ) -> None:
        from omni.isaac.kit import SimulationApp

        self.headless = headless
        self._simulation_app = SimulationApp({"headless": self.headless, "anti_aliasing": 0})
        self._skip_frame = skip_frame
        self._dt = physics_dt * self._skip_frame
        self._max_episode_length = max_episode_length
        self._steps_after_reset = int(rendering_dt / physics_dt)
        import omni.timeline
        from omni.isaac.core.world import World

        # Imports to be able to log to the terminal with fancy colors
        import carb
        from carb.input import KeyboardInput, KeyboardEventType
        # Imports from the Pegasus library
        from omni.isaac.core.objects import DynamicCuboid

        from pegasus.simulator.logic.backends import Backend
        from pegasus.simulator.logic.backends import simple_controller

        from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, ROBOTS_ASSETS
        from pegasus.simulator.logic.state import State
        from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
        from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.pg._world.scene.add_default_ground_plane()
        self._cube = self.pg._world.scene.add(DynamicCuboid(
                prim_path="/World/random_cube", # The prim path of the cube in the USD stage
                name="pick_box", # The unique name used to retrieve the object from the scene later on
                position=np.array([-2, 2, 0.1]), # Using the current stage units which is in meters by default.
                scale=np.array([0.02, 0.02, 0.02]), # most arguments accept mainly numpy arrays.
                color=np.array([0, 0, 1.0]), # RGB channels, going from 0-1
            ))

        # self.world = self.get_world()

        curr_dir = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve())
        self._config_multirotor = MultirotorConfig()
                # config_multirotor1.drag = HydroDynamicDrag(body= iris_body_config, hydro_dynamic_config=IrisHydroDynamicConfig)
        self._config_multirotor.backends = [simple_controller.SimpleController(
            trajectory_file=None, # self.curr_dir + "/trajectories/pitch_relay_90_deg_2.csv",
            results_file=curr_dir + "/single_statistics.npz",
        Kp=[1, 1, 1],
        Kd=[0.1,0.1, 0.1],
        Ki=[0.1, 0.1, 0.1],
            Kr=[2.0, 2.0, 2.0]
        )]

        self.drone = Multirotor(
            self.pg._world,
            "/World/quadrotor",
            ROBOTS_ASSETS+"/drone_models/cross_drone13_v10_mani_gripper.usd", #cross_drone13_v7.usd", 
            0,
            [0, 0, 3],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=self._config_multirotor,
            track_object_name="pick_box",
            position_controller=False,
        )

        # We add the task to the world here
        self.pg._world.add_task(RLTask(name="drone",robot=self.multimotor))


        self.seed(seed)
        self.reward_range = (-float("inf"), float("inf"))
        gymnasium.Env.__init__(self)
 
        # Change in velocity, future should be pitch, roll, yaw (PRY)
        self.action_space = gym.spaces.Box(low=np.array([-1, -1, -1]), high=np.array([1, 1, 1]), dtype=np.float32)

        # 3 positions, and 3 total velocities in each direction
        low = np.array([-25, -25, 0.0, -15, -15, -15])
        high = np.array([25, 25, 10.0, 15, 15, 15])
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        # current position and velocity
        self.current_step = 0

        # Location of the target object
        self.goal, _ = self._cube.get_world_pose()
        self.reference =  {"p" : np.zeros((3,)), "v" : np.zeros((3,)), "a" : np.zeros((3,))}
        self.reset_counter = 0
        return

    def get_dt(self):
        return self._dt

    def step(self, action):
        
        self.apply_action(action)
        # we apply our actions to the jetbot
        for i in range(self._skip_frame):
            # self.apply_action(act)
            self.pg._world.step(render=False)

        observations = self.get_observations()
        info = {}
        done = False
        truncated = False

        done = self.termination()
        reward = self._reward()

        return observation, done, reward
        
    
    def termination(self):
        # Make sure to consider all details and edge cases
        # False cases: drone goes outside boundaries, reaches max time steps
        # Success cases: we reach target pos, grabs the object and flies in the air
        termination = False

        if self.pg._world.current_time_step_index - self._steps_after_reset >= self._max_episode_length:
            done = True
            truncated = True

        position_x, position_y, position_z = self.drone._state.position
        low = np.array([-25, -25, 0.0, -15, -15, -15])
        high = np.array([25, 25, 10.0, 15, 15, 15])

        if position_x < -25 or position_x > 25 or position_y < -25 or position_y > 25 or position_z > 10:
            termination = True

        if abs(position_x - self.goal[0]) < 1 or abs(position_y - self.goal[1]) < 1 or abs(position_z - self.goal[2]) < 1:
            termination = True

        return termination
        

    def reward(self):
        
        # self.state[0:3] = self.drone._state.position
        # object_pose.r gives Rotation Quaternion
        distance = np.linalg.norm(self.drone._state.position-self.goal)
        reward = -distance

        flag = False
        if distance < 1:  
            flag = True
            reward += 100
            print(f"Goal Reached at x = {self.current_x}, y = {self.current_y}, z = {self.current_z}")

        return reward


    
    def apply_action(self, action):
        self.reference["v"] = action
            
        self.drone.update_target_reference(self.reference)

    def reset(self, seed=None):
        self._cube.get_applied_visual_material().set_color(color=np.array([0, 0, 1.0]))
        self._task_achieved = False
        self.reference =  {"p" : np.zeros((3,)), "v" : np.zeros((3,)), "a" : np.zeros((3,))}
        self.pg._world.reset()
        self.drone.reset()
        observations = self.get_observations()
        return observations, {}

    def get_observations(self):
        self.pg._world.render()
        obs = np.concatenate([self.drone._state.position,self.drone._state.linear_velocity])
        return obs

    def render(self, mode="human"):
        return

    def close(self):
        self._simulation_app.close()
        return

    def seed(self, seed=None):
        self.np_random, seed = gymnasium.utils.seeding.np_random(seed)
        np.random.seed(seed)
        return [seed]
