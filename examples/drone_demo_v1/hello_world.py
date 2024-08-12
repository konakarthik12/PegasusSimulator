import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.examples.base_sample import BaseSample
# from omni.isaac.examples.user_examples import SimpleController
import os

# Imports to be able to log to the terminal with fancy colors
import carb
from carb.input import KeyboardInput, KeyboardEventType
# Imports from the Pegasus library
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.backends import simple_controller

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, ROBOTS_ASSETS, ENV_ASSETS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
import omni.isaac.core.utils.prims as prim_utils
# from pegasus.simulator.logic.vehicles import BodyConfig
# from pegasus.simulator.logic.dynamics.hydrodynamic_drag import HydroDynamicDrag, HydroDynamicConfig

# from pegasus.simulator.logic.vehicles.multirotors.iris import IrisBodyConfig, IrisHydroDynamicConfig

# Import the custom python control backend
# from utils.nonlinear_controller import NonlinearController
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
from omni.kit.viewport.utility import get_active_viewport_window
from omni.isaac.core.utils.extensions import enable_extension
from PIL import Image
import numpy as np
import os
from scipy.spatial.transform import Rotation
from omni.isaac.core.tasks import BaseTask
from pathlib import Path

# class Camera():
#     def __init__(self, cam_path: str="quadrotor/body/Down_Camera"):
#         self.cam_path = cam_path
#         self.count = 0

#     def setup(self):
        
#         enable_extension("omni.isaac.synthetic_utils")
#         from omni.isaac.synthetic_utils import SyntheticDataHelper

#         self.sd_helper = SyntheticDataHelper()

#         self.viewport = get_active_viewport_window("Viewport").viewport_api

#     def get_image(self):
#         self.viewport.set_active_camera(self.cam_path)
#         gt = self.sd_helper.get_groundtruth(["rgb", "depth",], self.viewport,)
#         depth = gt["depth"]
#         rgb = gt["rgb"]
#         self.count += 1
#         return rgb, depth

#     def save_image_rgb(self,rgb):
#         image = Image.fromarray(rgb, 'RGBA')

#         # Save the image
#         image_path = r'C:\Users\willy\AppData\Local\ov\pkg\isaac_sim-2023.1.1\exts\omni.isaac.examples\omni\isaac\examples\user_examples\drone_trajectories\rgb_' + str(self.count) + '.png'
#         image.save(image_path)

#     def save_image_depth(self,depth):
#         depth_normalized = (255 * (depth - np.min(depth)) / (np.max(depth) - np.min(depth))).astype(np.uint8)
#         image = Image.fromarray(depth_normalized, "L")

#         # Save the image
#         image_path = r'C:\Users\willy\AppData\Local\ov\pkg\isaac_sim-2023.1.1\exts\omni.isaac.examples\omni\isaac\examples\user_examples\drone_trajectories\depth' + str(self.count) + '.png'
#         image.save(image_path)

class RLTask(BaseTask):

    #NOTE: we only cover here a subset of the task functions that are available,
    # checkout the base class for all the available functions to override.
    # ex: calculate_metrics, is_done..etc.
    def __init__(self, name, robot):
        super().__init__(name=name)
        # self._goal_position = np.array([-0.3, -0.3, 0.0515 / 2.0])
        self._task_achieved = False

        self.drone = robot
        self.reference =  {"p" : np.zeros((3,)), "v" : np.zeros((3,)), "a" : np.zeros((3,)), "yaw": 0}
        self.reference["p"] = self.drone._state.grippers_position
        self.reach_goal = False
        self.reach_cnt = None
        self.take_pictures = True
        self.prev_dist = 10000
        return

    # Here we setup all the assets that we care about in this tasfk.
    def set_up_scene(self, scene):

        super().set_up_scene(scene)

        # self._cube = scene.add(DynamicCuboid(
        #         prim_path="/World/random_cube", # The prim path of the cube in the USD stage
        #         name="pick_box", # The unique name used to retrieve the object from the scene later on
        #         position=np.array([0, 0, 0.1]), # Using the current stage units which is in meters by default.
        #         scale=np.array([0.015, 0.015, 0.015]), # most arguments accept mainly numpy arrays.
        #         color=np.array([0, 0, 1.0]), # RGB channels, going from 0-1
        #     ))
        
        # self.prev_dist = np.linalg.norm(self.drone._state.grippers_position - np.zeros((3,)))


        # Successful trajectory 1: Sphere, position=np.array([0.023, -0.0015, 0.1]), radius=0.005, mass = 0.00001
        self._cube = scene.add(DynamicSphere(
                prim_path="/World/random_sphere", # The prim path of the cube in the USD stage
                name="pick_ball", # The unique name used to retrieve the object from the scene later on
                position=np.array([0.0, 0.0, 0.1]), # Using the current stage units which is in meters by default.
                color=np.array([0,  0, 1.0]), # RGB channels, going from 0-1
                radius=0.005,
                mass = 0.00001
            ))
        
        # self._camera = Camera()
        # self._camera.setup()

        return


    # Information exposed to solve the task is returned from the task through get_observations

    def get_observations(self):
        cube_position, _ = self._cube.get_world_pose()
        
        observations = {
            "Robot" : self.drone._state.position,
            self._cube.name: {
                "position": cube_position,
                "goal_position": self._goal_position
            }
        }
        return observations
    

    # Called before each physics step,
    # for instance we can check here if the task was accomplished by
    # changing the color of the cube once its accomplished

    # Successful Trajectory 1: position_gripper[2] <= 0.0213, self.reference["p"][2] += 0.003
    def pre_step(self, control_index, simulation_time):
        print("Control index count: ", control_index)
        print("path : ",ROBOTS_ASSETS+"/drone_models/cross_drone13_v10_mani_gripper.usd")
        if self.drone.track_position:
            position, orientation = self._cube.get_world_pose() #self._sphere.get_world_pose()
            position_gripper = self.drone._state.grippers_position

            if self.reach_cnt is None:
                self.reference["p"] = np.array([0.0,0.0,1.0]) #position_gripper
                self.reach_cnt = simulation_time

            if simulation_time-self.reach_cnt > 0.01:
                new_dist = np.linalg.norm(self.drone._state.grippers_position - np.zeros((3,)))
                print(self.prev_dist)
                if self.prev_dist - new_dist >= 0:
                    print("new reference : ", self.reference, "| Position : ", position_gripper)
                    self.prev_dist = new_dist

                # if position_gripper[2] <= 0.25:
                #     if self.take_pictures and control_index % 25 == 0:
                #         rgb, depth = self._camera.get_image()
                #         self._camera.save_image_rgb(rgb)
                #         self._camera.save_image_depth(depth)
            
                if position_gripper[2] <= 0.021:
                    self.reference["p"][2] += 0.003
                    self.drone.open_gripper = False
                    self.drone.close_gripper = True
                    self.reach_goal = True
                    

            # Successful Trajectory 1: self.reference["p"][2] = 0.0235
            if simulation_time-self.reach_cnt>.3 and not self.reach_goal:
                if np.allclose(self.reference["p"][2], 0.0213, atol=1e-7):
                    self.take_pictures = False
                else:
                    self.reference["p"][2] -= 0.01
                self.reach_cnt = simulation_time
            
            if simulation_time-self.reach_cnt>.3 and self.reach_goal:
                self.reference["p"][2] += 0.003

                
        self.drone.update_target_reference(self.reference)
        return

    # Called after each reset,
    # for instance we can always set the gripper to be opened at the beginning after each reset
    # also we can set the cube's color to be blue
    def post_reset(self):
        # self._cube.get_applied_visual_material().set_color(color=np.array([0, 0, 1.0]))
        self._task_achieved = False
        self.reference =  {"p" : np.zeros((3,)), "v" : np.zeros((3,)), "a" : np.zeros((3,)), "yaw": 0}
        self.prev_dist = 10000  # np.linalg.norm(np.zeros((3,))-self.drone._state.grippers_position)
        self.drone.reset()
        return


class DroneDemoV1(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._command=None
        # self.multirotor = None

        # Start the Pegasus Interface
        self.pg = PegasusInterface()
        return

    def setup_scene(self):
        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
               # Start the Pegasus Interface
        self.pg = PegasusInterface()
        self.pg.load_environment(ENV_ASSETS + "/fluid_test_2.usd")
        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World()
        self.world = self.pg._world

        curr_dir = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve())
        self._config_multirotor = MultirotorConfig()
                # config_multirotor1.drag = HydroDynamicDrag(body= iris_body_config, hydro_dynamic_config=IrisHydroDynamicConfig)
        self._config_multirotor.backends = [simple_controller.SimpleController(
            trajectory_file=None, # self.curr_dir + "/trajectories/pitch_relay_90_deg_2.csv",
            results_file=curr_dir + "/single_statistics.npz",
            Kp=[1, 1, 1],
            Kd=[0.1,0.1, 0.1],
            Ki=[0.1, 0.1, 0.1],
            Kr=[2.0, 2.0, 2.0],
            verbose = False,
        )]

        self.multimotor = Multirotor(
            self.pg._world,
            "/World/quadrotor",
            ROBOTS_ASSETS+"/drone_models/cross_drone13_v10_mani_gripper.usd", #cross_drone13_v10_mani_gripper.usd", #cross_drone13_v7.usd", 
            0,
            [0, 0, 2],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=self._config_multirotor,
            track_object_name="pick_box",
            keyboard_control=True,
            position_controller=True,
            verbose = False,    
        )

        # We add the task to the world here
        self.pg._world.add_task(RLTask(name="drone",robot=self.multimotor))

    # async def setup_post_load(self):
    #     # self.world.add_physics_callback("get_ref", callback_fn=self.get_keyboard_input) #callback names have to be unique

    #     # Reset the simulation environment so that all articulations (aka robots) are initialized
    #     # self.world.reset()
    #     # print("multirotor print prim stage prefix : ", self.multimotor.stage_prefix)

    #     # Auxiliar variable for the timeline callback example
    #     self.stop_sim = False
    #     self.world.reset()
    #     return
#
    async def setup_post_reset(self):
        await self.world.play_async()
        return

    

