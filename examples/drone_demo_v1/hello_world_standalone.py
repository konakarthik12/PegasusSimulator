
import numpy as np
from scipy.spatial.transform import Rotation
import os
from pathlib import Path
from PIL import Image

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Imports to be able to log to the terminal with fancy colors
import carb
from carb.input import KeyboardInput, KeyboardEventType
# Imports from the Pegasus library
from omni.isaac.core.objects import DynamicCuboid

from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.backends import simple_controller

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, ROBOTS_ASSETS, ENV_ASSETS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface


import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
from omni.kit.viewport.utility import get_active_viewport_window
from omni.isaac.core.utils.extensions import enable_extension

from scipy.spatial.transform import Rotation
from omni.isaac.core.tasks import BaseTask
from pathlib import Path


# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.



# class RLTask(BaseTask):

#     #NOTE: we only cover here a subset of the task functions that are available,
#     # checkout the base class for all the available functions to override.
#     # ex: calculate_metrics, is_done..etc.
#     def __init__(self, name, robot):
#         super().__init__(name=name)
#         self._goal_position = np.array([-0.3, -0.3, 0.0515 / 2.0])
#         self._task_achieved = False

#         self.drone = robot
#         self.reference =  {"p" : np.zeros((3,)), "v" : np.zeros((3,)), "a" : np.zeros((3,))}
#         return

#     # Here we setup all the assets that we care about in this task.
#     def set_up_scene(self, scene):

#         super().set_up_scene(scene)

#         scene.add_default_ground_plane()
#         self._cube = scene.add(DynamicCuboid(
#                 prim_path="/World/random_cube", # The prim path of the cube in the USD stage
#                 name="pick_box", # The unique name used to retrieve the object from the scene later on
#                 position=np.array([-2, 2, 0.1]), # Using the current stage units which is in meters by default.
#                 scale=np.array([0.02, 0.02, 0.02]), # most arguments accept mainly numpy arrays.
#                 color=np.array([0, 0, 1.0]), # RGB channels, going from 0-1
#             ))

#         return


#     # Information exposed to solve the task is returned from the task through get_observations
#     def get_observations(self):
#         cube_position, _ = self._cube.get_world_pose()
        
#         observations = {
#             "Robot" : self.drone._state.position,
#             self._cube.name: {
#                 "position": cube_position,
#                 "goal_position": self._goal_position
#             }
#         }
#         return observations

#     # Called before each physics step,
#     # for instance we can check here if the task was accomplished by
#     # changing the color of the cube once its accomplished
#     def pre_step(self, control_index, simulation_time):
#         if self.drone.track_position:
#             position, orientation = self._cube.get_world_pose()
#             self.reference["p"] = position
#             self.reference["p"][1] -= 0.03

#             # first goto x,y coordinate then go down to grab the object

#             print("reference Position : ",self.reference["p"][:2], " | state position : ", self.drone._state.position[:2])
#             # print("X Y position match  : ", np.allclose(self.reference["p"][:2], self.drone._state.position[:2], atol=1e-2))
#             if np.allclose(self.reference["p"][:2], self.drone._state.position[:2], atol=1e-2):
#                 # if not self.reach_pick_position:
#                 #     self.prev_time = time.perf_counter()
#                 # # if prev_time = None:
#                 # if time.perf_counter()-self.prev_time >=0.01:
#                 self.reference["p"][2]+=0.2
#                 # self.reach_pick_position = True
#             else:
#                 # self.prev_time = time.perf_counter()
#                 self.reference["p"][2]+=1
            
#         self.drone.update_target_reference(self.reference)
#         return

#     # Called after each reset,
#     # for instance we can always set the gripper to be opened at the beginning after each reset
#     # also we can set the cube's color to be blue
#     def post_reset(self):
#         self._cube.get_applied_visual_material().set_color(color=np.array([0, 0, 1.0]))
#         self._task_achieved = False
#         self.reference =  {"p" : np.zeros((3,)), "v" : np.zeros((3,)), "a" : np.zeros((3,))}
#         self.drone.reset()
#         return



class App():
    def __init__(self):
        # Start the Pegasus Interface
        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg._world
        self.world.scene.add_default_ground_plane()
        # self.pg.load_environment(ENV_ASSETS + "/fluid_test_2.usd")
        
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
            Kr=[2.0, 2.0, 2.0],
            verbose=False
        )]

        self.cube_pos = np.array([2.0, 1.0, 0])
        multi_pos = self.cube_pos.copy()
        multi_pos[2] = 2
        self.multirotor = Multirotor(
            self.pg._world,
            "/World/quadrotor",
            ROBOTS_ASSETS+"/drone_models/cross_drone13_v10_mani_gripper.usd", #cross_drone13_v10_mani_gripper.usd", #cross_drone13_v7.usd", 
            0,
            multi_pos,
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=self._config_multirotor,
            track_object_name="pick_box",
            keyboard_control=False,
            position_controller=True,
            verbose = False,    
        )

        self.reference =  {"p" : np.zeros((3,)), "v" : np.zeros((3,)), "a" : np.zeros((3,)), "yaw": 0}
        self.reference["p"] = self.multirotor._state.grippers_position
        self.reach_goal = False
        self.task_completed = False
        self.reach_cnt = None
        self.take_pictures = True
        self.prev_dist = 10000


        self._cube = self.world.scene.add(DynamicSphere(
                prim_path="/World/random_sphere", # The prim path of the cube in the USD stage
                name="pick_ball", # The unique name used to retrieve the object from the scene later on
                position=self.cube_pos, # Using the current stage units which is in meters by default.
                color=np.array([0,  0, 1.0]), # RGB channels, going from 0-1
                radius=0.005,
                mass = 0.00001
            ))

        # We add the task to the world here
        # self.pg._world.add_task(RLTask(name="drone",robot=self.multirotor))

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.pg._world.reset()

    def run(self):
        # Start the simulation
        self.pg._world.play()

        # The "infinite" loop
        for i in range(3000):
        # while simulation_app.is_running():
            
            # Update the UI of the app and perform the physics step
            sim_time = self.pg._world.current_time
            # self.softpolicy(sim_time)
            self.hardpolicy(sim_time)
            self.pg._world.step(render=True)

            if self.task_completed:
                break

        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.pg._world.stop()
        self.pg._world.reset()

    def close():
        simulation_app.close()

    def hardpolicy(self, simulation_time):
        """
        This function controls the behavior of the drone during the simulation.
        It updates the reference position of the drone based on the current state of the simulation.
        
        Args:
            simulation_time (float): The current time of the simulation.
        """
        # Print various information for debugging purposes
        print("-------------")
        print("Simulation Time: ", simulation_time)
        print("Reach Count: ", self.reach_cnt)
        print("Take Pictures: ", self.take_pictures)
        print("Prev Dist: ", self.prev_dist)
        print("Reference: ", self.reference)
        print("Drone position: ",self.multirotor._state.position)
        print("Task completed:, ",self.task_completed)
        print("path : ",ROBOTS_ASSETS+"/drone_models/cross_drone13_v10_mani_gripper.usd")

        # Check if the drone is tracking the position
        if self.multirotor.track_position and not self.task_completed :
            print("Tracking Position")
            
            # Get the position and orientation of the cube
            position, orientation = self._cube.get_world_pose() 
            position_gripper = self.multirotor._state.grippers_position
            position_drone = self.multirotor._state.position
            print("Position Gripper: ", position_gripper)

            # Check if the drone needs to set the initial reference
            if self.reach_cnt is None:
                self.reference["p"] = position_drone #position_gripper # Keep the position gripper on top of the cube
                self.reference["p"][2] = 1.0 #position_gripper
                self.reach_cnt = simulation_time
                print("Setting Initial Reference: ", self.reference)

            # Check if the drone needs to update the reference position
            if simulation_time-self.reach_cnt > 0.01:
                new_dist = np.linalg.norm(self.multirotor._state.grippers_position - self.cube_pos)
                if self.prev_dist - new_dist >= 0:
                    print("New Reference: ", self.reference, "| Position : ", position_gripper)
                    self.prev_dist = new_dist

                # Check if the drone has reached the goal position
                if position_gripper[2] <= 0.02:
                    self.reference["p"][2] += 0.003
                    self.multirotor.open_gripper = False
                    self.multirotor.close_gripper = True
                    self.reach_goal = True
                    print("Reached Goal")

            # Check if the drone needs to adjust the reference position
            if simulation_time-self.reach_cnt>.1:
                if not self.reach_goal:
                    self.reference["p"][2] -= 0.01
                    self.reach_cnt = simulation_time
                    print("New Reference: ", self.reference)
                else:
                    self.reference["p"][2] += 0.003
                    print("New Reference: ", self.reference)
            # Check if the drone has completed the task
            if self.reach_goal and np.allclose(self.reference["p"][2], 1, atol=1e-3) and position[2]>0.5:
                print("Task Completed")
                self.task_completed = True

            print("Reference: ", self.reference)
               
        # Update the target reference of the drone
        self.multirotor.update_target_reference(self.reference)
        print("Update Target Reference: ", self.reference)
        print("-------------")
        return
    
    # for few seconds run the velocity controller and then change it to position controller
    def softpolicy(self, simulation_time):
        if simulation_time < 10:
            self.multirotor.change_to_velocity_controller()
        else:
            self.multirotor.change_to_position_controller()

def main():
    app = App()

    for i in range(10):
        app.run()


if __name__ == "__main__":
    main()