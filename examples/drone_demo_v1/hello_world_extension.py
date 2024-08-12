import os

from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.user_examples import DroneDemoV1
# from omni.isaac.examples.hello_world.peg import HelloWorld


class DroneDemoV1Extension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="",
            submenu_name="",
            name="DroneDemoV1",
            title="Drone Demo V1",
            doc_link="https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html",
            overview="This Example introduces pegasus simulator with keyboard input from the user with Isaac Sim through scripting in asynchronous mode.",
            file_path=os.path.abspath(__file__),
            sample=DroneDemoV1(),
        )
        return