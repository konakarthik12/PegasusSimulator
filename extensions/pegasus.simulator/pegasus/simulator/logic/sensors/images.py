from omni.kit.viewport.utility import get_active_viewport_window, get_active_viewport, get_viewport_from_window_name
from omni.isaac.core.utils.extensions import enable_extension
from PIL import Image
import omni
from  omni.isaac.core.utils.viewports import create_viewport_for_camera
class Camera:
    def __init__(self, cam_paths =["quadrotor/body/Down_Camera"]):
        self.cam_paths = cam_paths
        self.sensor_names = ["rgb", "depth",]

    def setup(self):
        
        enable_extension("omni.isaac.synthetic_utils")
        from omni.isaac.synthetic_utils import SyntheticDataHelper

        self.sd_helper = SyntheticDataHelper()
        
        # Reference for multiple camera : https://forums.developer.nvidia.com/t/accessing-camera-from-python/237161/6
        self.viewports = []

        # for using viewport
        ## Reference : https://docs.omniverse.nvidia.com/kit/docs/omni.kit.viewport.docs/latest/viewport_api.html#
        
        window_names = ["Viewport", "Viewport 2"]

        viewport_apis = [get_viewport_from_window_name(window_name) for window_name in window_names]
        viewport_windows = [omni.ui.Workspace.get_window(window_name) for window_name in window_names]

        for idx in range(len(window_names)): 
            viewport_window, viewport = (viewport_windows[idx], viewport_apis[idx])
            if viewport_window is not None:
                viewport_window.visible = True
                viewport.set_active_camera(self.cam_paths[idx])
            else:
                viewport_window = create_viewport_for_camera(window_names[idx],self.cam_paths[idx],width = 500, height = 500)
                print("created viewport with window name : ",window_names[idx])
                viewport_window.visible = True
                viewport = get_viewport_from_window_name(window_names[idx])
                viewport.set_active_camera(self.cam_paths[idx])
            viewport.resolution = (128, 128)
            # self.viewport.set_window_pos(1000, 400)
            # self.viewport.set_window_size(420, 420)

            # # initialize sensors first as it can take several frames before they are ready
            self.sd_helper.initialize(sensor_names=self.sensor_names, viewport_api=viewport)
            self.viewports.append(viewport)

    def get_image(self):
        rgb_images = []
        depth_images = []
        for viewport in self.viewports:
            gt = self.sd_helper.get_groundtruth(self.sensor_names,viewport,verify_sensor_init=False, wait_for_sensor_data=0)
            depth = gt["depth"]
            rgb = gt["rgb"]
            rgb_images.append(rgb[:,:,:3])
        return rgb_images

    def save_image_rgb(self,rgb, name):
        image = Image.fromarray(rgb, 'RGB')

        # Save the image
        image_path = f'{name}.png'
        image.save(image_path)

    def save_image_depth(self,depth):
        depth_normalized = (255 * (depth - np.min(depth)) / (np.max(depth) - np.min(depth))).astype(np.uint8)
        image = Image.fromarray(depth_normalized, "L")

        # Save the image
        image_path = 'depth.png'
        image.save(image_path)