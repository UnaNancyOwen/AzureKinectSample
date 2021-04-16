import numpy as np
import open3d as o3d
import k4a

class kinect:
    # Kinect
    _device = None
    _capture = None
    _transform = None

    # Color
    _color_image = None

    # Depth
    _depth_image = None

    # Transformed
    #_transformed_color_image = None
    _transformed_depth_image = None

    # Point Cloud
    _xyz_image = None

    # Open3D
    _visualizer = None
    _add_geometory = True
    _point_cloud = None
    _is_run = True

    def __init__(self):
        self.initialize()

    def __del__(self):
        self.finalize()

    def initialize(self):
        self.initialize_sensor()
        self.initialize_visualize()

    def finalize(self):
        self.finalize_sensor()
        self.finalize_visualize()

    def initialize_sensor(self, device_index=0):
        # Open Device
        self._device = k4a.Device.open(device_index)

        # Start Cameras
        device_config = k4a.DeviceConfiguration(
            color_format=k4a.EImageFormat.COLOR_BGRA32,
            color_resolution=k4a.EColorResolution.RES_720P,
            depth_mode=k4a.EDepthMode.NFOV_UNBINNED,
            camera_fps=k4a.EFramesPerSecond.FPS_30,
            synchronized_images_only=True
        )
        device_config.color_resolution = k4a.EColorResolution.RES_720P
        status = self._device.start_cameras(device_config)
        if status != k4a.EStatus.SUCCEEDED:
            raise IOError("failed start cameras!")

        # Get Calibration
        calibration = self._device.get_calibration(
            depth_mode=device_config.depth_mode,
            color_resolution=device_config.color_resolution)

        # Create Transformation
        self._transformation = k4a.Transformation(calibration)

    def initialize_visualize(self):
        # Create Visualizer
        self._visualizer = o3d.visualization.VisualizerWithKeyCallback()
        self._visualizer.create_window(window_name="point cloud", width=1280, height=720)

        # Register Keyboard Event Callback
        self._visualizer.register_key_callback(ord('Q'), self.keyboad_callback)

        # Create Geometry
        self._point_cloud = o3d.geometry.PointCloud()

    def keyboad_callback(self, visualizer):
        self._is_run = False

    def finalize_sensor(self):
        # Stop Cameras
        self._device.stop_cameras()

    def finalize_visualize(self):
        # Close Visualizer
        self._visualizer.destroy_window()

    def run(self):
        self._is_run = True

        while self._is_run:
            self.update()
            self.draw()
            self.show()

    def update(self):
        self.update_frame()
        self.update_color()
        self.update_depth()
        self.update_transformation()
        self.update_point_cloud()

    def update_frame(self):
        # Capture Frame
        self._capture = self._device.get_capture(-1)
        if self._capture is None:
            raise IOError("failed get capture!")
    
    def update_color(self):
        # Update Color
        self._color_image = self._capture.color

    def update_depth(self):
        # Update Depth
        self._depth_image = self._capture.depth

    def update_transformation(self):
        # Update Transformation
        #self._transformed_color_image = self._transformation.color_image_to_depth_camera(self._depth_image, self._color_image)
        self._transformed_depth_image = self._transformation.depth_image_to_color_camera(self._depth_image)

    def update_point_cloud(self):
        #self._xyz_image = self._transformation.depth_image_to_point_cloud(self._transformed_color_image, k4a.ECalibrationType.DEPTH)
        self._xyz_image = self._transformation.depth_image_to_point_cloud(self._transformed_depth_image, k4a.ECalibrationType.COLOR)

    def draw(self):
        self.draw_point_cloud()

    def draw_point_cloud(self):
        # Draw Point Cloud
        # Points
        points = self._xyz_image.data
        height, width, channels = points.shape            # get height, width and channels from shape
        points = points.reshape(height * width, channels) # convert shape to (height * width, channels) from (height, width, channels)
        points = points.astype(np.float64)                # convert dtype to float64 from uint16
        self._point_cloud.points = o3d.utility.Vector3dVector(points)
        # Colors
        #colors = self._transformed_color_image.data[:,:,2::-1]
        colors = self._color_image.data[:,:,2::-1]        # convert channels order to RGB fron BGRA
        height, width, channels = colors.shape            # get height, width and channels from shape
        colors = colors.reshape(height * width, channels) # convert shape to (height * width, channels) from (height, width, channels)
        colors = colors.astype(np.float64) / 255.0        # convert dtype to float64 from uint8 and convert values to 0.0-1.0 fron 0-255
        self._point_cloud.colors = o3d.utility.Vector3dVector(colors)

    def show(self):
        self.show_point_cloud()

    def show_point_cloud(self):
        # Show Point Cloud
        if self._add_geometory:
            self._visualizer.add_geometry(self._point_cloud)
            self._add_geometory = False

        self._visualizer.update_geometry(self._point_cloud)
        self._visualizer.poll_events()
        self._visualizer.update_renderer()

if __name__ == '__main__':
    try:
        kinect = kinect()
        kinect.run()
    except Exception as e:
        print(e)
