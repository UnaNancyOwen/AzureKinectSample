
import numpy as np
import cv2 as cv
import k4a

class kinect:
    # Kinect
    _device = None
    _capture = None
    _transform = None

    # Color
    _color_image = None
    _color = None

    # Depth
    _depth_image = None
    _depth = None

    # Transformed
    _transformed_color_image = None
    _transformed_depth_image = None
    _transformed_color = None
    _transformed_depth = None

    def __init__(self):
        self.initialize()

    def __del__(self):
        self.finalize()

    def initialize(self):
        self.initialize_sensor()

    def finalize(self):
        self.finalize_sensor()

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
        status = self._device.start_cameras(device_config)
        if status != k4a.EStatus.SUCCEEDED:
            raise IOError("failed start cameras!")

        # Get Calibration
        calibration = self._device.get_calibration(
            depth_mode=device_config.depth_mode,
            color_resolution=device_config.color_resolution)

        # Create Transformation
        self._transformation = k4a.Transformation(calibration)

    def finalize_sensor(self):
        # Stop Cameras
        self._device.stop_cameras()

    def run(self):
        while True:
            self.update()
            self.draw()
            self.show()

            key = cv.waitKey(10)
            if key == ord('q'):
                break

    def update(self):
        self.update_frame()
        self.update_color()
        self.update_depth()
        self.update_transformation()

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
        self._transformed_color_image = self._transformation.color_image_to_depth_camera(self._depth_image, self._color_image)
        self._transformed_depth_image = self._transformation.depth_image_to_color_camera(self._depth_image)

    def draw(self):
        self.draw_color()
        self.draw_depth()
        self.draw_transformation()

    def draw_color(self):
        # Draw Color
        self._color = self._color_image.data

    def draw_depth(self):
        # Draw Depth
        self._depth = self._depth_image.data

    def draw_transformation(self):
        # Draw Transformation
        self._transformed_color = self._transformed_color_image.data
        self._transformed_depth = self._transformed_depth_image.data

    def show(self):
        self.show_color()
        self.show_depth()
        self.show_transformation()

    def show_color(self):
        # Show Color
        cv.imshow("color", self._color)

    def show_depth(self):
        # Scaling Depth
        self._depth = self._depth * (-255.0 / 5000.0) + 255.0
        self._depth = self._depth.astype(np.uint8)

        # Show Depth
        cv.imshow("depth", self._depth)

    def show_transformation(self):
        # Scaling Transformed Depth
        self._transformed_depth = self._transformed_depth * (-255.0 / 5000.0) + 255.0
        self._transformed_depth = self._transformed_depth.astype(np.uint8)

        # Show Transformation
        cv.imshow("transformed color", self._transformed_color)
        cv.imshow("transformed depth", self._transformed_depth)

if __name__ == '__main__':
    try:
        kinect = kinect()
        kinect.run()
    except Exception as e:
        print(e)
