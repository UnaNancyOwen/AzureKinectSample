import numpy as np
import cv2 as cv
import k4a

class kinect:
    # Kinect
    _device = None
    _capture = None

    # Depth
    _depth_image = None
    _depth = None

    def __init__(self):
        self.initialize()

    def __del__(self):
        self.finalize()

    def initialize(self):
        self.initialize_sensor()

    def finalize(self):
        self.finalize_sensor()

    def initialize_sensor(self):
        # Open Device
        self._device = k4a.Device.open()

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
        self.update_depth()

    def update_frame(self):
        # Capture Frame
        self._capture = self._device.get_capture(-1)
        if self._capture is None:
            raise IOError("failed get capture!")

    def update_depth(self):
        # Update Depth
        self._depth_image = self._capture.depth

    def draw(self):
        self.draw_depth()

    def draw_depth(self):
        # Draw Depth
        self._depth = self._depth_image.data

    def show(self):
        self.show_depth()

    def show_depth(self):
        # Scaling Depth
        self._depth = self._depth * (-255.0 / 5000.0) + 255.0
        self._depth = self._depth.astype(np.uint8)

        # Show Depth
        cv.imshow("depth", self._depth)

if __name__ == '__main__':
    try:
        kinect = kinect()
        kinect.run()
    except Exception as e:
        print(e)