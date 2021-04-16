import numpy as np
import cv2 as cv
import k4a

class kinect:
    # Kinect
    _device = None
    _capture = None

    # Color
    _color_image = None
    _color = None

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

    def finalize_sensor(self):
        # Stop Cameras
        self._device.stop_cameras()
        self._device.close()

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

    def update_frame(self):
        # Capture Frame
        self._capture = self._device.get_capture(-1)
        if self._capture is None:
            raise IOError("failed get capture!")

    def update_color(self):
        # Update Color
        self._color_image = self._capture.color

    def draw(self):
        self.draw_color()

    def draw_color(self):
        # Draw Color
        self._color = self._color_image.data

    def show(self):
        self.show_color()

    def show_color(self):
        # Show Color
        cv.imshow("color", self._color)

if __name__ == '__main__':
    try:
        kinect = kinect()
        kinect.run()
    except Exception as e:
        print(e)
