import numpy as np
import cv2 as cv
import k4a

class kinect:
    # Kinect
    _device = None
    _capture = None

    # Infrared
    _infrared_image = None
    _infrared = None

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
        device_config = k4a.DEVICE_CONFIG_BGRA32_1080P_NFOV_UNBINNED_FPS15
        self._device.start_cameras(device_config)

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
        self.update_infrared()

    def update_frame(self):
        # Capture Frame
        self._capture = self._device.get_capture(-1)
        if self._capture is None:
            raise IOError("failed get capture!")
    
    def update_infrared(self):
        # Update Infrared
        self._infrared_image = self._capture.ir

    def draw(self):
        self.draw_infrared()

    def draw_infrared(self):
        # Draw Infrared
        self._infrared = self._infrared_image.data

    def show(self):
        self.show_infrared()

    def show_infrared(self):
        # Scaling Infrared
        self._infrared = self._infrared * 0.5
        self._infrared = self._infrared.astype(np.uint8)

        # Show Infrared
        cv.imshow("Infrared", self._infrared)

if __name__ == '__main__':
    try:
        kinect = kinect()
        kinect.run()
    except Exception as e:
        print(e)