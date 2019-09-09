#ifndef __KINECT__
#define __KINECT__

#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>

class kinect
{
private:
    // Kinect
    k4a_device_t device;
    k4a_capture_t capture;
    k4a_device_configuration_t device_configuration;
    uint32_t device_index;

    // Color
    k4a_image_t color_image;
    cv::Mat color;

public:
    // Constructor
    kinect( const uint32_t index = K4A_DEVICE_DEFAULT );

    // Destructor
    ~kinect();

    // Run
    void run();

    // Update
    void update();

    // Draw
    void draw();

    // Show
    void show();

private:
    // Initialize
    void initialize();

    // Initialize Sensor
    void initialize_sensor();

    // Finalize
    void finalize();

    // Update Frame
    void update_frame();

    // Update Color
    void update_color();

    // Draw Color
    void draw_color();

    // Show Color
    void show_color();
};

#endif // __KINECT__
