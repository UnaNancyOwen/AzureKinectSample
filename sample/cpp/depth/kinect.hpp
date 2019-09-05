#ifndef __KINECT__
#define __KINECT__

#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>

class kinect
{
private:
    // Kinect
    k4a::device device;
    k4a::capture capture;
    k4a_device_configuration_t device_configuration;
    uint32_t device_index;

    // Depth
    k4a::image depth_image;
    cv::Mat depth;

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

    // Update Depth
    void update_depth();

    // Draw Depth
    void draw_depth();

    // Show Depth
    void show_depth();
};

#endif // __KINECT__
