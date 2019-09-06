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
    k4a::calibration calibration;
    k4a::transformation transformation;
    k4a_device_configuration_t device_configuration;
    uint32_t device_index;

    // Color
    k4a::image color_image;
    cv::Mat color;

    // Depth
    k4a::image depth_image;
    cv::Mat depth;

    // Transformed
    k4a::image transformed_color_image;
    k4a::image transformed_depth_image;
    cv::Mat transformed_color;
    cv::Mat transformed_depth;

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

    // Update Depth
    void update_depth();

    // Update Transformation
    void update_transformation();

    // Draw Color
    void draw_color();

    // Draw Depth
    void draw_depth();

    // Draw Transformation
    void draw_transformation();

    // Show Color
    void show_color();

    // Show Depth
    void show_depth();

    // Show Transformation
    void show_transformation();
};

#endif // __KINECT__
