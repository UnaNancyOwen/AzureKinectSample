#ifndef __KINECT__
#define __KINECT__

#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include <opencv2/opencv.hpp>

#include <vector>

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

    // Body Tracking
    k4abt::tracker tracker;
    k4abt::frame frame;

    // Body Index Map
    k4a::image body_index_map_image;
    cv::Mat body_index_map;

    // Transformed
    k4a::image transformed_body_index_map_image;
    cv::Mat transformed_body_index_map;

    // Visualize
    std::vector<cv::Vec3b> colors;

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

    // Initialize Body Tracking
    void initialize_body_tracking();

    // Finalize
    void finalize();

    // Update Frame
    void update_frame();

    // Update Color
    void update_color();

    // Update Depth
    void update_depth();

    // Update Body Tracking
    void update_body_tracking();

    // Update Body Index Map
    void update_body_index_map();

    // Update Transformation
    void update_transformation();

    // Draw Color
    void draw_color();

    // Draw Body Index Map
    void draw_body_index_map();

    // Draw Transformation
    void draw_transformation();

    // Show Body Index Map
    void show_body_index_map();

    // Show Transformation
    void show_transformation();
};

#endif // __KINECT__
