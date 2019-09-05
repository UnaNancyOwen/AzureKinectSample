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

    // Infrared
    k4a::image infrared_image;
    cv::Mat infrared;

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

    // Update Infrared
    void update_infrared();

    // Draw Infrared
    void draw_infrared();

    // Show Infrared
    void show_infrared();
};

#endif // __KINECT__
