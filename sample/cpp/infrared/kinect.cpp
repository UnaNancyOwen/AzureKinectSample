#include "kinect.hpp"
#include "util.h"

#include <chrono>

// Constructor
kinect::kinect( const uint32_t index )
    : device_index( index )
{
    // Initialize
    initialize();
}

kinect::~kinect()
{
    // Finalize
    finalize();
}

// Initialize
void kinect::initialize()
{
    // Initialize Sensor
    initialize_sensor();
}

// Initialize Sensor
inline void kinect::initialize_sensor()
{
    // Get Connected Devices
    const int32_t device_count = k4a::device::get_installed_count();
    if( device_count == 0 ){
        throw k4a::error( "Failed to found device!" );
    }

    // Open Default Device
    device = k4a::device::open( device_index );

    // Start Cameras with Configuration
    device_configuration = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    device_configuration.color_format             = k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32;
    device_configuration.color_resolution         = k4a_color_resolution_t::K4A_COLOR_RESOLUTION_720P;
    device_configuration.depth_mode               = k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED;
    device_configuration.synchronized_images_only = true;
    device_configuration.wired_sync_mode          = k4a_wired_sync_mode_t::K4A_WIRED_SYNC_MODE_STANDALONE;
    device.start_cameras( &device_configuration );
}

// Finalize
void kinect::finalize()
{
    // Stop Cameras
    device.stop_cameras();

    // Close Device
    device.close();

    // Close Window
    cv::destroyAllWindows();
}

// Run
void kinect::run()
{
    // Main Loop
    while( true ){
        // Update
        update();

        // Draw
        draw();

        // Show
        show();

        // Wait Key
        constexpr int32_t delay = 30;
        const int32_t key = cv::waitKey( delay );
        if( key == 'q' ){
            break;
        }
    }
}

// Update
void kinect::update()
{
    // Update Frame
    update_frame();

    // Update Infrared
    update_infrared();

    // Release Capture Handle
    capture.reset();
}

// Update Frame
inline void kinect::update_frame()
{
    // Get Capture Frame
    constexpr std::chrono::milliseconds time_out( K4A_WAIT_INFINITE );
    const bool result = device.get_capture( &capture, time_out );
    if( !result ){
        this->~kinect();
    }
}

// Update Infrared
inline void kinect::update_infrared()
{
    // Get Infrared Image
    infrared_image = capture.get_ir_image();
}

// Draw
void kinect::draw()
{
    // Draw Infrared
    draw_infrared();
}

// Draw Infrared
inline void kinect::draw_infrared()
{
    if( !infrared_image.handle() ){
        return;
    }

    // Get cv::Mat from k4a::image
    infrared = k4a::get_mat( infrared_image );

    // Release Infrared Image Handle
    infrared_image.reset();
}

// Show
void kinect::show()
{
    // Show Infrared
    show_infrared();
}

// Show Infrared
inline void kinect::show_infrared()
{
    if( infrared.empty() ){
        return;
    }

    // Scaling Infrared
    infrared.convertTo( infrared, CV_8U, 0.5 );

    // Show Image
    const cv::String window_name = cv::format( "infrared (kinect %d)", device_index );
    cv::imshow( window_name, infrared );
}
