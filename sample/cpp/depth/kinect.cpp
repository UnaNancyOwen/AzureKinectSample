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

    // Update Depth
    update_depth();

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

// Update Depth
inline void kinect::update_depth()
{
    // Get Depth Image
    depth_image = capture.get_depth_image();
}

// Draw
void kinect::draw()
{
    // Draw Depth
    draw_depth();
}

// Draw Depth
inline void kinect::draw_depth()
{
    if( !depth_image.handle() ){
        return;
    }

    // Get cv::Mat from k4a::image
    depth = k4a::get_mat( depth_image );

    // Release Depth Image Handle
    depth_image.reset();
}

// Show
void kinect::show()
{
    // Show Depth
    show_depth();
}

// Show Depth
inline void kinect::show_depth()
{
    if( depth.empty() ){
        return;
    }

    // Scaling Depth
    depth.convertTo( depth, CV_8U, -255.0 / 5000.0, 255.0 );

    // Show Image
    const cv::String window_name = cv::format( "depth (kinect %d)", device_index );
    cv::imshow( window_name, depth );
}
