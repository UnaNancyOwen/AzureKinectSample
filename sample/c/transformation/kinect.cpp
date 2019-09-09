#include "kinect.hpp"
#include "util.h"

#include <sstream>
#include <stdexcept>
#include <chrono>

// Error Check Macro
#define K4A_RESULT_CHECK( ret )                                             \
    if( K4A_FAILED( ret ) ){                                                \
        std::stringstream ss;                                               \
        ss << "Failed to " #ret " " << std::hex << ret << "!" << std::endl; \
        throw std::runtime_error( ss.str().c_str() );                       \
    }

// Constructor
kinect::kinect( const uint32_t index )
    : device_index( index ),
      device( nullptr ),
      capture( nullptr ),
      color_image( nullptr ),
      depth_image( nullptr ),
      transformed_color_image( nullptr ),
      transformed_depth_image( nullptr )
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
    const int32_t device_count = k4a_device_get_installed_count();
    if( device_count == 0 ){
        throw std::runtime_error( "Failed to found device!" );
    }

    // Open Default Device
    K4A_RESULT_CHECK( k4a_device_open( device_index, &device ) );

    // Start Cameras with Configuration
    device_configuration = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    device_configuration.color_format             = k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32;
    device_configuration.color_resolution         = k4a_color_resolution_t::K4A_COLOR_RESOLUTION_720P;
    device_configuration.depth_mode               = k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED;
    device_configuration.synchronized_images_only = true;
    device_configuration.wired_sync_mode          = k4a_wired_sync_mode_t::K4A_WIRED_SYNC_MODE_STANDALONE;
    K4A_RESULT_CHECK( k4a_device_start_cameras( device, &device_configuration ) );

    // Get Calibration
    k4a_device_get_calibration( device, device_configuration.depth_mode, device_configuration.color_resolution, &calibration );

    // Create Transformation
    transformation = k4a_transformation_create( &calibration );
}

// Finalize
void kinect::finalize()
{
    // Destroy Transformation
    k4a_transformation_destroy( transformation );

    // Stop Cameras
    k4a_device_stop_cameras( device );

    // Close Device
    k4a_device_close( device );

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

    // Update Color
    update_color();

    // Update Depth
    update_depth();

    // Update Transformation
    update_transformation();

    // Release Capture Handle
    k4a_capture_release( capture );
}

// Update Frame
inline void kinect::update_frame()
{
    // Get Capture Frame
    constexpr std::chrono::milliseconds time_out( K4A_WAIT_INFINITE );
    const k4a_wait_result_t result = k4a_device_get_capture( device, &capture, static_cast<int32_t>( time_out.count() ) );
    if( result == k4a_wait_result_t::K4A_WAIT_RESULT_FAILED ){
        throw std::runtime_error( "Failed to get capture from device!" );
    }
    else if( result == k4a_wait_result_t::K4A_WAIT_RESULT_TIMEOUT ){
        this->~kinect();
    }
}

// Update Color
inline void kinect::update_color()
{
    if( !capture ){
        return;
    }

    // Get Color Image
    color_image = k4a_capture_get_color_image( capture );
}

// Update Depth
inline void kinect::update_depth()
{
    if( !capture ){
        return;
    }

    // Get Depth Image
    depth_image = k4a_capture_get_depth_image( capture );
}

// Update Transformation
inline void kinect::update_transformation()
{
    if( !color_image || !depth_image ){
        return;
    }

    int32_t stride_bytes;

    // Transform Color Image to Depth Camera
    const int32_t depth_width  = k4a_image_get_width_pixels( depth_image );
    const int32_t depth_height = k4a_image_get_height_pixels( depth_image );
    stride_bytes = depth_width * 4 * static_cast<int32_t>( sizeof( uint8_t ) );
    K4A_RESULT_CHECK( k4a_image_create( k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32, depth_width, depth_height, stride_bytes, &transformed_color_image ) );
    K4A_RESULT_CHECK( k4a_transformation_color_image_to_depth_camera( transformation, depth_image, color_image, transformed_color_image ) );

    // Transform Depth Image to Color Camera
    const int32_t color_width  = k4a_image_get_width_pixels( color_image );
    const int32_t color_height = k4a_image_get_height_pixels( color_image );
    stride_bytes = color_width * static_cast<int32_t>( sizeof( uint16_t ) );
    K4A_RESULT_CHECK( k4a_image_create( k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16, color_width, color_height, stride_bytes, &transformed_depth_image ) );
    K4A_RESULT_CHECK( k4a_transformation_depth_image_to_color_camera( transformation, depth_image, transformed_depth_image ) );
}

// Draw
void kinect::draw()
{
    // Draw Color
    draw_color();

    // Draw Depth
    draw_depth();

    // Draw Transformation
    draw_transformation();
}

// Draw Color
inline void kinect::draw_color()
{
    if( !color_image ){
        return;
    }

    // Get cv::Mat from k4a_image_t
    color = k4a_get_mat( color_image );

    // Release Color Image Handle
    k4a_image_release( color_image );
}

// Draw Depth
inline void kinect::draw_depth()
{
    if( !depth_image ){
        return;
    }

    // Get cv::Mat from k4a_image_t
    depth = k4a_get_mat( depth_image );

    // Release Depth Image Handle
    k4a_image_release( depth_image );
}

// Draw Transformation
inline void kinect::draw_transformation()
{
    if( !transformed_color_image || !transformed_depth_image ){
        return;
    }

    // Get cv::Mat from k4a_image_t
    transformed_color = k4a_get_mat( transformed_color_image );
    transformed_depth = k4a_get_mat( transformed_depth_image );

    // Release Transformed Image Handle
    k4a_image_release( transformed_color_image );
    k4a_image_release( transformed_depth_image );
}

// Show
void kinect::show()
{
    // Show Color
    show_color();

    // Show Depth
    show_depth();

    // Show Transformation
    show_transformation();
}

// Show Color
inline void kinect::show_color()
{
    if( color.empty() ){
        return;
    }

    // Show Image
    const cv::String window_name = cv::format( "color (kinect %d)", device_index );
    cv::imshow( window_name, color );
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

// Show Transformation
inline void kinect::show_transformation()
{
    if( transformed_color.empty() || transformed_depth.empty() ){
        return;
    }

    // Scaling Depth
    transformed_depth.convertTo( transformed_depth, CV_8U, -255.0 / 5000.0, 255.0 );

    // Show Image
    cv::String window_name;
    window_name = cv::format( "transformed color (kinect %d)", device_index );
    cv::imshow( window_name, transformed_color );
    window_name = cv::format( "transformed depth (kinect %d)", device_index );
    cv::imshow( window_name, transformed_depth );
}
