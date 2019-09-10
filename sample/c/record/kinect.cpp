#include "kinect.hpp"
#include "util.h"

#include <sstream>
#include <stdexcept>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <ostream>

// Error Check Macro
#define K4A_RESULT_CHECK( ret )                                   \
    if( K4A_FAILED( ret ) ){                                      \
        std::stringstream ss;                                     \
        ss << "failed " #ret " " << std::hex << ret << std::endl; \
        throw std::runtime_error( ss.str().c_str() );             \
    }

// Constructor
kinect::kinect( const uint32_t index )
    : device_index( index ), 
      device( nullptr ),
      record( nullptr ),
      capture( nullptr ),
      color_image( nullptr ),
      depth_image( nullptr )
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

    // Initialize Record
    initialize_record();
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
}

// Initialize Record
inline void kinect::initialize_record()
{
    // Generate Record File Name from Date (YYYY_MM_DD_hhmmss)
    const std::chrono::system_clock::time_point time_point = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t( time_point );
    const tm tm = *localtime( &time );

    std::ostringstream oss;
    oss << tm.tm_year + 1900   << "_"
        << std::setfill( '0' ) << std::setw( 2 ) << tm.tm_mon + 1 << "_"
        << std::setfill( '0' ) << std::setw( 2 ) << tm.tm_mday    << "_"
        << std::setfill( '0' ) << std::setw( 2 ) << tm.tm_hour
        << std::setfill( '0' ) << std::setw( 2 ) << tm.tm_min
        << std::setfill( '0' ) << std::setw( 2 ) << tm.tm_sec;

    // Create Record
    record_file = "./" + oss.str() + ".mkv";
    K4A_RESULT_CHECK( k4a_record_create( record_file.generic_string().c_str(), device, device_configuration, &record ) );

    // Write Header
    K4A_RESULT_CHECK( k4a_record_write_header( record ) );
}

// Finalize
void kinect::finalize()
{
    // Flush Record
    K4A_RESULT_CHECK( k4a_record_flush( record ) );

    // Close Record
    k4a_record_close( record );

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

    // Write Frame
    write_frame();

    // Update Color
    update_color();

    // Update Depth
    update_depth();

    // Release Capture Handle
    k4a_capture_release( capture );
}

// Update Frame
inline void kinect::update_frame()
{
    // Get Capture Frame
    constexpr std::chrono::milliseconds time_out( K4A_WAIT_INFINITE );
    const k4a_wait_result_t result = k4a_device_get_capture( device, &capture, static_cast<int32_t>( time_out.count() ) );
    if( result == K4A_WAIT_RESULT_FAILED ){
        throw std::runtime_error( "Failed to get capture from device!" );
    }
    else if( result == K4A_WAIT_RESULT_TIMEOUT ){
        this->~kinect();
    }
}

// Write Frame
inline void kinect::write_frame()
{
    if( !record || !capture ){
        return;
    }

    // Write Capture Frame
    K4A_RESULT_CHECK( k4a_record_write_capture( record, capture ) );
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

// Draw
void kinect::draw()
{
    // Draw Color
    draw_color();

    // Draw Depth
    draw_depth();
}

// Draw Color
inline void kinect::draw_color()
{
    if( !color_image ){
        return;
    }

    // Get cv::Mat from k4a::image
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

    // Get cv::Mat from k4a::image
    depth = k4a_get_mat( depth_image );

    // Release Depth Image Handle
    k4a_image_release( depth_image );
}

// Show
void kinect::show()
{
    // Show Color
    show_color();

    // Show Depth
    show_depth();
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
