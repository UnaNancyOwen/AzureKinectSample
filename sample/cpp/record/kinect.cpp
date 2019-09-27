#include "kinect.hpp"
#include "util.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <ostream>

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

    // Initialize Record
    initialize_record();
}

// Initialize Sensor
inline void kinect::initialize_sensor()
{
    // Get Connected Devices
    const int32_t device_count = k4a::device::get_installed_count();
    if( device_count == 0 ){
        throw k4a::error( "Failed to found device!" );
    }

    // Open Device
    device = k4a::device::open( device_index );

    // Start Cameras with Configuration
    device_configuration = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    device_configuration.color_format             = k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_MJPG;
    device_configuration.color_resolution         = k4a_color_resolution_t::K4A_COLOR_RESOLUTION_720P;
    device_configuration.depth_mode               = k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED;
    device_configuration.synchronized_images_only = true;
    device_configuration.wired_sync_mode          = k4a_wired_sync_mode_t::K4A_WIRED_SYNC_MODE_STANDALONE;
    device.start_cameras( &device_configuration );
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
    record = k4a::record::create( record_file.generic_string().c_str(), device, device_configuration );
    std::cout << record_file.generic_string().c_str() << std::endl;

    // Write Header
    record.write_header();
}

// Finalize
void kinect::finalize()
{
    // Flash Record
    record.flush();

    // Close Record
    record.close();

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
    while (true){
        // Update
        update();

        // Draw
        draw();

        // Show
        show();

        // Wait Key
        constexpr int32_t delay = 1;
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
    capture.reset();
}

// Update Frame
inline void kinect::update_frame()
{
    // Get Capture Frame
    constexpr std::chrono::milliseconds time_out( K4A_WAIT_INFINITE );
    const bool result = device.get_capture( &capture, time_out );
    if( !result ){
        throw k4a::error( "Failed to capture!" );
    }
}

// Write Frame
inline void kinect::write_frame()
{
    // Write Capture Frame
    record.write_capture( capture );
}

// Update Color
inline void kinect::update_color()
{
    // Get Color Image
    color_image = capture.get_color_image();
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
    // Draw Color
    draw_color();

    // Draw Depth
    draw_depth();
}

// Draw Color
inline void kinect::draw_color()
{
    if( !color_image.handle() ){
        return;
    }

    // Get cv::Mat from k4a::image
    color = k4a::get_mat( color_image );

    // Release Color Image Handle
    color_image.reset();
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
