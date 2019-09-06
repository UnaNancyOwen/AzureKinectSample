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

// Constructor
kinect::kinect( const filesystem::path path )
    : playback_file( path )
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
    if( playback_file.empty() ){
        // Initialize Sensor
        initialize_sensor();
    }
    else{
        // Initialize Playback
        initialize_playback();
    }
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
    device_configuration.color_format             = k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32;
    device_configuration.color_resolution         = k4a_color_resolution_t::K4A_COLOR_RESOLUTION_720P;
    device_configuration.depth_mode               = k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED;
    device_configuration.synchronized_images_only = true;
    device_configuration.wired_sync_mode          = k4a_wired_sync_mode_t::K4A_WIRED_SYNC_MODE_STANDALONE;
    device.start_cameras( &device_configuration );

    // Get Calibration
    calibration = device.get_calibration( device_configuration.depth_mode, device_configuration.color_resolution );

    // Create Transformation
    transformation = k4a::transformation( calibration );
}

// Initialize Playback
inline void kinect::initialize_playback()
{
    if( !filesystem::is_regular_file( playback_file ) || !filesystem::exists( playback_file ) ){
        throw k4a::error( "Failed to found file path!" );
    }

    // Open Playback
    playback = k4a::playback::open( playback_file.generic_string().c_str() );

    // Get Calibration
    calibration = playback.get_calibration();

    // Create Transformation
    transformation = k4a::transformation( calibration );
}

// Finalize
void kinect::finalize()
{
    // Destroy Transformation
    transformation.destroy();

    if( playback_file.empty() ){
        // Stop Cameras
        device.stop_cameras();

        // Close Device
        device.close();
    }
    else{
        // Close Playback
        playback.close();
    }

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

    // Update Color
    update_color();

    // Update Depth
    update_depth();

    // Update Transformation
    update_transformation();

    // Release Capture Handle
    capture.reset();
}

// Update Frame
inline void kinect::update_frame()
{
    // Get Capture Frame
    if( playback_file.empty() ){
        constexpr std::chrono::milliseconds time_out( K4A_WAIT_INFINITE );
        const bool result = device.get_capture( &capture, time_out );
        if( !result ){
            throw k4a::error( "Failed to capture!" );
        }
    }
    else{
        const bool result = playback.get_next_capture( &capture );
        if( !result ){
            // EOF
            std::exit( EXIT_SUCCESS );
        }
    }
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

// Update Transformation
inline void kinect::update_transformation()
{
    if( !color_image.handle() || !depth_image.handle() ){
        return;
    }

    if( playback_file.empty() ){
        // Transform Color Image to Depth Camera
        transformed_color_image = transformation.color_image_to_depth_camera( depth_image, color_image );
    }
    else{
        // Decode Motion JPEG, and Create Color Image from Buffer
        color = k4a::get_mat( color_image );
        k4a::image color_image = k4a::image::create_from_buffer( k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32, color.cols, color.rows, static_cast<int32_t>( color.step ), &color.data[0], static_cast<int32_t>( color.total() * color.elemSize() ), nullptr, nullptr );

        // Transform Color Image to Depth Camera
        transformed_color_image = transformation.color_image_to_depth_camera( depth_image, color_image );
    }

    // Transform Depth Image to Color Camera
    transformed_depth_image = transformation.depth_image_to_color_camera( depth_image );
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

// Draw Transformation
inline void kinect::draw_transformation()
{
    if( !transformed_color_image.handle() || !transformed_depth_image.handle() ){
        return;
    }

    // Get cv::Mat from k4a::image
    transformed_color = k4a::get_mat( transformed_color_image );
    transformed_depth = k4a::get_mat( transformed_depth_image );

    // Release Transformed Image Handle
    transformed_color_image.reset();
    transformed_depth_image.reset();
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
