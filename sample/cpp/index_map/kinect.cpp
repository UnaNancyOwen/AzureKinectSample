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

    // Initialize Body Tracking
    initialize_body_tracking();
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

    // Get Calibration
    calibration = device.get_calibration( device_configuration.depth_mode, device_configuration.color_resolution );

    // Create Transformation
    transformation = k4a::transformation( calibration );
}

// Initialize Body Tracking
inline void kinect::initialize_body_tracking()
{
    // Set Tracker Configuration
    k4abt_tracker_configuration_t tracker_configuration = K4ABT_TRACKER_CONFIG_DEFAULT;
    tracker_configuration.sensor_orientation = K4ABT_SENSOR_ORIENTATION_DEFAULT;

    // Create Tracker with Configuration
    tracker = k4abt::tracker::create( calibration, tracker_configuration );
    if( !tracker ){
        throw k4a::error( "Failed to create tracker!" );
    }

    // Create Color Table
    colors.push_back( cv::Vec3b( 255,   0,   0 ) );
    colors.push_back( cv::Vec3b(   0, 255,   0 ) );
    colors.push_back( cv::Vec3b(   0,   0, 255 ) );
    colors.push_back( cv::Vec3b( 255, 255,   0 ) );
    colors.push_back( cv::Vec3b(   0, 255, 255 ) );
    colors.push_back( cv::Vec3b( 255,   0, 255 ) );
    colors.push_back( cv::Vec3b( 128,   0,   0 ) );
    colors.push_back( cv::Vec3b(   0, 128,   0 ) );
    colors.push_back( cv::Vec3b(   0,   0, 128 ) );
    colors.push_back( cv::Vec3b( 128, 128,   0 ) );
    colors.push_back( cv::Vec3b(   0, 128, 128 ) );
    colors.push_back( cv::Vec3b( 128,   0, 128 ) );
}

// Finalize
void kinect::finalize()
{
    // Destroy Tracker
    tracker.destroy();

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

    // Update Color
    update_color();

    // Update Depth
    update_depth();

    // Update Body Tracking
    update_body_tracking();

    // Update Body Index Map
    update_body_index_map();

    // Update Transformation
    update_transformation();

    // Release Capture Handle
    capture.reset();

    // Release Body Frame Handle
    frame.reset();
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

// Update Body Tracking
inline void kinect::update_body_tracking()
{
    // Enqueue Capture
    tracker.enqueue_capture( capture );

    // Pop Body Tracking Result
    frame = tracker.pop_result();
}

// Update Body Index Map
void kinect::update_body_index_map()
{
    // Get Body Index Map
    body_index_map_image = frame.get_body_index_map();
}

// Update Transformation
inline void kinect::update_transformation()
{
    if( !depth_image.handle() || !body_index_map_image.handle() ){
        return;
    }

    // Transform Color Image to Depth Camera
    std::tie( std::ignore, transformed_body_index_map_image ) = transformation.depth_image_to_color_camera_custom( depth_image, body_index_map_image, k4a_transformation_interpolation_type_t::K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST, K4ABT_BODY_INDEX_MAP_BACKGROUND );
}

// Draw
void kinect::draw()
{
    // Draw Color
    draw_color();

    // Draw Body Index Map
    draw_body_index_map();

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

// Draw Body Index Map
inline void kinect::draw_body_index_map()
{
    if( !body_index_map_image.handle() ){
        return;
    }

    // Get cv::Mat from k4a::image
    body_index_map = k4a::get_mat( body_index_map_image );

    // Release Body Index Map Image Handle
    body_index_map_image.reset();
}

// Draw Transformation
inline void kinect::draw_transformation()
{
    if( !transformed_body_index_map_image.handle() ){
        return;
    }

    // Get cv::Mat from k4a::image
    transformed_body_index_map = k4a::get_mat( transformed_body_index_map_image );

    // Release Transformed Image Handle
    transformed_body_index_map_image.reset();
}

// Show
void kinect::show()
{
    // Show Body Index Map
    show_body_index_map();

    // Show Transformation
    show_transformation();
}

// Show Body Index Map
inline void kinect::show_body_index_map()
{
    if( body_index_map.empty() ){
        return;
    }

    // Visualize Body Index Map
    cv::Mat colorized_body_index_map = cv::Mat::zeros( body_index_map.size(), CV_8UC3 );
    colorized_body_index_map.forEach<cv::Vec3b>(
        [&]( cv::Vec3b& pixel, const int32_t* position ){
            const uint32_t body_index = body_index_map.at<uint8_t>( position[0], position[1] );
            if( body_index != K4ABT_BODY_INDEX_MAP_BACKGROUND ){
                pixel = colors[body_index % colors.size()];
            }
        }
    );

    // Show Image
    const cv::String window_name = cv::format( "body index map (kinect %d)", device_index );
    cv::imshow( window_name, colorized_body_index_map );
}

// Show Transformation
inline void kinect::show_transformation()
{
    if( transformed_body_index_map.empty() ){
        return;
    }

    // Visualize Transformed Body Index Map
    cv::Mat colorized_body_index_map = cv::Mat::zeros( transformed_body_index_map.size(), CV_8UC3 );
    colorized_body_index_map.forEach<cv::Vec3b>(
        [&]( cv::Vec3b& pixel, const int32_t* position ){
            const uint32_t body_index = transformed_body_index_map.at<uint8_t>( position[0], position[1] );
            if( body_index != K4ABT_BODY_INDEX_MAP_BACKGROUND ){
                pixel = colors[body_index % colors.size()];
            }
        }
    );

    // Visualize Transformed Body Index Map on Color
    if( !color.empty() ){
        // Convert Channels
        if( color.channels() == 4 ){
            cv::cvtColor( color, color, cv::COLOR_BGRA2BGR );
        }

        // Alpha Blend
        constexpr double alpha = 0.7;
        constexpr double beta  = 1.0 - alpha;
        cv::addWeighted( color, alpha, colorized_body_index_map, beta, 0.0, colorized_body_index_map );
    }

    // Show Image
    const cv::String window_name = cv::format( "transofrmed body index map (kinect %d)", device_index );
    cv::imshow( window_name, colorized_body_index_map );
}
