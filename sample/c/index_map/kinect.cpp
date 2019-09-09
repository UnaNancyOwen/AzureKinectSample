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
      tracker( nullptr ),
      frame( nullptr ),
      body_index_map_image( nullptr ),
      transformed_body_index_map_image( nullptr )
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
    K4A_RESULT_CHECK( k4a_device_get_calibration( device, device_configuration.depth_mode, device_configuration.color_resolution, &calibration ) );

    // Create Transformation
    transformation = k4a_transformation_create( &calibration );
}

// Initialize Body Tracking
inline void kinect::initialize_body_tracking()
{
    // Set Tracker Configuration
    k4abt_tracker_configuration_t tracker_configuration = K4ABT_TRACKER_CONFIG_DEFAULT;
    tracker_configuration.sensor_orientation = K4ABT_SENSOR_ORIENTATION_DEFAULT;

    // Create Tracker with Configuration
    K4A_RESULT_CHECK( k4abt_tracker_create( &calibration, tracker_configuration, &tracker ) );

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
    k4abt_tracker_destroy( tracker );

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

    // Update Body Tracking
    update_body_tracking();

    // Update Body Index Map
    update_body_index_map();

    // Update Transformation
    update_transformation();

    // Release Capture Handle
    k4a_capture_release( capture );

    // Release Body Frame Handle
    k4abt_frame_release( frame );
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

// Update Body Tracking
inline void kinect::update_body_tracking()
{
    constexpr std::chrono::milliseconds time_out( K4A_WAIT_INFINITE );
    k4a_wait_result_t result;

    // Enqueue Capture
    result = k4abt_tracker_enqueue_capture( tracker, capture, static_cast<int32_t>( time_out.count() ) );
    if( result == k4a_wait_result_t::K4A_WAIT_RESULT_FAILED ){
        throw std::runtime_error( "Failed to enqueue capture to tracker!" );
    }
    else if( result == k4a_wait_result_t::K4A_WAIT_RESULT_TIMEOUT ){
        this->~kinect();
    }

    // Pop Body Tracking Result
    result = k4abt_tracker_pop_result( tracker, &frame, static_cast<int32_t>( time_out.count() ) );
    if( result == k4a_wait_result_t::K4A_WAIT_RESULT_FAILED ){
        throw std::runtime_error( "Failed to pop result from tracker!" );
    }
    else if( result == k4a_wait_result_t::K4A_WAIT_RESULT_TIMEOUT ){
        this->~kinect();
    }
}

// Update Body Index Map
void kinect::update_body_index_map()
{
    // Get Body Index Map
    body_index_map_image = k4abt_frame_get_body_index_map( frame );
}

// Update Transformation
inline void kinect::update_transformation()
{
    if( !depth_image || !body_index_map_image ){
        return;
    }

    // Transform Body Index Map Image to Color Camera
    const int32_t color_width  = k4a_image_get_width_pixels( color_image );
    const int32_t color_height = k4a_image_get_height_pixels( color_image );
    int32_t stride_bytes;

    stride_bytes = color_width * static_cast<int32_t>( sizeof( uint16_t ) );
    K4A_RESULT_CHECK( k4a_image_create( k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16, color_width, color_height, stride_bytes, &transformed_depth_image ) );
    stride_bytes = color_width * static_cast<int32_t>( sizeof( uint8_t ) );
    K4A_RESULT_CHECK( k4a_image_create( k4a_image_format_t::K4A_IMAGE_FORMAT_CUSTOM8, color_width, color_height, stride_bytes, &transformed_body_index_map_image ) );

    K4A_RESULT_CHECK( k4a_transformation_depth_image_to_color_camera_custom( transformation, depth_image, body_index_map_image, transformed_depth_image, transformed_body_index_map_image, k4a_transformation_interpolation_type_t::K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST, K4ABT_BODY_INDEX_MAP_BACKGROUND ) );
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
    if( !color_image ){
        return;
    }

    // Get cv::Mat from k4a_image_t
    color = k4a_get_mat( color_image );

    // Release Color Image Handle
    k4a_image_release( color_image );
}

// Draw Body Index Map
inline void kinect::draw_body_index_map()
{
    if( !body_index_map_image ){
        return;
    }

    // Get cv::Mat from k4a_image_t
    body_index_map = k4a_get_mat( body_index_map_image );

    // Release Body Index Map Image Handle
    k4a_image_release( body_index_map_image );
}

// Draw Transformation
inline void kinect::draw_transformation()
{
    if( !transformed_body_index_map_image ){
        return;
    }

    // Get cv::Mat from k4a::image
    transformed_body_index_map = k4a_get_mat( transformed_body_index_map_image );

    // Release Transformed Image Handle
    k4a_image_release( transformed_depth_image );
    k4a_image_release( transformed_body_index_map_image );
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
