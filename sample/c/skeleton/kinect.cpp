#include "kinect.hpp"
#include "util.h"

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
      tracker( nullptr ),
      frame( nullptr )
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
}

// Initialize Body Tracking
inline void kinect::initialize_body_tracking()
{
    // Set Tracker Configuration
    k4abt_tracker_configuration_t tracker_configuration = K4ABT_TRACKER_CONFIG_DEFAULT;
    tracker_configuration.sensor_orientation = K4ABT_SENSOR_ORIENTATION_DEFAULT;
    tracker_configuration.processing_mode    = k4abt_tracker_processing_mode_t::K4ABT_TRACKER_PROCESSING_MODE_GPU;

    // Create Tracker with Configuration
    K4A_RESULT_CHECK( k4abt_tracker_create( &calibration, tracker_configuration, &tracker ) );

    // Set Temporal Smoothing Filter [0.0-1.0]
    constexpr float smoothing_factor = K4ABT_DEFAULT_TRACKER_SMOOTHING_FACTOR;
    k4abt_tracker_set_temporal_smoothing( tracker, smoothing_factor );

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

    // Update Body Tracking
    update_body_tracking();

    // Update Inference
    update_inference();

    // Update Skeleton
    update_skeleton();

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

// Update Inference
void kinect::update_inference()
{
    // Get Image that used for Inference
    k4a_capture_t capture = k4abt_frame_get_capture( frame );
    color_image = k4a_capture_get_color_image( capture );

    // Release Capture Handle
    k4a_capture_release( capture );
}

// Update Skeleton
inline void kinect::update_skeleton()
{
    // Clear Bodies
    bodies.clear();

    // Get Bodies
    const int32_t num_bodies = static_cast<int32_t>( k4abt_frame_get_num_bodies( frame ) );
    bodies.resize( num_bodies );
    for( int32_t i = 0; i < num_bodies; i++ ){
        k4abt_body_t body;
        body.id = k4abt_frame_get_body_id( frame, i );
        K4A_RESULT_CHECK( k4abt_frame_get_body_skeleton( frame, i, &body.skeleton ) );
        bodies[i] = body;
    }
}

// Draw
void kinect::draw()
{
    // Draw Color
    draw_color();
}

// Draw Color
inline void kinect::draw_color()
{
    if( !color_image ){
        return;
    }

    // Get cv::Mat from k4a_image_t
    color = k4a_get_mat( color_image );

    // Release Skeleton Image Handle
    k4a_image_release( color_image );
}

// Show
void kinect::show()
{
    // Show Skeleton
    show_skeleton();
}

// Show Skeleton
inline void kinect::show_skeleton()
{
    // Visualize Skeleton
    for( const k4abt_body_t& body : bodies ){
        for( const k4abt_joint_t& joint : body.skeleton.joints ){
            int32_t result = 0;
            k4a_float2_t position;
            K4A_RESULT_CHECK( k4a_calibration_3d_to_2d( &calibration, &joint.position, k4a_calibration_type_t::K4A_CALIBRATION_TYPE_DEPTH, k4a_calibration_type_t::K4A_CALIBRATION_TYPE_COLOR, &position, &result ) );
            if( !result ){
                continue;
            }

            const int32_t thickness = ( joint.confidence_level >= k4abt_joint_confidence_level_t::K4ABT_JOINT_CONFIDENCE_MEDIUM ) ? -1 : 1;
            const cv::Point point( static_cast<int32_t>( position.xy.x ), static_cast<int32_t>( position.xy.y ) );
            cv::circle( color, point, 5, colors[( body.id - 1 ) % colors.size()], thickness );
        }
    }

    // Show Image
    const cv::String window_name = cv::format( "skeleton (kinect %d)", device_index );
    cv::imshow( window_name, color );
}
