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
}

// Initialize Body Tracking
inline void kinect::initialize_body_tracking()
{
    // Set Tracker Configuration
    k4abt_tracker_configuration_t tracker_configuration = K4ABT_TRACKER_CONFIG_DEFAULT;
    tracker_configuration.sensor_orientation = K4ABT_SENSOR_ORIENTATION_DEFAULT;
    tracker_configuration.processing_mode    = k4abt_tracker_processing_mode_t::K4ABT_TRACKER_PROCESSING_MODE_GPU;

    // Create Tracker with Configuration
    tracker = k4abt::tracker::create( calibration, tracker_configuration );
    if( !tracker ){
        throw k4a::error( "Failed to create tracker!" );
    }

    // Set Temporal Smoothing Filter [0.0-1.0]
    constexpr float smoothing_factor = K4ABT_DEFAULT_TRACKER_SMOOTHING_FACTOR;
    tracker.set_temporal_smoothing( smoothing_factor );

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

    // Update Body Tracking
    update_body_tracking();

    // Update Inference
    update_inference();

    // Update Skeleton
    update_skeleton();

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

// Update Body Tracking
inline void kinect::update_body_tracking()
{
    // Enqueue Capture
    tracker.enqueue_capture( capture );

    // Pop Body Tracking Result
    frame = tracker.pop_result();
}

// Update Inference
void kinect::update_inference()
{
    // Get Image that used for Inference
    k4a::capture capture = frame.get_capture();
    color_image = capture.get_color_image();

    // Release Capture Handle
    capture.reset();
}

// Update Skeleton
inline void kinect::update_skeleton()
{
    // Clear Bodies
    bodies.clear();

    // Get Bodies
    const int32_t num_bodies = static_cast<int32_t>( frame.get_num_bodies() );
    bodies.resize( num_bodies );
    for( int32_t i = 0; i < num_bodies; i++ ){
        bodies[i] = frame.get_body( i );
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
    if( !color_image.handle() ){
        return;
    }

    // Get cv::Mat from k4a::image
    color = k4a::get_mat( color_image );

    // Release Skeleton Image Handle
    color_image.reset();
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
            k4a_float2_t position;
            const bool result = calibration.convert_3d_to_2d( joint.position, k4a_calibration_type_t::K4A_CALIBRATION_TYPE_DEPTH, k4a_calibration_type_t::K4A_CALIBRATION_TYPE_COLOR, &position );
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
