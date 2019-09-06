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

    // Initialize Viewer
    initialize_viewer();
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

// Initialize Viewer
inline void kinect::initialize_viewer()
{
    #ifdef HAVE_OPENCV_VIZ
    // Create Viewer
    const cv::String window_name = cv::format( "point cloud (kinect %d)", device_index );
    viewer = cv::viz::Viz3d( window_name );

    // Show Coordinate System Origin
    constexpr double scale = 100.0;
    viewer.showWidget( "origin", cv::viz::WCameraPosition( scale ) );
    #endif
}

// Finalize
void kinect::finalize()
{
    // Destroy Transformation
    transformation.destroy();

    // Stop Cameras
    device.stop_cameras();

    // Close Device
    device.close();

    // Close Window
    cv::destroyAllWindows();

    #ifdef HAVE_OPENCV_VIZ
    // Close Viewer
    viewer.close();
    #endif
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

        #ifdef HAVE_OPENCV_VIZ
        if( viewer.wasStopped() ){
            break;
        }
        #endif
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

    // Update Point Cloud
    update_point_cloud();

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
    if( !depth_image.handle() ){
        return;
    }

    // Transform Depth Image to Color Camera
    transformed_depth_image = transformation.depth_image_to_color_camera( depth_image );
}

// Update Point Cloud
inline void kinect::update_point_cloud()
{
    if( !transformed_depth_image.handle() ){
        return;
    }

    // Transform Depth Image to Point Cloud
    xyz_image = transformation.depth_image_to_point_cloud( transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR );
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

    // Draw Point Cloud
    draw_point_cloud();
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

    // Release Depth Image Handle
    depth_image.reset();
}

// Draw Transformation
inline void kinect::draw_transformation()
{
    if( !transformed_depth_image.handle() ){
        return;
    }

    // Get cv::Mat from k4a::image
    transformed_depth = k4a::get_mat( transformed_depth_image );

    // Release Transformed Image Handle
    transformed_depth_image.reset();
}

// Draw Point Cloud
inline void kinect::draw_point_cloud()
{
    if( !xyz_image.handle() ){
        return;
    }

    // Get cv::Mat from k4a::image
    xyz = k4a::get_mat( xyz_image );

    // Release Point Cloud Image Handle
    xyz_image.reset();
}

// Show
void kinect::show()
{
    // Show Color
    show_color();

    // Show Transformation
    show_transformation();

    // Show Point Cloud
    show_point_cloud();
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

// Show Transformation
inline void kinect::show_transformation()
{
    if( transformed_depth.empty() ){
        return;
    }

    // Scaling Depth
    transformed_depth.convertTo( transformed_depth, CV_8U, -255.0 / 5000.0, 255.0 );

    // Show Image
    const cv::String window_name = cv::format( "transformed depth (kinect %d)", device_index );
    cv::imshow( window_name, transformed_depth );
}

// Show Point Cloud
inline void kinect::show_point_cloud()
{
    if( xyz.empty() || color.empty() ){
        return;
    }

    #ifdef HAVE_OPENCV_VIZ
    // Create Point Cloud Widget
    cv::viz::WCloud cloud = cv::viz::WCloud( xyz, color );

    // Show Widget
    viewer.showWidget( "cloud", cloud );
    viewer.spinOnce();
    #endif
}
