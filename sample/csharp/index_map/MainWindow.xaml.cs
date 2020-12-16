using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using K4A = Microsoft.Azure.Kinect.Sensor;
using K4ABT = Microsoft.Azure.Kinect.BodyTracking;

namespace index_map
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // Kinect
        K4A.Device device = null;
        K4A.Calibration calibration;
        K4A.Transformation transformation = null;

        // Body Tracking
        K4ABT.Tracker tracker = null;

        // Visualize
        WriteableBitmap color_bitmap;
        WriteableBitmap body_index_map_bitmap;
        Int32Rect color_rect;
        Int32Rect body_index_map_rect;
        int color_stride;
        int body_index_map_stride;
        byte[] colorized_body_index_map_buffer;
        Color[] colors;

        public MainWindow()
        {
            InitializeComponent();

            // Open Device
            device = K4A.Device.Open();

            // Start Cameras
            device.StartCameras(new K4A.DeviceConfiguration
            {
                ColorFormat = K4A.ImageFormat.ColorBGRA32,
                ColorResolution = K4A.ColorResolution.R720p,
                DepthMode = K4A.DepthMode.NFOV_2x2Binned,
                SynchronizedImagesOnly = true
            });

            // Get Calibration
            calibration = device.GetCalibration(K4A.DepthMode.NFOV_2x2Binned, K4A.ColorResolution.R720p);

            // Create Transformation
            transformation = calibration.CreateTransformation();

            // Create Tracker
            tracker = K4ABT.Tracker.Create(
                calibration,
                new K4ABT.TrackerConfiguration
                {
                    SensorOrientation = K4ABT.SensorOrientation.Default,
                    ProcessingMode = K4ABT.TrackerProcessingMode.Gpu
                }
            );

            // Create Buffer
            int color_width  = calibration.ColorCameraCalibration.ResolutionWidth;
            int color_height = calibration.ColorCameraCalibration.ResolutionHeight;

            const int color_channles = 4;
            color_stride = color_width * sizeof(byte) * color_channles;
            color_rect   = new Int32Rect(0, 0, color_width, color_height);
            color_bitmap = new WriteableBitmap(color_width, color_height, 96.0, 96.0, PixelFormats.Bgra32, null);

            const int body_index_map_channles = 4;
            body_index_map_stride           = color_width * sizeof(byte) * body_index_map_channles;
            body_index_map_rect             = new Int32Rect(0, 0, color_width, color_height);
            body_index_map_bitmap           = new WriteableBitmap(color_width, color_height, 96.0, 96.0, PixelFormats.Bgra32, null);
            colorized_body_index_map_buffer = new byte[color_width * color_height * color_channles];

            // Generate Color LUT
            colors = new Color[6];
            colors[0] = new Color(){ B = 255, G =   0, R =   0, A = 128 };
            colors[1] = new Color(){ B =   0, G = 255, R =   0, A = 128 };
            colors[2] = new Color(){ B =   0, G =   0, R = 255, A = 128 };
            colors[3] = new Color(){ B = 255, G = 255, R =   0, A = 128 };
            colors[4] = new Color(){ B =   0, G = 255, R = 255, A = 128 };
            colors[5] = new Color(){ B = 255, G =   0, R = 255, A = 128 };

            // Bined Image Control
            Color_Image.Source = color_bitmap;
            Color_Image.Width  = color_width;
            Color_Image.Height = color_height;

            Index_Map.Source = body_index_map_bitmap;
            Index_Map.Width  = color_width;
            Index_Map.Height = color_height;
        }

        private async void Window_Loaded(object sender, RoutedEventArgs e)
        {
            while (true)
            {
                // Get Capture Frame
                using (K4A.Capture capture = await Task.Run(() => { return this.device.GetCapture(); }))
                {
                    // Enqueue Tracker
                    tracker.EnqueueCapture(capture);

                    // Pop Result
                    using (K4ABT.Frame frame = tracker.PopResult())
                    // Get Color Image
                    using(K4A.Image color_image = frame.Capture.Color)
                    {
                        // Get Color Buffer and Write Bitmap
                        byte[] color_buffer = color_image.Memory.ToArray();
                        color_bitmap.WritePixels(color_rect, color_buffer, color_stride, 0, 0);

                        // Transform Body Index Map to Color Camera
                        (K4A.Image _, K4A.Image body_index_map) reult = 
                            transformation.DepthImageToColorCameraCustom(
                                frame.Capture.Depth,
                                frame.BodyIndexMap,
                                K4A.TransformationInterpolationType.Nearest,
                                K4ABT.Frame.BodyIndexMapBackground
                            );

                        //Get Body Index Map Buffer
                        byte[] body_index_map_buffer = reult.body_index_map.Memory.ToArray();

                        // Draw Body Index Map and Write Bitmap
                        Parallel.For(0, body_index_map_buffer.Length, i =>
                        {
                            uint index = (uint)body_index_map_buffer[i];
                            if (index != K4ABT.Frame.BodyIndexMapBackground)
                            {
                                Color color = colors[index % colors.Length];
                                colorized_body_index_map_buffer[i * 4 + 0] = color.B;
                                colorized_body_index_map_buffer[i * 4 + 1] = color.G;
                                colorized_body_index_map_buffer[i * 4 + 2] = color.R;
                                colorized_body_index_map_buffer[i * 4 + 3] = color.A;
                            }
                            else{
                                colorized_body_index_map_buffer[i * 4 + 0] = 0;
                                colorized_body_index_map_buffer[i * 4 + 1] = 0;
                                colorized_body_index_map_buffer[i * 4 + 2] = 0;
                                colorized_body_index_map_buffer[i * 4 + 3] = 255;
                            }
                        });
                        body_index_map_bitmap.WritePixels(body_index_map_rect, colorized_body_index_map_buffer, body_index_map_stride, 0, 0);

                        // Clear Body Index Map
                        reult.body_index_map.Dispose();
                    }
                }
            }
        }

        private void Window_Closing(object sender, CancelEventArgs e)
        {
            // Shutdown Tracker
            if (tracker != null)
            {
                tracker.Shutdown();
                tracker.Dispose();
                tracker = null;
            }

            // Destroy Transformation
            if (transformation != null)
            {
                transformation.Dispose();
                transformation = null;
            }

            // Stop and Close Camera
            if (device != null)
            {
                device.StopCameras();
                device.Dispose();
                device = null;
            }
        }
    }
}
