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
using System.Numerics;
using K4A = Microsoft.Azure.Kinect.Sensor;
using K4ABT = Microsoft.Azure.Kinect.BodyTracking;

namespace skeleton
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // Kinect
        K4A.Device device = null;
        K4A.Calibration calibration;

        // Body Tracking
        K4ABT.Tracker tracker = null;

        // Visualize
        WriteableBitmap color_bitmap;
        Int32Rect color_rect;
        int color_stride;
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

            // Set Cnvas Size
            Canvas_Body.Width  = color_width;
            Canvas_Body.Height = color_height;
        }

        private async void Window_Loaded(object sender, RoutedEventArgs e)
        {
            while (true)
            {
                // Get Capture Frame
                using (K4A.Capture capture = await Task.Run(() => { return this.device.GetCapture(); }))
                {
                    // Enque Capture
                    tracker.EnqueueCapture(capture);

                    // Pop Result
                    using(K4ABT.Frame frame = tracker.PopResult())
                    // Get Color Image
                    using(K4A.Image color_image = frame.Capture.Color)
                    {
                        // Get Color Buffer and Write Bitmap
                        byte[] color_buffer = color_image.Memory.ToArray();
                        color_bitmap.WritePixels(color_rect, color_buffer, color_stride, 0, 0);

                        // Clear All Ellipse from Canvas
                        Canvas_Body.Children.Clear();

                        // Draw Skeleton
                        for (uint body_index = 0; body_index < frame.NumberOfBodies; body_index++)
                        {
                            // Get Skeleton and ID
                            K4ABT.Skeleton skeleton = frame.GetBodySkeleton(body_index);
                            uint id = frame.GetBodyId(body_index);

                            // Draw Joints
                            for (int joint_index = 0; joint_index < (int)K4ABT.JointId.Count; joint_index++)
                            {
                                // Get Joint and Position
                                K4ABT.Joint joint = skeleton.GetJoint(joint_index);
                                Vector2? position = calibration.TransformTo2D(joint.Position, K4A.CalibrationDeviceType.Depth, K4A.CalibrationDeviceType.Color);

                                if (!position.HasValue)
                                {
                                    continue;
                                }

                                // Create Ellipse
                                const int radius = 10;
                                SolidColorBrush stroke_color = new SolidColorBrush(colors[id % colors.Length]);
                                SolidColorBrush fill_color = new SolidColorBrush((joint.ConfidenceLevel >= K4ABT.JointConfidenceLevel.Medium) ? colors[id % colors.Length] : Colors.Transparent);
                                Ellipse ellipse = new Ellipse(){Width = radius, Height = radius, StrokeThickness = 1, Stroke = stroke_color, Fill = fill_color};

                                // Add Ellipse to Canvas
                                Canvas.SetLeft(ellipse, position.Value.X - (radius / 2));
                                Canvas.SetTop(ellipse, position.Value.Y - (radius / 2));
                                Canvas_Body.Children.Add(ellipse);
                            }
                        }
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
