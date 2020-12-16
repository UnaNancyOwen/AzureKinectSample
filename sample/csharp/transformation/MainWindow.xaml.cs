#define TO_COLOR

using System;
using System.Collections.Generic;
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

namespace transformation
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // Kinect
        K4A.Device device = null;
        K4A.Transformation transformation = null;

        // Visualize
        WriteableBitmap color_bitmap;
        WriteableBitmap depth_bitmap;
        Int32Rect color_rect;
        Int32Rect depth_rect;
        int color_stride;
        int depth_stride;

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
                DepthMode = K4A.DepthMode.NFOV_Unbinned,
                SynchronizedImagesOnly = true
            });

            // Create Transformation
            transformation = device.GetCalibration().CreateTransformation();

            // Create Buffer
            #if TO_COLOR
            int width  = device.GetCalibration().ColorCameraCalibration.ResolutionWidth;
            int height = device.GetCalibration().ColorCameraCalibration.ResolutionHeight;
            #else
            int width  = device.GetCalibration().DepthCameraCalibration.ResolutionWidth;
            int height = device.GetCalibration().DepthCameraCalibration.ResolutionHeight;
            #endif

            const int color_channles = 4;
            color_stride = width * sizeof(byte) * color_channles;
            color_rect   = new Int32Rect(0, 0, width, height);
            color_bitmap = new WriteableBitmap(width, height, 96.0, 96.0, PixelFormats.Bgra32, null);

            const int depth_channles = 1;
            depth_stride = width * sizeof(byte) * depth_channles;
            depth_rect   = new Int32Rect(0, 0, width, height);
            depth_bitmap = new WriteableBitmap(width, height, 96.0, 96.0, PixelFormats.Gray8, null);

            // Bined Image Control
            Color_Image.Source = color_bitmap;
            Color_Image.Width  = width;
            Color_Image.Height = height;

            Depth_Image.Source = depth_bitmap;
            Depth_Image.Width  = width;
            Depth_Image.Height = height;
        }

        private async void Window_Loaded(object sender, RoutedEventArgs e)
        {
            while (true)
            {
                // Get Capture Frame
                using (K4A.Capture capture = await Task.Run(() => { return this.device.GetCapture(); }))
                // Get Capture Image and Transformed Image
                #if TO_COLOR
                using (K4A.Image color_image = capture.Color)
                using (K4A.Image depth_image = transformation.DepthImageToColorCamera(capture))
                #else
                using (K4A.Image color_image = transformation.ColorImageToDepthCamera(capture))
                using (K4A.Image depth_image = capture.Depth)
                #endif
                {
                    // Get Color Buffer and Write Bitmap
                    byte[] color_buffer = color_image.Memory.ToArray();
                    color_bitmap.WritePixels(color_rect, color_buffer, color_stride, 0, 0);

                    // Get Depth Buffer, and Write Bitmap
                    ushort[] depth_buffer = depth_image.GetPixels<ushort>().ToArray();
                    depth_bitmap.WritePixels(depth_rect, Array.ConvertAll(depth_buffer, i => (byte)(i * (255.0 / 5000.0))), depth_stride, 0, 0);
                }
            }
        }

        private void Window_Closing(object sender, CancelEventArgs e)
        {
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
