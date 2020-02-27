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

namespace depth
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // Kinect
        K4A.Device device = null;

        // Visualize
        WriteableBitmap depth_bitmap;
        Int32Rect depth_rect;
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

            // Create Buffer
            int depth_width  = device.GetCalibration().DepthCameraCalibration.ResolutionWidth;
            int depth_height = device.GetCalibration().DepthCameraCalibration.ResolutionHeight;

            const int depth_channles = 1;
            depth_stride = depth_width * sizeof(byte) * depth_channles;
            depth_rect   = new Int32Rect(0, 0, depth_width, depth_height);
            depth_bitmap = new WriteableBitmap(depth_width, depth_height, 96.0, 96.0, PixelFormats.Gray8, null);

            // Bined Image Control
            Depth_Image.Source = depth_bitmap;
            Depth_Image.Width  = depth_width;
            Depth_Image.Height = depth_height;
        }

        private async void Window_Loaded(object sender, RoutedEventArgs e)
        {
            while (true)
            {
                // Get Capture Frame
                using (K4A.Capture capture = await Task.Run(() => { return this.device.GetCapture(); }))
                // Get Capture Image
                using (K4A.Image depth_image = capture.Depth)
                {
                    // Get Depth Buffer, and Write Bitmap
                    ushort[] depth_buffer = depth_image.GetPixels<ushort>().ToArray();
                    depth_bitmap.WritePixels(depth_rect, Array.ConvertAll(depth_buffer, i => (byte)(i * (255.0 / 5000.0))), depth_stride, 0, 0);
                }
            }
        }

        private void Window_Unloaded(object sender, RoutedEventArgs e)
        {
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
