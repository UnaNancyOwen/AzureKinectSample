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

namespace color
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // Kinect
        K4A.Device device = null;

        // Visualize
        WriteableBitmap color_bitmap;
        Int32Rect color_rect;
        int color_stride;

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
            int color_width  = device.GetCalibration().ColorCameraCalibration.ResolutionWidth;
            int color_height = device.GetCalibration().ColorCameraCalibration.ResolutionHeight;

            const int color_channles = 4;
            color_stride = color_width * sizeof(byte) * color_channles;
            color_rect   = new Int32Rect(0, 0, color_width, color_height);
            color_bitmap = new WriteableBitmap(color_width, color_height, 96.0, 96.0, PixelFormats.Bgra32, null);

            // Bined Image Control
            Color_Image.Source = color_bitmap;
            Color_Image.Width  = color_width;
            Color_Image.Height = color_height;
        }

        private async void Window_Loaded(object sender, RoutedEventArgs e)
        {
            while (true)
            {
                // Get Capture Frame
                using (K4A.Capture capture = await Task.Run(() => { return this.device.GetCapture(); }))
                // Get Capture Image
                using (K4A.Image color_image = capture.Color)
                {
                    // Get Color Buffer and Write Bitmap
                    byte[] color_buffer = color_image.Memory.ToArray();
                    color_bitmap.WritePixels(color_rect, color_buffer, color_stride, 0, 0);
                }
            }
        }

        private void Window_Closing(object sender, CancelEventArgs e)
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
