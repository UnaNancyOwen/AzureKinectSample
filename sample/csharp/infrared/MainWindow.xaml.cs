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

namespace infrared
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // Kinect
        K4A.Device device = null;

        // Visualize
        WriteableBitmap infrared_bitmap;
        Int32Rect infrared_rect;
        int infrared_stride;

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

            int infrared_width  = device.GetCalibration().DepthCameraCalibration.ResolutionWidth;
            int infrared_height = device.GetCalibration().DepthCameraCalibration.ResolutionHeight;

            const int infrared_channles = 1;
            infrared_stride = infrared_width * sizeof(byte) * infrared_channles;
            infrared_rect   = new Int32Rect(0, 0, infrared_width, infrared_height);
            infrared_bitmap = new WriteableBitmap(infrared_width, infrared_height, 96.0, 96.0, PixelFormats.Gray8, null);

            Infrared_Image.Source = infrared_bitmap;
            Infrared_Image.Width  = infrared_width;
            Infrared_Image.Height = infrared_height;
        }

        private async void Window_Loaded(object sender, RoutedEventArgs e)
        {
            while (true)
            {
                // Get Capture Frame
                using (K4A.Capture capture = await Task.Run(() => { return this.device.GetCapture(); }))
                // Get Capture Image
                using (K4A.Image infrared_image = capture.IR)
                {
                    // Get Infrared Buffer, and Write Bitmap
                    ushort[] infrared_buffer = infrared_image.GetPixels<ushort>().ToArray();
                    infrared_bitmap.WritePixels(infrared_rect, Array.ConvertAll(infrared_buffer, i => (byte)(i * 0.5)), infrared_stride, 0, 0);
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
