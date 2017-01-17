using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;



namespace RobotSwarmGUI
{
    /// <summary>
    /// Interaction logic for AddRobots.xaml
    /// </summary>
    /// 

    public partial class AddPairedRobots : Window
    {
        int comPort;
        int deviceNr;
        public AddPairedRobots()
        {
            InitializeComponent();
          
        }
        public void PassParameters(ref int iComPort, ref int iDeviceNr)
        {
            iComPort = comPort;
            iDeviceNr = deviceNr;
            
        }
        
        private void AddRobotButton_Click(object sender, RoutedEventArgs e)
        {

            if (ComPortTextBox.Text != "")
            {
                GlobalVar.PassComPort = Convert.ToInt32(ComPortTextBox.Text);
            }

            if (DeviceNrTextBox.Text != "")
            {
                GlobalVar.PassDeviceNr = DeviceNrTextBox.Text;
            }
            Close();
        }

        private void CancelButton_Click(object sender, RoutedEventArgs e)
        {
            Close();
        }
    }
}
