#region Usings
using System;
using System.IO;
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
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using System.Collections.ObjectModel;
using InTheHand.Net;
using InTheHand.Net.Sockets;
using InTheHand.Net.Bluetooth;
using System.Management;
using System.ComponentModel;
using System.Threading;
using System.Runtime.InteropServices;
using System.Diagnostics;
using System.Windows.Controls.Primitives;

#endregion //Usings

namespace RobotSwarmGUI
{
    
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 


    
    public partial class MainWindow : Window, INotifyPropertyChanged 
    {
        
        //public AddPairedRobots addRobotWindow;

        #region Fields
        private string infoText;
        #endregion //Fields

        #region Properties
        public ObservableCollection<ObservableDeviceClass> PairedDeviceCollection { get; set; } //Paired devices as ObservableCollection -->GUI
        public ObservableCollection<ComPortMapClass> MappingList { get; set; } // Name, RoboType, COM Port
        public ObservableCollection<AlgorithmStringClass> AlgorithmCollection { get; set; }
        public ObservableCollection<string> RemovedRoboBlobsCollection { get; set; } // priority list of robots removed from image
        public List<string> AlgorithmString { get; set; }
        public List<string> PairedDeviceList { get; set; } //Paired devices as List -->Lookup
        public List<string> CheckedDeviceList { get; set; }
        public List<string> ConnectedDeviceList { get; set; }
        public List<string> MovingDeviceList { get; set; }
        public CDict Dict { get; set; }
        public string CurrentAlgorithm { get; set; }
        public bool HelpCursor { get; set; }
        public bool HelpEnabled { get; set; }
        public string InfoText
        {
            get { return infoText; }
            set
            {
                if (value != infoText)
                {
                    infoText = value;
                    OnPropertyChanged("InfoText");
                }
            }
        }
        public bool Closing { get; set; }
        public bool FirstExecute { get; set; }
        DispatcherOperation dispatcherOperation { get; set; }
        #endregion //Properties

        #region Constructor
        public MainWindow()
        {
            InitializeComponent();
            Dict = new CDict();
            PairedDeviceList = new List<string>();
            CheckedDeviceList = new List<string>();
            ConnectedDeviceList = new List<string>();
            AlgorithmString = new List<string>();
            MovingDeviceList = new List<string>();
            CurrentAlgorithm = "";
            CreateAlgorithmCollection();
            CreatePairedDeviceCollection();
            CreateMappingList();
            CreateRemovedRoboBlobsCollection();
            InfoText = "";
            HelpEnabled = true;
            Closing = false;
            FirstExecute = true;

            //addRobotWindow = new AddPairedRobots();

        }
        #endregion //Constructor

        #region Events
        private void Proper_Search_Click(object sender, RoutedEventArgs e)
        {
            //Get list of paired devices
            InfoText = "Searching for devices...";
            Dispatcher.Invoke(new Action(() => { }), DispatcherPriority.ContextIdle, null); //wait for the rendering to finish
            GetPairedDevices(Dict);
            Console.WriteLine("GetPairedDevices exited");
            InfoText = "";
        }
        private void Quick_Search_Click(object sender, RoutedEventArgs e)
        {
            InfoText = "Checking for known devices...";
            Dispatcher.Invoke(new Action(() => { }), DispatcherPriority.ContextIdle, null); //wait for the rendering to finish
            GetRememberedDevices();
            Console.WriteLine("GetRememberedDevices exited");
            InfoText = "";
        }
        private void Connect_Click(object sender, RoutedEventArgs e)
        {
            //Stop moving devices
            for (int i = 0; i < MovingDeviceList.Count; i++)
            {
                for (int j = 0; j < MappingList.Count; j++)
                {
                    if (MovingDeviceList[i] == MappingList[j].DeviceName)
                    {
                        Stop(MappingList[j].ComPort);
                        break;
                    }
                }
            }

            bool bConnected = false;

            /*for (int i = 0; i < MappingList.Count; i++)
            {
                Stop(MappingList[i].ComPort);
            }*/

            for (int i = 0; i < MappingList.Count; i++)
            {
                if (CheckedDeviceList.Contains(MappingList[i].DeviceName) && !ConnectedDeviceList.Contains(MappingList[i].DeviceName))
                {
                   
                    bConnected = Connect(MappingList[i].DeviceType, MappingList[i].ComPort, i);
                    if (bConnected)
                    {
                        ConnectedDeviceList.Add(MappingList[i].DeviceName);
                        for (int j = 1; j < PairedDeviceCollection.Count; j++)
                        {
                            if (PairedDeviceCollection[j].DeviceName == MappingList[i].DeviceName)
                            {
                                PairedDeviceCollection[j].IsConnected = true;
                                PairedDeviceCollection[j].ConnectionState = Brushes.Green;
                                PairedDeviceCollection[j].Algorithm = "idle";
                                PairedDeviceCollection[j].AlgChanged = false;
                                Dispatcher.Invoke(new Action(() => { }), DispatcherPriority.ContextIdle, null); //wait for the rendering to finish
                                break;
                            }
                        }
                        
                    }
                    bConnected = false;
                }
            }

            Execute();
        }
        private void Disconnect_Click(object sender, RoutedEventArgs e)
        {
            bool bDisconnected = false;
            bool bStop = false;

            for (int i = 0; i < MappingList.Count; i++)
            {
                if (CheckedDeviceList.Contains(MappingList[i].DeviceName) && ConnectedDeviceList.Contains(MappingList[i].DeviceName))
                {
                    bStop = Stop(MappingList[i].ComPort);
                    if (bStop)
                    {
                        bDisconnected = Disconnect(MappingList[i].DeviceType, MappingList[i].ComPort);
                    }

                    if (bDisconnected)
                    {
                        for (int j = 0; j < ConnectedDeviceList.Count; j++)
                        {
                            if (ConnectedDeviceList[j] == MappingList[i].DeviceName)
                            {
                                ConnectedDeviceList.RemoveAt(j);
                                for (int k = 1; k < PairedDeviceCollection.Count; k++)
                                {
                                    if (PairedDeviceCollection[k].DeviceName == MappingList[i].DeviceName)
                                    {
                                        PairedDeviceCollection[k].IsConnected = false;
                                        PairedDeviceCollection[k].ConnectionState = Brushes.Gray;
                                        PairedDeviceCollection[k].Algorithm = "";
                                        break;
                                    }
                                }
                                break;
                            }
                        }
                        if (MovingDeviceList.Contains(MappingList[i].DeviceName))
                        {
                            for (int l = 0; l < MovingDeviceList.Count; l++)
                            {
                                if (MovingDeviceList[l] == MappingList[i].DeviceName)
                                {
                                    MovingDeviceList.RemoveAt(l);
                                    break;
                                }
                            }

                        }
                    }
                }
            }

        }
        private void Start_Click(object sender, RoutedEventArgs e)
        {
            

            CurrentAlgorithm = GetSelectedAlgorithm();

            for (int i = 1; i < PairedDeviceCollection.Count; i++)
            {
                if (PairedDeviceCollection[i].IsConnected && PairedDeviceCollection[i].IsChecked)
                {
                    PairedDeviceCollection[i].Algorithm = CurrentAlgorithm;
                }
            }

            dispatcherOperation = startButton.Dispatcher.BeginInvoke(
                        DispatcherPriority.Normal,
                        new NextDelegate(Execute));
        }
        private void Stop_Click(object sender, RoutedEventArgs e)
        {
            bool bStop = false;

            for (int i = 0; i < MappingList.Count; i++)
            {
                if (CheckedDeviceList.Contains(MappingList[i].DeviceName) && ConnectedDeviceList.Contains(MappingList[i].DeviceName))
                {
                    bStop = Stop(MappingList[i].ComPort);
                    if (bStop)
                    {
                        PairedDeviceCollection[i + 1].Algorithm = "stop";
                        if (MovingDeviceList.Contains(MappingList[i].DeviceName))
                        {
                            for (int l = 0; l < MovingDeviceList.Count; l++)
                            {
                                if (MovingDeviceList[l] == MappingList[i].DeviceName)
                                {
                                    MovingDeviceList.RemoveAt(l);
                                    break;
                                }
                            }
                        }
                    }
                }
            }


        }
        private void Remove_Click(object sender, RoutedEventArgs e)
        {
            //Stop moving devices
            for (int i = 0; i < MovingDeviceList.Count; i++)
            {
                for (int j = 0; j < MappingList.Count; j++)
                {
                    if (MovingDeviceList[i] == MappingList[j].DeviceName)
                    {
                        Stop(MappingList[j].ComPort);
                        break;
                    }
                }
            }

            bool bDisconnect = false;
            bool bStop = false;

            for (int i = 0; i < MappingList.Count; i++)
            {
                string sDeviceName = MappingList[i].DeviceName;

                if (CheckedDeviceList.Contains(sDeviceName))
                {
                    if (ConnectedDeviceList.Contains(sDeviceName))
                    {
                        //bStop = Stop(MappingList[i].ComPort);
                        if (bStop)
                        {
                            Disconnect(MappingList[i].DeviceType, MappingList[i].ComPort);
                        }
                        if (bDisconnect)
                        {
                            for (int j = 0; j < ConnectedDeviceList.Count; j++)
                            {
                                if (ConnectedDeviceList[j] == sDeviceName)
                                {
                                    ConnectedDeviceList.RemoveAt(j);
                                    break;
                                }
                            }
                        }
                    }

                    if (MappingList[i].DeviceType != (int)RoboType.Elisa) //Cannot unpair elisas
                    {
                        BluetoothAddress btAddress = Dict.Name_Mac[sDeviceName];
                        BluetoothSecurity.RemoveDevice(btAddress); //Unpair device
                    }

                    Console.WriteLine("{0} removed", sDeviceName);


                    for (int j = 0; j < MappingList.Count; j++)
                    {
                        if (MappingList[j].DeviceName == sDeviceName)
                        {
                            RemoveFromList(j - 1);
                            MappingList.RemoveAt(j);
                            break;
                        }
                    }
                    for (int j = 1; j < PairedDeviceCollection.Count; j++)
                    {
                        if (PairedDeviceCollection[j].DeviceName == sDeviceName)
                        {
                            PairedDeviceCollection.RemoveAt(j);
                            break;
                        }
                    }
                    for (int j = 0; j < PairedDeviceList.Count; j++)
                    {
                        if (PairedDeviceList[j] == sDeviceName)
                        {
                            PairedDeviceList.RemoveAt(j);
                            break;
                        }
                    }
                    if (CheckedDeviceList.Contains(sDeviceName))
                    {
                        for (int j = 0; j < CheckedDeviceList.Count; j++)
                        {
                            if (CheckedDeviceList[j] == sDeviceName)
                            {
                                CheckedDeviceList.RemoveAt(j);
                                break;
                            }
                        }
                    }
                    if (MovingDeviceList.Contains(sDeviceName))
                    {
                        for (int j = 0; j < MovingDeviceList.Count; j++)
                        {
                            if (MovingDeviceList[j] == sDeviceName)
                            {
                                MovingDeviceList.RemoveAt(j);
                                break;
                            }
                        }
                    }
                }
            }

            Execute();
        }
        private void Calibrate_Click(object sender, RoutedEventArgs e)
        {
            //Stop moving devices
            for (int i = 0; i < MovingDeviceList.Count; i++)
            {
                for(int j = 0; j < MappingList.Count; j++)
                {
                    if(MovingDeviceList[i] == MappingList[j].DeviceName)
                    {
                        Stop(MappingList[j].ComPort);
                        break;
                    }
                }            
            }
            
            //calibrate
            for (int i = 0; i < MappingList.Count; i++)
            {
                if (CheckedDeviceList.Contains(MappingList[i].DeviceName) && ConnectedDeviceList.Contains(MappingList[i].DeviceName))
                {
                    Calibrate(MappingList[i].ComPort);
                }
            }

            //start devices again
            Execute();
            /*for (int i = 0; i < MovingDeviceList.Count; i++)
            {
                for (int j = 0; j < PairedDeviceCollection.Count; j++)
                {
                    if (MovingDeviceList[i] == PairedDeviceCollection[j].DeviceName)
                    {
                        Navigate(PairedDeviceCollection[j].Algorithm, MappingList[j - 1].ComPort, MappingList[j - 1].DeviceType);
                    }
                }
            }*/
            
        }
        private void checkboxDevice_Checked(object sender, RoutedEventArgs e)
        {
            CheckBox currentBox = (CheckBox)sender;
            string sCurrentDeviceName = currentBox.Content.ToString();

            if (currentBox.Content.ToString() == "Select All")
            {
                for (int i = 0; i < PairedDeviceCollection.Count; i++)
                {
                    PairedDeviceCollection[i].IsChecked = true;
                    if (!CheckedDeviceList.Contains(PairedDeviceCollection[i].DeviceName))
                    {
                        CheckedDeviceList.Add(PairedDeviceCollection[i].DeviceName);
                    }
                }
            }
            else
            {
                if (!CheckedDeviceList.Contains(sCurrentDeviceName))
                {
                    CheckedDeviceList.Add(sCurrentDeviceName);
                }
            }
        }
        private void checkboxDevice_Unchecked(object sender, RoutedEventArgs e)
        {
            CheckBox currentBox = (CheckBox)sender;
            string sCurrentDeviceName = currentBox.Content.ToString();

            if (currentBox.Content.ToString() == "Select All")
            {
                for (int i = 0; i < PairedDeviceCollection.Count; i++)
                {
                    PairedDeviceCollection[i].IsChecked = false;
                }
                CheckedDeviceList.Clear();
            }
            else
            {
                for (int i = 0; i < CheckedDeviceList.Count; i++)
                {
                    if (CheckedDeviceList[i].ToString() == sCurrentDeviceName)
                    {
                        CheckedDeviceList.RemoveAt(i);
                        break;
                    }
                }
                /*PairedDeviceCollection[0].IsChecked = false; //Uncheck "Select All"
                for (int i = 0; i < PairedDeviceCollection.Count; i++) //...
                {
                    if (CheckedDeviceList.Contains(PairedDeviceCollection[i].DeviceName))
                    {
                        PairedDeviceCollection[i].IsChecked = true;
                    }
                }*/
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        protected void OnPropertyChanged(string propertyName)
        {
            PropertyChangedEventHandler handler = PropertyChanged;
            if (handler != null)
                handler(this, new PropertyChangedEventArgs(propertyName));
        }

        //closing main window event handler
        protected override void OnClosing(CancelEventArgs e)
        {
            base.OnClosing(e);
            Exit();
            Dict.Clear();
            PairedDeviceList.Clear();
            PairedDeviceCollection.Clear();
            CheckedDeviceList.Clear();
            ConnectedDeviceList.Clear();
            MappingList.Clear();
            AlgorithmCollection.Clear();
            MovingDeviceList.Clear();
            RemovedRoboBlobsCollection.Clear();
            Closing = true;
        }

        //end all involved processes
        protected override void OnClosed(EventArgs e)
        {
            base.OnClosed(e);

            try
            {
                foreach (Process proc in Process.GetProcessesByName("RobotSwarmGUI.vshost"))
                {
                    proc.Kill();
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }

        }

        private void listBox1_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

            //ToolTipService.SetToolTip(Button., "Hey");


        }

        /*private void Help_Click(object sender, RoutedEventArgs e)
        {
            Mouse.OverrideCursor = Cursors.Help;
            HelpCursor = true;
        }*/

        private void Disable_Help(object sender, RoutedEventArgs e)
        {

            if (MenuDisableHelp.IsChecked)
            {
                HelpEnabled = false;
            }
            else
            {
                HelpEnabled = true;
            }

            OnPropertyChanged("HelpEnabled");
        }



        private void Button_MouseEnter(object sender, MouseEventArgs e)
        {


            if (HelpCursor == true)
            {
                ToolTipService.SetIsEnabled(calibrateButton, true);
            }
            else
            {
                ToolTipService.SetIsEnabled(calibrateButton, false);
            }


        }
        //public event EventHandler AddRobotButton_Click;
        private AutoResetEvent _autoResetEvent = new AutoResetEvent(false);
        private void OnEvent(object sender, EventArgs e)
        {
            // _autoResetEvent.Set();
        }
        public int ComPortNumber;
        private void Add_Paired_Robot_Click(object sender, RoutedEventArgs e)
        {
            //Stop moving devices
            /*for (int i = 0; i < MovingDeviceList.Count; i++)
            {
                for (int j = 0; j < MappingList.Count; j++)
                {
                    if (MovingDeviceList[i] == MappingList[j].DeviceName)
                    {
                        Stop(MappingList[j].ComPort);
                        break;
                    }
                }
            }*/

            var AddPairedRobotsWindow = new AddPairedRobots();
            GlobalVar.PassComPort = -1;
            GlobalVar.PassDeviceNr = "";
            AddPairedRobotsWindow.ShowDialog();

            if (GlobalVar.PassComPort != -1)
            {
                String sDeviceName = "";// = "e-puck";
                int iDeviceType = 0; //= (int)RoboType.Epuck;

                if (sender.ToString().Contains("_Khepera"))
                {
                    sDeviceName = "KHIII";
                    iDeviceType = (int)RoboType.Khepera;
                }
                else if (sender.ToString().Contains("_Epuck"))
                {
                    sDeviceName = "e-puck";
                    iDeviceType = (int)RoboType.Epuck;
                }
                sDeviceName += GlobalVar.PassDeviceNr;
                PairedDeviceCollection.Add(new ObservableDeviceClass { DeviceName = sDeviceName, IsConnected = false, ConnectionState = Brushes.Gray, Algorithm = "" });
                MappingList.Add(new ComPortMapClass { DeviceName = sDeviceName, DeviceType = iDeviceType, ComPort = GlobalVar.PassComPort });
                PairedDeviceList.Add(sDeviceName);
                Console.WriteLine("{0} added", sDeviceName);

                Clear_vComPortPriority();
                for (int i = 0; i < MappingList.Count; i++)
                {
                    UpdateList(MappingList[i].ComPort);
                }

            }

            //Execute();

        }

        #endregion //Events

        #region Methods
        public string[] SearchAlgorithms()
        {
            
            string path = Directory.GetCurrentDirectory();
            path = path + "\\Algorithms\\";
            string[] filePaths = Directory.GetFiles(@path);
            string[] name = new string[filePaths.Length];

            for (int i = 0; i < filePaths.Length; i++)
            {
                name[i] = System.IO.Path.GetFileName(filePaths[i]);
                name[i] = name[i].Substring(0, name[i].Length - 2);
            }

            return name;
        }

		public delegate void NextDelegate();
        public void CreatePairedDeviceCollection()
        {
            PairedDeviceCollection = new ObservableCollection<ObservableDeviceClass>();
            PairedDeviceCollection.Add(new ObservableDeviceClass { DeviceName = "Select All"});
            this.DataContext = this;
        }
        public void CreateMappingList()
        {
            MappingList = new ObservableCollection<ComPortMapClass>();
            this.DataContext = this;
        }
        public void CreateAlgorithmCollection()
        {
            AlgorithmCollection = new ObservableCollection<AlgorithmStringClass>();

            AlgorithmCollection.Add(new AlgorithmStringClass { AlgName = "Walk" });
            AlgorithmString.Add("Walk");
            string[] AlgorithmNames = SearchAlgorithms();

            for (int i = 0; i < AlgorithmNames.Length; i++)
            {
                AlgorithmCollection.Add(new AlgorithmStringClass { AlgName = AlgorithmNames[i] });
                AlgorithmString.Add(AlgorithmNames[i]);
            }

            /*AlgorithmCollection.Add(new AlgorithmStringClass { AlgName = "Stop" });
            AlgorithmString.Add("Stop");*/

            //New algorithm names (e.g from XML file)

        }
        public void CreateRemovedRoboBlobsCollection()
        {
            RemovedRoboBlobsCollection = new ObservableCollection<string>();
            this.DataContext = this;
        }
        public string GetSelectedAlgorithm()
        {
            int index = AlgComboBox.SelectedIndex;

            if (index != -1)
            {
                return AlgorithmString[index];
            }
            else
            {
                return "";
            }
        }
        public void Execute()
        {
           /* if (FirstExecute)
            {
                Navigate("", 0, 0); // Fake navigate to force blob finding when first robot has to be assigned to a blob
                FirstExecute = false;
            }*/

       

            for (int i = 1; i < PairedDeviceCollection.Count; i++)
            {
                string sDeviceName = PairedDeviceCollection[i].DeviceName;
                string currentAlgorithm = PairedDeviceCollection[i].Algorithm;

                if (currentAlgorithm != "")
                {

                    if (currentAlgorithm != "stop" && currentAlgorithm != "idle")
                    {
                        /*if (currentAlgorithm == "Walk" && MappingList[i - 1].DeviceType != (int)RoboType.Khepera) //Non-loop algorithm
                        {
                            if (PairedDeviceCollection[i].AlgChanged == true)
                            {
                                Navigate(currentAlgorithm, MappingList[i - 1].ComPort, MappingList[i - 1].DeviceType);
                                PairedDeviceCollection[i].AlgChanged = false;
                            }
                            if (!MovingDeviceList.Contains(sDeviceName))
                            {
                                MovingDeviceList.Add(sDeviceName);
                            }
                        }
                        else*/
                        {
                            IntPtr removedListPtr = Navigate(currentAlgorithm, MappingList[i - 1].ComPort, MappingList[i - 1].DeviceType);



                            RemovedRoboBlobsCollection.Clear();

                            int[] pRemovedList = new int[100];

                            Marshal.Copy(removedListPtr, pRemovedList, 0, 100);

                            for (int j = 0; j < pRemovedList.Length; j++)
                            {
                                int comport = pRemovedList.ElementAt(j);
                               // Console.WriteLine("pRemovedList: ", comport);

                                if (pRemovedList[j] == 0)
                                {
                                    break;
                                }

                                for (int k = 0; k < MappingList.Count; k++)
                                {
                                    if (pRemovedList[j] == MappingList[k].ComPort)
                                    { 
                                        string name = MappingList[k].DeviceName;
                                        RemovedRoboBlobsCollection.Add(name);
                                        break;
                                    }
                                }

                                
                            }
                           
                            for (int j = 0; j < RemovedRoboBlobsCollection.Count; j++)
                            {
                                string name = RemovedRoboBlobsCollection.ElementAt(j);
                                //Console.WriteLine("RemovedRoboBlobsCollection: ", name);
                            }

                            //Dispatcher.Invoke(new Action(() => { }), DispatcherPriority.ContextIdle, null); //wait for the rendering to finish




                            if (PairedDeviceCollection[i].AlgChanged == true)
                            {
                                PairedDeviceCollection[i].AlgChanged = false;
                            }


                            if (!MovingDeviceList.Contains(sDeviceName))
                            {
                                MovingDeviceList.Add(sDeviceName);
                            }
                        }
                    }
                    else
                    {
                       // Navigate(currentAlgorithm, 0, -1);
                        if (MovingDeviceList.Contains(sDeviceName))
                        {
                            for (int l = 0; l < MovingDeviceList.Count; l++)
                            {
                                if (MovingDeviceList[l] == sDeviceName)
                                {
                                    MovingDeviceList.RemoveAt(l);
                                    break;
                                }
                            }
                        }
                    }
                }
                else
                {
                    //Navigate(currentAlgorithm, 0, -1);
                }
            }

            if (MovingDeviceList.Count > 0)
            {
                    startButton.Dispatcher.BeginInvoke(
                        System.Windows.Threading.DispatcherPriority.SystemIdle,
                        new NextDelegate(this.Execute));
            }

           

        }
        public void GetPairedDevices(CDict dict)
        {
            //Stop moving devices
            for (int i = 0; i < MovingDeviceList.Count; i++)
            {
                for (int j = 0; j < MappingList.Count; j++)
                {
                    if (MovingDeviceList[i] == MappingList[j].DeviceName)
                    {
                        Stop(MappingList[j].ComPort);
                        break;
                    }
                }
            }


            //Get Elisas

            Console.WriteLine("Elisa start");

            IntPtr elisaPtr = DiscoverElisa();
            int[] pElisa = new int[100];
            Marshal.Copy(elisaPtr, pElisa, 0, 100);

            for (int i = 0; i < pElisa.Length; i++)
            {
                if (pElisa[i] == 0)
                {
                    break;
                }
                string sAddress = pElisa[i].ToString();
                int iAddress = Convert.ToInt32(sAddress);
                string sName = "elisa";
                sName += sAddress;

                if (!PairedDeviceList.Contains(sName))
                {
                    PairedDeviceList.Add(sName);
                    PairedDeviceCollection.Add(new ObservableDeviceClass { DeviceName = sName, IsConnected = false, ConnectionState = Brushes.Gray, Algorithm = "" });
                    MappingList.Add(new ComPortMapClass { DeviceName = sName, DeviceType = (int)RoboType.Elisa, ComPort = iAddress });
                    Console.WriteLine("{0} added", sName);
                }
            }

            Console.WriteLine("Elisa end");


            //Get Epucks & Kheperas

            Console.WriteLine("Epucks,Khepera start");

            BluetoothClient bc = new BluetoothClient();
            bc.InquiryLength = TimeSpan.FromSeconds(20);
            BluetoothDeviceInfo[] array = bc.DiscoverDevices();

            Console.WriteLine("Discovery finished");

            int count = array.Length;

            for (int i = 0; i < count; i++)
            {

                string sDeviceName = array[i].DeviceName;

                if (PairedDeviceList.Contains(sDeviceName))
                {
                    continue;
                }

                BluetoothAddress sDeviceAddress = array[i].DeviceAddress;

                BluetoothDeviceInfo information = new BluetoothDeviceInfo(sDeviceAddress);
                String sInfo = information.DeviceAddress.ToString();
                String g = "";
                for (int j = 0; j < 10; j = j + 2)
                {
                    g = g + sInfo.Substring(j, 2) + ":";
                }
                g = g + sInfo.Substring(10, 2);

                DateTime startTime = DateTime.Now;
                TimeSpan duration;
                while (System.String.Compare(information.DeviceName.ToString(), g, false) == 0)
                {
                    information.Refresh();

                    DateTime currentTime = DateTime.Now;
                    duration = currentTime - startTime;
                    if (duration.Seconds >= 20)
                    {
                        Console.WriteLine("Time elapsed!");
                        break;
                    }
                }

                sDeviceName = information.DeviceName;
                array[i].DeviceName = sDeviceName;

                if (sDeviceName.StartsWith("KHIII") || sDeviceName.StartsWith("e-puck"))
                {
                    //Device device = new Device(array[i]);

                    sDeviceName = array[i].DeviceName;
                    sDeviceAddress = array[i].DeviceAddress;

                    BluetoothDeviceInfo info = new BluetoothDeviceInfo(sDeviceAddress);

                    string pin = "0000";

                    if (sDeviceName.StartsWith("e-puck"))
                    {
                        pin = sDeviceName.Substring(7, 4);
                    }

                    if (BluetoothSecurity.PairRequest(sDeviceAddress, pin))
                    {
                        if (info.Authenticated == false) //don't assign new port if already paired
                        {
                            info.SetServiceState(BluetoothService.SerialPort, true);    //Pair device
                        }
                        dict.Refresh_Name_Mac(sDeviceName, sDeviceAddress);
                        if (!PairedDeviceList.Contains(sDeviceName))
                        {
                            PairedDeviceCollection.Add(new ObservableDeviceClass { DeviceName = sDeviceName, IsConnected = false, ConnectionState = Brushes.Gray, Algorithm = "" });
                            Console.WriteLine("{0} added", sDeviceName);
                        }
                    }
                    else
                    {
                        Console.WriteLine("Could not pair {0}", sDeviceName);
                    }
                }
            }

            dict.Refresh_Mac_Com();

            for (int i = 1; i < PairedDeviceCollection.Count; i++)
            {
                string sDeviceName = PairedDeviceCollection[i].DeviceName;
                int iDeviceType;

                if (!PairedDeviceList.Contains(sDeviceName))
                {

                    if (sDeviceName.StartsWith("KHIII"))
                    {
                        iDeviceType = (int)RoboType.Khepera;
                    }
                    else
                    {
                        iDeviceType = (int)RoboType.Epuck;
                    }

                    dict.Refresh_Name_Com(sDeviceName);

                    MappingList.Add(new ComPortMapClass { DeviceName = sDeviceName, DeviceType = iDeviceType, ComPort = dict.Name_Com[sDeviceName] });
                    PairedDeviceList.Add(sDeviceName);
                }
                else
                {
                    if (!sDeviceName.StartsWith("elisa"))
                    {
                        Console.WriteLine("{0} already paired", sDeviceName);
                    }
                }

            }

            Clear_vComPortPriority();
            for (int i = 0; i < MappingList.Count; i++)
            {
                UpdateList(MappingList[i].ComPort);
            }

            Execute();

        }
        public void GetRememberedDevices() //List Devices
        {
            //Stop moving devices
            for (int i = 0; i < MovingDeviceList.Count; i++)
            {
                for (int j = 0; j < MappingList.Count; j++)
                {
                    if (MovingDeviceList[i] == MappingList[j].DeviceName)
                    {
                        Stop(MappingList[j].ComPort);
                        break;
                    }
                }
            }


            //Elisa start

            IntPtr elisaPtr = DiscoverElisa();
            int[] pElisa = new int[100];
            Marshal.Copy(elisaPtr, pElisa, 0, 100);

            for (int i = 0; i < pElisa.Length; i++)
            {
                if (pElisa[i] == 0)
                {
                    break;
                }
                string sAddress = pElisa[i].ToString();
                int iAddress = Convert.ToInt32(sAddress);
                string sName = "elisa";
                sName += sAddress;

                if (!PairedDeviceList.Contains(sName))
                {
                    PairedDeviceList.Add(sName);
                    PairedDeviceCollection.Add(new ObservableDeviceClass { DeviceName = sName, IsConnected = false, ConnectionState = Brushes.Gray, Algorithm = "" });
                    MappingList.Add(new ComPortMapClass { DeviceName = sName, DeviceType = (int)RoboType.Elisa, ComPort = iAddress });
                    Console.WriteLine("{0} added", sName);
                }
            }

            //Elisa end

            Console.WriteLine("Epucks,Khepera start");

            Dict.Refresh_Mac_Com();

            BluetoothClient bc = new BluetoothClient();
            bc.InquiryLength = TimeSpan.FromSeconds(20);
            Console.WriteLine("discovery started");
            BluetoothDeviceInfo[] array = bc.DiscoverDevices();
            Console.WriteLine("discovery finished");

            int iDeviceType;

            for (int i = 0; i < array.Length; i++)
            {
                string sDeviceName = array[i].DeviceName;
                BluetoothAddress DeviceAddress = array[i].DeviceAddress;

                if (!PairedDeviceList.Contains(sDeviceName) && Dict.Mac_Com.ContainsKey(DeviceAddress.ToString()))
                {

                    if (sDeviceName.StartsWith("KHIII") || sDeviceName.StartsWith("e-puck"))
                    {
                        Dict.Refresh_Name_Mac(sDeviceName, DeviceAddress);
                        Dict.Refresh_Name_Com(sDeviceName);


                        if (sDeviceName.StartsWith("KHIII"))
                        {
                            iDeviceType = (int)RoboType.Khepera;
                        }
                        else
                        {
                            iDeviceType = (int)RoboType.Epuck;
                        }

                        PairedDeviceCollection.Add(new ObservableDeviceClass { DeviceName = sDeviceName, IsConnected = false, ConnectionState = Brushes.Gray, Algorithm = "" });
                        MappingList.Add(new ComPortMapClass { DeviceName = sDeviceName, DeviceType = iDeviceType, ComPort = Dict.Name_Com[sDeviceName] });
                        PairedDeviceList.Add(sDeviceName);
                        Console.WriteLine("{0} added", sDeviceName);
                    }
                }
            }

            Clear_vComPortPriority();
            for (int i = 0; i < MappingList.Count; i++)
            {
                UpdateList(MappingList[i].ComPort);
            }

            Execute();
        }

        public IntPtr MappingListToIntPtr()
        {
            int elementSize = Marshal.SizeOf(typeof(IntPtr));
            IntPtr listPtr = Marshal.AllocHGlobal(MappingList.Count * elementSize);

            for (int i = 0; i < MappingList.Count; i++)
            {
                Marshal.WriteIntPtr(listPtr, MappingList[i].ComPort, ((IntPtr)(i + 1)));
            }

            return listPtr;
        }

        [DllImport("RobotSwarmDLL")]
        public static extern IntPtr DiscoverElisa();
        [DllImport("RobotSwarmDLL")]
        public static extern bool Connect(int iDeviceType, int iComPort, int iPriority);
        [DllImport("RobotSwarmDLL")]
        public static extern bool Disconnect(int iDeviceType, int iComPort);
        //[DllImport("RobotSwarmDLL")]
        //public static extern bool Walk(int iComPort);
        [DllImport("RobotSwarmDLL")]
        public static extern bool Stop(int iComPort);
        [DllImport("RobotSwarmDLL")]
        public static extern bool Calibrate(int iComPort);
        [DllImport("RobotSwarmDLL")]
        public static extern void Exit();
        [DllImport("RobotSwarmDLL")]
        public static extern IntPtr Navigate(string sAlgName, int iComPort, int iType);
        [DllImport("RobotSwarmDLL")]
        public static extern void UpdateList(int comPort);
        [DllImport("RobotSwarmDLL")]
        public static extern void RemoveFromList(int index);
        [DllImport("RobotSwarmDLL")]
        public static extern void Clear_vComPortPriority();
        
        #endregion //Methods
        





    }

    public class AlgorithmStringClass
    {
        public string AlgName { get; set; }
    }

    public class ObservableDeviceClass : INotifyPropertyChanged
    {
        private Brush connectionState;
        private string algorithm;
        private bool isChecked;
        

        public bool IsConnected { get; set; }
        public bool AlgChanged { get; set; }
        public string DeviceName { get; set; }
        public Brush ConnectionState
        {
            get { return connectionState; }
            set
            {
                if (value != connectionState)
                {
                    connectionState = value;
                    OnPropertyChanged("ConnectionState");
                }
            }
        }
        public string Algorithm
        {
            get { return algorithm; }
            set
            {
                if (value != algorithm)
                {
                    algorithm = value;
                    OnPropertyChanged("Algorithm");
                    AlgChanged = true;
                }
            }
        }
        public bool IsChecked
        {
            get { return isChecked; }
            set
            {
                if (value != isChecked)
                {
                    isChecked = value;
                    OnPropertyChanged("IsChecked");
                }
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        protected void OnPropertyChanged(string propertyName)
        {
            PropertyChangedEventHandler handler = PropertyChanged;
            if (handler != null)
                handler(this, new PropertyChangedEventArgs(propertyName));
        }

        public override string ToString()
        {
            return DeviceName;
        }
    }

    public class ComPortMapClass
    {
        public string DeviceName { get; set; }
        public int DeviceType { get; set; }
        public int ComPort { get; set; }
    }

    public class CDict
    {
        public Dictionary<string, int> Mac_Com { get; set; }
        public Dictionary<string, BluetoothAddress> Name_Mac { get; set; }
        public Dictionary<string, int> Name_Com { get; set; }

        public CDict()
        {
            Mac_Com = new Dictionary<string, int>();
            Name_Mac = new Dictionary<string, BluetoothAddress>();
            Name_Com = new Dictionary<string, int>();
        }

        public void Refresh_Mac_Com()
        {
            SelectQuery q = new SelectQuery("Win32_SerialPort");
            ManagementObjectSearcher s = new ManagementObjectSearcher(q);
            foreach (object cur in s.Get())
            {
                ManagementObject mo = (ManagementObject)cur;
                object id = mo.GetPropertyValue("DeviceID");
                string sComPort = id.ToString();
                int iComPort = Convert.ToInt32(sComPort.Substring(3));
                object desc = mo.GetPropertyValue("Description");

                if (desc.ToString() != "Standard Serial over Bluetooth link")
                {
                    continue;
                }

                object pnpId = mo.GetPropertyValue("PNPDeviceID");
                int pnpIdLength = pnpId.ToString().Length;
                string sMACAddr = pnpId.ToString().Substring(pnpIdLength - 22, 12);

                if (!(Mac_Com.ContainsKey(sMACAddr) || Mac_Com.ContainsValue(iComPort)))
                {
                    Mac_Com.Add(sMACAddr, iComPort);
                }
            }
        }

        public void Refresh_Name_Mac(string sName, BluetoothAddress sMACAddr)
        {
            if (!(Name_Mac.ContainsKey(sName) || Name_Mac.ContainsValue(sMACAddr)))
            {
                Name_Mac.Add(sName, sMACAddr);
            }
        }

        public void Refresh_Name_Com(string sName)
        {
            if (Name_Mac.ContainsKey(sName))
            {
                string sMACAddr = Name_Mac[sName].ToString();

                if (Mac_Com.ContainsKey(sMACAddr))
                {
                    int iComPort = Mac_Com[sMACAddr];

                    if (!(Name_Com.ContainsKey(sName) || Name_Com.ContainsValue(iComPort)))
                    {
                        Name_Com.Add(sName, iComPort);
                    }

                }
            }
        }

        public void ShowPorts()
        {
            foreach (var pair in Name_Com)
            {
                Console.WriteLine("{0}: {1}",
                pair.Key,
                pair.Value);
            }
        }

        public void Clear()
        {
            Mac_Com.Clear();
            Name_Mac.Clear();
            Name_Com.Clear();
        }
    }

    public class Device
    {

        public string DeviceName { get; set; }
        public BluetoothAddress DeviceAddress { get; set; }
        public bool Authenticated { get; set; }
        public bool Connected { get; set; }
        public ushort Nap { get; set; }
        public uint Sap { get; set; }
        public DateTime LastSeen { get; set; }
        public DateTime LastUsed { get; set; }
        public bool Remembered { get; set; }

        public Device(BluetoothDeviceInfo device_info)
        {
            Authenticated = device_info.Authenticated;
            Connected = device_info.Connected;
            DeviceName = device_info.DeviceName;
            DeviceAddress = device_info.DeviceAddress;
            LastSeen = device_info.LastSeen;
            LastUsed = device_info.LastUsed;
            Nap = device_info.DeviceAddress.Nap;
            Sap = device_info.DeviceAddress.Sap;
            Remembered = device_info.Remembered;
        }

        public override string ToString()
        {
            return this.DeviceName;
        }

    }

    enum RoboType { Khepera, Epuck, Elisa };

    public static class GlobalVar //for passing ComPort and DeviceNr from one window to another
    {

        static int iPassComPort;
        static String iPassDeviceNr;
        public static int PassComPort
        {
            get
            {
                return iPassComPort;
            }
            set
            {
                iPassComPort = value;
            }
        }

        public static String PassDeviceNr
        {
            get
            {
                return iPassDeviceNr;
            }
            set
            {
                iPassDeviceNr = value;
            }
        }
    }
   
}

