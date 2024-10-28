using System;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using SlimDX.DirectInput;
using System.Timers;
using System.IO;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Net.NetworkInformation;
using System.Windows.Media.Imaging;
using System.ComponentModel;
using HelixToolkit.Wpf;
using System.Windows.Media.Media3D;
using System.Windows.Input;


namespace Robot41
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private readonly System.Timers.Timer timerJoy;                      //Controller timer 50ms
        private readonly System.Timers.Timer timerBatt;                     //Battery timer    5000ms
        private readonly System.Timers.Timer timerIMU;                      //IMU timer        200ms
        private readonly System.Timers.Timer timerMap;                      //IMU timer        333ms
        private readonly Stopwatch stmPingWatch = new Stopwatch();          //STM32 Ping
        private readonly BackgroundWorker worker = new BackgroundWorker();  //RX
        private Joystick controller;

        static int mapSize = 256;
        static int mapX = mapSize;
        static int mapY = mapSize;
        WriteableBitmap MapBmp = BitmapFactory.New(mapX, mapY);
        WriteableBitmap JoyBmp = BitmapFactory.New(100, 100);
        byte[] battW = new byte[80];                                        //Width and Height of ImageBatt
        static byte battH = 28;
        WriteableBitmap BatBmp = BitmapFactory.New(80, battH);
        WriteableBitmap TrkBmp = BitmapFactory.New(81, 129);                //Show tracks

        int xX, yX, aX, bX, cX, dX, eX, fX = 0;                             //Axes. topori)
        bool cc = false;                                                    //cc-controller connect
        readonly int packetlenght = 300;                                    //both tr and rx, bytes, set ProgressBarNetworkLoad.maximum
        ushort dataTX, dataRX;
        byte comandaTX, comandaRX;
        byte[] shLem = new byte[5];                                         //RX. TX v kode
        float bat_volt = 0.01f;                                             //Battery vars
        float bat_current = 0f;
        int PwmTempL, PwmTempR, pwmR, pwmL;
        double[] trrR = new double[16];                                     //Tracks visualisation
        double[] trrL = new double[16];
        uint countErrors = 0;                                               //Hz. tupaya oshibka. Hochu znat'
        int pitch, yaw, roll, north = 0;                                    //IMU zhe..
        byte hatchStatus = 5;                                               //0-closed, 1-opened, 2-absent, 3-error, 4-over current, 5-Not defined
        bool lidar = false;                                                 //extend-retract
        int[] allRange = new int[1024];                                     //
        static short lidarDirection = 512;                                  //center
        byte lidarResolution = 4;                                           //32 or less i guess
        int[] xl320 = new int[6];                                           //Positions

        public string SETTINGS_FILE = "settings.air";
        public string hostname = "192.168.3.4";
        public string port = "54321";
        UdpClient udpClient = new UdpClient();
        private IPEndPoint RemoteIPEndPoint;

        readonly Random rnd = new Random();

        //3D:
        private const string MODEL_PATH = "robot.obj";
        int angleX = 0;//36;r
        int angleY = 0;//12;     y        //start position
        int angleZ = 180;//-114;p
        Matrix3D matrix;
        ModelVisual3D Robot3D = new ModelVisual3D();
        Model3DGroup model = null;
        private readonly System.Timers.Timer timer3D;


        public MainWindow()
        {
            SetValue(TextOptions.TextFormattingModeProperty, TextFormattingMode.Display);                                                    //Smoothing must die!
            ToolTipService.ShowDurationProperty.OverrideMetadata(typeof(DependencyObject), new FrameworkPropertyMetadata(Int32.MaxValue));   //ToolTips stays forever.
            //RenderOptions.SetBitmapScalingMode(this, BitmapScalingMode.HighQuality);

            InitializeComponent();

            ListBoxStatus.Items.Insert(0, "ROBOT control v4.1   10.2024   (c)Airrr(r)");

            timerJoy = new System.Timers.Timer(50);
            timerJoy.Elapsed += new ElapsedEventHandler(TimerJoy_Tick);
            timerJoy.Stop();
            timerBatt = new System.Timers.Timer(5000);
            timerBatt.Elapsed += new ElapsedEventHandler(TimerBatt_Tick);
            timerBatt.Stop();
            timerIMU = new System.Timers.Timer(200);
            timerIMU.Elapsed += new ElapsedEventHandler(TimerIMU_Tick);
            timerIMU.Stop();
            timerMap = new System.Timers.Timer(333);
            timerMap.Elapsed += new ElapsedEventHandler(TimerMap_Tick);
            timerMap.Stop();
            timer3D = new System.Timers.Timer(200);
            timer3D.Elapsed += new ElapsedEventHandler(Timer3D_Tick);
            timer3D.Stop();

            SliderLidar.Value = lidarDirection;   //Default

            worker.DoWork += Worker_DoWork;
            worker.RunWorkerCompleted += Worker_RunWorkerCompleted;

            //Some settings_file preparations
            if (File.Exists(SETTINGS_FILE))
            {
                StreamReader settingsfile = new StreamReader(SETTINGS_FILE);
                hostname = settingsfile.ReadLine();
                if (IPAddress.TryParse(hostname, out IPAddress address))
                {
                    port = settingsfile.ReadLine();
                    if (port == null || !port.All(char.IsDigit) || int.Parse(port) > 65535)
                    {
                        ListBoxStatus.Items.Insert(0, "Port number in setting file is invalid. Deleting. Creating default one.");
                        settingsfile.Close();
                        File.Delete(SETTINGS_FILE);
                        hostname = "192.168.3.4";
                        CreateFile();
                    }
                }
                else
                {
                    ListBoxStatus.Items.Insert(0, "IP in setting file is invalid. Deleting. Creating default one.");
                    settingsfile.Close();
                    File.Delete(SETTINGS_FILE);
                    hostname = "192.168.3.4";
                    CreateFile();
                }
                settingsfile.Close();
            }
            else
            {
                ListBoxStatus.Items.Insert(0, "No settings file? Creating default one.");
                ListBoxStatus.SelectedIndex = ListBoxStatus.Items.Count - 1;
                ListBoxStatus.Focus();
                hostname = "192.168.3.4";
                CreateFile();
            }
            labelip.Content = hostname + ":" + port;
            for (int ba = 0; ba < battW.Length - 1; ba++)          //Battery dummy sett
            {
                battW[ba] = 3;
            }
            ButtonPing_Click(null, null);

            for (byte trktemp = 0; trktemp < 16; trktemp++)     //Some tracks shit
            {
                trrR[trktemp] = trktemp * 8;
                trrL[trktemp] = trktemp * 8;
            }

            //3D:
            Robot3D.Content = Display3d(MODEL_PATH);
            viewPort3d.Children.Add(Robot3D);
            viewPort3d.ZoomExtents();
            
        }
        private void TimerJoy_Tick(object source, ElapsedEventArgs e)         //Controller tick!766666666666
        {
            if (controller.Acquire().IsFailure)
            {
                ListBoxStatus.Items.Insert(0, "Controller is not connected or out of scope, just go?");
                return;
            }
            if (controller.Poll().IsFailure)
            {
                ListBoxStatus.Items.Insert(0, "Controller poll failure, whatever this means.");
                return;
            }
            JoystickState state = new JoystickState();

            state = controller.GetCurrentState();

            int[] slider = state.GetSliders();
            xX = state.X;
            yX = state.Y;
            aX = state.Z;
            bX = state.RotationX;
            cX = state.RotationY;
            dX = state.RotationZ;
            eX = slider[0];
            fX = slider[1];

            this.Dispatcher.Invoke(() =>            //Drawing joystick
            {
                using (JoyBmp.GetBitmapContext())
                {
                    JoyBmp.Clear();
                    ImageXY.Source = JoyBmp;
                    JoyBmp.DrawEllipse(0, 0, 99, 99, Color.FromArgb(255, 255, 215, 5)); //Outer circle
                    JoyBmp.DrawEllipseCentered((int)(xX / 41.5), (int)(yX / 41.5), 2, 2, Color.FromArgb(255, 255, 255, 255)); //Inner circle
                }
                ProgressBarA.Value = aX / 41;
                ProgressBarB.Value = bX / 41;
                ProgressBarC.Value = cX / 41;
                ProgressBarD.Value = dX / 41;
                ProgressBarE.Value = eX / 41;
                ProgressBarF.Value = fX / 41;
                ProgressBarA.ToolTip = aX;
                ProgressBarB.ToolTip = bX;
                ProgressBarC.ToolTip = cX;
                ProgressBarD.ToolTip = dX;
                ProgressBarE.ToolTip = eX;
                ProgressBarF.ToolTip = fX;
                LabelX.Content = "X=" + xX;
                LabelY.Content = "Y=" + yX;
            });

            if (udpClient.Client.Connected)
            {
                if (xX > 2000 && xX < 2100) xX = 2048;                        //DEADZONE software control.
                if (yX > 2000 && yX < 2100) yX = 2048;                        //BETTER USE JOYSTICK WITH HARDWARE DEAD ZONES!
                PwmTempL = (((yX * 2) - 4096) - ((xX * 2) - 4096)) / -2;      //Differential
                PwmTempR = (((yX * 2) - 4096) + ((xX * 2) - 4096)) / -2;
                if (PwmTempL > 2048) PwmTempL = 2048;                         //Na vsyakii
                if (PwmTempL < -2047) PwmTempL = -2047;
                if (PwmTempR > 2048) PwmTempR = 2048;
                if (PwmTempR < -2047) PwmTempR = -2047;
                if (PwmTempL < 2 && PwmTempL > -2) PwmTempL = 0;              //2->- 2   2 for joystick with DZ
                if (PwmTempR < 2 && PwmTempR > -2) PwmTempR = 0;              //2->- 2

                if (PwmTempR != pwmR)
                {
                    comandaTX = 30;
                    dataTX = (ushort)(PwmTempR + 2047);
                    Posilka();
                }
                if (PwmTempL != pwmL)
                {
                    comandaTX = 31;
                    dataTX = (ushort)(PwmTempL + 2047);
                    Posilka();
                }
                DrawTracks();
            }
            pwmR = PwmTempR;
            pwmL = PwmTempL;
        }
        private void TimerBatt_Tick(object source, ElapsedEventArgs e)        //Battery tick!
        {
            this.Dispatcher.Invoke(() =>                                      //Clear canvas
            {
                BatBmp.Clear();
                for (int ba = 0; ba < battW.Length - 1; ba++)
                {
                    using (BatBmp.GetBitmapContext())
                    {
                        ImageBat.Source = BatBmp;
                        Color col = Colors.Crimson;
                        if (battW[ba] > 21) col = Colors.LemonChiffon;       //21 = 2*10.5v
                        if (battW[ba] > 23) col = Colors.LimeGreen;          //23 = 2*11.5v
                        BatBmp.DrawLine(ba + 1, battH, ba + 1, 28 - battW[ba], col);
                        BatBmp.DrawLine(ba + 1, battH, ba + 1, 28 - battW[ba], col);
                    }
                }
            });
            ButtonVolt_Click(null, null);
        }
        private void TimerIMU_Tick(object source, ElapsedEventArgs e)         //IMU timer tick
        {
            this.Dispatcher.Invoke(() =>                                      //Clear canvas
            {
                RotateTransform rotateTransform = new RotateTransform(yaw - north);
                ImageYAW.RenderTransform = rotateTransform;
                LabelYAW.Content = (yaw - north) + "°";
            });
            comandaTX = 119;
            dataTX = 10;
            Posilka();
        }
        private void TimerMap_Tick(object source, ElapsedEventArgs e)         //Map rendering tick
        {
            this.Dispatcher.Invoke(() =>
            {
                using (MapBmp.GetBitmapContext())
                {
                    MapBmp.Clear();
                    ImageMap.Source = MapBmp;
                    MapBmp.DrawRectangle(0, 0, mapX, mapY, Color.FromArgb(255, 255, 0, 0));  //border
                    MapBmp.DrawRectangle(mapX / 2 - 10, mapY / 2 - 5, mapX / 2 + 10, mapY / 2 + 25, Color.FromArgb(255, 255, 130, 0)); //robot
                    MapBmp.DrawEllipse(mapX / 2 - 2, mapY / 2 - 2, mapX / 2 + 2, mapY / 2 + 2, Color.FromArgb(255, 255, 130, 0));  //lidar
                    int highest = allRange.Max();
                    if (highest > 1200) highest = 1200;

                    for (ushort j = 0; j < 1023; j++)
                    {
                        //allRange[j] = rnd.Next(20, 80);

                        if (allRange[j] > 1200) allRange[j] = 1200;
                        //if (allRange[j] < 20) allRange[j] = 20;
                        if (allRange[j] != 0)
                        {
                            double lng = allRange[j] / (((double)highest / (double)(mapSize / 2D)));   ///AutoZoom!! 128- polovina imaga
                           // if (highest < (mapSize / 2)) lng = allRange[j];

                            var rot = 30;       //(360-300)/2
                            int xp = (int)(Math.Sin((((j * 0.293) + rot) * 2 * Math.PI) / 360) * lng);
                            int yp = (int)(Math.Cos((((j * 0.293) + rot) * 2 * Math.PI) / 360) * lng);
                            MapBmp.DrawLine(mapX / 2, mapY / 2, xp + mapX / 2, yp + mapY / 2, Color.FromArgb(255, 0, 255, 255));
                        }
                    }
                }
            });
        }
        private void Timer3D_Tick(object source, ElapsedEventArgs e)          //Show 3D!!!
        {
            comandaTX = 119;            //Get YAW, PITCH and ROLL at once.
            dataTX = 17;
            Posilka();

            this.Dispatcher.Invoke(() =>
            {
                matrix.Rotate(new Quaternion(new Vector3D(0, 0, 1) * matrix, (roll - angleX)));
                matrix.Rotate(new Quaternion(new Vector3D(0, 1, 0) * matrix, (angleY - yaw)));
                matrix.Rotate(new Quaternion(new Vector3D(1, 0, 0) * matrix, (pitch - angleZ)));
                Robot3D.Transform = new MatrixTransform3D(matrix);
                angleX = roll;
                angleY = yaw;
                angleZ = pitch;
                RotateTransform rotateTransform = new RotateTransform(yaw - north);
                ImageYAW.RenderTransform = rotateTransform;
                LabelYAW.Content = (yaw - north) + "°";
            });
        }
        private void JoystickControl_Click(object sender, RoutedEventArgs e)  //Controller En\Dis
        {
            DirectInput dinput = new DirectInput();
            if (cc)                                                           //Connected? release!
            {
                JoystickControl.Background = Brushes.Maroon;
                cc = false;
                timerJoy.Stop();
                controller.Unacquire();
                controller.Dispose();
                dinput.Dispose();
                ListBoxStatus.Items.Insert(0, "Controller disconnected!");
                JoystickControl.Content = "Start";
                return;
            }
            foreach (DeviceInstance device in dinput.GetDevices(DeviceClass.GameController, DeviceEnumerationFlags.AttachedOnly))
            {
                try
                {
                    controller = new Joystick(dinput, device.InstanceGuid);
                    controller.Acquire();
                    foreach (DeviceObjectInstance deviceObject in controller.GetObjects())
                    {
                        if ((deviceObject.ObjectType & ObjectDeviceType.Axis) != 0)
                        {
                            controller.GetObjectPropertiesById((int)deviceObject.ObjectType).SetRange(0, 4095);
                        }
                    }
                    controller.Acquire();
                    JoystickControl.Background = Brushes.DarkGreen;
                    cc = true;
                    timerJoy.Start();                                    //Start! Go timer tick
                    ListBoxStatus.Items.Insert(0, "Controller connected!");
                    JoystickControl.Content = "Stop";
                }
                catch (DirectInputException)
                {
                    ListBoxStatus.Items.Insert(0, "DirectInputException");
                    dinput.Dispose();
                }
            }
        }
        private void CreateFile()                                             //Create default settings.air
        {
            using (StreamWriter settingsfile = new StreamWriter(SETTINGS_FILE))
            {
                port = "54321";
                settingsfile.WriteLine(hostname);
                settingsfile.WriteLine(port);
                settingsfile.Close();
                labelip.Content = hostname + ":" + port;
            }
        }
        private void ButtonPing_Click(object sender, RoutedEventArgs e)       //Network PING
        {
            Ping ping = new Ping();
            PingReply pingresult = ping.Send(hostname);
            if (pingresult.Status.ToString() == "Success")
            {
                ButtonPing.Background = Brushes.DarkGreen;
                ListBoxStatus.Items.Insert(0, "Ping success.");
                ButtonNET.IsEnabled = true;
            }
            else
            {
                ButtonPing.Background = Brushes.Maroon;
                ListBoxStatus.Items.Insert(0, "Ping fail. (firewall?)");
            }
            ping.Dispose();
        }
        private void ButtonNET_Click(object sender, RoutedEventArgs e)        //Network connect!!
        {
            try
            {
                udpClient.Connect(hostname, int.Parse(port));
                if (udpClient.Client.Connected)
                {
                    udpClient.Client.ReceiveBufferSize = packetlenght;
                    udpClient.Client.SendBufferSize = packetlenght;
                    udpClient.DontFragment = true;
                    udpClient.Client.DontFragment = true;
                    udpClient.Client.ReceiveTimeout = 50;
                    udpClient.Client.Blocking = false;
                    ListBoxStatus.Items.Insert(0, "Seems connected.");
                    ButtonNET.Background = Brushes.DarkGreen;
                }
            }
            finally
            {
                if (!worker.IsBusy) worker.RunWorkerAsync();
                timerBatt.Start();
                ButtonSTM32ping.IsEnabled = true;
                ButtonFind.IsEnabled = false;
                comandaTX = 21;
                dataTX = 21;
                Posilka();                            //Hatch Status request
            }
        }
        private void Posilka()                                                //Transmit to robot
        {
            if (udpClient.Client.Connected)
            {
                shLem[0] = 65;                                   // A
                byte[] bytes = BitConverter.GetBytes(dataTX);
                shLem[1] = comandaTX;
                shLem[2] = bytes[1];
                shLem[3] = bytes[0];
                shLem[4] = (byte)(shLem[0] ^ shLem[1] ^ shLem[2] ^ shLem[3]);
                udpClient.Send(shLem, 5);
                Thread.Sleep(2);
            }
        }
        private void Worker_DoWork(object sender, DoWorkEventArgs e)          //Receiving!
        {
            // run all background tasks here
            int ava = udpClient.Client.Available;
            this.Dispatcher.Invoke(() =>                                      //Progress bar update
            {
                ProgressBarNetworkLoad.Value = ava;
            });
            if (ava >= 5)
            {
                //if (ava > 90) ListBoxStatus.Items.Insert(0, "RX buffer > 90 of " + packetlenght);  //blokiruet nah (((
                byte[] chiTaem = { 0, 0, 0, 0, 0 };                                            //TX
                chiTaem = udpClient.Receive(ref RemoteIPEndPoint);
                if (chiTaem[0] == 65 || chiTaem[0] == 82)                                      //Tupoi kostil. Debilnii do zhopi (( Should it be the greatest kostil ever?
                {
                    if (chiTaem[4] == (chiTaem[0] ^ chiTaem[1] ^ chiTaem[2] ^ chiTaem[3]))     //checksum OK
                    {
                        if (chiTaem[0] == 65)                                                  //Standard packet!
                        {
                            dataRX = (ushort)(chiTaem[2] * 256 + chiTaem[3]);
                            comandaRX = chiTaem[1];
                        }
                        if (chiTaem[0] == 82)                                                  //Range packet!
                        {
                            dataRX = (ushort)(chiTaem[2] * 256 + chiTaem[3]);
                            comandaRX = chiTaem[1];
                            var hzz1 = (ushort)((dataRX >> 12) << 8) | comandaRX;        //Ugol
                            var hzz2 = (ushort)(dataRX << 4) >> 4; // 'ostavit' 4 bita levie  //Range
                            //    If hzz1 > 1023 Or hzz2 > 10000 Then GoTo vse   ' na vsyakii sluchai. raz bilo pochemu-to
                            //    allrange(hzz1) = hzz2     ' POTOM SOKRATIT' ETU HEROTU
                            if (hzz1 < 1024 && hzz2 < 10000) allRange[hzz1] = hzz2;

                            this.Dispatcher.Invoke(() =>
                            {
                                LabelDist.Content = hzz2 + "cm, angle=" + hzz1 + ", new";
                            });

                        }
                    }
                }
                else
                {
                    countErrors++;   //Esli buffer overrun. mozhno vivesti.
                }
            }
        }
        private void Worker_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e) //RX Processing
        {
            if (comandaRX == 100 && dataRX == 100) ListBoxStatus.Items.Insert(0, "xaxa, defaultine nastroiki prinyati. ochen' stranno.");
            if (comandaRX == 20)                                           //Battery voltage
            {
                bat_volt = (float)Math.Round(dataRX * 0.00344, 2);
                if (bat_volt < 15)                                     //Bivaet vsyako
                {
                    buttonVolt.Content = bat_volt + "V";
                    for (int ba = 0; ba < battW.Length - 1; ba++)      //Shift and add
                    {
                        battW[ba] = battW[ba + 1];
                    }
                    battW[battW.Length - 2] = (byte)(bat_volt * 2);
                }
            }
            if (comandaRX == 19)                                           //Battery current
            {
                bat_current = (float)Math.Round((((((float)dataRX / (float)4095) * (float)3289) - (float)2530) / (float)170 * (float)1000));
                if (bat_current > -1100 && bat_current < 2000) ButtonCurrent.Content = bat_current + "mA";
            }
            if (comandaRX == 98) ListBoxStatus.Items.Insert(0, "Start sequence received with random " + dataRX);
            if (comandaRX == 55 && dataRX == 55)
            {
                stmPingWatch.Stop();
                TimeSpan tshz = stmPingWatch.Elapsed;
                ButtonSTM32ping.Content = tshz.Milliseconds + " ms";
            }                    //STM32 Ping
            if (comandaRX == 117) LabeldataPWMR.Content = dataRX;     //RPM right
            if (comandaRX == 118) LabeldataPWML.Content = dataRX;     //RPM left
            if (comandaRX == 120) yaw = dataRX;
            if (comandaRX == 121)
            {
                pitch = dataRX;
                ButtonPitch.Content = "Pitch: " + pitch + "°";
            }
            if (comandaRX == 122)
            {
                roll = dataRX;
                ButtonRoll.Content = "Roll: " + roll + "°";
            }
            if (comandaRX == 21)
            {
                hatchStatus = (byte)dataRX;
                switch (hatchStatus)
                {
                    case 0:
                        LabelHatch.Foreground = (Brushes.White);
                        LabelHatch.Content = "Closed";
                        ButtonLidar.IsEnabled = true;
                        ButtonLidar.Content = "Extend";
                        break;
                    case 1:
                        LabelHatch.Foreground = (Brushes.Green);
                        LabelHatch.Content = "Opened";
                        ButtonLidar.IsEnabled = true;
                        lidar = true;
                        timerMap.Start();
                        ButtonLidar.Content = "Retract";
                        break;
                    case 2:
                        LabelHatch.Foreground = (Brushes.Red);
                        LabelHatch.Content = "Absent";
                        ButtonLidar.IsEnabled = false;
                        break;
                    case 3:
                        LabelHatch.Foreground = (Brushes.Red);
                        LabelHatch.Content = "Error";
                        ButtonLidar.IsEnabled = false;
                        ListBoxStatus.Items.Insert(0, "Hatch ERROR!");
                        break;
                    case 4:
                        LabelHatch.Foreground = (Brushes.Red);
                        LabelHatch.Content = "Over current";
                        ButtonLidar.IsEnabled = false;
                        ListBoxStatus.Items.Insert(0, "Hatch over current!");
                        break;
                }

            }                                    //Hatch status change and print
            if (comandaRX == 50)
            {
                ButtonDist.Content = dataRX;
                allRange[lidarDirection] = dataRX;
                this.Dispatcher.Invoke(() =>
                {
                    LabelDist.Content = dataRX + "cm, angle=" + lidarDirection + ", old";
                });
            }                                    //Range at current lidar direction
            if (comandaRX == 45) ServoRead();                         //XL320

            if (comandaRX == 32 && dataRX == 1000) ListBoxStatus.Items.Insert(0, "Turn right failed.");
            if (comandaRX == 33 && dataRX == 1000) ListBoxStatus.Items.Insert(0, "Turn left failed.");
            if (comandaRX == 32 && dataRX < 1000) ListBoxStatus.Items.Insert(0, "Turn right complete. Diff: " + dataRX);
            if (comandaRX == 33 && dataRX < 1000) ListBoxStatus.Items.Insert(0, "Turn left complete. Diff: " + dataRX);

            if (comandaRX == 71 && dataRX == 5000) ListBoxStatus.Items.Insert(0, "To move > then current distance.");
            if (comandaRX == 71 && dataRX == 4000) ListBoxStatus.Items.Insert(0, "Too close, distance < 30cm.");
            if (comandaRX == 71 && dataRX == 3000) ListBoxStatus.Items.Insert(0, "Too close for maneuver.");
            if (comandaRX == 71 && dataRX == 2000) ListBoxStatus.Items.Insert(0, "Stuck! not moving.");
            if (comandaRX == 71 && dataRX < 1200) ListBoxStatus.Items.Insert(0, "Move complete. Diff: " + dataRX);

            Thread.Sleep(1);
            worker.RunWorkerAsync();   // This will make the BgWorker run again, and never runs before it is completed
            comandaRX = 0;
        }
        private void ButtonVolt_Click(object sender, RoutedEventArgs e)       //Get voltage
        {
            comandaTX = 20;
            dataTX = 20;
            Posilka();
        }
        private void ButtonCurrent_Click(object sender, RoutedEventArgs e)    //Get current
        {
            comandaTX = 19;
            dataTX = 19;
            Posilka();
        }
        private void ButtonFind_Click(object sender, RoutedEventArgs e)       //Find ip on the lan with constant hostname
        {
            try
            {
                IPAddress[] ipaddress = Dns.GetHostAddresses("orangepizero");
                hostname = ipaddress[0].ToString();
                ListBoxStatus.Items.Insert(0, "Found with IP: " + hostname);
                File.Delete(SETTINGS_FILE);
                CreateFile();
                ButtonPing_Click(null, null);
            }
            catch
            {
                ListBoxStatus.Items.Insert(0, "Robot not found on the current network.");
            }
        }
        private void DrawTracks()                                             //Draw tracks
        {
            this.Dispatcher.Invoke(() =>
            {
                TrkBmp.Clear();
                using (TrkBmp.GetBitmapContext())
                {
                    ImageTracks.Source = TrkBmp;

                    TrkBmp.DrawRectangle(0, 0, 20, 128, Colors.Crimson);          //Left rect
                    TrkBmp.DrawRectangle(60, 0, 80, 128, Colors.Crimson);         //Right rect

                    for (byte trkR = 0; trkR < 16; trkR++)                        //Display Tracks
                    {
                        trrR[trkR] = trrR[trkR] - (PwmTempR / 1024D);             //Display speed  0.05:slow  +1.2 -> -1.2: fast
                        if (trrR[trkR] <= 0) trrR[trkR] = 127;
                        if (trrR[trkR] >= 128) trrR[trkR] = 0;
                        TrkBmp.DrawLine(61, (int)trrR[trkR], 79, (int)trrR[trkR], Colors.Crimson);
                    }
                    for (byte trkL = 0; trkL < 16; trkL++)
                    {
                        trrL[trkL] = trrL[trkL] - (PwmTempL / 1024D);             //Display speed  0.05:slow  +1.2 -> -1.2: fast
                        if (trrL[trkL] <= 0) trrL[trkL] = 127;
                        if (trrL[trkL] >= 128) trrL[trkL] = 0;
                        TrkBmp.DrawLine(1, (int)trrL[trkL], 19, (int)trrL[trkL], Colors.Crimson);
                    }
                }
            });
        }
        private void ButtonSTM32ping_Click(object sender, RoutedEventArgs e)  //STM32 Ping
        {
            stmPingWatch.Reset();
            stmPingWatch.Start();
            comandaTX = 55;
            dataTX = 55;
            Posilka();
        }
        private void CheckBoxRPM_Clicked(object sender, RoutedEventArgs e)    //Show RPM
        {
            if (CheckBoxRPM.IsChecked == true)
            {
                comandaTX = 116;
                dataTX = 1;
                Posilka();
            }
            else
            {
                comandaTX = 116;
                dataTX = 0;
                Posilka();
            }
        }
        private void CheckBoxYAW_Clicked(object sender, RoutedEventArgs e)    //Show YAW
        {
            if (CheckBoxIMU.IsChecked == true)
            {
                timerIMU.Start();
                timer3D.Stop();
                CheckBox3D.IsChecked = false;
            }
            if (CheckBoxIMU.IsChecked == false)
            {
                timerIMU.Stop();
            }
        }
        private void CheckBox3D_Checked(object sender, RoutedEventArgs e)     //Show 3D
        {
            if (CheckBox3D.IsChecked == true)
            {
                timer3D.Start();
                timerIMU.Stop();
                CheckBoxIMU.IsChecked = false;
            }
            if (CheckBox3D.IsChecked == false)
            {
                timer3D.Stop();
            }
        }
        private void ButtonNorth_Click(object sender, RoutedEventArgs e)      //Set north
        {
            north = yaw;
        }
        private void TextBoxTurn_TextChanged(object sender, TextChangedEventArgs e)    //TextBox TURN
        {
            var s = TextBoxTurn.Text;
            if (s == "") return;                         //srazu net
            if (!s.All(char.IsDigit))
            {
                ListBoxStatus.Items.Insert(0, "Must be numeric.");
                TextBoxTurn.Text = "";
                return;
            }
            byte b = Convert.ToByte(TextBoxTurn.Text);   // tryParse i vot eto vse...
            if (b < 0 || b > 180)
            {
                ListBoxStatus.Items.Insert(0, "Must be in range 0-180.");
                TextBoxTurn.Text = "";
            }
        }
        private void ButtonTurnLeft_Click(object sender, RoutedEventArgs e)   //Autoturn left!
        {
            comandaTX = 33;
            dataTX = Convert.ToByte(TextBoxTurn.Text);
            Posilka();
        }
        private void ButtonTurnRight_Click(object sender, RoutedEventArgs e)   //Autoturn right!
        {
            comandaTX = 32;
            dataTX = Convert.ToByte(TextBoxTurn.Text);
            Posilka();
        }
        private void TextBoxTacho_TextChanged(object sender, TextChangedEventArgs e)   //TextBox TACHO
        {
            var s = TextBoxTacho.Text;
            if (s == "") return;                         //srazu net
            if (!s.All(char.IsDigit))
            {
                ListBoxStatus.Items.Insert(0, "Must be numeric.");
                TextBoxTacho.Text = "";
                return;
            }
            int b = Convert.ToInt16(TextBoxTacho.Text);   // tryParse i vot eto vse...
            if (b < 0 || b > 9999)
            {
                ListBoxStatus.Items.Insert(0, "Must be in range 0-9999mm.");
                TextBoxTacho.Text = "";
            }
        }
        private void ButtonTachoF_Click(object sender, RoutedEventArgs e)              //Move tacho forward
        {
            comandaTX = 61;
            dataTX = (ushort)Convert.ToInt16(TextBoxTacho.Text);
            Posilka();
        }
        private void ButtonTachoB_Click(object sender, RoutedEventArgs e)              //Move tacho backward
        {
            comandaTX = 68;
            dataTX = (ushort)Convert.ToInt16(TextBoxTacho.Text);
            Posilka();
        }
        private void TextBoxLidar_TextChanged(object sender, TextChangedEventArgs e)    //TextBox LIDAR
        {
            var s = TextBoxLidar.Text;
            if (s == "") return;                         //srazu net
            if (!s.All(char.IsDigit))
            {
                ListBoxStatus.Items.Insert(0, "Must be numeric.");
                TextBoxLidar.Text = "";
                return;
            }
            int b = Convert.ToInt16(TextBoxLidar.Text);   // tryParse i vot eto vse...
            if (b < 1 || b > 1000)
            {
                ListBoxStatus.Items.Insert(0, "Must be in range 1-1000cm.");
                TextBoxLidar.Text = "";
            }
        }
        private void ButtonLidarF_Click(object sender, RoutedEventArgs e)              //Move lidar forward
        {
            comandaTX = 71;
            dataTX = (ushort)Convert.ToInt16(TextBoxLidar.Text);
            Posilka();
        }
        private void ButtonPitch_Click(object sender, RoutedEventArgs e)               //Get pitch
        {
            comandaTX = 119;
            dataTX = 11;
            Posilka();
        }
        private void ButtonRoll_Click(object sender, RoutedEventArgs e)               //Get roll
        {
            {
                comandaTX = 119;
                dataTX = 12;
                Posilka();
            }
        }
        private void ListBoxStatus_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            //   var z = ListBoxStatus.Items;
            //   ListBoxStatus.ToolTip = z;
        }
        private void SliderLightLeft_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)   //Head lights. Left
        {
            if (SliderLightLeft != null && CheckBoxLights != null)        //...while form init.
            {
                comandaTX = 14;
                dataTX = (ushort)(SliderLightLeft.Value * 40 + 1);
                Posilka();
                if (CheckBoxLights.IsChecked == true)
                {
                    SliderLightRight.Value = SliderLightLeft.Value;
                    comandaTX = 15;
                    dataTX = (ushort)(SliderLightRight.Value * 40 + 1);
                    Posilka();
                }
                if (SliderLightLeft.Value > 60 || SliderLightRight.Value > 60)
                {
                    GroupBoxLights.Background = Brushes.Maroon;
                }
                else
                {
                    GroupBoxLights.Background = Brushes.Transparent;
                }
            }
        }
        private void SliderLightRight_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)  //Head lights. Right
        {
            if (SliderLightRight != null && CheckBoxLights != null)        //...while form init.
            {
                comandaTX = 15;
                dataTX = (ushort)(SliderLightRight.Value * 40 + 1);
                Posilka();
                if (CheckBoxLights.IsChecked == true)
                {
                    SliderLightLeft.Value = SliderLightRight.Value;
                    comandaTX = 14;
                    dataTX = (ushort)(SliderLightRight.Value * 40 + 1);
                    Posilka();
                }
                if (SliderLightRight.Value > 60 || SliderLightLeft.Value > 60)
                {
                    GroupBoxLights.Background = Brushes.Maroon;
                }
                else
                {
                    GroupBoxLights.Background = Brushes.Transparent;
                }
            }
        }
        private void ButtonHatch_Click(object sender, RoutedEventArgs e)                               //Hatch Status request     
        {
            comandaTX = 21;
            dataTX = 21;
            Posilka();
        }
        private void ButtonLidar_Click(object sender, RoutedEventArgs e)                               //Lidar extend-retract
        {
            if (lidar == false)
            {
                lidar = true;
                comandaTX = 39;
                dataTX = 0;
                Posilka();
                timerMap.Start();
            }
            else
            {
                lidar = false;
                comandaTX = 39;
                dataTX = 1;
                Posilka();
                timerMap.Stop();
            }
        }
        private void ButtonDist_Click(object sender, RoutedEventArgs e)                                //Simply get distance at current angle
        {
            if (lidar == true)
            {
                comandaTX = 50;
                dataTX = 50;
                Posilka();
                //---------------new:
                //  comandaTX = 51;
                //  dataTX = (ushort)(1023 - SliderLidar.Value);
                //  Posilka();
            }

        }
        private void SliderLidar_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e) //Lidar direction
        {
            if (lidar == true)
            {
                comandaTX = 40;
                lidarDirection = (short)(1023 - SliderLidar.Value);
                dataTX = (ushort)lidarDirection;
                Posilka();
            }
        }
        private async void ButtonScanRight_Click(object sender, RoutedEventArgs e)                          //Scan Left to Right
        {
            if (lidar == true)
            {
                lidarResolution = (byte)SliderResolution.Value;
                timerBatt.Stop();
                lidarDirection = 1023;
                comandaTX = 40;
                dataTX = (ushort)lidarDirection;
                Posilka();

                await Task.Delay(1500);

                for (lidarDirection = 1023; lidarDirection >= 0; lidarDirection -= lidarResolution)
                {
                    if (lidarDirection < 0) lidarDirection = 0;
                    if (lidarDirection > 1023) lidarDirection = 1023;
                    comandaTX = 51;
                    dataTX = (ushort)lidarDirection;
                    Posilka();
                    await Task.Delay(Math.Abs(20 + lidarResolution));
                }
                timerBatt.Start();
            }
        }
        private async void ButtonScanLeft_Click(object sender, RoutedEventArgs e)                           //Scan Right to Left
        {
            if (lidar == true)
            {
                lidarResolution = (byte)SliderResolution.Value;
                timerBatt.Stop();
                lidarDirection = 0;
                comandaTX = 40;
                dataTX = (ushort)lidarDirection;
                Posilka();

                await Task.Delay(1500);

                for (lidarDirection = 0; lidarDirection <= 1023; lidarDirection += lidarResolution)
                {
                    if (lidarDirection < 0) lidarDirection = 0;
                    if (lidarDirection > 1023) lidarDirection = 1023;
                    comandaTX = 51;
                    dataTX = (ushort)lidarDirection;
                    Posilka();
                    await Task.Delay(Math.Abs(20 + lidarResolution));
                }
                timerBatt.Start();
            }
        }
        private void ButtonClear_Click(object sender, RoutedEventArgs e)                                    //Clear mapimage to 0
        {
            for (int t = 0; t < 1023; t++)
            {
                allRange[t] = 0;
            }
        }
        private void ButtonGetPos1_Click(object sender, RoutedEventArgs e)                                  //XL320 Req position 1
        {
            comandaTX = 45;
            dataTX = 1;
            Posilka();
        }
        private void ButtonGetPos2_Click(object sender, RoutedEventArgs e)                                  //XL320 Req position 2
        {
            comandaTX = 45;
            dataTX = 2;
            Posilka();
        }
        private void ButtonGetPos3_Click(object sender, RoutedEventArgs e)                                  //XL320 Req position 3
        {
            comandaTX = 45;
            dataTX = 3;
            Posilka();
        }
        private void ButtonGetPos4_Click(object sender, RoutedEventArgs e)                                  //XL320 Req position 4
        {
            comandaTX = 45;
            dataTX = 4;
            Posilka();
        }
        private void ButtonGetPos5_Click(object sender, RoutedEventArgs e)                                  //XL320 Req position 5 - lidar
        {
            comandaTX = 45;
            dataTX = 5;
            Posilka();
        }
        private void ServoRead()                                                                            //XL320 Processing
        {
            var gp = dataRX >> 11;
            if (gp > 0 && gp < 6)                   // ok, 1-5
            {
                xl320[gp] = dataRX ^ (gp << 11);

                for (byte m = 1; m < 6; m++)
                {
                    var h = (Button)FindName("ButtonGetPos" + m);
                    h.Content = xl320[m];
                    if (xl320[m] == 2000) h.Content = "Absent";
                    if (xl320[m] == 2001) h.Content = "crc error";
                }
                //  this.Dispatcher.Invoke(() =>  //Mozhet prigoditsa
                //   {
                //   });
            }
        }
        private Model3D Display3d(string model2load)
        {

            try
            {
                viewPort3d.RotateGesture = new MouseGesture(MouseAction.LeftClick);
                ModelImporter import = new ModelImporter();
                model = import.Load(model2load);
                GeometryModel3D myModel = (GeometryModel3D)model.Children[0];
                myModel.Material = Materials.Gold;
                matrix = Robot3D.Transform.Value;
            }
            catch (Exception e)
            {
                MessageBox.Show("Exception Error : " + e.StackTrace);                // Handle exception in case can not find the 3D model file or some
            }
            return model;
        }
    }
}