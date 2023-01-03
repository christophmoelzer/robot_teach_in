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
using System.Windows.Threading;
using System.Threading;
using System.IO.Ports;
using System.Diagnostics;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.Controllers.Messaging;

namespace WPF_APP_TUTORIAL
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 
    public partial class SdkCommands
    {
        public byte[] linear_motion_commands = new byte[32];
        public byte[] angular_motion_commands = new byte[32];
        public string[] motion_commands = new string[20];
        public bool[] motion_commands_bool = new bool[20];
        public bool move_linear;
        public bool move_angular;
        public bool leadthrough;
        public SdkCommands()
        {
            motion_commands[0] = "sdk_x_n";
            motion_commands[1] = "sdk_x_p";
            motion_commands[2] = "sdk_y_n";
            motion_commands[3] = "sdk_y_p";
            motion_commands[4] = "sdk_z_n";
            motion_commands[5] = "sdk_z_p";
            motion_commands[6] = "sdk_rx_n";
            motion_commands[7] = "sdk_rx_p";
            motion_commands[8] = "sdk_ry_n";
            motion_commands[9] = "sdk_ry_p";
            motion_commands[10] = "sdk_rz_n";
            motion_commands[11] = "sdk_rz_p";
            motion_commands[12] = "sdk_lin_mode";
            motion_commands[13] = "sdk_rot_mode";
            motion_commands[14] = "sdk_leadthrough_on";
            //motion_commands[14] = "sdk_better_communictation";
            motion_commands[15] = "";
            motion_commands[16] = "";
            motion_commands[17] = "";
            motion_commands[18] = "";
            motion_commands[19] = "";

            linear_motion_commands[0] = 0;
            linear_motion_commands[1] = 1;
            linear_motion_commands[2] = 2;
            linear_motion_commands[3] = 3;
            linear_motion_commands[4] = 5;
            linear_motion_commands[5] = 4;
            linear_motion_commands[6] = 0;
            linear_motion_commands[7] = 1;
            linear_motion_commands[8] = 2;
            linear_motion_commands[9] = 3;
            linear_motion_commands[10] = 5;
            linear_motion_commands[11] = 4;
            linear_motion_commands[12] = 0;
            linear_motion_commands[13] = 1;
            linear_motion_commands[14] = 2;
            linear_motion_commands[15] = 3;
            linear_motion_commands[16] = 5;
            linear_motion_commands[17] = 4;
            linear_motion_commands[18] = 0;
            linear_motion_commands[19] = 1;
            linear_motion_commands[20] = 2;
            linear_motion_commands[21] = 3;
            linear_motion_commands[22] = 5;
            linear_motion_commands[23] = 4;
            linear_motion_commands[24] = 12;
            linear_motion_commands[25] = 13;
            linear_motion_commands[26] = 99;
            linear_motion_commands[27] = 99;
            linear_motion_commands[28] = 99;
            linear_motion_commands[29] = 14;
            linear_motion_commands[30] = 99;
            linear_motion_commands[31] = 99;

            angular_motion_commands[0] = 11;
            angular_motion_commands[1] = 10;
            angular_motion_commands[2] = 7;
            angular_motion_commands[3] = 6;
            angular_motion_commands[4] = 6;
            angular_motion_commands[5] = 7;
            angular_motion_commands[6] = 8;
            angular_motion_commands[7] = 9;
            angular_motion_commands[8] = 11;
            angular_motion_commands[9] = 10;
            angular_motion_commands[10] = 8;
            angular_motion_commands[11] = 9;
            angular_motion_commands[12] = 8;
            angular_motion_commands[13] = 9;
            angular_motion_commands[14] = 10;
            angular_motion_commands[15] = 11;
            angular_motion_commands[16] = 9;
            angular_motion_commands[17] = 8;
            angular_motion_commands[18] = 10;
            angular_motion_commands[19] = 11;
            angular_motion_commands[20] = 7;
            angular_motion_commands[21] = 6;
            angular_motion_commands[22] = 7;
            angular_motion_commands[23] = 6;
            angular_motion_commands[24] = 12;
            angular_motion_commands[25] = 13;
            angular_motion_commands[26] = 99;
            angular_motion_commands[27] = 99;
            angular_motion_commands[28] = 99;
            angular_motion_commands[29] = 14;
            angular_motion_commands[30] = 99;
            angular_motion_commands[31] = 99;
        }
    }
    public partial class MainWindow : Window
    {
        public delegate void NextPrimeDelegate();
        private bool run_dispatcher = false;

        // #############################################################
        // Serial communication
        // #############################################################
        SerialPort port;
        static byte received_wd_counter = new byte();
        byte prev_wd_counter = new byte();
        int wd_fail_counter = 0;
        Stopwatch watchdog_timer = new Stopwatch();
        bool EStop_motion = false;
        static bool[] buttons = new bool[32];
        static byte[] robot_state = new byte[1];

        // #############################################################
        // Robot communication
        // #############################################################
        private NetworkScanner scanner = null;
        private Controller controller = null;
        private ABB.Robotics.Controllers.RapidDomain.Task[] tasks = null;
        private NetworkWatcher networkWatcher = null;
        private static Mastership mastership;
        private List<string> ip_list = new List<string>();
        private static SdkCommands sdk_commands = new SdkCommands();

        // IPC
        private IpcQueue tRob1Queue;
        private IpcQueue myQueue;
        private IpcMessage sendMessage;
        private IpcMessage recvMessage;

        // #############################################################
        // DEBUG
        // #############################################################
        private static Stopwatch stopwatch = new Stopwatch();
        private bool motion_button_pressed = false;

        // #############################################################
        // STYLE
        // #############################################################
        private string btn_text = "default";

        public MainWindow()
        {
            InitializeComponent();
            tb_value_sdk_state.Background = Brushes.DarkRed;
            open_com_port(tb_comport.Text);
            connect_robot_controller();
            //RMQReceiveRecord();
            //ipc_function();
        }

        

        private void open_com_port(string com_port)
        {
            if (port != null) 
            {
                if (port.IsOpen)
                {
                    port.Close();
                    port = null;
                }
            }
            if (port == null)
            {
                port = new SerialPort(com_port, 115200);
                try
                {
                    port.Open();
                    port.DataReceived += new SerialDataReceivedEventHandler(DataReceived);
                    label5.Content= com_port + " connect";
                    tb_value_comport_state.Background = Brushes.DarkGreen;

                }
                catch
                {
                    label5.Content  = com_port + " not available";
                    tb_value_comport_state.Background = Brushes.DarkRed;
                    port = null;
                }
            }
            
        }

        private void connect_robot_controller()
        {
            scanner = new NetworkScanner();
            scanner.Scan();
            ControllerInfoCollection controllers = scanner.Controllers;
            ControllerInfo? controllerInfo1 = null;
            foreach(ControllerInfo controllerInfo in controllers)
            {
                //MessageBox.Show(controllerInfo.IPAddress.ToString());
                ip_list.Add(controllerInfo.IPAddress.ToString());
                controllerInfo1 = controllerInfo;
            }
            if(controllerInfo1 != null)
            {
                if(controllerInfo1.Availability == Availability.Available)
                {
                    if(controller != null)
                    {
                        controller.Logoff();
                        controller.Dispose();
                        controller = null;
                    }
                    controller = Controller.Connect(controllerInfo1, ConnectionType.Standalone, false);
                    UserInfo userInfo = new UserInfo("admin", "robotics");
                    controller.Logon(userInfo);
                    label3.Content = "connected";
                    tb_value_robot_state.Background = Brushes.DarkGreen;
                    label9.Content = controller.Rapid.ExecutionStatus.ToString();
                    if(controller.Rapid.ExecutionStatus == ExecutionStatus.Running)
                    {
                        tb_value_rapid_state.Background = Brushes.DarkGreen;
                    }
                    else
                    {
                        tb_value_rapid_state.Background = Brushes.DarkRed;
                    }
                }
                else
                {
                    label3.Content="selected controller not available.";
                    tb_value_robot_state.Background = Brushes.DarkRed;
                }
            }
            else
            {
                label3.Content = "no controller available";
                tb_value_robot_state.Background = Brushes.DarkRed;

            }

        }

        private void ipc_function()
        {
            tRob1Queue = controller.Ipc.GetQueue("RMQ_T_ROB1");

            if (!controller.Ipc.Exists("RAB_Q"))
            {
                //MessageBox.Show(controller.Ipc.GetQueueId("PC_SDK_Q").ToString());
                if (controller.Ipc.GetQueueId("PC_SDK_Q") >= 0)
                {
                    controller.Ipc.DeleteQueue(controller.Ipc.GetQueueId("PC_SDK_Q"));
                }
                myQueue = controller.Ipc.CreateQueue("PC_SDK_Q", 5, Ipc.MaxMessageSize);
                //MessageBox.Show(Ipc.MaxMessageSize.ToString());
                myQueue = controller.Ipc.GetQueue("PC_SDK_Q");

            }
            
            

            //SendMessage(true);
            //CheckReturnMsg();

        }

        public void SendMessage(bool boolMsg)
        {
            sendMessage = new IpcMessage();
            System.Byte[] data = null;
            
            if (boolMsg)
            {
                data = new UTF8Encoding().GetBytes("bool;True");
            }
            else
            {
                data = new UTF8Encoding().GetBytes("bool;False");
            }
            //MessageBox.Show(data.GetLength(0).ToString());
            sendMessage.SetData(data);
            sendMessage.Sender = myQueue.QueueId;
            tRob1Queue.Send(sendMessage);
            
        }

        private void CheckReturnMsg()
        {
            recvMessage = new IpcMessage();
            
            MessageBox.Show(recvMessage.Capacity.ToString());
            IpcReturnType ret = IpcReturnType.Timeout;
            string answer = "Bck";
            int timeout = 5000;
            
            ret = myQueue.Receive(timeout, recvMessage);
            MessageBox.Show("wait for answer...");
            if (ret == IpcReturnType.OK)
            {
                answer = new UTF8Encoding().GetString(recvMessage.Data);
                MessageBox.Show(answer);
            }
            else
            {
                MessageBox.Show("Timeout!");
            }
            MessageBox.Show("end...");
        }

        public void RMQReceiveRecord()
        {
            const string destination_slot = "RMQ_OtherTask";
            IpcQueue queue = controller.Ipc.CreateQueue(destination_slot, 16, Ipc.MaxMessageSize);
            while (true)
            {
                IpcMessage message = new IpcMessage();
                IpcReturnType retValue = IpcReturnType.Timeout;

                retValue = queue.Receive(1000, message);
                if(IpcReturnType.OK == retValue)
                {
                    Int32 x = (message.Data[0] << 16) | (message.Data[1] << 8) | (message.Data[2]);
                    Int32 y = (message.Data[3] << 16) | (message.Data[4] << 8) | (message.Data[5]);
                    MessageBox.Show(x.ToString());
                    MessageBox.Show(y.ToString());
                }
            }
            if (controller.Ipc.Exists(destination_slot))
            {
                controller.Ipc.DeleteQueue(controller.Ipc.GetQueueId(destination_slot));
            }
        }

        private void stop_motion()
        {
            for(int i=0; i<sdk_commands.motion_commands.Length; i++)
            {
                reset_rapid_variable(sdk_commands.motion_commands[i]);
            }
        }

        private void watchdog_clicked(object sender, EventArgs e)
        {
            if (run_dispatcher)
            {
                try
                {
                    mastership.Release();
                }
                catch (Exception ex)
                {
                    ;// MessageBox.Show(ex.ToString());
                }
                btn_watchdog.Background = Brushes.WhiteSmoke;
                btn_watchdog.Content = btn_text = "DISABLED";
                label7.Content = "disabled";
                tb_value_sdk_state.Background = Brushes.DarkRed;
                run_dispatcher = false;
            }
            else
            {
                try
                {
                    mastership = Mastership.Request(controller.Rapid);
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message);
                }
                btn_watchdog.Background = Brushes.LightGreen;
                btn_watchdog.Content = btn_text = "ENABLED";
                label7.Content = "enabled";
                tb_value_sdk_state.Background = Brushes.DarkGreen;

                run_dispatcher = true;
                btn_watchdog.Dispatcher.BeginInvoke(
                    DispatcherPriority.Normal,
                    new NextPrimeDelegate(watchdog));
            }
        }

        public void watchdog()
        {
            // Debug
            if (sdk_commands.motion_commands_bool[14])
            {
                tb_wd_counter.Text = "ON";
            }
            else
            {
                tb_wd_counter.Text = "OFF";
            }
            if (sdk_commands.move_linear)
            {
                tb_linear.Text = "ON";
            }
            else
            {
                tb_linear.Text = "OFF";
            }
            if (sdk_commands.move_angular)
            {
                tb_angular.Text = "ON";
            }
            else
            {
                tb_angular.Text = "OFF";
            }

            if (get_rapid_data("sdk_is_moving"))
            {
                robot_state[0] = 1;
                tb_moving.Text = "MOVING";
            }
            else
            {
                robot_state[0] = 2;
                tb_moving.Text = "STOPPED";
            }

            get_rapid_pose_data("sdk_pose");
            


            /*if (motion_button_pressed == false && get_rapid_data("sdk_is_moving"))
            {
                stopwatch.Reset();
                stopwatch.Start();
            }
            else if(motion_button_pressed == false && get_rapid_data("sdk_is_moving") == false)
            {
                stopwatch.Stop();
            }
            tb_timer.Text = "btn: " + motion_button_pressed.ToString() + " moving: " + get_rapid_data("sdk_is_moving").ToString() +" time:"+ stopwatch.ElapsedMilliseconds.ToString();
            */
            if (sdk_commands.motion_commands_bool[14] == true && buttons[29] == false)
            {
                stopwatch.Reset();
                stopwatch.Start();
            }
            else if (sdk_commands.motion_commands_bool[14] == false && buttons[29] == false)
            {
                stopwatch.Stop();
            }
            tb_timer.Text = "cmd: " + sdk_commands.motion_commands_bool[14].ToString() + " btn: " + buttons[29].ToString() + " time:" + stopwatch.ElapsedMilliseconds.ToString() + "wd:"+ received_wd_counter.ToString();

            // Watchdog Teachtool
            if (received_wd_counter == prev_wd_counter)
            {
                
                watchdog_timer.Start();
            }
            else
            {
                watchdog_timer.Stop();
                watchdog_timer.Reset();

            }
            if (watchdog_timer.ElapsedMilliseconds > 500)
            {
                EStop_motion = true; 
            }
            prev_wd_counter = received_wd_counter;


            // Watchdog Rapid
            set_rapid_byte("sdk_wd_counter", received_wd_counter);
            //tb_timer.Text =received_wd_counter.ToString();

            // Initialize commands
            for(int i = 0; i < sdk_commands.motion_commands_bool.Length; i++)
            {
                sdk_commands.motion_commands_bool[i] = false;
            }

            // Set commands
            motion_button_pressed = false;
            for(int button_index=0; button_index<buttons.Length; button_index++)
            {
                if (buttons[button_index])
                {
                    if (sdk_commands.linear_motion_commands[button_index] != 99 & sdk_commands.move_linear)
                    {
                        sdk_commands.motion_commands_bool[sdk_commands.linear_motion_commands[button_index]] = true;
                        if(button_index<24) motion_button_pressed = true;
                    }
                    if (sdk_commands.angular_motion_commands[button_index] != 99 & sdk_commands.move_angular)
                    {
                        sdk_commands.motion_commands_bool[sdk_commands.angular_motion_commands[button_index]] = true;
                        if (button_index < 24) motion_button_pressed = true;
                    }
                }
            }

            // Send commands
            tb_commands.Text = "";
            for(int i=0; i<sdk_commands.motion_commands_bool.Length; i++)
            {
                if (sdk_commands.motion_commands_bool[i] & EStop_motion==false) { 
                    set_rapid_variable(sdk_commands.motion_commands[i]);
                    tb_commands.Text += "1 ";
                }
                    else
                {
                    reset_rapid_variable(sdk_commands.motion_commands[i]);
                    tb_commands.Text += "0 ";

                }
            }
            
            


            if (run_dispatcher)
            {
                btn_watchdog.Dispatcher.BeginInvoke(
                    System.Windows.Threading.DispatcherPriority.SystemIdle,
                    new NextPrimeDelegate(this.watchdog));
                
            }
            
        }

        private bool get_rapid_data(string key_name)
        {
            if (key_name != "")
            {
                try
                {
                    Bool rapidBool;
                    RapidData rdBool = controller.Rapid.GetRapidData("T_ROB1", "module_mr19m010", key_name);
                    if (rdBool.Value is Bool)
                    {
                        rapidBool = (Bool)rdBool.Value;
                        bool boolValue = rapidBool.Value;
                        return boolValue;
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show(key_name + " --> " + ex.Message);
                    
                }
            }
            return false;
        }

        private void get_rapid_pose_data(string key_name)
        {
            if (key_name != "")
            {
                try
                {
                    Pose rapidPose;
                    RapidData rdPose = controller.Rapid.GetRapidData("T_ROB1", "module_mr19m010", key_name);
                    if (rdPose.Value is Pose)
                    {
                        rapidPose = (Pose)rdPose.Value;
                        tb_x.Text = "X: ";
                        tb_x.Text += Math.Round(rapidPose.Trans.X, 1).ToString();
                        tb_x.Text += " Y: ";
                        tb_x.Text += Math.Round(rapidPose.Trans.Y, 1).ToString();
                        tb_x.Text += " Z: ";
                        tb_x.Text += Math.Round(rapidPose.Trans.Z, 1).ToString();
                        if (rapidPose.Trans.X < 0)
                        {
                            pgb_x_r.Value = 0;
                            pgb_x_l.Value = -100* rapidPose.Trans.X / 10;
                            
                        }
                        else
                        {
                            pgb_x_l.Value = 0;
                            pgb_x_r.Value = 100 * rapidPose.Trans.X / 10;

                        }


                        if (rapidPose.Trans.Y < 0)
                        {
                            pgb_y_r.Value = 0;
                            pgb_y_l.Value = -100 * rapidPose.Trans.Y / 10;

                        }
                        else
                        {
                            pgb_y_l.Value = 0;
                            pgb_y_r.Value = 100 * rapidPose.Trans.Y / 10;

                        }

                        if (rapidPose.Trans.Z < 0)
                        {
                            //pgb_z_r.Value = 0;
                            pgb_z_l.Value = -100-100 * rapidPose.Trans.Z / 10;

                        }
                        else
                        {
                            //pgb_z_l.Value = 0;
                            pgb_z_r.Value = 100-(100 * rapidPose.Trans.Z / 10);

                        }


                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show(key_name + " --> " + ex.Message);

                }
            }
        }

        private void set_rapid_byte(string key_name, byte number=0)
        {
           
            if (key_name != "")
            {
                try
                {
                    ABB.Robotics.Controllers.RapidDomain.Num rapidNum;
                    RapidData rdNum = controller.Rapid.GetRapidData("T_COMM", "CommModule", key_name);
                    //RapidDataType sdk_rapid_data_type = controller.Rapid.GetRapidDataType("T_COMM", "CommModule", key_name);
                    //MessageBox.Show(sdk_rapid_data_type.ToString());
                    if (rdNum.Value is ABB.Robotics.Controllers.RapidDomain.Num)
                    {
                        rapidNum = (ABB.Robotics.Controllers.RapidDomain.Num)rdNum.Value;
                        //byte numberValue = (byte)rapidNum.Value;
                        //rapidNum.Value = number;
                        rapidNum.FillFromString2(number.ToString());
                        rdNum.Value = rapidNum;
                        
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show(key_name + " --> " + ex.Message);
                }
            }
        }

        private void set_rapid_variable(string key_name)
        {
            if (key_name != "")
            {
                try
                {
                    Bool rapidBool;
                    if (key_name=="sdk_better_communictation")
                    {
                        RapidData rdBool = controller.Rapid.GetRapidData("T_COMM", "CommModule", key_name);
                        if (rdBool.Value is Bool)
                        {
                            rapidBool = (Bool)rdBool.Value;
                            bool boolValue = rapidBool.Value;
                            rapidBool.Value = true;
                            rdBool.Value = rapidBool;

                        }
                    }
                    else
                    {
                        RapidData rdBool = controller.Rapid.GetRapidData("T_ROB1", "module_mr19m010", key_name);
                        if (rdBool.Value is Bool)
                        {
                            rapidBool = (Bool)rdBool.Value;
                            bool boolValue = rapidBool.Value;
                            rapidBool.Value = true;
                            rdBool.Value = rapidBool;

                        }
                    }
                    
                }
                catch (Exception ex)
                {
                    MessageBox.Show(key_name + " --> " + ex.Message);
                }
            }
            
        }

        private void reset_rapid_variable(string key_name)
        {
            if (key_name != "")
            {
                try
                {
                    Bool rapidBool;
                    if (key_name == "sdk_better_communictation")
                    {
                        RapidData rdBool = controller.Rapid.GetRapidData("T_COMM", "CommModule", key_name);
                        if (rdBool.Value is Bool)
                        {
                            rapidBool = (Bool)rdBool.Value;
                            bool boolValue = rapidBool.Value;
                            rapidBool.Value = false;
                            rdBool.Value = rapidBool;

                        }
                    }
                    else
                    {
                        RapidData rdBool = controller.Rapid.GetRapidData("T_ROB1", "module_mr19m010", key_name);
                        if (rdBool.Value is Bool)
                        {
                            rapidBool = (Bool)rdBool.Value;
                            bool boolValue = rapidBool.Value;
                            rapidBool.Value = false;
                            rdBool.Value = rapidBool;

                        }
                    }

                }
                catch (Exception ex)
                {
                    MessageBox.Show(key_name + " --> " + ex.Message);
                }
            }
            
        }

        private void MainWindow_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            try
            {
                if (port != null && port.IsOpen)
                {

                    port.DataReceived -= new SerialDataReceivedEventHandler(DataReceived);
                    port.Close();
                }
            }
            catch (Exception ex)
            {
                ;// MessageBox.Show(ex.ToString());
            }
            try {
                mastership.Release();
            }
            catch(Exception ex)
            {
                ;// MessageBox.Show(ex.ToString());
            }
        }

        private void CloseCommandHandler(object sender, ExecutedRoutedEventArgs e)
        {
            this.Close();
        }

        private static void DataReceived(Object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort serialPort = (SerialPort)sender;
            serialPort.ReadTimeout = 50;
            byte[] data = new byte[4];
            byte stop_byte = new byte();

            try
            {
                received_wd_counter = (byte)serialPort.ReadByte();
                for(int i=0; i<4; i++)
                {
                    data[i] = (byte)serialPort.ReadByte();
                }
                stop_byte = (byte)serialPort.ReadByte();

            }
            catch (TimeoutException)
            {
                ;
            }
            serialPort.DiscardInBuffer();

            // Send data to Arduino
            serialPort.Write(robot_state,0,1);
            serialPort.Write(data, 0, 4);

            for(int data_index=0; data_index<data.Length; data_index++)
            {
                for(int bit_index=0; bit_index<8; bit_index++)
                {
                    buttons[data_index*8+bit_index] = (data[data_index] & (1 << bit_index)) != 0;
                }
            }
            
            sdk_commands.move_linear = buttons[24];
            sdk_commands.move_angular = buttons[25];
            


        }

        private void dragWindow(object sender, MouseButtonEventArgs e)
        {
            DragMove();
        }


        private void btn_comport_Click(object sender, RoutedEventArgs e)
        {

        }

        private void btn_connect_comport_Click(object sender, RoutedEventArgs e)
        {
            open_com_port(tb_comport.Text);
        }

        private void btn_3_Click(object sender, RoutedEventArgs e)
        {
            //SendMessage(true);
            CheckReturnMsg();
        }

        private void btn_4_Click(object sender, RoutedEventArgs e)
        {
            //SendMessage(false);
            CheckReturnMsg();
        }
    }
}
