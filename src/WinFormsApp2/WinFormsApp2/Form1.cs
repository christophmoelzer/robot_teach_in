


using System.IO.Ports;

using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.Controllers.IOSystemDomain;
using System.Timers;
using ABB.Robotics.Controllers;
using System.Diagnostics;

namespace WinFormsApp2
{
    public partial class Form1 : Form
    {
        SerialPort port;
        //Controller aController;
        //Signal signal1 = aController.IOSystem.GetSignal("signal_name");
        //DigitalSignal diSig = (DigitalSIgnal)signal1;

        private NetworkScanner scanner = null;
        private static Controller controller = null;
        private ABB.Robotics.Controllers.RapidDomain.Task[] tasks = null;
        private NetworkWatcher networkwatcher = null;

        RapidData data1;
        ABB.Robotics.Controllers.RapidDomain.Num _num1;
        ABB.Robotics.Controllers.RapidDomain.RobTarget position;
        double num1;

        double serial_data = 0;

        private static Stopwatch stopwatch = new Stopwatch();
        private static Stopwatch lifesign_timer = new Stopwatch();
        private static Mastership mastership;

        static int myNum;
        static bool aux_leadthrough = false;
        static byte lifesign = 0;
        static byte aux_lifesign = 0;
        static bool stop_motion = true;


        public Form1()
        {


            InitializeComponent();
            
            this.FormClosed += new FormClosedEventHandler(Form1_FormClosed);
            

            this.scanner = new NetworkScanner();
            this.scanner.Scan();
            ControllerInfoCollection controllers = scanner.Controllers;
            ListViewItem item = null;
            foreach(ControllerInfo controllerInfo in controllers)
            {
                item = new ListViewItem(controllerInfo.IPAddress.ToString());
                item.SubItems.Add(controllerInfo.Id);
                item.SubItems.Add(controllerInfo.Availability.ToString());
                item.SubItems.Add(controllerInfo.IsVirtual.ToString());
                item.SubItems.Add(controllerInfo.SystemName);
                item.SubItems.Add(controllerInfo.Version.ToString());
                item.SubItems.Add(controllerInfo.ControllerName);
                this.listView1.Items.Add(item);
                item.Tag = controllerInfo;

            }

            if (item.Tag != null)
            {
                ControllerInfo controllerInfo = (ControllerInfo)item.Tag;
                if (controllerInfo.Availability == Availability.Available)
                {
                    if (controller != null)
                    {
                        controller.Logoff();
                        controller.Dispose();
                        controller = null;
                    }
                    controller = Controller.Connect(controllerInfo, ConnectionType.Standalone, false);
                    UserInfo userInfo = new UserInfo("admin", "robotics");
                    controller.Logon(userInfo);
                    try
                    {
                        mastership = Mastership.Request(controller.Rapid);
                    }
                    catch
                    {
                        if(mastership != null)
                        {
                            MessageBox.Show(mastership.IsMaster.ToString());
                        }
                        else
                        {
                            MessageBox.Show("ERROR");
                        }
                        

                    }
                    //MessageBox.Show(controller.Rapid.GetRapidData("T_ROB1","module_mr19m010","sdk_x_n").Value.ToString());

                }
                else
                {
                    MessageBox.Show("Selected controller not available.");
                }
            }

            if (port == null)
            {
                string com_port = "COM4";
                port = new SerialPort(com_port, 115200);
                try
                {
                    port.Open();
                    port.DataReceived += new SerialDataReceivedEventHandler(DataReceived);
                    ;// MessageBox.Show("Bediengerät ("+com_port+") verbunden.");
                }
                catch
                {
                    MessageBox.Show(com_port + " not available.");
                }
            }


         

        }

        void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
           mastership.Release();
            if(port != null && port.IsOpen)
            {

                port.DataReceived -= new SerialDataReceivedEventHandler(DataReceived);
                port.Close();
            }

        }

        private void button1_Click(object sender, EventArgs e)
        {
            PortWrite("1");
        }

        private void button2_Click(object sender, EventArgs e)
        {
            PortWrite("0");
        }
        private void PortWrite(string message)
        {
            if ( port != null && port.IsOpen)
            {
                port.Write(message);
            }
        }

        private static void set_lable_text(string text)
        {
            label1.Text = text;
        }

        private static void set_rapid_variable(string key_name)
        {
            ;// MessageBox.Show(controller.Rapid.GetTasks()[0].GetModules()[0].ToString());
            
            try
            {
                Bool rapidBool;               
                RapidData rdBool = controller.Rapid.GetRapidData("T_ROB1", "module_mr19m010", key_name);
                if (rdBool.Value is Bool)
                {
                    rapidBool = (Bool)rdBool.Value;

                    bool boolValue = rapidBool.Value;

                    rapidBool.Value = true;
                    
                    
                    
                    //using (Mastership.Request(controller.Rapid))
                    //{
                        rdBool.Value = rapidBool;
                    //}

                    
                }
            }
            catch(Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
            


        }

        private static void reset_rapid_variable(string key_name)
        {
            try
            {
                Bool rapidBool;
                RapidData rdBool = controller.Rapid.GetRapidData("T_ROB1", "module_mr19m010", key_name);
                if (rdBool.Value is Bool)
                {
                    rapidBool = (Bool)rdBool.Value;

                    bool boolValue = rapidBool.Value;

                    rapidBool.Value = false;

                    //using (Mastership.Request(controller.Rapid))
                    //{
                        rdBool.Value = rapidBool;
                    //}
                }

            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        private static void show_angle(UInt32 value0, UInt32 value1, UInt32 value2, 
            UInt32 value3, UInt32 value4, UInt32 value5)
        {
            label2.Text = value0.ToString() + " " + value1.ToString()
                + " " + value2.ToString() + " " + value3.ToString() 
                + " " + value4.ToString() + " " + value5.ToString();
        }

        private static void DataReceived(Object sender, SerialDataReceivedEventArgs e)
        {
            stopwatch.Start();
            SerialPort sp = (SerialPort)sender;
            sp.ReadTimeout = 50;           

            myNum +=1;

            

            byte[] data = new byte[4];
            byte[] angle_data = new byte[4];
            byte start_byte = new byte();
            byte stop_byte = new byte();

            try
            {
                start_byte = (byte)sp.ReadByte();
                data[0] = (byte)sp.ReadByte();
                data[1] = (byte)sp.ReadByte();
                data[2] = (byte)sp.ReadByte();
                data[3] = (byte)sp.ReadByte();
                stop_byte = (byte)sp.ReadByte();
            }
            catch (TimeoutException)
            {
                ;
            }

            stopwatch.Stop();

            lifesign = start_byte;
            if (lifesign == aux_lifesign)
            {
                lifesign_timer.Start();
            }
            else
            {
                lifesign_timer.Stop();
                lifesign_timer.Reset();
                stop_motion = false;
            }
            aux_lifesign = lifesign;

            if (lifesign_timer.ElapsedMilliseconds > 500)
            {
                stop_motion = true;
                MessageBox.Show("LIFESIGN ERROR");
            }



            //show_angle(start_byte, data[0], data[1], data[2], data[3], stop_byte);


            //angle_data[0] = (byte)sp.ReadByte();
            //angle_data[1] = (byte)sp.ReadByte();
            //angle_data[2] = (byte)sp.ReadByte();
            //angle_data[3] = (byte)sp.ReadByte();

            UInt32 angle = 0;
            angle += angle_data[0];
            angle += (UInt32)(angle_data[1] << 4);
            angle += (UInt32)(angle_data[2] << 8);
            angle += (UInt32)(angle_data[3] << 12);


            bool[] buttons = new bool[32];
            for(int i=0; i < 32; i++) { buttons[i] = false; }

            string[] sdk_commands = new string[32];
            sdk_commands[0] = "sdk_x_n";
            sdk_commands[1] = "sdk_x_p";
            sdk_commands[2] = "sdk_y_n";
            sdk_commands[3] = "sdk_y_p";
            sdk_commands[4] = "sdk_z_p";
            sdk_commands[5] = "sdk_z_n";
            sdk_commands[6] = "sdk_x_n";
            sdk_commands[7] = "sdk_x_p";
            sdk_commands[8] = "sdk_y_n";
            sdk_commands[9] = "sdk_y_p";
            sdk_commands[10] = "sdk_z_p";
            sdk_commands[11] = "sdk_z_n";
            sdk_commands[12] = "sdk_x_n";
            sdk_commands[13] = "sdk_x_p";
            sdk_commands[14] = "sdk_y_n";
            sdk_commands[15] = "sdk_y_p";
            sdk_commands[16] = "sdk_z_p";
            sdk_commands[17] = "sdk_z_n";
            sdk_commands[18] = "sdk_x_n";
            sdk_commands[19] = "sdk_x_p";
            sdk_commands[20] = "sdk_y_n";
            sdk_commands[21] = "sdk_y_p";
            sdk_commands[22] = "sdk_z_p";
            sdk_commands[23] = "sdk_z_n";
            sdk_commands[24] = "sdk_mode_lin";
            sdk_commands[25] = "sdk_mode_lin!";
            sdk_commands[26] = "n.d.";
            sdk_commands[27] = "n.d.";
            sdk_commands[28] = "n.d.";
            sdk_commands[29] = "sdk_leadthrough";
            sdk_commands[30] = "n.d.";
            sdk_commands[31] = "n.d.";

            string[] sdk_commands_rot = new string[32];
            sdk_commands_rot[0] = "sdk_rz_p";
            sdk_commands_rot[1] = "sdk_rz_n";
            sdk_commands_rot[2] = "sdk_rx_p";
            sdk_commands_rot[3] = "sdk_rx_n";
            sdk_commands_rot[4] = "sdk_rx_n";
            sdk_commands_rot[5] = "sdk_rx_p";

            sdk_commands_rot[6] = "sdk_ry_n";
            sdk_commands_rot[7] = "sdk_ry_p";
            sdk_commands_rot[8] = "sdk_rz_p";
            sdk_commands_rot[9] = "sdk_rz_n";
            sdk_commands_rot[10] = "sdk_ry_n";
            sdk_commands_rot[11] = "sdk_ry_p";

            sdk_commands_rot[12] = "sdk_ry_n";
            sdk_commands_rot[13] = "sdk_ry_p";
            sdk_commands_rot[14] = "sdk_rz_n";
            sdk_commands_rot[15] = "sdk_rz_p";
            sdk_commands_rot[16] = "sdk_ry_p";
            sdk_commands_rot[17] = "sdk_ry_n";

            sdk_commands_rot[18] = "sdk_rz_n";
            sdk_commands_rot[19] = "sdk_rz_p";
            sdk_commands_rot[20] = "sdk_rx_p";
            sdk_commands_rot[21] = "sdk_rx_n";
            sdk_commands_rot[22] = "sdk_rx_p";
            sdk_commands_rot[23] = "sdk_rx_n";

            sdk_commands_rot[24] = "sdk_mode_lin";
            sdk_commands_rot[25] = "sdk_mode_lin!";
            sdk_commands_rot[26] = "n.d.";
            sdk_commands_rot[27] = "n.d.";
            sdk_commands_rot[28] = "n.d.";
            sdk_commands_rot[29] = "sdk_leadthrough";
            sdk_commands_rot[30] = "n.d.";
            sdk_commands_rot[31] = "n.d.";

            byte aux = 0;
            string myString = "";
            byte[] send_data = new byte[2];

            for (int x=0; x<4; x++)
            {
                aux = data[x];
                
                
                
                for (int i = 0; i < 8; i++)
                {
                    //MessageBox.Show((aux % 2).ToString());
                    myString += ((aux % 2).ToString());

                    if (((aux % 2) == 1))
                    {                            
                        buttons[i + x * 8]=true;
                    }
                    else
                    {
                        buttons[i + x * 8] = false;
                    }
                    if ((i+x*8)==29)
                    {
                        ;// MessageBox.Show(buttons[i+x*8].ToString());

                    }


                    aux = (byte)(aux / 2);

                }
            }
            //MessageBox.Show(myString);

            bool foo = false;
            //MessageBox.Show(buttons[24].ToString());
            if (buttons[24])
            {
                for (int i = 0; i < 6; i++)
                {
                    if ((buttons[i] || buttons[i + 6] || buttons[i + 2 * 6] || buttons[i + 3 * 6]) && stop_motion == false)
                    {
                        set_rapid_variable(sdk_commands[i]);
                        //set_lable_text(sdk_commands[i]);
                        foo = true;
                        
                        send_data[0] = 1;
                        sp.Write(send_data, 0, 1);
                    }
                    else
                    {
                        reset_rapid_variable(sdk_commands[i]);
                        reset_rapid_variable(sdk_commands_rot[i]);
                        
                    }
                }

                if (foo == false) { 
                    //set_lable_text("---");
                    send_data[0] = 2;
                    sp.Write(send_data, 0, 1);
                    //MessageBox.Show(stopwatch.ElapsedMilliseconds.ToString());
                    stopwatch.Reset();
                }
            }
            else if (buttons[25])
            {
                set_lable_text("25");

                if ((buttons[0] || buttons[8] || buttons[15] || buttons[19]) && stop_motion == false)
                {
                    set_rapid_variable(sdk_commands_rot[0]);
                    set_lable_text("rzp"); 
                    send_data[0] = 1;
                    sp.Write(send_data, 0, 1);
                }
                else if ((buttons[1] || buttons[9] || buttons[14] || buttons[18]) && stop_motion == false)
                {
                    set_rapid_variable(sdk_commands_rot[1]);
                    set_lable_text("rzn");
                    send_data[0] = 1;
                    sp.Write(send_data, 0, 1);
                }
                else if ((buttons[2] || buttons[20] || buttons[22] || buttons[5]) && stop_motion == false)
                {
                    set_rapid_variable(sdk_commands_rot[2]);
                    set_lable_text("rxp");
                    send_data[0] = 1;
                    sp.Write(send_data, 0, 1);
                }
                else if ((buttons[3] || buttons[21] || buttons[23] || buttons[4]) && stop_motion == false)
                {
                    set_rapid_variable(sdk_commands_rot[3]);
                    set_lable_text("rxn");
                    send_data[0] = 1;
                    sp.Write(send_data, 0, 1);
                }
                else if ((buttons[6] || buttons[10] || buttons[12] || buttons[17]) && stop_motion == false)
                {
                    set_rapid_variable(sdk_commands_rot[6]);
                    set_lable_text("ryn");
                    send_data[0] = 1;
                    sp.Write(send_data, 0, 1);
                }
                else if ((buttons[7] || buttons[11] || buttons[13] || buttons[16]) && stop_motion == false)
                {
                    set_rapid_variable(sdk_commands_rot[7]);
                    set_lable_text("ryp");
                    send_data[0] = 1;
                    sp.Write(send_data, 0, 1);
                }
                else
                {
                    set_lable_text("---");
                    for (int i = 0; i <= 23; i++)
                    {
                        reset_rapid_variable(sdk_commands[i]);
                        reset_rapid_variable(sdk_commands_rot[i]);
                    }
                    send_data[0] = 2;
                    sp.Write(send_data, 0, 1);
                }
                
            }
            else
            {
                for (int i = 0; i <= 23; i++)
                {
                    reset_rapid_variable(sdk_commands[i]);
                    reset_rapid_variable(sdk_commands_rot[i]);
                }
            }
            
            
            if (buttons[29])
            {
                if (aux_leadthrough)
                {
                    aux_leadthrough = false;
                    set_rapid_variable("sdk_leadthrough_on");
                    reset_rapid_variable("sdk_leadthrough_off");
                    //MessageBox.Show("ON");
                }
                else
                {
                    aux_leadthrough = true;
                    set_rapid_variable("sdk_leadthrough_off");
                    reset_rapid_variable("sdk_leadthrough_on");
                    //MessageBox.Show("OFF");

                }

            }

            


            sp.DiscardInBuffer();
        }

        private void listView1_SelectedIndexChanged(object sender, EventArgs e)
        {

        }
      
        private void button_z_p_MouseDown(object sender, MouseEventArgs e)
        {
            set_rapid_variable("sdk_z_p");
        }

        private void button_z_p_MouseUp(object sender, MouseEventArgs e)
        {
            reset_rapid_variable("sdk_z_p");
        }

        private void button_y_p_MouseDown(object sender, MouseEventArgs e)
        {
            set_rapid_variable("sdk_y_p");
        }

        private void button_y_p_MouseUp(object sender, MouseEventArgs e)
        {
            reset_rapid_variable("sdk_y_p");
        }

        private void button_y_n_MouseDown(object sender, MouseEventArgs e)
        {
            set_rapid_variable("sdk_y_n");
        }

        private void button_y_n_MouseUp(object sender, MouseEventArgs e)
        {
            reset_rapid_variable("sdk_y_n");
        }

        private void button_z_n_MouseDown(object sender, MouseEventArgs e)
        {
            set_rapid_variable("sdk_z_n");
        }

        private void button_z_n_MouseUp(object sender, MouseEventArgs e)
        {
            reset_rapid_variable("sdk_z_n");
        }


        private void button_x_n_MouseDown(object sender, MouseEventArgs e)
        {
            set_rapid_variable("sdk_x_n");
        }

        private void button_x_n_MouseUp(object sender, MouseEventArgs e)
        {
            reset_rapid_variable("sdk_x_n");
        }

        private void button_x_p_MouseDown(object sender, MouseEventArgs e)
        {
            set_rapid_variable("sdk_x_p");
        }

        private void button_x_p_MouseUp(object sender, MouseEventArgs e)
        {
            reset_rapid_variable("sdk_x_p");
        }


        private void button_rz_p_MouseDown(object sender, MouseEventArgs e)
        {
            set_rapid_variable("sdk_rz_p");
        }

        private void button_rz_p_MouseUp(object sender, MouseEventArgs e)
        {
            reset_rapid_variable("sdk_rz_p");
        }

        private void button_ry_p_MouseDown(object sender, MouseEventArgs e)
        {
            set_rapid_variable("sdk_ry_p");
        }

        private void button_ry_p_MouseUp(object sender, MouseEventArgs e)
        {
            reset_rapid_variable("sdk_ry_p");
        }

        private void button_ry_n_MouseDown(object sender, MouseEventArgs e)
        {
            set_rapid_variable("sdk_ry_n");
        }

        private void button_ry_n_MouseUp(object sender, MouseEventArgs e)
        {
            reset_rapid_variable("sdk_ry_n");
        }

        private void button_rz_n_MouseDown(object sender, MouseEventArgs e)
        {
            set_rapid_variable("sdk_rz_n");
        }

        private void button_rz_n_MouseUp(object sender, MouseEventArgs e)
        {
            reset_rapid_variable("sdk_rz_n");
        }


        private void button_rx_n_MouseDown(object sender, MouseEventArgs e)
        {
            set_rapid_variable("sdk_rx_n");
        }

        private void button_rx_n_MouseUp(object sender, MouseEventArgs e)
        {
            reset_rapid_variable("sdk_rx_n");
        }

        private void button_rx_p_MouseDown(object sender, MouseEventArgs e)
        {
            set_rapid_variable("sdk_rx_p");
        }

        private void button_rx_p_MouseUp(object sender, MouseEventArgs e)
        {
            reset_rapid_variable("sdk_rx_p");
        }

        private void cb_lead_through_CheckedChanged(object sender, EventArgs e)
        {
            if (cb_lead_through.Checked)
            {
                set_rapid_variable("sdk_leadthrough_on");
                reset_rapid_variable("sdk_leadthrough_off");
            }
            else
            {
                set_rapid_variable("sdk_leadthrough_off");
                reset_rapid_variable("sdk_leadthrough_on");
                //MessageBox.Show("OFF");
            }
        }

        private void label1_Click(object sender, EventArgs e)
        {
            ;
        }
    }
}