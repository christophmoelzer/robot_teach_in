


using System.IO.Ports;

using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.IOSystemDomain;
using System.Timers;

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
        


        public Form1()
        {
            InitializeComponent();
            
            this.FormClosed += new FormClosedEventHandler(Form1_FormClosed);
            if (port == null)
            {
                port = new SerialPort("COM4", 9600);
                try
                {
                    port.Open();
                    port.DataReceived += new SerialDataReceivedEventHandler(DataReceived);
                }
                catch
                {
                    ;// MessageBox.Show("COM4 not available.");
                }
            }

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
                    controller.Logon(UserInfo.DefaultUser);
                    
                }
                else
                {
                    MessageBox.Show("Selected controller not available.");
                }
            }


        }

        void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
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

        static int myNum;


        private static void set_rapid_variable(string key_name)
        {
            Bool rapidBool;
            RapidData rdBool = controller.Rapid.GetRapidData("T_ROB1", "module_mr19m010", key_name);
            if (rdBool.Value is Bool)
            {
                rapidBool = (Bool)rdBool.Value;

                bool boolValue = rapidBool.Value;

                rapidBool.Value = true;

                using (Mastership.Request(controller.Rapid))
                {
                    rdBool.Value = rapidBool;
                }
            }
        }

        private static void reset_rapid_variable(string key_name)
        {
            Bool rapidBool;
            RapidData rdBool = controller.Rapid.GetRapidData("T_ROB1", "module_mr19m010", key_name);
            if (rdBool.Value is Bool)
            {
                rapidBool = (Bool)rdBool.Value;

                bool boolValue = rapidBool.Value;

                rapidBool.Value = false;

                using (Mastership.Request(controller.Rapid))
                {
                    rdBool.Value = rapidBool;
                }
            }
        }

        static bool aux_leadthrough = false;

        private static void DataReceived(Object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;

            myNum = 4;

            byte[] data = new byte[4];
            data[0] = (byte)sp.ReadByte();
            data[1] = (byte)sp.ReadByte();
            data[2] = (byte)sp.ReadByte();
            data[3] = (byte)sp.ReadByte();

            bool[] buttons = new bool[32];
            for(int i=0; i < 32; i++) { buttons[i] = false; }

            string[] sdk_commands = new string[32];
            sdk_commands[0] = "sdk_x_n";
            sdk_commands[1] = "sdk_x_p";
            sdk_commands[2] = "sdk_y_n";
            sdk_commands[3] = "sdk_y_p";
            sdk_commands[4] = "sdk_z_n";
            sdk_commands[5] = "sdk_z_p";
            sdk_commands[6] = "sdk_x_n";
            sdk_commands[7] = "sdk_x_p";
            sdk_commands[8] = "sdk_y_n";
            sdk_commands[9] = "sdk_y_p";
            sdk_commands[10] = "sdk_z_n";
            sdk_commands[11] = "sdk_z_p";
            sdk_commands[12] = "sdk_x_n";
            sdk_commands[13] = "sdk_x_p";
            sdk_commands[14] = "sdk_y_n";
            sdk_commands[15] = "sdk_y_p";
            sdk_commands[16] = "sdk_z_n";
            sdk_commands[17] = "sdk_z_p";
            sdk_commands[18] = "sdk_x_n";
            sdk_commands[19] = "sdk_x_p";
            sdk_commands[20] = "sdk_y_n";
            sdk_commands[21] = "sdk_y_p";
            sdk_commands[22] = "sdk_z_n";
            sdk_commands[23] = "sdk_z_p";
            sdk_commands[24] = "sdk_mode_lin";
            sdk_commands[25] = "sdk_mode_lin!";
            sdk_commands[26] = "n.d.";
            sdk_commands[27] = "n.d.";
            sdk_commands[28] = "n.d.";
            sdk_commands[29] = "sdk_leadthrough";
            sdk_commands[30] = "n.d.";
            sdk_commands[31] = "n.d.";


            byte aux = 0;
            string myString = "";

            for(int x=0; x<4; x++)
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

            for (int i=0; i<6; i++)
            {
                if (buttons[i] || buttons[i + 6] || buttons[i + 2*6] || buttons[i + 3 * 6])
                {
                    set_rapid_variable(sdk_commands[i]);
                }
                else
                {
                    reset_rapid_variable(sdk_commands[i]);
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
                MessageBox.Show("OFF");
            }
        }
    }
}