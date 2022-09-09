/*
#################################################################
author: Christoph Mölzer
email: christoph.moelzer@technikum-wien.at
github: 
#################################################################


This node communicates with the arduino.
It subcribes to the button states and publishes the displayed text and the LED states.
It also contains a state machine for the different motion settings

*/

#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sys/ipc.h>
#include <sys/shm.h>

//bool write_to_file (const char * filepath, std::vector<bool> bool_data);
bool write_to_file (const char * filepath, std::vector<bool> bool_data, std::vector<uint32_t> float_data);
//bool write_to_file (const char * filepath, std::vector<bool> bool_data, std::vector<uint8_t> int_data, std::vector<uint8_t> int_data);
//bool write_to_file (const char * filepath, std::vector<bool> bool_data, std::vector<uint8_t> int_data, std::vector<uint8_t> int_data, std::vector<uint8_t> int_data);

bool button_x_n = false;
bool button_x_p = false;
bool button_y_n = false;
bool button_y_p = false;
bool button_z_n = false;
bool button_z_p = false;

bool button_rx_n = false;
bool button_rx_p = false;
bool button_ry_n = false;
bool button_ry_p = false;
bool button_rz_n = false;
bool button_rz_p = false;

float angle = 0;


volatile sig_atomic_t stop;

void inthand(int signum){
  stop = 1;
}

class TON{
  public:
  ros::Duration et;
  ros::Duration pt;
  ros::Time now;
  ros::Time start;
  bool in;
  bool q;
  bool aux;

  void init(){
    et = ros::Duration(0);
    pt = ros::Duration(0);
    now = ros::Time::now();
    start = ros::Time::now();
    in = false;
    q = false;
    aux = false;
  }

  void run(){
    now = ros::Time::now();
    if (in == true && aux == false){
      aux = true;
      start = ros::Time::now();

    }
    et = now-start;
    if (in == true && et >= pt){
      q = true;
    }
    if (in == false){
      aux = false;
      q = false;
    }
  }

  private:
};

class TeachTool
{
  public:

  void init(){
    step = 0;
    entry = true;
    timer.in = false;
    timer.pt = ros::Duration(1);
    timer.run();
  }

  int state_machine(){

    switch (step)
    {
    case 0:   // Init
      // Entry
      if (entry == true){
        entry = false;
        ROS_INFO_STREAM("entry " << step);
      }
      // Cyclic
      timer.in = true;
      timer.pt = ros::Duration(1);

      // Exit
      if (timer.q == true){
        timer.in = false;
        entry = true;
        step = 20;
      }
      break;
    
    case 20:  // IDLE
      // Entry
      if (entry == true){
        entry = false;
        ROS_INFO_STREAM("entry " << step);

      }
      // Cyclic
      timer.in = true;
      timer.pt = ros::Duration(1);

      // Exit
      if (timer.q == true){
        timer.in = false;
        entry = true;
        step = 0;
      }
      break;

    case 100:  // Translation
      // Entry
      if (entry == true){
        entry = false;
        ROS_INFO_STREAM("entry " << step);

      }
      // Cyclic
      timer.in = true;
      timer.pt = ros::Duration(1);

      // Exit
      if (timer.q == true){
        timer.in = false;
        entry = true;
        step = 0;
      }
      break;
    
    case 200:  // Rotation
      // Entry
      if (entry == true){
        entry = false;
        ROS_INFO_STREAM("entry " << step);

      }
      // Cyclic
      timer.in = true;
      timer.pt = ros::Duration(1);

      // Exit
      if (timer.q == true){
        timer.in = false;
        entry = true;
        step = 0;
      }
      break;
    
    case 300:  // Lead Through
      // Entry
      if (entry == true){
        entry = false;
        ROS_INFO_STREAM("entry " << step);

      }
      // Cyclic
      timer.in = true;
      timer.pt = ros::Duration(1);

      // Exit
      if (timer.q == true){
        timer.in = false;
        entry = true;
        step = 0;
      }
      break;
    
    case 400:  // 
      // Entry
      if (entry == true){
        entry = false;
        ROS_INFO_STREAM("entry " << step);

      }
      // Cyclic
      timer.in = true;
      timer.pt = ros::Duration(1);

      // Exit
      if (timer.q == true){
        timer.in = false;
        entry = true;
        step = 0;
      }
      break;
    
    

    default:
      break;
    }

    timer.run();

    return step;
  }


  private:
  int step;
  bool entry;
  TON timer;

};

class Debug_Info
{
  public:
  ros::Duration t_et;
  ros::Duration t_pt;
  ros::Time t_now;
  ros::Time t_start;
  bool in;
  bool q;
  bool aux;

  void init(){
    t_et = ros::Duration(0);
    t_pt = ros::Duration(0);
    t_now = ros::Time::now();
    t_start = ros::Time::now();
    in = false;
    q = false;
    aux = false;
  }

  void run(){
    if (q == true && false){
      std::cout << "in: " << in << std::endl;
      std::cout << "aux: " << aux << std::endl;
      std::cout << "q: " << q << std::endl;
      std::cout << "pt: " << t_pt << std::endl;
      std::cout << "start: " << t_start << std::endl;
      std::cout << "now: " << t_now << std::endl;
      std::cout << "et: " << t_et << std::endl;
      std::cout << std::endl;
      //std::cout << "" <<  << std::endl;
    }
    t_now = ros::Time::now();
    if (in == true && aux == false){
      aux = true;
      t_start = ros::Time::now();
    }
    t_et = t_now-t_start;
    if (in == true && t_et >= t_pt){
      q = true;
    }
    if (in == false){
      aux = false;
      q = false;
    }
  }


  
};

class EGM_Motion
{

    public:
    Debug_Info debug_info;
    double x, y, z, rx, ry, rz;


    void init(){
      x=0;
      y=0;
      z=0;
      rx=0;
      ry=0;
      rz=0;
      t_prev = ros::Time::now();
      debug_info.init();
    }

       

    void step_x(double value){
      ROS_INFO_STREAM("move x: " << value << " mm");
      x = value*cos(angle);
      y = value*sin(angle);
      //step(value*cos(angle), value*sin(angle), 0, 0, 0, 0);
    }

    void step_y(double value){
      ROS_INFO_STREAM("move y: " << value << " mm");
      x = value*sin(-angle);
      y = value*cos(-angle);
      //step(value*sin(-angle), value*cos(-angle), 0, 0, 0, 0);
    }

    void step_z(double value){
      ROS_INFO_STREAM("move z: " << value << " mm");
      z = value;
      //step(0, 0, value, 0, 0, 0);
    }

    void step_rx(double value){
      ROS_INFO_STREAM("move rx: " << value << " °");
      //step(0, 0, 0, value, 0, 0);
    }

    void step_ry(double value){
      ROS_INFO_STREAM("move ry: " << value << " °");
      //step(0, 0, 0, 0, value, 0);
    }

    void step_rz(double value){
      ROS_INFO_STREAM("move rz: " << value << " °");
      //step(0, 0, 0, 0, 0, value);
    }

    void step_up(double value){
        ;//step(value);
    }

    void step_down(double value){
      ;//step(-value);
    }

    private:
    double position_reference;
    double new_x, new_y;
    double diff_old, diff;
    ros::Duration t_elapsed;
    ros::Time t_now;
    ros::Time t_prev;

};

class Display{
  public:
  ros::NodeHandle n;
  std::string str_line_1;
  std::string str_line_2;
  std::string str_button_1;
  std::string str_button_2;

  ros::Publisher pub_line_1;
  std_msgs::String line_1;

  Display(ros::NodeHandle& _n){
    n = _n;
    pub_line_1 = n.advertise<std_msgs::String>("/display_line_1", 1);
  }

  void publish(){
    line_1.data = "foo";
    pub_line_1.publish(line_1);
  }
  
};

class Communication{
    public:
    ros::NodeHandle n;
    ros::Publisher pub_1, pub_2, pub_3, pub_4, pub_5, pub_6 ;
    ros::Subscriber sub_1, sub_2, sub_3, sub_4, sub_5, sub_6;
    std_msgs::String msg_1, msg_2, msg_3, msg_4, msg_5;
    std_msgs::Bool msg_6;
    std::vector<std::string> button_names_translation,  button_names_rotation;
    std::vector<bool> button_values;
    std::vector<bool> bool_data;
    std::vector<uint8_t> int_data;
    std::vector<uint32_t> float_data;

    Communication(){
        ROS_INFO_STREAM("Init communication");
        pub_1 = n.advertise<std_msgs::String>("/text_line_1", 1);
        pub_2 = n.advertise<std_msgs::String>("/text_line_2", 1);
        pub_3 = n.advertise<std_msgs::String>("/text_button_left", 1);
        pub_4 = n.advertise<std_msgs::String>("/text_button_right", 1);
        pub_5 = n.advertise<std_msgs::String>("/text_button_center", 1);
        pub_6 = n.advertise<std_msgs::Bool>("/life_sign_to_mcu", 1);

        sub_1 = n.subscribe("/pushed", 100, &Communication::callback_motion_command, this);
        sub_2 = n.subscribe("angle", 100, &Communication::callback_angle, this);
        sub_3 = n.subscribe("speed", 100, &Communication::callback_3, this);
        sub_4 = n.subscribe("life_sign_from_mcu", 100, &Communication::callback_4, this);
    }

    void init_buttons(){

      // Translation
      button_names_translation.clear();
      // x-axis +
      button_names_translation.push_back("z-");
      button_names_translation.push_back("z+");
      button_names_translation.push_back("y+");
      button_names_translation.push_back("y-");
      button_names_translation.push_back("x-");

      // x-axis -
      button_names_translation.push_back("z-");
      button_names_translation.push_back("z+");
      button_names_translation.push_back("y-");
      button_names_translation.push_back("y+");
      button_names_translation.push_back("x+");

      // y-axis +
      button_names_translation.push_back("z-");
      button_names_translation.push_back("z+");
      button_names_translation.push_back("x-");
      button_names_translation.push_back("x+");
      button_names_translation.push_back("y-");

      // y-axis -
      button_names_translation.push_back("z-");
      button_names_translation.push_back("z+");
      button_names_translation.push_back("x+");
      button_names_translation.push_back("x-");
      button_names_translation.push_back("y+");

      // Rotation
      button_names_rotation.clear();
      // x-axis +
      button_names_rotation.push_back("ry+");
      button_names_rotation.push_back("ry-");
      button_names_rotation.push_back("rz+");
      button_names_rotation.push_back("rz-");
      button_names_rotation.push_back("n.d.");

      // x-axis -
      button_names_rotation.push_back("ry-");
      button_names_rotation.push_back("ry+");
      button_names_rotation.push_back("rz+");
      button_names_rotation.push_back("rz-");
      button_names_rotation.push_back("n.d.");

      // y-axis +
      button_names_rotation.push_back("rx-");
      button_names_rotation.push_back("rx+");
      button_names_rotation.push_back("rz+");
      button_names_rotation.push_back("rz-");
      button_names_rotation.push_back("n.d.");

      // y-axis -
      button_names_rotation.push_back("rx+");
      button_names_rotation.push_back("rx-");
      button_names_rotation.push_back("rz+");
      button_names_rotation.push_back("rz-");
      button_names_rotation.push_back("n.d.");



    }

    void callback_motion_command(const std_msgs::UInt32& msg){
        //ROS_INFO_STREAM("callback_buttons" << msg.data);
        converter(msg.data);
    }

    void callback_angle(const std_msgs::Float32& msg){
        angle = msg.data;
        float_data.clear();
        float_data.push_back(angle*1000*1000);
        ROS_INFO_STREAM(angle);
    }

    void callback_3(const std_msgs::UInt16& msg){
        ROS_INFO_STREAM("callback_speed");
    }

    void callback_4(const std_msgs::Bool& msg){
        ROS_INFO_STREAM("callback_life_sign_from_mcu");
    }

    void converter(uint32_t number){
        /* 
        *     LINEAR | ROTARY
        *   0 - X-   |   RZ+
        *   1 - X+   |   RZ-
        *   2 - Y-   |   RX+
        *   3 - Y+   |   RX-
        *   4 - Z-   |   RX-
        *   5 - Z+   |   RX+
        *   
        *   6 - X-   |   RY-
        *   7 - X+   |   RY+
        *   8 - Y-   |   RZ+
        *   9 - Y+   |   RZ-
        *  10 - Z-   |   RY-
        *  11 - Z+   |   RY+
        *   
        *  12 - X-   |   RY-
        *  13 - X+   |   RY+
        *  14 - Y-   |   RZ-
        *  15 - Y+   |   RZ+
        *  16 - Z-   |   RY+
        *  17 - Z+   |   RY-
        *   
        *  18 - X-   |   RZ-
        *  19 - X+   |   RZ+
        *  20 - Y-   |   RX+
        *  21 - Y+   |   RX-
        *  22 - Z-   |   RX+
        *  23 - Z+   |   RX-
        * 
        *  24 - MODE LIN
        *  25 - MODE ROT
        *  26 - MODE CONT
        *  27 - MODE STEP
        *  28 - MODE BUTTON
        *  29 - MODE LEAD THROUGH
        *  30 - 
        *  31 - 
        */  
        bool cmd_bits[32];
        bool mode_lin = false;
        bool mode_rot = false;
        bool mode_cont = false;
        bool mode_step = false;
        bool mode_button = false;
        bool mode_lead_through = false;

        bool_data.clear();
      
        for(int i=0; i<sizeof(cmd_bits); i++){
            cmd_bits[i]=false;       
        }
        for(int i=0; number>0; i++) {   
            cmd_bits[i]=number%2;    
            number= number/2;  
        }

        mode_lin = cmd_bits[24];
        mode_rot = cmd_bits[25];
        mode_cont = cmd_bits[26];
        mode_step = cmd_bits[27];
        mode_button = cmd_bits[28];
        mode_lead_through = cmd_bits[29];
        
        if (mode_lin && !mode_rot){
          button_x_n = cmd_bits[0] or cmd_bits[6] or cmd_bits[12] or cmd_bits[18];
          button_x_p = cmd_bits[1] or cmd_bits[7] or cmd_bits[13] or cmd_bits[19];
          button_y_n = cmd_bits[2] or cmd_bits[8] or cmd_bits[14] or cmd_bits[20];
          button_y_p = cmd_bits[3] or cmd_bits[9] or cmd_bits[15] or cmd_bits[21];
          button_z_n = cmd_bits[4] or cmd_bits[10] or cmd_bits[16] or cmd_bits[22];
          button_z_p = cmd_bits[5] or cmd_bits[11] or cmd_bits[17] or cmd_bits[23];
          button_rx_n = false;
          button_rx_p = false;
          button_ry_n = false;
          button_ry_p = false;
          button_rz_n = false;
          button_rz_p = false;

        }
        else if(mode_rot && !mode_lin){
          button_rx_n = cmd_bits[4] or cmd_bits[21] or cmd_bits[3] or cmd_bits[23];
          button_rx_p = cmd_bits[5] or cmd_bits[2] or cmd_bits[20] or cmd_bits[22];
          button_ry_n = cmd_bits[12] or cmd_bits[10] or cmd_bits[17] or cmd_bits[6];
          button_ry_p = cmd_bits[7] or cmd_bits[11] or cmd_bits[16] or cmd_bits[13];
          button_rz_n = cmd_bits[1] or cmd_bits[9] or cmd_bits[14] or cmd_bits[18];
          button_rz_p = cmd_bits[0] or cmd_bits[8] or cmd_bits[15] or cmd_bits[19];
          button_x_n = false;
          button_x_p = false;
          button_y_n = false;
          button_y_p = false;
          button_z_n = false;
          button_z_p = false;

        }        

        bool_data.push_back(button_x_p);
        bool_data.push_back(button_x_n);
        bool_data.push_back(button_y_p);
        bool_data.push_back(button_y_n);
        bool_data.push_back(button_z_p);
        bool_data.push_back(button_z_n);
        bool_data.push_back(button_rx_p);
        bool_data.push_back(button_rx_n);
        bool_data.push_back(button_ry_p);
        bool_data.push_back(button_ry_n);
        bool_data.push_back(button_rz_p);
        bool_data.push_back(button_rz_n);
        bool_data.push_back(mode_lead_through);

        ROS_INFO_STREAM("LIN: " << button_x_p << button_x_n << button_y_p << button_y_n << button_z_p << button_z_n);
        ROS_INFO_STREAM("ROT: " << button_rx_p << button_rx_n << button_ry_p << button_ry_n << button_rz_p << button_rz_n);
        ROS_INFO_STREAM("LEAD TROUGH: " << mode_lead_through);

    }

    void convert_float32_to_uint8_t(float angle){
        uint32_t bit_mask=1; 
        uint32_t val = angle*1000*1000;
        std::cout << "val = " << val << std::endl;
        for(int bit=0; bit<32; bit++){
          std::cout << "Bit " << bit << " = " << (bool)(val&bit_mask) << std::endl;
          bit_mask*=2;
        }
    }

    void publish(){
        msg_1.data = "l1";
        msg_2.data = "l2";
        msg_3.data = "btn_l";
        msg_4.data = "btn_r";
        msg_5.data = "btn_c";
        msg_6.data = true;
        pub_1.publish(msg_1);
        pub_2.publish(msg_2);
        pub_3.publish(msg_3);
        pub_4.publish(msg_4);
        pub_5.publish(msg_5);
        pub_6.publish(msg_6);
    }
  
};

class Buttons{
  public:
  ros::NodeHandle n;
  ros::Subscriber sub_buttons;
  
  Buttons(ros::NodeHandle& _n){
    n = _n;
    //sub_buttons = n.subscribe("/topic", 100, callback);
  }

  void converter(){
    bool buttons[32] = {true,false,false,false,false, // group 1
                        false,false,false,false,false, // group 2
                        false,false,false,false,false, // group 3
                        false,false,false,false,false, // group 4
                        false,false,false,false,false,
                        false,false,false,false,false,
                        false, false};
    int i=0;
    uint32_t n=13;
    int a[32];
    uint32_t result=0;

    for (i=0; i<sizeof(buttons); i++){
        ROS_INFO_STREAM(buttons[31-i]);
        if (buttons[31-i] == true){
            result+=pow(2,i);
        }
    }

    ROS_INFO_STREAM("result = " << result);
    n=result;
    for(i=0; n>0; i++)    
    {    
        a[i]=n%2;    
        n= n/2;  
    }    
    ROS_INFO_STREAM("Binary of the given number= ");    
    for(i=i-1 ;i>=0 ;i--)    
    {    
        ROS_INFO_STREAM(a[i]);    
    }   
}

  void callback(const std_msgs::UInt32::ConstPtr& msg){
    ;
  }

  void sub(){
    ;//sub_buttons = n.subscribe("/topic", 100, callback);
  }

};

void callback(const std_msgs::UInt32::ConstPtr& msg){
  ROS_INFO_STREAM("int: " << msg->data);
}

bool read_from_file (const char * filepath)
{
   FILE * fp;
   char ch;
   int eof_indicator;

   fp = fopen (filepath, "r");
   if (fp == NULL)
   {
      ROS_INFO_STREAM("could not open file");
      return false;
   }

   ch = fgetc (fp);
   ROS_INFO_STREAM(ch);
   eof_indicator = feof (fp);
   fclose (fp);

   if (eof_indicator)
   {
      ROS_INFO_STREAM("EOF");
      return false;
   }
   return ch == '1';
}

/**
 * Write a bool to a file
 *
 * @param filepath      In: Path to file
 * @return true if file exists and the first character is '1'
 */
bool write_to_file (const char * filepath, std::vector<bool> bool_data ,std::vector<uint32_t> float_data)
{
   FILE * fp;

   fp = fopen (filepath, "w");
   if (fp == NULL)
   {
      printf("could not open file\n");
      return false;
   }
   
   for(int i=0; i<64; i++){
      if(i<bool_data.size()){
        bool_data.at(i) ? fprintf(fp, "1") : fprintf(fp, "0");
      }
      else{
        fprintf(fp, "0");
      }
   }

   for(int i=0; i<2; i++){
      uint32_t bit_mask = 1;
      if(i<float_data.size()){
        for (int bit=0; bit<sizeof(float_data.at(0))*8; bit++){
          (float_data.at(i)&bit_mask) ? fprintf(fp, "1") : fprintf(fp, "0");
          bit_mask*=2;
        }
      }
      else{
        for(int bit=0; bit<32; bit++){
          fprintf(fp, "0");
        }
      }
   }
   fclose (fp);

   return true;
}



int main(int argc, char** argv)
{
  //----------------------------------------------------------
  // Preparations
  //----------------------------------------------------------
  // Initialize the node.
  ros::init(argc, argv, "pose_controller_node");
  Communication com;
  EGM_Motion robot;
  signal(SIGINT, inthand);
  ROS_INFO_STREAM("NODE STARTED");


  ros::Rate rate(250);
  //sensor_msgs::JointState msg;
  geometry_msgs::Pose msg;

    

  // Boost components for managing asynchronous UDP socket(s).
  /*TeachTool tool;
  Display disp(node_handle);
  tool.init();
  while (!stop){
    tool.state_machine();
    disp.publish();
    rate.sleep();
  }
  return 0;
  */

 
  bool wait = true;
  robot.init();
  wait = true;

  while(ros::ok()  && !stop)
  {
    // Wait for a new EGM message from the EGM client (with a timeout of 500 ms).
  
    //read_from_file("/home/chris/profinet/build/data.txt");
    
    //states_pub.publish(msg);
    //ros::Duration(0.5).sleep();
    write_to_file("/home/chris/catkin_ws/src/robot_teach_in/data/data_from_arduino.txt" ,com.bool_data, com.float_data);
    ros::spinOnce();
    rate.sleep();  
  } 
  

  return 0;
}
