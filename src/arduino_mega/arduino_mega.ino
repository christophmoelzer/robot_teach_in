/*
Author: Christoph Moelzer
Mail: christoph.moelzer@outlook.com
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ButtonDebounce.h>
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Bool.h>

#define nav_left 7
#define nav_right 6
#define nav_up 5
#define nav_down 4
#define nav_center 16
#define nav_abort 17
#define nav_enter 14
#define led_onboard 13

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32


Encoder encoder(2,3);

class Timer{
public:
  unsigned long st;
  unsigned long et;
  unsigned long pt;
  bool aux;
  bool in;
  bool q;

  void run(){
    if (in == true and aux == false){
      aux = true;
      st = millis();
      q = false;
      et = 0;
    }
    et = millis()-st;
    if (et > pt){
      q = true;
    }
    if (in == false){
      aux = false;
      q = false;
    }
  }  
private:
};

class WaitMs{
  public:

  void ms(int ms){
    for (int i=0; i < ms; i++){
      delayMicroseconds(1000);
    }
  }
  private:
};

class Button{
public:
ButtonDebounce db;
bool pressed;
int debounce_time;
String label;
Button(){
  ;
}
Button(int pin){
  debounce_time = 50;
  pressed = false;
  db = ButtonDebounce(pin, debounce_time);
}

Button(int pin, int dbt, String lb){
  label = lb;
  debounce_time = dbt;
  pressed = false;
  db = ButtonDebounce(pin, debounce_time);
}


bool update(){
  db.update();
  return state();
}

bool state(){
  bool state = db.state();
  if (state == LOW){
    Serial.println(label);
  }
  return state;
}

private:
};

class LED{
  public:
  int _pin_r;
  int _pin_g;
  int _pin_b;
  LED(){
    ;
  }
  LED(int pin_r, int pin_g, int pin_b){
    _pin_r = pin_r;
    _pin_g = pin_g;
    _pin_b = pin_b;
    pinMode(_pin_r, OUTPUT);
    pinMode(_pin_g, OUTPUT);
    pinMode(_pin_b, OUTPUT);
  }

  void red(){
    digitalWrite(_pin_r, HIGH);
    digitalWrite(_pin_g, LOW);
    digitalWrite(_pin_b, LOW);
  }

  void green(){
    digitalWrite(_pin_r, LOW);
    digitalWrite(_pin_g, HIGH);
    digitalWrite(_pin_b, LOW);
  }

  void blue(){
    digitalWrite(_pin_r, LOW);
    digitalWrite(_pin_g, LOW);
    digitalWrite(_pin_b, HIGH);
  }

  void white(){
    digitalWrite(_pin_r, HIGH);
    digitalWrite(_pin_g, HIGH);
    digitalWrite(_pin_b, HIGH);
  }

  void black(){
    digitalWrite(_pin_r, LOW);
    digitalWrite(_pin_g, LOW);
    digitalWrite(_pin_b, LOW);
  }

  void violett(){
    digitalWrite(_pin_r, HIGH);
    digitalWrite(_pin_g, LOW);
    digitalWrite(_pin_b, HIGH);
  }

   void yellow(){
    digitalWrite(_pin_r, LOW);
    digitalWrite(_pin_g, HIGH);
    digitalWrite(_pin_b, HIGH);
  }

   void light_blue(){
    digitalWrite(_pin_r, HIGH);
    digitalWrite(_pin_g, HIGH);
    digitalWrite(_pin_b, LOW);
  }

  private:
};

class Point{
  public:
  int x, y;
  Point(){
    x=0;
    y=0;
  }

  Point(int _x, int _y){
    x=_x;
    y=_y;
  }

  private:
};

class Line{
public:
Point start, end;
Line(){
  start = Point(0,0);
  end = Point(0,0);
}
Line(Point _start, Point _end){
  start = _start;
  end = _end;
}

private:
};

class Page{
public:
Line line[10];

Page(){

  line[0].start = Point(0, 0);
  line[0].end = Point(SCREEN_WIDTH-1, 0);

  line[1].start = Point(0, SCREEN_HEIGHT/3);
  line[1].end = Point(SCREEN_WIDTH-1, SCREEN_HEIGHT/3);

  line[2].start = Point(0, SCREEN_HEIGHT*2/3);
  line[2].end = Point(SCREEN_WIDTH-1, SCREEN_HEIGHT*2/3);

  line[3].start = Point(0, SCREEN_HEIGHT-1);
  line[3].end = Point(SCREEN_WIDTH-1, SCREEN_HEIGHT-1);

  line[4].start = Point(0, 0);
  line[4].end = Point(0, SCREEN_HEIGHT-1);

  line[5].start = Point(SCREEN_WIDTH/2, 0);
  line[5].end = Point(SCREEN_WIDTH/2, SCREEN_HEIGHT-1);

  line[6].start = Point(SCREEN_WIDTH-1, 0);
  line[6].end = Point(SCREEN_WIDTH-1, SCREEN_HEIGHT-1);
  
}



private:

};

class Motion{
  public:
  Button btn_grp_1_y_negative;
  Button btn_grp_1_z_negative;
  Button btn_grp_1_z_positive;
  Button btn_grp_1_x_negative;
  Button btn_grp_1_x_positive;
  
  Button btn_grp_2_x_positive;
  Button btn_grp_2_z_negative;
  Button btn_grp_2_z_positive;
  Button btn_grp_2_y_negative;
  Button btn_grp_2_y_positive;
  
  Button btn_grp_3_y_positive;
  Button btn_grp_3_z_negative;
  Button btn_grp_3_z_positive;
  Button btn_grp_3_x_negative;
  Button btn_grp_3_y_negative;
  
  Button btn_grp_4_x_positive;
  Button btn_grp_4_z_negative;
  Button btn_grp_4_z_positive;
  Button btn_grp_4_x_negative;
  Button btn_grp_4_y_positive;

  Motion(){
    btn_grp_1_x_negative = Button(52);
    btn_grp_1_x_positive = Button(48);
    btn_grp_1_y_negative = Button(46);
    btn_grp_1_z_negative = Button(53);
    btn_grp_1_z_positive = Button(50);

    btn_grp_2_x_positive = Button(51);
    btn_grp_2_y_negative = Button(45);
    btn_grp_2_y_positive = Button(49);
    btn_grp_2_z_negative = Button(43);
    btn_grp_2_z_positive = Button(47);

    btn_grp_3_x_negative = Button(41);
    btn_grp_3_y_negative = Button(39);
    btn_grp_3_y_positive = Button(35);
    btn_grp_3_z_negative = Button(33);
    btn_grp_3_z_positive = Button(37);

    btn_grp_4_x_positive = Button(25);
    btn_grp_4_x_negative = Button(29);
    btn_grp_4_y_positive = Button(31);
    btn_grp_4_z_negative = Button(23);
    btn_grp_4_z_positive = Button(27);

  }

  void update_buttons(){
    btn_grp_4_x_negative.db.update();
    if (btn_grp_4_x_negative.db.state() == LOW){
      Serial.println("g4xn pressed");
    }
    else{
      Serial.println("g4xn released");
    }

    btn_grp_4_x_positive.db.update();
    if (btn_grp_4_x_positive.db.state() == LOW){
      Serial.println("g4xp pressed");
    }
    else{
      Serial.println("g4xp released");
    }

    btn_grp_4_y_positive.db.update();
    if (btn_grp_4_y_positive.db.state() == LOW){
      Serial.println("g4yp pressed");
    }
    else{
      Serial.println("g4yp released");
    }

    btn_grp_4_z_negative.db.update();
    if (btn_grp_4_z_negative.db.state() == LOW){
      Serial.println("g4zn pressed");
    }
    else{
      Serial.println("g4zn released");
    }

    btn_grp_4_z_positive.db.update();
    if (btn_grp_4_z_positive.db.state() == LOW){
      Serial.println("g4zp pressed");
    }
    else{
      Serial.println("g4zp released");
    }

    
  }
  

  private:
};

class GUI{
public:
Adafruit_SSD1306 display;
Button btn_nav_left;
Button btn_nav_right;
Button btn_nav_up;
Button btn_nav_down;
Button btn_nav_center;
Button btn_nav_abort;
Button btn_nav_enter;
LED led_1, led_2, led_3, led_4;
bool nav_button_pressed;
int cursor_position;
int col_width;
float increment;
int velocity ;
bool mode_lin;
bool mode_rot;
WaitMs wait;

long pos_encoder;

GUI(){
   
  btn_nav_left = Button(nav_left);
  btn_nav_right = Button(nav_right);
  btn_nav_up = Button(nav_up);
  btn_nav_down = Button(nav_down);
  btn_nav_center = Button(nav_center);
  btn_nav_abort = Button(nav_abort);
  btn_nav_enter = Button(nav_enter);
  led_1 = LED(22, 24, 26);
  led_2 = LED(28, 30, 32);
  led_3 = LED(34, 36, 38);
  led_4 = LED(40, 42, 44);
  led_1.red();
  led_2.red();
  led_3.red();
  led_4.red();
  wait.ms(100);
  led_1.green();
  led_2.green();
  led_3.green();
  led_4.green();
  wait.ms(100);
  led_1.blue();
  led_2.blue();
  led_3.blue();
  led_4.blue();
  wait.ms(100);
  led_1.white();
  led_2.white();
  led_3.white();
  led_4.white();
  wait.ms(100);
  led_1.black();
  led_2.black();
  led_3.black();
  led_4.black();

}

void init(){
  display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    led_1.red();
    for(;;); // Don't proceed, loop forever
  }
  led_1.black();

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("FOO");
  display.display();

  cursor_position = 0;
  col_width = 42;
  increment = 0.1;
  velocity = 50;
  mode_lin = true;
  mode_rot = false;
  pos_encoder = -999;
  
}

void upt_encoder(){
  long new_value;
  new_value = encoder.read();
  if (new_value != pos_encoder){
    Serial.println(new_value);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println((new_value*360)/624);
    calc_arrow(new_value*2*PI/624);
    display.display();
    pos_encoder = new_value;
  }
  
}

void calc_arrow(float angle){
  int length = 31;

  Point start(128/2, 64/2);
  Point vector(length*cos(angle), length*sin(angle));
  Point end(start.x+vector.x, start.y+vector.y);
  Line line(start,end);  
  display.drawLine(line.start.x, line.start.y, line.end.x, line.end.y, SSD1306_WHITE);
  display.drawLine(line.start.x, line.start.y-1, line.end.x, line.end.y-1, SSD1306_WHITE);
  
}


void update_buttons(){
  btn_nav_left.db.update();
  btn_nav_right.db.update();
  btn_nav_up.db.update();
  btn_nav_down.db.update();
  btn_nav_center.db.update();
  btn_nav_abort.db.update();
  btn_nav_enter.db.update();

  if ((btn_nav_left.db.state() == LOW) and (btn_nav_left.pressed == false)){
    btn_nav_left.pressed = true;
    led_3.white();
    switch (cursor_position){
      case 0:
        if (mode_lin == false){
          mode_lin = true;
          mode_rot = false;
        }
        else {
          mode_lin = false;
          mode_rot = true;
        }
      break;
      case 1:
        if (velocity >= 1){
          velocity-=1;
        }
        else{
          velocity=1;
        }
      break;
      case 2:
        if (increment > 0.1){
          increment-=0.1;
        }
        else{
          increment=0;
        }
      break;
    }
  }
  else if (btn_nav_left.db.state() == HIGH){
    btn_nav_left.pressed = false;
    led_3.black();
  }


  if ((btn_nav_right.db.state() == LOW) and (btn_nav_right.pressed == false)){
    btn_nav_right.pressed = true;
    led_2.white();
    switch (cursor_position){
      case 0:
        if (mode_lin == false){
          mode_lin = true;
          mode_rot = false;
        }
        else {
          mode_lin = false;
          mode_rot = true;
        }
      break;
      case 1:
        if (velocity <= 50){
          velocity+=1;
        }
        else{
          velocity=50;
        }
      break;
      case 2:
        if (increment < 10){
          increment+=0.1;
        }
        else{
          increment=10;
        }
      break;
    }
  }
  else if (btn_nav_right.db.state() == HIGH){
    btn_nav_right.pressed = false;
    led_2.black();
  }


  if ((btn_nav_up.db.state() == LOW) and (btn_nav_up.pressed == false)){
    btn_nav_up.pressed = true;
    led_4.white();
    if (cursor_position > 0){
      cursor_position--;
    }
    else{
      cursor_position=0;
    }
  }
  else if (btn_nav_up.db.state() == HIGH){
    btn_nav_up.pressed = false;
    led_4.black();
  }


  if ((btn_nav_down.db.state() == LOW) and (btn_nav_down.pressed == false)){
    btn_nav_down.pressed = true;
    led_1.white();
    if (cursor_position < 2){
      cursor_position++;
    }
    else{
      cursor_position=2;
    }
  }
  else if (btn_nav_down.db.state() == HIGH){
    btn_nav_down.pressed = false;
    led_1.black();
  }


  if ((btn_nav_center.db.state() == LOW) and (btn_nav_center.pressed == false)){
    btn_nav_center.pressed = true;
  }
  else if (btn_nav_center.db.state() == HIGH){
    btn_nav_center.pressed = false;
  }


  if ((btn_nav_abort.db.state() == LOW) and (btn_nav_abort.pressed == false)){
    btn_nav_abort.pressed = true;
  }
  else if (btn_nav_abort.db.state() == HIGH){
    btn_nav_abort.pressed = false;
  }


  if ((btn_nav_enter.db.state() == LOW) and (btn_nav_enter.pressed == false)){
    btn_nav_enter.pressed = true;
  }
  else if (btn_nav_enter.db.state() == HIGH){
    btn_nav_enter.pressed = false;
  }

  if (btn_nav_left.pressed or btn_nav_right.pressed or btn_nav_up.pressed or btn_nav_down.pressed or btn_nav_center.pressed or btn_nav_abort.pressed or btn_nav_enter.pressed){
    nav_button_pressed = true;
  }
  else{
    ;
  }
}

void draw_lines(void){
  int gap = display.height()/3;
  display.drawLine(0, 0, display.width()-1, 0, SSD1306_WHITE);
  display.drawLine(0, gap, display.width()-1, gap, SSD1306_WHITE);
  display.drawLine(0, gap*2, display.width()-1, gap*2, SSD1306_WHITE);
  display.drawLine(0, display.height()-1, display.width()-1, display.height()-1, SSD1306_WHITE);
  display.drawLine(0, 0, 0, display.height()-1, SSD1306_WHITE);
  display.drawLine(display.width()-1, 0, display.width()-1, display.height()-1, SSD1306_WHITE);
  display.drawLine(col_width, 0, col_width, display.height()-1, SSD1306_WHITE);
  
}

  void draw_background(bool invert){
    int gap = display.height()/3;
    static bool bg_black[3] = {false, false, false};
    if (invert == true){
      display.fillRect(2, (cursor_position*gap)+2, col_width-3, gap-3, SSD1306_WHITE);
      display.fillRect(col_width+2, (cursor_position*gap)+2, display.width()-col_width-4, gap-3, SSD1306_WHITE);
      bg_black[cursor_position] = false;
    }
    else{
      if (bg_black[0] == false){
        bg_black[0] = true;
        display.fillRect(2, (0*gap)+2, col_width-3, gap-3, SSD1306_BLACK);
        display.fillRect(col_width+2, (0*gap)+2, display.width()-col_width-4, gap-3, SSD1306_BLACK);
      }
      if (bg_black[1] == false){
        bg_black[1] = true;
        display.fillRect(2, (1*gap)+2, col_width-3, gap-3, SSD1306_BLACK);
        display.fillRect(col_width+2, (1*gap)+2, display.width()-col_width-4, gap-3, SSD1306_BLACK);
      }
      if (bg_black[2] == false){
        bg_black[2] = true;
        display.fillRect(2, (2*gap)+2, col_width-3, gap-3, SSD1306_BLACK);
        display.fillRect(col_width+2, (2*gap)+2, display.width()-col_width-4, gap-3, SSD1306_BLACK);
      }
    }
  }

  void show_content(){
    String label[3] = {"MOD", "VEL", "INC"};
    String values[3] = {"LIN", "50mm/s", "0.5mm"};

    if (nav_button_pressed == true){
        display.clearDisplay();
        display.display();
      }

    if ((mode_lin == true) and (mode_rot == false)){
      values[0] = "LIN";
    }
    else if ((mode_lin == false) and (mode_rot == true)){
      values[0] = "ROT";
    }
    values[1] = String(velocity);
    values[2] = String(increment);
    int gap = display.height()/3;
    display.setTextSize(2);             // Normal 1:1 pixel scale
    
    draw_background(true);
    draw_lines();
    
    for (int i=0; i<3; i++){
      if (i == cursor_position){
        display.setTextColor(SSD1306_BLACK);
      }
      else{
        display.setTextColor(SSD1306_WHITE);
      }
      display.setCursor(4,gap*i+4);
      display.println(label[i]);
      display.setCursor(5+col_width,gap*i+4);
      display.println(values[i]);
      if (nav_button_pressed == true){
        display.display();
        nav_button_pressed = false;
      }
    }
  }

private:
};

GUI gui;
//Motion mbtns;

  Button btn1(23, 500, "btn1 - 23");
  Button btn2(25, 500, "btn2 - 25");
  Button btn3(27, 500, "btn3 - 27");
  Button btn4(29, 500, "btn4 - 29");
  Button btn5(31, 500, "btn5 - 31");


//ros::NodeHandle nh;
//std_msgs::Bool pushed_msg;
//ros::Publisher pub_button("pushed", &pushed_msg);

bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;

void setup() {
  Serial.begin(9600);
  gui.init();  
  //nh.initNode();
  //nh.advertise(pub_button);

  //last_reading =! btn5.update();

}





void loop() {
  //mbtns.update_buttons();
  gui.upt_encoder();
 //gui.update_buttons();
  //gui.show_content();  

  btn1.update();
  btn2.update();
  btn3.update();
  btn4.update();
  btn5.update();

  /*bool reading =! btn5.update();

  if(last_reading != reading){
    last_debounce_time = millis();
    published = false;
  }

  if (!published && (millis() - last_debounce_time) > debounce_delay){
    pushed_msg.data = reading;
    pub_button.publish(&pushed_msg);
    published = true;
  }

  last_reading = reading;

  nh.spinOnce();
*/
}
