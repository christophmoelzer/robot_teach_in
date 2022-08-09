/*
Author: Christoph Moelzer
Mail: christoph.moelzer@outlook.com
*/

// rosrun rosserial_arduino serial_node.py /dev/ttyACM1 oder ttyACM0

/*
* Color state:
* red     -> error
* yellow  -> hand guided
* blue    -> linear motion
* green   -> rotation motion
* white   -> automatic
*/



#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ButtonDebounce.h>
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>

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

class Button{
public:

//state
bool input;
bool last_input;
bool output;
uint8_t _pin;
unsigned long t_last_change;
int _t_debounce;
bool published;
bool debounced;
String label;

Button(){
  ;
}
Button(uint8_t pin){
  _pin = pin;
  _t_debounce = 50;
  output = false;
  pinMode(_pin, INPUT_PULLUP);
  last_input =! digitalRead(_pin);
}

Button(uint8_t pin, int t_debounce, String lb){
  _pin = pin;
  label = lb;
  _t_debounce = t_debounce;
  output = false;
  pinMode(_pin, INPUT_PULLUP);
  last_input =! digitalRead(_pin);
  debounced = false;
}


bool update(){
  // Statusänderung prüfen. Wenn Änderung dann Zeit zurücksetzen
  // Wenn Zeit überschritten dann Ausgang setzen
  input =! digitalRead(_pin);

  if(last_input != input){
    t_last_change = millis();
    published = false;
    debounced = false;
  }

  if(!published && (millis() - t_last_change) > _t_debounce){
    output = input;
    debounced = true;
  }

  last_input = input;
  
  return output;
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
  int debounce_time;
  bool btn_pressed;
  bool button_array[32];
  bool published;
  bool debounced;
  uint32_t coded_motion_buttons;

  Button btn_grp_1_x_negative;
  Button btn_grp_1_x_positive;
  Button btn_grp_1_y_negative;
  //Button btn_grp_1_y_positive;
  Button btn_grp_1_z_negative;
  Button btn_grp_1_z_positive;
  LED led_grp_1;
  bool grp_1_active;
  
  //Button btn_grp_2_x_negative;
  Button btn_grp_2_x_positive;
  Button btn_grp_2_y_negative;
  Button btn_grp_2_y_positive;
  Button btn_grp_2_z_negative;
  Button btn_grp_2_z_positive;
  LED led_grp_2;
  bool grp_2_active;

  Button btn_grp_3_x_negative;
  //Button btn_grp_3_x_positive;
  Button btn_grp_3_y_negative;
  Button btn_grp_3_y_positive;
  Button btn_grp_3_z_negative;
  Button btn_grp_3_z_positive;
  LED led_grp_3;
  bool grp_3_active;
  
  Button btn_grp_4_x_negative;
  Button btn_grp_4_x_positive;
  //Button btn_grp_4_y_negative;
  Button btn_grp_4_y_positive;
  Button btn_grp_4_z_negative;
  Button btn_grp_4_z_positive;
  LED led_grp_4;
  bool grp_4_active;


  Motion(){
    btn_pressed = false;
    debounce_time = 50;
    coded_motion_buttons = 0;
    published = false;
    grp_1_active = false;
    grp_2_active = false;
    grp_3_active = false;
    grp_4_active = false;
    
    btn_grp_1_x_negative = Button(52, debounce_time, "1 - x neg");
    btn_grp_1_x_positive = Button(48, debounce_time, "1 - x pos");
    btn_grp_1_y_negative = Button(46, debounce_time, "1 - y neg");
    btn_grp_1_z_negative = Button(53, debounce_time, "1 - z neg");
    btn_grp_1_z_positive = Button(50, debounce_time, "1 - z pos");
    led_grp_1 = LED(40, 42, 44);

    btn_grp_2_x_positive = Button(51, debounce_time, "2 - x pos");
    btn_grp_2_y_negative = Button(45, debounce_time, "2 - y neg");
    btn_grp_2_y_positive = Button(49, debounce_time, "2 - y pos");
    btn_grp_2_z_negative = Button(43, debounce_time, "2 - z neg");
    btn_grp_2_z_positive = Button(47, debounce_time, "2 - z pos");
    led_grp_2 = LED(34, 36, 38);

    btn_grp_3_x_negative = Button(41, debounce_time, "3 - x neg");
    btn_grp_3_y_negative = Button(39, debounce_time, "3 - y neg");
    btn_grp_3_y_positive = Button(35, debounce_time, "3 - y pos");
    btn_grp_3_z_negative = Button(33, debounce_time, "3 - z neg");
    btn_grp_3_z_positive = Button(37, debounce_time, "3 - z pos");
    led_grp_3 = LED(28, 30, 32);

    btn_grp_4_x_positive = Button(25, debounce_time, "4 - x pos");
    btn_grp_4_x_negative = Button(29, debounce_time, "4 - x neg");
    btn_grp_4_y_positive = Button(31, debounce_time, "4 - y pos");
    btn_grp_4_z_negative = Button(23, debounce_time, "4 - z neg");
    btn_grp_4_z_positive = Button(27, debounce_time, "4 - z pos");
    led_grp_4 = LED(22, 24, 26);
  }  

  void init_button_array(){
    for (int i=0; i<sizeof(button_array); i++){
      button_array[i] = false;
    }
  }

  void update(){
    btn_pressed = false;

    btn_grp_1_x_negative.update();
    btn_grp_1_x_positive.update();
    btn_grp_1_y_negative.update();
    //btn_grp_1_y_positive.update();
    btn_grp_1_z_negative.update();
    btn_grp_1_z_positive.update();
    
    //btn_grp_2_x_negative.update();
    btn_grp_2_x_positive.update();
    btn_grp_2_y_negative.update();
    btn_grp_2_y_positive.update();
    btn_grp_2_z_negative.update();
    btn_grp_2_z_positive.update();
    
    btn_grp_3_x_negative.update();
    //btn_grp_3_x_positive.update();
    btn_grp_3_y_negative.update();
    btn_grp_3_y_positive.update();
    btn_grp_3_z_negative.update();
    btn_grp_3_z_positive.update();
    
    btn_grp_4_x_negative.update();
    btn_grp_4_x_positive.update();
    //btn_grp_4_y_negative.update();
    btn_grp_4_y_positive.update();
    btn_grp_4_z_negative.update();
    btn_grp_4_z_positive.update();
    
    if (btn_grp_1_x_negative.debounced && !btn_grp_1_x_negative.published){
      btn_grp_1_x_negative.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_1_x_positive.debounced && !btn_grp_1_x_positive.published) {
      btn_grp_1_x_positive.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_1_y_negative.debounced && !btn_grp_1_y_negative.published) {
      btn_grp_1_y_negative.published = true;
      published = false;
      debounced = true;
    }
    //(btn_grp_1_y_positive.debounced && !btn_grp_1_y_positive.published) {
          //btn_grp_1_y_positive.published = true;
    if (btn_grp_1_z_negative.debounced && !btn_grp_1_z_negative.published) {
      btn_grp_1_z_negative.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_1_z_positive.debounced && !btn_grp_1_z_positive.published) {
      btn_grp_1_z_positive.published = true;
      published = false;
      debounced = true;
    }
    //if (btn_grp_2_x_negative.debounced && !btn_grp_2_x_negative.published) {
          //btn_grp_2_x_negative.published = true;
    if (btn_grp_2_x_positive.debounced && !btn_grp_2_x_positive.published) {
      btn_grp_2_x_positive.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_2_y_negative.debounced && !btn_grp_2_y_negative.published) {
      btn_grp_2_y_negative.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_2_y_positive.debounced && !btn_grp_2_y_positive.published) {
      btn_grp_2_y_positive.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_2_z_negative.debounced && !btn_grp_2_z_negative.published) {
      btn_grp_2_z_negative.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_2_z_positive.debounced && !btn_grp_2_z_positive.published) {
      btn_grp_2_z_positive.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_3_x_negative.debounced && !btn_grp_3_x_negative.published) {
      btn_grp_3_x_negative.published = true;
      published = false;
      debounced = true;
    }
    //if (btn_grp_3_x_positive.debounced && !btn_grp_3_x_positive.published) {
          //btn_grp_3_x_positive.published = true;
    if (btn_grp_3_y_negative.debounced && !btn_grp_3_y_negative.published) {
      btn_grp_3_y_negative.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_3_y_positive.debounced && !btn_grp_3_y_positive.published) {
      btn_grp_3_y_positive.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_3_z_negative.debounced && !btn_grp_3_z_negative.published) {
      btn_grp_3_z_negative.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_3_z_positive.debounced && !btn_grp_3_z_positive.published) {
      btn_grp_3_z_positive.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_4_x_negative.debounced && !btn_grp_4_x_negative.published) {
      btn_grp_4_x_negative.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_4_x_positive.debounced && !btn_grp_4_x_positive.published) {
      btn_grp_4_x_positive.published = true;
      published = false;
      debounced = true;
    }
    //if (btn_grp_4_y_negative.debounced && !btn_grp_4_y_negative.published) {
          //btn_grp_4_y_negative.published = true;
    if (btn_grp_4_y_positive.debounced && !btn_grp_4_y_positive.published) {
      btn_grp_4_y_positive.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_4_z_negative.debounced && !btn_grp_4_z_negative.published) {
      btn_grp_4_z_negative.published = true;
      published = false;
      debounced = true;
    }
    if (btn_grp_4_z_positive.debounced && !btn_grp_4_z_positive.published) {
      btn_grp_4_z_positive.published = true;
      published = false;
      debounced = true;
    }

    if(btn_grp_1_x_negative.output or 
       btn_grp_1_x_positive.output or 
       btn_grp_1_y_negative.output or 
       //btn_grp_1_y_positive.output or 
       btn_grp_1_z_negative.output or 
       btn_grp_1_z_positive.output or 
       //btn_grp_2_x_negative.output or 
       btn_grp_2_x_positive.output or 
       btn_grp_2_y_negative.output or 
       btn_grp_2_y_positive.output or 
       btn_grp_2_z_negative.output or 
       btn_grp_2_z_positive.output or 
       btn_grp_3_x_negative.output or 
       //btn_grp_3_x_positive.output or 
       btn_grp_3_y_negative.output or 
       btn_grp_3_y_positive.output or 
       btn_grp_3_z_negative.output or 
       btn_grp_3_z_positive.output or 
       btn_grp_4_x_negative.output or 
       btn_grp_4_x_positive.output or 
       //btn_grp_4_y_negative.output or 
       btn_grp_4_y_positive.output or 
       btn_grp_4_z_negative.output or 
       btn_grp_4_z_positive.output){
        btn_pressed = true;
       }
        

    coded_motion_buttons = 0;
    init_button_array();

    button_array[0] = btn_grp_1_x_negative.output; // 2^0
    button_array[1] = btn_grp_1_x_positive.output; // 2^1
    button_array[2] = btn_grp_1_y_negative.output; // 2^2
    button_array[3] = false;//btn_grp_1_y_positive.output; // 2^3
    button_array[4] = btn_grp_1_z_negative.output; // 2^4
    button_array[5] = btn_grp_1_z_positive.output; // 2^5
    
    button_array[6] = false;//btn_grp_2_x_negative.output; // 2^6
    button_array[7] = btn_grp_2_x_positive.output; // 2^7
    button_array[8] = btn_grp_2_y_negative.output; // 2^8
    button_array[9] = btn_grp_2_y_positive.output; // 2^9
    button_array[10] = btn_grp_2_z_negative.output; // 2^10
    button_array[11] = btn_grp_2_z_positive.output; // 2^11
    
    button_array[12] = btn_grp_3_x_negative.output; // 2^12
    button_array[13] = false;//btn_grp_3_x_positive.output; // 2^13
    button_array[14] = btn_grp_3_y_negative.output; // 2^14
    button_array[15] = btn_grp_3_y_positive.output; // 2^15
    button_array[16] = btn_grp_3_z_negative.output; // 2^16
    button_array[17] = btn_grp_3_z_positive.output; // 2^17
    
    button_array[18] = btn_grp_4_x_negative.output; // 2^18
    button_array[19] = btn_grp_4_x_positive.output; // 2^19
    button_array[20] = false;//btn_grp_4_y_negative.output; // 2^20
    button_array[21] = btn_grp_4_y_positive.output; // 2^21
    button_array[22] = btn_grp_4_z_negative.output; // 2^22
    button_array[23] = btn_grp_4_z_positive.output; // 2^23


    grp_1_active = false;
    grp_2_active = false;
    grp_3_active = false;
    grp_4_active = false;
    for(int i=0; i<sizeof(button_array); i++){
      if (button_array[i] == true){
          coded_motion_buttons+=bit(i);      
      }
    }

    for(int i=0; i<6; i++){
      if(button_array[i]){
        grp_1_active = true;  
      }
      if(button_array[i+6]){
        grp_2_active = true;  
      }
      if(button_array[i+12]){
        grp_3_active = true;  
      }
      if(button_array[i+18]){
        grp_4_active = true;  
      }
    }
      
    if (grp_1_active){
      led_grp_1.blue();
    }else{
      led_grp_1.white();
    }
    if (grp_2_active){
      led_grp_2.blue();
    }else{
      led_grp_2.white();
    }
    if (grp_3_active){
      led_grp_3.blue();
    }else{
      led_grp_3.white();
    }
    if (grp_4_active){
      led_grp_4.blue();
    }else{
      led_grp_4.white();
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
}

void init(){
  display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for(;;); // Don't proceed, loop forever
  }
  

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
    //Serial.println(new_value);
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
  btn_nav_left.update();
  btn_nav_right.update();
  btn_nav_up.update();
  btn_nav_down.update();
  btn_nav_center.update();
  btn_nav_abort.update();
  btn_nav_enter.update();

/*
  if ((btn_nav_left.update() == LOW) and (btn_nav_left.pressed == false)){
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
  else if (btn_nav_left.update() == HIGH){
    btn_nav_left.pressed = false;
    led_3.black();
  }


  if ((btn_nav_right.update() == LOW) and (btn_nav_right.pressed == false)){
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
  else if (btn_nav_right.update() == HIGH){
    btn_nav_right.pressed = false;
    led_2.black();
  }


  if ((btn_nav_up.update() == LOW) and (btn_nav_up.pressed == false)){
    btn_nav_up.pressed = true;
    led_4.white();
    if (cursor_position > 0){
      cursor_position--;
    }
    else{
      cursor_position=0;
    }
  }
  else if (btn_nav_up.update() == HIGH){
    btn_nav_up.pressed = false;
    led_4.black();
  }


  if ((btn_nav_down.update() == LOW) and (btn_nav_down.pressed == false)){
    btn_nav_down.pressed = true;
    led_1.white();
    if (cursor_position < 2){
      cursor_position++;
    }
    else{
      cursor_position=2;
    }
  }
  else if (btn_nav_down.update() == HIGH){
    btn_nav_down.pressed = false;
    led_1.black();
  }


  if ((btn_nav_center.update() == LOW) and (btn_nav_center.pressed == false)){
    btn_nav_center.pressed = true;
  }
  else if (btn_nav_center.update() == HIGH){
    btn_nav_center.pressed = false;
  }


  if ((btn_nav_abort.update() == LOW) and (btn_nav_abort.pressed == false)){
    btn_nav_abort.pressed = true;
  }
  else if (btn_nav_abort.update() == HIGH){
    btn_nav_abort.pressed = false;
  }


  if ((btn_nav_enter.update() == LOW) and (btn_nav_enter.pressed == false)){
    btn_nav_enter.pressed = true;
  }
  else if (btn_nav_enter.update() == HIGH){
    btn_nav_enter.pressed = false;
  }

  if (btn_nav_left.pressed or btn_nav_right.pressed or btn_nav_up.pressed or btn_nav_down.pressed or btn_nav_center.pressed or btn_nav_abort.pressed or btn_nav_enter.pressed){
    nav_button_pressed = true;
  }
  else{
    ;
  }*/
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
Motion motion;
ros::NodeHandle nh;
std_msgs::UInt32 pushed_msg;
ros::Publisher pub_button("pushed", &pushed_msg);

void setup() {
  //Serial.begin(9600);
  gui.init();  
  nh.initNode();
  nh.advertise(pub_button);
}

void loop() {
  gui.upt_encoder();
  //gui.update_buttons();
  //gui.show_content();  

  motion.update();
  if(motion.debounced && !motion.published){
    pushed_msg.data = motion.coded_motion_buttons;
    pub_button.publish(&pushed_msg);
    motion.published = true;
  }


  nh.spinOnce();

}


