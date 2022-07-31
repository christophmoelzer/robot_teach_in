/*
Author: Christoph Moelzer
Mail: christoph.moelzer@outlook.com
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ButtonDebounce.h>


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

class Button{
public:
ButtonDebounce db;
bool pressed;
int debounce_time;
Button(){
  ;
}
Button(int pin){
  debounce_time = 50;
  pressed = false;
  db = ButtonDebounce(pin, debounce_time);
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

  

  private:
};

class AnalogLED{
  public:
  void set(int val){
    analogWrite(26, 50);
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
AnalogLED led;
int cursor_position;
int col_width;
float increment;
int velocity ;
bool mode_lin;
bool mode_rot;

GUI(){
  display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
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
  /*led_1.green();
  led_2.red();
  led_3.red();
  led_4.blue();*/
  cursor_position = 0;
  col_width = 42;
  increment = 0.1;
  velocity = 50;
  mode_lin = true;
  mode_rot = false;
  
}

void update_buttons(){
  btn_nav_left.db.update();
  btn_nav_right.db.update();
  btn_nav_up.db.update();
  btn_nav_down.db.update();
  btn_nav_center.db.update();
  btn_nav_abort.db.update();
  btn_nav_enter.db.update();


    if ((btn_nav_left.db.state() == HIGH) and (btn_nav_left.pressed == false)){
    btn_nav_left.pressed = true;
    display.clearDisplay();
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
  else if (btn_nav_left.db.state() == LOW){
    btn_nav_left.pressed = false;
  }
  if ((btn_nav_right.db.state() == HIGH) and (btn_nav_right.pressed == false)){
    btn_nav_right.pressed = true;
    display.clearDisplay();
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
  else if (btn_nav_right.db.state() == LOW){
    btn_nav_right.pressed = false;
  }
  if ((btn_nav_up.db.state() == HIGH) and (btn_nav_up.pressed == false)){
    btn_nav_up.pressed = true;
    Serial.println("up");
    if (cursor_position > 0){
      cursor_position--;
    }
    else{
      cursor_position=0;
    }
  }
  else if (btn_nav_up.db.state() == LOW){
    btn_nav_up.pressed = false;
  }
  if ((btn_nav_down.db.state() == HIGH) and (btn_nav_down.pressed == false)){
    btn_nav_down.pressed = true;
    Serial.println("down");
    if (cursor_position < 2){
      cursor_position++;
    }
    else{
      cursor_position=2;
    }
  }
  else if (btn_nav_down.db.state() == LOW){
    btn_nav_down.pressed = false;
  }
  if ((btn_nav_center.db.state() == HIGH) and (btn_nav_center.pressed == false)){
    btn_nav_center.pressed = true;
    ;
  }
  else if (btn_nav_center.db.state() == LOW){
    btn_nav_center.pressed = false;
  }
  if ((btn_nav_abort.db.state() == HIGH) and (btn_nav_abort.pressed == false)){
    btn_nav_abort.pressed = true;
    ;
  }
  else if (btn_nav_abort.db.state() == LOW){
    btn_nav_abort.pressed = false;
  }
  if ((btn_nav_enter.db.state() == HIGH) and (btn_nav_enter.pressed == false)){
    btn_nav_enter.pressed = true;
    ;
  }
  else if (btn_nav_enter.db.state() == LOW){
    btn_nav_enter.pressed = false;
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
    //display.display();
  }

  void draw_info_line(bool invert){
    String label[3] = {"MOD", "VEL", "INC"};
    String values[3] = {"LIN", "50mm/s", "0.5mm"};
    if ((mode_lin == true) and (mode_rot == false)){
      values[0] = "LIN";
      led_1.white();
    }
    else if ((mode_lin == false) and (mode_rot == true)){
      values[0] = "ROT";
      led_1.blue();
    }
    values[1] = String(velocity);
    values[2] = String(increment);
    int gap = display.height()/3;
    display.setTextSize(2);             // Normal 1:1 pixel scale
    
    draw_background(invert);
    draw_lines();
    
    for (int i=0; i<3; i++){
      if (i == cursor_position){
        if (invert == true){
          display.setTextColor(SSD1306_BLACK);
        }
        else{
          display.setTextColor(SSD1306_WHITE);
        }
      }
      else{
        display.setTextColor(SSD1306_WHITE);
      }
      display.setCursor(4,gap*i+4);
      display.println(label[i]);
      display.setCursor(5+col_width,gap*i+4);
      display.println(values[i]);
      display.display();
    }
    
    
  }

  void draw_menu(void) {
    int gap = display.height()/3;
    display.clearDisplay();
    draw_lines();
    draw_info_line(true);
    display.display();
  }

  void update_display(int x){
    draw_menu();
  }


private:
};

GUI gui;
Timer ton;

int state=0;
bool entry = true;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(led_onboard, OUTPUT);
  
  
  Serial.println("INIT");
  
  if(!gui.display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  gui.display.display();
  gui.display.clearDisplay();
  gui.update_display(1);
  
}

void loop() {
  gui.update_buttons();
  
  switch (state){
    case 0:
    if (entry == true){
      entry = false; 
      gui.draw_info_line(true);
    }
    ton.in = true;
    ton.pt = 400;
    
    
    if (ton.q == true){
      ton.in = false;
      state = 1;
      entry = true;
    }
    break;

    case 1:
    if (entry == true){
      entry = false;
      gui.draw_info_line(false);
    }
    ton.in = true;
    ton.pt = 120;

    if (ton.q == true){
      ton.in = false;
      state = 0;
      entry = true;
    }
    break;
  }
  ton.run();
  
}

