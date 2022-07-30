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




Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int col_width = 42;
int cursor_position = 0;
float increment = 0.1;
int velocity = 50;
bool mode_lin = true;
bool mode_rot = false;

Timer ton;
int state=0;
bool entry = true;

bool nav_left_pressed = false;
bool nav_right_pressed = false;
bool nav_up_pressed = false;
bool nav_down_pressed = false;
bool nav_center_pressed = false;
bool nav_abort_pressed = false;
bool nav_enter_pressed = false;

int debounce_time = 50;
ButtonDebounce btn_nav_left(nav_left, debounce_time);
ButtonDebounce btn_nav_right(nav_right, debounce_time);
ButtonDebounce btn_nav_up(nav_up, debounce_time);
ButtonDebounce btn_nav_down(nav_down, debounce_time);
ButtonDebounce btn_nav_center(nav_center, debounce_time);
ButtonDebounce btn_nav_abort(nav_abort, debounce_time);
ButtonDebounce btn_nav_enter(nav_enter, debounce_time);
  


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(led_onboard, OUTPUT);
  
  Serial.println("INIT");

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  display.display();
  display.clearDisplay();
  update_display(1);
  
}

void loop() {
  btn_nav_left.update();
  btn_nav_right.update();
  btn_nav_up.update();
  btn_nav_down.update();
  btn_nav_center.update();
  btn_nav_abort.update();
  btn_nav_enter.update();
  

  switch (state){
    case 0:
    if (entry == true){
      entry = false; 
      draw_info_line(true);
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
      draw_info_line(false);
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
  
  
  // put your main code here, to run repeatedly:
  if ((btn_nav_left.state() == HIGH) and (nav_left_pressed == false)){
    nav_left_pressed = true;
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
  else if (btn_nav_left.state() == LOW){
    nav_left_pressed = false;
  }
  if ((btn_nav_right.state() == HIGH) and (nav_right_pressed == false)){
    nav_right_pressed = true;
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
  else if (btn_nav_right.state() == LOW){
    nav_right_pressed = false;
  }
  if ((btn_nav_up.state() == HIGH) and (nav_up_pressed == false)){
    nav_up_pressed = true;
    Serial.println("up");
    if (cursor_position > 0){
      cursor_position--;
    }
    else{
      cursor_position=0;
    }
  }
  else if (btn_nav_up.state() == LOW){
    nav_up_pressed = false;
  }
  if ((btn_nav_down.state() == HIGH) and (nav_down_pressed == false)){
    nav_down_pressed = true;
    Serial.println("down");
    if (cursor_position < 2){
      cursor_position++;
    }
    else{
      cursor_position=2;
    }
  }
  else if (btn_nav_down.state() == LOW){
    nav_down_pressed = false;
  }
  if ((btn_nav_center.state() == HIGH) and (nav_center_pressed == false)){
    nav_center_pressed = true;
    ;
  }
  else if (btn_nav_center.state() == LOW){
    nav_center_pressed = false;
  }
  if ((btn_nav_abort.state() == HIGH) and (nav_abort_pressed == false)){
    nav_abort_pressed = true;
    ;
  }
  else if (btn_nav_abort.state() == LOW){
    nav_abort_pressed = false;
  }
  if ((btn_nav_enter.state() == HIGH) and (nav_enter_pressed == false)){
    nav_enter_pressed = true;
    ;
  }
  else if (btn_nav_enter.state() == LOW){
    nav_enter_pressed = false;
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
  }
  else if ((mode_lin == false) and (mode_rot == true)){
    values[0] = "ROT";
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
