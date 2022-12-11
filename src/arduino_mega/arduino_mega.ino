/*
Author: Christoph Moelzer
Mail: christoph.moelzer@outlook.com
*/

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
#include <Encoder.h>


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
    digitalWrite(_pin_r, HIGH);
    digitalWrite(_pin_g, HIGH);
    digitalWrite(_pin_b, LOW);
  }

   void light_blue(){
    digitalWrite(_pin_r, LOW);
    digitalWrite(_pin_g, HIGH);
    digitalWrite(_pin_b, HIGH);
  }

  private:
};

class Button{
  public:

  //state
  bool input;
  bool last_input;
  bool output;
  bool aux;
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

  bool mode_lin;
  bool mode_lead_through;
  bool button_lead_through;

  uint8_t coded_buttons[4];

  uint32_t mask[4];
  

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
    published = false;
    grp_1_active = false;
    grp_2_active = false;
    grp_3_active = false;
    grp_4_active = false;
    mode_lin=true;
    mode_lead_through = false;

    mask[0] = 0x000F;
    mask[1] = 0x00F0;
    mask[2] = 0x0F00;
    mask[3] = 0xF000;
    
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

    button_array[24] = mode_lin;
    button_array[25] =! mode_lin;

    button_array[29] = button_lead_through;

    grp_1_active = false;
    grp_2_active = false;
    grp_3_active = false;
    grp_4_active = false;

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
  bool mode_lead_through;
  bool changed;
  WaitMs wait;
  bool debounced;
  bool published;

  long pos_encoder;
  int16_t angle_degrees;
  float angle_radians;

  uint8_t angle_radians_serial[4];

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
    mode_lead_through = false;
    pos_encoder = -999;
    debounced=false;
    published=false;
    
  }

  void convert_angle_to_bytes(float rad){
    uint32_t no_comma_angle = (uint32_t)(rad*1000);
    uint32_t mask[4];
    mask[0] = 0x000F;
    mask[1] = 0x00F0;
    mask[2] = 0x0F00;
    mask[3] = 0xF000;

    for(int i=0; i<4; i++){    
      angle_radians_serial[i] = 0;
      angle_radians_serial[i] = mask[i] & no_comma_angle;
    }
  
  }

  void upt_encoder(){
    long new_value;
    changed = false;
    new_value = encoder.read();
    if (new_value != pos_encoder){
      //Serial.println(new_value);
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(96-15, 32-7);
      pos_encoder = new_value;
      new_value = new_value%624;
      if(new_value<0) new_value+=624;
      angle_degrees = (new_value*360/624);
      angle_radians = new_value*2*PI/624;
      convert_angle_to_bytes(angle_radians);
      calc_arrow(angle_radians);
      display.println(angle_degrees);
      changed = true;
      display.display();
    }
    
  }

  void calc_arrow(float angle){
    int length = 31;

    Point null(128/2+64/2, 64/2);
    Point vector_start((length*3/4)*cos(angle), (length*3/4)*sin(angle));
    Point vector_end(length*cos(angle), length*sin(angle));
    Point start(null.x+vector_start.x, null.y+vector_start.y);
    Point end(null.x+vector_end.x, null.y+vector_end.y);
    Line line(start,end);  
    display.drawLine(line.start.x, line.start.y, line.end.x, line.end.y, SSD1306_WHITE);
    display.drawCircle(null.x, null.y, length, SSD1306_WHITE);
  }


  void update_buttons(){
    btn_nav_left.update();
    btn_nav_right.update();
    btn_nav_up.update();
    btn_nav_down.update();
    btn_nav_center.update();
    btn_nav_abort.update();
    btn_nav_enter.update();

    if (btn_nav_center.output and not(btn_nav_center.aux)){
      btn_nav_center.aux = true;
      mode_lin =! mode_lin;
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(5, 5);
      if (mode_lin){
        display.println("LIN");
      }else{
        display.println("ROT");
      }
      display.display();
    }
    if (not(btn_nav_center.output)){
      btn_nav_center.aux = false;
    }

    if (btn_nav_abort.output and not(btn_nav_abort.aux)){
      btn_nav_abort.aux = true;
      mode_lead_through =! mode_lead_through;
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(5, 25);
      if (mode_lead_through){
        display.println("LEAD_THROUGH");
      }else{
        display.println("NORMAL MODE");
      }
      display.display();
    }
    if (not(btn_nav_abort.output)){
      btn_nav_abort.aux = false;
    }

    if (btn_nav_abort.debounced && !btn_nav_abort.published) {
        btn_nav_abort.published = true;
        published = false;
        debounced = true;
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

      if ((mode_lin == true)){
        values[0] = "LIN";
      }
      else if ((mode_lin == false)){
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
WaitMs startDelay;

char datenpaket;
int serial_counter=0;
bool aux_changed = false;
bool aux_last = false;

void setup() {
  Serial.begin(115200);
  gui.init();  
  startDelay.ms(3000);
}

void loop() {
  
  gui.upt_encoder();
  gui.update_buttons();
  //gui.show_content();  
  motion.mode_lin = gui.mode_lin;
  motion.mode_lead_through = gui.mode_lead_through;
  motion.button_lead_through = gui.btn_nav_abort.output and gui.btn_nav_abort.debounced;
  motion.update();


  
  if (motion.btn_pressed != aux_last){
    if(motion.btn_pressed == false){
      Serial.write(0);
      Serial.write(0);
      Serial.write(0);
      Serial.write(0);
      gui.display.clearDisplay();
      gui.display.setTextSize(2);
      gui.display.setTextColor(SSD1306_WHITE);
      gui.display.setCursor(5, 5);
      gui.display.println("STOP");
      gui.display.display();
    }
    else{

      for(int x=0; x<4; x++){
        motion.coded_buttons[x]=0;
        for(int i=(x*8); i<((x*8)+8); i++){
          if(motion.button_array[i]){
            motion.coded_buttons[x] += (uint8_t)bit(i-(x*8));
          }
        }
      }

      Serial.write(motion.coded_buttons[0]);  // 1
      Serial.write(motion.coded_buttons[1]);  // 1
      Serial.write(motion.coded_buttons[2]);  // 1
      Serial.write(motion.coded_buttons[3]);  // 1

      gui.display.clearDisplay();
      gui.display.setTextSize(2);
      gui.display.setTextColor(SSD1306_WHITE);
      gui.display.setCursor(5, 5);
      gui.display.println(motion.coded_buttons[0]);
      gui.display.display();

    }

    //Serial.write(gui.angle_radians_serial[0]);
    //Serial.write(gui.angle_radians_serial[1]);
    //Serial.write(gui.angle_radians_serial[2]);
    //Serial.write(gui.angle_radians_serial[3]);
  }
  aux_last = motion.btn_pressed;

  
  

  if(gui.changed){
    ;
    //angle_msg.data = gui.angle_radians;
    //pub_angle.publish(&angle_msg);
  }

}
