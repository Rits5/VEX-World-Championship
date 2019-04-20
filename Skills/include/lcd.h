#ifndef LCD_H_
#define LCD_H_

//LCD variables

  #define RED 1
  #define BLUE 0

  extern int auton;
  extern int red;
  extern int blue;
  extern int max_auton_limit;

  void auton_limiter(int counter, bool colour);
  void red_selector(int counter);
  void blue_selector(int counter);
  void on_right_button();
  void on_left_button();  


#endif
