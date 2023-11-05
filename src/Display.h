#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define MENU_COUNT 4

class Display {
  
  public:
    int state;
    double pressure, targetPressure, P, I, D;
    int vals[MENU_COUNT];
    Adafruit_SSD1306 screen;
  
    Display();

    void init();
    
    void increment(int count);
    
    void button(int count);

    void drawMenu();

    void testScreen();
};