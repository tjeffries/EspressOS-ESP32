#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define MENU_COUNT 5

class Display {
  
  public:
    int state;
    double pressure;
    int counts[MENU_COUNT];
    Adafruit_SSD1306 screen;
  
    Display();

    void init();
    
    void increment(int count);
    
    void button(int count);

    void drawMenu();

    void testScreen();
};