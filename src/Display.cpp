#include <Display.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define MENU_COUNT 4
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

int state;
double pressure, targetPressure, P, I, D;
double vals[MENU_COUNT] = {};
String valNames[MENU_COUNT] = {"Pressure", "P", "I", "D"};
Adafruit_SSD1306 screen;

Display::Display() {

}

void Display::init(){
    screen = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
    
    if(!screen.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed")); 
        for(;;); // Don't proceed, loop forever
    }

    testScreen();

    state = 0;
    pressure = 0;
}

void Display::increment(int count) {
    //vals[state] += count;
    switch (state)
    {
    case 0:
        targetPressure += count;
        break;
    case 1:
        P += count;
        break;
    case 2:
        I += count;
        break;
    case 3:
        D += count;
        break;
    }
    drawMenu();
}

void Display::button(int count) {
    state += count;
    state = state%MENU_COUNT; // loop
    drawMenu();
}

void Display::drawMenu() {
    screen.clearDisplay();
    screen.setCursor(0, 16);   // Start position

    screen.print("Parameter: ");
    screen.println(valNames[state]);
    
    screen.print("Setting: ");
    //screen.println(vals[state]);
    switch (state)
    {
    case 0:
        screen.println(targetPressure);
        break;
    case 1:
        screen.println(P);
        break;
    case 2:
        screen.println(I);
        break;
    case 3:
        screen.println(D);
        break;
    }

    screen.print("pressure: ");
    screen.println(pressure);
    
    screen.display();
}

void Display::testScreen() {
    screen.display(); // show logo
    delay(500); // pause for serial to initialize and MAX chips to stabilize
    screen.clearDisplay();

    Serial.println("EspressOS test");

    screen.setTextSize(1);      // Normal 1:1 pixel scale
    screen.setTextColor(SSD1306_WHITE); // Draw white text
    screen.setCursor(32, 32);   // Start position
    screen.cp437(true);         // Use full 256 char 'Code Page 437' font
    
    screen.println("SSD1306 Test");
    screen.display();
    delay(2000); // Pause for 2 seconds

    screen.clearDisplay();
    screen.display();
}