#include <Wire.h>
//Mega2560 pin 20 (SDA), pin 21 (SCL)
//SPI pin
// MOSI 51
//MISO 50
//SCK 52
//SS 53
#include <Adafruit_SSD1331.h>
///////////////////////////////////////////////////////
//OLED
int sclk = 52; //brown--- connect this to the display module CLK pin (Serial Clock)
int mosi = 51; //orange--- connect this to the display module DIN pin (Serial Data)
int rst  = 49; //yellow--- connect this to the display module RES pin (Reset)
int dc   = 50; //green--- connect this to the display module D/C  pin (Data or Command)
int cs   = 48; //blue--- connect this to the display module CS  pin (Chip Select)
// Color definitions

#define BLACK          0x0000
#define BLUE            0x0006
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
#define BACKGROUND      0x0000
Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);


void setup()
{
  ///////////////////////////
  //OLED
  display.begin();
  display.fillScreen(BLACK);
  tftPrintTest();
  delay(2000);
}

/////////////////////////////
//Main Loop
void loop()
{
}

void tftPrintTest() {
  display.fillScreen(BLACK);
  display.setCursor(15, 5);
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.println("Demo");
  display.setCursor(0, 25);
  display.setTextColor(CYAN);
  display.setTextSize(1);
  display.println("OLED");
  display.setCursor(15, 50);
  display.setTextColor(YELLOW);
  display.setTextSize(2);
  display.println("Start");
  delay(1500);
  display.setTextColor(WHITE);
}
