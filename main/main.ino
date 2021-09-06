//Gerekli kütüphaneler ve tüm proje için göz atın: https://github.com/furkanbakkal/DLP-Printer-v1.0

#include <Adafruit_GFX.h>     
#include <Adafruit_ILI9341_AS.h>  
#include <SPI.h>              
#include <SdFat.h>              
SdFat SD;

#include <LiquidCrystal.h>
LiquidCrystal lcd(A0, A1, A2, A3, A4, A5); //16x2 LCD ekran pinleri



#define TFT_RST  6          //LCD ekran reset pin
#define TFT_CS   10           //LCD ekran CS pin
#define TFT_DC   9            //LCD ekran DC pin
#define TFT_MISO 12         //Arduino NANO MISO pin
#define TFT_MOSI 11          //Arduino NANO MOSI pin
#define TFT_CLK 13         //LCD ekran CLK pin (ya da SCK)

#define SD_CS    8            //SD kart CS pin

#define LED      7           //röle pin
#define buton    2           //buton pin

#define BU_BMP 1            // ekran yönü
#define TD_BMP 0 

String FileName;
int number = 1;

Adafruit_ILI9341_AS tft = Adafruit_ILI9341_AS (TFT_CS, TFT_DC, TFT_RST); //LCD ekran tanıtımı

//Variables
bool print_finished = false;
bool success = false;
bool UP = HIGH;
bool DOWN = LOW;

int dirPin = 3;
int stepPin = 4;
int power = 5;
int lift_print_distance = 5; //5mm yukarı
float layer_height = 0.15; //mm cinsinden (katman yüksekliği)
float unit = 800.0; // (trapez mil, nema17 ve full microstep DRV8825 için adım ayarı)Platform 1mm yukarı çıkması için 800 birim hareket yapmalı.

uint32_t drawTime = 0;

#include <AccelStepper.h> 
#define motorInterfaceType 1 
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin); //step motor tanıtımı

float LED_on_time = 30 * 1000; // pozlama süresi 
float max_height = 180; //makine max yükseklik
float actual_height = 0;
bool first_print = false;



void setup(void) {
  //Serial.begin(9600);

  lcd.begin(16, 2);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);       //UV ledler kapalı 

  pinMode(dirPin, OUTPUT);
  pinMode(power, OUTPUT);
  pinMode(stepPin, OUTPUT);

  stepper.setMaxSpeed(2000); // hız
  stepper.setAcceleration(20000); //ivme
  
  tft.init();

  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(1);

  //Serial.print("Initializing SD card...");

  lcd.clear();
  lcd.print("SD Card control");
  delay(2000);


  if (!SD.begin(SD_CS, SPI_FULL_SPEED)) {
    // Serial.println("failed!");

    lcd.setCursor(0, 1);
    lcd.print("SD init Failed!");
    lcd.clear();
    return;

  }

  Serial.println("OK!");

  lcd.setCursor(0, 1); 
  lcd.print("SD init OK!");


  delay(2000);
  lcd.clear();

  lcd.print("Home machine...");

  while (true)
  {
    if (digitalRead(buton) == 0) {
      //Serial.println("Waiting for a manuel homing");

    }


    if (digitalRead(buton) == 1) {

      //Serial.println("Machine homed manually");
      lcd.clear();
      lcd.print("Lifting...");

      first_resin_lift();

      lcd.clear();
      lcd.print("Fill resin...");

      while (true) {

        // Serial.println("Waiting for resin");



        if (digitalRead(buton) == 1) {
          Serial.println("Resin vat is full");

          lcd.clear();
          lcd.print("Homing...");
          break;

        }

      }
      if (digitalRead(buton) == 1) {

        go_home();
        lcd.clear();

        break;
      }
    }
  }
}


///////////////////////VOID LOOP////////////////////////////
////////////////////////////////////////////////////////////
void loop() {


  if (!print_finished) {
    print_next_bmp();
    if (!print_finished) {
      turn_on_LED(LED_on_time);
      lift_print();
      lower_print();
    }
  }
  if (print_finished && !success) {
    lift_finished_print();
    success = true;
  }
}//End void loop


////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void go_home() { //ilk başta home yapıyor 
  //Serial.println("Going to first layer");

  digitalWrite(power, HIGH);
  stepper.moveTo(0);
  stepper.runToPosition();
  digitalWrite(power, LOW);


  //Serial.println("Print starting in a 3 seconds");
  delay(3000);

}

void first_resin_lift() {
  //Serial.println("Platform lifting for resin");

  float resin_dist = ((max_height - 140) * unit);

  digitalWrite(power, HIGH);
  stepper.moveTo(resin_dist);
  stepper.runToPosition();
  digitalWrite(power, LOW);


}

void lower_print() { // platformu indir

  //Serial.println("Lowering platform");

  float lower_dist = (number - 1) * (layer_height) * unit ;

  digitalWrite(power, HIGH);
  stepper.moveTo(lower_dist);
  stepper.runToPosition();
  digitalWrite(power, LOW);
  actual_height = (lower_dist / unit);

  //Serial.print("Actual height: "); Serial.println(actual_height);
  delay(1000);
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void lift_finished_print() { //baskı bittiyse yükseğe çık

  //Serial.println("Rising finished print");

  float finished_dist = (((number - 1) * layer_height) + 30) * unit;

  digitalWrite(power, HIGH);
  stepper.moveTo(finished_dist);
  stepper.runToPosition();
  digitalWrite(power, LOW);
  actual_height = finished_dist / unit;

  //Serial.print("Actual height: "); Serial.println(actual_height);

  while (true) {
    //Serial.println("PRINT FINISHED!");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PRINT FINISHED...");

    delay(1000);
  }

}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void lift_print() { // katman değiştirmek için yükseğe çık

  //Serial.println("Rising platform");

  float lift_dist = (((number - 1) * layer_height) + lift_print_distance) * unit;
  digitalWrite(power, HIGH);
  stepper.moveTo(lift_dist);
  stepper.runToPosition();
  digitalWrite(power, LOW);
  actual_height = (lift_dist / unit);

  //Serial.print("Actual height: "); Serial.println(actual_height);
  delay(1000);
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void turn_on_LED(float t) { //UV ledi aç

    digitalWrite(LED, HIGH);
    delay(t);
    delay(t);
    digitalWrite(LED, LOW);
    delay(100);

  tft.fillScreen(ILI9341_BLACK);

}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void print_next_bmp() { //bir sonraki resmi bas
  FileName += number;
  FileName += ".bmp";
  char NameChar[20];

  FileName.toCharArray(NameChar, 20);
  tft.setRotation(3);
  //Serial.println(NameChar);

  drawBMP(NameChar, 0, 0, BU_BMP);

  lcd.setCursor(0, 0);
  lcd.print("Printing...");

  lcd.setCursor(0, 1);
  lcd.print("Layer:");
  lcd.print(number);

  delay(1000);
  number++;
  FileName = "";
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

#define BUFF_SIZE 80

void drawBMP(char *filename, int x, int y, boolean flip) { //ekrana bmp resmi çizdirme
  if ((x >= tft.width()) || (y >= tft.height())) return;
  File     bmpFile;
  int16_t  bmpWidth, bmpHeight;  
  //uint8_t  bmpDepth;            
  uint32_t bmpImageoffset;       
  uint32_t rowSize;               
  uint8_t  sdbuffer[3 * BUFF_SIZE];    
  uint16_t tftbuffer[BUFF_SIZE];       
  uint8_t  sd_ptr = sizeof(sdbuffer); 
  boolean  goodBmp = false;            
  int16_t  w, h, row, col;            
  //uint8_t  r, g, b;  
  uint8_t rotation;     
  uint8_t  tft_ptr = 0;  

  
  if ((bmpFile = SD.open(filename)) == NULL) {


    lcd.clear();
    lcd.print("File not found!");

    print_finished = true;

    return;
  }

  drawTime = millis();

 
  if (read16(bmpFile) == 0x4D42) { 
    read32(bmpFile);      
    read32(bmpFile);      
    bmpImageoffset = read32(bmpFile); 
    read32(bmpFile);       
    bmpWidth  = read32(bmpFile); 
    bmpHeight = read32(bmpFile);  

    if ((read16(bmpFile) == 1) && (read16(bmpFile) == 24) && (read32(bmpFile) == 0)) { 
      
      rowSize = (bmpWidth * 3 + 3) & ~3;
   
      w = bmpWidth;
      h = bmpHeight;

      rotation = tft.getRotation();
  
      if (flip) tft.setRotation((rotation + (flip << 2)) % 8);

      switch (rotation) {
        case 0:
          if (flip) y = tft.height() - y - h; break;
        case 1:
          y = tft.height() - y - h; break;
          break;
        case 2:
          if (flip) y = tft.height() - y - h; break;
          break;
        case 3:
          y = tft.height() - y - h; break;
          break;
      }

   
      tft.setAddrWindow(x, y, x + w - 1, y + h - 1);


      for (uint32_t pos = bmpImageoffset; pos < bmpImageoffset + h * rowSize ; pos += rowSize) {
        
        if (bmpFile.position() != pos) {
          bmpFile.seek(pos);
          sd_ptr = sizeof(sdbuffer);
        }

    
        for (col = 0; col < w; col++) {
       
          if (sd_ptr >= sizeof(sdbuffer)) {
           
            if (tft_ptr) {
            
              tft.pushColors(tftbuffer, tft_ptr);
              tft_ptr = 0;
            }
          
            bmpFile.read(sdbuffer, sizeof(sdbuffer));
            sd_ptr = 0; // Set buffer index to start
          }
     
          tftbuffer[tft_ptr++] = (sdbuffer[sd_ptr++] >> 3) | ((sdbuffer[sd_ptr++] & 0xFC) << 3) | ((sdbuffer[sd_ptr++] & 0xF8) << 8);
        } 
      }   


      if (tft_ptr) tft.pushColors(tftbuffer, tft_ptr);
      drawTime = millis() - drawTime;
    } 
  }   


  bmpFile.close();
 
  tft.setRotation(rotation); 
}
uint16_t read16(File& f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); 
  ((uint8_t *)&result)[1] = f.read(); 
  return result;
}

uint32_t read32(File& f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); 
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); 
  return result;
}

/*
Kaynaklar: 
https://electronoobs.com/eng_arduino_tut148.php
https://www.instructables.com/Arduino-TFT-display-of-bitmap-images-from-an-SD-Ca/
*/
