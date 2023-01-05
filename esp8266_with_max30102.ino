#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#define SCREEN_WIDTH 128 // 屏幕宽度
#define SCREEN_HEIGHT 64 // 屏幕高度
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define LOGO_HEIGHT   24
#define LOGO_WIDTH    24

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MAX30105 particleSensor;
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
/***
 * 相关展示图标
***/
static const unsigned char PROGMEM huxi_bmp [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x24, 0x00, 0x00,
0x24, 0x00, 0x03, 0x24, 0xC0, 0x04, 0xA5, 0x20, 0x08, 0x24, 0x10, 0x00, 0x66, 0x00, 0x10, 0x42,
0x08, 0x10, 0x81, 0x08, 0x20, 0x00, 0x04, 0x20, 0x18, 0x04, 0x20, 0xC3, 0x04, 0x20, 0x42, 0x04,
0x00, 0x42, 0x00, 0x00, 0x42, 0x00, 0x20, 0x42, 0x04, 0x27, 0x81, 0xE4, 0x10, 0x00, 0x08, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

static const unsigned char PROGMEM xinzang_bmp [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x40, 0x00, 0x20, 0x40, 0x00, 0x00, 0x00, 0x80, 0x80,
0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00,
0x0C, 0x00, 0x00, 0x04, 0x20, 0x01, 0x00, 0x80, 0x3A, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x01, 0x08, 0x04, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

static const unsigned char PROGMEM xinzang_big_bmp [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00, 0x90, 0x00, 0x04, 0x50, 0x00, 0x02,
0x3E, 0x70, 0x11, 0x01, 0x08, 0x08, 0x84, 0x28, 0x00, 0x08, 0x70, 0x07, 0x10, 0x80, 0x09, 0x10,
0x40, 0x09, 0x02, 0x40, 0x09, 0x24, 0x00, 0x08, 0x18, 0x40, 0x08, 0x08, 0x40, 0x08, 0x08, 0x00,
0x00, 0x00, 0x80, 0x04, 0x11, 0x00, 0x04, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x04, 0x00, 0x00,
0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

void setup(){
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Start the OLED display
  display.display();
  delay(3000);

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30102 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void loop(){
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  display.clearDisplay(); 
  display.setTextSize(1);                                   //Near it display the average BPM you can display the BPM if you want
  display.setTextColor(WHITE); 
  display.setCursor(30,5);                
  display.println("wait for");    
  display.setCursor(30,15);                
  display.println("initing...");               
  display.display();
    
  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }
  
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

//      //send samples and calculation result to terminal program through UART
//      Serial.print(F("red="));
//      Serial.print(redBuffer[i], DEC);
//      Serial.print(F(", ir="));
//      Serial.print(irBuffer[i], DEC);
//
//      Serial.print(F(", HR="));
//      Serial.print(heartRate, DEC);
//
//      Serial.print(F(", HRvalid="));
//      Serial.print(validHeartRate, DEC);
//
//      Serial.print(F(", SPO2="));
//      Serial.print(spo2, DEC);
//
//      Serial.print(F(", SPO2Valid="));
//      Serial.println(validSPO2, DEC);
        
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}
