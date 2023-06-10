#include <Arduino.h>
#include "soc/rtc.h"
#include "HX711.h"
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

float tien, tien2;

//loadcell
const int LOADCELL_DOUT_PIN = 26; //DT - D27
const int LOADCELL_SCK_PIN = 27;  //SCK - D26
float k = 0, can, M;
HX711 scale;

//servo
Servo myServo;
int servoPin = 15; //pin D15

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

//GPS
SoftwareSerial gpsSerial(32, 33);
TinyGPSPlus gps;
float lat1, lon1, lat2, lon2, distance, distance2;

//Nut nhan
const int buttonPin = 4;  // GPIO 4
//int nut = 0;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  gpsSerial.begin(9600);
  // myServo.attach(servoPin); 

  lcd.init();
  lcd.backlight();

  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(497.8556);
  scale.tare();

  pinMode(buttonPin, INPUT_PULLUP);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("XIN MOI NHAN NUT");
}

void loop() {
  int buttonState = digitalRead(buttonPin);
  if (buttonState == LOW) {  // Nếu nút nhấn được nhấn xuống, thực hiện gắp vật
    Serial2.println("g");
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("DAT HANG VAO!");
    myServo.write(80);
    delay(1000);
    myServo.write(30);
  }
  if (Serial2.available() > 0)
  {
    char data1 = Serial2.read();
    //Serial.println(data1);
    if (data1 == 'a') //tín hiệu nhận từ UAV
    {
      lcd.clear();
      CanKhoiLuong();
      //Serial.println("Cân Khối Lượng");
    }
    while (true)
    {
      TinhQuangDuong();
    }
  }
}
void TinhQuangDuong()
{
  //Serial.println("Tính quãng đường!");
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      lat2 = gps.location.lat();
      lon2 = gps.location.lng();

      if (lat1 != 0 && lon1 != 0) {
        distance += (TinyGPSPlus::distanceBetween(lat1, lon1, lat2, lon2)) / 1000;
      }
      lat1 = lat2;
      lon1 = lon2;
      distance2 += distance;
      distance = 0;
//      Serial.print("Vĩ độ: ");
//      Serial.println(lat2);
//      Serial.print("Kinh độ: ");
//      Serial.println(lon2);
//      Serial.print("S: ");
//      Serial.print(distance);
//      Serial.println(" km");
      lcd.setCursor(0, 1);
      lcd.print("QD: ");
      lcd.setCursor(3, 1);
      lcd.print(distance2);
      lcd.setCursor(7, 1);
      lcd.print("km|");
      //delay(500);

      char data2 = Serial2.read();
      //Serial.println(data2);
      if (data2 == 'n') //tín hiệu nhận từ UAV
      {
        myServo.write(80);
        //Serial.println("Thả vật");
        TinhTien();
      }
    }
  }
}
void CanKhoiLuong()
{
  k = scale.get_units();
  can = (k - 1559.3836) / 1000;
  
  if (can < 0) can = 0;
  //Serial.print(can);
  //Serial.println("kg");
  lcd.setCursor(0, 0);
  lcd.print("KL: ");
  lcd.setCursor(3, 0);
  lcd.print(can);
  lcd.setCursor(7, 0);
  lcd.print("kg|");
  delay(500);

}
void TinhTien()
{
  //Serial.println("Tính tiền!");
  //distance2 = distance;
  tien = distance2 * 5 + can * 1;
  lcd.setCursor(12, 0);
  lcd.print("TT: ");
  lcd.setCursor(11, 1);
  lcd.print(tien);
  lcd.setCursor(15, 1);
  lcd.print("$");
}
