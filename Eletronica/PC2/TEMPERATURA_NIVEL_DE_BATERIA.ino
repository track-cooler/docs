// Interfacing Arduino with DS18B20 temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>              // include LCD library code
//#include "BluetoothSerial.h"
#include <stdio.h>
#define DS18B20_PIN   10
#define     vmax   3.6
 
//BluetoothSerial SerialBT;
LiquidCrystal_I2C lcd(0x20,16,2);

int raw_temp;
float temp=0.0;
char txt[] = " C ";
float voltage = 0.0; 
float voltage1 = 0.0; 
float voltage2 = 0.0;  

void setup(){
lcd.init();
lcd.backlight();
//SerialBT.begin("ESP32")
}

void loop(){
  // set cursor to first column, first row
   if(ds18b20_read(&raw_temp)) {
    Serial.print("Temperature= ");
    temp = (float)raw_temp / 16;     // Convert temperature raw value into degree Celsius (temp in °C = raw/16)      
    Serial.print(temp);              // Print temperature value in degree Celsius
    Serial.println(" ºC");             // Print '°C'
    Serial.print("percentual da bateria= ");
    
    // Display temperature on LCD
      if(temp>=0){
        lcd.setCursor(0, 0);//primeiro valor indica coluna, segundo linha
        lcd.print("Temp= ");
        lcd.setCursor(7, 0);
        lcd.write(B00101011);
        lcd.setCursor(8, 0);
        lcd.print(temp,2);
        lcd.setCursor(14, 0);
        lcd.write(B11011111);
        lcd.setCursor(15, 0);
        lcd.print("C");
      } else {
        lcd.setCursor(0, 0);//primeiro valor indica coluna, segundo linha
        lcd.print("Temp= ");
        lcd.setCursor(6, 0);
        //lcd.write(B00101101);
        //lcd.setCursor(9, 0);
        lcd.print(temp,2);
        lcd.setCursor(14, 0);
        lcd.write(B11011111);
        lcd.setCursor(15, 0);
        lcd.print("C");
      }
    voltage = (analogRead(A2)*vmax)/1023.0;
    voltage1=float((voltage*28100)/4100);
    voltage2=float((24/voltage1)*100);
    lcd.setCursor(0,1);
    lcd.write(B00100101);
    lcd.setCursor(1,1);
    lcd.print("Charge=");
    lcd.setCursor(9,1);
    lcd.print(voltage2,2);
    lcd.setCursor(15,1);
    lcd.write(B00100101);       
    //char bateria[3];
    //gcvt(voltage, 3, bateria); 
      //for(int i = 0; i < sizeof(bateria); i++){
         //SerialBT.write(bateria[i]);          
    //}    
    
  }
  else {
    Serial.println("Communication Error!");
    lcd.setCursor(4, 1);
    lcd.print(" Error! ");
  }
  delay(1000);
}

bool ds18b20_start(){
  bool ret = 0;
  digitalWrite(DS18B20_PIN, LOW);     // Send reset pulse to the DS18B20 sensor
  pinMode(DS18B20_PIN, OUTPUT);
  delayMicroseconds(500);             // Wait 500 us
  pinMode(DS18B20_PIN, INPUT);
  delayMicroseconds(100);             //wait to read the DS18B20 sensor response
  if (!digitalRead(DS18B20_PIN)) {
    ret = 1;                          // DS18B20 sensor is present
    delayMicroseconds(400);           // Wait 400 us
  }
  return(ret);
}
 
void ds18b20_write_bit(bool value){
  digitalWrite(DS18B20_PIN, LOW);
  pinMode(DS18B20_PIN, OUTPUT);
  delayMicroseconds(2);
  digitalWrite(DS18B20_PIN, value);
  delayMicroseconds(80);
  pinMode(DS18B20_PIN, INPUT);
  delayMicroseconds(2);
}
 
void ds18b20_write_byte(byte value){
  byte i;
  for(i = 0; i < 8; i++)
    ds18b20_write_bit(bitRead(value, i));
}
 
bool ds18b20_read_bit(void) {
  bool value;
  digitalWrite(DS18B20_PIN, LOW);
  pinMode(DS18B20_PIN, OUTPUT);
  delayMicroseconds(2);
  pinMode(DS18B20_PIN, INPUT);
  delayMicroseconds(5);
  value = digitalRead(DS18B20_PIN);
  delayMicroseconds(100);
  return value;
}
 
byte ds18b20_read_byte(void) {
  byte i, value;
  for(i = 0; i  <8; i++)
    bitWrite(value, i, ds18b20_read_bit());
  return value;
}
 
bool ds18b20_read(int *raw_temp_value) {
  if (!ds18b20_start())                     // Send start pulse
    return(0);                              // Return 0 if error
  ds18b20_write_byte(0xCC);                 // Send skip ROM command
  ds18b20_write_byte(0x44);                 // Send start conversion command
  while(ds18b20_read_byte() == 0);          // Wait for conversion complete
  if (!ds18b20_start())                     // Send start pulse
    return(0);                              // Return 0 if error
  ds18b20_write_byte(0xCC);                 // Send skip ROM command
  ds18b20_write_byte(0xBE);                 // Send read command
  *raw_temp_value = ds18b20_read_byte();    // Read temperature LSB byte and store it on raw_temp_value LSB byte
  *raw_temp_value |= (unsigned int)(ds18b20_read_byte() << 8);     // Read temperature MSB byte and store it on raw_temp_value MSB byte
  return(1);                                // OK --> return 1
}
