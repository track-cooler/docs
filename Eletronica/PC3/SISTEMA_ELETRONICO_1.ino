#define TRACK_COOLER
#include <LiquidCrystal_I2C.h> 
#include <Wire.h>
#include "BluetoothSerial.h"
#include <stdio.h>
#include <HardwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "TinyGPS.h"

TinyGPS gps; // 

BluetoothSerial SerialBT;

//Pinos dos sensores ultrassônicos:
#define trigPin1 15
#define echoPin1 7
#define trigPin2 5
#define echoPin2 4
#define trigPin3 35
#define echoPin3 34
#define trigPin4 16
#define echoPin4 17

//Pinos do sistema de indicação de temperatura e nível da bateria
#define DS18B20_PIN  14
#define DIV_TEN_PIN  25 
#define SDA_LCD_PIN  13
#define SCL_LCD_PIN  12

//Tensão máxima permitida nos pinos da ESP
#define     vmax   3.6 

//Pino GPS
#define GPS_TX_PIN TX

HardwareSerial nss(1);
// Definições para o GPS  e Bússola

#define GPS_STREAM_TIMEOUT 18
#define GPS_WAYPOINT_TIMEOUT 45
#define GPS_UPDATE_INTERVAL 1000

// Declinação do ângulo com base no ângulo do usuário necessário para a calibração da bússola.
#define DECLINATION_ANGLE 0.21f

//com base no norte verdadeiro da bússola
#define BUSSOLA_OFFSET 0.0f
struct GeoLoc {
  float lat;
  float lon;
};

//função do LCD
LiquidCrystal_I2C lcd(0x20,16,2);

//Variáveis para cálculo da distância com os sensores ultrassônicos:
long duration, distance, RightSensor,BackSensor,FrontSensor,LeftSensor;

//Pinos do PWM para controle do motor 1 (Controlará a esteira direita)
int RPWM_dir = 18; //Para frente
int LPWM_dir = 19; //Para trás

//Pinos do PWM para controle do motor 2 (Controlará a esteira esquerda)
int RPWM_esq = 26; //Para frente
int LPWM_esq = 27; //Para trás

//Declaração dos reles que serão usados no projeto
int rele_motor_freio = 32 ;
int rele_solenoide = 33;

//Definindo as propriedades do PWM
const int freq = 3000;
const int dir_pwmChannel_rpwm = 0;
const int dir_pwmChannel_lpwm = 1;
const int esq_pwmChannel_rpwm = 2;
const int esq_pwmChannel_lpwm = 3;
const int resolution = 10;

//Definir variáveis de alerta
int alerta_parado = 0;
int alerta_obst = 0;
int nivel_bateria = 0;

//Variáveis para o cálculo da temperatura e nível da bateria
int raw_temp;
float temp=0.0;
char txt[] = " C ";
float voltage = 0.0; 
float voltage1 = 0.0; 
float voltage2 = 0.0; 

BluetoothSerial SerialBT; //Comando para utilizar o Bluetooth 

// Habilitar mestre
bool enabled = false;

//configuração seçao crítica da ISR da temperatura e nível de bateria
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // instancia o mux

portENTER_CRITICAL_ISR(&mux); // abre a seção critica
   state = !state; // altera o valor da variavel
portEXIT_CRITICAL_ISR(&mux); // fecha a seção critica

// Bússola
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//incializando GPS e conferindo
   GeoLoc confGPS() {
     Serial.println("Lendo GPS: ");
     bool novos_dados = false;
     unsigned long start = millis(); //configura dados do GPS para o país habilitado
    // nativo da bibilioteca para correção de dados 
     while (millis() - start < GPS_UPDATE_INTERVAL) {
      if (liga_gps())
       novos_dados = true;
  }
    if (novos_dados) {
      return gpsdump(gps);
  }

  GeoLoc track_coolerLoc;
  track_coolerLoc.lat = 0.0;
  track_coolerLoc.lon = 0.0;
  
  return track_coolerLoc;
}

// Obter e processar dados GPS

GeoLoc gpsdump(TinyGPS &gps) {
  float flat, flon;
  unsigned long periodo; //dados do gps, como latitude, longitude, data, hora.
  
  gps.f_get_position(&flat, &flon, &periodo); //obtendo posição

  GeoLoc track_coolerLoc;
  track_coolerLoc.lat = flat;
  track_coolerLoc.lon = flon;

  Serial.print(track_coolerLoc.lat, 7); Serial.print(", "); Serial.println(track_coolerLoc.lon, 7);

  return track_coolerLoc;
}

// Dados conforme são obtidos via satélite. Função nativa da biblioteca.
bool liga_gps() {
  while (nss.available()) {
    if (gps.encode(nss.read())) 
      return true;
  }
  return false;
}

//Transimissão dos dados do GPS do Track Cooler e conexão com celular do usuário.
  //Função dentro do app do Track Cooler
 void TRACK_COOLERr(){
  //parâmetros do GPS do usuário obtidos após a conexão
    GpsParam = gps(param);
 
  Serial.println("GPS remoto recebido: ");
  
  // Imprimir 6 casas decimais 
  Serial.print(gps.get_position(&lat), 6); Serial.print(", "); Serial.println(gps.get_position(&lon), 6);
  
  // dados do GPS do usuário sendo recebidos.
  GeoLoc usuLoc;
  usuLoc.lat = gps.get_position(&lat);
  usuLoc.lon = gps.get_position(&lon);

  // Dados do usuário recebidos para começar a seguir
  desloca(usuLoc, GPS_STREAM_TIMEOUT);

}


// Terminal para conexão
void  TRACK_COOLER(){
  Serial.print("Texto Recebido: ");
  Serial.println(param.asStr());

  String rawInput(param.asStr());
  int colonIndex;
  int commaIndex;
  
  do {
    commaIndex = rawInput.indexOf(',');
    colonIndex = rawInput.indexOf(':');
    
    if (commaIndex != -1) {
      String latStr = rawInput.substring(0, commaIndex);
      String lonStr = rawInput.substring(commaIndex+1);

      if (colonIndex != -1) {
         lonStr = rawInput.substring(commaIndex+1, colonIndex);
      }
    
      float lat = latStr.toFloat();
      float lon = lonStr.toFloat();
    
      if (lat != 0 && lon != 0) {
        GeoLoc encontro;
        encontro.lat = lat;
        encontro.lon = lon;
    
        Serial.print("Ponto de encontro: "); Serial.print(lat); Serial.println(lon);
        desloca(encontro, GPS_WAYPOINT_TIMEOUT);
      }
    }
    
    rawInput = rawInput.substring(colonIndex + 1);
    
  } while (colonIndex != -1);
}

//para obter dados da bussóla e fazer  conexão usuário e track cooler

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

float geoBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float y = sin(b.lon-a.lon) * cos(b.lat);
  float x = cos(a.lat)*sin(b.lat) - sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
  return atan2(y, x) * RADTODEG;
}

float geoDistancia(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km
  float p1 = a.lat * DEGTORAD; // mudando de graus para radiano
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat-a.lat) * DEGTORAD;
  float dl = (b.lon-a.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

float geoHeading() {
 
  sensors_event_t event; 
  mag.getEvent(&event);

  //Corrige sinais do eixo em ralação x e y
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Descolamento
  heading -= DECLINATION_ANGLE;
  heading -= BUSSOLA_OFFSET;
  
  // Para o sinais da Bússola invertidos. 
  if(heading < 0)
    heading += 2*PI;
    
  //Verificação após o uso da declinação 
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Converte radianos para graus
  float headingDegrees = heading * 180/M_PI; 

  // Mapa para -180 - 180
  while (headingDegrees < -180) headingDegrees += 360;
  while (headingDegrees >  180) headingDegrees -= 360;

  return headingDegrees;
}

void desloca(struct GeoLoc &loc, int timeout) {
  int direcao = 0;
  nss.listen();
  GeoLoc track_coolerLoc = checkGPS();
  bluetoothSerial.listen();

  if (track_coolerLoc.lat != 0 && track_coolerLoc.lon != 0 && enabled) {
    float d = 0;
    
    do {
      nss.listen();
      track_coolerLoc = liga_GPS();
      bluetoothSerial.listen();
      
      d = geoDistancia(track_coolerLoc, usuloc);
      float t = geoBearing(track_coolerLoc, usuloc) - geoHeading();
      
      Serial.print("Distancia: ");
      Serial.println(geoDistancia(track_coolerLoc, usuloc));
    
      Serial.print("Bearing: ");
      Serial.println(geoBearing(track_coolerLoc, usuloc));

      Serial.print("heading: ");
      Serial.println(track_geoHeading());
      
    if(t < abs(20)){
    anda_frente();
    }else if(t > abs(20)){
    while (t < -180) t += 360;
      while (t >  180) t -= 360;
    if (t < 0) {
      direcao = 1;
    } else if (t > 0){
      direcao = 2;
    }
    vira(direcao);
    }
      timeout -= 1;
    } while (d > 0.5 && enabled && timeout>0); //  

    stop();
  }
}

void setupBussola() {
   // Inicializa Bússola
  if(!mag.begin())
  {
    //Caso não consiga detecyat a bússola
    Serial.println("Bússola não detectada, verifique a conexão!");
    while(1);
  }

void setup() {
   Serial.begin(9600);
   pinMode(RPWM_dir, OUTPUT);
   pinMode(LPWM_dir, OUTPUT);
   pinMode(RPWM_esq, OUTPUT);
   pinMode(LPWM_esq, OUTPUT);
   pinMode(rele_motor_freio, OUTPUT);
   pinMode(rele_solenoide, OUTPUT);
   pinMode(trigPin1, OUTPUT);
   pinMode(echoPin1, INPUT);
   pinMode(trigPin2, OUTPUT);
   pinMode(echoPin2, INPUT);
   pinMode(trigPin3, OUTPUT);
   pinMode(echoPin3, INPUT);
   pinMode(trigPin4, OUTPUT);
   pinMode(echoPin4, INPUT);
   SerialBT.begin("Track Cooler");
   
//Inicialização do LCD
   lcd.init();
   lcd.backlight();
   lcd.clear();
   Wire.begin(GPIO13,GPIO12);

//rotina de interrupção
startTimer();

 //Configurando o sinal PWM
   ledcSetup(dir_pwmChannel_rpwm, freq, resolution);
   ledcSetup(dir_pwmChannel_lpwm, freq, resolution);
   ledcSetup(esq_pwmChannel_rpwm, freq, resolution);
   ledcSetup(esq_pwmChannel_lpwm, freq, resolution);

 //Configurando as propriedades do PWM
 //GPIO de onde o PWM vem e GPIO de onde quero o sinal
   ledcAttachPin(RPWM_dir, dir_pwmChannel_rpwm);
   ledcAttachPin(LPWM_dir, dir_pwmChannel_lpwm);
   ledcAttachPin(RPWM_esq, esq_pwmChannel_rpwm);
   ledcAttachPin(LPWM_esq, esq_pwmChannel_lpwm);

  //Bluetooth
  bluetoothSerial.begin();

   //GPS
  nss.begin();

  // Bússola
  setupBussola();
 }

void TEMP_CHARGE() {
   if(ds18b20_read(&raw_temp)) {
    temp = (float)raw_temp / 16;     // Convert temperature raw value into degree Celsius (temp in °C = raw/16)      
    // Display temperature on LCD
      if(temp>=0){
        lcd.setCursor(0, 0);
        lcd.print("Temp= ");
        lcd.setCursor(7, 0);
        lcd.print(temp,2);
        lcd.setCursor(14, 0);
        lcd.write(B11011111);
        lcd.setCursor(15, 0);
        lcd.print("C");
        serial.print("Temp =+",temp)
      } else {
        lcd.setCursor(0, 0);
        lcd.print("Temp= ");
        lcd.setCursor(6, 0);
        lcd.print(temp,2);
        lcd.setCursor(14, 0);
        lcd.write(B11011111);
        lcd.setCursor(15, 0);
        lcd.print("C");
        serial.print("Temp =-",temp)
      }
    voltage = (analogRead(A2)*vmax)/1023.0;
    voltage1=float((voltage*28100)/4100);
    voltage2=float((24/voltage1)*100);
    if (voltage2 <= 20){
      serial.print("ALERTA DE BATERIA BAIXA")
    }
    
    lcd.setCursor(0,1);
    lcd.write(B00100101);
    lcd.setCursor(1,1);
    lcd.print("Charge=");
    lcd.setCursor(9,1);
    lcd.print(voltage2,2);
    lcd.setCursor(15,1);
    lcd.write(B00100101);       
  }
  else {
    Serial.println("Communication Error!");
    lcd.setCursor(4, 1);
    lcd.print(" Error! ");
  }
}

//rotina da interrupção por período de tempo

hw_timer_t * timer = NULL;

void interrupt_timer(){
    static unsigned int counter = 1;
    counter++;
}

void startTimer(){
    //timerBegin(canal,divisão do valor de 80Mhz por 8 para gerar
    //período de 1 us, contador progressivo
    timer = timerBegin(0, 80, true);

    //timerAttach(instancia do hw, endereço da funçao
    //edge gera interrupção)
    timerAttachInterrupt(timer, &TEMP_CHARGE, true);

    //timerAlarm(instancia o timer, valor de 60 us para 60s,
    //repetição do processo)
    timerAlarmWrite(timer, 60000000, true); 

    //ativa o alarme
    timerAlarmEnable(timer);
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


void loop() {
    ledcWrite(dir_pwmChannel_rpwm, 0);
    ledcWrite(dir_pwmChannel_lpwm, 0);
    ledcWrite(esq_pwmChannel_rpwm, 0);
    ledcWrite(esq_pwmChannel_lpwm, 0);
    
}

int anda_frente(){
  int time_turn = 0;
  sensor1_front = SonarSensor(trigPin1, echoPin1);
  sensor2_front = SonarSensor(trigPin2, echoPin2);
  sensor1_down = SonarSensor(trigPin3, echoPin3);
  sensor2_down = SonarSensor(trigPin4, echoPin4);
  while(sensor1_front>=80 || sensor2_front>=80 || sensor1_down<=21 || sensor2_down<=21){
     ledcWrite(dir_pwmChannel_rpwm, 1023);
     ledcWrite(dir_pwmChannel_lpwm, 0);
     ledcWrite(esq_pwmChannel_rpwm, 1023);
     ledcWrite(esq_pwmChannel_lpwm, 0);
     ledcWrite(pwmChannel, dutyCycle);
     sensor1_front = SonarSensor(trigPin1, echoPin1);
     sensor2_front = SonarSensor(trigPin2, echoPin2);
     sensor1_down = SonarSensor(trigPin3, echoPin3);
     sensor2_down = SonarSensor(trigPin4, echoPin4);
  }
  int dutycycle1 = 716;
  int dutycycle2 = 0;
  
  if(distace1 >= 21 || distance2 >=21){
    time_turn = 0;
    //aciona a ré
    delay(2000) //fica dois segundos dando ré}
  if(sensor1_front<=80 || sensor2_front<=80 || distance1>=21 || distance2>=21){
   while(distance1>=21 || distance2>=21 || sensor1_front<=80 || sensor2_front<=80){
     ledcWrite(dir_pwmChannel_rpwm, dutycycle1); //motor da direita gira com 70% do PWM
     ledcWrite(dir_pwmChannel_lpwm, dutycycle2);
     ledcWrite(esq_pwmChannel_rpwm, dutycycle2);
     ledcWrite(esq_pwmChannel_lpwm, dutycycle1); //motor da esquerda gira invertido com 70% do PWM
     sensor1_front = SonarSensor(trigPin1, echoPin1);
     sensor2_front = SonarSensor(trigPin2, echoPin2);
     sensor1_down = SonarSensor(trigPin3, echoPin3);
     sensor2_down = SonarSensor(trigPin4, echoPin4);
      time_turn+=1;
    if(time_turn ==10){
     //Muda os dutyCycles para ele virar para a direita
     dutycycle1 = 0; 
     dutycyle2 = 716;
     }
    if(time_turn == 20){
     stop_motor();
     }
   }
  }    
}
}

void stop_motor(){
    ledcWrite(dir_pwmChannel_rpwm, 0);
    ledcWrite(dir_pwmChannel_lpwm, 0);
    ledcWrite(esq_pwmChannel_rpwm, 0);
    ledcWrite(esq_pwmChannel_lpwm, 0);
    delay(1000); //espera 1 segundo
    digitalWrite(rele_motor_freio, HIGH);
}

void vira(int direcao){
 //comprimento do arco;
 double ce, alpha, ci, t, vi, ve;
    ce = (alpha*3,14*0,630)/180;
    vi = (ci/ce);
    DC = (vi/ve); //DutyCycle para realizar a curva
    int DC_final = int(DC*1023);
    
    if(direcao == 1){
    ledcWrite(dir_pwmChannel_rpwm, DC);
    ledcWrite(dir_pwmChannel_lpwm, 0);
    ledcWrite(esq_pwmChannel_rpwm, 1023);
    ledcWrite(esq_pwmChannel_lpwm, 0);
    delay(ce*1000);
    }
    if(direcao == 2){
    ledcWrite(dir_pwmChannel_rpwm, 1023);
    ledcWrite(dir_pwmChannel_lpwm, 0);
    ledcWrite(esq_pwmChannel_rpwm, DC);
    ledcWrite(esq_pwmChannel_lpwm, 0);
    delay(ce*1000);
    }

}

double SonarSensor(int trigPin,int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return (duration/2) / 29.1;
}
