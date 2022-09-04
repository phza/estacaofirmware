//v5 com bme280
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>

#include "RTClib.h"
#include "SdFat.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <U8x8lib.h>

float bh_multiplicador       = 1.0000;
float bme_temp_multiplicador = 1.0000;
float bme_temp_offset        = 0.0000;
float bme_umi_multiplicador  = 1.0000;
float bme_umi_offset         = 0.0000;

// *******************************************************
// ***********            CONFIGURAÇÕES          *********
// *******************************************************
#define FILE_BASE_NAME "data_"
const uint8_t chipSelect = SS;

// *******************************************************
// ***********          Variaveis globais        *********
// *******************************************************
struct {
  int   record = 0;
  float battV;
  float bme_temp;         
  float bme_press;                
  float bme_umi;         
  long  bh_lux;                        
} sensorData;  

DateTime now;
int minute_last;

// *******************************************************
// ***********          Instancia Objetos        *********
// *******************************************************
Adafruit_BME280 bme;     // I2C BME280 pressão, temperatura e umidade
BH1750 bh;               // I2C BH1750 Luminosidade


SdFat sd; // File system object.
SdFile file; // Log file.
#define error(msg) sd.errorHalt(F(msg))

RTC_DS3231 rtc;

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

// *******************************************************
// ***********        Leitura dos Sensores       *********
// *******************************************************
void readSensors(){
  sensorData.record   += 1;
  sensorData.battV     = float(readVcc())   /1000.0 ;
  sensorData.bme_temp  = bme.readTemperature();
  sensorData.bme_press = bme.readPressure() /100.0F  ; // em hPa
  sensorData.bme_umi   = bme.readHumidity()    *bme_umi_multiplicador + bme_umi_offset ; // calibrados
  sensorData.bh_lux    = bh.readLightLevel()   *bh_multiplicador ; // em wm-2
}

void writeHeader(int serial = 0) {
  file.print(F("\"TIMESTAMP\",\"RECORD\",\"battV\","));
  file.print(F("\"bme_temp\",\"bme_press\","));
  file.print(F("\"bme_umi\",\"bh_lux\""));
  file.println();
  
  if(serial){
    Serial.print(F("\"TIMESTAMP\",\"RECORD\",\"battV\","));
    Serial.print(F("\"bme_temp\",\"bme_press\","));
    Serial.print(F("\"bme_umi\",\"bh_lux\""));
    Serial.println();    
  }
}

void writeSensors(int serial = 0, int card = 1){
  char agora[18];
  sprintf(agora, "%02d-%02d-%02d %02d:%02d:00", now.year(), now.month(), now.day(), now.hour(), now.minute());
         
  if(serial){
    Serial.print(F("\"")); 
    Serial.print(agora);  
    Serial.print(F("\","));  
    Serial.print(sensorData.record);
    Serial.print(F(","));
    Serial.print(sensorData.battV);
    Serial.print(F(","));   
    Serial.print(sensorData.bme_temp);
    Serial.print(F(",")); 
    Serial.print(sensorData.bme_press);
    Serial.print(F(",")); 
    Serial.print(sensorData.bme_umi);
    Serial.print(F(",")); 
    Serial.print(sensorData.bh_lux);
    Serial.println();
  }
  
  if(card){   
    file.print(F("\"")); 
    file.print(agora);  
    file.print(F("\","));  
    file.print(sensorData.record);
    file.print(F(","));
    file.print(sensorData.battV);
    file.print(F(","));   
    file.print(sensorData.bme_temp);
    file.print(F(",")); 
    file.print(sensorData.bme_press);
    file.print(F(",")); 
    file.print(sensorData.bme_umi);
    file.print(F(",")); 
    file.print(sensorData.bh_lux);
    file.println();
    
    if (!file.sync() || file.getWriteError()) {
      error("write error");
    }
  }
     
}



void writeDisplay() {
  
  u8x8.setFont(u8x8_font_chroma48medium8_r); 
  u8x8.clearDisplay();     
  char timertc[6];
  sprintf(timertc, "%02d:%02d", now.hour(), now.minute()); 
  u8x8.drawString(0,0,"PGFA");
  u8x8.drawString(11,0,timertc);
  char datertc[9];
  sprintf(datertc, "%02d/%02d/%02d", now.day(), now.month(), now.year());
  u8x8.drawString(6,1,datertc);

  u8x8.setCursor(0, 2);
  u8x8.print("S: ");  
  u8x8.print(sensorData.record);
  u8x8.setCursor(11, 2);  
  u8x8.print(sensorData.battV);
  u8x8.print("V"); 
  
  u8x8.setCursor(0, 3); 
  u8x8.print("BME: ");  
  u8x8.print(sensorData.bme_temp);
  u8x8.print(" C"); 
  u8x8.setCursor(0, 4); 
  u8x8.print("BME: "); 
  u8x8.print(sensorData.bme_press);
  u8x8.print(" hPa"); 

  //u8x8.setCursor(0, 5);  
  //u8x8.print("DHT: ");
  //u8x8.print(sensorData.hmp_temp);
  //u8x8.print(" C"); 
  u8x8.setCursor(0, 6);
  u8x8.print("BME: ");  
  u8x8.print(sensorData.bme_umi);
  u8x8.print(" %"); 

  u8x8.setCursor(0, 7);
  u8x8.print("BH: ");  
  u8x8.print(sensorData.bh_lux);
  u8x8.print(" lux"); 

}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}


void setup() {
  // *******************************************************
  // ***********   Inicialização dos periféricos   *********
  // *******************************************************
  Serial.begin(9600);
  
  if (! rtc.begin()) {
    Serial.println(F("Não encontrei o RTC"));
    Serial.flush();
    abort();
  }
  now = rtc.now();
  minute_last = now.minute();
  
  u8x8.begin();
  u8x8.setPowerSave(0);
  
  // *******************************************************
  // ***********     Inicialização dos sensores    *********
  // *******************************************************
  unsigned status;
  status = bme.begin(0x76);  

  if (!status) {
    Serial.println(F("Error: BMP280"));
    while (1);
  }
  if (!bh.begin()) {
    Serial.println(F("Error: BH1750"));
    while (1);
  };

  
  /* Weather station ettings from datasheet. */
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X16, // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X16, // humidity
                    Adafruit_BME280::FILTER_OFF   );

  // *******************************************************
  // ********** Inicialização do cartão e arquivo  *********
  // *******************************************************
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";
  
  if (!sd.begin(chipSelect, SD_SCK_MHZ(4))) {
    sd.initErrorHalt();
  }
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      error("Can't create file name");
    }
  }
  if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL)) {
    error("file.open");
  }
  Serial.print(F("Logging to: "));
  Serial.println(fileName);
  // Write data header.
  writeHeader(1);
  
}

void loop() {
  //Check time
  now = rtc.now();
    
  if (now.minute() != minute_last){
    bme.takeForcedMeasurement();
    readSensors();
    writeSensors(1);
    writeDisplay();
     
    minute_last = now.minute();
  }
  
  //calm down pgfa:)
  delay(2000);
}
