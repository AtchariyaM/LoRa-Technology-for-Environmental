#include <pgmspace.h>
#include <EEPROM.h>

#include <SPI.h>
#include <LoRa.h>
#include "SSD1306.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;                                            
HardwareSerial SerialGPS(1);

#include <Wire.h>
#include <Adafruit_ADS1015.h>

 Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use thi for the 12-bit version */


int TempValue;
String GPS;
float latt;
float lon;

//String packet;


int counter = 0;
// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO23 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)
//OLED pins to ESP32 0.96OLEDGPIOs via this connecthin:
//OLED_SDA -- GPIO22
//OLED_SCL -- GPIO21
//OLED_RST -- GPIO16

//pH_SENSOR -- GPIO14
//Temp_SENSOR -- GPIO4
//ORP_SENSOR -- GPIO34
//DO_SENSOR -- GPIO35

SSD1306  display(0x3c, 21, 22); // SDA = 21 SCL = 22
#define SS      18
#define RST     23
#define DI0     26
#define BAND    923E6  

/***********************ph***************************/
//#define SensorPin A0            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int16_t pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;

double Avergearray(int16_t* Arr, int Number){
  int i;
  int max,min;
  double avg_ph;
  long amount=0;
  if(Number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(Number<5){   //less than 5, calculated directly statistics
    for(i=0;i<Number;i++){
      amount+=Arr[i];
    }
    avg_ph = amount/Number;
    return avg_ph;
  }else{
    if(Arr[0]<Arr[1]){
      min = Arr[0];max=Arr[1];
    }
    else{
      min=Arr[1];max=Arr[0];
    }
    for(i=2;i<Number;i++){
      if(Arr[i]<min){
        amount+=min;        //arr<min
        min=Arr[i];
      }else {
        if(Arr[i]>max){
          amount+=max;    //arr>max
          max=Arr[i];
        }else{
          amount+=Arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg_ph = (double)amount/(Number-2);
  }//if
  return avg_ph;
}
/***********************temp************************/

#define ONE_WIRE_BUS 4                 // Data wire is connected to GPIO25
OneWire oneWire(ONE_WIRE_BUS);          // Setup a oneWire instance to communicate with a OneWire device
DallasTemperature sensors(&oneWire);    // Pass our oneWire reference to Dallas Temperature sensor 

DeviceAddress Temp = { 0x28, 0x1D, 0x2B, 0x99, 0xA, 0x0, 0x0, 0x9F };

//**********************orp*************************

#define VOLTAGE 5.00    //system voltage
#define OFFSET 0        //zero drift voltage

int orpValue;

#define ArrayLenth  40    //times of collection
//#define orpPin A1          //orp meter output,connect to Arduino controller ADC pin

int16_t orpArray[ArrayLenth];
int orpArrayIndex=0;

double avergearray(int16_t* arr, int number){
  int j;
  int max,min;
  double avg_orp;
  long amount1=0;
  if(number<=0){
    printf("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(j=0;j<number;j++){
      amount1+=arr[j];
    }
    avg_orp = amount1/number;
    return avg_orp;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(j=2;j<number;j++){
      if(arr[j]<min){
        amount1+=min;        //arr<min
        min=arr[j];
      }else {
        if(arr[j]>max){
          amount1+=max;    //arr>max
          max=arr[j];
        }else{
          amount1+=arr[j]; //min<=arr<=max
        }
      }//if
    }//for
    avg_orp = (double)amount1/(number-2);
  }//if
  return avg_orp;
}

//************************************************************

//#define DoSensorPin  35     //dissolved oxygen sensor analog output pin to arduino mainboard
#define VREF 3300    //for arduino uno, the ADC reference is the AVCC, that is 5000mV(TYP)
float doValue;      //current dissolved oxygen value, unit; mg/L
float temperature = 25;    //default temperature is 25^C, you can use a temperature sensor to read it

#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength+1];    // store the serial command
byte receivedBufferIndex = 0;

#define SCOUNT  30           // sum of sample point
int16_t analogBuffer[SCOUNT];    //store the analog value in the array, readed from ADC
int16_t analogBufferTemp[SCOUNT];
int16_t analogBufferIndex = 0,copyIndex = 0;

#define SaturationDoVoltageAddress 12          //the address of the Saturation Oxygen voltage stored in the EEPROM
#define SaturationDoTemperatureAddress 16      //the address of the Saturation Oxygen temperature stored in the EEPROM
float SaturationDoVoltage,SaturationDoTemperature;
float averageVoltage;

const float SaturationValueTab[41] PROGMEM = {      //saturation dissolved oxygen concentrations at various temperatures
14.46, 14.22, 13.82, 13.44, 13.09,
12.74, 12.42, 12.11, 11.81, 11.53,
11.26, 11.01, 10.77, 10.53, 10.30,
10.08, 9.86,  9.66,  9.46,  9.27,
9.08,  8.90,  8.73,  8.57,  8.41,
8.25,  8.11,  7.96,  7.82,  7.69,
7.56,  7.43,  7.30,  7.18,  7.07,
6.95,  6.84,  6.73,  6.63,  6.53,
6.41,
};



void setup()
{
    Serial.begin(115200);
  //  pinMode(DoSensorPin,INPUT);
    readDoCharacteristicValues();      //read Characteristic Values calibrated from the EEPROM

    SerialGPS.begin(9600, SERIAL_8N1, 12, 15);   //17-TX 18-RX
    sensors.begin();

//  Serial.println("Getting single-ended readings from AIN0..3");
//  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  
   //                                                                 ADS1015  ADS1115
  //                                                                  -------  -------
   ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.begin();

    
    pinMode(25, OUTPUT); //Send success, LED will bright 1 second
    while (!Serial);
    pinMode(16, OUTPUT);
    digitalWrite(14, LOW);    // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(14, HIGH); // while OLED is running, must set GPIO16 in high
    
    Serial.println("LoRa Sender");    
    SPI.begin(5, 19, 27, 18); 
    LoRa.setPins(SS, RST, DI0);   
    if (!LoRa.begin(923E6)) {
      Serial.println("Starting LoRa failed!");
      while (1); 
      
    }
    
    Serial.println("LoRa Initial OK!");
    // Initialising the UI will init the display too.
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
  }

void loop()
{
  int16_t adc0, adc1, adc2;
  float volt0, volt1, volt2;
//  adc0 = ads.readADC_SingleEnded(0);
//  volt0 = adc0*0.000188;
  
//  adc1 = ads.readADC_SingleEnded(1);
//  volt1 = adc1*0.000188;

//  Serial.print("AIN0: ");
//  Serial.print(adc0);
//  Serial.print(" ");
//  Serial.print(volt0, 4);
//  Serial.println(" vdc");
//  Serial.print("AIN1: ");
//  Serial.print(adc1);
//  Serial.print(" ");
//  Serial.print(volt1, 4);
//  Serial.println(" vdc");
//  Serial.print("AIN2: ");
//  Serial.print(adc2);
//  Serial.print(" ");
//  delay(2000);
  
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 30U)     //every 30 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = ads.readADC_SingleEnded(2);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }
   
   static unsigned long tempSampleTimepoint = millis();
   if(millis()-tempSampleTimepoint > 500U)  // every 500 milliseconds, read the temperature
   {
      tempSampleTimepoint = millis();
      //temperature = readTemperature();  // add your temperature codes here to read the temperature, unit:^C
   }
   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 1000U)
   {

      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
      {
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      }

      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT)*0.1875; // read the value more stable by the median filtering algorithm
//      Serial.print(F("Temperature:"));
//      Serial.print(temperature,1);
//      Serial.print(F("^C"));

     

      doValue = pgm_read_float_near( &SaturationValueTab[0] + (int)(SaturationDoTemperature+0.5) ) * averageVoltage / SaturationDoVoltage;  //calculate the do value, doValue = Voltage / SaturationDoVoltage * SaturationDoValue(with temperature compensation)
      Serial.print(F("DO Value: "));
      Serial.print(doValue,2);
      Serial.println(F("mg/L"));
      
/**********************************************************************************************************************/
              //PH SENSOR
               
              static unsigned long samplingTime = millis();
              static unsigned long printtime = millis();
             float pHValue,voltage;
              if(millis()-samplingTime > samplingInterval)
              {
                  pHArray[pHArrayIndex++]= ads.readADC_SingleEnded(0);
                  orpArray[pHArrayIndex++]= ads.readADC_SingleEnded(1);    //read an analog value every 20ms
                  if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
                  
                  voltage = Avergearray(pHArray, ArrayLenth)*0.000188;
                  pHValue = 3.5*voltage+Offset;
                  orpValue=((30*(double)VOLTAGE*1000)-(75*Avergearray(orpArray, ArrayLenth)*0.1875))/75-OFFSET; 
                  samplingTime=millis();
              }
              if(millis() - printtime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
              {
 
                Serial.print("pH : ");
                Serial.println(pHValue,2);

                 Serial.print("ORP: ");
                 Serial.print((int)orpValue);
                 Serial.println("mV");
                    printtime=millis();
  }
            
            // Temp SENSOR
              sensors.requestTemperatures(); // Send the command to get temperatures
              TempValue = sensors.getTempC(Temp);
//              String thistemp = String(Temp1,HEX);
              Serial.print("Temp (*C)= ");
              Serial.print(TempValue); 
              Serial.print(" Temp (*F)= ");
              Serial.println(sensors.getTempF(Temp)); 
                          
            
//            // ORP SENSOR
//              static unsigned long orpTimer=millis();   //analog sampling interval
//              static unsigned long printTime=millis();
//              if(millis() >= orpTimer)
//              {
//                orpTimer=millis()+20;
//            //    orpArray[orpArrayIndex++]=analogRead(orpPin);    //read an analog value every 20ms
//                orpArray[orpArrayIndex++]= ads.readADC_SingleEnded(1);    //read an analog value every 20ms
//                if (orpArrayIndex==ArrayLenth) {
//                  orpArrayIndex=0;
//                }   
//               orpValue=((30*(double)VOLTAGE*1000)-(75*avergearray(orpArray, ArrayLenth)*0.1875))/75-OFFSET; 
//            
//                //convert the analog value to orp according the circuit
//              }
//              if(millis() >= printTime)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
//              {
//  
//
//              printTime=millis()+800;
//              Serial.print("ORP: ");
//              Serial.print((int)orpValue);
//               Serial.println("mV");
//  }


            // GPS
              Serial.print("Latitude  : ");
              Serial.println(gps.location.lat(), 5);
              Serial.print("Longitude : ");
              Serial.println(gps.location.lng(), 5);
          
                                   
              smartDelay(1000);
            
              if (millis() > 5000 && gps.charsProcessed() < 10)
                Serial.println(F("No GPS data received: check wiring"));
                delay(2000);
     
//                latt = 13.27598;
//                lon  = 100.92276;
             latt = gps.location.lat();
             lon  = gps.location.lng();



                
            Serial.println(packet_txt(pHValue,TempValue,orpValue,doValue,latt,lon));

              delay(1000);

              
              // send packet
              LoRa.beginPacket();
              LoRa.print(packet_txt(pHValue,TempValue,orpValue,doValue,latt,lon));
              LoRa.endPacket();
            
              Serial.print("Send '");
              Serial.print(packet_txt(pHValue,TempValue,orpValue,doValue,latt,lon));
              Serial.println("'");
              Serial.println("**********************");
              delay(1000);
              
              display.clear();
              display.setTextAlignment(TEXT_ALIGN_LEFT);
              display.drawString(10, 5, "Sending:");
              display.drawString(10, 20, "packet " + String(counter));
              display.display();        // write the buffer to the display

              
              digitalWrite(25, HIGH);   // turn the LED on (HIGH is the voltage level)
              delay(1000);                       // wait for a second
              digitalWrite(25, LOW);    // turn the LED off by making the voltage LOW
              delay(1000);                       // wait for a second


/**********************************************************************************************************************/      
   }
   
   if(serialDataAvailable() > 0)
   {
      byte modeIndex = uartParse();  //parse the uart command received
      doCalibration(modeIndex);    // If the correct calibration command is received, the calibration function should be called.
   }
   
}

boolean serialDataAvailable(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while ( Serial.available() > 0 ) 
  {   
    if (millis() - receivedTimeOut > 500U) 
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer,0,(ReceivedBufferLength+1));
    }
    receivedTimeOut = millis();
    receivedChar = Serial.read();
    if (receivedChar == '\n' || receivedBufferIndex == ReceivedBufferLength)
    {
  receivedBufferIndex = 0;
  strupr(receivedBuffer);
  return true;
    }else{
        receivedBuffer[receivedBufferIndex] = receivedChar;
        receivedBufferIndex++;
    }
  }
  return false;
}

byte uartParse()
{
    byte modeIndex = 0;
    if(strstr(receivedBuffer, "CALIBRATION") != NULL) 
        modeIndex = 1;
    else if(strstr(receivedBuffer, "EXIT") != NULL) 
        modeIndex = 3;
    else if(strstr(receivedBuffer, "SATCAL") != NULL)   
        modeIndex = 2;
    return modeIndex;
}

void doCalibration(byte mode)
{
    char *receivedBufferPtr;
    static boolean doCalibrationFinishFlag = 0,enterCalibrationFlag = 0;
    float voltageValueStore;
    switch(mode)
    {
      case 0:
      if(enterCalibrationFlag)
         Serial.println(F("Command Error"));
      break;
      
      case 1:
      enterCalibrationFlag = 1;
      doCalibrationFinishFlag = 0;
      Serial.println();
      Serial.println(F(">>>Enter Calibration Mode<<<"));
      Serial.println(F(">>>Please put the probe into the saturation oxygen water! <<<"));
      Serial.println();
      break;
     
     case 2:
      if(enterCalibrationFlag)
      {
         Serial.println();
         Serial.println(F(">>>Saturation Calibration Finish!<<<"));
         Serial.println();
         EEPROM_write(SaturationDoVoltageAddress, averageVoltage);
         EEPROM_write(SaturationDoTemperatureAddress, temperature);
         SaturationDoVoltage = averageVoltage;
         SaturationDoTemperature = temperature;
         doCalibrationFinishFlag = 1;
      }
      break;

        case 3:
        if(enterCalibrationFlag)
        {
            Serial.println();
            if(doCalibrationFinishFlag)      
               Serial.print(F(">>>Calibration Successful"));
            else 
              Serial.print(F(">>>Calibration Failed"));       
            Serial.println(F(",Exit Calibration Mode<<<"));
            Serial.println();
            doCalibrationFinishFlag = 0;
            enterCalibrationFlag = 0;
        }
        break;
    }
}

int16_t getMedianNum(int16_t bArray[], int iFilterLen) 
{
      int16_t bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
    bTab[i] = bArray[i];
      }
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
    for (i = 0; i < iFilterLen - j - 1; i++) 
          {
      if (bTab[i] > bTab[i + 1]) 
            {
    bTemp = bTab[i];
          bTab[i] = bTab[i + 1];
    bTab[i + 1] = bTemp;
       }
    }
      }
      if ((iFilterLen & 1) > 0)
  bTemp = bTab[(iFilterLen - 1) / 2];
      else
  bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void readDoCharacteristicValues(void)
{
    EEPROM_read(SaturationDoVoltageAddress, SaturationDoVoltage);  
    EEPROM_read(SaturationDoTemperatureAddress, SaturationDoTemperature);
    if(EEPROM.read(SaturationDoVoltageAddress)==0xFF && EEPROM.read(SaturationDoVoltageAddress+1)==0xFF && EEPROM.read(SaturationDoVoltageAddress+2)==0xFF && EEPROM.read(SaturationDoVoltageAddress+3)==0xFF)
    {
      SaturationDoVoltage = 1127.6;   //default voltage:1127.6mv
      EEPROM_write(SaturationDoVoltageAddress, SaturationDoVoltage);
    }
    if(EEPROM.read(SaturationDoTemperatureAddress)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+1)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+2)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+3)==0xFF)
    {
      SaturationDoTemperature = 25.0;   //default temperature is 25^C
      EEPROM_write(SaturationDoTemperatureAddress, SaturationDoTemperature);
    }    
}
static void smartDelay(unsigned long ms)             
{
  unsigned long start = millis();
  do
  {
    while (SerialGPS.available())
      gps.encode(SerialGPS.read());
  } while (millis() - start < ms);
}

String packet_txt(float pH_val,int temp_val,int ORP_val,float DO_val,float LAT_val,float LON_val){
  String pck = PH(pH_val)+TEMP(temp_val)+ORP(ORP_val)+DO(DO_val)+LAT(LAT_val)+LON(LON_val);
  return pck;
}

String PH(float pH_val){
//  return String(pH_val,HEX); 

  int ph_adapt = pH_val*100.00f;
  String preText;
  String data;
  if(String(ph_adapt,HEX).length()<3){
    for(int i = 0;i<(3-String(ph_adapt,HEX).length());i++){
      preText += "0";
    }
  }
  data = preText+String(ph_adapt,HEX);
  return data; // 3 Bytes
}

String TEMP(int temp_val){
  String preText;
  String data;
  if(String(temp_val,HEX).length()<2){
    for(int i = 0;i<(2-String(temp_val,HEX).length());i++){
      preText += "0";
    }
  }
  data = preText+String(temp_val,HEX);
  return data; // 2 Byte
}

String ORP(int ORP_val){
  String dat;
  String preText;
  String data;
  if(String(abs(ORP_val),HEX).length()<3){
    for(int i = 0;i<(3-String(abs(ORP_val),HEX).length());i++){
      preText += "0";
    }
  }
  data = preText+String(abs(ORP_val),HEX);
  
  if(ORP_val<0){
    dat = "0"+data;
    
  }else{
    dat = "1"+data;
  }
  return dat ; // 3 Bytes
}
String DO(float DO_val){
  int do_adapt = DO_val*100.00f;
  String preText;
  String data;
  if(String(do_adapt,HEX).length()<3){
    for(int i = 0;i<(3-String(do_adapt,HEX).length());i++){
      preText += "0";
    }
  }
  data = preText+String(do_adapt,HEX);
  return data; // 3 Bytes
}
String LAT(float LAT_val){
  int lat_adapt = LAT_val*100000;
  String preText;
  String data;
  if(String(lat_adapt,HEX).length()<6){
    for(int i = 0;i<(6-String(lat_adapt,HEX).length());i++){
      preText += "0";
    }
  }
  data = preText+String(lat_adapt,HEX);
  return data;// 6 Bytes
}
String LON(float LON_val){
  int lon_adapt = LON_val*100000;
  String preText;
  String data;
  if(String(lon_adapt,HEX).length()<6){
    for(int i = 0;i<(6-String(lon_adapt,HEX).length());i++){
      preText += "0";
    }
  }
  data = preText+String(lon_adapt,HEX);
  return data;// 6 Bytes
}
String Depacket(String pck,uint8_t x){
  String dePCK[6];
  dePCK[0] = pck.substring(0,3);
  dePCK[1] = pck.substring(3,5);
  dePCK[2] = pck.substring(5,9);
  dePCK[3] = pck.substring(9,12);
  dePCK[4] = pck.substring(12,18);
  dePCK[5] = pck.substring(18,24);
  
  return dePCK[x];
}
unsigned int hexToDec(String hexString) {
  
  unsigned int decValue = 0;
  int nextInt;
  
  for (int i = 0; i < hexString.length(); i++) {
    
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9); //check 0-9 ascii in DEC
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15); //check A-F ascii in DEC
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15); //check a-f ascii in DEC
    nextInt = constrain(nextInt, 0, 15);
    
    decValue = (decValue * 16) + nextInt;
  }
  
  return decValue;
}

