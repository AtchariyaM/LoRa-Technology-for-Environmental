#include <ETH.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiSTA.h>
#include <WiFiType.h>
#include <WiFiUdp.h>

#include <SPI.h>
#include <LoRa.h>

#include <time.h>

#include <IOXhop_FirebaseESP32.h>
#include "WiFi.h"


#define FIREBASE_HOST "https://project-708ec.firebaseio.com/"
#define FIREBASE_AUTH "QonY2FV4TU5mNfFPVUrMXDdHEunB0BKgA147lFBB"
//#define WIFI_SSID "Ppooky"
//#define WIFI_PASSWORD "p12341234"

//#define WIFI_SSID "YuTTYL-AP"
//#define WIFI_PASSWORD "0810191902Ton"f

#define WIFI_SSID "Mn"
#define WIFI_PASSWORD "0928604039mon"

#define SS      18
#define RST     23
#define DI0     26
#define BAND    923E6

 float pH = 0.00f;           //กำหนดตัวแปรเก็บค่าอุณหภูมิ
 int Temp = 0;
 int ORP = 0;
 float DO = 0.00f;
 float latt = 0.00000f;
 float lon = 0.00000f;

char ntp_server1[20] = "time.navy.mi.th";
char ntp_server2[20] = "clock.nectec.or.th";
char ntp_server3[20] = "th.pool.ntp.org";

void setup() {
  Serial.begin(115200);
  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);

    }
  configTime(7 * 3600, 0, ntp_server1, ntp_server2, ntp_server3);
  Serial.println("Waiting for time");
  while (!time(nullptr)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("Now: " + NowString());
  while (!Serial);

  Serial.println("LoRa Receiver");

   LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(923E6)) {
    Serial.println("Starting LoRa failed!");
    while(1);
  }
  LoRa.setTimeout(100);
  
  Serial.println();
  Serial.println("connected: ");
  Serial.println(WiFi.localIP());
  
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);



}
String Depacket(String pck,uint8_t x){
  String dePCK[6];
  dePCK[0] = pck.substring(0,3);
  dePCK[1] = pck.substring(3,5);
  dePCK[2] = pck.substring(5,9);
  dePCK[3] = pck.substring(9,12);
  dePCK[4] = pck.substring(12,18);
  dePCK[5] = pck.substring(18,24);
//  dePCK[6] = pck.substring(3,4);
    
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

void loop() { 
  
 if (LoRa.parsePacket() > 0) {
    String packet = LoRa.readString();
    Serial.print("Receiver '");
    Serial.print(packet);
    Serial.print("' RSSI: ");
    Serial.println(LoRa.packetRssi());
    
   String msg =  String(packet);
   
  pH = hexToDec(Depacket(msg,0))/100.00f;
  Temp = hexToDec(Depacket(msg,1));
//  if(Depacket(msg,2)=="0"){
//     ORP = 0-(hexToDec(Depacket(msg,2)));
//  }else if(Depacket(msg,2)=="1"){
//     ORP = hexToDec(Depacket(msg,2));
//  }

  if(Depacket(msg,2).substring(0,1)=="0"){
     ORP = 0-(hexToDec(Depacket(msg,2).substring(1)));
  }else if(Depacket(msg,2).substring(0,1)=="1"){
     ORP = hexToDec(Depacket(msg,2).substring(1));
  }
  
  DO = hexToDec(Depacket(msg,3))/100.00f;
  latt = hexToDec(Depacket(msg,4))/100000.00000f;
  lon = hexToDec(Depacket(msg,5))/100000.00000f;
  
  Serial.print("pH: ");
  Serial.print(pH);
  
  Serial.print(" Temperature: ");
  Serial.print(Temp);
  Serial.print(" *C ");
  
  Serial.print("ORP: ");
  Serial.print(ORP);

  Serial.print(" DO: ");
  Serial.print(DO);
  Serial.print(" mg/L ");
  
  Serial.print("latt: ");
  Serial.print(latt,5);
  Serial.print(" * ");
  Serial.print("lon: ");
  Serial.print(lon,5);
  Serial.print(" * "); 
  Serial.println();

  
 StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["pH"] = pH;
  root["temperature"] = Temp;
  root["ORP"] = ORP;
  root["DO"] = DO; 
  root["latt"] = String(latt,5);
  root["lon"] = String(lon,5);
  root["time"] = NowString();

// append a new value to /logDHT
  String name = Firebase.push("WATERData", root);
 
// handle error
  if (Firebase.failed()) {
      Serial.print("pushing /logWATER failed:");
      Serial.println(Firebase.error());  
      return;
  }
  
  Serial.print("pushed: /logWATER/");
  Serial.println(name);
   }
  //delay(5000);
  
 // delay(1000);
  // put your main code here, to run repeatedly:
//Firebase.setString("message", "hello world");
}
String NowString() {
  time_t now = time(nullptr);
  struct tm* newtime = localtime(&now);

  String tmpNow = "";
  tmpNow += String(newtime->tm_year + 1900);
  tmpNow += "-";
  tmpNow += String(newtime->tm_mon + 1);
  tmpNow += "-";
  tmpNow += String(newtime->tm_mday);
  tmpNow += " ";
  tmpNow += String(newtime->tm_hour);
  tmpNow += ":";
  tmpNow += String(newtime->tm_min);
  tmpNow += ":";
  tmpNow += String(newtime->tm_sec);
  return tmpNow;
}
