float ph = 7.45f;
int tmp = 40;
int orp = -225;
float Do = 1.01f;
float lat  = 5.8909461f;
float lon  = 102.7690833f;
void setup() {
  Serial.begin(9600);
  
}

void loop() {
  String packetData = packet_txt(ph,tmp,orp,Do,lat,lon);
  float ph_data = hexToDec(Depacket(packetData,0))/100.00f;
  int temp_data = hexToDec(Depacket(packetData,1));
//  Serial.println(Depacket(packetData,2).substring(0,1));
  int orp_data;
  if(Depacket(packetData,2).substring(0,1)=="0"){
    orp_data = 0-(hexToDec(Depacket(packetData,2).substring(1)));
  }else{
    orp_data = hexToDec(Depacket(packetData,2).substring(1));
  }
  float do_data = hexToDec(Depacket(packetData,3))/100.00f;
  float lat_data = hexToDec(Depacket(packetData,4))/100000.00000f;
  float lon_data = hexToDec(Depacket(packetData,5))/100000.00000f;
  Serial.println(packetData);
//long number = hexToDec(lat_data);
  Serial.print("PH :");Serial.println(ph_data,2);
  Serial.print("TEMP :");Serial.println(temp_data);
  Serial.print("ORP :");Serial.println(orp_data);
  Serial.print("DO :");Serial.println(do_data,2);
  Serial.print("Lat :");Serial.println(lat_data,5);
  Serial.print("Lon :");Serial.println(lon_data,5);

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

//  dePCK[0] = pck.substring(0,1);
//  dePCK[1] = pck.substring(1,3);
//  dePCK[2] = pck.substring(3,7);
//  dePCK[3] = pck.substring(7,10);
//  dePCK[4] = pck.substring(10,16);
//  dePCK[5] = pck.substring(16,22);
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

