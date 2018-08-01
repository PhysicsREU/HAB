///////////////////////////////////////
// High Altitude Balloon Sensor Code //
///////////////////////////////////////
//REU Members - Rachel Greenland, Anne-Katherine Burns, Ruby O'Brien Metzger, Valeria Rascon, Amy Pierce, Tru Quach, Brian Hollman, 
//Anthony Holman, Miguel Castro, Sean Dougall, Ryan Iuliano, Brandon Mishler

//////////////////////////////////////////////////////////
//                Legend                                //    
// # | Sensor                 | Memory  | PINS          //
//------------------------------------------------------//  
// 1 | Humidity               | 12%     | D7            //
// 2 | Temp/Pressure/Altitude | 24%     | A4,A5         //
// 3 | Accelerometer          |  7%     | A1,A2,A3      //
// 4 | Geiger Counter         |  7%     | A0            //
// 5 | GPS                    |         |               //
// 6 | SD Card                |         |(SPI Pins) + 4 //
//////////////////////////////////////////////////////////
#include <SimpleDHT.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include  <SPI.h>  
#include <SD.h>
//GPS library - best I could find - very lightweight and can be used with Uno or Due
#include <NMEAGPS.h>
#include <GPSport.h>

//Humidity Initial//
//      VCC: 5V or 3V
//      GND: GND
//      DATA: 2
int pinDHT11 = 2;
SimpleDHT11 dht11;
int err = 0;

//Accel Initial//
int axpin=11; 
int aypin=10;
int azpin=9;
float x1 = 615;
float x2= 401;
float Mx=19.6/(x1-x2);
float Bx=-Mx*(x1+x2)*0.5;
float Y1 = 603;
float Y2= 401;
float My=19.6/(Y1-Y2);
float By=-My*(Y1+Y2)*0.5;
float z1 = 617;
float z2= 409;
float Mz=19.6/(z1-z2);
float Bz=-Mz*(z1+z2)*0.5;
  
//Relevant variables
float Lat, Long, Speed, Alt_bmp, Alt_GPS, Temp, Press, AX, AY, AZ, GeigerSignal;
byte temperature_DHT = 0;
byte humidity = 0;
//Temp/Pressure/Altitude Initial//
Adafruit_BMP085 bmp;     // object for the thermometer

//Timing variables
int save_file_every = 60000;    // save file after this amount of time (min)
int dum_cnt=1;
int dum_cnt2=1;
int dum_cnt3=1;
float detection_time=0.0;
float last_detection = 0.0;

//Geiger Counter Initial//
int  counts=0;             
float t0=0.0;
//GPS Initial//
NMEAGPS  gps;                   // Call the GPS module gps. This parses the GPS characters
gps_fix  fix;                   // This holds on to the latest GPS values

File DATFILE;
void setup(){
  Serial.begin(115200);
  pinMode(A0, INPUT);      //Pin for the Geiger counter
  pinMode(axpin, INPUT);      //Pin for x component of acceleration
  pinMode(aypin, INPUT);      //Pin for y component of acceleration
  pinMode(azpin, INPUT);      //Pin for z component of acceleration
  pinMode(2,  INPUT);      

  bmp.begin();             //Start thermomemter

  gpsPort.begin(9600);
if(!gps.available(gpsPort) && millis()>5000){
    Serial.println("No GPS device deteceted. Check wiring");
}
else{
    Serial.println("GPS WORKING");
}

// Open data file to write to -- make sure CS pin on SD card is on Pin 4
if (!SD.begin(4)) {
    Serial.println("Could not find a valid memory device, check wiring!");while(1){}
}
else{Serial.println("MEM WORKING - OPENING FILE...");
  DATFILE=SD.open("HAB_data.txt", FILE_WRITE);
  DATFILE.println("#[1]TIMESTAMP [2]COUNTS [3]GEIGER(V) [4]LAT [5]LONG [6]ALT_GPS(m) [7]ALT_BMP(m) [8]AX(m/s^2) [9]AY(m/s^2) [10]AZ(m/s^2) [11]SPEED(m/s) [12]PRESS(kPa) [13]TEMP_BMP(C) [14]TEMP_DHT(C) [15]HUMIDITY(H)");
  DATFILE.close();
  DATFILE=SD.open("HAB_data.txt", FILE_WRITE);
}
 
Serial.println("#[1]TIMESTAMP [2]COUNTS [3]GEIGER(V) [4]LAT [5]LONG [6]ALT_GPS(m) [7]ALT_BMP(m) [8]AX(m/s^2) [9]AY(m/s^2) [10]AZ(m/s^2) [11]SPEED(m/s) [12]PRESS(kPa) [13]TEMP_BMP(C) [14]TEMP_DHT(C) [15]HUMIDITY(H)");
delay(2000);
t0=millis();

}



void loop()
{  

float timestamp = millis()/1000.;
///////////////////////
// 1. Geiger Counter //
///////////////////////
float GeigerSignal = analogRead(A0);
//Serial.println(GeigerSignal);
//
if (GeigerSignal < 100){
  counts ++;
  float dmill = millis() - t0;

  //update sensors every 10 seconds
  if(millis()>10000*dum_cnt2){
    /////////////////
    // 1. Humidity //
    /////////////////
    err = dht11.read(pinDHT11, &temperature_DHT, &humidity, NULL);
    ///////////////////////////////////////////
    // 2. Temperature, Pressure, & Altitude //
    ///////////////////////////////////////////
    Temp = bmp.readTemperature();
    Press = bmp.readPressure();
    Alt_bmp = bmp.readAltitude();
    //////////////////////
    // 3. Accelerometer //
    //////////////////////
    float x=(analogRead(axpin));
    AX=Mx*x + Bx;
    float y=analogRead(aypin);
    AY=My*y+By;
    float z=(analogRead(azpin));
    AZ=Mz*z+Bz;
    dum_cnt2 ++;
    //Serial.println("UPDATED SENSORS");
  }
  //update GPS every 10 counts
  if(counts > 10*dum_cnt3){
    ////////////////////
    // 4. GPS         //
    ////////////////////  
    fix = gps.read();
    Lat=fix.latitude();
    Long=fix.longitude();
    Alt_GPS=fix.altitude();  
    Speed=fix.speed();
    dum_cnt3++;
    //Serial.println("UPDATED GPS");
  }

  ////////////////
  // 5. SD Card //
  //////////////// 
  DATFILE.print(timestamp);
  DATFILE.print(" ");
  DATFILE.print(counts);
  DATFILE.print(" ");
  DATFILE.print(GeigerSignal);
  DATFILE.print(" ");
  DATFILE.print(Lat);
  DATFILE.print(" ");
  DATFILE.print(Long);
  DATFILE.print(" ");
  DATFILE.print(Alt_GPS);
  DATFILE.print(" ");
  DATFILE.print(Alt_bmp);
  DATFILE.print(" ");
  DATFILE.print(AX);
  DATFILE.print(" ");
  DATFILE.print(AY);
  DATFILE.print(" ");
  DATFILE.print(AZ);
  DATFILE.print(" ");
  DATFILE.print(Speed);
  DATFILE.print(" ");
  DATFILE.print(Press);
  DATFILE.print(" ");
  DATFILE.print(Temp);
  DATFILE.print(" ");
  DATFILE.print(temperature_DHT);
  DATFILE.print(" ");
  DATFILE.println(humidity);

  Serial.print(timestamp);
  Serial.print(" ");
  Serial.print(counts);
  Serial.print(" ");
  Serial.print(GeigerSignal);
  Serial.print(" ");
  Serial.print(Lat);
  Serial.print(" ");
  Serial.print(Long);
  Serial.print(" ");
  Serial.print(Alt_GPS);
  Serial.print(" ");
  Serial.print(Alt_bmp);
  Serial.print(" ");
  Serial.print(AX);
  Serial.print(" ");
  Serial.print(AY);
  Serial.print(" ");
  Serial.print(AZ);
  Serial.print(" ");
  Serial.print(Speed);
  Serial.print(" ");
  Serial.print(Press);
  Serial.print(" ");
  Serial.print(Temp);
  Serial.print(" ");
  Serial.print(temperature_DHT);
  Serial.print(" ");
  Serial.println(humidity);

//  if(dmill >= 60000){
//    Serial.println(" ");
//    Serial.print(dmill);
//    Serial.print(" ");
//    Serial.println(counts/(dmill/60000));
//    Serial.println(" ");
//  }
  
}  

/*SAVE FILE EVERY X SECONDS*/
    if((DATFILE) and (millis()>save_file_every*dum_cnt)){
      dum_cnt++;
      DATFILE.close();
      DATFILE=SD.open("HAB_data.txt", FILE_WRITE);

    }
}
