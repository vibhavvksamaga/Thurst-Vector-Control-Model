#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <SFE_BMP180.h>
const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

SFE_BMP180 pressure;
TinyGPSPlus gps;
#define ALTITUDE 886.0
Servo servox,servoy;
Adafruit_MPU6050 mpu;
void setup(void) {

//The serial monitor is begin
  Serial.begin(115200);

// This code sets the servo motor PIN
  servoy.attach(18);
  servox.attach(23);

//The wire library is begin
  Wire.begin();

//The MPU6050 sensor is begin
  mpu.begin();

// Servo motor turns to starting position
  servox.write(0);
  servoy.write(0);

//These code lines set the accelerometer and gyroscope ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);//2_G,4_G,8_G,16_G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);//250,500,1000,2000
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {

// Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

// Accelerator Y-axis values are put into the variable
  int valuex = a.acceleration.x;
  int valuey= a.acceleration.y;
  int valuez= a.acceleration.z;
  int valuegx = g.gyro.x;
  int valuegy= g.gyro.y;
// This value is converted from 0 to 180
  valuex = map(valuex,  -10, 10, 180, 0);
    valuey = map(valuey,  -10, 10, 180, 0);

//This code line rotate the servo motor using that value
  servox.write(valuex);
  servoy.write(valuey);
// That value is printed on the serial monitor
  //Serial.println("Acceleration of X : Acceleration of Y");
  Serial.print(a.acceleration.x);
  Serial.print(" , ");
  Serial.print(a.acceleration.y);
  Serial.print(" , ");
  //Serial.println("Gyro of X : Gyro of Y");
  Serial.print(g.gyro.x);
  Serial.print(" , ");
  Serial.print(g.gyro.y);
  //Serial.println("Angle X : Angle Y");
  Serial.print(" , ");
  Serial.print(valuex);
  Serial.print(" , ");
  Serial.print(valuey);
  Serial.print(" , ");
  //Above commented because of unnecessary printing
  delay(5);
  bmpSensor();
  delay(5);
  gpsNav();

}
void bmpSensor()
{
  //Wire.begin();//To begin the connection
  if (pressure.begin())
  {
    //Serial.println("BMP180 init success");
  }
  else
  {
    //This is usually a connection problem,
    Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.
  }
  char status;
  double T,P,p0,a;
  // We're using a constant called ALTITUDE(Altitude at your specific location) in this sketch:
  
  //Serial.println();
  //Serial.print("provided altitude: ");
  Serial.print(ALTITUDE,0);
  //Serial.print(" meters, ");
  Serial.print(" , ");
  Serial.print(ALTITUDE*3.28084,0);
  //Serial.println(" feet");
   Serial.print(" , ");
  
  // If you want to measure altitude, and not pressure, you will instead need
  // to provide a known baseline pressure. This is shown at the end of the sketch.

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      //Serial.print("temperature: ");
      Serial.print(T,2);
      //Serial.print(" deg C, ");
      Serial.print(" , ");
      Serial.print((9.0/5.0)*T+32.0,2);
      //Serial.println(" deg F");
       Serial.print(" , ");
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          //Serial.print("absolute pressure: ");
          Serial.print(P,2);
          //Serial.print(" mb, ");
           Serial.print(" , ");
          Serial.print(P*0.0295333727,2);
          //Serial.println(" inHg");
           Serial.print(" , ");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
          //Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0,2);
          //Serial.print(" mb, ");
           Serial.print(" , ");
          Serial.print(p0*0.0295333727,2);
           Serial.print(" , ");
          //Serial.println(" inHg");

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          a = pressure.altitude(P,p0);
          //Serial.print("computed altitude: ");
          Serial.print(a,0);
           Serial.print(" , ");
          //Serial.print(" meters, ");
          Serial.print(a*3.28084,0);
          //Serial.println(" feet");
          Serial.println();
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");  // Pause for 1 seconds.
 // Wire.endTransmission();
}
void gpsNav()
{
  while (*gpsStream)
    if (gps.encode(*gpsStream++))
      displayInfo();
}
void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
