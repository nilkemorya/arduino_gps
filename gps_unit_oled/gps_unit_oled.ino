// Code to make a GPS clock/altimeter/speedometer using MTK3329/MTK3339 driver
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset, as well as the Adafruit 128x64 i2c/SPI oled screen
// and the LMS303 Magnetometer chip.

//This code is intended for use with Arduino Mega

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Adafruit_GFX.h"
#include <TimeLib.h>        //http://www.arduino.cc/playground/Code/Time
#include <Timezone.h>    //https://github.com/JChristensen/Timezone
#include <avr/wdt.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#define OLED_RESET 4 // Specify reset pin for the OLED display.
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

const int buttonPin = 2;  //Define the button to push for changing state
const int button2Pin = 3; //Define the button to push for setting your waypoint
int buttonState;
int button2State;
int gps_year;
int gps_day;
int gps_month;
int gps_hour;
int gps_minute;
int trouble_count = 0;
int display_selector = 0;
bool firstrun = true;
bool display_switch = false;
float storedLat;
float storedLong;
float tripLat;
float tripLong;
float time_average_distance = 0;
int prev_dist_time = 0;
float time_average_distance_waypoint = 0;
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);


void softwareReset( uint8_t prescaller) {
  // start watchdog with the provided prescaller
  wdt_enable( prescaller);
  // wait for the prescaller time to expire
  // without sending the reset signal by using
  // the wdt_reset() method
  while(1) {}
}

float calc_dist(float flat1, float flon1, float flat2, float flon2)
/*Distance calculator which uses 2 lat long sets and gives the linear distance between them. Uses the haversine equation. Inspired by many internet sources.
inputs:
  flat1: Decimal Latitude for your first point
  flon1: Decimal Longitude for your first point
  flat2: Decimal Latitude for your second point
  flon2: Decimal Longitude for your second point
 outputs:
  dist_calc: Float distance in miles between your two points.*/
{
float dist_calc=0;
float dist_calc2=0;
float diflat=0;
float diflon=0;

//I've to split all the calculation in several steps for clarity.
diflat=radians(flat2-flat1);
diflon=radians((flon2)-(flon1));
flat1=radians(flat1);
flat2=radians(flat2);

dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
dist_calc2= cos(flat1);
dist_calc2*=cos(flat2);
dist_calc2*=sin(diflon/2.0);
dist_calc2*=sin(diflon/2.0);
dist_calc +=dist_calc2;

dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

dist_calc*=3963.0; //Converting to miles
return dist_calc;
}

float latLongPoint(float baseLat, float baseLon, float targetLat, float targetLon)
/* This function will take two sets of coordinates then calculate the angle from north of the line between them.
 *  inputs:
 *    baseLat: decimal Latitude of your zero point
 *    baseLon: decimal Longitude of your zero point
 *    targetLat: decimal Latitude of your Target
 *    targetLon: decimal Longitude of your Target
 *  outputs:
 *    theta: Decimal angle degrees between your two points referenced to compass North.
*/
{
  //Define Local variables
  float latDiff, lonDiff, theta;
  double servoPosition;

  //Calculate difference
  latDiff = (targetLat - baseLat);
  lonDiff = (targetLon - baseLon);
  theta =   atan2 (lonDiff, latDiff);
  //Calculate angle, correcting for whichever quadrant your target point is in.
  if (lonDiff > 0 and latDiff >0){
  theta = theta * 180.0 / 3.142;
  }
  else if ((lonDiff > 0 and latDiff < 0) or (lonDiff < 0 and latDiff < 0)){
    theta = (theta * 180.0 / 3.142) + 180;
  }
  else {
    theta = (theta * 180.0 / 3.142) + 360;
  }
  return theta;

}

float compassPoint()
/*This function uses a 3 axis magnetometer to generate a simple compass heading based on magnetic north.
 * 
 *  Inputs: None!
 *  
 *  Outputs: 
 *    heading: Decimal degrees from north as reported by your magnetometer.
*/
{
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / 3.142;

  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  return heading;
}

float knotsToMph(float knots){
  return knots / 1.15078;
}

void drawCompass(int startX, int startY, int total_length, float theta_deg)
/* This function will draw a pointer line for you using a theta angle.*/
{
  /* Inputs:
   *    StartX -> Pixel X location to start line
   *    StartY -> Pixel Y location to start line
   *    total_length -> Approximate length of line in pixels(may be slightly off due to rounding error)
   *    Theta -> Angle in Radians from 0
   */
    float theta = radians(theta_deg);
    int x_dist = sin(theta) * total_length;
    int y_dist = -cos(theta) * total_length;
    display.drawLine(startX,startY, startX + x_dist, startY + y_dist, WHITE);
}


// This puts you in the Pacific timezone with the current US rules for DST. Change it if you live elsewhere, or replace with a simple subtract if you live somewhere without DST.
TimeChangeRule myDST = {"PDT", Second, Sun, Mar, 2, -420};    //Daylight time = UTC - 7 hours
TimeChangeRule mySTD = {"PST", First, Sun, Nov, 2, -480};     //Standard time = UTC - 8 hours
Timezone myTZ(myDST, mySTD);

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using hardware serial:
//   Connect the GPS TX (transmit) pin to Arduino RX3
//   Connect the GPS RX (receive) pin to matching TX3
Adafruit_GPS GPS(&Serial3);
HardwareSerial mySerial = Serial3;

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

void setup()
{
  pinMode(buttonPin, INPUT);
  pinMode(button2Pin, INPUT);
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  //Serial.begin(115200);
  // Serial debugging if needed uncomment.
  //Serial.println("Wait begins");
  mag.begin();

  // write it out!
  delay(5000);
  //Serial.println("Adafruit GPS library basic test!");

  // 9600 is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

   // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(2000);
  // Clear the buffer.
  display.clearDisplay();

  display.display();
  // Turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 5 Hz update rate

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  // Sets display for dark background light text, saves power.
  display.setTextColor(WHITE);
  display.setCursor(0,0);
}

uint32_t timer = millis();



void loop()                
{

  char c = GPS.read();
  float bearing = 0;
  time_t utc, local;

  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH && display_switch== false){
    display_selector += 1;
    display_switch = true;  //should prevent freewheeling display selection.
  }
  if (display_selector >=5){
    display_selector = 0;
  }
  if (buttonState == LOW){
    display_switch = false;
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();
  if (prev_dist_time > millis())  prev_dist_time = millis();

  // approximately every .5 seconds or so, print out the current stats
  if (millis() - timer > 500) {
    timer = millis(); // reset the timer
    if (GPS.fix) {
      if(firstrun) //Gives us some initial GPS stuff to work with, in case future captures are corrputed.
      {
        gps_year = GPS.year;
        gps_hour = GPS.hour;
        tripLat = GPS.latitudeDegrees;
        tripLong = GPS.longitudeDegrees;
        firstrun = false;
      }
      time_average_distance += ((millis() - prev_dist_time) * knotsToMph(GPS.speed)) /3600000.0;
      time_average_distance_waypoint += ((millis() - prev_dist_time) * knotsToMph(GPS.speed)) /3600000.0;
      prev_dist_time = millis();
      button2State = digitalRead(button2Pin);
      if(button2State == HIGH){
        storedLat = GPS.latitudeDegrees;
        storedLong = GPS.longitudeDegrees;
        time_average_distance_waypoint = 0;
      }
      //This bit of magic just prevents setting our time based on bad GPS data. Stops the clock from jumping about.
      if(GPS.year<= gps_year + 1 && GPS.year>=gps_year && ((GPS.hour <= gps_hour +1 && GPS.hour >= gps_hour) || (GPS.hour == 0 && gps_hour == 23)))
      {
        setTime(GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, GPS.year);
        gps_year = GPS.year;
        gps_month = GPS.month;
        gps_day = GPS.day;
        gps_hour = GPS.hour;
        gps_minute = GPS.minute;
        trouble_count = 0;
      }
      
      if (display_selector == 0){
          /*This is the simple clock display, good to the millisecond in a perfect world, probably more like second here*/
          display.setTextSize(2);
          setTime(gps_hour, gps_minute, GPS.seconds, gps_day, gps_month, gps_year);
          // set every digit to the buffer
          local = myTZ.toLocal(now());
          int hour_int = hour(local);
          int minute_int = minute(local);
          int seconds = second(local);
          String print_string;
          if(hour_int >= 10){
          display.print(hour_int);
          }
          else{
            print_string = "0" + String(hour_int);
            display.print(print_string);
          }
          display.print(":");
          if(minute_int >= 10){
          display.print(minute_int);
          }
          else{
            print_string = "0" + String(minute_int);
            display.print(print_string);
          }
          display.print(":");
          if(seconds >= 10){
          display.println(seconds);
          }
          else{
            print_string = "0" + String(seconds);
            display.println(print_string);
          }
          display.print(GPS.month);
          display.print("/");
          display.print(GPS.day);
          display.print("/");
          display.println(GPS.year);
          display.display();
          delay(1);
          display.setCursor(0,0);
          display.clearDisplay();
        }
      
      else if (display_selector == 1){
          /* Here we have the GPS debug display, prints out a bunch of raw GPS info, plus your commpass heading. Either GPS compass if moving, or the Magnetic one if standing still*/
          display.setTextSize(1);
          display.print(GPS.latitudeDegrees);
          display.print(" * ");
          display.println(GPS.longitudeDegrees);
  
          display.print("Speed (Mph): "); display.println(knotsToMph(GPS.speed));
          if(GPS.speed >= 0.1){
          display.print("Angle: "); display.println(GPS.angle);
          }
          else{
            display.print("Angle: "); display.println(compassPoint());
          }
          display.print("Altitude: "); display.println(GPS.altitude);
          display.print("Satellites: "); display.println((int)GPS.satellites);
          display.display();
          delay(1);
          display.setCursor(0,0);
          display.clearDisplay();
         }
       
      else if (display_selector == 2){
          /*This is a speedometer plus compass. Uses magnetic if no GPS velocity available.*/
          display.setTextSize(3);
          display.println(knotsToMph(GPS.speed));
          display.setTextSize(1);
          if(GPS.speed >= 0.1){
          bearing = GPS.angle;
          }
          else{
            bearing = compassPoint();
          }
  
          if(bearing == 0) display.println("N/A");
          else if((bearing < 22.5)  || (bearing > 337.5 ))  display.println("North");
          else if((bearing > 22.5)  && (bearing < 67.5 ))   display.println("North-East");
          else if((bearing > 67.5)  && (bearing < 112.5 ))  display.println("East");
          else if((bearing > 112.5) && (bearing < 157.5 ))  display.println("South-East");
          else if((bearing > 157.5) && (bearing < 202.5 ))  display.println("South");
          else if((bearing > 202.5) && (bearing < 247.5 ))  display.println("South-West");
          else if((bearing > 247.5) && (bearing < 292.5 ))  display.println("West");
          else if((bearing > 292.5) && (bearing < 337.5 ))  display.println("North-West");
          drawCompass(100, 30, 20, -bearing);
          display.println(bearing);
          display.display();
          delay(1);
          display.setCursor(0,0);
          display.clearDisplay();
         }
     else if (display_selector == 3){

          /* This will point back to your waypoint set by button 2, as well as giving bearing, linear distance, and approximate travel distance.*/
          float lat_long_angle = latLongPoint(GPS.latitudeDegrees, GPS.longitudeDegrees, storedLat, storedLong);
          if(GPS.speed >= 0.1){
            bearing = GPS.angle;
            }
            else{
              bearing = compassPoint();
            }
          display.setTextSize(1);
          display.println("Waypoint-1");
          display.println("Distance: ");
          display.println(calc_dist(GPS.latitudeDegrees, GPS.longitudeDegrees, storedLat, storedLong));
          display.print("TD: ");
          display.println(time_average_distance_waypoint);
          display.print("Bearing: ");
          display.println(lat_long_angle - bearing);
          drawCompass(50, 50, 10, lat_long_angle - bearing);
          display.display();
          delay(1);
          display.setCursor(0,0);
          display.clearDisplay();
       }
     else if (display_selector == 4){
          /* This will point back to your origin point, as well as giving bearing, linear distance, and approximate travel distance.*/
          float lat_long_angle = latLongPoint(GPS.latitudeDegrees, GPS.longitudeDegrees, tripLat, tripLong);
          if(GPS.speed >= 0.1){
            bearing = GPS.angle;
            }
            else{
              bearing = compassPoint();
            }
          display.setTextSize(1);
          display.println("Trip");
          display.print("Distance: ");
          display.println(calc_dist(GPS.latitudeDegrees, GPS.longitudeDegrees, tripLat, tripLong));
          display.print("TD: ");
          display.println(time_average_distance_waypoint);
          display.print("Bearing: ");
          display.println(lat_long_angle - bearing);
          drawCompass(50, 50, 10, lat_long_angle - bearing);
          display.display();
          delay(1);
          display.setCursor(0,0);
          display.clearDisplay();
       }
        trouble_count++;
        if(trouble_count > 10){
          softwareReset(WDTO_60MS);
        }
      }
    }
  }

