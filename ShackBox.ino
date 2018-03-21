/* ====================================================================
 * Copyright (c) 2018 Martin D. Waller - G0PJO.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer. 
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * 3. All advertising materials mentioning features or use of this
 *    software must display the following acknowledgment:
 *    "This product includes software developed by Martin D. Waller - G0PJO"
 *
 * 4. The names "ShackBox" must not be used to endorse or promote 
 *    products derived from this software without
 *    prior written permission.
 *
 * 5. Products derived from this software may not be called "ShackBox"
 *    nor may "ShackBox" appear in their names without prior written
 *    permission of Martin D. Waller.
 *
 * 6. Redistributions of any form whatsoever must retain the following
 *    acknowledgment:
 *    "This product includes software developed by Martin D. Waller - G0PJO"
 *
 * THIS SOFTWARE IS PROVIDED BY Martin D. Waller ``AS IS'' AND ANY
 * EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE OpenSSL PROJECT OR
 * ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 * ====================================================================
 *
 */
 
//
//  December 12th, 2017 M.D.Waller
//  a) Added support for Worked All Britain squares. It will now
//  swap the QRA with the WAB every second. The code for this was
//  derived from http://www.dorcus.co.uk/carabus/ll_ngr.html
//  by Roger Muggleton.
//
//  December 19th, 2017 M.D.Waller
//  a) Added code to correct the pressure reading for altitude. Once
//  it knows the altitude from the GPS data the pressure will be
//  corrected to read as it would do at sea level.
//  b) Shuffled the display slightly to ensure data gets displayed
//  properly.
//
//  March 18th, 2018 M.D.Waller
//  Following requests from people building the ShackBox the changes
//  below have been made.
//  a) The FLIP_SECONDS manifest can be used to control the frequency
//  with which the data formats are flipped.
//  b) The format used to display Latitude and Longitude can be controlled
//  by the LATLON_FORMAT_ODD and LATLON_FORMAT_EVEN manifests. These can be
//  set to one of three values LATLON_FORMAT_DMS (degrees minutes seconds), 
//  LATLON_FORMAT_DD_MMMMM (degrees decimal degrees), or LATLON_FORMAT_DDMM_MMM
//  (degrees minutes decimal minutes).

#include <NeoSWSerial.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>

// https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
// https://github.com/SlashDevin/NeoSWSerial

// Version / Copyright deatils

#define PROGRAM_NAME "ShackBox"
#define PROGRAM_VERSION "V1.6?"
#define BETA_TEXT ""
#define G0PJO_TEXT "M.D.Waller G0PJO"

// CHANGE THIS TO ALTER THE SECOND COUNT BETWEEN FLIPPING THE DATA FORMAT

#define FLIP_SECONDS 1

#define LATLON_FORMAT_DMS 0
#define LATLON_FORMAT_DD_DDDDD 1
#define LATLON_FORMAT_DDMM_MMM 2

// CHANGE THESE TO ALTER THE LATITUDE / LONGUTIDE DISPLAY FORMATS. 

#define LATLON_FORMAT_ODD LATLON_FORMAT_DMS
#define LATLON_FORMAT_EVEN LATLON_FORMAT_DD_DDDDD

// Math constants

#define PI 3.1415926535897932384626433832795

// Units

#define UNITS_CENTIGRADE "C"
#define UNITS_FAHRENHEIT "F"
#define UNITS_FEET "ft"
#define UNITS_METRES "m"
#define UNITS_MILLIBAR "mb"

// The following manifests are used to drive the GPS module

#define GPS_RX_DIGIT_INPUT 3
#define GPS_TX_DIGIT_OUTPUT 4
#define GPS_BAUD_RATE 9600
#define GPS_BUFFER_SIZE 256

// We make use of the NMEA GPGGA sentence and the GPRMC sentence these manifests
// are used to determine where in the sentence data is pulled from.

#define GPGGA "$GPGGA"
#define GPGGA_TIME_ELEMENT 1
#define GPGGA_LATITUDE_ELEMENT 2
#define GPGGA_LATITUDE_NS_ELEMENT 3
#define GPGGA_LONGITUDE_ELEMENT 4
#define GPGGA_LONGITUDE_EW_ELEMENT 5
#define GPGGA_NUMBER_SATELLITES_ELEMENT 7
#define GPGGA_ALTITUDE_ELEMENT 9

#define GPRMC "$GPRMC"
#define GPRMC_DATE_ELEMENT 9

// The following manifests determine which row the various data parts
// are displayed

#define TITLE_ROW 1
#define BETA_ROW 2
#define G0PJO_ROW 3
#define DATE_DAY_TIME_ROW 0
#define TEMP_PRESSURE_ALTITUDE_ROW 1
#define LAT_LON_ROW 2
#define QRA_ROW 3

// The first line contains the date/day and time. It will be formatted
// as:
//
//  01234567890123456789
//
//  19-DEC-2017 10:47:34
//  TUESDAY     10:47:34
//

#define DATE_DAY_START 0
#define TIME_START 12

//
// The second line contains temperature, pressure and alititude. It will
// be formatted as:
//
//  01234567890123456789
//
//  099F ?*1023mb 1000m

#define TEMPERATURE_START 0
#define PRESSURE_START 5
#define ALTITUDE_START 14

//
// The third line contains latitude and longitude. It will
// be formatted as:
//
//  01234567890123456789
//
//  51.98974N 1.20924E

#define LATITUDE_START 0
#define LONGITUDE_START 10

//
// The fourth line contains QRA/WAB and NSat. It will
// be formatted as:
//
//  01234567890123456789
//
//     JO01OX  NSat: 99

#define QRA_WAB_START 3
#define NSAT_START 11

// The following manifests are used in converting latitude
// longitude values to QRA values.
 
#define ASCII_A 'A'
#define ASCII_0 '0'

#define FIELDLONGITUDE 0
#define FIELDLATITUDE 1
#define SQUARELONGITUDE 2
#define SQUARELATITUDE 3
#define SUBSQUARELONGITUDE 4
#define SUBSQUARELATITUDE 5

struct DegreesMinutesSeconds
{
  double Degrees;
  double Minutes;
  double Seconds;
  double DecimalDegrees;
  double DegreesDecimalMinutes;
  char Suffix;
};

const char *dayNames[] = {
    "SATURDAY",     // 0
    "SUNDAY",       // 1
    "MONDAY",       // 2
    "TUESDAY",      // 3
    "WEDNESDAY",    // 4
    "THURSDAY",     // 5
    "FRIDAY"        // 6
};

const char *monthNames[] = {
    "JAN",    
    "FEB",  
    "MAR",  
    "APR",  
    "MAY",  
    "JUN",  
    "JUL",  
    "AUG",  
    "SEP",  
    "OCT",  
    "NOV",  
    "DEC"
};

// To support detailed pressure descriptions we're going to need to save
// three hours worth of data. For the moment we will save one pressure
// reading every 10 minutes.

#define PRESSURE_BUFFER_SIZE (3 * 6)

double pressureRingBuffer[PRESSURE_BUFFER_SIZE];
int ringBufferHead = -1;
int ringBufferTail = -1;
int lastPressureChangeMinute = 0;
bool isFullRingBuffer = false;
double lastPressure = 0.0;
char currentDirection = ' ';

// Position Related Data

char gpsBuffer[GPS_BUFFER_SIZE];
int gpsBufferIndex = 0;
char elementBuffer[80];
bool oddLine = false;
int flipCount = FLIP_SECONDS;
struct DegreesMinutesSeconds dmsLatitude[1];
struct DegreesMinutesSeconds dmsLongitude[1];

#define DISPLAYLINEBUFFERLENGTH 20

char displayLineBuffer[DISPLAYLINEBUFFERLENGTH + 1];
int displayLineBufferIndex;

#define DAY_DATE_BUFFER_LENGTH 11

char dateBuffer[DAY_DATE_BUFFER_LENGTH + 1];
char dayBuffer[DAY_DATE_BUFFER_LENGTH + 1];

NeoSWSerial gpsSerialPort(GPS_RX_DIGIT_INPUT, GPS_TX_DIGIT_OUTPUT);
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
Adafruit_BMP085 bmp;

// Custom Characters

// The following manifests are used to identify each character. These are based at 1 and
// not 0 because character 0 is treated as the null on the end of the string when the display
// buffer is printed! These characters were designed using:
//
//  https://omerk.github.io/lcdchargen/

#define CHAR_UPARROW 1
#define CHAR_DOWNARROW 2
#define CHAR_ALTITUDE 3

byte upArrow[8] = {0b00100,0b01110,0b10101,0b00100,0b00100,0b00100,0b00100,0b00000};
byte downArrow[8] = {0b00100,0b00100,0b00100,0b00100,0b10101,0b01110,0b00100,0b00000};
byte altitude[8] = {0b11111,0b00100,0b01110,0b10101,0b00100,0b00100,0b11111,0b00000};

// Ring Buffer Code

void initRingBuffer()
{
  ringBufferHead = -1;
  ringBufferTail = -1; 
  lastPressureChangeMinute = 0; 
  isFullRingBuffer = false;
  lastPressure = 0.0;
}

int incRingBufferIndex(int current)
{
  return (current + 1) % PRESSURE_BUFFER_SIZE;
}

int decRingBufferIndex(int current)
{
  current -= 1;
  if (-1 == current)
    current = PRESSURE_BUFFER_SIZE - 1;

  return current;
}

double getRingBuffer(int index)
{
  return pressureRingBuffer[index];
}

void addRingBuffer(double v)
{
  if (-1 == ringBufferHead)
  {
    // This is the first entry that we have, set the head / tail accordingly

    ringBufferHead = 0;
    ringBufferTail = 0;
  }
  else
  {
    // Update the head index

    ringBufferHead = incRingBufferIndex(ringBufferHead);

    // Update the isFullRingBuffer flag

    if ((false == isFullRingBuffer) && (0 == ringBufferHead))
      isFullRingBuffer = true;
    
    // Have we hit the tail?

    if (ringBufferHead == ringBufferTail)
    {
      ringBufferTail = incRingBufferIndex(ringBufferTail);
    }
  }

  // Save the value

  pressureRingBuffer[ringBufferHead] = v;
}

boolean isEmptyRingBuffer()
{
  return -1 == ringBufferHead;
}

// Display Code

void clearDisplayLineBuffer()
{
  for(int i = 0; i < DISPLAYLINEBUFFERLENGTH; i++)
    displayLineBuffer[i] = ' ';
    
  displayLineBuffer[DISPLAYLINEBUFFERLENGTH] = '\0';
  displayLineBufferIndex = 0;
}

void padDisplayLineBufferTo(int length, char with)
{
  while (displayLineBufferIndex < length)
    displayLineBuffer[displayLineBufferIndex++] = with;
}

void addCharToDisplayLineBuffer(const char c)
{
  if (displayLineBufferIndex < DISPLAYLINEBUFFERLENGTH)
    displayLineBuffer[displayLineBufferIndex++] = c;
}

void addStringToDisplayLineBuffer(const char* str)
{
  char *p = (char *)str;
  while (*p != 0 && displayLineBufferIndex < DISPLAYLINEBUFFERLENGTH)
    displayLineBuffer[displayLineBufferIndex++] = *p++;
}

void addDoubleToDisplayLineBuffer(double v,int decimalPlaces)
{
  char buffer[20];
  char *p = dtostrf(v,10,decimalPlaces,buffer);

  while (*p != 0 && displayLineBufferIndex < DISPLAYLINEBUFFERLENGTH)
  {
    if (*p != ' ')
      displayLineBuffer[displayLineBufferIndex++] = *p;
    p++;
  }
}

void addDMSAsDD_DDDDDToDisplayLineBuffer(struct DegreesMinutesSeconds *dms)
{
  double valueToDisplay = (dms->DecimalDegrees > 0) ? dms->DecimalDegrees : -1 * dms->DecimalDegrees;
  addDoubleToDisplayLineBuffer(valueToDisplay,5);  
}

void addDMSAsDMSToDisplayLineBuffer(struct DegreesMinutesSeconds *dms)
{
  addDoubleToDisplayLineBuffer(dms->Degrees,0);
  addCharToDisplayLineBuffer(' ');
  addDoubleToDisplayLineBuffer(dms->Minutes,0);
  addCharToDisplayLineBuffer(' ');
  addDoubleToDisplayLineBuffer(dms->Seconds,0);  
}

void addDMSAsDDMM_MMMToDisplayLineBuffer(struct DegreesMinutesSeconds *dms)
{
  double valueToDisplay = (dms->DegreesDecimalMinutes > 0) ? dms->DegreesDecimalMinutes : -1 * dms->DegreesDecimalMinutes;
  addDoubleToDisplayLineBuffer(valueToDisplay,3);  
}

void addDMSToDisplayLineBufferInFormat(struct DegreesMinutesSeconds *dms, int format)
{ 
  switch(format)
  {
    case LATLON_FORMAT_DMS:

      addDMSAsDMSToDisplayLineBuffer(dms);
      break;
      
    case LATLON_FORMAT_DD_DDDDD:
    
      addDMSAsDD_DDDDDToDisplayLineBuffer(dms);
      break;

    case LATLON_FORMAT_DDMM_MMM:
    
      addDMSAsDDMM_MMMToDisplayLineBuffer(dms);
      break;
  }

  addCharToDisplayLineBuffer(dms->Suffix); 
}

void addDMSToDisplayLineBuffer(struct DegreesMinutesSeconds *dms)
{ 
  if (false == oddLine)
  {
    addDMSToDisplayLineBufferInFormat(dms,LATLON_FORMAT_ODD);
  }
  else
  {
    addDMSToDisplayLineBufferInFormat(dms,LATLON_FORMAT_EVEN);
  }      
}

void writeDisplayLineBuffer(int row)
{
  lcd.setCursor(0,row); 
  lcd.print(displayLineBuffer);
}

/*
 * This method is called to convert decimal latitude / longitude into a 
 * standard QRA lacator. It will return a pointer to a zero terminated
 * array of characters.
 */
char* toQRA(double latitude, double longitude)
{
    static char ms[7];
    
    latitude += 90.0;
    longitude += 180.0;

    int v = (int)(longitude / 20);
    ms[FIELDLONGITUDE] = ASCII_A + v;
    longitude -= v * 20;

    v = (int)(latitude / 10);
    ms[FIELDLATITUDE] = ASCII_A + v;
    latitude -= v * 10;

    v = (int)(longitude / 2);
    ms[SQUARELONGITUDE] = ASCII_0 + v;
    longitude -= v * 2;

    v = (int)latitude;
    ms[SQUARELATITUDE] = ASCII_0  + v;
    latitude -= v;

    v = (int)(longitude * 12);
    ms[SUBSQUARELONGITUDE] = ASCII_A + v;

    v = (int)(latitude * 24);
    ms[SUBSQUARELATITUDE] = ASCII_A + v;

    return ms;
}

// WAB Related Code

double Marc(double bf0, double n, double phi0, double phi)
{
  double Marc = bf0 * (((1 + n + ((5 / 4) * (n * n)) + ((5 / 4) * (n * n * n))) * (phi - phi0))
     - (((3 * n) + (3 * (n * n)) + ((21 / 8) * (n * n * n))) * (sin(phi - phi0)) * (cos(phi + phi0)))
     + ((((15 / 8) * (n * n)) + ((15 / 8) * (n * n * n))) * (sin(2 * (phi - phi0))) * (cos(2 * (phi + phi0))))
     - (((35 / 24) * (n * n * n)) * (sin(3 * (phi - phi0))) * (cos(3 * (phi + phi0)))));
     
  return(Marc);
}
  
char *toWAB(double lat, double lon)
{
  static char wab[5];
  
  double deg2rad = PI / 180;
  double rad2deg = 180.0 / PI;
  double phi = lat * deg2rad;      // convert latitude to radians
  double lam = lon * deg2rad;   // convert longitude to radians
  double a = 6377563.396;       // OSGB semi-major axis
  double b = 6356256.91;        // OSGB semi-minor axis
  double e0 = 400000;           // OSGB easting of false origin
  double n0 = -100000;          // OSGB northing of false origin
  double f0 = 0.9996012717;     // OSGB scale factor on central meridian
  double e2 = 0.0066705397616;  // OSGB eccentricity squared
  double lam0 = -0.034906585039886591;  // OSGB false east
  double phi0 = 0.85521133347722145;    // OSGB false north
  double af0 = a * f0;
  double bf0 = b * f0;
  
  // easting
    
  double slat2 = sin(phi) * sin(phi);
  double nu = af0 / (sqrt(1 - (e2 * (slat2))));
  double rho = (nu * (1 - e2)) / (1 - (e2 * slat2));
  double eta2 = (nu / rho) - 1;
  double p = lam - lam0;
  double IV = nu * cos(phi);
  double clat3 = pow(cos(phi),3);
  double tlat2 = tan(phi) * tan(phi);
  double V = (nu / 6) * clat3 * ((nu / rho) - tlat2);
  double clat5 = pow(cos(phi), 5);
  double tlat4 = pow(tan(phi), 4);
  double VI = (nu / 120) * clat5 * ((5 - (18 * tlat2)) + tlat4 + (14 * eta2) - (58 * tlat2 * eta2));
  double east = e0 + (p * IV) + (pow(p, 3) * V) + (pow(p, 5) * VI);
  
  // northing
    
  double n = (af0 - bf0) / (af0 + bf0);
  double M = Marc(bf0, n, phi0, phi);
  double I = M + (n0);
  double II = (nu / 2) * sin(phi) * cos(phi);
  double III = ((nu / 24) * sin(phi) * pow(cos(phi), 3)) * (5 - pow(tan(phi), 2) + (9 * eta2));
  double IIIA = ((nu / 720) * sin(phi) * clat5) * (61 - (58 * tlat2) + tlat4);
  double north = I + ((p * p) * II) + (pow(p, 4) * III) + (pow(p, 6) * IIIA);
  
    east = round(east);       // round to whole number
    north = round(north);     // round to whole number

  double eX = east / 500000;
  double nX = north / 500000;
  double tmp = floor(eX)-5.0 * floor(nX)+17.0;
  nX = 5 * (nX - floor(nX));
  eX = 20 - 5.0 * floor(nX) + floor(5.0 * (eX - floor(eX)));
  if (eX > 7.5)
    eX = eX + 1;
  if (tmp > 7.5)
    tmp = tmp + 1;

  wab[0] = char(tmp + 65);
  wab[1] = char(eX + 65);
  wab[2] = char((int)(east / 10000) % 10 + '0');
  wab[3] = char((int)(north / 10000) % 10 + '0');
  wab[4] = '\0';

  return wab;
}

/*
 * This method is called to locate the n'th element in an NMEA sentence. It
 * will only return a pointer if the element has been found and is not of
 * zero length. The returned pointer will point to a zero terminated array
 * of characters.
 */
char *findNMEAElement(int requiredElement)
{
  char *retVal = NULL;

  bool found = false;
  int currentElementNo = 0;
  char *p = &gpsBuffer[0];

  if (0 == requiredElement)
  {
    // This is easy!

    found = true;    
  }
  else
  {
    while(('\0' != *p) && (requiredElement != currentElementNo))
    {
      if (',' == *p)
      {
        currentElementNo++;    
      }

      p++;
    }

    if (currentElementNo == requiredElement)
      found = true;
  }

  if (true == found)
  {
    char *o = &elementBuffer[0];
    while(('\0' != *p) && (','!= *p))
    {
      *o++ = *p++;
    }

    *o = '\0';
    retVal = &elementBuffer[0];
  }
  return retVal;
}

/*
 * This method is called to determine if the given pointer points
 * to a string with a length. If null is passed in or the pointer
 * points to a zero length string it will return true.
 */
bool emptyString(char *stringPointer)
{
  bool retVal = true;

  if (NULL != stringPointer)
  {
    if (strlen(stringPointer) > 0)
      retVal = false;
  }

  return retVal;
}

void nmeaDecimalDegreesToDMS(double nmeaDecimalDMS,struct DegreesMinutesSeconds *dms)
{
    dms->DegreesDecimalMinutes = nmeaDecimalDMS;
    
    // Don't use abs(), the value gets turned into an integer!
    
    double absNmeaDecimalDMS = nmeaDecimalDMS > 0 ? nmeaDecimalDMS : -1 * nmeaDecimalDMS;
    
    double deg = (double)((int)(absNmeaDecimalDMS / 100));
    double min = absNmeaDecimalDMS - deg * 100;
    double decimalDeg = deg + min / 60;

    int degree = (int)decimalDeg;
    int minutes = (int) ((decimalDeg - (float)degree) * 60.f);
    double seconds = ((decimalDeg - (float)degree - (float)minutes/60.f) * 60.f * 60.f); 
    
    dms->Degrees = degree;
    dms->Minutes = minutes;
    dms->Seconds = seconds;    

    dms->DecimalDegrees = dms->Degrees + dms->Minutes / 60 + dms->Seconds / 3600;

    if (nmeaDecimalDMS < 0)
      dms->DecimalDegrees *= -1;
}

void setup() {

  // Initialise the pressure ring buffer

  initRingBuffer();

  // Clear down the day and date buffer

  dayBuffer[0] = '\0';
  dateBuffer[0] = '\0';
  
  // Open up the serial port
  
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } 

  // Initialise the display

  lcd.begin();
  lcd.backlight();

  lcd.createChar(CHAR_UPARROW,upArrow);
  lcd.createChar(CHAR_DOWNARROW,downArrow);
  lcd.createChar(CHAR_ALTITUDE,altitude);

  clearDisplayLineBuffer();
  addStringToDisplayLineBuffer("    ");
  addStringToDisplayLineBuffer(PROGRAM_NAME);
  addCharToDisplayLineBuffer(' ');
  addStringToDisplayLineBuffer(PROGRAM_VERSION);
  
  writeDisplayLineBuffer(TITLE_ROW);

  clearDisplayLineBuffer();
  addStringToDisplayLineBuffer("       ");
  addStringToDisplayLineBuffer(BETA_TEXT);
  writeDisplayLineBuffer(BETA_ROW);

  clearDisplayLineBuffer();
  addStringToDisplayLineBuffer("  ");
  addStringToDisplayLineBuffer(G0PJO_TEXT);
  writeDisplayLineBuffer(G0PJO_ROW);  

  // Start the BMP device
  
  bmp.begin();
 
  // We have a Software serial port to read data from the GPS device. We need
  // to open it here.
  
  gpsSerialPort.begin(GPS_BAUD_RATE);  

  delay(2000);
}

bool isLeapYear(int year)
{
  return (year % 400 == 0 || (year % 4 == 0 && year % 100 != 0));
}
        
int dayOfWeek(int year, int month, int day)
{
  int months[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 };

  long days = (long)year * 365L;       
  for (int i = 4; i < year; i += 4) if (isLeapYear(i)) days++;
  days += months[month - 1] + day;    
  if ((month > 2) && isLeapYear(year)) days++;

  return days % 7;
}
        
void loop() {
  
  // Here we will flush any data that we may have from the GPS unit.

  if (gpsSerialPort.available()) {

    char c = gpsSerialPort.read();
    //Serial.print(c);

    switch(c)
    {
      case '\r':

        // This is the end of the line. We can process the data here. We're only interested
        // in the $GPRMC sentence so lets look to see if we have one.

        if (gpsBufferIndex > 5)
        {
          
          gpsBuffer[gpsBufferIndex] = '\0';
           
          if (0 == strncmp(gpsBuffer,GPRMC,strlen(GPRMC)))
          {
            char *datePointer = findNMEAElement(GPRMC_DATE_ELEMENT);
            if (false == emptyString(datePointer))
            {
              // Pull out the day, month and year

              int day = datePointer[0] - '0';
              day = day * 10 + datePointer[1] - '0';

              int month = datePointer[2] - '0';
              month = month * 10 + datePointer[3] - '0';

              int year = datePointer[4] - '0';
              year = year * 10 + datePointer[5] - '0';                            
              year += 2000;
              
              // Build the day buffer
              
              // Get the index for the day of the week
              
              int dayOfWeekIndex = dayOfWeek(year,month,day);
              
              // Copy over the day name
              
              strcpy(dayBuffer,dayNames[dayOfWeekIndex]);

              // We now need to pad the buffer out with space so the display does not
              // go mad!

              memset(&dayBuffer[strlen(dayBuffer)],' ',DAY_DATE_BUFFER_LENGTH - strlen(dayBuffer));

              // Next we build the date buffer
              
              int i = 0;
              dateBuffer[i++] = datePointer[0];
              dateBuffer[i++] = datePointer[1];
              dateBuffer[i++] = '-';

              strcpy(&dateBuffer[i],monthNames[month - 1]);
              i += 3;

              dateBuffer[i++] = '-';
              dateBuffer[i++] = '2';
              dateBuffer[i++] = '0';              // Force this to 20 will see me out!
              dateBuffer[i++] = datePointer[4];
              dateBuffer[i++] = datePointer[5];
              dateBuffer[i++] = '\0';
            }
            else {

              // No date information, empty both the day and the date buffer
              
              dayBuffer[0] = '\0';
              dateBuffer[0] = '\0';
            }
          }
          else if (0 == strncmp(gpsBuffer,GPGGA,strlen(GPGGA)))
          {

            // Line 0 - Time

            clearDisplayLineBuffer();
            addStringToDisplayLineBuffer("     Waiting...");
             
            char *timePointer = findNMEAElement(GPGGA_TIME_ELEMENT);
            if (false == emptyString(timePointer))
            {
              // Clear the display buffer and pad out to the correct
              // starting point
              
              clearDisplayLineBuffer();
              padDisplayLineBufferTo(DATE_DAY_START,' ');
              
              if (false == oddLine)
                addStringToDisplayLineBuffer(dateBuffer);
              else
                addStringToDisplayLineBuffer(dayBuffer);

              // Pad out to the start of the time and put the time in

              padDisplayLineBufferTo(TIME_START,' ');

              // Copy over the time
              
              addCharToDisplayLineBuffer(timePointer[0]); 
              addCharToDisplayLineBuffer(timePointer[1]);
              addCharToDisplayLineBuffer(':');
              addCharToDisplayLineBuffer(timePointer[2]);
              addCharToDisplayLineBuffer(timePointer[3]);
              addCharToDisplayLineBuffer(':');
              addCharToDisplayLineBuffer(timePointer[4]);
              addCharToDisplayLineBuffer(timePointer[5]);

              // At this point we have the time available to us. We need
              // to save away the pressure every 10 minutes. Do do this
              // we will take the minutes and if mintues % 10 = 0 then
              // we'll save the pressure away. The only real impact of
              // this is that we may be 9 minutes late producing the 
              // R4 style description!

              int minutes = (timePointer[2] - '0') * 10 + (timePointer[3] - '0');

              if (0 == (minutes % 10) && (minutes != lastPressureChangeMinute))
              {
                // Yes, save away the pressure

                addRingBuffer(bmp.readPressure());
                lastPressureChangeMinute = minutes;
              }
            }

            // Display the line

            writeDisplayLineBuffer(DATE_DAY_TIME_ROW);       

            // Line 1 - Temperature, Pressure, Altitude

            clearDisplayLineBuffer();
            padDisplayLineBufferTo(TEMPERATURE_START,' ');

            // Add the temperature

            double temperatureCelcius = bmp.readTemperature();
            if (false == oddLine)
            {
              addDoubleToDisplayLineBuffer(temperatureCelcius,0);
              addStringToDisplayLineBuffer(UNITS_CENTIGRADE);
            }
            else
            {
              addDoubleToDisplayLineBuffer(temperatureCelcius * 1.8 + 32.0,0);
              addStringToDisplayLineBuffer(UNITS_FAHRENHEIT);
            }

            // Before we add the pressure we need to look to see if we have the
            // altitude as this will impact on what we display

            // Assume we have no altitude
            
            bool haveAltitude = false;
            double altitudeMetres = 0.0;
            
            // Go and look it up in the GPGGA
            
            char *altitudePointer = findNMEAElement(GPGGA_ALTITUDE_ELEMENT);
            if (false == emptyString(altitudePointer))
            {
              // We have it, convert it and save it away. Also set
              // the flag to say that we have it.
              
              altitudeMetres = atof(altitudePointer);
              haveAltitude = true;
            } 
            
            // Add the pressure

            padDisplayLineBufferTo(PRESSURE_START,' ');
            if ((false == oddLine) || (true == isEmptyRingBuffer()))
            {                      
              // Read the current pressure. We may reas the altitude correct
              // version if we know out height.

              double currentPressure;
              if (true == haveAltitude)
              {
                currentPressure = bmp.readSealevelPressure(altitudeMetres) / 100;

                // Display the current pressure

                addCharToDisplayLineBuffer(CHAR_ALTITUDE);
                addDoubleToDisplayLineBuffer(currentPressure,0);
              }
              else
              {
                currentPressure = bmp.readPressure() / 100;

                // Display the current pressure
              
                addDoubleToDisplayLineBuffer(currentPressure,0);
              }

              addStringToDisplayLineBuffer(UNITS_MILLIBAR);

              // Update the last pressure if it is different

              if (lastPressure != currentPressure)
              {
                // We have seen a pressure change so we can update the
                // current direction.

                currentDirection = (currentPressure > lastPressure ? CHAR_UPARROW : CHAR_DOWNARROW); // 'R' : 'F');

                // Then update the last pressure

                lastPressure = currentPressure;
              }
            }
            else
            {
              if (false == isFullRingBuffer)
                addStringToDisplayLineBuffer("*");
                
              double pressureNow = getRingBuffer(ringBufferHead);
              double pressureThen = getRingBuffer(ringBufferTail);

                //Serial.print("head = ");Serial.print(ringBufferHead);Serial.print("\n");
                //Serial.print("tail = ");Serial.print(ringBufferTail);Serial.print("\n");
                //Serial.print("pressureNow = ");Serial.print(pressureNow);Serial.print("\n");
                //Serial.print("pressureThen = ");Serial.print(pressureThen);Serial.print("\n");
          

              double diff = (pressureNow - pressureThen) / 100;
              if (diff < 0)
                diff *= -1;
                
              Serial.print("diff = ");Serial.print(diff);Serial.print("\n");
              
              if (pressureNow > pressureThen)
              {
                // Going Up!

                addCharToDisplayLineBuffer(CHAR_UPARROW);
                addStringToDisplayLineBuffer(" ");
              }
              else
              {
                // Going Down!

                addCharToDisplayLineBuffer(CHAR_DOWNARROW);
                addStringToDisplayLineBuffer(" ");
              }

              if (diff >= 6.0)
                addStringToDisplayLineBuffer("VR ");
              else if (diff >= 3.6)
                addStringToDisplayLineBuffer("Q  ");
              else if (diff >= 1.6)
              {
                addStringToDisplayLineBuffer("   ");  
              }
              else if (diff >= 0.1)
                addStringToDisplayLineBuffer("S  ");
              else
                addStringToDisplayLineBuffer("MS ");

              // Finally add the current direction

              //addCharToDisplayLineBuffer(' ');
              addCharToDisplayLineBuffer(currentDirection);
            }

            // Add in the Altitude

            
            if (true == haveAltitude)
            {
              padDisplayLineBufferTo(ALTITUDE_START,' ');
              
              if (false == oddLine)
              {
                addDoubleToDisplayLineBuffer(altitudeMetres,0);
                addStringToDisplayLineBuffer(UNITS_METRES);
              }
              else
              {
                addDoubleToDisplayLineBuffer(altitudeMetres * 3.28,0);
                addStringToDisplayLineBuffer(UNITS_FEET);
              }
            }            
          
            // Display the line

            writeDisplayLineBuffer(TEMP_PRESSURE_ALTITUDE_ROW);

            // Line 2 - Latitude, Longitude

            boolean latitudeValid = false;
            char *latitudePointer = findNMEAElement(GPGGA_LATITUDE_ELEMENT);
            if (false == emptyString(latitudePointer))
            {
              double latitude = atof(latitudePointer);

              dmsLatitude[0].Suffix = '?';
              char *latitudeNSPointer = findNMEAElement(GPGGA_LATITUDE_NS_ELEMENT);
              if (false == emptyString(latitudeNSPointer))
              {
                dmsLatitude[0].Suffix = *latitudeNSPointer;
                if (0 == strcmp(latitudeNSPointer,"S"))
                  latitude *= -1;
              }

              nmeaDecimalDegreesToDMS(latitude,&dmsLatitude[0]);
              latitudeValid = true;
            }

            boolean longitudeValid = false;
            char *longitudePointer = findNMEAElement(GPGGA_LONGITUDE_ELEMENT);
            if (false == emptyString(longitudePointer))
            {
              double longitude = atof(longitudePointer);

              dmsLongitude[0].Suffix = '?';
              char *longitudeNSPointer = findNMEAElement(GPGGA_LONGITUDE_EW_ELEMENT);
              if (false == emptyString(longitudeNSPointer))
              {
                dmsLongitude[0].Suffix = *longitudeNSPointer;
                if (0 == strcmp(longitudeNSPointer,"W"))
                  longitude *= -1;
              }

              nmeaDecimalDegreesToDMS(longitude,&dmsLongitude[0]);
              longitudeValid = true;
            }

            clearDisplayLineBuffer();
           
            if ((true == latitudeValid) && (true == longitudeValid))
            {
              // Display the latitude and longitude

              padDisplayLineBufferTo(LATITUDE_START,' ');
              
              addDMSToDisplayLineBuffer(&dmsLatitude[0]);
              
              padDisplayLineBufferTo(LONGITUDE_START,' ');;
              
              addDMSToDisplayLineBuffer(&dmsLongitude[0]);
            }  

            // Display the line

            writeDisplayLineBuffer(LAT_LON_ROW);

            // Line 3 - QRA / WAB

            clearDisplayLineBuffer();
            if ((true == latitudeValid) && (true == longitudeValid))
            {
              // Calculate the QRA

              padDisplayLineBufferTo(QRA_WAB_START,' ');
              
              if (true == oddLine)
                addStringToDisplayLineBuffer(toQRA(dmsLatitude[0].DecimalDegrees,dmsLongitude[0].DecimalDegrees));
              else
                addStringToDisplayLineBuffer(toWAB(dmsLatitude[0].DecimalDegrees,dmsLongitude[0].DecimalDegrees));

              padDisplayLineBufferTo(10,' ');
            }  

            // Put in the number of satellites

            char *nSatPointer = findNMEAElement(GPGGA_NUMBER_SATELLITES_ELEMENT);
            if (false == emptyString(nSatPointer))
            {
              padDisplayLineBufferTo(NSAT_START,' ');
              
              double nSat = atof(nSatPointer);

              addStringToDisplayLineBuffer("NSat: ");
              addDoubleToDisplayLineBuffer(nSat,0);
            }

            writeDisplayLineBuffer(QRA_ROW);                            
            
            // Invert the line indicator as required

            flipCount--;
            if (0 == flipCount)
            {
              oddLine = !oddLine;
              flipCount = FLIP_SECONDS;
            }
          }
        }

        // Finally we can reset the buffer index

        gpsBufferIndex = 0;
        
        break;
        
      case '\n':
        break;
        
      default:
        //Serial.write(c);

        // Here we need to add the character to the buffer.

        gpsBuffer[gpsBufferIndex] = c;
        gpsBufferIndex++;

        // Next we need to check for buffer overflow

        if (gpsBufferIndex >= GPS_BUFFER_SIZE)
          gpsBufferIndex = 0;
      }
    }
}
