/* The example is for CubeCell_GPS,
 * GPS works only before lorawan uplink, the board current is about 45uA when in lowpower mode.
 */
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530.h"
#include "GPS_Air530Z.h"
#include "HT_SSD1306Wire.h"
#include "qrcode.h"

Air530Class GPS;
extern SSD1306Wire display;

#define LPP_GPS_LAT_LON_MULT 100000

// when gps waked, if in GPS_UPDATE_TIMEOUT, gps not fixed then into low power mode
#define GPS_UPDATE_TIMEOUT 60000

/*
   set LoraWan_RGB to Active,the RGB active in loraWan
   RGB red means sending;
   RGB purple means joined done;
   RGB blue means RxWindow1;
   RGB yellow means RxWindow2;
   RGB green means received done;
*/

/* OTAA para*/
// uint8_t devEui[] = { 0x00, 0x07, 0x30, 0x26, 0x0e, 0x3a, 0x75, 0x27 };
uint8_t devEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// static uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = {0x50, 0xc9, 0x90, 0xd6, 0xd0, 0x97, 0xdd, 0xf0, 0x04, 0x96, 0xdc, 0xef, 0xcf, 0xe2, 0x75, 0xbb};

/* ABP para*/
uint8_t nwkSKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appSKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint32_t devAddr = (uint32_t)0x00000000;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = {0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 60000;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 0;
/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

int32_t fracPart(double val, int n)
{
  return (int32_t)((val - (int32_t)(val)) * pow(10, n));
}

void VextON(void)
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) // Vext default OFF
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

void displayGPSInfo()
{
#ifdef DISPLAY_ON
  return;
#endif
  char str[30];
  display.clear();
  display.setFont(ArialMT_Plain_10);
  int index = sprintf(str, "%02d-%02d-%02d", GPS.date.year(), GPS.date.day(), GPS.date.month());
  str[index] = 0;
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, str);

  index = sprintf(str, "%02d:%02d:%02d", GPS.time.hour(), GPS.time.minute(), GPS.time.second(), GPS.time.centisecond());
  str[index] = 0;
  display.drawString(60, 0, str);

  if (GPS.location.age() < 1000)
  {
    display.drawString(120, 0, "A");
  }
  else
  {
    display.drawString(120, 0, "V");
  }

  index = sprintf(str, "alt: %d.%d", (int)GPS.altitude.meters(), fracPart(GPS.altitude.meters(), 2));
  str[index] = 0;
  display.drawString(0, 16, str);

  index = sprintf(str, "hdop: %d.%d", (int)GPS.hdop.hdop(), fracPart(GPS.hdop.hdop(), 2));
  str[index] = 0;
  display.drawString(0, 32, str);

  index = sprintf(str, "lat :  %d.%d", (int)GPS.location.lat(), fracPart(GPS.location.lat(), 4));
  str[index] = 0;
  display.drawString(60, 16, str);

  index = sprintf(str, "lon:%d.%d", (int)GPS.location.lng(), fracPart(GPS.location.lng(), 4));
  str[index] = 0;
  display.drawString(60, 32, str);

  index = sprintf(str, "speed: %d.%d km/h", (int)GPS.speed.kmph(), fracPart(GPS.speed.kmph(), 3));
  str[index] = 0;
  display.drawString(0, 48, str);
  display.display();
}

void printGPSInfo()
{
  Serial.print("Date/Time: ");
  if (GPS.date.isValid())
  {
    Serial.printf("%d/%02d/%02d", GPS.date.year(), GPS.date.day(), GPS.date.month());
  }
  else
  {
    Serial.print("INVALID");
  }

  if (GPS.time.isValid())
  {
    Serial.printf(" %02d:%02d:%02d.%02d", GPS.time.hour(), GPS.time.minute(), GPS.time.second(), GPS.time.centisecond());
  }
  else
  {
    Serial.print(" INVALID");
  }
  Serial.println();

  Serial.printf("SATS: %d ; ", GPS.satellites.value());
  Serial.printf(", HDOP: %10.7f ; ", GPS.hdop.hdop());
  Serial.printf(", LAT: %10.7f ; ", GPS.location.lat());
  Serial.printf(", LON: %10.7f ;", GPS.location.lng());
  Serial.printf(", AGE: %d ;", GPS.location.age());
  Serial.printf(", ALT: %10.7f ;", GPS.altitude.meters());
  Serial.printf(", COURSE: %10.7f ;", GPS.course.deg());
  Serial.printf(", SPEED: %f\n", GPS.speed.kmph());
}

uint8_t
encodeDegrees(uint8_t sendBuffer[], const uint8_t position, const float degrees)
{
  uint8_t offset = position;
  // thanks, arduino, for not supporting Math.round(), but only trunc() (casting); I do it myself
  int32_t scaled = (degrees + (degrees < 0 ? -0.000005 : 0.000005)) * LPP_GPS_LAT_LON_MULT;

  sendBuffer[offset++] = scaled >> 16;
  sendBuffer[offset++] = scaled >> 8;
  sendBuffer[offset++] = scaled;

  return offset;
}

bool waitForGPSFix()
{
  Serial.println("Waiting for GPS FIX ...");

#ifdef DISPLAY_ON
  VextON(); // oled power on;
  delay(10);
  display.init();
  display.clear();

  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 32 - 16 / 2, "GPS Searching...");
  display.drawString(80, 5, "start");
  Serial.println("GPS Searching...");
  display.display();
#endif

  GPS.begin();
  GPS.setmode(MODE_GPS);

  uint32_t start = millis();

  while (((millis() - start) < GPS_UPDATE_TIMEOUT) && (GPS.location.age() > 1000))
  {
    while (GPS.available() > 0)
    {
      if (GPS.encode(GPS.read()))
      {
        if (GPS.location.isValid())
        {
          printGPSInfo();
          displayGPSInfo();

          if (GPS.location.age() > 1000)
          {
            break;
          }
        }
      }
    }
    if (!GPS.location.isValid())
    {
// delay(1000);
// #define GO_FASTER
#ifdef GO_FASTER
      char remaining[8];
      sprintf(remaining, "%4d", (uint16_t)(GPS_UPDATE_TIMEOUT - (millis() - start)) / 1000);
      display.clear();
      display.display();
      display.drawString(64, 32 - 16 / 2, "GPS Searching...");
      display.drawString(80, 5, (String)GPS.location.age());
      display.display();
      Serial.print("visible satellites: ");
      Serial.print(GPS.satellites.value());
      Serial.print("; hdop: ");
      Serial.println(GPS.hdop.hdop());
#endif
    }
  }
  Serial.printf("leaving waiting; age is %d\n", GPS.location.age());
  return (GPS.location.isValid() && (GPS.location.age() < 1000));
}

static void prepareTxFrame()
{
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
    appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
    if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
    if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
    for example, if use REGION_CN470,
    the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */

  if (waitForGPSFix())
  {
    float lat, lon, alt, course, speed, hdop, sats;
    uint16_t batteryVoltage = getBatteryVoltage();

    Serial.print("BatteryVoltage:");
    Serial.println(batteryVoltage);

    lat = GPS.location.lat();
    lon = GPS.location.lng();
    alt = GPS.altitude.meters();
    course = GPS.course.deg();
    speed = GPS.speed.kmph();
    sats = GPS.satellites.value();
    hdop = GPS.hdop.hdop();

    appDataSize = 0;
    appDataSize = encodeDegrees(appData, appDataSize, lat);
    appDataSize = encodeDegrees(appData, appDataSize, lon);
    //  appData[appDataSize++] = (uint8_t)(batteryVoltage >> 8);
    //  appData[appDataSize++] = (uint8_t)batteryVoltage;

    Serial.print("sending bytes ");
    Serial.println(appDataSize);

// if position fixed, then let GPS sleep; otherwise, leave it on to keep searching
// #define TOO_SLOW_TO_ACQURE
#ifdef TOO_SLOW_TO_ACQURE
    GPS.end();
#else
    String cmd = "$PGKC051,0*36";
    GPS.sendcmd(cmd);
#endif

    // port 2 is gps data
    appPort = 2;
  }
  else
  {
#ifdef DISPLAY_ON
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 32 - 16 / 2, "No GPS signal");
    Serial.println("No GPS signal");
    display.display();

    delay(2000);
#endif
    // port 3 is failure
    appPort = 3;
    appDataSize = 0;
    appData[appDataSize++] = (uint8_t)(GPS.satellites.value() >> 8);
    appData[appDataSize++] = (uint8_t)(GPS.satellites.value());
    uint16_t intHDOP = GPS.hdop.hdop();
    appData[appDataSize++] = (uint8_t)(intHDOP >> 8);
    appData[appDataSize++] = (uint8_t)(intHDOP);
  }
#ifdef DISPLAY_ON
  display.clear();
  display.display();
  display.stop();
  VextOFF(); // oled power off
#endif
}

void displayQR(String message)
{
  // Create the QR code
  QRCode qrcode;
  const int qrcodeVersion = 3;
  const int pixelsPerSquare = 2;
  const int ecc = 0;

  uint8_t qrcodeData[qrcode_getBufferSize(qrcodeVersion)];
  qrcode_initText(&qrcode, qrcodeData, qrcodeVersion, ecc, (const char *)message.c_str());

  VextON(); // oled power on;
  delay(10);
  display.init();
  display.clear();

  for (uint8_t y = 0; y < qrcode.size; y++)
  {
    for (uint8_t x = 0; x < qrcode.size; x++)
    {
      // If pixel is on, we draw a ps x ps black square
      if (qrcode_getModule(&qrcode, x, y))
      {
        for (int xi = x * pixelsPerSquare + 2; xi < x * pixelsPerSquare + pixelsPerSquare + 2; xi++)
        {
          for (int yi = y * pixelsPerSquare + 2; yi < y * pixelsPerSquare + pixelsPerSquare + 2; yi++)
          {
            display.setPixel(xi, yi);
          }
        }
      }
    }
  }
  display.display();
  delay(15000);
}

void setup()
{
  Serial.begin(115200);

#if (AT_SUPPORT)
  enableAt();
#endif
  LoRaWAN.displayMcuInit();

  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
}

bool qrShown = false;

void loop()
{
  switch (deviceState)
  {
  case DEVICE_STATE_INIT:
  {
#if (LORAWAN_DEVEUI_AUTO)
    LoRaWAN.generateDeveuiByChipID();
    if (!qrShown)
    {
      String qrURL = "https://bit.ly/3MGU39B/?dev=";
      char devEUIasText[17];
      sprintf(devEUIasText, "%02x%02x%02x%02x%02x%02x%02x%02x", devEui[0], devEui[1], devEui[2], devEui[3], devEui[4], devEui[5], devEui[6], devEui[7]);
      Serial.println(devEUIasText);
      displayQR(qrURL + devEUIasText);
      qrShown = true;
    }
#endif
#if (AT_SUPPORT)
    getDevParam();
#endif
    printDevParam();
    LoRaWAN.init(loraWanClass, loraWanRegion);
    deviceState = DEVICE_STATE_JOIN;
    break;
  }
  case DEVICE_STATE_JOIN:
  {
    LoRaWAN.displayJoining();
    LoRaWAN.join();
    break;
  }
  case DEVICE_STATE_SEND:
  {
    prepareTxFrame();
    LoRaWAN.displaySending();
    LoRaWAN.send();
    deviceState = DEVICE_STATE_CYCLE;
    break;
  }
  case DEVICE_STATE_CYCLE:
  {
    // Schedule next packet transmission
    txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
    LoRaWAN.cycle(txDutyCycleTime);
    deviceState = DEVICE_STATE_SLEEP;
    break;
  }
  case DEVICE_STATE_SLEEP:
  {
    LoRaWAN.displayAck();
    // after one downlink, switch into unconfirmed moden for ifSkipJoin to work properly
    // https://github.com/HelTecAutomation/CubeCell-Arduino/issues/67
    // groot
    //      isTxConfirmed = false;
    display.stop();
    VextOFF(); // oled power off

    LoRaWAN.sleep();

    VextON(); // oled power on;
    delay(10);
    display.init();
    break;
  }
  default:
  {
    deviceState = DEVICE_STATE_INIT;
    break;
  }
  }
}