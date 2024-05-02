#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <machine/endian.h>
#include <CayenneLPP.h>//the library is needed ��https://github.com/ElectronicCats/CayenneLPP��
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/* LoraWAN, CubeCell and The Things Network (TTN)
   https://heltec-automation-docs.readthedocs.io/en/latest/cubecell/quick_start.html
   https://heltec-automation-docs.readthedocs.io/en/latest/cubecell/lorawan/connect_to_gateway.html
   https://github.com/LukePrior/TTN-BME280-Weather-Station-Heltec-CubeCell-HTCC-AB01

  TTN: "HTCC-AB01 Class C"

  Board:  "CubeCell-Board (HTCC-AB01)
  REGION_EU868
  CLASS_C
  DevEUI: CUSTOM
  OOTA - should therefore configure things like RX2/SF9,etc automatic.
  ADR = ON (because the device is essentially stationary)
  Linkmode = UNCONFIRMED
  NetReservation = OFF
  AT = On
  RGB = defactive
  PREAMBLE = 8 (lorawan, default)

  when set LoraWan_RGB to Active,the RGB active in loraWan
   RGB red means sending;
   RGB purple means joined done;
   RGB blue means RxWindow1;
   RGB yellow means RxWindow2;
   RGB green means received done;
*/

// This sensor is wired to Vext - so it can be powered off.
//
#define ONE_WIRE_BUS GPIO5

#ifdef ONE_WIRE_BUS
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
#endif

#if 0
#include "secrets.h"
#else
uint64_t chipID = ( uint64_t) 0x0123456789012345;

/* OTAA para*/
uint8_t devEui[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
// TTN v3 in OOTA may not like leading zero's - so give it random valuesl
uint8_t appEui[8] = {  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
// uint8_t appEui[] = { 0xB6, 0xC7, 0xB1, 0x1B, 0x76, 0x1C, 0xF6, 0x54 };
uint8_t appKey[16] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

/* ABP para -- curenntly not used*/
uint8_t nwkSKey[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t appSKey[16] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 
                    };
uint32_t devAddr =  0x01020304;;
#endif

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycleMax = 120 * 60 * 1000; // upload every 10 mins.
uint32_t appTxDutyCycle = 10 * 1000; // upload every 10 mins.

/* OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 42;
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
uint8_t confirmedNbTrials = 4; // SF8/DR
// uint8_t confirmedNbTrials = 16; // SF9/DR3 ???

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
  // list of options at https://github.com/ElectronicCats/CayenneLPP/blob/master/API.md
  //
  CayenneLPP lpp(LORAWAN_APP_DATA_MAX_SIZE);
  lpp.reset();

#ifdef ONE_WIRE_BUS
  digitalWrite(Vext, LOW);
  delay(20);
#if 0
  byte addr[8];
  if (oneWire.search(addr)) {
    oneWire.reset();
    oneWire.select(addr);
    oneWire.write(0x44, 1);        // start conversion, with parasite power on at the end
    delay(1000); // well over 750 mSeconds
    byte present = oneWire.reset();
    oneWire.select(addr);
    oneWire.write(0xBE);
    byte data[12];
    for (int i = 0; i < 9; i++)          // we need 9 bytes
      data[i] = oneWire.read();
    oneWire.depower() ;
    digitalWrite(Vext, HIGH);

    int16_t raw = (data[1] << 8) | data[0];
    if (addr[0] == 0x10) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    float temp = (float)raw / 16.0;
#else
  {
    sensors.requestTemperatures();
    delay(1000); // well over 750 mSeconds - needed for the conversion - should use a loraSleep here
    // sensors.requestTemperatures();
    float temp = sensors.getTempCByIndex(0);
    digitalWrite(Vext, HIGH);
#endif
    Serial.print("Temperature [C]: "); Serial.println(temp);
    lpp.addTemperature(1, temp /* is in Celcius */);
  };
#endif

#if  0
  static unsigned int cnt = 0;
  lpp.addLuminosity(2, cnt++); // closed to an unsigned short (frequency is unsighed int)
#endif

#if 0
  double battV = getBatteryVoltage() / 1000.;
  Serial.printf("Battery voltage [V]:"); Serial.println(battV);
  lpp.addAnalogInput(3, battV /* is in mVolt */);
  //  lpp.addVoltage(4, battV);
#endif

#if 1
  float h = gethum();
  lpp.addAnalogInput(5, h);
#endif

  // lpp.addGPS(1, 4.724002f, 52.036585f, 0.f); // ok met TTN
  // lpp.addVoltage(1, getBatteryVoltage() / 1000. /* is in mVolt */); /// niet ok met TTN
  // lpp.addVoltage(1, 0.50);

  digitalWrite(Vext, HIGH); // keep power to instruments off if we do not need them.

  appDataSize = lpp.getSize();
  uint8_t * buff = lpp.getBuffer();
  memcpy(appData, buff, appDataSize);

#if 1
  Serial.printf("Data (%d bytes <= %d max packet): ", appDataSize, LORAWAN_APP_DATA_MAX_SIZE);
  for (int i = 0; i < appDataSize ;) {
    Serial.printf(" %02x", buff[i]);
    i++;
    if ((0 == i % 8) || (i == appDataSize)) Serial.print("\n");
    if ((0 == i % 8) && (i < appDataSize)) Serial.print("                                   ");
  };
#endif
}

const char * state2str(eDeviceState_LoraWan state) {
  switch (state) {
    case DEVICE_STATE_INIT: return "LoraWan-init";
      break;
    case DEVICE_STATE_JOIN: return "LoraWan-join";
      break;
    case DEVICE_STATE_SEND: return "LoraWan-send";
      break;
    case DEVICE_STATE_CYCLE: return "LoraWan-cycle";
      break;
    case DEVICE_STATE_SLEEP: return "LoraWan-sleep";
      break;
    default:
      break;
  }
  return "LoraWan-other";
}

TimerEvent_t wakeUp;

void setup() {
  delay(1000); // terminal is slow to come up.
  Serial.begin(115200);
  while (!Serial) {};
  Serial.printf("Build: %s\n", rindex(__FILE__, '/'));
  Serial.println("Compiled:" __DATE__" " __TIME__);

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH); // keep power to instruments off if we do not need them.

#if(AT_SUPPORT)
  passthroughMode = true;
  enableAt();
#endif

#ifdef ONE_WIRE_BUS
  sensors.begin();
  if (!(sensors.getWaitForConversion()))
    Serial.println("** going wrong - no waitForConversion");
#endif

  getDevParam();
  printDevParam();

  uint64_t i = getID();
#if 1
  Serial.printf("ChipID:");
  for (int j = 0; j < sizeof(i); j++)
    Serial.printf(" %02X", ((uint8_t *)&i)[sizeof(i) - j - 1]);
  Serial.println();
  Serial.printf("      : ");
  for (int j = 0; j < sizeof(i); j++)
    Serial.printf("0x%02X,", ((uint8_t *)&i)[sizeof(i) - j - 1]);
  Serial.println();

  Serial.printf("      : 0x%08X%08X\n", (uint32_t)(i >> 32), (uint32_t)i);
#endif


#if(LORAWAN_DEVEUI_AUTO)
#else
  if (i != chipID) {
    Serial.printf("Halting - as this ChipID has not been registered uet.");
    while (1) {};
  }
#endif

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(20);

#if 1
  float battV = getBatteryVoltage() / 1000.;
  Serial.print("Batt: "); Serial.println(battV);
#endif


#ifdef ONE_WIRE_BUS
  byte addr[8];
  Serial.println("Scanning Onewire");
  while (oneWire.search(addr)) {
    Serial.print("Device = ");
    for (int i = 0; i < sizeof(addr); i++) {
      Serial.write(',');
      Serial.print(addr[i], HEX);
    };
    Serial.println();
  };
  Serial.println("Scan completed");

  sensors.requestTemperatures();
  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);
  Serial.print("Temp: "); Serial.println(temp);
#endif
  digitalWrite(Vext, HIGH);


  deviceState = DEVICE_STATE_SLEEP;
  LoRaWAN.ifskipjoin();
  deviceState = DEVICE_STATE_INIT;

  //TimerInit( &wakeUp, onWakeUp );
  Serial.println("*** LOOP started ****");
}

uint8_t lowpower = 0;
void onWakeUp()
{
  Serial.println("Woke up");
  lowpower = 0;
}

float gethum() {
  float hum0 = 0, hum1 = 0;
  pinMode(GPIO1, OUTPUT);
  digitalWrite(GPIO1, HIGH);
  for (int i = 0; i < 100; i++) {
    hum0 += analogRead(ADC);
  };
  digitalWrite(GPIO1, LOW);
  for (int i = 0; i < 100; i++) {
    hum1 += analogRead(ADC);
  };
  pinMode(GPIO1, INPUT);
  Serial.println(hum0 / 100.);
  Serial.println(hum1 / 100.);
  Serial.println(hum0 / hum1);
  Serial.println("");
  return hum0 / hum1;
}

void loop()
{

  static eDeviceState_LoraWan lstate = deviceState;
  if (lstate != deviceState) {
    Serial.print("State change: ");
    Serial.print(state2str(lstate));
    Serial.print(" (");
    Serial.print(lstate);
    Serial.print(")->");
    Serial.print(state2str(deviceState));
    Serial.print(" (");
    Serial.print(deviceState);
    Serial.println(")");
    lstate =  deviceState;
    delay(50);
  } else {
    static unsigned long lst = millis();
    static int cnt = 0;
    ++cnt;
    if (millis() - lst > 1 * 1000) {
      Serial.print("Counter: ");
      Serial.println(cnt);
      Serial.flush();
      delay(5);
      lst = millis();
    };
  };

  switch ( deviceState )
  {
    case DEVICE_STATE_INIT:
      {
#if(LORAWAN_DEVEUI_AUTO)
        Serial.println("Generating ChipID");
        LoRaWAN.generateDeveuiByChipID();
#else
        Serial.println("Using hardcoded DevUID");
#endif
#if(AT_SUPPORT)
        getDevParam();
#endif
        printDevParam();
        LoRaWAN.init(loraWanClass, loraWanRegion);
        deviceState = DEVICE_STATE_JOIN;
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        if (deviceState == DEVICE_STATE_SLEEP)
          deviceState = DEVICE_STATE_SEND;
        break;
      }
    case DEVICE_STATE_SEND:
      {

        Serial.print("Sending...");
        prepareTxFrame( appPort );
        LoRaWAN.send();
        Serial.println("completed");

        deviceState = DEVICE_STATE_CYCLE;
      }
      break;
    case DEVICE_STATE_CYCLE:
      {
        if (0) {
          prepareTxFrame( appPort );
          LoRaWAN.send();
        };

        txDutyCycleTime = appTxDutyCycle - APP_TX_DUTYCYCLE_RND / 2 + randr( 0, APP_TX_DUTYCYCLE_RND );

        appTxDutyCycle = appTxDutyCycle * 2;
        if (appTxDutyCycle > appTxDutyCycleMax) appTxDutyCycle = appTxDutyCycleMax;

        Serial.print("Schedule next transmisson in ");
        Serial.print(txDutyCycleTime / 1000.);
        Serial.println(" seconds.");

        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        LoRaWAN.sleep();

        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep();
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}
