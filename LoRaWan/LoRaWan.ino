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

   set LoraWan_RGB to Active,the RGB active in loraWan
   RGB red means sending;
   RGB purple means joined done;
   RGB blue means RxWindow1;
   RGB yellow means RxWindow2;
   RGB green means received done;
*/

// This sensor is wired to Vext - so it can be powered off.
//
#define ONE_WIRE_BUS GPIO4

#ifdef ONE_WIRE_BUS
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
#endif

#if IROOT
#include "secrets.h"
#else
uint64_t chipID = ( uint64_t) 0x1234567890123456;

/* OTAA para*/
uint8_t devEui[] = { 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x04, 0x05 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x04, 0x05,
                     0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x04, 0x05 
                   };

/* ABP para -- curenntly not used*/
uint8_t nwkSKey[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t appSKey[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint32_t devAddr =  0;
#endif

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15 * 60 * 1000; // upload every 15 mins.

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
uint8_t confirmedNbTrials = 4;

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
  // list of options at https://github.com/ElectronicCats/CayenneLPP/blob/master/API.md
  //
  CayenneLPP lpp(LORAWAN_APP_DATA_MAX_SIZE);
  lpp.reset();

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(20);

#if 1
  float battV = getBatteryVoltage() / 1000.;
  Serial.printf("Battery voltage %.1f Volt\n", battV);
  lpp.addTemperature(1, battV /* is in mVolt */);
#endif

#ifdef ONE_WIRE_BUS
  sensors.begin();


  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);
  lpp.addTemperature(1, temp /* is in Celcius */);
#endif


#if  1
  static unsigned int cnt = 0;
  lpp.addDigitalInput(2, cnt++);
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
    if ((0 == i % 8) && (i < appDataSize)) Serial.print("                                  ");
  };
#endif
}


TimerEvent_t wakeUp;

void setup() {
  Serial.begin(115200);
  while (!Serial) {};
  Serial.printf("Build: %s\n", rindex(__FILE__, '/'));
  Serial.println("Compiled:" __DATE__" " __TIME__);

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH); // keep power to instruments off if we do not need them.

#if(AT_SUPPORT)
  enableAt();
#endif

#ifdef ONE_WIRE_BUS
  sensors.begin();
#endif

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
  float temp = sensors.getTempCByIndex(0);
  Serial.print("Temp: "); Serial.println(temp);
#endif
  digitalWrite(Vext, HIGH);

#if(LORAWAN_DEVEUI_AUTO)
#else
  if (i != chipID) {
    Serial.printf("Halting - as this ChipID has not been registered uet.");
    while (1) {};
  }
#endif

  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
  TimerInit( &wakeUp, onWakeUp );
}

uint8_t lowpower = 0;
void onWakeUp()
{
  Serial.println("Woke up");
  lowpower = 0;
}

void loop()
{
  static int lstate = deviceState;
  if (lstate != deviceState) {
    Serial.print("State change: ");
    Serial.print(lstate);
    Serial.print("->");
    Serial.println(deviceState);
    lstate =  deviceState;
    delay(50);
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
        break;
      }
    case DEVICE_STATE_SEND:
      {
        prepareTxFrame( appPort );
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;

        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep();

#if 0
        Serial.printf("Going into lowpower mode, %d ms later wake up.\r\n", txDutyCycleTime);
        lowpower = 1;

        //timetillwakeup ms later wake up;
        TimerSetValue( &wakeUp, txDutyCycleTime );
        TimerStart( &wakeUp );
#endif

        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}
