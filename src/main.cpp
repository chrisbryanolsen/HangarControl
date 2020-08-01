#include <Arduino.h>
#include <RTCZero.h>
#include <lmic.h>
#include <hal/hal.h>
#include <ArduinoJson.h>

/*
 * Allow logging to be turned on / off
 */
const boolean LOGGING_ENABLED = true;
#define logMsg(M)  (LOGGING_ENABLED == true ? SerialUSB.print(M) : false)

/*
 * Real Time Clock for the SAM21 / Zero
 */
RTCZero rtc;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = {0x76, 0x0C, 0x03, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0x39, 0x46, 0x52, 0x41, 0x47, 0x4E, 0x41, 0x48};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = {0xD9, 0x36, 0xC1, 0xB3, 0x69, 0x96, 0x63, 0x22, 0x03, 0x37, 0x53, 0x34, 0x34, 0x8B, 0x09, 0xFF};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

/*
 * Job / thread that runs the status updates
 */
static osjob_t statusJob;

static uint8_t mydata[] = "Hello, world!";

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 12, // RFM Chip Select
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 7,           // RFM Reset
    .dio = {6, 10, 11}, // RFM Interrupt, RFM LoRa pin, RFM LoRa pin
};

/*
 * How often to send status and startup requests out.
 *  1. The startup request will trigger a 'upload' request from the control server to get the 
 *     update the current power schedule.
 *  2. Include the power relay status and the current schedule for turning power on / off
 *
 * Schedule TX every this many seconds (might become longer due to duty cycle limitations).
 */
const unsigned TX_INTERVAL = 300;   // 5 min. 

/*
 * Are we currenty waiting for a transmission to complete, if so a new tx will not be initiated
 */
static boolean txInProg = false;

/*
 * Flag to indicate if the time / rtc has been updated from the network.
 * If not then our local time is not yet valid and no scheduling should occur
 */
static boolean timeSet = false;

/*
 * Request network time, hub should send the GPS time.
 * Update our RTC with the current time for later use.
 */
static lmic_time_reference_t netTime;
void network_time_cb(void * pUserData, int flagSuccess) {
  if (LMIC_getNetworkTimeReference(&netTime) > 0) {
    logMsg(F("Network Time Recived, Update RTC, time: "));
    logMsg(netTime.tNetwork);
    rtc.setEpoch(netTime.tNetwork);
    timeSet = true;
  }
}

void requestTime() {
  char *ptr = NULL;
  LMIC_requestNetworkTime(&network_time_cb, ptr);
}

/*
 * If logging is turned on get the serial port setup and wait for it to be ready
 */
void initSerial() {
   if (LOGGING_ENABLED == true) {
     SerialUSB.begin(115200);
     
    // Serial communication on startup is not consistent on the SAMD21. The
    // following line waits for the serial monitor to be opened before
    // continuing. Uncomment if not needed.
    while(!SerialUSB);
    
    SerialUSB.println("Starting");
   }
}

void print2digits(int number) {
  if (number < 10) {
    logMsg(F("0")); // print a 0 before if the number is < than 10
  }
  logMsg(number);
}

void printRTCTime() {
  // Print date...
  print2digits(rtc.getDay());
  logMsg(F("/"));
  print2digits(rtc.getMonth());
  logMsg(F("/"));
  print2digits(rtc.getYear());
  logMsg(F(" "));

  // ...and time
  print2digits(rtc.getHours());
  logMsg(F(":"));
  print2digits(rtc.getMinutes());
  logMsg(F(":"));
  print2digits(rtc.getSeconds());
}

void processDownlink(lmic_t LMIC) {
  if (LMIC.dataLen) {
    logMsg(F("Received "));
    logMsg(LMIC.dataLen);
    logMsg(F(" bytes of payload\n"));

    // Data is in: LMIC.frame + LMIC.dataBeg, LMIC.dataLen
  }
}

void onEvent (ev_t ev) {
    //logMsg(os_getTime());
    printRTCTime();
    logMsg(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            logMsg(F("EV_SCAN_TIMEOUT\n"));
            break;
        case EV_BEACON_FOUND:
            logMsg(F("EV_BEACON_FOUND\n"));
            break;
        case EV_BEACON_MISSED:
            logMsg(F("EV_BEACON_MISSED\n"));
            break;
        case EV_BEACON_TRACKED:
            logMsg(F("EV_BEACON_TRACKED\n"));
            break;
        case EV_JOINING:
            logMsg(F("EV_JOINING\n"));
            break;
        case EV_JOINED:
            logMsg(F("EV_JOINED\n"));
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            requestTime();
            break;
        case EV_RFU1:
            logMsg(F("EV_RFU1\n"));
            break;
        case EV_JOIN_FAILED:
            logMsg(F("EV_JOIN_FAILED\n"));
            break;
        case EV_REJOIN_FAILED:
            logMsg(F("EV_REJOIN_FAILED\n"));
            break;

        case EV_TXCOMPLETE:
            logMsg(F("EV_TXCOMPLETE (includes waiting for RX windows)\n"));
            if (LMIC.txrxFlags & TXRX_ACK) {
              logMsg(F("Received ack\n"));
            }

            // Mark the transmission complete
            txInProg = false;

            // If any data recieved, process it
            processDownlink(LMIC);

            break;
        case EV_LOST_TSYNC:
            logMsg(F("EV_LOST_TSYNC\n"));
            break;
        case EV_RESET:
            logMsg(F("EV_RESET\n"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            logMsg(F("EV_RXCOMPLETE\n"));

            // If any data recieved, process it
            processDownlink(LMIC);
            break;
        case EV_LINK_DEAD:
            logMsg(F("EV_LINK_DEAD\n"));
            break;
        case EV_LINK_ALIVE:
            logMsg(F("EV_LINK_ALIVE\n"));
            break;
        case EV_TXSTART:
            logMsg(F("EV_TXSTART\n"));
            break;
        case EV_JOIN_TXCOMPLETE :
            logMsg(F("EV_JOIN_TXCOMPLETE \n"));
            break;
         default:
            logMsg(F("Unknown event: "));
            logMsg(ev);
            logMsg("\n");
            break;
    }
}

void do_send() {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        logMsg(F("OP_TXRXPEND, not sending\n"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        logMsg(F("Packet queued\n"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

/*
 * Main work method, will perform any scheduled tasks and send status updates
 *  1. Check the schedule and turn the power on or off if needed.
 *  2. Send a status update with the current power state and schedule
 */
void statusUpdate(osjob_t* j) {
    /*
     * Check the current schedule for any power on / off changes
     */

    /*
     * Trigger the status update transmit
     */
    if (txInProg == false) {
        do_send();
    }

    /*
     * Schedule the next status / work update run
     */
    os_setTimedCallback(&statusJob, os_getTime()+sec2osticks(TX_INTERVAL), statusUpdate);
}

void setup()
{
  initSerial();
  rtc.begin();  // Start up the Real Time Clock

  // LMIC init
  // Reset the MAC state. Session and pending data transfers will be discarded.
  os_init();
  LMIC_reset();

#if defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif

  // Start job (sending automatically starts OTAA too)
  statusUpdate(&statusJob);
}

void loop()
{
  os_runloop_once();
}