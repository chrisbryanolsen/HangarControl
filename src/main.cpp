#include <Arduino.h>
#include <RTCZero.h>
#include <lmic.h>
#include <hal/hal.h>
#include <ArduinoJson.h>
#include <Schedule.hpp>

/*
 * Allow logging to be turned on / off
 */
const boolean LOGGING_ENABLED = true;
#define logMsg(M) (LOGGING_ENABLED == true ? SerialUSB.print(M) : false)

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

/*
 * Command uplink queue and structure
 */
static StaticJsonDocument<MAX_LEN_PAYLOAD> cmdJson;
static StaticJsonDocument<512> payloadJson;
static boolean startUpComplete = false;

/*
 * Array of Power schedules
 */
static Schedule powerSched[25];
static u_int8_t schedCount = 0;
static boolean powerState[] = {false, false}; // Default both power switches to OFF

/*
 * Command uplink queue and structure
 */

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 12, // RFM Chip Select
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 7,           // RFM Reset
    .dio = {6, 10, 11}, // RFM Interrupt, RFM LoRa pin, RFM LoRa pin
};

/*
 * How often to send status and startup requests out.
 *  1. The startup request will trigger a downlink request from the control server to get the 
 *     the current power schedule and time.
 *  2. Include the power relay status and the current schedule for turning power on / off
 *
 * Schedule TX every this many seconds (might become longer due to duty cycle limitations).
 */
const unsigned TX_INTERVAL = 30;

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
void network_time_cb(void *pUserData, int flagSuccess)
{
    if (LMIC_getNetworkTimeReference(&netTime) > 0)
    {
        logMsg(F("Network Time Recived, Update RTC, time: "));
        logMsg(netTime.tNetwork);
        rtc.setEpoch(netTime.tNetwork);
        timeSet = true;
    }
}

void requestTime()
{
    logMsg(F("Network Time Requested\n"));
    char *ptr = NULL;
    LMIC_requestNetworkTime(&network_time_cb, ptr);
}

/*
 * If logging is turned on get the serial port setup and wait for it to be ready
 */
void initSerial()
{
    if (LOGGING_ENABLED == true)
    {
        SerialUSB.begin(115200);

        // Serial communication on startup is not consistent on the SAMD21. The
        // following line waits for the serial monitor to be opened before
        // continuing. Uncomment if not needed.
        while (!SerialUSB)
            ;

        SerialUSB.println("Starting");
    }
}

void print2digits(int number)
{
    if (number < 10)
    {
        logMsg(F("0")); // print a 0 before if the number is < than 10
    }
    logMsg(number);
}

void printRTCTime()
{
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

// Calculate the current day of the week as an integer
//   now - Unix timestamp like that from time(NULL)
//   tz_offset - Number of hours off from UTC; i.e. PST = -8
//   Return value: Sunday=0, Monday=1, ... Saturday=6
int dayOfWeek(time_t now, int tz_offset)
{
    // Calculate number of seconds since midnight 1 Jan 1970 local time
    time_t localtime = now + (tz_offset * 60 * 60);
    // Convert to number of days since 1 Jan 1970
    int days_since_epoch = localtime / 86400;
    // 1 Jan 1970 was a Thursday, so add 4 so Sunday is day 0, and mod 7
    int day_of_week = (days_since_epoch + 4) % 7;

    return day_of_week;
}

void checkSchedules()
{
    int curDOW = dayOfWeek(rtc.getEpoch(), 0);

    logMsg(F("Check Power Schedule, Current DOW: "));
    logMsg(curDOW);
    logMsg(F("\n"));

    // Run through all schedules
    boolean newState = false;
    for (int i = 0; i < schedCount; ++i)
    {
        /*
        logMsg(F("Sched: "));
        logMsg(i);
        logMsg(F("\n"));

        logMsg(powerSched[i].dow);
        logMsg(F(" == "));
        logMsg(curDOW);
        logMsg(F("\n"));

        logMsg(powerSched[i].hour);
        logMsg(F(" == "));
        logMsg(rtc.getHours());
        logMsg(F("\n"));

        logMsg(powerSched[i].min);
        logMsg(F(" >= "));
        logMsg(rtc.getMinutes());
        logMsg(F("\n"));

        logMsg(powerSched[i].powerState);
        logMsg(F(" == "));
        logMsg(powerState[0]);
        logMsg(F("\n"));
        */

        if (curDOW == powerSched[i].dow && rtc.getHours() == powerSched[i].hour && rtc.getMinutes() >= powerSched[i].min)
        {
            newState = powerSched[i].powerState;
        }
    }

    if (powerState[0] != newState)
    {
        powerState[0] = newState;
        if (newState == true)
        {
            logMsg(F("\n*** Turn Power ON ***\n\n"));
        }
        else
        {
            logMsg(F("\n*** Turn Power OFF ***\n\n"));
        }
    }
}

void processDownlink(lmic_t LMIC)
{
    if (LMIC.dataLen)
    {
        logMsg(F("Received "));
        logMsg(LMIC.dataLen);
        logMsg(F(" bytes of payload\n"));

        // Data is in: LMIC.frame + LMIC.dataBeg, LMIC.dataLen
        payloadJson.clear();
        DeserializationError err = deserializeMsgPack(payloadJson, LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        if (err)
        {
            logMsg(F("deserializeJson() failed with code: "));
            logMsg(err.c_str());
            logMsg(F("\n"));
        }
        else
        {
            const String cmd = payloadJson["cmd"];
            const u_int32_t curTime = payloadJson["cur-time"];

            logMsg(F("Command: "));
            logMsg(cmd);
            logMsg(F(", Time: "));
            logMsg(curTime);
            logMsg(F("\n"));

            if (cmd.equalsIgnoreCase("init") == true)
            {
                rtc.setEpoch(curTime);

                const JsonArray schedAry = payloadJson["cmd-data"];
                schedCount = schedAry.size();
                for (int i = 0; i < schedCount; ++i)
                {
                    const String sched = schedAry.getElement(i);
                    DynamicJsonDocument oneSched(JSON_OBJECT_SIZE(3) + 20);
                    deserializeJson(oneSched, sched);

                    powerSched[i].powerState = oneSched["st"].as<bool>();
                    powerSched[i].dow = oneSched["dow"].as<int>();

                    const char *time = oneSched["tm"].as<char *>();
                    char hour[3];
                    strncpy(hour, time, 2);
                    char min[3];
                    strncpy(min, time + 2, 2);

                    powerSched[i].hour = atoi(hour);
                    powerSched[i].min = atoi(min);

                    logMsg(F("Sched ["));
                    logMsg(i);
                    logMsg(F("]: State: "));
                    logMsg(powerSched[i].powerState);
                    logMsg(F(", DOW: "));
                    logMsg(powerSched[i].dow);

                    logMsg(F(", Time: "));
                    print2digits(powerSched[i].hour);
                    logMsg(F(":"));
                    print2digits(powerSched[i].min);
                    logMsg(F("\n"));
                }

                startUpComplete = true;
                checkSchedules();
            }
        }
    }
}

// We have completed joining the network
// 1. set the LMIC / TTN options required
// 2. Request the network time (This does not seem to be working yet on the TTN)
// 3. Send the Startup command to the HangarServer
void joinComplete()
{
    logMsg(F("EV_JOINED\n"));

    // Disable link check validation (automatically enabled
    // during join, but not supported by TTN at this time).
    LMIC_setLinkCheckMode(0);
    requestTime();
}

void onEvent(ev_t ev)
{
    //logMsg(os_getTime());
    printRTCTime();
    logMsg(": ");
    switch (ev)
    {
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
        joinComplete();
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
        if (LMIC.txrxFlags & TXRX_ACK)
        {
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
    case EV_JOIN_TXCOMPLETE:
        logMsg(F("EV_JOIN_TXCOMPLETE \n"));
        break;
    default:
        logMsg(F("Unknown event: "));
        logMsg(ev);
        logMsg(F("\n"));
        break;
    }
}

void do_send()
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        logMsg(F("OP_TXRXPEND, not sending\n"));
    }
    else
    {
        /*
     * If we have any commands queued internally then prepare upstream data transmission at the next possible time.
     * And add the command to the LMIC send queue.
     */
        printRTCTime();
        logMsg(F(" Command JSON, Entries: "));
        logMsg(cmdJson.size());
        logMsg(F("\n"));

        if (cmdJson.size() > 0)
        {
            logMsg(F("MessagePack, size: "));
            logMsg(measureJson(cmdJson));
            logMsg(F("\n"));

            size_t msgLen = serializeMsgPack(cmdJson, &LMIC.pendTxData, MAX_LEN_PAYLOAD);
            lmic_tx_error_t sndErr = LMIC_setTxData2(1, NULL, msgLen, 0);
            if (sndErr != 0)
            {
                logMsg(F("Send Command error : "));
                logMsg(sndErr);
                logMsg(F("\n"));
            }
            else
            {
                logMsg(F("Transmit, size: "));
                logMsg(msgLen);
                logMsg(F("\n"));
            }

            cmdJson.clear();
        }
    }
}

/*
 * Main work method, will perform any scheduled tasks and send status updates
 *  1. Check the schedule and turn the power on or off if needed.
 *  2. Send a status update with the current power state and schedule
 */
void statusUpdate(osjob_t *j)
{

    /*
     * Send the initial startup commond just once
     */
    if (startUpComplete == false)
    {
        logMsg(F("Queue Startup Req\n"));
        cmdJson["cmd"] = "start";
        cmdJson["my-time"] = rtc.getEpoch();
    }

    /*
     * Send a status / power state update every 5 min.
     */
    if (startUpComplete && rtc.getMinutes() % 5 == 0)
    {
        DynamicJsonDocument startDoc(JSON_ARRAY_SIZE(3));
        JsonArray stateArray = startDoc.to<JsonArray>();
        stateArray.add(powerState[0]); // Power port 1 status
        stateArray.add(powerState[1]); // Power port 2 status

        logMsg(F("Queue Status Req\n"));
        cmdJson["cmd"] = "status";
        cmdJson["my-time"] = rtc.getEpoch();
        cmdJson["state"] = stateArray;
    }

    /*
     * Check the current schedule for any power on / off changes
     */
    if (startUpComplete)
    {
        checkSchedules();
    }

    /*
     * Attempt to send any queued commands.
     */
    if (txInProg == false)
    {
        do_send();
    }

    /*
     * Schedule the next status / work update run
     */
    os_setTimedCallback(&statusJob, os_getTime() + sec2osticks(TX_INTERVAL), statusUpdate);
}

void setup()
{
    initSerial();
    rtc.begin(); // Start up the Real Time Clock

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