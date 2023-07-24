#include <Arduino.h>
#include <ArduinoBLE.h>
#include <mbed.h>
#include <rtos.h>
#include <Wire.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <iomanip>
#include "PN532/Adafruit_PN532.h"
#include "NDEF/NDEF_Message.h"
#include "CRC32/SerialBuffer.h"
#include "CRC32/CRC32.h"

#pragma once
#include <stdint.h>

//------------------------------------------------------------------------------------------------

using namespace mbed;
using namespace std::chrono;

//------------------------------------------------------------------------------------------------

#pragma region BLUETOOTH LOW ENERGY SUPPORT
// BLE service descriptors
#define UUID_SERVICE_NORDIC_SPP "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"         // UUID for NORDIC SPP UART
#define UUID_SERVICE_BATTERY "0000180F-0000-1000-8000-00805F9B34Fb"            // UUID for the battery service
#define UUID_SERVICE_DEVICE_INFORMATION "0000180A-0000-1000-8000-00805F9B34Fb" // UUID for the device information service

// BLE service characteristics
#define UUID_CHARACTERISTIC_SPP_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"       // NORDIC SPP UART receive data
#define UUID_CHARACTERISTIC_SPP_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"       // NORDIC SSP UART transmit data
// ------------------------------------------------------------------------------------------------
#define UUID_CHARACTERISTIC_BATTERY "00002A19-0000-1000-8000-00805f9b34fb"      // battery level characteristic
// ------------------------------------------------------------------------------------------------
#define UUID_CHARACTERISTIC_MODEL "00002A24-0000-1000-8000-00805f9b34fb"        // model number characteristic
#define UUID_CHARACTERISTIC_SERIAL "00002A25-0000-1000-8000-00805f9b34fb"       // serial number characteristic
#define UUID_CHARACTERISTIC_FIRMWARE "00002A26-0000-1000-8000-00805f9b34fb"     // firmware revision characteristic
#define UUID_CHARACTERISTIC_HARDWARE "00002A27-0000-1000-8000-00805f9b34fb"     // hardware revision characteristic
#define UUID_CHARACTERISTIC_MANUFACTURER "00002A29-0000-1000-8000-00805f9b34fb" // manufacturers name for device information service

// device characteristics
#define LOCAL_NAME_OF_PERIPHERAL "Ecco-Smart-HF BLE"
#define PERIPERHAL_DEVICE_NAME "Ecco-Smart-HF BLE"
#define MANUFACTURER_NAME_STRING "FEIG ELECTRONIC GmbH"
#define MODEL_NAME_STRING "Ecco-Smart-Plus"
#define HARDWARE_NAME_STRING "Emulator by SKF (UK)"
#define FIRMWARE_NAME_STRING "20230703-2202"
#define SERIAL_NO_NAME_STRING "HF-BLE-0000001-DEV"

// set the manufacturer code to 'SKF (U.K.) Limited'
const uint8_t SKF_MANUFACTURER_CODE[2] = {0x0e, 0x04};

// Setup the incoming data characteristic (RX).
#define RX_BUFFER_SIZE 32
#define RX_BUFFER_FIXED_LENGTH false

// Setup the outgoinging data characteristic (TX).
#define TX_BUFFER_SIZE 32
#define TX_BUFFER_FIXED_LENGTH false

#define BLE_ATTRIBUTE_MAX_VALUE_LENGTH 512

// Buffer to read samples into, each sample is 16-bits
uint8_t configBuffer[RX_BUFFER_SIZE];

// add each of the core services
BLEService nearFieldService(UUID_SERVICE_NORDIC_SPP);
BLEService deviceInfoService(UUID_SERVICE_DEVICE_INFORMATION);
BLEService batteryService(UUID_SERVICE_BATTERY);

// RX / TX Characteristics for BYTE ARRAYS
BLECharacteristic rxChar(UUID_CHARACTERISTIC_SPP_RX, BLEWriteWithoutResponse | BLEWrite, RX_BUFFER_SIZE, RX_BUFFER_FIXED_LENGTH);
BLECharacteristic txChar(UUID_CHARACTERISTIC_SPP_TX, BLERead | BLENotify, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);

// battery characteristics
BLECharacteristic batteryCharacteristic(UUID_CHARACTERISTIC_BATTERY, BLERead | BLENotify, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);

// device characteristics
BLECharacteristic manufacturerCharacteristic(UUID_CHARACTERISTIC_MANUFACTURER, BLERead, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);
BLECharacteristic firmwareRevisionCharacteristic(UUID_CHARACTERISTIC_FIRMWARE, BLERead, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);
BLECharacteristic modelNumberCharacteristic(UUID_CHARACTERISTIC_MODEL, BLERead, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);
BLECharacteristic hardwareCharacteristic(UUID_CHARACTERISTIC_HARDWARE, BLERead, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);
BLECharacteristic serialNumberCharacteristic(UUID_CHARACTERISTIC_SERIAL, BLERead, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);
#pragma endregion

//------------------------------------------------------------------------------------------------

#define UID_LENGTH 7               // byte size of NTAG UID
#define BLOCK_SIZE 16              // block size in bytes
#define BLOCK_COUNT 16             // number of blocks to read from the card
#define TOTAL_BLOCKS 86            // total number of NDEF blocks
#define BYTES_PER_BLOCK 4          // number of bytes per block
#define SERIAL_BAUD_RATE 115200    // serial port baud rate
#define WAIT_FOR_CARD_MS 100       // how long to wait for a card before continuing
#define COMMAND_BYTES 6            // how many NDEF records should we ordinarily expect?
#define SYSTEM_TIMEOUT 30000       // reset system to default after this mS
#define COMMAND_TIMEOUT 1000       // reset system to default after this mS
#define NEXT_SCAN_DELAY 1000       // how long to wait before the next scan
#define PLEASE_WAIT 0x2e           // full stop character
#define BLOCK_SIZE_BLE 16          // block size in bytes
#define BLOCK_WAIT_BLE 50000       // wait 50ms between each BLE transmit packet
#define BATTERY_READ_DELAY 1000    // wait 1ms between each read of the battery pin
#define COMMAND_LED_FLASH 20       // wait 20ms between each BLE transmit packet
#define TICK_RATE_MS 200ms         // update rate for the mbed timer
#define READ_BATTERY_AVG 10        // how many samples to average to calculate the supply voltage
#define BATTERY_UPDATE_COUNTER 300 // how many MBED timer clicks before we update the battery
#define BATTERY_DESCALE 15.163     // multiply ADC value with this to get the measured mV

//------------------------------------------------------------------------------------------------

#define NTAG_IC_TYPE 12              // NTAG byte which describes the actual card type
#define NTAG_CAPABILITY_CONTAINER 14 // NTAG byte which details the total number of user bytes available
#define NTAG_DEFAULT_PAGE_CLEAR 16   // how many pages should be cleared by default before a write action
#define NTAG_SINGLE_WRITE_BYTES 48   //	number of characters per single NDEF record write
#define NTAG_SINGLE_BINARY_BYTES 49  //	number of characters per single NDEF record write
#define NTAG_MAX_RECORD_BYTES 448    //	absolute maximum number of characters per NDEF record

//------------------------------------------------------------------------------------------------

#define INVALID_NDEF "INVALID NDEF RECORD" // no valid NDEF records could be found
#define OPCODE_BYTES 2                     // how many bytes make up an OPCODE
#define OPERAND_BYTES 4                    // how many bytes make up an OPERAND

//------------------------------------------------------------------------------------------------

#define NTAG_213_IC 0x12 // byte code (payload[14]) that identifies and NTAG-213 card
#define NTAG_215_IC 0x3e // byte code (payload[14]) that identifies and NTAG-215 card
#define NTAG_216_IC 0x6d // byte code (payload[14]) that identifies and NTAG-216 card

#define NTAG_213_MEMORY 0x0090 // 144 bytes
#define NTAG_215_MEMORY 0x01f0 // 496 bytes
#define NTAG_216_MEMORY 0x0386 // 872 bytes

//------------------------------------------------------------------------------------------------

#define PN532_SCK (13)  // SPI pin SCLK
#define PN532_MISO (12) // SPI pin MISO
#define PN532_MOSI (11) // SPI pin MOSI
#define PN532_SS (10)   // SPI pin SS
#define GPIO_PIN_4 4    // BLE connected LED pin
#define COMMS_LED 7     // NFC activity LED

//------------------------------------------------------------------------------------------------

// DEBUG CONTROLLERS - REMOVE COMMENT BLOCKS TO ENABLE OUTPUT OVER SERIAL

// #define READER_DEBUG
// #define READER_DEBUG_APPEND_FUNCTIONALITY
// #define SERIAL_RECEIVE_DEBUG
#define READER_BROADCAST_DEBUG
#define READER_DEBUGPRINT Serial

//------------------------------------------------------------------------------------------------

/// @brief UNCOMMENT THIS LINE TO ALLOW SUPPORT FOR NORDIC SPP UART
#define NORDIC_SPP_FUNCTIONALITY

/// @brief  when set TRUE, hex strings are expressed in UPPER CASE
#define HEX_UPPER_CASE true

/// @brief  when set TRUE, when SCAN output is to be ASCII (binary) instead of HEX based text
#define SET_OUTPUT_AS_BINARY true

//------------------------------------------------------------------------------------------------

/// @brief configure and initialise the NFC reader (we use configurable I/O for this)
Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

//------------------------------------------------------------------------------------------------

uint8_t READER_TIMEOUT[5] = {0x2a, 0x54, 0x2f, 0x0d, 0x0a};
uint8_t CC[4] = {0xe1, 0x10, 0x12, 0x00};

//------------------------------------------------------------------------------------------------
// RECEIVED COMMAND SET:
//
// These are commands received from the connected controller. They are of the format:
//      [ 00 ], [ 01 ] VERB or OPCODE
//      [ 02 ], [ 03 ] .. [ 26 ] NOUN or OPERAND
//
// List of supported OPCODES:
//

/// @brief > continuous read and return of detected card contents
uint8_t CONTINUOUS_READ_CARD[OPCODE_BYTES] = {0x00, 0x00};

/// @brief > on receipt of this command, read once and return detected card contents
uint8_t SINGLE_READ_CARD[OPCODE_BYTES] = {0x00, 0x01};

/// @brief > add a single NDEF record to the reader's cache memory
uint8_t ADD_NDEF_RECORD[OPCODE_BYTES] = {0x00, 0x02};

/// @brief > add a single NDEF record to the reader's cache memory
uint8_t APPEND_NDEF_RECORD[OPCODE_BYTES] = {0x00, 0x03};

/// @brief > how many NDEF records are in the device's cache memory
uint8_t COUNT_NDEF_RECORDS[OPCODE_BYTES] = {0x00, 0x04};

/// @brief > manually clear reader's internal NDEF cache memory
uint8_t CLEAR_NDEF_CACHE[OPCODE_BYTES] = {0x00, 0x05};

/// @brief > brute-force publish all cached NDEF records to the detected card
uint8_t PUBLISH_TO_CARD[OPCODE_BYTES] = {0x00, 0x06};

/// @brief > erase all NDEF records from a detected card
uint8_t ERASE_CARD_CONTENTS[OPCODE_BYTES] = {0x00, 0x07};

/// @brief > what is the total number of bytes in the NDEF cache?
uint8_t GET_ENCODED_SIZE[OPCODE_BYTES] = {0x00, 0x08};

/// @brief > the previous payload was received without errors
uint8_t LAST_PAYLOAD_RECEIVED[OPCODE_BYTES] = {0x00, 0x09};

/// @brief > the previous payload failed and needs to be resent
uint8_t RESEND_FAILED_PAYLOAD[OPCODE_BYTES] = {0x00, 0x0a};

//------------------------------------------------------------------------------------------------
// TRANSMITTED REPLY PACKETS:
//
// These are the four byte packets that will be returned in response to a received COMMAND
// They are of the format:
//      [ 00 ], [ 01 ], [ CR ], [ LF ]
//
// byte [ 00 ] - value 0x00 = SUCCESS, value 0x0n = FAILURE WITH ERROR CODE (default is 0x00)
//      [ 01 ] - value between 0x00 and 0xff (default is 0x00)
//
// > end of successfully transmitted payload

#define HEADER_BYTES 19           // how many bytes make up the complete SCOMP PROTOCOL header
#define QUERY_HEADER_BYTES 10     // how many bytes in a QUERY payload form the SCOMP PROTOCOL header
#define RESPONSE_HEADER_BYTES 10  // how many bytes in a RESPONSE payload form the SCOMP PROTOCOL header
#define RFID_RESPONSE_BYTES 9     // how many bytes in the SCOMP RFID response data header
#define FOOTER_BYTES 4            // how many bytes make up the CRC32 block
#define LENGTH_BYTES 2            // how many bytes make up the CRC32 block
#define QUERY_OFFSET_BYTES 4      // how many bytes should we skip before we hit the QUERY (Q) char
#define CRC32_CHARACTERS 8        // how many ASCII HEX characters are in a CRC32
#define RECEIVE_BUFFER_LENGTH 128 // maximum number of command bytes we can accept

/// @brief  > RECORD HEADER
/// @brief    These ten bytes describe both the data type as well as the total number of bytes
uint8_t PAYLOAD_LEGTH[LENGTH_BYTES] = {0x00, 0x00};

/// @brief  > END OF RECORD four byte CRC32
uint8_t EOR[FOOTER_BYTES] = {0x00, 0x00, 0x00, 0x00};

/// @brief  > search bytes to detect the start of an NDEF record
uint8_t NDEF_RECORD_HEADER[3] = {0x02, 0x65, 0x6E};

/// @brief  > BASIC CARRIAGE RETURN \ LINE FEED
uint8_t CR_LF[2] = {0x0d, 0x0a};

/// @brief  > error in attempting to publish to card (out of memory)
uint8_t READ_ERROR_UNKNOWN[4] = {0x01, 0x01, 0x0d, 0x0a};

/// @brief  > error in attempting to publish to card (card could not be found)
uint8_t WRITE_ERROR_DISCONNECT[4] = {0x02, 0x01, 0x0d, 0x0a};

/// @brief  > error in attempting to publish to card (out of memory)
uint8_t WRITE_ERROR_OVERRUN[4] = {0x03, 0x01, 0x0d, 0x0a};

/// @brief  > error in attempting to read NDEF data from an empty card
uint8_t CARD_ERROR_EMPTY[4] = {0x04, 0x01, 0x0d, 0x0a};

/// @brief  > stores a copy of the last read UID
uint8_t NTAG_UUID[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//------------------------------------------------------------------------------------------------

uint8_t NDEF_EN_RECORD_EXTRA_PAGE_BYTES = 0x05;
uint8_t NDEF_EN_RECORD_TNF = 0x03;
uint8_t INVALID_UID = 0xff;

//------------------------------------------------------------------------------------------------

///
/// @brief  Describes each of the commands that this reader supports
///
enum PN532_command : uint8_t
{
    ReadCardContinuous,
    ReadCardOnce,
    AddNdefRecordToCache,
    AppendToCachedNdefRecord,
    CountCachedNdefRecords,
    EraseCachedNdefRecords,
    PublishCacheToCard,
    EraseCardContents,
    GetEncodedSize,
    LastPayloadReceived,
    ResendFailedPayload
};

///
/// @brief  Supported NTAG formats
///
enum NTAG : uint8_t
{
    UNKNOWN,
    TYPE_213,
    TYPE_215,
    TYPE_216,
};

//------------------------------------------------------------------------------------------------

///
/// @brief  Describes each of the commands that this reader supports
///
enum SCOMP_command : uint8_t
{
    none = 0x00,
    getversion = 0x01,
    beep = 0x02,
    vibrate = 0x03,
    leds = 0x04,
    barscan = 0x05,
    rfidscanUSR = 0x06,
    rfidscanTID = 0x07,
    rfidwrite = 0x08,
    getcache = 0x09,
    clearcache = 0x0A,
    rfiderase = 0x0B 
};

const size_t SCOMP_COMMAND_COUNT = 11;

const char SCOMP_GET_VERSION[] = "getversion:";
const char SCOMP_BEEP[] = "beep:";
const char SCOMP_VIBRATE[] = "vibrate:";
const char SCOMP_LEDS[] = "leds:";
const char SCOMP_BAR_SCAN[] = "barscan:";
const char SCOMP_RFID_SCAN_USR[] = "rfidscan:usr";
const char SCOMP_RFID_SCAN_TID[] = "rfidscan:tid";
const char SCOMP_RFID_WRITE[] = "rfidwrite:";
const char SCOMP_GET_CACHE[] = "getcache:";
const char SCOMP_CLEAR_CACHE[] = "clearcache:";
const char SCOMP_RFID_ERASE[] = "rfiderase:";

///
/// @brief array of command strings
///
const std::string scompCommands[SCOMP_COMMAND_COUNT] = {SCOMP_GET_VERSION,
                                                        SCOMP_BEEP,
                                                        SCOMP_VIBRATE,
                                                        SCOMP_LEDS,
                                                        SCOMP_BAR_SCAN,
                                                        SCOMP_RFID_SCAN_USR,
                                                        SCOMP_RFID_SCAN_TID,
                                                        SCOMP_RFID_WRITE,
                                                        SCOMP_GET_CACHE,
                                                        SCOMP_CLEAR_CACHE,
                                                        SCOMP_RFID_ERASE};

//------------------------------------------------------------------------------------------------

///
/// @brief  MBED* control the BLE connected pin
///
DigitalOut LED_SetConnectedToBLE(digitalPinToPinName(GPIO_PIN_4));

//------------------------------------------------------------------------------------------------

#pragma region PRIVATE MEMBERS
/// @brief MBED RTOS timer
Ticker timer;
volatile bool timerEvent = false;

/// @brief  current time (for TIMEOUT management)
unsigned long currentTime = 0;

/// @brief  what is the reader required to do?
PN532_command _command = ReadCardContinuous;

/// @brief  block access to the reader hardware?
volatile bool _blockReader = false;

/// @brief  when set true, we need to block all other I/O activites
volatile bool _readerBusy = false;

/// @brief  references the UID from the TAG to block multiple reads
uint8_t _headerdata[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/// @brief  create a new NDEF message
NDEF_Message *ndef_message = new NDEF_Message();

/// @brief  enable TIMEOUTS (for WRITE or ONE SHOT)?
volatile bool _enableTimeouts = false;

/// @brief  when incremented to a specified value, publish the current battery level
uint16_t _batteryCount = BATTERY_UPDATE_COUNTER;

/// @brief  calculate the CRC value of any byte or character stream
CRC32 crc;

/// @brief managed serial receive buffer (non-rotating!)
SerialBuffer<RECEIVE_BUFFER_LENGTH> _SerialBuffer;

/// @brief SCANNDY SCOMP message identifier as a 16 bit unsigned integer
uint16_t _messageIdentifier = 0x0000;

/// @brief has the reader received an SCANNDY SCOMP query?
volatile bool _queryReceived = false;

/// @brief has the reader received an SCANNDY SCOMP query?
volatile bool _invalidQueryReceived = false;

/// @brief what command type was issued by the connected client?
SCOMP_command _scomp_command = none;

/// @brief create the default SCANNDY PROTOCOL header for returning NFC payload data
char scomp_rfid_response_header[] = "0000R0000#rfiddata:";

/// @brief create the default SCANNDY PROTOCOL header for returning an OK response
char scomp_ok_response_header[] = "0000R0000#";

/// @brief scomp query and response message identifier represented as a four character long string
char scomp_query_ID[] = "0000";

/// @brief scomp default response to a successfully received and processed query
char scomp_response_ok[] = "ok";

/// @brief scomp default response to a invalid processed query (E.g. wrong CRC32 value)
char scomp_response_error[] = "error";

/// @brief SKF INTERNAL ** 
char scomp_response_processing[] = "...";
#pragma endregion

//------------------------------------------------------------------------------------------------

#pragma region METHOD PROTOTYPES
int GetPageCount(int);
char *Substring(char *, int, int);
const char *HexStr(const uint8_t *, int, bool);
NTAG GetCardType(uint8_t *);
size_t WriteToSPP(uint8_t);
static void onBLEWritten(BLEDevice, BLECharacteristic);
uint16_t GetTotalCardMemory(NTAG);
uint16_t ReadBattery(pin_size_t, int);
uint8_t Read_PN532(uint8_t *, uint8_t *);
void AddBatteryServiceBLE();
void AddDataServiceBLE();
void AddDeviceServiceBLE();
void AddNdefRecordToMessage(byte *, int);
void AddNdefTextRecordToMessage(byte *, int);
void AtTime(void);
void ClearTagIdentifier();
void ClearTheCard(uint8_t *, uint8_t *);
void ConnectToReader(void);
void DebugPrintCache();
void ExecuteReaderCommands(uint8_t *, uint8_t *);
void FlashLED(int, int);
void GetCachedRecordCount(uint8_t &);
void InsertSubstring(char *, const char *, int);
void onBLEConnected(BLEDevice);
void onBLEDisconnected(BLEDevice);
void onRxCharValueUpdate(BLEDevice, BLECharacteristic);
void ProcessClearCache();
void ProcessEraseTag();
void ProcessGetCache();
void ProcessReceivedQueries();
void ProcessRfidWriteQuery(char *, size_t);
void ProcessSingleScanUSR(char *, size_t);
void PublishBattery();
void PublishBinaryPayloadToBluetooth(uint8_t *, uint8_t *);
void PublishBinaryUIDToBluetooth(uint8_t *);
void PublishHardwareDetails();
void PublishHexPayloadToBluetooth(uint8_t *, uint8_t *);
void PublishResponseToBluetooth(char *, size_t);
void ResetReader();
void SetupBLE();
void StartBLE();
void ToggleLED(bool);
void WriteNdefMessagePayload(uint8_t *, bool);

void SetTagIdentifier(uint8_t *);
bool CompareTagIdentifier(uint8_t *);
#pragma endregion

//------------------------------------------------------------------------------------------------