#include <Arduino.h>
#include <ArduinoBLE.h>
#include <mbed.h>
#include <rtos.h>
#include <Wire.h>
#include <iostream>
#include <vector>
#include "PN532/Adafruit_PN532.h"
#include "NDEF/NDEF_Message.h"

#pragma once
#include <stdint.h>

//------------------------------------------------------------------------------------------------

using namespace mbed;
using namespace std::chrono;

//------------------------------------------------------------------------------------------------

const char *localNameOfPeripheral = "NFC Reader";
const char *deviceNameOfPeripheral = "NFC Reader";
const char *uuidOfService = "00002A5D-0000-1000-8000-00805F9B34FB"; // environmental sensing
const char *uuidOfTxData = "0000290c-0000-1000-8000-00805f9b34fb";  // measurement data
const char *uuidOfRxData = "0000290b-0000-1000-8000-00805f9b34fb";  // configuration data (JSON)

// set the manufacturer code to 'SKF (U.K.) Limited'
uint8_t manufacturer[2] = {0x0e, 0x04}; 

// Setup the incoming data characteristic (RX).
const int RX_BUFFER_SIZE = 32;
bool RX_BUFFER_FIXED_LENGTH = false;

// Buffer to read samples into, each sample is 16-bits
uint8_t configBuffer[RX_BUFFER_SIZE];

// Setup the outgoinging data characteristic (TX).
const int TX_BUFFER_SIZE = 32;
bool TX_BUFFER_FIXED_LENGTH = false;

// BLE Service
BLEService nearFieldService(uuidOfService);

// RX / TX Characteristics for BYTE ARRAYS
BLECharacteristic rxChar(uuidOfRxData, BLEWriteWithoutResponse | BLEWrite, RX_BUFFER_SIZE, RX_BUFFER_FIXED_LENGTH);
BLECharacteristic txChar(uuidOfTxData, BLERead | BLENotify, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);

//------------------------------------------------------------------------------------------------

#define UID_LENGTH 7            // byte size of NTAG UID
#define BLOCK_SIZE 16           // block size in bytes
#define BLOCK_COUNT 16          // number of blocks to read from the card
#define TOTAL_BLOCKS 86         // total number of NDEF blocks
#define BYTES_PER_BLOCK 4       // number of bytes per block
#define SERIAL_BAUD_RATE 115200 // serial port baud rate
#define WAIT_FOR_CARD_MS 100    // how long to wait for a card before continuing
#define COMMAND_BYTES 6         // how many NDEF records should we ordinarily expect?
#define SYSTEM_TIMEOUT 30000    // reset system to default after this mS
#define COMMAND_TIMEOUT 1000    // reset system to default after this mS
#define NEXT_SCAN_DELAY 1000    // how long to wait before the next scan
#define PLEASE_WAIT 0x2e        // full stop character
#define BLOCK_SIZE_BLE 16       // block size in bytes
#define BLOCK_WAIT_BLE 20000    // wait 20ms between each BLE transmit packet
#define TICK_RATE_MS 200ms      // update rate for the mbed timer

//------------------------------------------------------------------------------------------------

#define NTAG_IC_TYPE 12              // NTAG byte which describes the actual card type
#define NTAG_CAPABILITY_CONTAINER 14 // NTAG byte which details the total number of user bytes available
#define NTAG_DEFAULT_PAGE_CLEAR 16   // how many pages should be cleared by default before a write action
#define NTAG_MAX_RECORD_BYTES 24     //	maximum number of characters per NDEF record

//------------------------------------------------------------------------------------------------

#define INVALID_NDEF "INVALID NDEF RECORD"     // no valid NDEF records could be found
#define OPCODE_BYTES 2                         // how many bytes make up an OPCODE
#define OPERAND_BYTES 4                        // how many bytes make up an OPERAND

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

#define READER_DEBUG
#define READER_DEBUGPRINT Serial

//------------------------------------------------------------------------------------------------

// configure and initialise the NFC reader (we use configurable I/O for this)
Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

//------------------------------------------------------------------------------------------------

uint8_t READER_TIMEOUT[5] = {0x2a, 0x54, 0x2f, 0x0d, 0x0a};
uint8_t CC[4] = {0xe1, 0x10, 0x12, 0x00 };

//------------------------------------------------------------------------------------------------
// RECEIVED COMMAND SET:
// 
// These are commands received from the connected controller. They are of the format:  
//      [ 00 ], [ 01 ] VERB or OPCODE
//      [ 02 ], [ 03 ] .. [ 26 ] NOUN or OPERAND
// 
// List of supported OPCODES:
//
// > continuous read and return of detected card contents
uint8_t CONTINUOUS_READ_CARD[OPCODE_BYTES] = {0x00, 0x00}; 

// > on receipt of this command, read once and return detected card contents
uint8_t SINGLE_READ_CARD[OPCODE_BYTES] = {0x00, 0x01}; 

// > add a single NDEF record to the reader's cache memory
uint8_t ADD_NDEF_RECORD[OPCODE_BYTES] = {0x00, 0x02}; 

// > how many NDEF records are in the device's cache memory
uint8_t COUNT_NDEF_RECORDS[OPCODE_BYTES] = {0x00, 0x03}; 

// > manually clear reader's internal NDEF cache memory
uint8_t CLEAR_NDEF_CACHE[OPCODE_BYTES] = {0x00, 0x04};

// > brute-force publish all cached NDEF records to the detected card
uint8_t PUBLISH_TO_CARD[OPCODE_BYTES] = {0x00, 0x05}; 

// > erase all NDEF records from a detected card
uint8_t ERASE_CARD_CONTENTS[OPCODE_BYTES] = {0x00, 0x06}; 

// > read the current battery voltage
uint8_t GET_ENCODED_SIZE[OPCODE_BYTES] = {0x00, 0x07}; 

// > read the current battery voltage
uint8_t READ_BATTERY_VOLTAGE[OPCODE_BYTES] = {0x00, 0x08}; 

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
uint8_t EOR[4] = {0x00, 0x00, 0x0d, 0x0a};
// > error in attempting to publish to card (out of memory)
uint8_t READ_ERROR_UNKNOWN[4] = {0x01, 0x01, 0x0d, 0x0a};
// > error in attempting to publish to card (card could not be found)
uint8_t WRITE_ERROR_DISCONNECT[4] = {0x02, 0x01, 0x0d, 0x0a};
// > error in attempting to publish to card (out of memory)
uint8_t WRITE_ERROR_OVERRUN[4] = {0x03, 0x01, 0x0d, 0x0a};

//------------------------------------------------------------------------------------------------

uint8_t NDEF_EN_RECORD_EXTRA_PAGE_BYTES = 0x05;
uint8_t NDEF_EN_RECORD_TNF = 0x03;
uint8_t INVALID_UID = 0xff;

//------------------------------------------------------------------------------------------------

/// <summary>
/// Describes each of the commands that this reader supports
/// </summary>
enum PN532_command : uint8_t
{
    ReadCardContinuous,
    ReadCardOnce,
    EraseCachedNdefRecords,
    AddNdefRecordToCashe,
    CountCachedNdefRecords,
    PublishCacheToCard,
    EraseCardContents,
    GetEncodedSize,
    ReadBatteryVoltage
};

/// <summary>
/// Supported NTAG formats
/// </summary>
enum NTAG : uint8_t
{
    UNKNOWN,
    TYPE_213,
    TYPE_215,
    TYPE_216,
};

//------------------------------------------------------------------------------------------------

/// <summary>
/// MBED* control the BLE connected pin
/// </summary>
DigitalOut SetConnectedToBLE(digitalPinToPinName(GPIO_PIN_4));

//------------------------------------------------------------------------------------------------

#pragma region METHOD PROTOTYPES
int GetPageCount(int);
NTAG GetCardType(uint8_t *);
PN532_command GetCommandType(uint8_t *);
uint16_t GetTotalCardMemory(NTAG);
uint8_t Read_PN532(uint8_t *, uint8_t *);
void AddNdefRecordToMessage(byte *, int);
void AtTime(void);
void ConnectToReader(void);
void ExecuteReaderCommands(uint8_t *, uint8_t *);
void FlashLED(int, int);
void GetCachedRecordCount(uint8_t &);
void onBLEConnected(BLEDevice);
void onBLEDisconnected(BLEDevice);
void onRxCharValueUpdate(BLEDevice, BLECharacteristic);
void ProcessControlMessage(byte *, int);
void PublishPayloadToBluetooth(uint8_t *, uint8_t *);
void PublishResponseToBluetooth(uint8_t *);
void ResetReader();
void SetupBLE();
void StartBLE();
void ToggleLED(bool);
void WriteNdefMessagePayload(uint8_t *, bool);
#pragma endregion

//------------------------------------------------------------------------------------------------