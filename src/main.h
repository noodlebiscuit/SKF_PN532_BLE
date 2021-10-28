#include <Arduino.h>
#include <ArduinoBLE.h>
#include <mbed.h>
#include <rtos.h>
#include <Wire.h>
#include <iostream>
#include <vector>
#include <Adafruit_PN532.h>
#include "NDEF/NDEF_Message.h"

#pragma once
#include <stdint.h>

//------------------------------------------------------------------------------------------------

using namespace mbed;
using namespace std::chrono;

//------------------------------------------------------------------------------------------------

const char *nameOfPeripheral = "SKF NFC Reader";
const char *uuidOfService = "0000181a-0000-1000-8000-00805f9b34fb"; // environmental sensing
const char *uuidOfTxData = "0000290c-0000-1000-8000-00805f9b34fb";  // measurement data
const char *uuidOfRxData = "0000290b-0000-1000-8000-00805f9b34fb";  // configuration data (JSON)

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

//------------------------------------------------------------------------------------------------

#define NTAG_213_IC 0x12 // byte code (payload[1]) that identifies and NTAG-213 card
#define NTAG_215_IC 0x3e // byte code (payload[1]) that identifies and NTAG-215 card
#define NTAG_216_IC 0x6d // byte code (payload[1]) that identifies and NTAG-216 card

//------------------------------------------------------------------------------------------------

#define PN532_SCK (13)  // SPI pin SCLK
#define PN532_MISO (12) // SPI pin MISO
#define PN532_MOSI (11) // SPI pin MOSI
#define PN532_SS (10)   // SPI pin SS
#define GPIO_PIN_4 4    // BLE connected LED pin
#define COMMS_LED 7     // NFC activity LED

//------------------------------------------------------------------------------------------------

#define IS_DEGUG true // returns serial debug data

//------------------------------------------------------------------------------------------------

// configure and initialise the NFC reader (we use configurable I/O for this)
Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

//------------------------------------------------------------------------------------------------

byte READER_TIMEOUT[5] = {0x2a, 0x54, 0x2f, 0x0d, 0x0a};

//------------------------------------------------------------------------------------------------

uint8_t EOR[4] = {0x00, 0x00, 0x0d, 0x0a}; // End of record payload
uint8_t CONTINUOUS_READ[2] = {0x00, 0x00}; // continuous read of detected TAG
uint8_t SINGLE_READ[2] = {0x00, 0x01};     // read detected TAG once on each receipt of this command
uint8_t ADD_RECORD[2] = {0x00, 0x02};      // add a single NDEF message to the the local cache
uint8_t ERASE_TAG[2] = {0x00, 0x03};       // clear all TAG contents
uint8_t UPDATE_TAG[2] = {0x00, 0x04};      // publish all cached NDEF messages to the TAG

//------------------------------------------------------------------------------------------------

uint8_t NDEF_EN_RECORD_EXTRA_PAGE_BYTES = 0x05;
uint8_t NDEF_EN_RECORD_TNF = 0x03;
uint8_t INVALID_UID = 0xff;

//------------------------------------------------------------------------------------------------

/// <summary>
/// MBED* control the BLE connected pin
/// </summary>
DigitalOut SetConnectedToBLE(digitalPinToPinName(GPIO_PIN_4));

//------------------------------------------------------------------------------------------------

/// <summary>
/// Describes each of the commands that this reader supports
/// </summary>
enum PN532_command : uint8_t
{
    ReadContinuous,
    ReadOnce,
    ClearTag,
    AddNdefRecord,
    WriteNdefMessage,
};

//------------------------------------------------------------------------------------------------

#pragma region METHOD PROTOTYPES
void startBLE();
void setupBLE();
void onBLEDisconnected(BLEDevice);
void onBLEConnected(BLEDevice);
void onRxCharValueUpdate(BLEDevice, BLECharacteristic);
void processControlMessage(byte *message, int messageSize);
void PublishPayloadToBluetooth(uint8_t *, uint8_t *);

/// <summary>
/// Clears and overwrites the complete contents of the NDEF message block
/// </summary>
/// <param name="headerdata">reference to the read NDEF message header</param>
/// <param name="clearCard">clear card before write action</param>
void WriteNdefMessagePayload(uint8_t *headerdata, bool clearCard);

/// <summary>
/// How many pages are required to cover all message bytes?
/// </summary>
/// <param name="byteCount">number of message bytes</param>
int getPageCount(int byteCount);

/// <summary>
/// Appends a received NDEF record to an existing NDEF message
/// </summary>
/// <param name="message">pointer to the received command message byte array</param>
/// <param name="messageSize">number of bytes in the command message</param>
void AddNdefRecordToMessage(byte *message, int messageSize);

/// <summary>
/// INTERUPT SERVICE ROUTINE
/// </summary>
void AtTime(void);

/// <summary>
/// Writes the NDEF contents of a card to the serial port
/// </summary>
/// <param name="message">reference to the read NDEF message</param>
void ConnectToReader(void);

/// <summary>
/// Clear TAG contents or write complete NDEF message
/// </summary>
/// <param name="headerdata">reference to the read NDEF message header</param>
/// <param name="pagedata">reference to the read NDEF message body</param>
void ExecuteReaderCommands(uint8_t *headerdata, uint8_t *pagedata);

/// <summary>
/// Writes the NDEF contents of a card to the serial port
/// </summary>
/// <param name="pagedata">returns the NDEF message payload</param>
/// <param name="headerdata">returns the NDEF meassage header</param>
uint8_t Read_PN532(uint8_t *, uint8_t *);

/// <summary>
/// How many pages are required to cover all message bytes?
/// </summary>
/// <param name="byteCount">number of message bytes</param>
int GetPageCount(int);

/// <summary>
/// Get the received command type
/// </summary>
/// <param name="buffer">byte array to search against</param>
PN532_command GetCommandType(uint8_t *buffer);

/// <summary>
/// Toggle the LED ON or OFF every time this method is called
/// </summary>
/// <param name="period">true for toggle else false for LED OFF</param>
void ToggleLED(bool);

/// <summary>
/// Flashes the COMMS LED
/// </summary>
/// <param name="period">milliseconds to illuminate for</param>
void FlashLED(int, int);

/// <summary>
/// INTERUPT SERVICE ROUTINE
/// </summary>
void AtTime(void);

/// <summary>
/// Reset the reader after RTOS timeout
/// </summary>
void ResetReader();
#pragma endregion

//------------------------------------------------------------------------------------------------