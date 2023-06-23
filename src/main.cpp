/**************************************************************************************************
 * Author: Alex Pinkerton
 *
 * License: (c) 2021, MIT LICENSE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions: The above copyright notice and this
 * permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Description: main.cpp
 * Main NFC reader for the Nordic NRF-52840 Microcontroller
 *
 ***************************************************************************************************/

#include "main.h"

//------------------------------------------------------------------------------------------------

#pragma region PRIVATE MEMBERS
// MBED RTOS timer
Ticker timer;
volatile bool timerEvent = false;

// current time (for TIMEOUT management)
unsigned long currentTime = 0;

// what is the reader required to do?
PN532_command _command = ReadCardContinuous;

// block access to the reader hardware?
volatile bool _blockReader = false;

// whwn set true, we need to block all other I/O activites
volatile bool _readerBusy = false;

// references the UID from the TAG to block multiple reads
uint8_t _headerdata[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// create a new NDEF message
NDEF_Message *ndef_message = new NDEF_Message();

// enable TIMEOUTS (for WRITE or ONE SHOT)?
volatile bool _enableTimeouts = false;

// when incremented to a specified value, publish the current battery level
uint8_t _batteryCount = BATTERY_UPDATE_COUNTER;
#pragma endregion

//------------------------------------------------------------------------------------------------

#pragma region BLUETOOTH LOW ENERGY SUPPORT
/// <summary>
/// Configure the BLE hardware
/// </summary>`
void SetupBLE()
{
   // initiate BLE comms
   StartBLE();

   // Create BLE service and characteristics.
   BLE.setDeviceName(PERIPERHAL_DEVICE_NAME);
   BLE.setLocalName(LOCAL_NAME_OF_PERIPHERAL);
   BLE.setManufacturerData(SKF_MANUFACTURER_CODE, 2);

   // configure the main NFC communications service
   AddDataServiceBLE();

   // configure the battery level service
   AddBatteryServiceBLE();

   // configure the device information service
   AddDeviceServiceBLE();

   // Bluetooth LE connection handlers.
   BLE.setEventHandler(BLEConnected, onBLEConnected);
   BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);

   // Event driven reads.
   rxChar.setEventHandler(BLEWritten, onRxCharValueUpdate);

   // Let's tell all local devices about us.
   BLE.advertise();

   // set the default characteristics
   manufacturerCharacteristic.writeValue(MANUFACTURER_NAME_STRING, false);
   modelNumberCharacteristic.writeValue(MODEL_NAME_STRING, false);
   hardwareCharacteristic.writeValue(HARDWARE_NAME_STRING, false);
   firmwareRevisionCharacteristic.writeValue(FIRMWARE_NAME_STRING, false);
   serialNumberCharacteristic.writeValue(SERIAL_NO_NAME_STRING, false);

   // register the current battery voltage
   PublishBattery();
}

/// <summary>
/// Add the device information service and associate the required characteristics
/// </summary>
void AddDeviceServiceBLE()
{
   BLE.setAdvertisedService(deviceInfoService);
   deviceInfoService.addCharacteristic(manufacturerCharacteristic);
   deviceInfoService.addCharacteristic(modelNumberCharacteristic);
   deviceInfoService.addCharacteristic(firmwareRevisionCharacteristic);
   deviceInfoService.addCharacteristic(serialNumberCharacteristic);
   deviceInfoService.addCharacteristic(hardwareCharacteristic);
   BLE.addService(deviceInfoService);
}

/// <summary>
/// Add the battery state service and associate the required characteristics
/// </summary>
void AddBatteryServiceBLE()
{
   BLE.setAdvertisedService(batteryService);
   batteryService.addCharacteristic(batteryCharacteristic);
   BLE.addService(batteryService);
}

/// <summary>
/// Add the core data service and associate the required characteristics
/// </summary>
void AddDataServiceBLE()
{
   BLE.setAdvertisedService(nearFieldService);
   nearFieldService.addCharacteristic(rxChar);
   nearFieldService.addCharacteristic(txChar);
   BLE.addService(nearFieldService);
}

/// <summary>
/// Start the BLE service!
/// </summary>
void StartBLE()
{
   if (!BLE.begin())
   {
#ifdef READER_DEBUG
      READER_DEBUGPRINT.println("starting BLE failed!");
#endif
      while (1)
         ;
   }
}

/// <summary>
/// A BLE device has connected to our sensor - illuminate the connection LED
/// </summary>
/// <param name="central">BLE device</param>
void onBLEConnected(BLEDevice central)
{
   LED_SetConnectedToBLE = HIGH;
   ResetReader();
}

/// <summary>
/// A BLE device has just disconnected from our sensor - power off the connection LED
/// </summary>
/// <param name="central">BLE device</param>
void onBLEDisconnected(BLEDevice central)
{
   LED_SetConnectedToBLE = LOW;
}

/// <summary>
/// Process received data serial data
/// </summary>
/// <param name="central">BLE device</param>
/// <param name="characteristic">BLE characteristic referenced</param>
void onRxCharValueUpdate(BLEDevice central, BLECharacteristic characteristic)
{
   // read and cache the received BLE message
   byte tmp[RX_BUFFER_SIZE];
   int dataLength = rxChar.readValue(tmp, RX_BUFFER_SIZE);

   // process the received BLE message
   ProcessControlMessage(tmp, dataLength);
}

/// <summary>
/// Process any received ProtoBuf message payload
/// </summary>
/// <param name="message">pointer to the received PB message byte array</param>
/// <param name="messageSize">number of bytes in the PB message</param>
void ProcessControlMessage(byte *message, int messageSize)
{

   READER_DEBUGPRINT.print(messageSize);
   READER_DEBUGPRINT.print("> ");
   for (int i = 0; i < messageSize; ++i)
   {
      READER_DEBUGPRINT.print(message[i], HEX);
      READER_DEBUGPRINT.print(" ");
   }
   READER_DEBUGPRINT.println("");

   // initialise responses here (cannot be done within the switch() statement)
   uint8_t cachedRecordCount = 0x00;
   uint8_t encodedSizeLow = 0x00;
   uint8_t encodedSizeHigh = 0x00;
   uint8_t *responsePayload = new uint8_t[OPERAND_BYTES];
   uint16_t encodedSize = 0x0000;
   uint16_t batteryVoltage = 0x0000;

   // process any received payload
   _command = GetCommandType(message);

   // OK, and what's expected of us here?
   switch (_command)
   {
   // *************************************************************************
   // Read detected card continuously
   // *************************************************************************
   case ReadCardContinuous:
      _blockReader = false;
#ifdef READER_DEBUG
      READER_DEBUGPRINT.println("Read continuous (executed within main reader loop)");
#endif
      break;

   // *************************************************************************
   // Read detected card just once (re-issue this command to repeat)
   // *************************************************************************
   case ReadCardOnce:
      _blockReader = false;
#ifdef READER_DEBUG
      READER_DEBUGPRINT.println("Read once (executed within main reader loop)");
#endif
      break;

   // *************************************************************************
   // Add a new record to the NDEF message cache and confirm with write-back
   // *************************************************************************
   case AddNdefRecordToCache:
      _command = ReadCardContinuous;
      _blockReader = false;
      AddNdefRecordToMessage(message, messageSize);
      GetCachedRecordCount(cachedRecordCount);
      responsePayload[0] = 0x00;
      responsePayload[1] = cachedRecordCount;
      responsePayload[2] = 0x0d;
      responsePayload[3] = 0x0a;
      delayMicroseconds(BLOCK_WAIT_BLE);
      PublishResponseToBluetooth(responsePayload);
#ifdef READER_DEBUG
      READER_DEBUGPRINT.println("Add single record to cache");
#endif
      break;

   // *************************************************************************
   // Add a new record to the NDEF message cache and confirm with write-back
   // *************************************************************************
   case AppendToCachedNdefRecord:
      _command = ReadCardContinuous;
      _blockReader = false;
      if (AppendToNdefRecordMessage(message, messageSize))
      {
         GetCachedRecordCount(cachedRecordCount);
         responsePayload[0] = 0x00;
         responsePayload[1] = cachedRecordCount;
         responsePayload[2] = 0x0d;
         responsePayload[3] = 0x0a;
         delayMicroseconds(BLOCK_WAIT_BLE);
         PublishResponseToBluetooth(responsePayload);
#ifdef READER_DEBUG
         READER_DEBUGPRINT.println("Append new data to current record in cache");
#endif
      }
      else
      {
         delayMicroseconds(BLOCK_WAIT_BLE);
         PublishResponseToBluetooth(WRITE_ERROR_OVERRUN);
#ifdef READER_DEBUG
         READER_DEBUGPRINT.println("ERROR: CACHE OVERRUN");
#endif
      }
      break;

   // *************************************************************************
   // Erase all existing cache contents and confirm with write-back
   // *************************************************************************
   case EraseCachedNdefRecords:
      if (ndef_message->getRecordCount() > 0)
      {
         ndef_message->dropAllRecords();
      }
      GetCachedRecordCount(cachedRecordCount);
      responsePayload[0] = 0x00;
      responsePayload[1] = cachedRecordCount;
      responsePayload[2] = 0x0d;
      responsePayload[3] = 0x0a;
      delayMicroseconds(BLOCK_WAIT_BLE);
      PublishResponseToBluetooth(responsePayload);
      _command = ReadCardContinuous;

#ifdef READER_DEBUG
      READER_DEBUGPRINT.println("Erase all cached records");
#endif
      break;

   // *************************************************************************
   // count and return the number of NDEF records currently in the cache
   // *************************************************************************
   case CountCachedNdefRecords:
      GetCachedRecordCount(cachedRecordCount);
      responsePayload[0] = 0x00;
      responsePayload[1] = cachedRecordCount;
      responsePayload[2] = 0x0d;
      responsePayload[3] = 0x0a;
      delayMicroseconds(BLOCK_WAIT_BLE);
      PublishResponseToBluetooth(responsePayload);
      _command = ReadCardContinuous;
#ifdef READER_DEBUG
      READER_DEBUGPRINT.println("Return number of cached NDEF records");
#endif
      break;

   // *************************************************************************
   // read and return the current battery voltage
   // *************************************************************************
   case ReadBatteryVoltage:
      batteryVoltage = ReadBattery(A0, READ_BATTERY_AVG);
      Serial.println(batteryVoltage);
      responsePayload[0] = (uint8_t)(batteryVoltage >> 8);
      responsePayload[1] = (uint8_t)(batteryVoltage & 0x00ff);
      responsePayload[2] = 0x0d;
      responsePayload[3] = 0x0a;
      delayMicroseconds(BLOCK_WAIT_BLE);
      PublishResponseToBluetooth(responsePayload);
      _command = ReadCardContinuous;
#ifdef READER_DEBUG
      READER_DEBUGPRINT.println("Read the current battery voltage");
#endif
      break;

   // *************************************************************************
   // count and return the encoded message size in bytes
   // *************************************************************************
   case GetEncodedSize:
      if (ndef_message->getRecordCount() > 0)
      {
         encodedSize = (uint16_t)ndef_message->getEncodedSize();
         if (encodedSize <= NTAG_216_MEMORY)
         {
            encodedSizeLow = (uint8_t)(encodedSize & 0x00ff);
            encodedSizeHigh = (uint8_t)(encodedSize >> 8);
         }
      }
      responsePayload[0] = encodedSizeHigh;
      responsePayload[1] = encodedSizeLow;
      responsePayload[2] = 0x0d;
      responsePayload[3] = 0x0a;
      delayMicroseconds(BLOCK_WAIT_BLE);
      PublishResponseToBluetooth(responsePayload);
      _command = ReadCardContinuous;
#ifdef READER_DEBUG
      READER_DEBUGPRINT.println("Get the encoded cache size");
#endif
      break;

   // *************************************************************************
   // erase all contents of the current card
   // *************************************************************************
   case EraseCardContents:
#ifdef READER_DEBUG
      READER_DEBUGPRINT.println("Erase card contents (executed within main reader loop)");
#endif
      break;

   // *************************************************************************
   // publish the NDEF record cache to the next detected card
   // *************************************************************************
   case PublishCacheToCard:
#ifdef READER_DEBUG
      READER_DEBUGPRINT.println("Publish cache to card (executed within main reader loop)");
#endif
      break;

   // *************************************************************************
   // debug only (unknown command)
   // *************************************************************************
   default:
#ifdef READER_DEBUG
      READER_DEBUGPRINT.println("Unknown");
#endif
      break;
   }
   delete[] responsePayload;

   // single instance LED flash
   FlashLED(COMMAND_LED_FLASH, 0);
}

/// <summary>
/// Reads and returns an averaged value for the 3.7V Lithium ion battery
/// </summary>
/// <param name="PIN">what analogue pin are we connecting to?</param>
/// <param name="average">how many samples to read and average. Must not be greater than 10!</param>
/// <returns>read battery voltage between 0 and 2047</returns>
uint16_t ReadBattery(pin_size_t PIN, int average)
{
   int16_t value = 0x0000;
   for (int i = 0; i < average; ++i)
   {
      value += analogRead(PIN);
      delayMicroseconds(BATTERY_READ_DELAY);
   }
   return (uint16_t)(value / average);
}

/// <summary>
/// Streams the NDEF contents out over Bluetooth as a series of 16 byte packets
/// </summary>
/// <param name="pagedata">raw NDEF message payload</param>
/// <param name="headerdata">NDEF meassage header with UUID</param>
void PublishResponseToBluetooth(uint8_t *responsePayload)
{
   // make sure we don't have any NFC scanning overlaps here
   _readerBusy = true;

   // send the payload terminator
   delayMicroseconds(BLOCK_WAIT_BLE);
   txChar.writeValue(responsePayload, 4);

   // release the blocker
   _readerBusy = false;
}

/// <summary>
/// Streams the NDEF contents out over Bluetooth as a series of 16 byte packets
/// </summary>
/// <param name="pagedata">raw NDEF message payload</param>
/// <param name="headerdata">NDEF meassage header with UUID</param>
void PublishPayloadToBluetooth(uint8_t *pagedata, uint8_t *headerdata)
{
   // reset the CRC
   CRC_out = 0xe4;

   // make sure we don't have any NFC scanning overlaps here
   _readerBusy = true;

   // generate the CRC for the header block
   for (uint8_t i = 0; i < BLOCK_SIZE_BLE; ++i)
   {
      calculateCRC(OUT, headerdata[i]);
   }

   // write the header block
   txChar.writeValue(headerdata, BLOCK_SIZE_BLE);

   // what is the total message size in bytes?
   int message_length = pagedata[1] + 3;

   // reset the page index
   int index = 0;

   // write out each block of the received payload
   while (message_length >= 0)
   {
      delayMicroseconds(BLOCK_WAIT_BLE);
      if (message_length >= BLOCK_SIZE_BLE)
      {
         txChar.writeValue(pagedata + (index * BLOCK_SIZE_BLE), BLOCK_SIZE_BLE);
         index++;
      }
      else
      {
         txChar.writeValue(pagedata + (index * BLOCK_SIZE_BLE), message_length + 1);
      }
      message_length -= BLOCK_SIZE_BLE;
   }

   // append the CRC based on the transmitted payload
   message_length = pagedata[1] + 3;
   for (int i = 0; i < message_length; ++i)
   {
      calculateCRC(OUT, pagedata[i]);
   }

   //
   // send the EOR packet with the CHECKSUM at position - 0x00, CRC, 0x0D, 0x0A
   //
   delayMicroseconds(BLOCK_WAIT_BLE);
   EOR[1] = CRC_out;
   txChar.writeValue(EOR, 4);

   // release the blocker
   _readerBusy = false;
}
#pragma endregion

//------------------------------------------------------------------------------------------------

#pragma region APPLICATION CORE AND OS TIMER
/// <summary>
/// Setup the ARDUINO
/// </summary>
void setup(void)
{
   // set the LED pin
   pinMode(COMMS_LED, OUTPUT);

   // three flashes to confirm the reader is active
   for (int i = 0; i < 3; ++i)
   {
      FlashLED(100, 150);
   }

   // initialise the serial port
   Serial.begin(SERIAL_BAUD_RATE);

   // set the timeout value
   timer.attach(&AtTime, TICK_RATE_MS);

   // initiate connection to the PN532 board
   nfc.begin();

   // configure board to read RFID tags
   nfc.SAMConfig();

   // get the NFC firmware
   uint32_t adaFruit_NFC_firmare_version = nfc.getFirmwareVersion();
   Serial.print(adaFruit_NFC_firmare_version);

   // now we setup all services within the BLE layer
   SetupBLE();
}

/// <summary>
/// APPLICATION SUPER LOOP
/// </summary>
void loop(void)
{
   // top priority here is the BLE controller
   BLEDevice central = BLE.central();
   if (central)
   {
      // Only send data if we are connected to a central device.
      while (central.connected())
      {
         // we still need to process SPI ticks
         if (timerEvent)
         {
            timerEvent = false;
            ConnectToReader();
            PublishBattery();
         }
      }
   }
}

/// <summary>
/// MBED timer tick event *** APPLICATION CORE ***
/// </summary>
void AtTime()
{
   timerEvent = true;
}
#pragma endregion

//------------------------------------------------------------------------------------------------

#pragma region NEAR FIELD COMMUNICATIONS SUPPORT METHODS
/// <summary>
/// Publish the current battery voltage, if and when it's possible to do so
/// </summary>
void PublishBattery()
{
   // if the reader is blocked, then bypass this method completely
   if (!(_blockReader | _readerBusy))
   {
      // increment the battery read count
      if (++_batteryCount > BATTERY_UPDATE_COUNTER)
      {
         _batteryCount = 0x00;
         uint8_t *responsePayload = new uint8_t[2];
         uint16_t batteryVoltage = ReadBattery(A0, READ_BATTERY_AVG);
#ifdef READER_DEBUG
         READER_DEBUGPRINT.print("Battery: ");
         READER_DEBUGPRINT.println(batteryVoltage);
#endif
         responsePayload[0] = (uint8_t)(batteryVoltage >> 8);
         responsePayload[1] = (uint8_t)(batteryVoltage & 0x00ff);
         batteryCharacteristic.writeValue(responsePayload, 2);
         delete[] responsePayload;
      }
   }
}

/// <summary>
/// Main entry point for PN532 scanning
/// </summary>
void ConnectToReader(void)
{
   // if the reader is blocked, then bypass this method completely
   if (!(_blockReader | _readerBusy))
   {
      uint8_t *pagedata = new uint8_t[TOTAL_BLOCKS * BYTES_PER_BLOCK];
      uint8_t *headerdata = new uint8_t[BLOCK_SIZE_BLE];

      // read the card
      uint8_t uidLength = Read_PN532(pagedata, headerdata);

#ifdef READER_DEBUG
      if (uidLength > 0)
      {
         READER_DEBUGPRINT.print("UID Length: ");
         READER_DEBUGPRINT.println(uidLength);
      }
#endif

      // if the UID is valid, then the data should be OK
      if (uidLength == UID_LENGTH)
      {
         // make a temporary copy of the received UID
         uint8_t *uidRecord = new uint8_t[UID_LENGTH];
         uidRecord[0] = headerdata[0];
         uidRecord[1] = headerdata[1];
         uidRecord[2] = headerdata[2];
         uidRecord[3] = headerdata[4];
         uidRecord[4] = headerdata[5];
         uidRecord[5] = headerdata[6];
         uidRecord[6] = headerdata[7];

         //
         // if this is the same UID then we don't process this - otherwise we end
         // up in a never ending loop of reading the TAG and sending the data back
         //
         if (memcmp(_headerdata, uidRecord, UID_LENGTH) != 0)
         {
            for (uint8_t i = 0; i < UID_LENGTH; ++i)
            {
               _headerdata[i] = uidRecord[i];
            }

            // does this message contain a valid NDEF record?
            if (pagedata[0] == NDEF_EN_RECORD_TNF)
            {
               //
               // we only publish the NDEF data if the current command is either
               // a read continuous or a read just once
               //
               if ((_command == ReadCardContinuous) | (_command == ReadCardOnce))
               {
                  // if we have at least one NDEF record then write this to USB
                  PublishPayloadToBluetooth(pagedata, headerdata);
               }
            }

            // clear TAG contents or write complete NDEF message
            ExecuteReaderCommands(headerdata, pagedata);

            // reset any command that might have been received
            _command = ReadCardContinuous;
         }
         // release this object and leave method right here!
         delete[] uidRecord;
         delete[] pagedata;
         delete[] headerdata;
         return;
      }
      else if (uidLength == INVALID_UID)
      {
#ifdef READER_DEBUG
         READER_DEBUGPRINT.println(INVALID_NDEF);
#endif
         // clear TAG contents or write complete NDEF message
         ExecuteReaderCommands(headerdata, pagedata);

         // post two 0xff bytes to signify end of write sequence
         PublishWriteFeedback(CARD_ERROR_EMPTY[0], CARD_ERROR_EMPTY[1]);
      }

      delete[] pagedata;
      delete[] headerdata;

      // if we've reached this point then we need to reset the received UID
      for (uint8_t i = 0; i < UID_LENGTH; ++i)
      {
         _headerdata[i] = 0x00;
      }
   }
}

/// <summary>
/// Reads the card contents and returns over Bluetooth
/// </summary>
/// <param name="pagedata">returns the NDEF message payload</param>
/// <param name="headerdata">returns the NDEF meassage header</param>
uint8_t Read_PN532(uint8_t *pagedata, uint8_t *headerdata)
{
   uint8_t uidLength = 0;
   uint8_t success;

   // buffer for a single blocj
   uint8_t data[BLOCK_SIZE];

   // Buffer to store the returned UID
   uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};

   //
   // Wait for an NTAG2XX card.  When one is found 'uid' will be populated with
   // the UID, and uidLength will indicate the size of the UUID (normally 7 bytes)
   //
   success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, WAIT_FOR_CARD_MS);

   // did we get a valid UID from this card?
   if (success & (uidLength == UID_LENGTH))
   {
      // illuminate the status LED
      ToggleLED(true);

      // read the header block
      if (nfc.mifareclassic_ReadDataBlock(0, data))
      {
         memcpy(headerdata, data, BLOCK_SIZE);
      }

      // reset the block index
      uint8_t block = 4;

      // set the next block
      for (uint8_t i = 0; i < BLOCK_COUNT; i++)
      {
         //
         // try and read the next block. If successful then append
         // the received block to the complete page array
         //
         if (nfc.mifareclassic_ReadDataBlock(block, data))
         {
            // if reader contents are corrupted, then abort here
            if (i == 0 && ((data[1] == 0) | (data[2] == 0) | (data[3] == 0)))
            {
               uidLength = INVALID_UID;
               break;
            }

            // build the return payload
            memcpy(pagedata + (i * BLOCK_SIZE), data, BLOCK_SIZE);
         }

         // increment the block index
         block += BYTES_PER_BLOCK;
      }
   }

   // disable the status LED
   ToggleLED(false);

   // return the number UID bytes
   return uidLength;
}

/// <summary>
/// Reads the cards CAPACITY CONTAINER (from Page #3) and returns the TAG type
/// This page spans bytes 12, 13, 14 and 15, where 14 is the card type
/// </summary>
/// <param name="headerdata">reference to the read NDEF message header</param>
NTAG GetCardType(uint8_t *headerdata)
{
   if ((headerdata[12] == CC[0]) & (headerdata[13] == CC[1]) & (headerdata[15] == CC[3]))
   {
      if (headerdata[14] == NTAG_213_IC)
      {
         return TYPE_213;
      }
      else if (headerdata[14] == NTAG_215_IC)
      {
         return TYPE_215;
      }
      else if (headerdata[14] == NTAG_216_IC)
      {
         return TYPE_216;
      }
   }
   return UNKNOWN;
}

/// <summary>
/// What is the maximum available memory for a specific card type
/// </summary>
/// <param name="headerdata">embedded card type (NTAG213, NTAG215, NTAG216)</param>
uint16_t GetTotalCardMemory(NTAG cardType)
{
   if (cardType == TYPE_213)
   {
      return NTAG_213_MEMORY;
   }
   else if (cardType == TYPE_215)
   {
      return NTAG_215_MEMORY;
   }
   else if (cardType == TYPE_216)
   {
      return NTAG_216_MEMORY;
   }
   return NTAG_213_MEMORY;
}

/// <summary>
/// Clear TAG contents or write complete NDEF message
/// </summary>
/// <param name="headerdata">reference to the read NDEF message header</param>
/// <param name="pagedata">reference to the read NDEF message body</param>
void ExecuteReaderCommands(uint8_t *headerdata, uint8_t *pagedata)
{
   // is this an NTAG compatible card?
   bool isNTAG = false;

   // while we're here, get some details on this card
   NTAG cardType = GetCardType(headerdata);

#ifdef READER_DEBUG
   // for debug, let's see what the card type actually is
   switch (cardType)
   {
   case UNKNOWN:
      READER_DEBUGPRINT.println("CARD NOT RECOGNISED");
      break;
   case TYPE_213:
      READER_DEBUGPRINT.println("CARD IS NTAG-213");
      break;
   case TYPE_215:
      READER_DEBUGPRINT.println("CARD IS NTAG-215");
      break;
   case TYPE_216:
      READER_DEBUGPRINT.println("CARD IS NTAG-216");
      break;
   }
#endif

   // process the two supported commands (CLEAR and WRITE NDEF MESSAGE)
   if (_command == EraseCardContents)
   {
      // at this point we don't want the reader to keep checking for Tags
      _blockReader = true;

      // clear all card contents
      ClearTheCard(headerdata, pagedata);

      // lastly we revert to the default command state (read continuous) and unblock the reader
      _command = ReadCardContinuous;
      _blockReader = false;
   }
   else if (_command == PublishCacheToCard)
   {
      // at this point we don't want the reader to keep checking for Tags
      _blockReader = true;

      //
      // now we publish the cached payload and drop all existing records from the NDEF cache
      // Firstly we need to see how many bytes will be needed to write the cached payload
      //
      uint16_t pendingBytes = ndef_message->getEncodedSize() + NDEF_EN_RECORD_EXTRA_PAGE_BYTES;

      // now we need to see how many bytes are ACTUALLY available in the NTAG card
      uint16_t availableBytes = 0x0000;
      if (cardType == TYPE_213)
      {
         availableBytes = NTAG_213_MEMORY;
      }
      else if (cardType == TYPE_215)
      {
         availableBytes = NTAG_215_MEMORY;
      }
      else if (cardType == TYPE_216)
      {
         availableBytes = NTAG_216_MEMORY;
      }

      //
      // provided the available memory on the card exceeds the pending cache memory then we're
      // OK to write all pending data to the NTAG card
      //
      if (availableBytes > pendingBytes)
      {
         WriteNdefMessagePayload(headerdata, isNTAG);
      }
      else
      {
         PublishResponseToBluetooth(WRITE_ERROR_OVERRUN);
      }

      // irrespective of whether we managed to update the card, we always empty the reader cache
      ndef_message->dropAllRecords();

      // lastly we revert to the default command state (read continuous) and unblock the reader
      _command = ReadCardContinuous;
      _blockReader = false;
   }
   else if (_command == ReadCardContinuous)
   {
      // for continuous (default) operation, we have to re-enable the reader
      _blockReader = false;
      return;
   }
   else if (_command == ReadCardOnce)
   {
      // for a read once, we block any further reads right here!
      _blockReader = true;
   }

   // force a timeout reset after ONE SECOND
   currentTime = millis() - (SYSTEM_TIMEOUT - COMMAND_TIMEOUT);
}

/// <summary>
/// Appends a received NDEF BINARY record to an existing NDEF message
/// Note: this differs from an NDEF TEXT record in that it allows invalid
/// characters - specifically /ESC and 0x00
/// </summary>
/// <param name="message">pointer to the received command message byte array</param>
/// <param name="messageSize">number of bytes in the command message</param>
void AddNdefRecordToMessage(byte *message, int messageSize)
{
   //
   // ensure the message size does not exceed the set limit plus the
   // number of bytes we've already allocated for the reader commands
   //
   if (messageSize > NTAG_SINGLE_WRITE_BYTES + 2)
   {
      messageSize = NTAG_SINGLE_WRITE_BYTES + 2;
   }

   // create a new ndef record string buffer
   byte *ndefRecord = new byte[NTAG_SINGLE_WRITE_BYTES + 1];

   // clear the serial read buffer contents
   memset(ndefRecord, 0, NTAG_SINGLE_WRITE_BYTES + 1);

   //
   // strip away the first two command characters but start the payload
   // index at THREE. This is to allow us to write both the three-bytes
   // header as well as the message to be handled
   //
   uint8_t index = 0x03;
   for (int i = 2; i < messageSize; ++i)
   {
      ndefRecord[index] = message[i];
      ++index;
   }

   //
   // add the number of bytes used to describe the format
   // For this reader, we'll stick to 'en'
   //
   ndefRecord[0] = 0x02;

   // add the encloding type 'en'
   ndefRecord[1] = 0x65;
   ndefRecord[2] = 0x6e;

   // add an NDEF binary record to the NDEF message
   ndef_message->addBinaryRecord(ndefRecord, messageSize + 1);

#ifdef READER_DEBUG_APPEND_FUNCTIONALITY
   DebugPrintCache();
#endif

   delete[] ndefRecord;
}

/// <summary>
/// Appends a received NDEF TEXT record to an existing NDEF message
/// Note: this differs from an NDEF BINARY record in that it blocks invalid
/// characters such as /ESC and 0x00
/// </summary>
/// <param name="message">pointer to the received command message byte array</param>
/// <param name="messageSize">number of bytes in the command message</param>
void AddNdefTextRecordToMessage(byte *message, int messageSize)
{
   // ensure the message size does not exceed the set limit
   if (messageSize > NTAG_SINGLE_WRITE_BYTES + 2)
   {
      messageSize = NTAG_SINGLE_WRITE_BYTES + 2;
   }

   // create a new ndef record string buffer
   char *ndefRecord = new char[NTAG_SINGLE_WRITE_BYTES];

   // clear the serial read buffer contents
   memset(ndefRecord, 0, NTAG_SINGLE_WRITE_BYTES);

   // strip away the first two command characters
   uint8_t index = 0;
   for (int i = 2; i < messageSize; ++i)
   {
      ndefRecord[index] = message[i];
      ++index;
   }

   // add an NDEF text record to the NDEF message
   ndef_message->addTextRecord(ndefRecord);

#ifdef READER_DEBUG_APPEND_FUNCTIONALITY
   DebugPrintCache();
#endif

   delete[] ndefRecord;
}

/// <summary>
/// Appends a received data to the current NDEF message
/// </summary>
/// <param name="message">pointer to the received command message byte array</param>
/// <param name="messageSize">number of bytes in the command message</param>
bool AppendToNdefRecordMessage(byte *message, int messageSize)
{
   // get the current record (last active) record from the cache
   NDEF_Record record = ndef_message->getRecord(ndef_message->getRecordCount() - 1);

   // get the index of the last byte in the currently cached record
   int index = record.getPayloadLength();

   if ((index + messageSize) < NTAG_MAX_RECORD_BYTES)
   {
      // create a full size buffer that we can load the current NDEF record into
      byte *ndefRecord = new byte[NTAG_MAX_RECORD_BYTES];

      // reset the contents of this buffer
      memset(ndefRecord, 0, NTAG_MAX_RECORD_BYTES);

      // now get the current NDEF record from the cache
      record.getPayload(ndefRecord);

      // append the new message to the end of the old one
      for (int i = 2; i < messageSize; ++i)
      {
         ndefRecord[index] = message[i];
         index++;
      }

      // overwrite the current NDEF record with the newly updated one
      record.setPayload(ndefRecord, index);

      // now update the parent NDEF message
      ndef_message->setRecord(ndef_message->getRecordCount() - 1, record);

      // release buffer resouces
      delete[] ndefRecord;
   }
   else
   {
      //
      // if we're here, then we've exceeded the number of bytes that can
      // be appended to a single NDEF record, so need to return FALSE
      //
      return false;
   }

#ifdef READER_DEBUG_APPEND_FUNCTIONALITY
   DebugPrintCache();
#endif

   // well, if we reached this point, then all we can do is hope for the best!
   return true;
}

/// <summary>
/// Clears and overwrites the complete contents of the NDEF message block
/// </summary>
/// <param name="headerdata">reference to the read NDEF message header</param>
/// <param name="clearCard">clear card before write action</param>
void WriteNdefMessagePayload(uint8_t *headerdata, bool clearCard)
{
   // create the page buffer
   uint8_t pageBuffer[BYTES_PER_BLOCK] = {0, 0, 0, 0};

   // default page clear count
   int pages = NTAG_DEFAULT_PAGE_CLEAR;

   //
   // if this is an NTAG card then we need to clear some of the
   // initial pages before we write back to them.
   //
   if (clearCard)
   {
      // clear either the default number (16) or all pages on the card
      for (uint8_t i = 4; i < pages + 4; i++)
      {
         memset(pageBuffer, 0, 4);
         nfc.ntag2xx_WritePage(i, pageBuffer);
         ToggleLED(true);
      }
   }

   // how many bytes are now in this new message
   uint8_t totalBytes = ndef_message->getEncodedSize() + NDEF_EN_RECORD_EXTRA_PAGE_BYTES;

   // allocate memory for a working buffer
   byte *buffer = new byte[totalBytes];

   // ensure all memory is initialised with the value 0x00
   memset(buffer, 0, totalBytes);

   // the buffer now contains everything from byte #2 onwards
   ndef_message->encode(buffer + 2);

   // insert the two NTAG header bytes (TNF + total bytes)
   buffer[0] = NDEF_EN_RECORD_TNF;
   buffer[1] = (uint8_t)(ndef_message->getEncodedSize());

   // how many pages will it take to write this to the card?
   pages = GetPageCount(totalBytes);

   // initialse the page indexes
   uint8_t page = BYTES_PER_BLOCK;
   uint8_t offset = 0;

   // write to each of the required pages
   for (int i = 0; i < pages; i++)
   {
      memcpy(pageBuffer, buffer + offset, BYTES_PER_BLOCK);
      nfc.ntag2xx_WritePage(page, pageBuffer);
      ++page;
      offset += BYTES_PER_BLOCK;
      ToggleLED(true);
      PublishWriteFeedback((uint8_t)((pages - i) & 0xff), (uint8_t)(((pages - i) >> 8) & 0xff));
   }

   // release buffer resouces
   delete[] buffer;

   // post two 0xff bytes to signify end of write sequence
   PublishWriteFeedback(0x00, 0x00);

   // reset the LED
   ToggleLED(false);
}

/// <summary>
/// Completely wipes an NTAG card of all contents
/// </summary>
/// <param name="headerdata">reference to the read NDEF message header</param>
/// <param name="pagedata">reference to the read NDEF message payload</param>
void ClearTheCard(uint8_t *headerdata, uint8_t *pagedata)
{
   // create the page buffer
   uint8_t pageBuffer[BYTES_PER_BLOCK] = {0, 0, 0, 0};

   // default page clear count
   int pages = headerdata[NTAG_CAPABILITY_CONTAINER] * 2;

   // clear either the default number (16) or all pages on the card
   for (uint8_t i = 4; i < pages + 4; i++)
   {
      memset(pageBuffer, 0, 4);
      nfc.ntag2xx_WritePage(i, pageBuffer);
      ToggleLED(true);
      PublishWriteFeedback((uint8_t)((pages - i) & 0xff), (uint8_t)(((pages - i) >> 8) & 0xff));
   }

   // post two 0xff bytes to signify end of write sequence
   PublishWriteFeedback(0x00, 0x00);

   // now we clear EVERYTHING!! This includes local memory and the NFC reader cache!
   std::fill_n(pagedata, TOTAL_BLOCKS * BYTES_PER_BLOCK, 0);
   std::fill_n(headerdata, BLOCK_SIZE_BLE, 0);
   if (ndef_message->getRecordCount() > 0)
   {
      ndef_message->dropAllRecords();
   }

   // reset the LED
   ToggleLED(false);
}
#pragma endregion

//------------------------------------------------------------------------------------------------

#pragma region PRIVATE SUPPORT METHODS
/// <summary>
/// Prints the cache for the current NDEF record
/// </summary>
void DebugPrintCache()
{
   NDEF_Record debugRecord = ndef_message->getRecord(ndef_message->getRecordCount() - 1);
   int recordLength = debugRecord.getPayloadLength();
   byte *debugNdefRecord = new byte[NTAG_MAX_RECORD_BYTES];
   debugRecord.getPayload(debugNdefRecord);
   READER_DEBUGPRINT.print(recordLength);
   READER_DEBUGPRINT.print(":  ");
   for (int i = 0; i < recordLength; ++i)
   {
      READER_DEBUGPRINT.print(debugNdefRecord[i], HEX);
      READER_DEBUGPRINT.print(" ");
   }
   READER_DEBUGPRINT.println("");
   delete[] debugNdefRecord;
}

/// <summary>
/// Publish the current page number being written
/// </summary>
/// <param name="pageLow">low byte value</param>
/// <param name="pageHigh">high byte value</param>
void PublishWriteFeedback(byte pageLow, byte pageHigh)
{
   uint8_t *responsePayload = new uint8_t[OPERAND_BYTES];
   responsePayload[0] = pageHigh;
   responsePayload[1] = pageLow;
   responsePayload[2] = 0x0d;
   responsePayload[3] = 0x0a;
   PublishResponseToBluetooth(responsePayload);
   delete[] responsePayload;
}

/// <summary>
/// How many pages are required to cover all message bytes?
/// </summary>
/// <param name="byteCount">number of message bytes</param>
int GetPageCount(int byteCount)
{
   int pages = byteCount / BYTES_PER_BLOCK;
   int pagesModulo = byteCount % BYTES_PER_BLOCK;
   if (pagesModulo > 0)
   {
      ++pages;
   }
   return pages;
}

/// <summary>
/// Reset the reader after RTOS timeout and erase the NDEF message cache
/// </summary>
void ResetReader()
{
   _readerBusy = false;
   _blockReader = false;
   _command = ReadCardContinuous;
   if (ndef_message->getRecordCount() > 0)
   {
      ndef_message->dropAllRecords();
   }
}

/// <summary>
/// How many NDEF records are currently stored in the hardware cache?
/// </summary>
void GetCachedRecordCount(uint8_t &cachedRecordCount)
{
   cachedRecordCount = (uint8_t)ndef_message->getRecordCount();
}

/// <summary>
/// Get the received command type
/// </summary>
/// <param name="buffer">byte array to search against</param>
PN532_command GetCommandType(uint8_t *buffer)
{
   if (memcmp(buffer, CONTINUOUS_READ_CARD, 2) == 0)
   {
      return ReadCardContinuous;
   }
   else if (memcmp(buffer, SINGLE_READ_CARD, 2) == 0)
   {
      return ReadCardOnce;
   }
   else if (memcmp(buffer, ADD_NDEF_RECORD, 2) == 0)
   {
      return AddNdefRecordToCache;
   }
   else if (memcmp(buffer, APPEND_NDEF_RECORD, 2) == 0)
   {
      return AppendToCachedNdefRecord;
   }
   else if (memcmp(buffer, PUBLISH_TO_CARD, 2) == 0)
   {
      return PublishCacheToCard;
   }
   else if (memcmp(buffer, ERASE_CARD_CONTENTS, 2) == 0)
   {
      return EraseCardContents;
   }
   else if (memcmp(buffer, COUNT_NDEF_RECORDS, 2) == 0)
   {
      return CountCachedNdefRecords;
   }
   else if (memcmp(buffer, CLEAR_NDEF_CACHE, 2) == 0)
   {
      return EraseCachedNdefRecords;
   }
   else if (memcmp(buffer, GET_ENCODED_SIZE, 2) == 0)
   {
      return GetEncodedSize;
   }
   else if (memcmp(buffer, READ_BATTERY_VOLTAGE, 2) == 0)
   {
      return ReadBatteryVoltage;
   }
   else if (memcmp(buffer, RESEND_FAILED_PAYLOAD, 2) == 0)
   {
      return ResendFailedPayload;
   }
   else
   {
      return ReadCardContinuous;
   }
}

/// <summary>
/// Toggle the LED ON or OFF every time this method is called
/// </summary>
/// <param name="period">true for toggle else false for LED OFF</param>
void ToggleLED(bool enableToggle)
{
   // if we're forcing the LED to be OFF, then do so here
   if (!enableToggle)
   {
      digitalWrite(COMMS_LED, LOW);
   }

   else
   {
      // otherwise just toggle the existing state
      if (digitalRead(COMMS_LED) == HIGH)
      {
         digitalWrite(COMMS_LED, LOW);
      }
      else
      {
         digitalWrite(COMMS_LED, HIGH);
      }
   }
}

/// <summary>
/// Flashes the COMMS LED
/// </summary>
/// <param name="period">milliseconds to illuminate for</param>
void FlashLED(int onPeriod, int offPeriod)
{
   digitalWrite(COMMS_LED, HIGH);
   delay(onPeriod);
   digitalWrite(COMMS_LED, LOW);
   delay(offPeriod);
}

/// <summary>
/// This simple method references a specific byte against a return value
/// in a CRC lookup table. There are two CRC types, one for outgoing and
/// another for incoming binary data. The two values CRC_in and CRC_out
/// are local to this class.
/// </summary>
/// <param name="inOut">true if data is outbound to control</param>
/// <param name="newChar">Byte to lookup</param>
void calculateCRC(bool inOut, byte new_char)
{
   if (inOut == OUT) // If out-going CRC is calculated
   {
      CRC_out = CRC_TABLE[CRC_out ^ new_char];
   }
   else
   {
      CRC_in = CRC_TABLE[CRC_in ^ new_char];
   }
}
#pragma endregion
