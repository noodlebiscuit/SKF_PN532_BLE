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
#pragma endregion

//------------------------------------------------------------------------------------------------

#pragma region BLUETOOTH LOW ENERGY SUPPORT
/// <summary>
/// Configure the BLE hardware
/// </summary>
void SetupBLE()
{
   // initiate BLE comms
   StartBLE();

   // Create BLE service and characteristics.
   BLE.setLocalName(nameOfPeripheral);
   BLE.setAdvertisedService(nearFieldService);
   nearFieldService.addCharacteristic(rxChar);
   nearFieldService.addCharacteristic(txChar);
   BLE.addService(nearFieldService);

   // Bluetooth LE connection handlers.
   BLE.setEventHandler(BLEConnected, onBLEConnected);
   BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);

   // Event driven reads.
   rxChar.setEventHandler(BLEWritten, onRxCharValueUpdate);

   // Let's tell devices about us.
   BLE.advertise();
}

/// <summary>
/// Start the BLE service!
/// </summary>
void StartBLE()
{
   if (!BLE.begin())
   {
      if (IS_DEGUG)
      {
         Serial.println("starting BLE failed!");
      }
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
   SetConnectedToBLE = HIGH;
   ResetReader();
}

/// <summary>
/// A BLE device has just disconnected from our sensor - power off the connection LED
/// </summary>
/// <param name="central">BLE device</param>
void onBLEDisconnected(BLEDevice central)
{
   SetConnectedToBLE = LOW;
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
   // initialise responses here (cannot be done within the switch() statement)
   uint8_t cachedRecordCount = 0x00;
   uint8_t *responsePayload = new uint8_t[OPERAND_BYTES];

   // process any received payload
   _command = GetCommandType(message);

   // OK, and what's expected of us here?
   switch (_command)
   {
   case ReadCardContinuous:
      _blockReader = false;
      Serial.println("Read continuous (executed within main reader loop)");
      break;

   case ReadCardOnce:
      _blockReader = false;
      Serial.println("Read once (executed within main reader loop)");
      break;

   case AddNdefRecordToCashe:
      _command = ReadCardContinuous;
      _blockReader = false;

      AddNdefRecordToMessage(message, messageSize);
      Serial.println("Add single record to cache");
      break;

   case CountCachedNdefRecords:
      GetCachedRecordCount(cachedRecordCount);

      responsePayload[0] = 0x00;
      responsePayload[1] = cachedRecordCount;
      responsePayload[2] = 0x0d;
      responsePayload[3] = 0x0a;

      PublishResponseToBluetooth(responsePayload);

      _command = ReadCardContinuous;
      Serial.println("Return number of cached NDEF records");
      break;

   case EraseCardContents:
      Serial.println("Erase card contents (executed within main reader loop)");
      break;

   case PublishCacheToCard:
      Serial.println("Publish cache to card (executed within main reader loop)");
      break;

   default:
      Serial.println("Unknown");
      break;
   }
   delete[] responsePayload;
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
   // make sure we don't have any NFC scanning overlaps here
   _readerBusy = true;

   // write the header block
   txChar.writeValue(headerdata, BLOCK_SIZE_BLE);

   // what is the total message size in bytes?
   int message_length = pagedata[1] + 3;
   Serial.println(message_length);

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

   // send the payload terminator
   delayMicroseconds(BLOCK_WAIT_BLE);
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

   uint32_t versiondata = nfc.getFirmwareVersion();
   Serial.print(versiondata);

   // lastly we setup the BLE layer
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
/// Writes the NDEF contents of a card to the serial port
/// </summary>
/// <param name="message">reference to the read NDEF message</param>
void ConnectToReader(void)
{
   // if the reader is blocked, then bypass this method completely
   if (!(_blockReader | _readerBusy))
   {
      uint8_t pagedata[TOTAL_BLOCKS * BYTES_PER_BLOCK];
      uint8_t headerdata[BLOCK_SIZE_BLE];

      // read the card
      uint8_t uidLength = Read_PN532(pagedata, headerdata);

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
            else
            {
               Serial.println(INVALID_NDEF);
            }

            // clear TAG contents or write complete NDEF message
            ExecuteReaderCommands(headerdata, pagedata);

            // reset any command that might have been received
            _command = ReadCardContinuous;
         }
         // release this object and leave method right here!
         delete[] uidRecord;
         return;
      }

      // if we've reached this point then we need to reset the received UID
      for (uint8_t i = 0; i < UID_LENGTH; ++i)
      {
         _headerdata[i] = 0x00;
      }
   }
}

/// <summary>
/// Writes the NDEF contents of a card to the serial port
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
/// Clear TAG contents or write complete NDEF message
/// </summary>
/// <param name="headerdata">reference to the read NDEF message header</param>
/// <param name="pagedata">reference to the read NDEF message body</param>
void ExecuteReaderCommands(uint8_t *headerdata, uint8_t *pagedata)
{
   // is this an NTAG compatible card?
   bool isNTAG = false;

   if ((pagedata[12] == CC[0]) & (pagedata[13] == CC[1]) & (pagedata[15] == CC[3]))
   {
      if (pagedata[14] == NTAG_213_IC)
      {
         isNTAG = true;
         Serial.println("NTAG213");
      }
      else if (pagedata[14] == NTAG_215_IC)
      {
         isNTAG = true;
         Serial.println("NTAG215");
      }
      else if (pagedata[14] == NTAG_216_IC)
      {
         isNTAG = true;
         Serial.println("NTAG216");
      }
   }

   // process the two supported commands (CLEAR and WRITE NDEF MESSAGE)
   if (_command == EraseCardContents)
   {
      _blockReader = true;
      // ClearTheCard(headerdata);
   }
   else if (_command == PublishCacheToCard)
   {
      // at this point we don't want the reader to keep checking for Tags
      _blockReader = true;

      // now we publish the cached payload and drop all existing records from the NDEF cache
      // WriteNdefMessagePayload(headerdata, isNTAG);
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
/// Appends a received NDEF record to an existing NDEF message
/// </summary>
/// <param name="message">pointer to the received command message byte array</param>
/// <param name="messageSize">number of bytes in the command message</param>
void AddNdefRecordToMessage(byte *message, int messageSize)
{
   // ensure the message size does not exceed the set limit
   if (messageSize > NTAG_MAX_RECORD_BYTES + 2)
   {
      messageSize = NTAG_MAX_RECORD_BYTES + 2;
   }

   // create a new ndef record string buffer
   char *ndefRecord = new char[NTAG_MAX_RECORD_BYTES];

   // clear the serial read buffer contents
   memset(ndefRecord, 0, NTAG_MAX_RECORD_BYTES);

   // strip away the first two command characters
   uint8_t index = 0;
   for (int i = 2; i < messageSize; ++i)
   {
      ndefRecord[index] = message[i];
      ++index;
   }

   // how many bytes would be needed to encode the current NTAG payload?
   uint8_t totalBytes = ndef_message->getEncodedSize() + NDEF_EN_RECORD_EXTRA_PAGE_BYTES;

   // if we add the current record size, how many bytes does that rise to?
   int newRecordSize = totalBytes + index;

   // provided that doesn't exceed the physical maximum, then add the current record to the cache
   if (newRecordSize < NTAG_MAX_TOTAL_BYTES)
   {
      // add an NDEF text record to the NDEF message
      ndef_message->addTextRecord(ndefRecord);

      // post feedback
      Serial.println(totalBytes);
      Serial.write(ndefRecord, index);

      // terminate with a CR and LF
      Serial.write(0x0d);
      Serial.write(0x0a);
   }
   else
   {
      Serial.println("ERROR: CARD IS FULL");
   }

   delete[] ndefRecord;
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
   byte buffer[totalBytes];

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
   }

   // reset the LED
   ToggleLED(false);
}
#pragma endregion

//------------------------------------------------------------------------------------------------

#pragma region PRIVATE SUPPORT METHODS
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
/// Reset the reader after RTOS timeout
/// </summary>
void ResetReader()
{
   _readerBusy = false;
   _blockReader = false;
   _command = ReadCardContinuous;
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
      return AddNdefRecordToCashe;
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
#pragma endregion
