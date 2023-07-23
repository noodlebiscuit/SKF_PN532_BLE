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
 *
 * Description: main.cpp
 * =====================
 *
 * The software allows you to emulate the functionality of a FEIG ECCO+ BLE NearField reader
 * using an ARDUINO NANO 33 BLE (with a Nordic NRF-52840 Microcontroller) as well as a PN532
 * NFC reader board from either SUNFOUNDER or ADA FRUIT.
 *
 * The protocol being used is the proprietary SCANNDY SComP by PANMOBIL (FEIG)
 *
 * No source code from either FEIG or PANMOBIL is contained in this firmware, and it is provided
 * purely to allow engineers who are developing for the ECCO+, to be able to debug Bluetooth data
 * at a VERY low level. It is ABSOLUTELY NOT intended for use in ANY commercial application!
 *
 * The author CANNOT guarantee that everything here is correct, and FEIG has no involvement with
 * the project at ANY level.
 *
 * July 2023
 *
 ***************************************************************************************************/

#include "main.h"

//------------------------------------------------------------------------------------------------

#pragma region BLUETOOTH LOW ENERGY SUPPORT
///
/// @brief Configure the BLE hardware
///
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
   // rxChar.setEventHandler(BLEWritten, onRxCharValueUpdate);
   rxChar.setEventHandler(BLEWritten, onBLEWritten);

   // Let's tell all local devices about us.
   BLE.advertise();

   // set the default characteristics
   PublishHardwareDetails();

   // register the current battery voltage
   PublishBattery();

   // clear the BLE serial receive buffer
   _SerialBuffer.clear();
}

///
/// @brief Publish the initial details of this device at POWER ON
///
void PublishHardwareDetails()
{
   manufacturerCharacteristic.writeValue(MANUFACTURER_NAME_STRING, false);
   modelNumberCharacteristic.writeValue(MODEL_NAME_STRING, false);
   hardwareCharacteristic.writeValue(HARDWARE_NAME_STRING, false);
   firmwareRevisionCharacteristic.writeValue(FIRMWARE_NAME_STRING, false);
   serialNumberCharacteristic.writeValue(SERIAL_NO_NAME_STRING, false);
}

///
/// @brief Add the device information service and associate the required characteristics
///
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

///
/// @brief Add the battery state service and associate the required characteristics
///
void AddBatteryServiceBLE()
{
   BLE.setAdvertisedService(batteryService);
   batteryService.addCharacteristic(batteryCharacteristic);
   BLE.addService(batteryService);
}

///
/// @brief Add the core data service and associate the required characteristics
///
void AddDataServiceBLE()
{
   BLE.setAdvertisedService(nearFieldService);
   nearFieldService.addCharacteristic(rxChar);
   nearFieldService.addCharacteristic(txChar);
   BLE.addService(nearFieldService);
}

///
/// @brief Start the BLE service!
///
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

///
/// @brief A BLE device has connected to our sensor - illuminate the connection LED
/// @param cental BLE device
///
void onBLEConnected(BLEDevice central)
{
   LED_SetConnectedToBLE = HIGH;
   ResetReader();
}

///
/// @brief A BLE device has just disconnected from our sensor - power off the connection LED
/// @param central BLE device
///
void onBLEDisconnected(BLEDevice central)
{
   LED_SetConnectedToBLE = LOW;
}

///
/// @brief Reads and returns an averaged value for the 3.7V Lithium ion battery in MV
/// @param PIN what analogue pin are we connecting to?
/// @param average how many samples to read and average
/// @return voltage as a big-endian integer
///
uint16_t ReadBattery(pin_size_t PIN, int average)
{
   float value = 0.0;
   for (int i = 0; i < average; ++i)
   {
      value += (float)analogRead(PIN);
      delayMicroseconds(BATTERY_READ_DELAY);
   }
   value = (value * BATTERY_DESCALE) / average;
   return (uint16_t)(value);
}

///
/// @brief Streams the NDEF contents out over Bluetooth as a series HEX NOTATION characters
/// @brief Select using SET_OUTPUT_AS_BINARY
/// @brief Example: UID 04:4d:ec:b4 will be returned as "044decb4"
/// @param pagedata raw NDEF message payload
/// @param headerdata NDEF meassage header with UUID
///
void PublishHexPayloadToBluetooth(uint8_t *pagedata, uint8_t *headerdata)
{
   const char *hexNotation;
   uint8_t *queryBody = new uint8_t[BLOCK_SIZE_BLE];

   // make sure we don't have any NFC scanning overlaps here
   _readerBusy = true;

   // what is the total message size in bytes? We get this from the TAG data itself
   int message_length = pagedata[1] + 3;

   // convert the TAG header into HEX NOTATION strings (as opposed to raw binary)
   hexNotation = HexStr(headerdata, BLOCK_SIZE_BLE, HEX_UPPER_CASE);

   // now we need to double the length of the message string (ONE byte value needs TWO chars)
   message_length *= 2;

   // how many bytes is this payload going to contain in total?
   uint16_t totalBytes = RFID_RESPONSE_BYTES + (BLOCK_SIZE_BLE * 2) + (message_length);

   // set the SCOMP PROTOCOL total TAG payload length
   PAYLOAD_LEGTH[0] = (uint8_t)((totalBytes & 0xff00) >> 8);
   PAYLOAD_LEGTH[1] = (uint8_t)(totalBytes & 0x00ff);

   // insert the payload length into the SCOMP PROTOCOL RFID DATA HEADER
   const char *payloadLength = HexStr(PAYLOAD_LEGTH, LENGTH_BYTES, HEX_UPPER_CASE);
   for (int i = 0; i < (int)sizeof(payloadLength); i++)
   {
      scomp_rfid_response_header[i + 5] = payloadLength[i];
   }

   // generate the CRC for the SCANNDY PROTOCOL HEADER
   crc.update(scomp_rfid_response_header, HEADER_BYTES);

   // PUBLISH SCANNDY PROTOCOL HEADER TO BLUETOOTH
   txChar.writeValue(scomp_rfid_response_header, false);
   delayMicroseconds(BLOCK_WAIT_BLE);

   // generate the CRC for the NFC (ISO 14443) header block
   crc.update(hexNotation, (BLOCK_SIZE_BLE * 2));

   // reset the page index
   int index = 0;

   // PUBLISH THE NTAG (ISO14443) 16 BYTES UUID HEADER
   for (int k = 0; k < 2; k++)
   {
      memset(queryBody, 0, BLOCK_SIZE_BLE);
      for (int i = 0; i < BLOCK_SIZE_BLE; i++)
      {
         queryBody[i] = (uint8_t)hexNotation[i + (index * BLOCK_SIZE_BLE)];
      }
      txChar.writeValue(queryBody, BLOCK_SIZE_BLE);
      delayMicroseconds(BLOCK_WAIT_BLE);
      index++;
   }

   // convert the TAG PAGE DATA into HEX NOTATION strings (as opposed to raw binary)
   hexNotation = HexStr(pagedata, message_length, HEX_UPPER_CASE);

   // generate the CRC for the NFC (ISO 14443) user data payload (NDEF)
   crc.update(hexNotation, (message_length));

   // reset the page index
   index = 0;

   // PUBLISH THE NTAG (ISO14443) USER DATA (AKA NDEF)
   while (message_length >= 0)
   {
      // flush the transmission buffer and allow for some delay
      memset(queryBody, 0, BLOCK_SIZE_BLE);
      delayMicroseconds(BLOCK_WAIT_BLE);

      //
      // we initially transmit data in 16 byte blocks, then transmit the
      // remaining bytes together in a single payload
      //
      if (message_length >= BLOCK_SIZE_BLE)
      {
         for (int i = 0; i < BLOCK_SIZE_BLE; i++)
         {
            queryBody[i] = (uint8_t)hexNotation[i + (index * BLOCK_SIZE_BLE)];
         }
         txChar.writeValue(queryBody, BLOCK_SIZE_BLE);
         index++;
      }
      else
      {
         for (int i = 0; i < message_length; i++)
         {
            queryBody[i] = (uint8_t)hexNotation[i + (index * BLOCK_SIZE_BLE)];
         }
         txChar.writeValue(queryBody, message_length);
      }
      message_length -= BLOCK_SIZE_BLE;
   }

   // release this memory
   delete[] queryBody;

   // add the serial port delay to improve comms efficiency
   delayMicroseconds(BLOCK_WAIT_BLE);

   // publish the final CRC as an array of bytes
   crc.finalizeAsArray(EOR);
   const char *crcValue = HexStr(EOR, FOOTER_BYTES, HEX_UPPER_CASE);
   txChar.writeValue(crcValue, false);
   crc.reset();

   // close for DEBUG
   delayMicroseconds(BLOCK_WAIT_BLE);
   txChar.writeValue(CR_LF, 2);

   // release the blocker
   _readerBusy = false;
}

///
/// @brief Streams the NDEF contents out over Bluetooth as a series of 16 byte packets
/// @brief Select using SET_OUTPUT_AS_BINARY
/// @param pagedata raw NDEF message payload
/// @param headerdata NDEF meassage header with UUID
///
void PublishBinaryPayloadToBluetooth(uint8_t *pagedata, uint8_t *headerdata)
{
   // make sure we don't have any NFC scanning overlaps here
   _readerBusy = true;

   // what is the total message size in bytes?
   int message_length = pagedata[1] + 3;

   // how many bytes is this payload going to contain?
   uint16_t totalBytes = RFID_RESPONSE_BYTES + BLOCK_SIZE_BLE + message_length;

   // set the SCOMP PROTOCOL total TAG payload length
   PAYLOAD_LEGTH[0] = (uint8_t)((totalBytes & 0xff00) >> 8);
   PAYLOAD_LEGTH[1] = (uint8_t)(totalBytes & 0x00ff);

   // insert the payload length into the SCOMP PROTOCOL RFID DATA HEADER
   const char *payloadLength = HexStr(PAYLOAD_LEGTH, LENGTH_BYTES, HEX_UPPER_CASE);
   for (int i = 0; i < (int)sizeof(payloadLength); i++)
   {
      scomp_rfid_response_header[i + 5] = payloadLength[i];
   }

   // generate the CRC for the payload header block
   crc.update(scomp_rfid_response_header, HEADER_BYTES);

   // PUBLISH SCANNDY PROTOCOL HEADER TO BLUETOOTH
   txChar.writeValue(scomp_rfid_response_header, false);

   // generate the CRC for the NFC (ISO 14443) header block
   crc.update(headerdata, BLOCK_SIZE_BLE);

   // PUBLISH ISO14443 TAG DATA TO BLUETOOTH
   delayMicroseconds(BLOCK_WAIT_BLE);
   txChar.writeValue(headerdata, BLOCK_SIZE_BLE);

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
         txChar.writeValue(pagedata + (index * BLOCK_SIZE_BLE), message_length);
      }
      message_length -= BLOCK_SIZE_BLE;
   }

   // append the CRC based on the transmitted payload
   message_length = pagedata[1] + 3;
   crc.update(pagedata, message_length);

   // add the serial port delay to improve comms efficiency
   delayMicroseconds(BLOCK_WAIT_BLE);

   // publish the final CRC as an array of bytes
   crc.finalizeAsArray(EOR);
   const char *crcValue = HexStr(EOR, FOOTER_BYTES, HEX_UPPER_CASE);
   txChar.writeValue(crcValue, false);
   crc.reset();

   // close for DEBUG
   delayMicroseconds(BLOCK_WAIT_BLE);
   txChar.writeValue(CR_LF, 2);

   // release the blocker
   _readerBusy = false;
}

///
/// @brief Streams the NDEF UID header out over Bluetooth as a single sixteen byte packet
/// @param headerdata NDEF meassage header with UUID
///
void PublishBinaryUIDToBluetooth(uint8_t *headerdata)
{
   // make sure we don't have any NFC scanning overlaps here
   _readerBusy = true;

   // how many bytes is this payload going to contain?
   uint16_t totalBytes = RFID_RESPONSE_BYTES + BLOCK_SIZE_BLE;

   // set the SCOMP PROTOCOL total TAG payload length
   PAYLOAD_LEGTH[0] = (uint8_t)((totalBytes & 0xff00) >> 8);
   PAYLOAD_LEGTH[1] = (uint8_t)(totalBytes & 0x00ff);

   // insert the payload length into the SCOMP PROTOCOL RFID DATA HEADER
   const char *payloadLength = HexStr(PAYLOAD_LEGTH, LENGTH_BYTES, HEX_UPPER_CASE);
   for (int i = 0; i < (int)sizeof(payloadLength); i++)
   {
      scomp_rfid_response_header[i + 5] = payloadLength[i];
   }

   // generate the CRC for the payload header block
   crc.update(scomp_rfid_response_header, HEADER_BYTES);

   // PUBLISH SCANNDY PROTOCOL HEADER TO BLUETOOTH
   txChar.writeValue(scomp_rfid_response_header, false);

   // generate the CRC for the NFC (ISO 14443) header block
   crc.update(headerdata, BLOCK_SIZE_BLE);

   // PUBLISH ISO14443 TAG DATA TO BLUETOOTH
   delayMicroseconds(BLOCK_WAIT_BLE);
   txChar.writeValue(headerdata, BLOCK_SIZE_BLE);

   // add the serial port delay to improve comms efficiency
   delayMicroseconds(BLOCK_WAIT_BLE);

   // publish the final CRC as an array of bytes
   crc.finalizeAsArray(EOR);
   const char *crcValue = HexStr(EOR, FOOTER_BYTES, HEX_UPPER_CASE);
   txChar.writeValue(crcValue, false);
   crc.reset();

   // close for DEBUG
   delayMicroseconds(BLOCK_WAIT_BLE);
   txChar.writeValue(CR_LF, 2);

   // release the blocker
   _readerBusy = false;
}
#pragma endregion

//-------------------------------------------------------------------------------------------------

#pragma region STRING MANAGEMENT AND SUPPORT
///
/// @brief inserts on string into another
/// @param a source string
/// @param b substring
/// @param position insert position
///
void InsertSubstring(char *a, const char *b, int position)
{
   char *f, *e;
   int length;

   length = strlen(a);

   f = Substring(a, 1, position - 1);
   e = Substring(a, position, length - position + 1);

   strcpy(a, "");
   strcat(a, f);
   free(f);
   strcat(a, b);
   strcat(a, e);
   free(e);
}

///
/// @brief
/// @param string raw string
/// @param position starting character index
/// @param length number of characters to extract
/// @return
///
char *Substring(char *string, int position, int length)
{
   char *pointer;
   int c;

   pointer = (char *)malloc(length + 1);

   if (pointer == NULL)
      exit(EXIT_FAILURE);

   for (c = 0; c < length; c++)
      *(pointer + c) = *((string + position - 1) + c);

   *(pointer + c) = '\0';

   return pointer;
}

///
/// @brief converts an array of bytes into an ASCII string
/// @brief E.g. {0x22, 0x4a, 0x0f, 0xe2} would return as '224a0fe2'
/// @param data array of bytes
/// @param len number of bytes to process
/// @param uppercase return hex string with value A->F in upper case
/// @return standard C string (array of chars)
///
const char *HexStr(const uint8_t *data, int len, bool uppercase)
{
   std::stringstream ss;
   ss << std::hex;

   for (int i(0); i < len; ++i)
   {
      ss << std::setw(2) << std::setfill('0') << (int)data[i];
   }

   std::string x = ss.str();
   if (uppercase)
   {
      std::transform(x.begin(), x.end(), x.begin(), ::toupper);
   }
   return x.c_str();
}
#pragma endregion

//-------------------------------------------------------------------------------------------------

#pragma region APPLICATION CORE AND OS TIMER
///
/// @brief Setup the ARDUINO
///
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

///
/// @brief APPLICATION SUPER LOOP
/// @param void
///
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
            ProcessReceivedQueries();
         }
      }
   }
}

///
/// @brief MBED timer tick event *** APPLICATION CORE ***
///
void AtTime()
{
   timerEvent = true;
}
#pragma endregion

//------------------------------------------------------------------------------------------------

#pragma region NEAR FIELD COMMUNICATIONS SUPPORT METHODS
///
/// @brief Publish the current battery voltage, if and when it's possible to do so
///
void PublishBattery()
{
   // if the reader is blocked, then bypass this method completely
   if (!(_blockReader | _readerBusy))
   {
      // increment the battery read count
      if (++_batteryCount > BATTERY_UPDATE_COUNTER)
      {
         _batteryCount = 0x0000;
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

///
/// @brief Main entry point for PN532 scanning
/// @param void
///
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

         // what is the total message size in bytes?
         int message_length = pagedata[1] + 3;

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
                  if (SET_OUTPUT_AS_BINARY)
                  {
                     // publish data in raw binary format
                     PublishBinaryPayloadToBluetooth(pagedata, headerdata);
                  }
                  else
                  {
                     // publish data as formatted hex string
                     PublishHexPayloadToBluetooth(pagedata, headerdata);
                  }
               }
            }
            else
            {
               PublishBinaryUIDToBluetooth(headerdata);
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
         /* TODO ADD ERROR HERE*/
         // PublishWriteFeedback(CARD_ERROR_EMPTY[0], CARD_ERROR_EMPTY[1]);
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

///
/// @brief Reads the card contents and returns over Bluetooth
/// @param pagedata returns the NDEF message payload
/// @param headerdata returns the NDEF meassage header
/// @return number of bytes in the TAG UUID
///
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

///
/// @brief Reads the cards CAPACITY CONTAINER (from Page #3) and returns the TAG type
/// @brief this page spans bytes 12, 13, 14 and 15, where 14 is the card type
/// @param headerdata reference to the read NDEF message header
/// @return NTAG card version (i.e. NTAG-215)
///
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

///
/// @brief What is the maximum available memory for a specific card type
/// @param cardType embedded card type
/// @return available bytes
///
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

///
/// @brief Executes a specific reader command
/// @param headerdata reference to the read NDEF message header
/// @param pagedata reference to the read NDEF message body
///
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
         PublishResponseToBluetooth(scomp_response_error, sizeof(scomp_response_error) - 1);
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

///
/// @brief Appends a received NDEF BINARY record to an existing NDEF message
/// @brief Note: this differs from an NDEF TEXT record in that it allows invalid
/// @brief characters - specifically /ESC and 0x00
/// @param message pointer to the received command message byte array
/// @param messageSize number of bytes in the command message
///
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

///
/// @brief Appends a received NDEF TEXT record to an existing NDEF message
/// @brief Note: this differs from an NDEF BINARY record in that it blocks invalid
/// @brief characters such as /ESC and 0x00
/// @param message pointer to the received command message byte array
/// @param messageSize number of bytes in the command message
///
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

///
/// @brief Clears and overwrites the complete contents of the NDEF message block
/// @param headerdata reference to the read NDEF message header
/// @param clearCard clear card before write action
///
void WriteNdefMessagePayload(uint8_t *headerdata, bool clearCard)
{
   // let the user know we've detected the tag or sensor
   PublishResponseToBluetooth(scomp_response_processing, sizeof(scomp_response_processing) - 1);

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
   }

   // release buffer resouces
   delete[] buffer;

   // let the user know we're done here!
   PublishResponseToBluetooth(scomp_response_ok, sizeof(scomp_response_ok) - 1);

   // reset the LED
   ToggleLED(false);
}

///
/// @brief Completely wipes an NTAG card of all contents
/// @param headerdata reference to the read NDEF message header
/// @param pagedata reference to the read NDEF message payload
///
void ClearTheCard(uint8_t *headerdata, uint8_t *pagedata)
{
   // let the user know we've detected the tag or sensor
   PublishResponseToBluetooth(scomp_response_processing, sizeof(scomp_response_processing) - 1);

   READER_DEBUGPRINT.print("ClearTheCard()");

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
      READER_DEBUGPRINT.print(i);
      READER_DEBUGPRINT.print(" ");
   }

   // let the user know we're done here!
   PublishResponseToBluetooth(scomp_response_ok, sizeof(scomp_response_ok) - 1);

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
///
/// @brief Prints the cache for the current NDEF record
///
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

///
/// @brief How many pages are required to cover all message bytes?
/// @param byteCount number of message bytes
/// @return number of bytes counted
///
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

///
/// @brief Reset the reader after RTOS timeout and erase the NDEF message cache
///
void ResetReader()
{
   _readerBusy = false;
   _blockReader = false;
   _command = ReadCardContinuous;
   _SerialBuffer.clear();
   _messageIdentifier = 0x0000;
   _queryReceived = false;
   _invalidQueryReceived = false;
   _scomp_command = none;

   if (ndef_message->getRecordCount() > 0)
   {
      ndef_message->dropAllRecords();
   }
}

///
/// @brief How many NDEF records are currently stored in the hardware cache?
/// @param cachedRecordCount referenced return value
///
void GetCachedRecordCount(uint8_t &cachedRecordCount)
{
   cachedRecordCount = (uint8_t)ndef_message->getRecordCount();
}

///
/// @brief Toggle the LED ON or OFF every time this method is called
/// @param enableToggle when true, wwitch LED ON and then OFF
///
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

///
/// @brief Flashes the COMMS LED
/// @param onPeriod milliseconds to illuminate for
/// @param offPeriod milliseconds for LED to be OFF
///
void FlashLED(int onPeriod, int offPeriod)
{
   digitalWrite(COMMS_LED, HIGH);
   delay(onPeriod);
   digitalWrite(COMMS_LED, LOW);
   delay(offPeriod);
}
#pragma endregion

//-------------------------------------------------------------------------------------------------

// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************

///
/// @brief EVENT on data written to SPP
/// @brief Message is of format as shown below. Initially we search for chars 'Q'  and  '#'
/// @brief
/// @brief     ->    nnnn Q pppp # n....n cccccccc
/// @brief
/// @brief When we've detected this two character sequence,  we then need to move back FOUR
/// @brief characters so that we're pointing to the characters 'PPPP' - which represent the
/// @brief complete length of the payload as a four-character long HEX string.
/// @brief
/// @brief     ->    .... Q PPPP # nnnnnn cccccccc
/// @brief
/// @brief That done we read PPPP bytes from the input stream and take that as the message.
/// @brief Then we take all received characters (nnnnQpppp#n....n) and calculate the CRC32.
/// @brief Lastly we compate the two CRC32 values to determine if the query is valid
/// @param central connected Bluetooth device
/// @param characteristic data associated with the serial (SPP) BLE service
///
void onBLEWritten(BLEDevice central, BLECharacteristic characteristic)
{
   // at this point we don't want the reader to keep checking for Tags
   _blockReader = true;

   //
   // before we do anything, we need to confirm that we're not looking at a potential buffer
   // overrun here! If we are, then we need to flush the serial receive buffer
   //
   if ((characteristic.valueLength() + _SerialBuffer.getLength()) > _SerialBuffer.getSize())
   {
      _SerialBuffer.clear();
   }

   // that done, we should now be safe to load the received BLE data into the receive buffer
   for (int i = 0; i < characteristic.valueLength(); i++)
   {
      _SerialBuffer.add(characteristic.value()[i]);
   }

   //
   // now we need to find the start of an SCOMP QUERY. This is done by searhing for the
   // character sequence nnnnQpppp#, where nn and pp are two character HEX strings
   //
   int startOfSequence = 0;
   if (_SerialBuffer.getLength() > QUERY_HEADER_BYTES)
   {
      for (int i = 0; i < (_SerialBuffer.getLength() - 5); i++)
      {
         if ((char)_SerialBuffer.get(i) == 'Q' & (char)_SerialBuffer.get(i + 5) == '#')
         {
            startOfSequence = i;
            break;
         }
      }
   }

   // if posn is four or above, then we have detected the start of an SCOMP QUERY string
   if (startOfSequence >= QUERY_OFFSET_BYTES)
   {
      // jump back to the start of the message (to include the request ID)
      startOfSequence -= QUERY_OFFSET_BYTES;

      // we need a temporary buffer here
      char *buffer = new char[RECEIVE_BUFFER_LENGTH];

      // clear the buffer contents
      memset(buffer, 0, RECEIVE_BUFFER_LENGTH);

      // OK, lock and load, and let's see what's in the barrel!
      int index = 0;
      for (int i = startOfSequence; i < (int)(_SerialBuffer.getLength() - startOfSequence); i++)
      {
         buffer[index++] = (char)_SerialBuffer.get(i);
      }

      // get the total payload length
      char *payloadLengthString = new char[5];
      memset(payloadLengthString, 0, 5);

      //
      // well, it's the payload length. Now we need to covert this four character long
      // representation of of a sixteen bit number - into an actual sixteen bit number
      //
      for (int i = 0; i < 4; i++)
      {
         payloadLengthString[i] = buffer[i + 5];
      }
      payloadLengthString[4] = '\n';

      //
      // now we need to convert the eight character long length string into a LONG value
      // and then move that to a sixteen bit unsigned value
      //
      char *ptr;
      uint16_t payloadLength = (uint16_t)strtol(payloadLengthString, &ptr, 16);

      // But what is the total length of the message (i.e. how many chars to we need?)
      uint16_t totalPayloadLength = payloadLength + CRC32_CHARACTERS + QUERY_HEADER_BYTES;

      // we need to extract the core payloads - one with, and one without the CRC32
      char *queryPayload = new char[payloadLength + QUERY_HEADER_BYTES + 1];
      char *queryID = new char[QUERY_OFFSET_BYTES + 1];
      char *queryBody = new char[payloadLength + 1];
      char *queryCRC32 = new char[CRC32_CHARACTERS + 1];
      char *valueCRC32 = new char[3];

#ifdef SERIAL_RECEIVE_DEBUG
      READER_DEBUGPRINT.print("payload string: ");
      READER_DEBUGPRINT.println(payloadLengthString);
      READER_DEBUGPRINT.print("length of command payload: ");
      READER_DEBUGPRINT.println(payloadLength);
      READER_DEBUGPRINT.print("total length of payload (with CRC): ");
      READER_DEBUGPRINT.println(totalPayloadLength);
      READER_DEBUGPRINT.print("receive buffer length: ");
      READER_DEBUGPRINT.println(_SerialBuffer.getLength());
#endif

      // OK, have all the required character been received yet?
      if (totalPayloadLength == _SerialBuffer.getLength())
      {
         // we're done
         // #ifdef SERIAL_RECEIVE_DEBUG
         READER_DEBUGPRINT.println(".");
         // #endif

         // clear the buffer contents again..
         memset(buffer, 0, RECEIVE_BUFFER_LENGTH);
         memset(queryPayload, 0, payloadLength + QUERY_HEADER_BYTES + 1);
         memset(queryCRC32, 0, CRC32_CHARACTERS + 1);
         memset(valueCRC32, 0, 3);
         memset(queryID, 0, QUERY_OFFSET_BYTES + 1);
         memset(queryBody, 0, QUERY_OFFSET_BYTES + 1);

         // extract the complete complete SCOMP QUERY payload (with CRC32)
         index = 0;
         for (int i = startOfSequence; i < (int)(totalPayloadLength - (startOfSequence)); i++)
         {
            buffer[index++] = _SerialBuffer.get(i);
         }

         // extract ONLY the HEADER and the QUERY (we use this to calculate the CRC32)
         for (int i = 0; i < payloadLength + QUERY_HEADER_BYTES; i++)
         {
            queryPayload[i] = buffer[i];
         }
         queryPayload[payloadLength + QUERY_HEADER_BYTES] = (char)0x00;

         // extract only the payload body
         for (int i = 0; i < payloadLength; i++)
         {
            queryBody[i] = buffer[i + QUERY_HEADER_BYTES];
         }
         queryBody[payloadLength] = (char)0x00;

         // extract only the query message identifier
         for (int i = 0; i < QUERY_OFFSET_BYTES; i++)
         {
            queryID[i] = buffer[i];
            scomp_query_ID[i] = buffer[i];
         }
         queryID[QUERY_OFFSET_BYTES] = (char)0x00;
         _messageIdentifier = (uint16_t)strtol(queryID, &ptr, 16);

         // extract the embedded CRC32 value from the received SCOMP QUERY
         for (int i = 0; i < CRC32_CHARACTERS; i++)
         {
            queryCRC32[i] = buffer[i + payloadLength + QUERY_HEADER_BYTES];
         }
         queryCRC32[CRC32_CHARACTERS] = (char)0x00;

         // now we extract the CRC32 from the [*queryBuffer]
         crc.update(queryPayload, payloadLength + QUERY_HEADER_BYTES);
         crc.finalizeAsArray(EOR);

         bool crcIsConfirmed = true;
         index = 0;
         for (size_t i = 0; i < FOOTER_BYTES; i++)
         {
            valueCRC32[0] = queryCRC32[index++];
            valueCRC32[1] = queryCRC32[index++];
            valueCRC32[2] = '\n';
            uint16_t checkValue = (uint16_t)strtol(valueCRC32, &ptr, 16);
            if ((uint16_t)EOR[i] != checkValue)
            {
               crcIsConfirmed = false;
               break;
            }
         }

#ifdef SERIAL_RECEIVE_DEBUG
         READER_DEBUGPRINT.print(">> ID: [");
         READER_DEBUGPRINT.print(_messageIdentifier);
         READER_DEBUGPRINT.print("],   query body: [");
         READER_DEBUGPRINT.print(queryBody);
         READER_DEBUGPRINT.print("],   CRC32: [");
         READER_DEBUGPRINT.print(queryCRC32);
         READER_DEBUGPRINT.print("]    REQUIRED CRC: [");
         for (int i = 0; i < 4; i++)
         {
            READER_DEBUGPRINT.print(EOR[i]);
            READER_DEBUGPRINT.print(",");
         }
         if (crcIsConfirmed)
            READER_DEBUGPRINT.println(" - VALID]");
         else
            READER_DEBUGPRINT.println(" - INVALID]");
#endif

         if (crcIsConfirmed)
         {
            _queryReceived = true;
            _SerialBuffer.clear();
            for (size_t i = 0; i < payloadLength; i++)
            {
               _SerialBuffer.add(queryBody[i]);
            }
         }
         else
         {
            _invalidQueryReceived = true;
            _messageIdentifier = 0x0000;
            _SerialBuffer.clear();
         }
      }
      else
      {
#ifdef SERIAL_RECEIVE_DEBUG
         READER_DEBUGPRINT.print(".");
#endif
      }

      delete[] queryID;
      delete[] queryBody;
      delete[] valueCRC32;
      delete[] queryCRC32;
      delete[] queryPayload;
      delete[] payloadLengthString;
      delete[] buffer;
      crc.reset();

      // enable the reader again
      _blockReader = false;
   }
}

///
/// @brief Process any received query
/// @brief RUN FROM LOOP()
///
void ProcessReceivedQueries()
{
   // if the reader is blocked, then bypass this method completely
   if (!(_blockReader | _readerBusy))
   {
      // if an invalid QUERY was received, then we need to let the client know
      if (_invalidQueryReceived & (_messageIdentifier == 0x000))
      {
         PublishResponseToBluetooth(scomp_response_error, sizeof(scomp_response_error) - 1);
         _queryReceived = false;
         _invalidQueryReceived = false;
         _messageIdentifier = 0x0000;
         _SerialBuffer.clear();
      }

      // otherwise, return feedback and process the query
      else if (_queryReceived & (_messageIdentifier > 0x000))
      {
         // load the query string into its own string for post-processing
         char *queryBody = new char[_SerialBuffer.getLength() + 1];
         memset(queryBody, 0, _SerialBuffer.getLength() + 1);
         for (size_t i = 0; i < _SerialBuffer.getLength(); i++)
         {
            queryBody[i] = _SerialBuffer.get(i);
         }

         std::string search(queryBody);
         for (size_t i = 0; i < SCOMP_COMMAND_COUNT; i++)
         {
            if (search.find(scompCommands[i]) == 0)
            {
               _scomp_command = SCOMP_command(i + 1);
               break;
            }
         }

         // extract the the command payload
         if (_scomp_command != SCOMP_command::none)
         {
            size_t colon = search.find(':');
            char *subs = Substring(queryBody, colon + 2, _SerialBuffer.getLength() - (colon + 1));

            // #ifdef SERIAL_RECEIVE_DEBUG
            READER_DEBUGPRINT.println(subs);
            // #endif
            PublishResponseToBluetooth(scomp_response_ok, sizeof(scomp_response_ok) - 1);

            // process the query command
            switch (_scomp_command)
            {
            case SCOMP_command::barscan:
               READER_DEBUGPRINT.println("BAR SCAN");
               break;
            case SCOMP_command::beep:
               READER_DEBUGPRINT.println("BEEP");
               break;
            case SCOMP_command::getcache:
               ProcessGetCache();
               break;
            case SCOMP_command::getversion:
               READER_DEBUGPRINT.println("GET VERSION");
               break;
            case SCOMP_command::leds:
               READER_DEBUGPRINT.println("LEDS");
               break;
            case SCOMP_command::none:
               READER_DEBUGPRINT.println("NONE");
               break;
            case SCOMP_command::rfidscanUSR:
               ProcessSingleScanUSR(subs, _SerialBuffer.getLength() - (colon + 1));
               break;
            case SCOMP_command::rfidscanTID:
               READER_DEBUGPRINT.println("RFID SCAN TID");
               break;
            case SCOMP_command::rfidwrite:
               ProcessRfidWriteQuery(subs, _SerialBuffer.getLength() - (colon + 1));
               break;
            case SCOMP_command::vibrate:
               READER_DEBUGPRINT.println("VIBRATE");
               break;
            case SCOMP_command::clearcache:
               ProcessClearCache();
               break;
            case SCOMP_command::rfiderase:
               ProcessEraseTag();
               break;
            }

            free(subs);
         }

         delete[] queryBody;
         _queryReceived = false;
         _invalidQueryReceived = false;
         _messageIdentifier = 0x0000;
         _SerialBuffer.clear();
      }
   }
}

///
/// @brief returns total number of bytes in the NDEF Message
///
void ProcessGetCache()
{
#ifdef SERIAL_RECEIVE_DEBUG
   READER_DEBUGPRINT.print("GET CACHE");
#endif
   uint16_t records = ndef_message->getRecordCount();
   uint16_t totalSize = ndef_message->getEncodedSize();
   std::string payload = std::to_string(records) + "," + std::to_string(totalSize);
   PublishResponseToBluetooth(&payload[0], payload.size());
}

///
/// @brief Erase all USR (NDEF) memory in the NTAG
/// @brief Note: this is NOT an official SCANNDY command!
///
void ProcessEraseTag()
{
#ifdef SERIAL_RECEIVE_DEBUG
   READER_DEBUGPRINT.println("ERASE NDEF RECORDS");
#endif

   // clear all existing TAG records
   if (ndef_message->getRecordCount() > 0)
   {
      ndef_message->dropAllRecords();
   }

   //  we want to completely erase the contents of the card
   _command = PN532_command::EraseCardContents;
}

///
/// @brief Drops all NDEF records from the reader's internal memory
/// @brief Note: this is NOT an official SCANNDY command!
///
void ProcessClearCache()
{
#ifdef SERIAL_RECEIVE_DEBUG
   READER_DEBUGPRINT.println("CLEAR CACHE");
#endif
   if (ndef_message->getRecordCount() > 0)
   {
      ndef_message->dropAllRecords();
   }
}

///
/// @brief force the reader to scan just the ONCE
/// @param query received query string
/// @param length number of characters in received query string
///
void ProcessSingleScanUSR(char *query, size_t length)
{
#ifdef SERIAL_RECEIVE_DEBUG
   READER_DEBUGPRINT.println("RFID SCAN USR");
#endif
   _command = PN532_command::ReadCardOnce;
}

///
/// @brief construct a complete NDEF message of discreet NDEF records, and
/// @brief then set the _COMMAND flag to [PublishCacheToCard], which will
/// @brief force the reader to post the message to the NTAG card/sensor
/// @param query
/// @param length
///
void ProcessRfidWriteQuery(char *query, size_t length)
{
   int recordLength = 0;
   int startIndex = 0;

   char *subs = Substring(query, 4 + 1, length - 4);
   std::string search(subs);
   size_t comma = search.find(',');

   //
   // get the starting address and the ndef payload. We pretty much ignore the
   // starting address, although we DO need the NDEF payload!
   //
   char *address = Substring(subs, 1, comma);
   char *ndef = Substring(subs, comma + 2, length - (comma - 1));
   int ndefPayloadLength = length - (comma + 5);

#ifdef SERIAL_RECEIVE_DEBUG
   READER_DEBUGPRINT.print("RFID WRITE-");
   READER_DEBUGPRINT.print(ndef);
   READER_DEBUGPRINT.print("-");
   READER_DEBUGPRINT.print(address);
   READER_DEBUGPRINT.print("-");
   READER_DEBUGPRINT.println(ndefPayloadLength);
#endif

   // go through all characters in the NDEF payload and extract each NDEF record
   uint16_t records = 0;
   uint16_t *ndefIndexes = new uint16_t[24];
   memset(ndefIndexes, 0, 24);

   for (int i = 0; i < (ndefPayloadLength - 3); i++)
   {
      if ((ndef[i] == NDEF_RECORD_HEADER[0]) & (ndef[i + 1] == NDEF_RECORD_HEADER[1]) & (ndef[i + 2] == NDEF_RECORD_HEADER[2]))
      {
         ndefIndexes[records] = (uint16_t)i;
         records++;
      }
   }

   //
   // at this point we have indexes for each of the NDEF records so
   // we can split the query payload string into a series of seperate
   // NDEF records, which can then be added to the NDEF message.
   //
   for (uint16_t i = 0; i < records; i++)
   {
      if (ndefIndexes[i + 1] > 0)
      {
         recordLength = ndefIndexes[i + 1] - (ndefIndexes[i] + 1);
         startIndex = ndefIndexes[i] + 2;
      }
      else
      {
         recordLength = ndefPayloadLength - (ndefIndexes[i] + 1);
         startIndex = ndefIndexes[i] + 2;
      }

      char *record = Substring(ndef, startIndex, recordLength);
      AddNdefRecordToMessage((byte *)record, recordLength);
      free(record);
   }

   // we want to publish the buffered NDEF message
   _command = PN532_command::PublishCacheToCard;

   // and not forgetting!
   free(address);
   free(ndef);
   free(subs);
   delete[] ndefIndexes;
}

///
/// @brief Streams the NDEF contents out over Bluetooth as a series of 16 byte packets
/// @param message_length raw NDEF message payload
/// @param headerdata NDEF meassage header with UUID
///
void PublishResponseToBluetooth(char *pagedata, size_t message_length)
{
   // make sure we don't have any NFC scanning overlaps here
   _readerBusy = true;

   // insert the message identified
   for (int i = 0; i < QUERY_OFFSET_BYTES; i++)
   {
      scomp_ok_response_header[i] = scomp_query_ID[i];
   }

   // set the SCOMP PROTOCOL total TAG payload length
   PAYLOAD_LEGTH[0] = (uint8_t)((message_length & 0xff00) >> 8);
   PAYLOAD_LEGTH[1] = (uint8_t)(message_length & 0x00ff);

   // insert the payload length into the SCOMP PROTOCOL RFID DATA HEADER
   const char *payloadLength = HexStr(PAYLOAD_LEGTH, LENGTH_BYTES, HEX_UPPER_CASE);
   for (int i = 0; i < (int)sizeof(payloadLength); i++)
   {
      scomp_ok_response_header[i + 5] = payloadLength[i];
   }

   // PUBLISH SCANNDY PROTOCOL HEADER TO BLUETOOTH
   txChar.writeValue(scomp_ok_response_header, false);
   crc.update(scomp_ok_response_header, RESPONSE_HEADER_BYTES);
   delayMicroseconds(BLOCK_WAIT_BLE);

   // PUBLISH THE PAYLOAD MESSAGE TO BLUETOOTH
   txChar.writeValue(pagedata, false);
   crc.update(pagedata, message_length);
   delayMicroseconds(BLOCK_WAIT_BLE);

   // publish the final CRC as an array of bytes
   crc.finalizeAsArray(EOR);
   const char *crcValue = HexStr(EOR, FOOTER_BYTES, HEX_UPPER_CASE);
   txChar.writeValue(crcValue, false);
   crc.reset();

   // release the blocker
   _readerBusy = false;
}

// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************

//-------------------------------------------------------------------------------------------------
