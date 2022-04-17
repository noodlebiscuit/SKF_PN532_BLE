#pragma once
/*
 * ============================================================================
 * Manages NDEF formatted NFC payload Messages
 * ----------------------------------------------------------------------------
 * 
 * This Class was created by SKF UK Ltd for use with SKF INSIGHT RAIL 
 * It is not intended for public release, primarily as its functionality is 
 * based on private (internal SKF testing) of INSIGHT sensor commissioning
 * 
 * This library is based work produced by Elechouse back in 2016
 * 
 * REVISION 1.0 April 2020
 * Alex Pinkerton
 * 
 * ============================================================================
 */

#define NDEF_Message_h

#include "NDEF_Record.h"

#define MAX_NDEF_RECORDS 12

class NDEF_Message
{
    public:
        NDEF_Message(void);
        NDEF_Message(const byte *data, const int numBytes);
        NDEF_Message(const NDEF_Message& rhs);
        ~NDEF_Message();
        NDEF_Message& operator=(const NDEF_Message& rhs);

        int getEncodedSize(); // need so we can pass array to encode
        void encode(byte *data);

        boolean addRecord(NDEF_Record& record);
        void addBinaryRecord(byte *message, int messageSize);
        void addMimeMediaRecord(String mimeType, String payload);
        void addMimeMediaRecord(String mimeType, byte *payload, int payloadLength);
        void addTextRecord(String text);
        void addTextRecord(String text, String encoding);
        void addUriRecord(String uri);
        void addEmptyRecord();
        void dropAllRecords();
        void setRecord(int index, NDEF_Record& record);

        unsigned int getRecordCount();
        NDEF_Record getRecord(int index);
        NDEF_Record operator[](int index);

        void print();
    private:
        NDEF_Record _records[MAX_NDEF_RECORDS];
        unsigned int _recordCount;
};