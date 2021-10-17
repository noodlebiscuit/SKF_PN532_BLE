#pragma once
/*
 * ============================================================================
 * Manages NDEF formatted NFC payload Records
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

#define NDEF_Record_h

#include "Due.h"
#include <Arduino.h>

#define TNF_EMPTY 0x0
#define TNF_WELL_KNOWN 0x01
#define TNF_MIME_MEDIA 0x02
#define TNF_ABSOLUTE_URI 0x03
#define TNF_EXTERNAL_TYPE 0x04
#define TNF_UNKNOWN 0x05
#define TNF_UNCHANGED 0x06
#define TNF_RESERVED 0x07

class NDEF_Record
{
    public:
        NDEF_Record();
        NDEF_Record(const NDEF_Record& rhs);
        ~NDEF_Record();
        NDEF_Record& operator=(const NDEF_Record& rhs);

        int getEncodedSize();
        void encode(byte *data, bool firstRecord, bool lastRecord);

        unsigned int getIdLength();
        unsigned int getTypeLength();
        int getPayloadLength();

        byte getTnf();
        void getType(byte *type);
        void getPayload(byte *payload);
        void getId(byte *id);

        String getType();
        String getId();

        void setTnf(byte tnf);
        void setType(const byte *type, const unsigned int numBytes);
        void setPayload(const byte *payload, const int numBytes);
        void setId(const byte *id, const unsigned int numBytes);
    
    private:
        byte getTnfByte(bool firstRecord, bool lastRecord);
        byte _tnf; 
        unsigned int _typeLength;
        int _payloadLength;
        unsigned int _idLength;
        byte *_type;
        byte *_payload;
        byte *_id;
};