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
#include "NDEF_Message.h"

// ============================================================================

/// <summary>
/// Default constructor
/// </summary>
NDEF_Message::NDEF_Message(void)
{
    _recordCount = 0;
}

/// <summary>
/// Overload constructor
/// </summary>
/// <param name="data">byte array containing the complete NDEF message</param>
/// <param name="numBytes">number of bytes to read from the NDEF message</param>
NDEF_Message::NDEF_Message(const byte *data, const int numBytes)
{
    _recordCount = 0;
    int index = 0;

    while (index <= numBytes)
    {
        //
        // decode the TYPE NAME FORMAT. Details of this block are:
        //
        //          Bit 7     6       5       4       3       2       1       0
        //         ------  ------  ------  ------  ------  ------  ------  ------
        // BYTE 0: [ MB ]  [ ME ]  [ CF ]  [ SR ]  [ IL ]  [        TNF         ]
        //         ------  ------  ------  ------  ------  ------  ------  ------
        // BYTE 1: [ TYPE LENGTH                                                ]
        // BYTE 2: [ PAYLOAD LENGTH                                             ]
        // BYTE 3: [ ID LENGTH                                                  ]
        // BYTE 4: [ RECORD TYPE                                                ]
        // BYTE 5: [ ID                                                         ]
        // BYTE 6: [ PAYLOAD                                                    ]
        //         ------  ------  ------  ------  ------  ------  ------  ------
        //
        //  MB: Message BEGIN - indicates if this is the start of an NDEF message
        //
        //  ME: Message END   - indicates if this is the last record in the message
        //
        //  CF: CHUNK flag    - indicates if this is the first record chunk or a
        //                      middle record chunk)
        //
        //  SR: SHORT RECORD  - The SR flag is set to one if the PAYLOAD LENGTH
        //                      field is 1 byte (8 bits/0-255) or less. This allows
        //                      for more compact records
        //
        //  IL: LENGTH        - indicates if the ID Length Field is preent or not.
        //                      If this is set to 0, then the ID Length Field is
        //                      ommitted in the record
        //
        byte tnf_byte = data[index];

        // get the three relevant bit fields
        bool me = (tnf_byte & 0x40) != 0;
        bool sr = (tnf_byte & 0x10) != 0;
        bool il = (tnf_byte & 0x8) != 0;

        // get the three bit TNF format
        byte tnf = (tnf_byte & 0x7);

        // create a new NDEF record and apply the TNF
        NDEF_Record record = NDEF_Record();
        record.setTnf(tnf);

        // get the number of bytes as stated in the header
        index++;
        int typeLength = data[index];

        // check the SR to determine what record length to use
        int payloadLength = 0;
        if (sr)
        {
            index++;
            payloadLength = data[index];
        }
        else
        {
            payloadLength =
                ((0xFF & data[++index]) << 24) | ((0xFF & data[++index]) << 16) | ((0xFF & data[++index]) << 8) | (0xFF & data[++index]);
        }

        int idLength = 0;
        if (il)
        {
            index++;
            idLength = data[index];
        }

        // apply the type to our new NDEF record
        index++;
        record.setType(&data[index], typeLength);
        index += typeLength;

        if (il)
        {
            record.setId(&data[index], idLength);
            index += idLength;
        }

        // add the payload to this NDEF record
        record.setPayload(&data[index], payloadLength);

        // append the new NDEF record to the NDEF message
        addRecord(record);

        // increment the position index
        index += payloadLength;

        // is this the MESSAGE END? If it is then exit here
        if (me)
        {
            break;
        }
    }
}

/// <summary>
/// Overload constructor
/// </summary>
/// <param name="rhs">formatted NDEF message payload</param>
NDEF_Message::NDEF_Message(const NDEF_Message &rhs)
{

    _recordCount = rhs._recordCount;
    for (int i = 0; i < _recordCount; i++)
    {
        _records[i] = rhs._records[i];
    }
}

/// <summary>
/// Release resources
/// </summary>
NDEF_Message::~NDEF_Message()
{
}

/// <summary>
/// Overload (copy) constructor
/// </summary>
NDEF_Message &NDEF_Message::operator=(const NDEF_Message &rhs)
{
    if (this != &rhs)
    {
        for (int i = 0; i < _recordCount; i++)
        {
            _records[i] = NDEF_Record();
        }

        _recordCount = rhs._recordCount;
        for (int i = 0; i < _recordCount; i++)
        {
            _records[i] = rhs._records[i];
        }
    }
    return *this;
}

// ============================================================================

/// <summary>
/// How many NDEF records in the current message?
/// </summary>
unsigned int NDEF_Message::getRecordCount()
{
    return _recordCount;
}

/// <summary>
/// What is the serialised size of the current message in bytes?
/// </summary>
int NDEF_Message::getEncodedSize()
{
    int size = 0;
    for (int i = 0; i < _recordCount; i++)
    {
        size += _records[i].getEncodedSize();
    }
    return size;
}

/// <summary>
/// Serialise the current message
/// </summary>
void NDEF_Message::encode(uint8_t *data)
{
    uint8_t *data_ptr = &data[0];
    for (int i = 0; i < _recordCount; i++)
    {
        _records[i].encode(data_ptr, i == 0, (i + 1) == _recordCount);
        data_ptr += _records[i].getEncodedSize();
    }
}

/// <summary>
/// Add a new NDEF record to the current message
/// </summary>
boolean NDEF_Message::addRecord(NDEF_Record &record)
{
    if (_recordCount < MAX_NDEF_RECORDS)
    {
        _records[_recordCount] = record;
        _recordCount++;
        return true;
    }
    else
    {
        Serial.println(F("PAGE ERROR"));
        return false;
    }
}

/// <summary>
/// Add a new NDEF Media record reference to the current message
/// </summary>
void NDEF_Message::addMimeMediaRecord(String mimeType, String payload)
{

    byte payloadBytes[payload.length() + 1];
    payload.getBytes(payloadBytes, sizeof(payloadBytes));

    addMimeMediaRecord(mimeType, payloadBytes, payload.length());
}

/// <summary>
/// Add a new NDEF Media record reference to the current message
/// </summary>
void NDEF_Message::addMimeMediaRecord(String mimeType, uint8_t *payload, int payloadLength)
{
    NDEF_Record r = NDEF_Record();
    r.setTnf(TNF_MIME_MEDIA);

    byte type[mimeType.length() + 1];
    mimeType.getBytes(type, sizeof(type));
    r.setType(type, mimeType.length());

    r.setPayload(payload, payloadLength);

    addRecord(r);
}

/// <summary>
/// Add a new NDEF Text record to the current message with EN encoding
/// </summary>
void NDEF_Message::addTextRecord(String text)
{
    addTextRecord(text, "en");
}

/// <summary>
/// Add a new NDEF Text record to the current message with configurable encoding
/// </summary>
void NDEF_Message::addTextRecord(String text, String encoding)
{
    NDEF_Record r = NDEF_Record();
    r.setTnf(TNF_WELL_KNOWN);

    uint8_t RTD_TEXT[1] = {0x54}; // TODO this should be a constant or preprocessor
    r.setType(RTD_TEXT, sizeof(RTD_TEXT));

    // X is a placeholder for encoding length
    // TODO is it more efficient to build w/o string concatenation?
    String payloadString = "X" + encoding + text;

    byte payload[payloadString.length() + 1];
    payloadString.getBytes(payload, sizeof(payload));

    // replace X with the real encoding length
    payload[0] = encoding.length();

    r.setPayload(payload, payloadString.length());

    addRecord(r);
}

/// <summary>
/// Add a new NDEF URI link record reference to the current message
/// </summary>
void NDEF_Message::addUriRecord(String uri)
{
    NDEF_Record *r = new NDEF_Record();
    r->setTnf(TNF_WELL_KNOWN);

    uint8_t RTD_URI[1] = {0x55}; // TODO this should be a constant or preprocessor
    r->setType(RTD_URI, sizeof(RTD_URI));

    // X is a placeholder for identifier code
    String payloadString = "X" + uri;

    byte payload[payloadString.length() + 1];
    payloadString.getBytes(payload, sizeof(payload));

    // add identifier code 0x0, meaning no prefix substitution
    payload[0] = 0x0;

    r->setPayload(payload, payloadString.length());

    addRecord(*r);
    delete (r);
}

/// <summary>
/// Add an empty NDEF record to the message
/// </summary>
void NDEF_Message::addEmptyRecord()
{
    NDEF_Record *r = new NDEF_Record();
    r->setTnf(TNF_EMPTY);
    addRecord(*r);
    delete (r);
}

/// <summary>
/// Drop all records from the current NDEF message
/// </summary>
void NDEF_Message::dropAllRecords()
{
    for (int i = 0; i < _recordCount - 1; i++)
    {
        _records[i] = NDEF_Record();
    }
    _recordCount = 0;
}

/// <summary>
/// Overwrite a specific NDEF record in the message
/// </summary>
void NDEF_Message::setRecord(int index, NDEF_Record &record)
{
    if (index > -1 && index < _recordCount)
    {
        _records[index] = record;
    }
}

/// <summary>
/// Returns a specific NDEF record reference from the current message
/// </summary>
NDEF_Record NDEF_Message::getRecord(int index)
{
    if (index > -1 && index < _recordCount)
    {
        return _records[index];
    }
    else
    {
        return NDEF_Record(); // would rather return NULL
    }
}

/// <summary>
/// Returns a specific NDEF record reference from the current message
/// </summary>
NDEF_Record NDEF_Message::operator[](int index)
{
    return getRecord(index);
}
