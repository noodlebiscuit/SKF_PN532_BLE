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

#include "NDEF_Record.h"

/// <summary>
/// Default constructor (null record)
/// </summary>
NDEF_Record::NDEF_Record()
{
    _tnf = 0;
    _typeLength = 0;
    _payloadLength = 0;
    _idLength = 0;
    _type = (byte *)NULL;
    _payload = (byte *)NULL;
    _id = (byte *)NULL;
}

/// <summary>
/// Overload constructor
/// </summary>
/// <param name="rhs">populated NDEF record</param>
NDEF_Record::NDEF_Record(const NDEF_Record& rhs)
{
    _tnf = rhs._tnf;
    _typeLength = rhs._typeLength;
    _payloadLength = rhs._payloadLength;
    _idLength = rhs._idLength;
    _type = (byte *)NULL;
    _payload = (byte *)NULL;
    _id = (byte *)NULL;

    if (_typeLength)
    {
        _type = (byte*)malloc(_typeLength);
        memcpy(_type, rhs._type, _typeLength);
    }

    if (_payloadLength)
    {
        _payload = (byte*)malloc(_payloadLength);
        memcpy(_payload, rhs._payload, _payloadLength);
    }

    if (_idLength)
    {
        _id = (byte*)malloc(_idLength);
        memcpy(_id, rhs._id, _idLength);
    }
}

/// <summary>
/// Release resources
/// </summary>
NDEF_Record::~NDEF_Record()
{
    if (_typeLength)
    {
        free(_type);
    }
    if (_payloadLength)
    {
        free(_payload);
    }
    if (_idLength)
    {
        free(_id);
    }
}

/// <summary>
/// Overload constructor (copy contents of existing record)
/// </summary>
/// <param name="rhs">populated NDEF record</param>
NDEF_Record& NDEF_Record::operator=(const NDEF_Record& rhs)
{
    if (this != &rhs)
    {
        // free existing
        if (_typeLength)
        {
            free(_type);
        }
        if (_payloadLength)
        {
            free(_payload);
        }
        if (_idLength)
        {
            free(_id);
        }

        _tnf = rhs._tnf;
        _typeLength = rhs._typeLength;
        _payloadLength = rhs._payloadLength;
        _idLength = rhs._idLength;

        if (_typeLength)
        {
            _type = (byte*)malloc(_typeLength);
            memcpy(_type, rhs._type, _typeLength);
        }
        if (_payloadLength)
        {
            _payload = (byte*)malloc(_payloadLength);
            memcpy(_payload, rhs._payload, _payloadLength);
        }
        if (_idLength)
        {
            _id = (byte*)malloc(_idLength);
            memcpy(_id, rhs._id, _idLength);
        }
    }
    return *this;
}

/// <summary>
/// Get the size of this record in bytes
/// </summary>
int NDEF_Record::getEncodedSize()
{
    int size = 2; // tnf + typeLength
    if (_payloadLength > 0xFF)
    {
        size += 4;
    }
    else
    {
        size += 1;
    }

    if (_idLength)
    {
        size += 1;
    }

    size += (_typeLength + _payloadLength + _idLength);
    return size;
}

/// <summary>
/// Encode the contents of this NDEF record into NDEF format
/// </summary>
/// <param name="data">points to the record buffer</param>
/// <param name="firstRecord">is this the first record in an NDEF message?</param>
/// <param name="firstRecord">is this the last record in an NDEF message?</param>
void NDEF_Record::encode(byte *data, bool firstRecord, bool lastRecord)
{
    uint8_t* data_ptr = &data[0];

    *data_ptr = getTnfByte(firstRecord, lastRecord);
    data_ptr += 1;

    *data_ptr = _typeLength;
    data_ptr += 1;

    if (_payloadLength <= 0xFF) {  // short record
        *data_ptr = _payloadLength;
        data_ptr += 1;
    } else { // long format
        // 4 bytes but we store length as an int
        data_ptr[0] = 0x0; // (_payloadLength >> 24) & 0xFF;
        data_ptr[1] = 0x0; // (_payloadLength >> 16) & 0xFF;
        data_ptr[2] = (_payloadLength >> 8) & 0xFF;
        data_ptr[3] = _payloadLength & 0xFF;
        data_ptr += 4;
    }

    if (_idLength)
    {
        *data_ptr = _idLength;
        data_ptr += 1;
    }

    //Serial.println(2);
    memcpy(data_ptr, _type, _typeLength);
    data_ptr += _typeLength;

    memcpy(data_ptr, _payload, _payloadLength);
    data_ptr += _payloadLength;

    if (_idLength)
    {
        memcpy(data_ptr, _id, _idLength);
        data_ptr += _idLength;
    }
}

/// <summary>
/// Populate and return the TNF (Type Name Format) byte
/// </summary>
/// <param name="firstRecord">is this the first record in an NDEF message?</param>
/// <param name="firstRecord">is this the last record in an NDEF message?</param>
/// <returns>populated TNF byte</returns>
byte NDEF_Record::getTnfByte(bool firstRecord, bool lastRecord)
{
    int value = _tnf;

    if (firstRecord) { // mb
        value = value | 0x80;
    }

    if (lastRecord) { //
        value = value | 0x40;
    }

    if (_payloadLength <= 0xFF) {
        value = value | 0x10;
    }

    if (_idLength) {
        value = value | 0x8;
    }

    return value;
}

/// <summary>
/// Return the TNF (Type Name Format) byte
/// </summary>
/// <returns>populated TNF byte</returns>
byte NDEF_Record::getTnf()
{
    return _tnf;
}

/// <summary>
/// Populate the TNF (Type Name Format) byte
/// </summary>
/// <param name="tnf">populated TNF byte</param>
void NDEF_Record::setTnf(byte tnf)
{
    _tnf = tnf;
}

/// <summary>
/// Gets the TNF (Type Name Format) byte
/// </summary>
/// <returns>populated TNF byte</returns>
unsigned int NDEF_Record::getTypeLength()
{
    return _typeLength;
}

/// <summary>
/// Gets the byte length of the record payload field
/// </summary>
/// <returns>size in bytes</returns>
int NDEF_Record::getPayloadLength()
{
    return _payloadLength;
}

/// <summary>
/// Gets the byte length of the record ID field
/// </summary>
/// <returns>size in bytes</returns>
unsigned int NDEF_Record::getIdLength()
{
    return _idLength;
}

/// <summary>
/// Gets the NDEF record type as a string
/// </summary>
/// <returns>string representing the record type</returns>
String NDEF_Record::getType()
{
    char type[_typeLength + 1];
    memcpy(type, _type, _typeLength);
    type[_typeLength] = '\0'; // null terminate
    return String(type);
}

/// <summary>
/// Return the TNF (Type Name Format) byte
/// Note: this assumes that the caller has set "type" correctly
/// </summary>
/// <param name="type">pointer to populated data array</param>
void NDEF_Record::getType(uint8_t* type)
{
    memcpy(type, _type, _typeLength);
}

/// <summary>
/// Sets the TYPE for this NDEF record
/// Note: this assumes that the caller has set "type" correctly
/// </summary>
/// <param name="type">pointer to n byte TYPE array</param>
/// <param name="numBytes">number of bytes within the TYPE array</param>
void NDEF_Record::setType(const byte * type, const unsigned int numBytes)
{
    if(_typeLength)
    {
        free(_type);
    }

    _type = (uint8_t*)malloc(numBytes);
    memcpy(_type, type, numBytes);
    _typeLength = numBytes;
}

/// <summary>
/// Return the payload by pointer reference
/// Note: this assumes that the caller has set "type" correctly
/// </summary>
/// <param name="type">pointer to populated data array</param>
void NDEF_Record::getPayload(byte *payload)
{
    memcpy(payload, _payload, _payloadLength);
}

/// <summary>
/// Sets the actual PAYLOAD for this NDEF record
/// </summary>
/// <param name="payload">pointer to n byte PAYLOAD array</param>
/// <param name="numBytes">number of bytes within the PAYLOAD array</param>
void NDEF_Record::setPayload(const byte * payload, const int numBytes)
{
    if (_payloadLength)
    {
        free(_payload);
    }

    _payload = (byte*)malloc(numBytes);
    memcpy(_payload, payload, numBytes);
    _payloadLength = numBytes;
}

/// <summary>
/// Gets the ID of the current record
/// </summary>
/// <returns>formatted string</returns>
String NDEF_Record::getId()
{
    char id[_idLength + 1];
    memcpy(id, _id, _idLength);
    id[_idLength] = '\0'; // null terminate
    return String(id);
}

/// <summary>
/// Gets the ID of the current record
/// </summary>
/// <param name="id">pointer to ID byte array</param>
void NDEF_Record::getId(byte *id)
{
    memcpy(id, _id, _idLength);
}

/// <summary>
/// Sets the ID for this NDEF record
/// </summary>
/// <param name="type">pointer to n byte ID array</param>
/// <param name="numBytes">number of bytes within the ID array</param>
void NDEF_Record::setId(const byte * id, const unsigned int numBytes)
{
    if (_idLength)
    {
        free(_id);
    }
    _id = (byte*)malloc(numBytes);
    memcpy(_id, id, numBytes);
    _idLength = numBytes;
}
