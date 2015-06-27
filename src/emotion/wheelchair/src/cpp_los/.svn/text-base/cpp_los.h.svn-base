//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 8.4.2007
// Modify by SAR LAMG
// Last modif 09.7.2009
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef C_LOS_HH
#define C_LOS_HH
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <inttypes.h>
#include <string>
#include <sstream>
#include <errno.h>
#include <stdexcept>
#include "connection.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef NULL
#define NULL (void *)0
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Flag for compilation, tells if little or big endian is to be used.
// In case you want to use big endian comment the upper part out and
// uncomment the lower part.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef LOS_IS_LITTLE_ENDIAN
#define LOS_IS_LITTLE_ENDIAN
#endif
//#ifdef LOS_IS_LITTLE_ENDIAN
//#undef LOS_IS_LITTLE_ENDIAN
//#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// data types known to LOS.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define LOS_VOID                0
#define LOS_BOOLEAN             1
#define LOS_BOOLEAN_ARRAY       2
#define LOS_INT8                3
#define LOS_INT8_ARRAY          4
#define LOS_INT16               5
#define LOS_INT16_ARRAY         6
#define LOS_INT32               7
#define LOS_INT32_ARRAY         8
#define LOS_INT64               9
#define LOS_INT64_ARRAY         10
#define LOS_FLOAT32             11
#define LOS_FLOAT32_ARRAY       12
#define LOS_FLOAT64             13
#define LOS_FLOAT64_ARRAY       14
#define LOS_STRING              15
#define LOS_STRING_ARRAY        16
#define LOS_ARRAY               17
#define LOS_CALL                18
#define LOS_CALL_RESULT         19
#define LOS_CALL_EXCEPTION      20
#define LOS_STRUCT     21

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Datatypes required for serialisation and de-serialisation.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
union Type8bit
{
  int8_t Int8Type;
  uint8_t UInt8Type;
};
//-------------------------------------------------------------------
union Type16bit
{
  int16_t Int16Type;
  uint16_t UInt16Type;
  uint8_t ByteString[2];
};
//-------------------------------------------------------------------
union Type32bit
{
  int32_t Int32Type;
  uint32_t UInt32Type;
  float Float32Type;
  uint8_t ByteString[4];
};
//-------------------------------------------------------------------
union Type64bit
{
  int64_t Int64Type;
  uint64_t UInt64Type;
  double Float64Type;
  uint8_t ByteString[8];
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The class 'CppLos' implements the Lightweighted Object Streaming
// protocol (LOS). The implementation includes the low-level com-
// munication over a TCP socket plus buffer handling.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class CppLos
{
  protected:
//-------------------------------------------------------------------
// Buffer- and socket-related private attributes.
//-------------------------------------------------------------------
    Connection *p_socket_;
    uint32_t stream_buffer_size_;
    uint8_t *stream_buffer_;
    uint32_t read_index_;
    uint32_t write_index_;
//-------------------------------------------------------------------
// Private attributes for the LOS protocol.
//-------------------------------------------------------------------
    Type64bit common_temp_64_;
    Type32bit common_temp_32_;
    Type16bit common_temp_16_;
    Type8bit common_temp_8_;
//-------------------------------------------------------------------
// Private methods for the LOS protocol.
//-------------------------------------------------------------------
    uint8_t view1Byte(void);
    void read1Byte(void);
    void write1Byte(void);
    void read2Bytes(void);
    void write2Bytes(void);
    void read4Bytes(void);
    void write4Bytes(void);
    void read8Bytes(void);
    void write8Bytes(void);
    void deserialiseString(std::string &data);
    void serialiseString(std::string &data);
//-------------------------------------------------------------------
// General private methods.
//-------------------------------------------------------------------
    void cleanup(void);
    void configuration(std::string host, int port, double timeout);
  public:
//-------------------------------------------------------------------
// General public methods.
//-------------------------------------------------------------------
    CppLos();
    ~CppLos();
    void connect(std::string host, int port, double timeout);
//-------------------------------------------------------------------
// Buffer-related public methods.
//-------------------------------------------------------------------
    void init(void);
    void flush(void);
//-------------------------------------------------------------------
// Public methods for the LOS protocol.
//-------------------------------------------------------------------
    uint8_t getTypeID(void);
    bool readBoolean(void);
    void writeBoolean(bool data);
    void readBooleanArray(bool data[], uint32_t &length);
    void writeBooleanArray(bool data[], uint32_t length);
    int8_t readInt8(void);
    void writeInt8(int8_t data);
    void readInt8Array(int8_t data[], uint32_t &length);
    void writeInt8Array(int8_t data[], uint32_t length);
    int16_t readInt16(void);
    void writeInt16(int16_t data);
    void readInt16Array(int16_t data[], uint32_t &length);
    void writeInt16Array(int16_t data[], uint32_t length);
    int32_t readInt32(void);
    void writeInt32(int32_t data);
    void readInt32Array(int32_t data[], uint32_t &length);
    void writeInt32Array(int32_t data[], uint32_t length);
    int64_t readInt64(void);
    void writeInt64(int64_t data);
    void readInt64Array(int64_t data[], uint32_t &length);
    void writeInt64Array(int64_t data[], uint32_t length);
    float readFloat32(void);
    void writeFloat32(float data);
    void readFloat32Array(float data[], uint32_t &length);
    void writeFloat32Array(float data[], uint32_t length);
    double readFloat64(void);
    void writeFloat64(double data);
    void readFloat64Array(double data[], uint32_t &length);
    void writeFloat64Array(double data[], uint32_t length);
    void readString(std::string &data);
    void writeString(std::string &data);
    void readStringArray(std::string data[], uint32_t &length);
    void writeStringArray(std::string data[], uint32_t length);
    uint32_t readArray(void);
    void writeArray(uint32_t length);
    uint32_t readCall(std::string &name);
    void writeCall(std::string &name, uint32_t length);
    bool readCallResult(void);
    void writeCallResult(bool b_further_arguments);
    bool readCallException(std::string &name, std::string &Msg);
    void writeCallException(std::string &name, std::string &Msg, bool b_further_arguments);
    uint32_t readStruct(void);
    void writeStruct(uint32_t length);
    void writeStructKey(std::string &key);
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};
#endif
