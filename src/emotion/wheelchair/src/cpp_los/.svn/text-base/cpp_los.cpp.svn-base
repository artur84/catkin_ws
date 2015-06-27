//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 8.4.2007
// Modify by SAR, LAMG
// Last modif 09.7.2009
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "cpp_los.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Private methods for the LOS protocol.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads one byte from the buffer and returns it. The data is NOT
// consumed. If the buffer holds an insufficient amount of data, the
// TCP socket is read.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint8_t CppLos::view1Byte(void)
{
  uint32_t u;
  // Check if the buffer holds enough data.
  if ((write_index_ - read_index_) < 1)
  {
    // Defragment the buffer before storing new data.
    if (read_index_ > 0)
    {
      u = write_index_ - read_index_;
      for (write_index_ = 0; write_index_ < u; write_index_++)
        stream_buffer_[write_index_] = stream_buffer_[read_index_ + write_index_];
      read_index_ = 0;
    }
    bool keep_trying = true;
    while (keep_trying)
    {
      u = stream_buffer_size_ - write_index_;
      if (u == 0)
        throw std::runtime_error("CppLos::view1Byte(): Error.Buffer.Full");
      u = p_socket_->read(&stream_buffer_[write_index_], u);
      write_index_ = write_index_ + u;

      if (u==0) //Timeout: read returns 0 if timeout
      {
        throw std::runtime_error("CppLos::view1Byte(): Error.Read.Timeout");
      }

      // If u !=0 there was some data in the socket, maybe not enough, but we can check; right?
      if ((write_index_ - read_index_) >= 1)
      {
        // If we have enough data after reading just break the loop
        keep_trying = false;
      }
    }
  }
  // Read and return one byte, but leave it in the buffer.
  return stream_buffer_[read_index_];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads one byte from the buffer and stores it in the member attri-
// bute 'common_temp_8_'. If the buffer holds an insufficient amount of
// data, the TCP socket is read.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::read1Byte(void)
{
  uint32_t u;

// Check if the buffer holds enough data.
  if ((write_index_ - read_index_) < 1)
  {
// Defragment the buffer before storing new data.
    if (read_index_ > 0)
    {
      u = write_index_ - read_index_;
      for (write_index_ = 0; write_index_ < u; write_index_++)
        stream_buffer_[write_index_] = stream_buffer_[read_index_ + write_index_];
      read_index_ = 0;
    }
    bool keep_trying = true;
    while (keep_trying)
    {
      u = stream_buffer_size_ - write_index_;
      if (u == 0)
        throw std::runtime_error("CppLos::read1Byte(): Error.Buffer.Full");
      u = p_socket_->read(&stream_buffer_[write_index_], u);
      write_index_ = write_index_ + u;

      if (u==0) //Timeout: read returns 0 if timeout
      {
        throw std::runtime_error("CppLos::read1Byte(): Error.Read.Timeout");
      }

      // If u !=0 there was some data in the socket, maybe not enough, but we can check; right?
      if ((write_index_ - read_index_) >= 1)
      {
        // If we have enough data after reading just break the loop
        keep_trying = false;
      }
    }
  }
// Transfer one byte from the buffer to the member attribute.
  common_temp_8_.UInt8Type = stream_buffer_[read_index_++];
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes one byte from the member attribute 'common_temp_8_' to the
// buffer. If the buffer provides insufficient free space, write the
// buffer to the TCP socket.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::write1Byte(void)
{
  uint32_t u;

// Check if the buffer provides enough space.
  if (write_index_ >= stream_buffer_size_)
  {
// If not, write the buffer to the TCP socket.
    u = write_index_ - read_index_;
    u = p_socket_->write(&stream_buffer_[read_index_], u);
// Update the buffer management data.
    read_index_ = read_index_ + u;
// Defragment the buffer after reading data.
    if (read_index_ > 0)
    {
      u = write_index_ - read_index_;
      for (write_index_ = 0; write_index_ < u; write_index_++)
        stream_buffer_[write_index_] = stream_buffer_[read_index_ + write_index_];
      read_index_ = 0;
    }
// If there's still too little space after writing, end.
    if (write_index_ >= stream_buffer_size_)
      throw std::runtime_error("CppLos::write1Byte(): Error.Write.Timeout");
  }
// Transfer one byte from the member attribute to the buffer.
  stream_buffer_[write_index_++] = common_temp_8_.UInt8Type;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads two bytes from the buffer and stores them in the member att-
// ribute 'common_temp_16_'. If the buffer holds an insufficient amount
// of data, the TCP socket is read.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::read2Bytes(void)
{
  uint32_t u;

// Check if the buffer holds enough data.
  if ((write_index_ - read_index_) < 2)
  {
// Defragment the buffer before storing new data.
    if (read_index_ > 0)
    {
      u = write_index_ - read_index_;
      for (write_index_ = 0; write_index_ < u; write_index_++)
        stream_buffer_[write_index_] = stream_buffer_[read_index_ + write_index_];
      read_index_ = 0;
    }
    bool keep_trying = true;
    while (keep_trying)
    {
      u = stream_buffer_size_ - write_index_;
      if (u == 0)
        throw std::runtime_error("CppLos::read2Bytes(): Error.Buffer.Full");
      u = p_socket_->read(&stream_buffer_[write_index_], u);
      write_index_ = write_index_ + u;

      if (u==0) //Timeout: read returns 0 if timeout
      {
        throw std::runtime_error("CppLos::read2Bytes(): Error.Read.Timeout");
      }

      // If u !=0 there was some data in the socket, maybe not enough, but we can check; right?
      if ((write_index_ - read_index_) >= 2)
      {
        // If we have enough data after reading just break the loop
        keep_trying = false;
      }
    }
// Transfer two bytes from the buffer to the member attribute.
  }
#ifdef LOS_IS_LITTLE_ENDIAN
  common_temp_16_.ByteString[0] = stream_buffer_[read_index_++];
  common_temp_16_.ByteString[1] = stream_buffer_[read_index_++];
#else
  common_temp_16_.ByteString[1] = stream_buffer_[read_index_++];
  common_temp_16_.ByteString[0] = stream_buffer_[read_index_++];
#endif
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes two bytes from the member attribute 'common_temp_16_' to the
// buffer. If the buffer provides insufficient free space, write the
// buffer to the TCP socket.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::write2Bytes(void)
{
  uint32_t u;

// Check if the buffer provides enough space.
  if ((write_index_ + 2) > stream_buffer_size_)
  {
// If not, write the buffer to the TCP socket.
    u = write_index_ - read_index_;
    u = p_socket_->write(&stream_buffer_[read_index_], u);
// Update the buffer management data.
    read_index_ = read_index_ + u;
// Defragment the buffer after reading data.
    if (read_index_ > 0)
    {
      u = write_index_ - read_index_;
      for (write_index_ = 0; write_index_ < u; write_index_++)
        stream_buffer_[write_index_] = stream_buffer_[read_index_ + write_index_];
      read_index_ = 0;
    }
// If there's still too little space after writing, end.
    if ((write_index_ + 2) > stream_buffer_size_)
      throw std::runtime_error("CppLos::write2Bytes(): Error.Write.Timeout");
  }
// Transfer two bytes from the member attribute to the buffer.
#ifdef LOS_IS_LITTLE_ENDIAN
  stream_buffer_[write_index_++] = common_temp_16_.ByteString[0];
  stream_buffer_[write_index_++] = common_temp_16_.ByteString[1];
#else
  stream_buffer_[write_index_++] = common_temp_16_.ByteString[1];
  stream_buffer_[write_index_++] = common_temp_16_.ByteString[0];
#endif
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads four bytes from the buffer and stores them in the member
// attribute 'common_temp_32_'. If the buffer holds an insufficient
// amount of data, the TCP socket is read.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::read4Bytes(void)
{
  uint32_t u;

  // Check if the buffer holds enough data.
  if ((write_index_ - read_index_) < 4)
  {
    // Defragment the buffer before storing new data.
    if (read_index_ > 0)
    {
      u = write_index_ - read_index_;
      for (write_index_ = 0; write_index_ < u; write_index_++)
        stream_buffer_[write_index_] = stream_buffer_[read_index_ + write_index_];
      read_index_ = 0;
    }
    bool keep_trying = true;
    while (keep_trying)
    {
      u = stream_buffer_size_ - write_index_;
      if (u == 0)
        throw std::runtime_error("CppLos::read4Bytes(): Error.Buffer.Full");
      u = p_socket_->read(&stream_buffer_[write_index_], u);
      write_index_ = write_index_ + u;

      if (u==0) //Timeout: read returns 0 if timeout
      {
        throw std::runtime_error("CppLos::read4Bytes(): Error.Read.Timeout");
      }

      // If u !=0 there was some data in the socket, maybe not enough, but we can check; right?
      if ((write_index_ - read_index_) >= 4)
      {
        // If we have enough data after reading just break the loop
        keep_trying = false;
      }
    }
  }
// Transfer four bytes from the buffer to the member attribute.
#ifdef LOS_IS_LITTLE_ENDIAN
  common_temp_32_.ByteString[0] = stream_buffer_[read_index_++];
  common_temp_32_.ByteString[1] = stream_buffer_[read_index_++];
  common_temp_32_.ByteString[2] = stream_buffer_[read_index_++];
  common_temp_32_.ByteString[3] = stream_buffer_[read_index_++];
#else
  common_temp_32_.ByteString[3] = stream_buffer_[read_index_++];
  common_temp_32_.ByteString[2] = stream_buffer_[read_index_++];
  common_temp_32_.ByteString[1] = stream_buffer_[read_index_++];
  common_temp_32_.ByteString[0] = stream_buffer_[read_index_++];
#endif
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes four bytes from the member attribute 'common_temp_32_' to the
// buffer. If the buffer provides insufficient free space, write the
// buffer to the socket.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::write4Bytes(void)
{
  uint32_t u;

// Check if the buffer provides enough space.
  if ((write_index_ + 4) > stream_buffer_size_)
  {
// If not, write the buffer to the TCP socket.
    u = write_index_ - read_index_;
    u = p_socket_->write(&stream_buffer_[read_index_], u);
// Update the buffer management data.
    read_index_ = read_index_ + u;
// Defragment the buffer after reading data.
    if (read_index_ > 0)
    {
      u = write_index_ - read_index_;
      for (write_index_ = 0; write_index_ < u; write_index_++)
        stream_buffer_[write_index_] = stream_buffer_[read_index_ + write_index_];
      read_index_ = 0;
    }
// If there's still too little space after writing, end.
    if ((write_index_ + 4) > stream_buffer_size_)
      throw std::runtime_error("CppLos::write4Bytes(): Error.Write.Timeout");
  }
// Transfer four bytes from the member attribute to the buffer.
#ifdef LOS_IS_LITTLE_ENDIAN
  stream_buffer_[write_index_++] = common_temp_32_.ByteString[0];
  stream_buffer_[write_index_++] = common_temp_32_.ByteString[1];
  stream_buffer_[write_index_++] = common_temp_32_.ByteString[2];
  stream_buffer_[write_index_++] = common_temp_32_.ByteString[3];
#else
  stream_buffer_[write_index_++] = common_temp_32_.ByteString[3];
  stream_buffer_[write_index_++] = common_temp_32_.ByteString[2];
  stream_buffer_[write_index_++] = common_temp_32_.ByteString[1];
  stream_buffer_[write_index_++] = common_temp_32_.ByteString[0];
#endif
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads eight bytes from the buffer and stores them in the member
// attribute 'common_temp_64_'. If the buffer holds an insufficient
// amount of data, the TCP socket is read.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::read8Bytes(void)
{
  uint32_t u;

// Check if the buffer holds enough data.
  if ((write_index_ - read_index_) < 8)
  {
// Defragment the buffer before storing new data.
    if (read_index_ > 0)
    {
      u = write_index_ - read_index_;
      for (write_index_ = 0; write_index_ < u; write_index_++)
        stream_buffer_[write_index_] = stream_buffer_[read_index_ + write_index_];
      read_index_ = 0;
    }

    bool keep_trying = true;
    while (keep_trying)
    {
      u = stream_buffer_size_ - write_index_;
      if (u == 0)
        throw std::runtime_error("CppLos::read8Bytes(): Error.Buffer.Full");
      u = p_socket_->read(&stream_buffer_[write_index_], u);
      write_index_ = write_index_ + u;

      if (u==0) //Timeout: read returns 0 if timeout
      {
        throw std::runtime_error("CppLos::read8Bytes(): Error.Read.Timeout");
      }

      // If u !=0 there was some data in the socket, maybe not enough, but we can check; right?
      if ((write_index_ - read_index_) >= 8)
      {
        // If we have enough data after reading just break the loop
        keep_trying = false;
      }
    }
  }
// Transfer eight bytes from the buffer to the member attribute.
#ifdef LOS_IS_LITTLE_ENDIAN
  common_temp_64_.ByteString[0] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[1] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[2] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[3] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[4] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[5] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[6] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[7] = stream_buffer_[read_index_++];
#else
  common_temp_64_.ByteString[7] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[6] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[5] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[4] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[3] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[2] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[1] = stream_buffer_[read_index_++];
  common_temp_64_.ByteString[0] = stream_buffer_[read_index_++];
#endif
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes eight bytes from the member attribute 'common_temp_64_' to the
// buffer. If the buffer provides insufficient free space, write the
// buffer to the socket.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::write8Bytes(void)
{
  uint32_t u;

// Check if the buffer provides enough space.
  if ((write_index_ + 8) > stream_buffer_size_)
  {
// If not, write the buffer to the TCP socket.
    u = write_index_ - read_index_;
    u = p_socket_->write(&stream_buffer_[read_index_], u);
// Update the buffer management data.
    read_index_ = read_index_ + u;
// Defragment the buffer after reading data.
    if (read_index_ > 0)
    {
      u = write_index_ - read_index_;
      for (write_index_ = 0; write_index_ < u; write_index_++)
        stream_buffer_[write_index_] = stream_buffer_[read_index_ + write_index_];
      read_index_ = 0;
    }
// If there's still too little space after writing, end.
    if ((write_index_ + 8) > stream_buffer_size_)
      throw std::runtime_error("CppLos::write8Bytes(): Error.Write.Timeout");
  }
// Transfer eight bytes from the member attribute to the buffer.
#ifdef LOS_IS_LITTLE_ENDIAN
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[0];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[1];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[2];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[3];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[4];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[5];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[6];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[7];
#else
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[7];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[6];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[5];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[4];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[3];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[2];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[1];
  stream_buffer_[write_index_++] = common_temp_64_.ByteString[0];
#endif
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// De-serialises a string (32bit length and ASCII).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::deserialiseString(std::string &data)
{
  uint32_t u;

// De-serialise and set the string length.
  read4Bytes();
  data.resize(common_temp_32_.UInt32Type);
// De-serialise the string.
  for (u = 0; u < common_temp_32_.UInt32Type; u++)
  {
    read1Byte();
    data[u] = common_temp_8_.Int8Type;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Serialises a string (32bit length and ASCII).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::serialiseString(std::string &data)
{
  uint32_t u;

// Serialise the string length.
  common_temp_32_.UInt32Type = (uint32_t)data.length();
  write4Bytes();
// Serialise the string.
  for (u = 0; u < common_temp_32_.UInt32Type; u++)
  {
    common_temp_8_.Int8Type = data[u];
    write1Byte();
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Frees all resources that have been allocated.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::cleanup(void)
{
// Delete the instance of the TCP socket.
  if (p_socket_ != (Connection *)NULL)
  {
    delete p_socket_;
    p_socket_ = (Connection *)NULL;
  }
// Delete the TCP buffer.
  if (stream_buffer_ != (uint8_t *)NULL)
  {
    delete [] stream_buffer_;
    stream_buffer_ = (uint8_t *)NULL;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// This method reads the configuration data from an XML-file by
// making use of the instance of the class 'cXML' that is provided
// as argument.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::configuration(std::string host, int port, double timeout)
{
  stream_buffer_ = new uint8_t[stream_buffer_size_];
  if (stream_buffer_ == (uint8_t *)NULL)
    throw std::runtime_error("CppLos::configuration(): Error.New.Failed");

  // Create a new instance of the class 'cTCp_socket_'.
  try
  {
    p_socket_ = new Connection(host,port,timeout);
    if (p_socket_ == (Connection *)NULL)
      throw std::runtime_error("CppLos::configuration(): Error.New.Failed");
  }
  catch (std::runtime_error e)
  {
    cleanup();
    throw;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// General public methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// The constructor of the class 'CppLos'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
CppLos::CppLos() : stream_buffer_size_(4096)
{
// Initially there are no resources allocated.
  p_socket_ = (Connection *)NULL;
  stream_buffer_ = (uint8_t *)NULL;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// The destructor of the class 'CppLos'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
CppLos::~CppLos()
{
  cleanup();
}

void CppLos::connect(std::string host, int port, double timeout)
{
  configuration(host,port,timeout);
  // Initialise the stream buffer.
  init();
  // Connect to the destination host
  p_socket_->open();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Buffer-related public methods.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Prepares the buffer for writing a command/request to the TCP socket
// or for reading response or from the TCP socket.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::init(void)
{
// Just clear the stream buffer.
  read_index_ = 0;
  write_index_ = 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes all data that is left in the buffer to the TCP socket.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::flush(void)
{
  uint32_t u;

// Check if the buffer contains any data.
  if (write_index_ > read_index_)
  {
// If so, try to write the buffer to the TCP socket.
    u = write_index_ - read_index_;
    u = p_socket_->write(&stream_buffer_[read_index_], u);
// Update the buffer management data.
    read_index_ = read_index_ + u;
// Defragment the buffer after reading data.
    if (read_index_ > 0)
    {
      u = write_index_ - read_index_;
      for (write_index_ = 0; write_index_ < u; write_index_++)
        stream_buffer_[write_index_] = stream_buffer_[read_index_ + write_index_];
      read_index_ = 0;
    }
// Check if there's still data left in the buffer.
    if ((write_index_ != 0) || (read_index_ != 0))
      throw std::runtime_error("CppLos::flush(): Error.flush.Incomplete");
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//************************************************************************
// Public methods for the LOS protocol.
//************************************************************************
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Returns the first byte's value that is stored in the stream buffer.
// This method is invoked by higher-level methods in order to deter-
// mine which processing is required given the data in the buffer.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint8_t CppLos::getTypeID(void)
{
  return view1Byte();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads a single boolean value and returns it.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool CppLos::readBoolean(void)
{
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_BOOLEAN)
    throw std::runtime_error("CppLos::readBoolean(): Error.Datatype.Wrong");
// Read the boolean value.
  read1Byte();
  if (common_temp_8_.UInt8Type == 0)
    return false;
  else
    return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes a single boolean value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeBoolean(bool data)
{
// Set the type ID of a boolean.
  common_temp_8_.UInt8Type = LOS_BOOLEAN;
  write1Byte();
// Write the boolean value.
  if (data)
    common_temp_8_.UInt8Type = 1;
  else
    common_temp_8_.UInt8Type = 0;
  write1Byte();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads a packed boolean array. The data is placed in the buffer that
// is the method's first argument. Furthermore, the number of boolean
// values read is stored in the method's second argument.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::readBooleanArray(bool data[], uint32_t &length)
{
  uint32_t u;
  uint8_t mask;

// Check the validity of the arguments.
  if (data == (bool *)NULL)
    throw std::runtime_error("CppLos::readBooleanArray(): Error.Pointer.NULL");
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_BOOLEAN_ARRAY)
    throw std::runtime_error("CppLos::readBooleanArray(): Error.Datatype.Wrong");
// Get the length of the boolean array.
  read4Bytes();
  length = common_temp_32_.UInt32Type;
// Unpack the boolean array.
  mask = 0x00;
  for (u = 0; u < length; u++)
  {
    if (mask == 0x00)
    {
      mask = 0x01;
      read1Byte();
    }
    data[u] = ((common_temp_8_.UInt8Type & mask) != 0);
    mask = mask << 1;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes 'length' booleans. The boolean values are coded in a packed
// format. This means that the input booleans are coded as bitstring
// (LSB first).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeBooleanArray(bool data[], uint32_t length)
{
  uint32_t u;
  uint8_t mask;

// Check the validity of the arguments.
  if (data == (bool *)NULL)
    throw std::runtime_error("CppLos::writeBooleanArray(): Error.Pointer.NULL");
// Set the type ID of a boolean array.
  common_temp_8_.UInt8Type = LOS_BOOLEAN_ARRAY;
  write1Byte();
// Set the length of the boolean array.
  common_temp_32_.UInt32Type = length;
  write4Bytes();
// Pack the boolean array into a bitstring.
  common_temp_8_.UInt8Type = 0;
  mask = 0x01;
  for (u = 0; u < length; u++)
  {
    if (data[u])
      common_temp_8_.UInt8Type = common_temp_8_.UInt8Type | mask;
    mask = mask << 1;
    if (mask == 0x00)
    {
      write1Byte();
      common_temp_8_.UInt8Type = 0;
      mask = 0x01;
    }
  }
  if (mask != 0x01)
    write1Byte();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads a single int8 value and returns it.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int8_t CppLos::readInt8(void)
{
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_INT8)
    throw std::runtime_error("CppLos::readInt8(): Error.Datatype.Wrong");
// Read the int8 value.
  read1Byte();
  return common_temp_8_.Int8Type;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes a single int8 value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeInt8(int8_t data)
{
// Write the type ID of an int8.
  common_temp_8_.UInt8Type = LOS_INT8;
  write1Byte();
// Write the int8 value.
  common_temp_8_.Int8Type = data;
  write1Byte();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads an array of int8 values. The data is placed in the buffer
// that is the method's first argument. Furthermore, the number of
// int8 values read is stored in the method's second argument.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::readInt8Array(int8_t data[], uint32_t &length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (int8_t *)NULL)
    throw std::runtime_error("CppLos::readInt8Array(): Error.Pointer.NULL");
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_INT8_ARRAY)
    throw std::runtime_error("CppLos::readInt8Array(): Error.Datatype.Wrong");
// Get the length of the int8 array.
  read4Bytes();
  length = common_temp_32_.UInt32Type;
// Read the int8 values.
  for (u = 0; u < length; u++)
  {
    read1Byte();
    data[u] = common_temp_8_.Int8Type;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes 'length' int8 values.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeInt8Array(int8_t data[], uint32_t length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (int8_t *)NULL)
    throw std::runtime_error("CppLos::writeInt8Array(): Error.Pointer.NULL");
// Write the type ID of an int8 array.
  common_temp_8_.UInt8Type = LOS_INT8_ARRAY;
  write1Byte();
// Set the length of the int8 array.
  common_temp_32_.UInt32Type = length;
  write4Bytes();
// Write the int8 values.
  for (u = 0; u < length; u++)
  {
    common_temp_8_.Int8Type = data[u];
    write1Byte();
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads a single int16 value and returns it.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int16_t CppLos::readInt16(void)
{
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_INT16)
    throw std::runtime_error("CppLos::readInt16(): Error.Datatype.Wrong");
// Read the int16 value.
  read2Bytes();
  return common_temp_16_.Int16Type;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes a single int16 value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeInt16(int16_t data)
{
// Write the type ID of an int16.
  common_temp_8_.UInt8Type = LOS_INT16;
  write1Byte();
// Write the int16 value.
  common_temp_16_.Int16Type = data;
  write2Bytes();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads an int16 array. The data is placed in the buffer that is the
// method's first argument. Furthermore, the number of int16 values
// read is stored in the method's second argument.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::readInt16Array(int16_t data[], uint32_t &length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (int16_t *)NULL)
    throw std::runtime_error("CppLos::readInt16Array(): Error.Pointer.NULL");
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_INT16_ARRAY)
    throw std::runtime_error("CppLos::readInt16Array(): Error.Datatype.Wrong");
// Get the length of the int16 array.
  read4Bytes();
  length = common_temp_32_.UInt32Type;
// Read the int16 values.
  for (u = 0; u < length; u++)
  {
    read2Bytes();
    data[u] = common_temp_16_.Int16Type;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes 'length' int16 values (LSB first).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeInt16Array(int16_t data[], uint32_t length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (int16_t *)NULL)
    throw std::runtime_error("CppLos::writeInt16Array(): Error.Pointer.NULL");
// Set the type ID of an int16 array.
  common_temp_8_.UInt8Type = LOS_INT16_ARRAY;
  write1Byte();
// Set the length of the int16 array.
  common_temp_32_.UInt32Type = length;
  write4Bytes();
// Write the int16 values.
  for (u = 0; u < length; u++)
  {
    common_temp_16_.Int16Type = data[u];
    write2Bytes();
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads a single int32 value and returns it.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int32_t CppLos::readInt32(void)
{
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_INT32)
    throw std::runtime_error("CppLos::readInt32(): Error.Datatype.Wrong");
// Read the int32 value.
  read4Bytes();
  return common_temp_32_.Int32Type;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes a single int32 value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeInt32(int32_t data)
{
// Set the type ID of an int32.
  common_temp_8_.UInt8Type = LOS_INT32;
  write1Byte();
// Write the int32 value.
  common_temp_32_.Int32Type = data;
  write4Bytes();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads an int32 array. The data is placed in the buffer that is the
// method's first argument. Furthermore, the number of int32 values
// read is stored in the method's second argument.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::readInt32Array(int32_t data[], uint32_t &length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (int32_t *)NULL)
    throw std::runtime_error("CppLos::readInt32Array(): Error.Pointer.NULL");
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_INT32_ARRAY)
    throw std::runtime_error("CppLos::readInt32Array(): Error.Datatype.Wrong");
// Get the length of the int32 array.
  read4Bytes();
  length = common_temp_32_.UInt32Type;
// Read the int32 values.
  for (u = 0; u < length; u++)
  {
    read4Bytes();
    data[u] = common_temp_32_.Int32Type;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes 'length' int32 values (LSB first).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeInt32Array(int32_t data[], uint32_t length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (int32_t *)NULL)
    throw std::runtime_error("CppLos::writeInt32Array(): Error.Pointer.NULL");
// Set the type ID of an int32 array.
  common_temp_8_.UInt8Type = LOS_INT32_ARRAY;
  write1Byte();
// Set the length of the int32 array.
  common_temp_32_.UInt32Type = length;
  write4Bytes();
// Write the int32 values.
  for (u = 0; u < length; u++)
  {
    common_temp_32_.Int32Type = data[u];
    write4Bytes();
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads a single int64 value and returns it.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int64_t CppLos::readInt64(void)
{
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_INT64)
    throw std::runtime_error("CppLos::readInt64(): Error.Datatype.Wrong");
// Read the int64 value.
  read8Bytes();
  return common_temp_64_.Int64Type;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes a single int64 value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeInt64(int64_t data)
{
// Write the type ID of an int64.
  common_temp_8_.UInt8Type = LOS_INT64;
  write1Byte();
// Write the int64 value.
  common_temp_64_.Int64Type = data;
  write8Bytes();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads an int64 array. The data is placed in the buffer that is the
// method's first argument. Furthermore, the number of int64 values
// read is stored in the method's second argument.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::readInt64Array(int64_t data[], uint32_t &length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (int64_t *)NULL)
    throw std::runtime_error("CppLos::readInt64Array(): Error.Pointer.NULL");
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_INT64_ARRAY)
    throw std::runtime_error("CppLos::readInt64Array(): Error.Datatype.Wrong");
// Get the length of the int64 array.
  read4Bytes();
  length = common_temp_32_.UInt32Type;
// Read the int64 values.
  for (u = 0; u < length; u++)
  {
    read8Bytes();
    data[u] = common_temp_64_.Int64Type;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes 'length' int64 values (LSB first).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeInt64Array(int64_t data[], uint32_t length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (int64_t *)NULL)
    throw std::runtime_error("CppLos::writeInt64Array(): Error.Pointer.NULL");
// Set the type ID of an int64 array.
  common_temp_8_.UInt8Type = LOS_INT64_ARRAY;
  write1Byte();
// Set the length of the int64 array.
  common_temp_32_.UInt32Type = length;
  write4Bytes();
// Write the int64 values.
  for (u = 0; u < length; u++)
  {
    common_temp_64_.Int64Type = data[u];
    write8Bytes();
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads a single float32 value and returns it.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
float CppLos::readFloat32(void)
{
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_FLOAT32)
    throw std::runtime_error("CppLos::readFloat32(): Error.Datatype.Wrong");
// Read the float32 value.
  read4Bytes();
  return common_temp_32_.Float32Type;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes a single float32 value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeFloat32(float data)
{
// Set the type ID of a float32.
  common_temp_8_.UInt8Type = LOS_FLOAT32;
  write1Byte();
// Write the float32 value.
  common_temp_32_.Float32Type = data;
  write4Bytes();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads a float32 array. The data is placed in the buffer that is the
// method's first argument. Furthermore, the number of float32 values
// read is stored in the method's second argument.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::readFloat32Array(float data[], uint32_t &length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (float *)NULL)
    throw std::runtime_error("CppLos::readFloat32Array(): Error.Pointer.NULL");
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_FLOAT32_ARRAY)
    throw std::runtime_error("CppLos::readFloat32Array(): Error.Datatype.Wrong");
// Get the length of the float32 array.
  read4Bytes();
  length = common_temp_32_.UInt32Type;
// Read the float32 values.
  for (u = 0; u < length; u++)
  {
    read4Bytes();
    data[u] = common_temp_32_.Float32Type;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes 'length' float32 values.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeFloat32Array(float data[], uint32_t length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (float *)NULL)
    throw std::runtime_error("CppLos::writeFloat32Array(): Error.Pointer.NULL");
// Set the type ID of a float32 array.
  common_temp_8_.UInt8Type = LOS_FLOAT32_ARRAY;
  write1Byte();
// Set the length of the float32 array.
  common_temp_32_.UInt32Type = length;
  write4Bytes();
// Write the float32 values.
  for (u = 0; u < length; u++)
  {
    common_temp_32_.Float32Type = data[u];
    write4Bytes();
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads a single float64 value and returns it.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double CppLos::readFloat64(void)
{
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_FLOAT64)
    throw std::runtime_error("CppLos::readFloat64(): Error.Datatype.Wrong");
// Read the float64 value.
  read8Bytes();
  return common_temp_64_.Float64Type;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes a single float64 value.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeFloat64(double data)
{
// Set the type ID of a float64.
  common_temp_8_.UInt8Type = LOS_FLOAT64;
  write1Byte();
// Write the float64 value.
  common_temp_64_.Float64Type = data;
  write8Bytes();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads a float64 array. The data is placed in the buffer that is the
// method's first argument. Furthermore, the number of float64 values
// read is stored in the method's second argument.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::readFloat64Array(double data[], uint32_t &length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (double *)NULL)
    throw std::runtime_error("CppLos::readFloat64Array(): Error.Pointer.NULL");
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_FLOAT64_ARRAY)
    throw std::runtime_error("CppLos::readFloat64Array(): Error.Datatype.Wrong");
// Get the length of the float64 array.
  read4Bytes();
  length = common_temp_32_.UInt32Type;
// Read the float64 values.
  for (u = 0; u < length; u++)
  {
    read8Bytes();
    data[u] = common_temp_64_.Float64Type;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes 'length' float64 values (LSB first).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeFloat64Array(double data[], uint32_t length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (double *)NULL)
    throw std::runtime_error("CppLos::writeFloat64Array(): Error.Pointer.NULL");
// Set the type ID of a float64 array.
  common_temp_8_.UInt8Type = LOS_FLOAT64_ARRAY;
  write1Byte();
// Set the length of the float64 array.
  common_temp_32_.UInt32Type = length;
  write4Bytes();
// Write the float64 values.
  for (u = 0; u < length; u++)
  {
    common_temp_64_.Float64Type = data[u];
    write8Bytes();
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads a single ASCII string.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::readString(std::string &data)
{
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_STRING)
    throw std::runtime_error("CppLos::readString(): Error.Datatype.Wrong");
// Read the string.
  deserialiseString(data);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes a single ASCII string.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeString(std::string &data)
{
// Set the type ID of a string.
  common_temp_8_.UInt8Type = LOS_STRING;
  write1Byte();
// Write the string.
  serialiseString(data);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Reads a string array. The data is placed in the array that is the
// method's first argument. Furthermore, the number of strings read
// is stored in the method's second argument.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::readStringArray(std::string data[], uint32_t &length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (std::string *)NULL)
    throw std::runtime_error("CppLos::readStringArray(): Error.Pointer.NULL");
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_STRING_ARRAY)
    throw std::runtime_error("CppLos::readStringArray(): Error.Datatype.Wrong");
// Get the length of the string array.
  read4Bytes();
  length = common_temp_32_.UInt32Type;
//Read the strings.
  for (u = 0; u < length; u++)
    deserialiseString(data[u]);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes 'length' ASCII strings.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeStringArray(std::string data[], uint32_t length)
{
  uint32_t u;

// Check the validity of the arguments.
  if (data == (std::string *)NULL)
    throw std::runtime_error("CppLos::writeStringArray(): Error.Pointer.NULL");
// Set the type ID of a string array.
  common_temp_8_.UInt8Type = LOS_STRING_ARRAY;
  write1Byte();
// Set the length of the string array.
  common_temp_32_.UInt32Type = length;
  write4Bytes();
// Write the strings.
  for (u = 0; u < length; u++)
    serialiseString(data[u]);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Returns the number of objects that a generic array consists of.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t CppLos::readArray(void)
{
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_ARRAY)
    throw std::runtime_error("CppLos::readArray(): Error.Datatype.Wrong");
// Get the length of the generic array.
  read4Bytes();
  return common_temp_32_.UInt32Type;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes a generic array consisting of 'length' objects.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeArray(uint32_t length)
{
// Set the type ID of a generic array.
  common_temp_8_.UInt8Type = LOS_ARRAY;
  write1Byte();
// Set the length of the generic array.
  common_temp_32_.UInt32Type = length;
  write4Bytes();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Returns the name (string) of a call and the number of its arguments.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t CppLos::readCall(std::string &name)
{
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_CALL)
    throw std::runtime_error("CppLos::readCall(): Error.Datatype.Wrong");
// Get the name of the call.
  deserialiseString(name);
// Get the length of the argument array.
  read4Bytes();
  return common_temp_32_.UInt32Type;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes a call. This method only writes the name of the call and
// the argument number. The argument values themselves are written
// somewhere else.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeCall(std::string &name, uint32_t length)
{
// Set the type ID of a call.
  common_temp_8_.UInt8Type = LOS_CALL;
  write1Byte();
// Set the name of the call.
  serialiseString(name);
// Set the length of the argument array.
  common_temp_32_.UInt32Type = length;
  write4Bytes();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Checks if there are any arguments for the call result. If not,
// 'false' is returned, otherwise 'true'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool CppLos::readCallResult(void)
{
  // Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_CALL_RESULT)
    throw std::runtime_error("CppLos::readCallResult(): Error.Datatype.Wrong");
  // If there are no arguments just consume the void type.
  if (view1Byte() == LOS_VOID)
  {
    read1Byte();
    return false;
  }
  else
    return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes a call result. This method only writes the ID of the call
// result type and a void type if the call result has no arguments.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeCallResult(bool b_further_arguments)
{
// Set the type ID of a call result.
  common_temp_8_.UInt8Type = LOS_CALL_RESULT;
  write1Byte();
// If there are no arguments, set the ID of a void type.
  if (!b_further_arguments)
  {
    common_temp_8_.UInt8Type = LOS_VOID;
    write1Byte();
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Returns the name and the error message of the call exception. Be-
// sides, it checks if there are further arguments. If not, 'false'
// is returned, otherwise 'true'.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool CppLos::readCallException(std::string &name, std::string &Msg)
{
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_CALL_EXCEPTION)
    throw std::runtime_error("CppLos::readCallException(): Error.Datatype.Wrong");
// Get the call exception's name and error message.
  deserialiseString(name);
  deserialiseString(Msg);
// If there are no arguments just consume the void type.
  if (view1Byte() == LOS_VOID)
  {
    read1Byte();
    return false;
  }
  else
    return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes a call exception.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeCallException(std::string &name, std::string &Msg, bool b_further_arguments)
{
// Set the type ID of a call exception.
  common_temp_8_.UInt8Type = LOS_CALL_EXCEPTION;
  write1Byte();
// Set the call exception's name and error message.
  serialiseString(name);
  serialiseString(Msg);
// If there are no arguments, set the ID of a void type.
  if (!b_further_arguments)
  {
    common_temp_8_.UInt8Type = LOS_VOID;
    write1Byte();
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Returns the number of elements of the struct
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t CppLos::readStruct(void)
{
// Check if it is the correct data type.
  read1Byte();
  if (common_temp_8_.UInt8Type != LOS_STRUCT)
    throw std::runtime_error("CppLos::readStruct(): Error.Datatype.Wrong");
  // Get the length of the Struct.
  read4Bytes();
  return common_temp_32_.UInt32Type;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes a call exception.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeStruct(uint32_t length)
{
// Set the type ID of a struct.
  common_temp_8_.UInt8Type = LOS_STRUCT;
  write1Byte();
  // Set the length of the struct
  common_temp_32_.UInt32Type = length;
  write4Bytes();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ <OK>
// Writes struct key
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CppLos::writeStructKey(std::string &key)
{
  // Write the string key.
  serialiseString(key);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
