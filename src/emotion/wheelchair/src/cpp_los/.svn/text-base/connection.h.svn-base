/**
 * Half-duplex LOS socket client connection
 * Code inspired from los package written by Peter Einramhof
 *
 * @author SAR
 *
 * Copyright (C) 2009 INRIA
 */

#ifndef _CONNECTION_H_
#define _CONNECTION_H_

#include <string>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h> //socket
#include <arpa/inet.h> // sockaddr_in

#ifndef NULL
const void *NULL = 0;
#endif
//
const int TIME_US = 1000000;

/**
 * Half-duplex LOS socket client connection.
 *
 * This class manages a client socket connection to a LOS server,
 * and allows calling remote procedures in a half-duplex fashion,
 * i.e. the call is serialized and transmitted, it is executed on
 * the server, the result is serialized and transmitted back, and
 * finally the result is returned either as a return value or as
 * an exception. Consequently, there is no pipelining of calls.
 */
class Connection
{

  private:
    std::string host_;
    int port_;
    int socket_;
    struct sockaddr_in serv_addr_;
    fd_set fdset_;
    struct timeval tv_;
    uint32_t connect_timeout_s_;
    uint32_t connect_timeout_us_;
    uint32_t read_timeout_s_;
    uint32_t read_timeout_us_;
    uint32_t write_timeout_s_;
    uint32_t write_timeout_us_;


    /**
     * Configure connection parameters.
     *
     * @param host the host to connect to
     * @param port the port to use for the connection
     * throws runtime_error
     */
    void config(std::string host, int port); // throw(std::runtime_error);


    /**
     * Actually open the socket to the server.
     * throws runtime_error
     */
    void doOpen(); //throw(std::runtime_error);


    /**
     * Convert a string address to a inet compatible to socket address
     * throws runtime_error
     */
    void strAddrToInetAddr(const std::string host, struct sockaddr_in *sockAddr);

  public:
    /**
     * Construct a client connection to the given host on the given port.
     *
     * @param host the host to connect to
     * @param port the port to use for the connection
     * throws runtime_error
     */
    Connection(std::string host, int port, double timeout);
    ~Connection();


    std::string getHost()
    {
      return host_;
    };
    int getPort()
    {
      return port_;
    };


    /**
     * Open the connection.  And instantiate writer and reader
     * stream object
     * throws runtime_error
     */
    void open(); //throw(std::runtime_error);

    /**
     * Close the connection.
     * throws runtime_error
     */
    void close(); //throw(std::runtime_error);

    /**
     * Check if the connection is open.
     *
     * @return true iff the connection is open
     */
    bool isOpen();


    /**
     * Set the read/open timeout for the connection.
     *
     * @param time the timeout in seconds, or zero for an infinite timeout
     */
    void setTimeout(double time);


    /**
     * Checks via select() if there is data to be read on the socket.
     * This is done using a read timeout. If data is available, the data
     * is stored in the buffer and the number of bytes read is returned.
     *
     * @param data data received
     * @param read_count number of bytes read
     * @return number of bytes read, if zero a timeout has occured
     */
    uint32_t read(uint8_t *data, uint32_t read_count = 1);

    /**
     * Writes data to the socket. If the message can't be sent immediately
     * the method blocks. The return value is the number of bytes that
     * has been sent.
     * THE CALLING INSTANCE MUST CHECK THE RETURN VALUE!!!
     *
     * @param data message to write to socket
     * @param write_count number of bytes written
     * @return number of bytes sent
     */
    uint32_t write(uint8_t *data, uint32_t write_count = 1);

    /**
     * Returns the number of bytes available for reding
     *
     * @return number of bytes available for reading
     */
    uint32_t numberOfBytesToRead(void);
};
#endif //_CONNECTION_H_
