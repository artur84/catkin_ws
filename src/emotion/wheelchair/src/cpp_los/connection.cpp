/**
 * Half-duplex LOS socket client connection
 *
 * @author SAR
 * Copyright (C) 2009 INRIA
 *
 */

#include<iostream>
#include<stdexcept>
#include<netdb.h>

/* According to POSIX.1-2001 */
#include <sys/select.h>

#include <unistd.h> // close
#include<string.h> //memset
#include<fcntl.h> // fcntl
#include <sys/ioctl.h> //ioctl
#include <stdlib.h> // exit

#include "connection.h"

Connection::Connection(std::string host, int port, double timeout)
{
  try
  {
    setTimeout(timeout);
    config(host, port);
  }
  catch (std::runtime_error &e)
  {
    std::cerr << "Connection::Connection ERROR except = " << e.what() << std::endl;
    exit(-1);
  }
}

Connection::~Connection()
{

  if (isOpen())
  {
    try
    {
      close();
    }
    catch (std::runtime_error &e)
    {
      std::cerr << "Connection::~Connection ERROR except = " << e.what() << std::endl;
    }
  }
}

void Connection::open()// throw(std::runtime_error)
{
  try
  {
    doOpen();
  }
  catch (std::runtime_error &e)
  {
    std::cerr << "Connection::open ERROR except = " << e.what() << std::endl;
    exit(-2);
  }
}

void Connection::close() //throw(std::runtime_error) // synchronized
{
  if (socket_ != -1)
  {
    if (::close(socket_) == -1)
      throw std::runtime_error("Connection::close(): Error.Socket.Close");
    socket_ = -1;
  }
}

void Connection::config(std::string host, int port)
{
  host_ = host;
  port_ = port;

  // Socket Init
  socket_ = -1;

  // Set up the inet host address
  memset(&serv_addr_, 0, sizeof(serv_addr_)); /* System V */
  serv_addr_.sin_family = AF_INET;
  serv_addr_.sin_port = htons(port_);
  try
  {
    strAddrToInetAddr(host_, &serv_addr_);
  }
  catch (std::runtime_error &e)
  {
    throw std::runtime_error("Connection::config(): Error.Socket.HOSTNAME.Invalid");
  }
}

bool Connection::isOpen()
{
  return (socket_ != -1);
}


void Connection::setTimeout(double time)
{
  connect_timeout_s_ = (int)(time / TIME_US);
  connect_timeout_us_ = ((int)time) % TIME_US;

  read_timeout_s_ = (int)(time / TIME_US);
  read_timeout_us_ = ((int)time) % TIME_US;

  write_timeout_s_ = (int)(time / TIME_US);
  write_timeout_us_ = ((int)time) % TIME_US;

}

void Connection::doOpen() // throw(std::runtime_error)
{
  socklen_t sock_length;
  int temp, ret_val, sock_flags;

  // Check if there is already a socket.
  if (socket_ != -1)
    throw std::runtime_error("Connection::doOpen(): Error.Socket.Is.Open");

  // Creation of a stream socket.
  socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_ == -1)
    throw std::runtime_error("Connection::doOpen(): Error.Socket.Open");

  // Make the socket non-blocking.
  sock_flags = fcntl(socket_, F_GETFL);
  if (sock_flags == -1)
    throw std::runtime_error("Connection::doOpen(): Error.Socket.Fcntl");
  if (fcntl(socket_, F_SETFL, (sock_flags | O_NONBLOCK)) == -1)
    throw std::runtime_error("Connection::doOpen(): Error.Socket.Fcntl");

  // Connect to the host - if not immediately connected, wait max. for timeout.
  if (connect(socket_, (sockaddr *)&serv_addr_, sizeof(serv_addr_)) == -1)
  {
    if (errno != EINPROGRESS)
      throw std::runtime_error("Connection::doOpen(): Error.Socket.Connect");
    else
    {
      FD_ZERO(&fdset_);
      FD_SET(socket_, &fdset_);
      tv_.tv_sec = (time_t)connect_timeout_s_;
      tv_.tv_usec = (suseconds_t)connect_timeout_us_;
      ret_val = select((socket_ + 1), NULL, &fdset_, NULL, NULL);
      if (ret_val == -1)
        throw std::runtime_error("Connection::doOpen(): Error.Socket.Open.Select");
      else
        if (ret_val == 0)
          throw std::runtime_error("Connection::doOpen(): Error.Socket.Timeout");
      sock_length = sizeof(temp);
      if (getsockopt(socket_, SOL_SOCKET, SO_ERROR, (void *)(&temp), &sock_length) == -1)
        throw std::runtime_error("Connection::doOpen(): Error.Socket.GetSockOpt");
      if (temp != 0)
        throw std::runtime_error("Connection::doOpen(): Error.Socket.TCPlevel");
    }
  }
  // Make the socket blocking again.
  if (fcntl(socket_, F_SETFL, (sock_flags & ~O_NONBLOCK)) == -1)
    throw std::runtime_error("Connection::doOpen(): Error.Socket.Fcntl");
}

void Connection::strAddrToInetAddr(const std::string host, struct sockaddr_in *sockAddr)
{
  unsigned long in_addr;
  struct hostent *p_host_ent;  /* pointer to a host table entry */

  /* IP Address (notation point-decimal) */
  if ((in_addr = inet_addr(host.c_str())) != (unsigned long) - 1L)
  {
    memcpy((char *) &(sockAddr->sin_addr), (char *) &in_addr, sizeof(in_addr));
  }
  /* Host Name */
  else
  {
    /* Convert host logic address name to equivalent IP address */
    if ((p_host_ent = gethostbyname(host.c_str())) == NULL)
    {
      throw std::runtime_error("Connection::strAddrToInetAddr - Error.Socket.GetHostByName");
    }
    sockAddr->sin_addr = *((struct in_addr *)p_host_ent->h_addr);
  }
}

uint32_t Connection::read(uint8_t *data, uint32_t read_count)
{
  socklen_t length;
  int temp, ret_val;

// Check the validity of the pointer to the destination buffer'.
  if (data == (uint8_t *)NULL)
    throw std::runtime_error("Connection::Read(): Error.Pointer.NULL");

  // Check if there is data to read (with timeout).
  FD_ZERO(&fdset_);
  FD_SET(socket_, &fdset_);
  tv_.tv_sec = read_timeout_s_;
  tv_.tv_usec = read_timeout_us_;
  ret_val = select((socket_ + 1), &fdset_, NULL, NULL, NULL);
  // Check if there has been an error...
  if (ret_val == -1)
    throw std::runtime_error("Connection::Read(): Error.Socket.Read.Select");
  // ...or a timeout.
  else
    if (ret_val == 0)
      return 0;
  length = sizeof(temp);
  if (getsockopt(socket_, SOL_SOCKET, SO_ERROR, (void *)(&temp), &length) == -1)
    throw std::runtime_error("Connection::Read(): Error.Socket.GetSockOpt");
  if (temp != 0)
    throw std::runtime_error("Connection::Read(): Error.Socket.TCPlevel");

  // If neither of both is true, there must be data for reading.
  ret_val = ::recv(socket_, (void *)data, read_count, 0);
  if (ret_val == -1)
    throw std::runtime_error("Connection::Read(): Error.Socket.Receive");
// Return the number of bytes read.
  return (uint32_t)ret_val;
}

uint32_t Connection::write(uint8_t *data, uint32_t write_count)
{

  socklen_t length;
  int temp, ret_val;
// Check the validity of the pointer to the source buffer'.
  if (data == (uint8_t *)NULL)
    throw std::runtime_error("Connection::Write(): Error.Pointer.NULL");
// Check if the data buffer contains any data.
  if (write_count == 0)
    throw std::runtime_error("Connection::Write(): Error.Buffer.Empty");
// Check the writeability of the socket.
  FD_ZERO(&fdset_);
  FD_SET(socket_, &fdset_);
  tv_.tv_sec = write_timeout_s_;
  tv_.tv_usec = write_timeout_us_;
  ret_val = select((socket_ + 1), NULL, &fdset_, NULL, NULL);
  if (ret_val == -1)
  {
    throw std::runtime_error("Connection::Write(): Error.Socket.Write.Select");
  }
  else if (ret_val == 0)
    throw std::runtime_error("Connection::Write(): Error.Socket.Timeout");
  length = sizeof(temp);
  if (getsockopt(socket_, SOL_SOCKET, SO_ERROR, (void *)(&temp), &length) == -1)
    throw std::runtime_error("Connection::Write(): Error.Socket.GetSockOpt");
  if (temp != 0)
    throw std::runtime_error("Connection::Write(): Error.Socket.TCPlevel");

  // Write the data to the socket.
  ret_val = ::send(socket_, (void *)data, write_count, 0);
  if (ret_val == -1)
    throw std::runtime_error("Connection::Write(): Error.Socket.Send");

  // Return the number of bytes written to the socket.
  return (uint32_t)ret_val;
}

uint32_t Connection::numberOfBytesToRead(void)
{
  unsigned int u;

  if (ioctl(socket_, FIONREAD, &u) == -1)
    throw std::runtime_error("Connection::NumberOfBytesToRead(): Error.Port.Ioctl");

  return u;
}
