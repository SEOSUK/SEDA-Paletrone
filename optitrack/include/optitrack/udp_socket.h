/**
 * @file  udp_socket.h
 * @brief UDP Socket with Multicast Support. Wraps UNIX socket API.
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 28 March 2019
 */

#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <cerrno>

#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

namespace acl {
namespace utils {

  /**
   * @brief      Exception for signaling socket errors.
   */
  class SocketException : public std::runtime_error
  {
    public:
      
      /**
       * @brief      Constructor
       *
       * @param[in]  description  Error message
       */
      SocketException ( std::string description ) : std::runtime_error( description ) {}
      
      ~SocketException () throw() {}
  };



  class UDPSocket
  {
  public:
    /**
     * @brief      Binds a socket to a local UDP endpoint described by a
     *             specific IP address and port.
     *
     *             Use other constructor to bind to any interface.
     *             Use localPort = 0 to have kernel decide which port
     *             to bind to.
     *
     * @param[in]  localIP    The local ip
     * @param[in]  localPort  The local port
     */
    UDPSocket(const std::string& localIP, const int localPort);

    /**
     * @brief      Binds a socket to a local UDP endpoint on any address
     *             and on a specific port
     *
     *             IP address 0.0.0.0 is used to bind on any interface/address.
     *             Use localPort = 0 to have the kernel decide with port
     *             to bind to.
     *
     *             NOTE: Use this constructor when joining a multicast group,
     *             otherwise data cannot be received via this socket.
     *
     * @param[in]  localPort  The local port
     */
    UDPSocket(const int localPort);

    /**
     * @brief      Destroys the object.
     */
    ~UDPSocket();

    /**
     * @brief      Allow the blocking 'receive' method to timeout
     *
     * @param[in]  seconds  The seconds (int)
     * @param[in]  micros   The microseconds (int)
     *
     * @return     false if unsuccessful
     */
    bool setReceiveTimeout(const int seconds = 1, const int micros = 0);

    /**
     * @brief      Joins the socket to a Multicast group
     *
     * @param[in]  ifIP              IP addr of the interface that should
     *                               be joining the multicast group. To verify
     *                               that iface joined group: 'netstat -gn'.
     * @param[in]  multicastGroupIP  The multicast group ip
     *
     * @return     false if unsuccessful
     */
    bool joinMulticastGroup(const std::string& ifIP, const std::string& multicastGroupIP);

    /**
     * @brief      Receives data transmitted to the UDP endpoint.
     *             This is a blocking call.
     *
     * @param      buf     The buffer to store the received data
     * @param[in]  buflen  The buffer size
     *
     * @return     false if unsuccessful (e.g., on timeout)
     */
    bool receive(char * buf, size_t buflen);

    /**
     * @brief      Send data through the UDP endpoint to a remote host.
     *
     * @param[in]  remoteIP    The remote host IP
     * @param[in]  remotePort  The remote host port
     * @param[in]  buf         The buffer of data to send
     * @param[in]  buflen      THe buffer size
     *
     * @return     false if unsuccessful
     */
    bool send(const std::string& remoteIP, const int remotePort,
              const char * buf, const size_t buflen);

  private:
    int socket_; ///< UNIX socket file descriptor
    sockaddr_in localAddr_; ///< Local address info for socket

    static constexpr int BUFSIZE_1MB = 0x100000; ///< 1MB buffer size option
    
  };

} // ns utils
} // ns acl
