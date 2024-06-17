/**
 * @file  udp_socket.cpp
 * @brief UDP Socket with Multicast Support. Wraps UNIX socket API.
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 28 March 2019
 */

#include "optitrack/udp_socket.h"

namespace acl {
namespace utils {

UDPSocket::UDPSocket(const std::string& localIP, const int localPort)
{
  // create a UDP socket
  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_ == -1) throw SocketException(strerror(errno));

  // allow multiple clients on same machine to use address/port
  int y = 1;
  if (setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, (char *)&y, sizeof(y)) == -1) {
    close(socket_);
    throw SocketException(strerror(errno));
  }

  // create a 1MB buffer
  int optval = BUFSIZE_1MB;
  setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, (char *)&optval, sizeof(optval));
  getsockopt(socket_, SOL_SOCKET, SO_RCVBUF, (char *)&optval, nullptr);
  if (optval != BUFSIZE_1MB) {
    close(socket_);
    throw SocketException(strerror(errno));
  }

  // local address struct
  memset(&localAddr_, 0, sizeof(localAddr_));
  localAddr_.sin_family = AF_INET;
  localAddr_.sin_port = htons(localPort);
  localAddr_.sin_addr.s_addr = inet_addr(localIP.c_str()); // htonl(INADDR_ANY)

  // bind the socket to the local address
  if (bind(socket_, (sockaddr *)&localAddr_, sizeof(localAddr_) ) == -1) {
    close(socket_);
    throw SocketException(strerror(errno));
  }
}

// ----------------------------------------------------------------------------

UDPSocket::UDPSocket(const int localPort)
  // use INADDR_ANY for localIP
: UDPSocket("0.0.0.0", localPort) {}

// ----------------------------------------------------------------------------

UDPSocket::~UDPSocket()
{
  close(socket_); // be a good computer citizen
}

// ----------------------------------------------------------------------------

bool UDPSocket::setReceiveTimeout(const int seconds, const int micros)
{
  // set a receive timeout
  struct timeval timeout = { .tv_sec = seconds, .tv_usec = micros };
  int ret = setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));

  return !(ret == -1);
}

// ----------------------------------------------------------------------------

bool UDPSocket::joinMulticastGroup(const std::string& ifIP, const std::string& multicastGroupIP)
{
  // for multicast, the local addr is required to be INADDR_ANY (0.0.0.0).
  // So we will check to see if a specific local IP was given.
  // see https://stackoverflow.com/a/23718680
  if (localAddr_.sin_addr.s_addr != htonl(INADDR_ANY)) {
    throw SocketException("Attempting to join multicast group "
                          "but a localIP was specified.");
  }

  // fill multicast request struct
  struct ip_mreq mreq{};
  mreq.imr_multiaddr.s_addr = inet_addr(multicastGroupIP.c_str());
  mreq.imr_interface.s_addr = inet_addr(ifIP.c_str());

  // n.b., we precisely indicate mreq.imr_interface.s_addr
  // (instead of just using "0.0.0.0") because on multihomed hosts
  // (i.e., multiple NICs), the kernel will choose which interface should
  // join the group --- and it might choose the wrong one.
  // See https://sites.ualberta.ca/dept/chemeng/AIX-43/share/man/info/C/a_doc_lib/aixprggd/progcomc/skt_multicast.htm
  //    "If no interface is specified then the interface leading to the default route is used"

  // join multicast group
  int ret = setsockopt(socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq));

  return !(ret == -1);
}

// ----------------------------------------------------------------------------

bool UDPSocket::receive(char * buf, size_t buflen)
{
  sockaddr_in remoteAddr{};
  socklen_t addrlen = sizeof(struct sockaddr);

  ssize_t r = recvfrom(socket_, buf, buflen, 0,
                      (sockaddr *)&remoteAddr, &addrlen);

  return !(r == -1);
}

// ----------------------------------------------------------------------------

bool UDPSocket::send(const std::string& remoteIP, const int remotePort,
                     const char * buf, const size_t buflen)
{
  sockaddr_in hostAddr;
  memset(&hostAddr, 0, sizeof(hostAddr));
  hostAddr.sin_family = AF_INET;
  hostAddr.sin_port = htons(remotePort);
  hostAddr.sin_addr.s_addr = inet_addr(remoteIP.c_str());

  ssize_t r = sendto(socket_, buf, buflen, 0,
                     (sockaddr *)&hostAddr, sizeof(hostAddr));

  return !(r == -1);
}

} // ns utils
} // ns acl
