/******************************************************************************
 * Copyright (C) 2019 Italian Institute of Technology
 *
 *    Developers:
 *    Fernando Caponetto (2019-, fernando.caponetto@iit.it)
 *    Siddharth Deore (2021- siddharth.deore@iit.it)
 *
 ******************************************************************************/


#ifndef _UDP_SOCKET_H_
#define _UDP_SOCKET_H_

#include "ISocket.h"

namespace ToM
{
template<typename S, typename R = S>
class UdpSocket : public ISocket<S,R>
{
public:

    /**
     * Create UDP Socket
     * @throws std::runtime_error in case of failure
     */
    void sock_init()
    {
        // Close socket if open
        // close_socket();
#ifdef _WIN32
        /* Initialize Winsock */
		consoleLog->info("Initialising Winsock...");
		if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
		{
			consoleLog->error("WSAStartup failed with error : {}", WSAGetLastError());
			//WSACleanup();
		}
#endif

        if ((this->_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
        {
#if VERBOSE
#ifdef _WIN32
            std::cerr << "Could not create socket(): " << WSAGetLastError() << std::endl;
#elif __linux__
            std::cerr << "Could not create socket(): " << perror << std::endl;
#endif
#endif
            throw std::runtime_error("socket() udp failed");
        }

#ifdef _WIN32
        unsigned long on = 1;
		ioctlsocket(_socket, FIONBIO, &on); /* Non-blocking */
#elif __linux__
        int opt = 1;
        ioctl(this->_socket, FIONBIO, &opt); /* Non-blocking */
#endif
    }


    void set_recv_buf(size_t& sendBuff)
    {
//        socklen_t optlen = sizeof(int);
//        size_t sendbuff;
//
        if ( setsockopt( this->_socket, SOL_SOCKET, SO_RCVBUF, &sendBuff, sizeof(sendBuff) ) == SOCKET_ERROR )
        {
            throw std::runtime_error("error setsockopt()");
//
        }
//
//        // Get buffer size
//        auto res = getsockopt(this->socket_, SOL_SOCKET, SO_RCVBUF, &sendbuff, &optlen);
//
//        if(res == -1)
//            throw std::runtime_error("error getsockopt()");
//        else
//            printf("revc buffer size = %d\n", sendbuff);
    }
    
    void close_socket()
    {
        try
        {

#ifdef _WIN32
            closesocket(this->_socket);
#elif __linux__
            close(this->_socket);
#endif


        }
        catch (std::runtime_error& e)
        {
          std::cerr << e.what() << '\n';
          throw;
        }

    }
};

}

#endif //_UDP_SOCKET_H_
