/******************************************************************************
 * Copyright (C) 2019 Italian Institute of Technology
 *
 *    Developers:
 *    Fernando Caponetto (2019-2020, fernando.caponetto@iit.it)
 *    Siddharth Deore (2021-), <siddharthdeore@iit.it>
 *
 ******************************************************************************/

#ifndef _TCP_SOCKET_H_
#define _TCP_SOCKET_H_

#include <ISocket.h>

namespace ToM
{
template<typename S, typename R = S>
class TcpSocket : public ISocket<S,R>
{
public:

    TcpSocket() = default;

    /**
     * Create TCP Socket
     * @throws std::runtime_error in case of failure
     */
    void  sock_init() override
    {
        // Close if socket already open This is forced opening a socket
        // close_socket();
        if ((this->_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == SOCKET_ERROR)
        {
#if VERBOSE
#ifdef _WIN32
            std::cerr << "Could not create socket(): " << WSAGetLastError() << std::endl;
#elif __linux__
            std::cerr << "Could not create socket(): " << errno << std::endl;
#endif
#endif
            throw std::runtime_error("socket() tcp failed");
        }
    }

    void set_recv_buf(size_t& sendBuff)
    {
        if ( setsockopt( this->_socket, SOL_SOCKET, SO_RCVBUF, &sendBuff, sizeof(sendBuff) ) == SOCKET_ERROR )
        {
            throw std::runtime_error("error setsockopt()");
        }
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

#endif //_TCP_SOCKET_H_
