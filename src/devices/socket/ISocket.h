/******************************************************************************
 * Copyright (C) 2019 Italian Institute of Technology
 *
 *    Developers:
 *    Fernando Caponetto (2019-, fernando.caponetto@iit.it)
 *    Siddharth Deore (2021- siddharth.deore@iit.it)
 *
 ******************************************************************************/



#ifndef _SOCKET_H_
#define _SOCKET_H_
#include <memory>
#include <iostream>
#include <fcntl.h>

#ifdef _WIN32
#include <winsock2.h>
typedef long long ssize_t;
#elif __linux__
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#endif

#ifndef _WIN32
#define SOCKET int
#define SOCKET_ERROR (SOCKET)(-1)
#define NO_ERROR 0
#endif // _WIN32

#define VERBOSE 0

namespace ToM
{

template<typename S, typename R = S>
class ISocket
{
public:
    typedef std::shared_ptr<ISocket> ptr;


    virtual ~ISocket()
    {
#ifdef _WIN32
        closesocket(_socket);
#elif __linux__
        close(_socket);
#endif
    }

    virtual void sock_init() = 0;

    virtual void set_recv_buf(size_t& sendBuff) = 0;

    /**
     * Bind the socket for incoming data
     * @param address Local address
     * @param port Local port
     * @throws std::runtime_error in case of failure
     */
    void sock_bind(const char *address, uint16_t port)
    {
        /* zero out structure */
        memset((char *)(&local_), 0, sizeof(local_));

        /* initialize address to bind */
        local_.sin_family = AF_INET;
        local_.sin_port = htons(port);
        local_.sin_addr.s_addr = inet_addr(address);

        /* bind socket to address and port */
        if (bind(_socket, (struct sockaddr *)(&local_), sizeof(local_)) != NO_ERROR)
        {
#if VERBOSE
#ifdef _WIN32
            std::cerr << "Bind failed with error " << WSAGetLastError() << std::endl;
//            consoleLog->error("bind() failed with error: {}", WSAGetLastError());
            closesocket(_socket);
#elif __linux__
            std::cerr << "bind() failed with error " << errno << std::endl;
            close(_socket);
#endif
#endif
            throw std::runtime_error("bind() failed");
        }
        std::cout << "Socket Bind successfull, Local Port " << port << std::endl;
    }


    /**
     * Connect to the destination socket
     * @param address Destination address
     * @param port Destination port
     * @throws std::runtime_error in case of failure
     */
    void sock_connect(const char *address, uint16_t port)
    {
        memset((char *) &remote_, 0, sizeof(remote_));
        remote_.sin_family = AF_INET;
        remote_.sin_port = htons(port);
        remote_.sin_addr.s_addr = inet_addr(address);

        if( connect(_socket, (struct sockaddr *)(&remote_), sizeof(remote_)) != NO_ERROR)
        {
#if VERBOSE
#ifdef _WIN32
            std::cerr << "connect() failed with error: " << WSAGetLastError() << std::endl;
            closesocket(_socket);
#elif __linux__
            std::cerr << "connect() failed with error: " << errno << std::endl;
            close(_socket);
#endif
#endif
            throw std::runtime_error("connect() failed");
        }

        std::cout << "Socket Connect successfull, Remote Port " << port << std::endl;
    }

    /**
     * Send data by using socket
     * @tparam Sock_Tx_Types type to be sent
     * @param tx data to be sent
     * @return sszize_t number of bytes sent
     */
    template<typename Sock_Tx_Types>
    ssize_t sock_send(Sock_Tx_Types &tx, const size_t size = 0) {

        if (size) {
            size_sender_ = size;
        }

        memcpy(buffer_sender_.get(), &tx, size_sender_);
        ssize_t bytes =
                send(_socket,                // Connected socket
                     buffer_sender_.get(),    // Data buffer
                     size_sender_,            // Length of data
                     0);

        return bytes;
    }

    /**
     * Receive data by using socket
     * @tparam Sock_Rx_Types
     * @param rx
     * @return
     */
    template <typename  Sock_Rx_Types>
    ssize_t sock_receive(Sock_Rx_Types & rx)
    {
        if (sizeof(rx) != size_receiver_)
        {
#if VERBOSE
            std::cerr << "The receive size does not match!" << std::endl;
#endif
        }

#ifdef _WIN32
        int si_len = sizeof(si_recv);
#elif __linux__
        socklen_t si_len = sizeof(si_recv);
#endif
        ssize_t bytes =
                recv(_socket,
                     buffer_receiver_.get(),
                     size_receiver_, 0);

        if(bytes > 0)
            memcpy(&rx, buffer_receiver_.get(), size_receiver_);

        return bytes;
    }

protected:

    ISocket()
    {
        size_receiver_ = sizeof(R);
        buffer_receiver_ = std::make_unique<char[]>(size_receiver_);
        // if(S != R)
        // {
        size_sender_ = sizeof(S);
        buffer_sender_ = std::make_unique<char[]>(size_sender_);
        // }
    }

//    std::shared_ptr<spdlog::logger> consoleLog{ spdlog::get("console") };

    std::unique_ptr<char[]> buffer_sender_;
    std::unique_ptr<char[]> buffer_receiver_;

    size_t size_receiver_;

    size_t size_sender_;

#ifdef _WIN32
    WSADATA wsaData;
#endif

    SOCKET _socket;
    struct sockaddr_in local_, remote_, si_recv;

};
}

#endif // _SOCKET_H_

