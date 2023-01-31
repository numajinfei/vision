/*
 * @Descripttion: 
 * @version: 
 * @Author: hw
 * @Date: 2022-05-02 22:05:44
 * @LastEditors: hw
 * @LastEditTime: 2022-05-07 15:23:15
 */
#include <iostream>
#include "tcp/tcp.hpp"
// extern "C"
// {
#include "wrap_socket/wrap_socket.hpp"
// }

#include <sys/select.h>
#include <unistd.h>

// #include <chrono>

namespace net
{

#define BUF_SIZE 1024
using namespace std::placeholders;  // 对于 _1, _2, _3...


//!< ------------------------------------------- class Server functions definitions --------------------------------

Server::~Server()
{
    run_.join();
    Close();       
    std::cout << "[tcp].(Server)--> destory complete!!!" << std::endl;
}

void Server::Create()
{
    if((srv_fd_ = srv_listen(port_)) < 0)
    {
        throw std::runtime_error("[tcp].(Server)--> listen " + std::to_string(port_) +  " failed");        
    }

    if((conn_fd_ = srv_accept(srv_fd_)) < 0)
    {
        throw std::runtime_error("[tcp].(Server)--> accept failed");
    }

    run_ = std::thread(&Server::Read, this);
}

void Server::Close()
{
    if(srv_fd_)
    {
        socket_close(srv_fd_);
        srv_fd_ = -1;
    }
        
    if(conn_fd_)
    {
        socket_close(conn_fd_);
        conn_fd_ = -1;
    }        
}

void Server::Read()
{
    int n, ret;
    fd_set readfd;
    char buf[BUF_SIZE];
    struct timeval tv;
    while(1)
    {
        tv.tv_sec = 0;
        tv.tv_usec = 50 * 1000; //50ms
        FD_ZERO(&readfd);
        FD_SET(conn_fd_, &readfd);

        ret = select(conn_fd_ + 1, &readfd, NULL, NULL, &tv); 
        if (ret < 0)
        {
            Close();
            throw std::runtime_error("[tcp].(Client)--> select failed");
        }
        else if(ret == 0)//timeout
        {
            continue;
        }

        if(FD_ISSET(conn_fd_, &readfd))
        {
            if((n = socket_read(conn_fd_, buf, BUF_SIZE)) > 0)
            {
                std::cout << "rx data from Server...\n";
                if(messageCallback_)
                    messageCallback_(buf, n);
                else
                    throw std::runtime_error("[tcp].(Client)--> Invalid messageCallback");
            }
            else if(n < 0)
            {
                throw std::runtime_error("[tcp].(Client)--> Read failed: " + std::to_string(errno));
            } 
        }
    }
}

int Server::Write(char *buf, int len)
{
    // if(writeCallback_)
    //     writeCallback_(conn_fd_, buf, n);

    return socket_write(conn_fd_, buf, len);
}

//!< ------------------------------------------- class Client functions definitions --------------------------------

Client::~Client()
{
    // run_.join();
    Close();
    std::cout << "[tcp].(Client)--> desoty completed!!!\n";
}

void Client::Create()
{
    int times = 50;
    struct timeval tv;

    if((cli_fd_ = cli_sock()) < 0)
    {
        throw std::runtime_error("[tcp].(Client)--> socket() failed, errno:" + std::to_string(errno));
    }

    do
    {
        // if((cli_fd_ = cli_connect(ip_.c_str(), port_)) < 0)
        if(cli_connect1(cli_fd_, ip_.c_str(), port_) < 0)
        {
            std::cout << "[tcp].(Client)--> link server: " <<  ip_ <<  " port. " <<  std::to_string(port_) <<   " failed, and try again..." << std::endl; 
            // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

            tv.tv_sec = 3;
            tv.tv_usec = 0;
            select(0, NULL, NULL, NULL, &tv);
        }
        else
        {
            setConnected(true);
            break;
        }
    }while(times--);
    
    if(times <= 0)
    {
        setConnected(false);
        throw std::runtime_error("[tcp].(Client)--> link server: " + ip_ + " port. " + std::to_string(port_) +  " failed"); 
    }
    
    if(isConnected())
    {
        std::cout << "[tcp].(Client)--> fd is " << cli_fd_ << std::endl;
        std::thread rd(&Client::Read, this);
        rd.detach();
    }
    
}

void Client::Close()
{
    if(cli_fd_)
    {
        socket_close(cli_fd_);
        cli_fd_ = -1;
    }
}

void Client::Read()
{
    int n, ret;
    fd_set readfd;
    char buf[BUF_SIZE];
    struct timeval tv;

    while(isConnected())
    {
        tv.tv_sec = 0;
        tv.tv_usec = 50 * 1000; //50ms
        FD_ZERO(&readfd);
        FD_SET(cli_fd_, &readfd);

        ret = select(cli_fd_ + 1, &readfd, NULL, NULL, &tv); 
        if (ret < 0)
        {
            Close();
            throw std::runtime_error("[tcp].(Client)--> select failed");
        }
        else if(ret == 0)//timeout
        {
            continue;
        }
        
        if(FD_ISSET(cli_fd_, &readfd))
        {
            if((n = socket_read(cli_fd_, buf, BUF_SIZE)) > 0)
            {
                if(messageCallback_)
                    messageCallback_(buf, n);
                else
                    throw std::runtime_error("[tcp].(Client)--> Invalid messageCallback");
            }
            else if(n < 0)
            {
                // throw std::runtime_error("[tcp].(Client)--> Read failed: " + std::to_string(errno));
                std::cout << "[tcp].(Client)--> Read failed, errno: "  << std::to_string(errno) << std::endl;
            }    
        }    
    }
}

int Client::Write(char *buf, int len)
{
    // if(writeCallback_)
    //     writeCallback_(cli_fd_, buf, n);
    if(cli_fd_ < 2)
        throw std::runtime_error("[tcp].(Client)--> Invalid file descriptor");

    if(isConnected())
        return socket_write(cli_fd_, buf, len);
    
    return -1;
}

void Client::setConnected(bool status)
{
    connected_ = status;
}

bool Client::isConnected()
{
    return connected_;
}
}