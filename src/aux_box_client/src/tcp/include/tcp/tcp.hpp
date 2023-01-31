/*
 * @Descripttion: 
 * @version: 
 * @Author: hw
 * @Date: 2022-05-02 22:05:51
 * @LastEditors: hw
 * @LastEditTime: 2022-05-07 15:23:06
 */
#pragma once
#include <iostream>
#include <functional>
#include <thread>
namespace net
{
typedef std::function<void(char* buf, int len)> MessageCallback;
    
class Server
{
public:
    explicit Server(int port)
      : port_(port)
    { }
    ~Server();

    void Create();
    void Close();

    //!< Enale/disable SO_REUSEPORT
    void setReusePort(bool on);

    void setMessageCallback(const MessageCallback& cb)
    { messageCallback_ = cb; }

    void setWriteCallback(const MessageCallback& cb)
    { writeCallback_ = cb; }

    int Write(char* buf, int len);


private:
    void Read();    

private:
    int port_ = -1;
    int srv_fd_ = -1;
    int conn_fd_ = -1;
    bool reuse_port_ = false;
    MessageCallback messageCallback_;
    MessageCallback writeCallback_;

    std::thread run_;
};

class Client
{
public:
    explicit Client(std::string& srv_ip, const int port)
      : ip_(srv_ip), port_(port)
    { }
    ~Client();

    void Create();
    void Close();

    void setMessageCallback(const MessageCallback& cb)
    { messageCallback_ = cb; }

    void setWriteCallback(const MessageCallback& cb)
    { writeCallback_ = cb; }

    int Write(char *buf, int len);

    bool isConnected();

private:
    void Read();
    void setConnected(bool status);
private:
    std::string ip_;
    int port_;
    // int sock_fd_ = -1;
    int cli_fd_ = -1;
    bool connected_ = false;

    MessageCallback messageCallback_;
    MessageCallback writeCallback_;
    // std::thread run_;
};
}