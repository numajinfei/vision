/*
 * @Descripttion: 
 * @version: 
 * @Author: hw
 * @Date: 2022-04-29 15:28:42
 * @LastEditors: hw
 * @LastEditTime: 2022-05-07 20:47:03
 */
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <strings.h>
#include <string.h>
#include <ctype.h>
#include <arpa/inet.h>

#include <stdexcept> 
extern "C" 
{
#include "wrap.h"
}

#define CONNECT_LIMIT 8
/**
 * @description: create a server endpoint of a connection.
 * @param name: socket file pathname        
 * @return: server fd or error
 */
int srv_listen(int port)
{
    int fd, ret;

    struct sockaddr_in srv_addr;
    
    //create a UNIX domain stream socket
    if((fd = Socket(AF_INET, SOCK_STREAM, 0)) < 0) 
        return fd;    
    

    //fill in socket address structure    
    bzero(&srv_addr, sizeof(srv_addr));  
    srv_addr.sin_family = AF_INET;
    srv_addr.sin_addr.s_addr = htonl(INADDR_ANY);  
    srv_addr.sin_port = htons(port); 

    //address reuse
    int opt = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));


    //bind the name to the descriptor    
    ret = Bind(fd, (struct sockaddr *)&srv_addr, sizeof(srv_addr));
    if(ret < 0)
        return ret;

    ret = Listen(fd, CONNECT_LIMIT);
    if(ret < 0)
        return ret;

    // socklen_t len = sizeof(struct sockaddr);
    // if(0 == getsockname(fd, (struct sockaddr *)&srv_addr, &len))
    // {
    //     // printf("server IP is %s:%d\n", inet_ntop(AF_INET, &srv_addr.sin_addr, ip, sizeof(ip)), ntohs(srv_addr.sin_port));
    //     printf("server IP is %s:%d\n", inet_ntoa(srv_addr.sin_addr), ntohs(srv_addr.sin_port));
    // }
    return fd;
}

/**
 * @description: waitting for a client connection to arrive, and accept it.
 * @param listenfd: client listen id      
 * @return: client fd or error
 */
int srv_accept(int listenfd)
{
    int cli_fd, rval;
    struct sockaddr_in cli_addr;
    socklen_t cli_addr_len;
    char cli_ip[64];

    //listen client, is blocking
    cli_addr_len = sizeof(cli_addr_len); //get client address struct size
    if((cli_fd = Accept(listenfd, (struct sockaddr *)&cli_addr, &cli_addr_len)) < 0){
        // close(listenfd);
        return cli_fd;
    }

    //!< 获取cli_fd表示的连接上的本地地址
    long srv_len = sizeof(struct sockaddr);
    if(0 == getsockname(cli_fd, (struct sockaddr *)&cli_addr, (socklen_t *)&srv_len))
    {
        printf("server IP is %s:%d\n", inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port));        
    }
    
    struct sockaddr_in peer_addr;
    socklen_t len = sizeof(peer_addr);
    if(getpeername(cli_fd, (struct sockaddr *)&peer_addr, &len) == -1)
    {
        printf("getpeername error\n");
        return -1;
    }
    printf("peer client IP:%s\tport:%d\n", 
            inet_ntop(AF_INET, &peer_addr.sin_addr, cli_ip, sizeof(cli_ip)), 
            ntohs(peer_addr.sin_port));

    return (cli_fd);
}

int cli_sock()
{
    return Socket(AF_INET, SOCK_STREAM, 0);
}
/**
 * @description: creata a client endpoint and connect to a server
 * @param srv_ip: the address of server's ip
 * @param port: the port of server     
 * @return: client fd or error
 */
int cli_connect(const char *srv_ip, int port)
{
    struct sockaddr_in srv_addr;
    int fd;
    int ret;

    fd = Socket(AF_INET, SOCK_STREAM, 0);
    if(fd < 0)
    {   
        throw std::runtime_error("[socket].(Client)--> : call socket() failed");
        return fd;
    }

    bzero(&srv_addr, sizeof(srv_addr));                       
    srv_addr.sin_family = AF_INET;                             
    inet_pton(AF_INET, srv_ip, &srv_addr.sin_addr.s_addr);    
    srv_addr.sin_port = htons(port);                      

    //Specifies the file descriptor associated with the socket
    ret = Connect(fd, (struct sockaddr *)&srv_addr, sizeof(srv_addr));
    if(ret < 0)
    {        
        throw std::runtime_error("[socket].(Client)--> : call connect() failed");
        return ret;
    }
        

    return fd;
}

int cli_connect1(const int sock_fd, const char *srv_ip, int port)
{
    struct sockaddr_in srv_addr;
    //int fd;
    int ret;
    

    bzero(&srv_addr, sizeof(srv_addr));                       
    srv_addr.sin_family = AF_INET;                             
    inet_pton(AF_INET, srv_ip, &srv_addr.sin_addr.s_addr);    
    srv_addr.sin_port = htons(port);                      

    //Specifies the file descriptor associated with the socket
    ret = Connect(sock_fd, (struct sockaddr *)&srv_addr, sizeof(srv_addr));
    if(ret < 0)
    {        
        // throw std::runtime_error("[socket].(Client)--> : call connect() failed");
        printf("[socket].(Client)--> : call connect() failed\n");
    }
        
    return ret;
}

int socket_close(int socketfd)
{
    return Close(socketfd);
}

int socket_read(const int fd, char* buf, int len)
{
    return Read(fd, buf, len);
}

int socket_write(const int fd, const char* buf, int len)
{
    return Write(fd, buf, len);
}