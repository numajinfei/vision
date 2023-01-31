/*
 * @Descripttion: 
 * @version: 
 * @Author: hw
 * @Date: 2022-04-29 15:25:26
 * @LastEditors: hw
 * @LastEditTime: 2022-05-05 15:49:19
 */
#ifndef WRAP_SOCKET
#define WRAP_SOCKET

// #ifdef __cplusplus
// extern "C" {
// #endif

int srv_listen(int port);
int srv_accept(int listenfd);
int cli_sock();
int cli_connect(const char *srv_ip, int port);
int cli_connect1(const int sock_fd, const char *srv_ip, int port);
int socket_close(int socketfd);
int socket_read(const int fd, char *buf, int len);
int socket_write(const int fd, const char *buf, int len);

// #ifdef __cplusplus
// }
// #endif

namespace net
{
class Socket
{
public:
    explicit Socket(int sockfd)
      : sockfd_(sockfd)
    { }

    ~Socket();

    

    int fd() const { return sockfd_; }

    //!< Enale/disable SO_REUSEPORT
    void setReusePort(bool on);
    
private:
    const int sockfd_;
};
} //!< namespace net

#endif // !WRAP_SOCKET