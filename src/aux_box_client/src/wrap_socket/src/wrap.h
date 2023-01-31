/*
 * @Descripttion: 
 * @version: 
 * @Author: hw
 * @Date: 2022-04-29 14:39:15
 * @LastEditors: hw
 * @LastEditTime: 2022-05-03 00:09:41
 */
#ifndef WRAP_H_
#define WRAP_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cpluscplus */

void perr_exit(const char *s);
int Accept(int fd, struct sockaddr *sa, socklen_t *salenptr);
int Bind(int fd, const struct sockaddr *sa, socklen_t salen);
int Connect(int fd, const struct sockaddr *sa, socklen_t salen);
int Listen(int fd, int backlog);
int Socket(int family, int type, int protocol);
ssize_t Read(int fd, void *ptr, size_t nbytes);
ssize_t Write(int fd, const void *ptr, size_t nbytes);
int Close(int fd);
ssize_t Readn(int fd, void *vptr, size_t n);
ssize_t Writen(int fd, const void *vptr, size_t n);
ssize_t my_read(int fd, char *ptr);
ssize_t Readline(int fd, void *vptr, size_t maxlen);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif
