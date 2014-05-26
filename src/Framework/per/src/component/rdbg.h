#ifndef RDBG__H
#define RDBG__H
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <strings.h>
#include <string.h>


using std::string;
inline void rdbg(const char* str)
{
  string ip = "127.0.0.1";
  int port = 28005;
  int sockfd;
  struct sockaddr_in servaddr;
  sockfd=socket(AF_INET,SOCK_DGRAM,0);
  bzero(&servaddr,sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr=inet_addr(ip.c_str());
  servaddr.sin_port=htons(port);
  sendto(sockfd,str,strlen(str),0,(struct sockaddr *)&servaddr,sizeof(servaddr));
  close(sockfd);

}




#endif