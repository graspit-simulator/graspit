#ifndef TEST_H
#include <stdio.h>

#ifdef WIN32
#include <winsock2.h>
int initWinsock();
#else
#define SOCKET int
#endif

int ConnectTo( char *host, short port );
void CloseConnection(int sockd);
int Readline(int sockd, void *vptr, int maxlen);
int Writeline(int sockd, const void *vptr, int n);

#define TEST_H
#endif

