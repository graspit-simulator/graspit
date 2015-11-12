#include <stdlib.h>
#include <stdio.h>
#include "connectToServer.h"
#include "mex.h"

#ifndef WIN32
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>
#define SOCKET_ERROR -1
#else

int initWinsock()
{
  WORD      wVersionRequested;
  WSADATA   wsaData;

  wVersionRequested = MAKEWORD(1,1);
  if (WSAStartup(wVersionRequested, &wsaData) != 0)
  {   	mexErrMsgTxt("Init Failed");      return 1;     } 
  return 0;
}

#endif

/*  Read a line from a socket  */

int Readline(int sockd, void *vptr, int maxlen) {
    int n, rc;
    char    c, *buffer;

    buffer = vptr;

    for ( n = 1; n < maxlen; n++ ) {
	
	if ( (rc = recv(sockd, &c, 1,0)) == 1 ) {
	    *buffer++ = c;
	    if ( c == '\n' )
		break;
	}
	else if ( rc == 0 ) {
	    if ( n == 1 )
		return 0;
	    else
		break;
	}
	else {
	    return -1;
    }
    
	}
    *buffer = 0;
    return n;
}


/*  Write a line to a socket  */

int Writeline(int sockd, const void *vptr, int n) {
    int      nleft;
    int      nwritten;
    const char *buffer;

    buffer = vptr;
    nleft  = n;

    while ( nleft > 0 ) {
	if ( (nwritten = send(sockd, buffer, nleft,0)) == SOCKET_ERROR ) {
		return -1;
	}
	nleft  -= nwritten;
	buffer += nwritten;
    }

    return n;
}

int ConnectTo( char *host, short port ){
    int                  sck;
    struct sockaddr_in   writer;
    struct hostent       *hp;

#ifdef WIN32
    initWinsock();
#endif

    /* get host information */
    hp = gethostbyname( host );

    if( hp == 0x0 ){
	mexErrMsgTxt("Host unknown");
	return 0;
    }
    /* reset writer struct and copy over what we know */
    memset( (char *) &writer, 0, sizeof(writer) );
    memcpy( (char *) &writer.sin_addr, hp->h_addr, hp->h_length );
    writer.sin_family = hp->h_addrtype;
    writer.sin_port = htons( (short)port );
    /* create socket */
    sck = socket( AF_INET, SOCK_STREAM, 0 );
    if ( sck < 0 ){
		mexErrMsgTxt("Could not open socket.");
	    return 0;
    }
    /* Establish connection */
    if( connect( sck, (struct sockaddr *) &writer, sizeof(writer) ) < 0 )  {
	  mexErrMsgTxt("Could not establish connection.");
	  return 0;
    }
    return sck;
}

void CloseConnection(int sockd)
{
  closesocket(sockd);
#ifdef WIN32 
	WSACleanup();
#endif
}

/*
int
main(int argc,char **argv)
{
  FILE *sp;
  int done=0;
  int numRead;
  char buf[500];
  
  initWinsock();
  sp = ConnectTo("aurora",4765);
  printf("opened socket\n");
  fprintf(sp,"get contacts\n");


  while (!done) {
    fgets(buf,500,sp);
    printf("%s\n",buf);
    if (!strcmp(buf,"done\n")) done = 1;
  }
  fclose(sp);
  return 0;
}
*/
