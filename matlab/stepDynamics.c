#include "connectToServer.h"
#include <stdio.h>
#ifdef WIN32
#include <winsock2.h>
#else
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>
#endif

#define MAXLENGTH 500

#include "mex.h"

void mexFunction(int nlhs,       mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
  char buf[MAXLENGTH];
  double time;
  double *pind;
  int sockd;

   
  /* Check for proper number of arguments */
  
  if (nrhs != 0) {
    mexErrMsgTxt("stepDynamics takes no arguments");
  } else if (nlhs > 1) {
    mexErrMsgTxt("stepDynamics takes at most one output argument.");
  }

  sockd = ConnectTo("localhost",4765);

  if (sockd < 0)
    mexErrMsgTxt("Could not connect");

  Writeline(sockd,"step dynamics\n",14);

  Readline(sockd,buf,MAXLENGTH);
  sscanf(buf,"%lf\n",&time);

  plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
  pind = mxGetPr(plhs[0]);
  pind[0] = time;

  CloseConnection(sockd);
  return;
}
