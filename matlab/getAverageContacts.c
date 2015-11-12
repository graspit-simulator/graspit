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
  char numStr[20];
  double x,y,z,tx,ty,tz;
  double *pind,*pind1;
  int i,numBodies;
  int sockd;

   
  /* Check for proper number of arguments */
  
  if (nrhs > 1) {
    mexErrMsgTxt("get average contacts takes at most one input argument.");
  } else if (nlhs > 2) {
    mexErrMsgTxt("get average contacts requires at most two output arguments");
  }

  sockd = ConnectTo("localhost",4765);

  if (sockd < 0)
    mexErrMsgTxt("Could not connect");

  strcpy(buf,"getAverageContacts ");
  if (nrhs==0) {
	strcat(buf,"ALL\n");
	Writeline(sockd,buf,strlen(buf));

    Readline(sockd,buf,MAXLENGTH);
    sscanf(buf,"%d\n",&numBodies);
  }
  else {
	numBodies = mxGetNumberOfElements(prhs[0]);
	if (numBodies>0) {
	  sprintf(numStr,"%d ",numBodies);
	  strcat(buf,numStr);
	  pind = mxGetPr(prhs[0]);
	  for (i=0;i<numBodies-1;i++) {
	    sprintf(numStr,"%d ",(int)pind[i]-1);
  	    strcat(buf,numStr);
	  }
	  sprintf(numStr,"%d\n",(int)pind[i]-1);
	  strcat(buf,numStr);
	  Writeline(sockd,buf,strlen(buf));
	}
  }

  if (numBodies == 0) {
	plhs[0] = NULL;
	plhs[0] = NULL;
  }
  else {
    plhs[0] = mxCreateDoubleMatrix(numBodies,6, mxREAL);
    pind = mxGetPr(plhs[0]);

	if (nlhs > 1) {
	  plhs[1] = mxCreateDoubleMatrix(numBodies,3, mxREAL);
	  pind1 = mxGetPr(plhs[1]);
    }

    for (i=0;i<numBodies;i++) {
      Readline(sockd,buf,MAXLENGTH);
      sscanf(buf,"%lf %lf %lf %lf %lf %lf\n",&x,&y,&z,&tx,&ty,&tz);
      pind[i] = x;
      pind[numBodies+i] = y;
      pind[2*numBodies+i] = z;
      pind[3*numBodies+i] = tx;
      pind[4*numBodies+i] = ty;
      pind[5*numBodies+i] = tz;
	  Readline(sockd,buf,MAXLENGTH);
	  if (nlhs>1) {
		sscanf(buf,"%lf %lf %lf\n",&x,&y,&z);

		pind1[i] = x;
		pind1[numBodies+i] = y;
		pind1[2*numBodies+i] = z;
	  }
	}
  }

  CloseConnection(sockd);
  return;
}
