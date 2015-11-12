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
  double value;
  double *pvals,*pind;
  int i,j,numRobots,numDOF;
  int sockd;
  mxArray *vals;

  /* Check for proper number of arguments */
  
  if (nrhs > 1) {
    mexErrMsgTxt("getDOFVals takes at most one input argument.");
  } else if (nlhs > 1) {
    mexErrMsgTxt("getDOFVals takes one output argument.");
  }
  if (nlhs == 0 ) nlhs=1;

  sockd = ConnectTo("localhost",4765);

  if (sockd < 0)
    mexErrMsgTxt("Could not connect");
 
  strcpy(buf,"getDOFVals ");

  // if no robots were specified by user, read total number of robots
  if (nrhs==0) {
	strcat(buf,"ALL\n");
	Writeline(sockd,buf,strlen(buf));

    Readline(sockd,buf,MAXLENGTH);
    sscanf(buf,"%d\n",&numRobots);
  }
  // otherwise send the body list
  else {
	numRobots = mxGetNumberOfElements(prhs[0]);
	if (numRobots>0) {
	  sprintf(numStr,"%d ",numRobots);
	  strcat(buf,numStr);

	  pind = mxGetPr(prhs[0]);
	  for (i=0;i<numRobots-1;i++) {
	    sprintf(numStr,"%d ",(int)pind[i]-1);
  		strcat(buf,numStr);
	  }
	  sprintf(numStr,"%d\n",(int)pind[i]-1);
	  strcat(buf,numStr);
	  Writeline(sockd,buf,strlen(buf));
	}
  }

  if (numRobots == 0) {
	plhs[0] = NULL;
	plhs[0] = NULL;
  }
  else {
	if (numRobots > 1){
	  for (i=0;i<nlhs;i++)
        plhs[i] = mxCreateCellArray(1,&numRobots);
    }
	
    for (i=0;i<numRobots;i++) {
	  Readline(sockd,buf,MAXLENGTH);
	   if (!strncmp(buf,"Error",5)) {
		mexErrMsgTxt(buf);
		break;
	  }

      sscanf(buf,"%d\n",&numDOF);
	  sprintf(buf,"NumDOF read: %d",numDOF);

	  if (numDOF == 0) {
		vals = mxCreateScalarDouble(0);
	  }
	  else {
	    vals = mxCreateDoubleMatrix(numDOF,1,mxREAL);
	    pvals = mxGetPr(vals);
      }
      for (j=0;j<numDOF;j++) {
        Readline(sockd,buf,MAXLENGTH);
        sscanf(buf,"%lf\n",&value);
        pvals[j] = value;
	  }
	  if (numRobots == 1) {
		plhs[0] = vals;
	  } else {
	    mxSetCell(plhs[0],i,vals);
	  }
	}
  }

  CloseConnection(sockd);
  return;
}