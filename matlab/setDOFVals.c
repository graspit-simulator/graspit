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
  double *pind,*poutvals;
  const double *pvals;
  int i,j,numRobots,numDOF;
  int sockd;
  const mxArray *vals,*stepby;
  mxArray *outvals;

  /* Check for proper number of arguments */
  
  if (nrhs < 2) {
    mexErrMsgTxt("setDOFVals takes at least two input arguments.");
  } 
  if (nrhs > 3) {
	mexErrMsgTxt("setDOFVals takes at most three input arguments.");
  }
  if (nlhs > 1) {
    mexErrMsgTxt("setDOFVals takes at most one output argument.");
  }
  if (nlhs == 0 ) nlhs=1;

  // more argument checking is needed 

  sockd = ConnectTo("localhost",4765);

  if (sockd < 0)
    mexErrMsgTxt("Could not connect");
 
  strcpy(buf,"setDOFVals ");

  // if no robots were specified by user, read total number of robots
  if (nrhs<3) {
	strcat(buf,"ALL ");
	if (mxIsCell(plhs[0]))
	  numRobots = mxGetNumberOfElements(plhs[0]);
	else numRobots = 1;
	  
  }
  // otherwise send the body list
  else {
	numRobots = mxGetNumberOfElements(prhs[2]);
	if (numRobots>0) {
	  sprintf(numStr,"%d ",numRobots);
	  strcat(buf,numStr);

	  pind = mxGetPr(prhs[2]);
	  for (i=0;i<numRobots-1;i++) {
	    sprintf(numStr,"%d ",(int)pind[i]-1);
  		strcat(buf,numStr);
	  }
	  sprintf(numStr,"%d\n",(int)pind[i]-1);
	  strcat(buf,numStr);
	}
  }

  if (numRobots > 1) {
	if (!mxIsCell(prhs[0])  || !mxIsCell(prhs[1]))
	  mexErrMsgTxt("Arguments 1 and 2 should be cell arrays.");
	if (mxGetNumberOfElements(prhs[0]) != numRobots)
	  mexErrMsgTxt("Argument 1 should have a cell for each robot.");
	if (mxGetNumberOfElements(prhs[1]) != numRobots)
	  mexErrMsgTxt("Argument 2 should have a cell for each robot.");
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
	  if (numRobots>1) {
		vals = mxGetCell(prhs[0],i);
        stepby = mxGetCell(prhs[1],i);
	  }
      else {
		vals = prhs[0];
		stepby = prhs[1];
	  }
      
	  numDOF = mxGetNumberOfElements(vals);
      sprintf(numStr,"%d\n",numDOF);

	  pvals = mxGetPr(vals);
	  sprintf(buf,"%d\n",numDOF);
	  Writeline(sockd,buf,strlen(buf));
      
	  
	  for (j=0;j<numDOF;j++) {
        sprintf(buf,"%lf\n",pvals[j]);
        Writeline(sockd,buf,strlen(buf));
	  }
      pvals = mxGetPr(stepby);

	  for (j=0;j<numDOF;j++) {
        sprintf(buf,"%lf\n",pvals[j]);
        Writeline(sockd,buf,strlen(buf));
	  }

	  outvals = mxCreateDoubleMatrix(numDOF,1,mxREAL);
	  poutvals = mxGetPr(outvals);

	  for (j=0;j<numDOF;j++) {
        Readline(sockd,buf,MAXLENGTH);
		if (!strncmp(buf,"Error",5)) {
		  mexErrMsgTxt(buf);
		}
        sscanf(buf,"%lf\n",&value);
        poutvals[j] = value;
	  }
	  
	  if (numRobots == 1) {
		plhs[0] = outvals;
	  } else {
	    mxSetCell(plhs[0],i,outvals);
	  }
	}
  }

  CloseConnection(sockd);
  return;
}