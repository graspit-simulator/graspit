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
  double *pwrench,*ploc,*perr,*pind;
  int i,j,numBodies,numContacts;
  int sockd;
  mxArray *wrench,*loc,*err;

  /* Check for proper number of arguments */
  
  if (nrhs > 1) {
    mexErrMsgTxt("get average contacts takes at most one input argument.");
  } else if (nlhs > 3) {
    mexErrMsgTxt("get average contacts takes at most three output arguments.");
  }

  sockd = ConnectTo("localhost",4765);

  if (sockd < 0)
    mexErrMsgTxt("Could not connect");

  // send how many types of date we're collecting
  if (nlhs == 0 ) nlhs=1;
  sprintf(buf,"getContacts %d ",nlhs);

  // if no bodies were specified by user, read total number of bodies
  if (nrhs==0) {
	strcat(buf,"ALL\n");
	Writeline(sockd,buf,strlen(buf));

    Readline(sockd,buf,MAXLENGTH);
    sscanf(buf,"%d\n",&numBodies);
  }
  // otherwise send the body list
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
	if (numBodies > 1){
	  for (i=0;i<nlhs;i++)
        plhs[i] = mxCreateCellArray(1,&numBodies);
    }
	
    for (i=0;i<numBodies;i++) {
	  Readline(sockd,buf,MAXLENGTH);
      sscanf(buf,"%d\n",&numContacts);

	  if (numContacts == 0) {
		wrench = mxCreateScalarDouble(0);
		if (nlhs > 1) {loc = mxCreateScalarDouble(0);}
	    if (nlhs > 2) {err = mxCreateScalarDouble(0);}
	  }
	  else {
	    wrench = mxCreateDoubleMatrix(numContacts,6,mxREAL);
	    pwrench = mxGetPr(wrench);
	    if (nlhs > 1) {loc = mxCreateDoubleMatrix(numContacts,3,mxREAL); ploc=mxGetPr(loc);}
	    if (nlhs > 2) {err = mxCreateDoubleMatrix(numContacts,1,mxREAL); perr=mxGetPr(err);}
      }

      for (j=0;j<numContacts;j++) {
        Readline(sockd,buf,MAXLENGTH);
        sscanf(buf,"%lf %lf %lf %lf %lf %lf\n",&x,&y,&z,&tx,&ty,&tz);
        pwrench[j] = x;
        pwrench[numContacts+j] = y;
        pwrench[2*numContacts+j] = z;
        pwrench[3*numContacts+j] = tx;
        pwrench[4*numContacts+j] = ty;
        pwrench[5*numContacts+j] = tz;
		if (nlhs > 1) {
		  Readline(sockd,buf,MAXLENGTH);
		  sscanf(buf,"%lf %lf %lf\n",&x,&y,&z);

  	      ploc[j] = x;
	      ploc[numContacts+j] = y;
	      ploc[2*numContacts+j] = z;
		}
		if (nlhs > 2) {
		  Readline(sockd,buf,MAXLENGTH);
		  sscanf(buf,"%lf\n",perr+j);
        }
	  }
	  if (numBodies == 1) {
		plhs[0] = wrench;
        if (nlhs > 1) plhs[1] = loc;
        if (nlhs > 2) plhs[2] = err;
	  } else {
	    mxSetCell(plhs[0],i,wrench);
	    if (nlhs > 1) mxSetCell(plhs[1],i,loc);
	    if (nlhs > 2) mxSetCell(plhs[2],i,err);
	  }
	}
  }

  CloseConnection(sockd);
  return;

}