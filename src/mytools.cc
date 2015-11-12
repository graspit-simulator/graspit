/* standard C includes */

#include <stdio.h>
#include <stdlib.h>

/* ACIS includes */

//	Required for all ACIS functions
#include "kernel/acis.hxx"

//	Required for function(s):       all api functions
#include "kernel/kernapi/api/api.hxx"

//	Required for type(s):    	logical
//#include "kernel/logical.h" 

//      Required for function(s):       api_restore_entity_list
//					api_save_entity_list
#include "kernel/kernapi/api/kernapi.hxx"

//      Required for class:             ENTITY_LIST
#include "kernel/kerndata/lists/lists.hxx"

#include "mytools.hxx"

outcome get_ent_from_file(char *modelfile, ENTITY_LIST &entities){
  FILE *fi;
  outcome result(0);

  if ((fi = fopen(modelfile,"r")) == NULL) {
     pr_error("could't open input file");
     return outcome(API_FAILED);
   }

  result = api_restore_entity_list(fi,TRUE,entities);
  fclose(fi);

  return result;
}

outcome save_ent_to_file(char *modelfile, ENTITY_LIST &entities){
  FILE *fi;
  outcome result(0);

  if ((fi = fopen(modelfile,"w")) == NULL) {
     pr_error("could't open ouput file");
     return outcome(API_FAILED);
   }

  result = api_save_entity_list(fi,TRUE,entities);
  fclose(fi);

  return result;
}

void show_errors(outcome result, char *mess)
{
  
  if (mess == NULL)
    mess = "unidentified routine";
  
  if (!result.ok())
    {
      pr_errArgs((stderr,"Error in %s %d: %s", mess,
		 result.error_number(),
		 find_err_mess(result.error_number())));
    }
  
  
  err_mess_type *warnings;
  
  int nwarn = get_warnings(warnings);
  for(int i = 0; i < nwarn; ++i)
    {
      pr_errArgs((stderr,"Warning in %s %d : %s\n", mess,
	     warnings[i],
	     find_err_mess(warnings[i])));
    }
  
}		
