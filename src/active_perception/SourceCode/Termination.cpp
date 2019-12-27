/*
 * Termination.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: pourya
 */

//Headers
#include "Termination.h"

//Defining the function to handle termination signals
void terminationHandler(int signum)
{
  //Printing the reason of exiting
  fprintf(stderr, "\nEnding the program. The reason is: ");
  if(signum==SIGINT)
    fprintf(stderr, "The program was interrupted, possibly by the user (SIGINT raised)\n");
  else if(signum==SIGTERM)
    fprintf(stderr, "Termination request received by the program (SIGTERM raised)\n");
  else if(signum==SIGABRT)
    fprintf(stderr, "Abnormality in the program execution (SIGABRT raised)\n");
  else
    fprintf(stderr, "Unknown\n");

  //Exiting the program
  exit(EXIT_FAILURE);
}

/*
//Defining the function to run at exit (by exit or return commands)
void atExitHandler(void)
{
}
*/
