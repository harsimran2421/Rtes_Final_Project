/***********************************************
Function to calculate time difference for profiling
Code Credit: Prof. Sam Siewert  
***********************************************/
#include "shape_detect_profiling.h"
#include "profiling_functions.h"

int delta_time(struct timespec* stop, struct timespec* start, struct timespec* delta)
{
    int delta_sec = stop->tv_sec - start->tv_sec;
    int delta_nsec = stop->tv_nsec - start->tv_nsec;
    if(delta_sec >= 0)
    {
      if(delta_nsec >= 0)
      {
        delta->tv_sec = delta_sec;
        delta->tv_nsec = delta_nsec;
      }
      else 
      {
        delta->tv_sec = delta_sec-1;
        delta->tv_nsec = NSEC_PER_SEC + delta_nsec;
      }
    }
    else 
    {
        if(delta_nsec >= 0)
        {
            delta->tv_sec = delta_sec;
            delta->tv_nsec = delta_nsec;
        }
        else 
        {
          delta->tv_sec = delta_sec-1;
          delta->tv_nsec = NSEC_PER_SEC + delta_nsec;
        }
    }
    return 1; 
}
