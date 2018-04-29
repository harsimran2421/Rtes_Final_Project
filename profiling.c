#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>

void main()
{
  struct timeval start;
  struct timeval stop;
  float time_executed_us;
  gettimeofday(&start,(struct timezone *)0);
  usleep(100);
  gettimeofday(&stop,(struct timezone *)0);
  time_executed_us =((stop.tv_sec - start.tv_sec)*1000000) + ((stop.tv_usec - start.tv_usec));
  printf("%f\n",time_executed_us);
}/
