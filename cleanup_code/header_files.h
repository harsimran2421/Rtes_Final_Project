#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>

#include <raspicam/raspicam_cv.h>
#include <cmath>
#include </usr/include/festival/festival.h>

#include <iostream>

/*pthread header files*/
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> 
#include <sys/time.h> 
#include <stdlib.h> 
#include <stdio.h>
#include <signal.h>
#include <stdint.h>

/*header file for wiring pi*/
#include <wiringPi.h>

#define deadlinecamera_capture 120000
#define deadline_shape_detect 400000
#define deadline_shape_verify 10
#define deadline_audio 1300000
#define deadline_led_mat 200000 

#define NSEC_PER_SEC 1000000000

#define DATA        0 // GPIO 17 (WiringPi pin num 0)  header pin 11
#define CLOCK       3 // GPIO 22 (WiringPi pin num 3)   header pin 15
#define LOAD        4 // GPIO 23 (WiringPi pin num 4)   header pin 16


#define HRES 640
#define VRES 480

extern unsigned char disp1[10][8];
