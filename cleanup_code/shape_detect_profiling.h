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
//#include "shit_header_file.h"

/*header file for wiring pi*/
#include <wiringPi.h>

#define DATA        0 // GPIO 17 (WiringPi pin num 0)  header pin 11
#define CLOCK       3 // GPIO 22 (WiringPi pin num 3)   header pin 15
#define LOAD        4 // GPIO 23 (WiringPi pin num 4)   header pin 16

#define deadlinecamera_capture 120000
#define deadline_shape_detect 400000
#define deadline_shape_verify 10
#define deadline_audio 1300000
#define deadline_led_mat 200000 

#define NSEC_PER_SEC 1000000000

#define HRES 640
#define VRES 480

using namespace std;
using namespace cv;

typedef enum __detect_shape_e {
  SHAPE_RECT = 10,
  SHAPE_TRI = 20,
  SHAPE_CIRCLE = 30,
  SHAPE_PENTA = 40,
  SHAPE_HEXA = 50,
  DEFAULT_SHAPE = 60
} detect_shape_e;

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */

/*initialising the semaphores*/
sem_t service_shape_detect_sig, service_camera_cap_sig, service_shape_verify_sig, service_audio_sig, service_led_mat_sig;

/*gloabal variables for image capturing and processing*/
cv::Mat imgOriginal;
raspicam::RaspiCam_Cv Camera;
cv::Mat src;
cv::Mat bw;

/*2d array for led matrix*/
unsigned char disp1[10][8]={
  {0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x3C},//0
  {0x10,0x30,0x50,0x10,0x10,0x10,0x10,0x10},//1
  {0x7E,0x2,0x2,0x7E,0x40,0x40,0x40,0x7E},//2
  {0x3E,0x2,0x2,0x3E,0x2,0x2,0x3E,0x0},//3
  {0x8,0x18,0x28,0x48,0xFE,0x8,0x8,0x8},//4
  {0x3C,0x20,0x20,0x3C,0x4,0x4,0x3C,0x0},//5
  {0x3C,0x20,0x20,0x3C,0x24,0x24,0x3C,0x0},//6
  {0x3E,0x22,0x4,0x8,0x8,0x8,0x8,0x8},//7
  {0x0,0x3E,0x22,0x22,0x3E,0x22,0x22,0x3E},//8
  {0x3E,0x22,0x22,0x3E,0x2,0x2,0x2,0x3E},//9
};
/*global variable to pass the number of sides to led_mat thread*/
int sides = 0;
int frame_count = 0;
int prio_Max = 0;
int prio_Min = 0;

char *error_string = "Wrong Shape, Try Again!";

/* Global var that holds the shape detected */
detect_shape_e shape = SHAPE_CIRCLE;
uint8_t shape_order = 0;

/***********************************************
Arrays for storing jitter   
***********************************************/
float camera_capture_jitter[100]; 
float shape_detect_jitter[100]; 
float shape_verify_jitter[100]; 
float audio_jitter[100]; 
float led_mat_jitter[100]; 

/***********************************************
jitter analysis variables for all three transforms
***********************************************/
/*camera_capture*/ 
static float avg_execution_camera_capture = 0; 
static float total_execution_camera_capture = 0;
static float avg_jitter_camera_capture = 0; 
static float total_jitter_camera_capture = 0; 
static float avg_pjitter_camera_capture = 0; 
static float total_pjitter_camera_capture = 0; 
static float avg_njitter_camera_capture = 0; 
static float total_njitter_camera_capture = 0; 
static uint32_t count_pjitter_camera_capture = 0; 
static uint32_t count_njitter_camera_capture = 0; 

/*shape_verify*/ 
static float avg_execution_shape_verify = 0; 
static float total_execution_shape_verify = 0; 
static float avg_jitter_shape_verify = 0; 
static float total_jitter_shape_verify = 0; 
static float avg_pjitter_shape_verify = 0; 
static float total_njitter_shape_verify = 0; 
static float avg_njitter_shape_verify = 0; 
static float total_pjitter_shape_verify = 0; 
static uint32_t count_pjitter_shape_verify = 0; 
static uint32_t count_njitter_shape_verify = 0; 

/*shape_detect*/
static float avg_execution_shape_detect = 0; 
static float total_execution_shape_detect = 0; 
static float avg_jitter_shape_detect = 0; 
static float total_jitter_shape_detect = 0; 
static float avg_pjitter_shape_detect = 0; 
static float total_pjitter_shape_detect = 0; 
static float avg_njitter_shape_detect = 0; 
static float total_njitter_shape_detect = 0; 
static uint32_t count_pjitter_shape_detect = 0; 
static uint32_t count_njitter_shape_detect = 0; 

/*Audio*/
static float avg_execution_audio = 0; 
static float total_execution_audio = 0; 
static float avg_jitter_audio = 0; 
static float total_jitter_audio = 0; 
static float avg_pjitter_audio = 0; 
static float total_pjitter_audio = 0; 
static float avg_njitter_audio = 0; 
static float total_njitter_audio = 0; 
static uint32_t count_pjitter_audio = 0; 
static uint32_t count_njitter_audio = 0; 

/*led matrix*/
static float avg_execution_led_mat = 0; 
static float total_execution_led_mat = 0; 
static float avg_jitter_led_mat = 0; 
static float total_jitter_led_mat = 0; 
static float avg_pjitter_led_mat = 0; 
static float total_pjitter_led_mat = 0; 
static float avg_njitter_led_mat = 0; 
static float total_njitter_led_mat = 0; 
static uint32_t count_pjitter_led_mat = 0; 
static uint32_t count_njitter_led_mat = 0; 

/***********************************************
variables used for profiling of frame captures
***********************************************/
static struct timespec start_time = {0,0};
static struct timespec end_time = {0,0};
static struct timespec delta_t = {0,0};
//long delta_time = 0;
long milliseconds_time = 0; 
uint8_t noofframes = 20; 
