#include "header_files.h"

using namespace std;
using namespace cv;
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
float avg_execution_camera_capture = 0; 
float total_execution_camera_capture = 0;
float avg_jitter_camera_capture = 0; 
float total_jitter_camera_capture = 0; 
float avg_pjitter_camera_capture = 0; 
float total_pjitter_camera_capture = 0; 
float avg_njitter_camera_capture = 0; 
float total_njitter_camera_capture = 0; 
uint32_t count_pjitter_camera_capture = 0; 
uint32_t count_njitter_camera_capture = 0; 

/*shape_verify*/ 
float avg_execution_shape_verify = 0; 
float total_execution_shape_verify = 0; 
float avg_jitter_shape_verify = 0; 
float total_jitter_shape_verify = 0; 
float avg_pjitter_shape_verify = 0; 
float total_njitter_shape_verify = 0; 
float avg_njitter_shape_verify = 0; 
float total_pjitter_shape_verify = 0; 
uint32_t count_pjitter_shape_verify = 0; 
uint32_t count_njitter_shape_verify = 0; 

/*shape_detect*/
float avg_execution_shape_detect = 0; 
float total_execution_shape_detect = 0; 
float avg_jitter_shape_detect = 0; 
float total_jitter_shape_detect = 0; 
float avg_pjitter_shape_detect = 0; 
float total_pjitter_shape_detect = 0; 
float avg_njitter_shape_detect = 0; 
float total_njitter_shape_detect = 0; 
uint32_t count_pjitter_shape_detect = 0; 
uint32_t count_njitter_shape_detect = 0; 

/*Audio*/
float avg_execution_audio = 0; 
float total_execution_audio = 0; 
float avg_jitter_audio = 0; 
float total_jitter_audio = 0; 
float avg_pjitter_audio = 0; 
float total_pjitter_audio = 0; 
float avg_njitter_audio = 0; 
float total_njitter_audio = 0; 
uint32_t count_pjitter_audio = 0; 
uint32_t count_njitter_audio = 0; 

/*led matrix*/
float avg_execution_led_mat = 0; 
float total_execution_led_mat = 0; 
float avg_jitter_led_mat = 0; 
float total_jitter_led_mat = 0; 
float avg_pjitter_led_mat = 0; 
float total_pjitter_led_mat = 0; 
float avg_njitter_led_mat = 0; 
float total_njitter_led_mat = 0; 
uint32_t count_pjitter_led_mat = 0; 
uint32_t count_njitter_led_mat = 0; 

/***********************************************
variables used for profiling of frame captures
***********************************************/
struct timespec start_time = {0,0};
struct timespec end_time = {0,0};
struct timespec delta_t = {0,0};
//long delta_time = 0;
long milliseconds_time = 0; 
uint8_t noofframes = 20; 

pthread_attr_t camera_thread_attr, shape_detect_attr, shape_verify_attr, audio_attr, led_mat_attr; 

struct sched_param camera_thread_param, shape_detect_thread_param, shape_verify_thread_param,\
                     audio_thread_param, led_mat_thread_param; 

int delta_time(struct timespec* stop, struct timespec* start, struct timespec* delta);

void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);

static void Send16bits (unsigned short output);

static void MAX7219Send (unsigned char reg_number, unsigned char dataout);

void setup_led();

void printnumber(int i);

void set_thread_attr(void);

void *camera_capture(void *args);

void *shape_detection(void *args);

void *shape_verify(void *params);

void *led_matrix(void *params);

void *audio_output(void *params);

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
