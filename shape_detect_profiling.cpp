/**
 * Simple shape detector program.
 * It loads an image and tries to find simple shapes (rectangle, triangle, circle, etc) in it.
 * This program is a modified version of `squares.cpp` found in the OpenCV sample dir.
 */

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

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;

  return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

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


/***********************************************
Function to calculate time difference for profiling
Code Credit: Prof. Sam Siewert  
***********************************************/
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

/**
 * Helper function to display text in the center of a contour
 */
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 0.4;
  int thickness = 1;
  int baseline = 0;

  cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
  cv::Rect r = cv::boundingRect(contour);

  cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
  cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
  cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

/*functions for led matrix code*/
static void Send16bits (unsigned short output)
{

  unsigned char i;

  for (i=16; i>0; i--) 
  {
    unsigned short mask = 1 << (i - 1); // calculate bitmask

    digitalWrite(CLOCK, 0);  // set clock to 0

    // Send one bit on the data pin

    if (output & mask)   
      digitalWrite(DATA, 1);          
    else                              
      digitalWrite(DATA, 0);  

    digitalWrite(CLOCK, 1);  // set clock to 1

  }

}


// Take a reg numer and data and send to the max7219

static void MAX7219Send (unsigned char reg_number, unsigned char dataout)
{
  digitalWrite(LOAD, 1);  // set LOAD 1 to start
  Send16bits((reg_number << 8) + dataout);   // send 16 bits ( reg number + dataout )
  digitalWrite(LOAD, 0);  // LOAD 0 to latch
  digitalWrite(LOAD, 1);  // set LOAD 1 to finish
}


void setup_led()
{
  MAX7219Send(0x09, 0x00);
  MAX7219Send(0x0a, 0x10);
  MAX7219Send(0x0b, 0x07);
  MAX7219Send(0x0c, 0x01);
  MAX7219Send(0x0f, 0x00);
}

void printnumber(int i)
{
  //cout << "In print Number\n" <<endl; 
  for(int j = 1; j<9 ;j++)
  {
    MAX7219Send(j, disp1[i][j-1]);
  }
  //delay(50);
}



void *camera_capture(void *args)
{

  struct timeval start;
  struct timeval stop;
  unsigned long int time_executed_us = 0;
  double current_time;
  
  
  Camera.set(CV_CAP_PROP_FRAME_WIDTH,320);
  Camera.set(CV_CAP_PROP_FRAME_HEIGHT,240);
  Camera.set(CV_CAP_PROP_BRIGHTNESS,50);
  Camera.set(CV_CAP_PROP_CONTRAST,50);
  Camera.set(CV_CAP_PROP_SATURATION,50);
  Camera.set(CV_CAP_PROP_GAIN,50);
  Camera.set(CV_CAP_PROP_FORMAT,CV_8UC3);
  Camera.set(CV_CAP_PROP_EXPOSURE,-1);
  Camera.set(CV_CAP_PROP_WHITE_BALANCE_RED_V,-1);
  Camera.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U,-1);

  if ( !Camera.open())  // if not success, exit program
  {
    cout << "Cannot open the web cam" << endl;
  }
  int n = 0;
  //while(1) { 
  while(frame_count < noofframes) { 
    sem_wait(&service_camera_cap_sig);
  
	  clock_gettime(CLOCK_REALTIME, &start_time); 
    
    //gettimeofday(&start, (struct timezone *)0);

    bool bSuccess = Camera.grab(); // read a new frame from video

    if (!bSuccess) //if not success, break loop
    {
  if (wiringPiSetup () == -1) exit (1) ;
      cout << "Cannot read a frame from video stream" << endl;
      //break;
    }

    Camera.retrieve(imgOriginal);
    imshow("image",imgOriginal);
    string imagename;
    imagename = "/home/pi/Desktop/test2/assests/basic_shapes2.jpg";	

    //cv::Mat src = cv::imread("polygon.png");

    //cv::Mat src = cv::imread("/home/pi/Desktop/test2/assets/basic_shapes.jpg");
    imgOriginal.copyTo(src);
    if (src.empty())
    {		
      cout << "ended " << endl;
      //return -1;
      //break;
    }

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(src, gray, CV_BGR2GRAY);

    // Use Canny instead of threshold to catch squares with gradient shading
    cv::Canny(gray, bw, 0, 50, 5);
    cv::waitKey(3); 
    
    clock_gettime(CLOCK_REALTIME, &end_time); 
    delta_time(&end_time, &start_time, &delta_t);
    float exec_time = float(delta_t.tv_sec * 1000000) + float(delta_t.tv_nsec / 1000);
    total_execution_camera_capture += exec_time; 
    if (frame_count >= 0)
    {
      camera_capture_jitter[frame_count] = exec_time - deadlinecamera_capture; 
      total_jitter_camera_capture += camera_capture_jitter[frame_count]; 
      if(exec_time > deadlinecamera_capture) //positive jitter 
      {
        count_pjitter_camera_capture += 1;
        total_pjitter_camera_capture += camera_capture_jitter[frame_count]; 
      }
      else //negative jitter 
      {
        count_njitter_camera_capture += 1;
        total_njitter_camera_capture += camera_capture_jitter[frame_count];
      }
    }   
    //gettimeofday(&stop, (struct timezone *)0);
    
    //time_executed_us =((stop.tv_sec - start.tv_sec)*1000000) + ((stop.tv_usec - start.tv_usec));
    n++;
    
    frame_count++;
    
    sem_post(&service_shape_detect_sig);
  }
}

void *shape_detection(void *args)
{
  struct timeval start;
  struct timeval stop;
  unsigned long int time_executed_us = 0;
  double current_time;


  //while(1) {
  while(frame_count < noofframes) { 
    sem_wait(&service_shape_detect_sig);
    clock_gettime(CLOCK_REALTIME, &start_time); 
    //gettimeofday(&start, (struct timezone *)0);

    // Find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> approx;
    cv::Mat dst = src.clone();

    for (int i = 0; i < contours.size(); i++) {

      // Approximate contour with accuracy proportional
      // to the contour perimeter
      cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

      // Skip small or non-convex objects 
      if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
        continue;

      if	(approx.size() < 3) { //detects annything with three sides ==>needs to be looked at 
        cout << "none" <<endl;
        setLabel(dst, "NONE", contours[i]);    // Triangles
      } else if (approx.size() == 3) { //detects annything with three sides ==>needs to be looked at 
        shape = SHAPE_TRI;
        setLabel(dst, "TRI", contours[i]);    // Triangles
        //sides = 3;
      } else if (approx.size() >= 4 && approx.size() <= 6) {

        // Number of vertices of polygonal curve
        int vtc = approx.size();

        // Get the cosines of all corners
        std::vector<double> cos;

        for (int j = 2; j < vtc+1; j++)
          cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

        // Sort ascending the cosine values
        std::sort(cos.begin(), cos.end());

        // Get the lowest and the highest cosine
        double mincos = cos.front();
        double maxcos = cos.back();

        // Use the degrees obtained above and the number of vertices
        // to determine the shape of the contour
        if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3) {
          shape = SHAPE_RECT;
          setLabel(dst, "RECT", contours[i]);
          //sides = 4;
        } else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27) {
          shape = SHAPE_PENTA;
          setLabel(dst, "PENTA", contours[i]);
          //sides = 5;
        } else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45) {
          shape = SHAPE_HEXA;
          setLabel(dst, "HEXA", contours[i]);
          //sides = 6;
        }
      } else {

        // Detect and label circles
        double area = cv::contourArea(contours[i]);

        cv::Rect r = cv::boundingRect(contours[i]);
        int radius = r.width / 2;
        if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
            std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2) {
          shape = SHAPE_CIRCLE;  
          setLabel(dst, "CIR", contours[i]);
          //sides = 0;
        }
      }
    }

    //cv::imshow("src", src);
    cv::imshow("dst", dst);
    cv::waitKey(333);
   

    /* Increment the shapes detected by one only! */ 
    shape_order++;
    //gettimeofday(&stop, (struct timezone *)0);
    clock_gettime(CLOCK_REALTIME, &end_time); 
    delta_time(&end_time, &start_time, &delta_t);
    float exec_time = float(delta_t.tv_sec * 1000000) + float(delta_t.tv_nsec / 1000);
    total_execution_shape_detect += exec_time; 
    
    if (frame_count >= 0)
    {
      shape_detect_jitter[frame_count] = exec_time - deadline_shape_detect;  
      total_jitter_shape_detect += shape_detect_jitter[frame_count]; 
      if(exec_time > deadline_shape_detect) //positive jitter 
      {
        count_pjitter_shape_detect += 1;
        total_pjitter_shape_detect += shape_detect_jitter[frame_count]; 
      }
      else //negative jitter 
      {
        count_njitter_shape_detect += 1;
        total_njitter_shape_detect += shape_detect_jitter[frame_count];
      }
    }
 
    sem_post(&service_shape_verify_sig);
  }
}

bool verified = false;

void *shape_verify(void *params)
{
  static uint8_t int_order = 10;
  struct timeval start;
  struct timeval stop;
  unsigned long int time_executed_us = 0;
  double current_time;


  //while(1) 
  while(frame_count < noofframes) { 
    sem_wait(&service_shape_verify_sig);
    //printf("In threads: %s\n", __func__);
    //gettimeofday(&start, (struct timezone *)0);
    clock_gettime(CLOCK_REALTIME, &start_time); 
#ifdef EXIT   
    if(int_order == shape) {
      //printf("int_order: %d; shape: %d\n", int_order, shape);
      verified = true;
      int_order+= 10;
      //exit(0);
      /* post and exit */
    } else {
      /* 1. Do an audio output indicating an error
       * 2. Call the camera cap functions again! 
       */
    //  printf("int_order: %d; shape: %d\n", int_order, shape);
      verified = false;
    }
#endif

    if ( shape == SHAPE_RECT || shape == SHAPE_TRI || shape == SHAPE_CIRCLE || shape == SHAPE_PENTA || shape == SHAPE_HEXA )
    {
      verified = true;

      if(shape==SHAPE_RECT && verified == true)
        sides = 4;
      else if(shape==SHAPE_CIRCLE && verified == true)
        sides = 0; 
      else if(shape==SHAPE_TRI && verified == true)
        sides = 3; 

      else if(shape==SHAPE_PENTA && verified == true)
        sides = 5; 
      else if(shape==SHAPE_HEXA && verified == true)
        sides = 6; 
    }
    else
    {
       verified = false;
    }
    
    //gettimeofday(&stop, (struct timezone *)0);
    clock_gettime(CLOCK_REALTIME, &end_time); 
    delta_time(&end_time, &start_time, &delta_t);
    float exec_time = float(delta_t.tv_sec * 1000000) + float(delta_t.tv_nsec / 1000);
    total_execution_shape_verify += exec_time; 

    if (frame_count >= 0)
    {
      shape_verify_jitter[frame_count] = exec_time - deadline_shape_verify;  
      total_jitter_shape_verify += shape_verify_jitter[frame_count]; 
      if(exec_time > deadline_shape_verify) //positive jitter 
      {
        count_pjitter_shape_verify += 1;
        total_pjitter_shape_verify += shape_verify_jitter[frame_count]; 
      }
      else //negative jitter 
      {
        count_njitter_shape_verify += 1;
        total_njitter_shape_verify += shape_verify_jitter[frame_count];
      }
    }

      sem_post(&service_led_mat_sig);
  }
    return NULL;
}

void *led_matrix(void *params)
{
  int side1;
    //if (wiringPiSetup () == -1) exit (1) ;
    //setup_led();
    //while(1)
  while(frame_count < noofframes) { 
    sem_wait(&service_led_mat_sig);
    delay(600);
    clock_gettime(CLOCK_REALTIME, &start_time); 
    // if(side1 != sides)
    // {
    //sides = 5;
    //delay(600);
    printnumber(sides);
    //delay(600);
    // }
    // side1 = sides; 

    clock_gettime(CLOCK_REALTIME, &end_time); 
    delay(600);
    delta_time(&end_time, &start_time, &delta_t);
    float exec_time = float(delta_t.tv_sec * 1000000) + float(delta_t.tv_nsec / 1000);
    total_execution_led_mat += exec_time; 

    if (frame_count >= 0)
    {
      led_mat_jitter[frame_count] = exec_time - deadline_led_mat;  
      total_jitter_led_mat += led_mat_jitter[frame_count]; 
      if(exec_time > deadline_led_mat) //positive jitter 
      {
        count_pjitter_led_mat += 1;
        total_pjitter_led_mat += led_mat_jitter[frame_count]; 
      }
      else //negative jitter 
      {
        count_njitter_led_mat += 1;
        total_njitter_led_mat += led_mat_jitter[frame_count];
      }
    }

    sem_post(&service_audio_sig);
  }
  return NULL; 

}

void *audio_output(void *params)
{ 
    struct timeval start;
    struct timeval stop;
    unsigned long int time_executed_us = 0;
    double current_time;

  
    int heap_size = 210000; 
    int load_init_files = 3; 
    festival_initialize(2, heap_size); 
    festival_eval_command("(voice_kal_diphone)");  
  //while(1)
  //{
  while(frame_count < noofframes) { 
    sem_wait(&service_audio_sig);
    clock_gettime(CLOCK_REALTIME, &start_time); 
    //gettimeofday(&start, (struct timezone *)0);

    //if(shape==SHAPE_RECT)
    if(shape==SHAPE_RECT && verified == true)
      //festival_say_file("./audiopart1.txt"); 
      festival_say_text("rectangle");
    else if(shape==SHAPE_TRI && verified == true)
      //festival_say_file("./audiopart2.txt"); 
      festival_say_text("triangle");
    else if(shape==SHAPE_CIRCLE && verified == true)
      //festival_say_file("./audiopart3.txt"); 
      festival_say_text("circle");
    else if(shape==SHAPE_PENTA && verified == true)
      festival_say_text("pentagon");
    else if(shape==SHAPE_HEXA && verified == true)
      festival_say_text("hexagon");
    //festival_say_file("./audiopart4.txt"); 
    else if(verified == false)
      festival_say_text("error");

    //gettimeofday(&stop, (struct timezone *)0);
    clock_gettime(CLOCK_REALTIME, &end_time); 
    delta_time(&end_time, &start_time, &delta_t);
    float exec_time = float(delta_t.tv_sec * 1000000) + float(delta_t.tv_nsec / 1000);
    total_execution_audio += exec_time; 

    if (frame_count >= 0)
    {
      audio_jitter[frame_count] = exec_time - deadline_audio;  
      total_jitter_audio += audio_jitter[frame_count]; 
      if(exec_time > deadline_audio) //positive jitter 
      {
        count_pjitter_audio += 1;
        total_pjitter_audio += audio_jitter[frame_count]; 
      }
      else //negative jitter 
      {
        count_njitter_audio += 1;
        total_njitter_audio += audio_jitter[frame_count];
      }
    }

    //time_executed_us =((stop.tv_sec - start.tv_sec)*1000000) + ((stop.tv_usec - start.tv_usec));
    //printf("Service_4 time: %d\n", time_executed_us);
    //delay(100);
    sem_post(&service_camera_cap_sig);
  }
  return NULL;
}

void intHandler(int dummy) {
  cout<<"Signal Handler Caught!"<<endl;  
  exit(0);
}

pthread_attr_t camera_thread_attr, shape_detect_attr, shape_verify_attr, audio_attr, led_mat_attr; 

struct sched_param camera_thread_param, shape_detect_thread_param, shape_verify_thread_param,\
                     audio_thread_param, led_mat_thread_param; 

void set_thread_attr(void)
{
    pthread_attr_init(&camera_thread_attr); 
    pthread_attr_setinheritsched(&camera_thread_attr, PTHREAD_EXPLICIT_SCHED); 
    pthread_attr_setschedpolicy(&camera_thread_attr, SCHED_FIFO); 
    camera_thread_param.sched_priority = prio_Max; 
    pthread_attr_setschedparam(&camera_thread_attr, &camera_thread_param); 
    
    pthread_attr_init(&shape_detect_attr); 
    pthread_attr_setinheritsched(&shape_detect_attr, PTHREAD_EXPLICIT_SCHED); 
    pthread_attr_setschedpolicy(&shape_detect_attr, SCHED_FIFO); 
    shape_detect_thread_param.sched_priority = prio_Max; 
    pthread_attr_setschedparam(&shape_detect_attr, &shape_detect_thread_param); 
    
    pthread_attr_init(&shape_verify_attr); 
    pthread_attr_setinheritsched(&shape_verify_attr, PTHREAD_EXPLICIT_SCHED); 
    pthread_attr_setschedpolicy(&shape_verify_attr, SCHED_FIFO); 
    shape_verify_thread_param.sched_priority = prio_Max; 
    pthread_attr_setschedparam(&shape_verify_attr, &shape_verify_thread_param); 
    
    pthread_attr_init(&audio_attr); 
    pthread_attr_setinheritsched(&audio_attr, PTHREAD_EXPLICIT_SCHED); 
    pthread_attr_setschedpolicy(&audio_attr, SCHED_FIFO); 
    audio_thread_param.sched_priority = prio_Max; 
    pthread_attr_setschedparam(&audio_attr, &audio_thread_param); 
    
    pthread_attr_init(&led_mat_attr); 
    pthread_attr_setinheritsched(&led_mat_attr, PTHREAD_EXPLICIT_SCHED); 
    pthread_attr_setschedpolicy(&led_mat_attr, SCHED_FIFO); 
    led_mat_thread_param.sched_priority = prio_Max; 
    pthread_attr_setschedparam(&led_mat_attr, &led_mat_thread_param); 
}

int main(int argc, char **argv)
{

  signal(SIGINT, intHandler);
  
  prio_Max = sched_get_priority_max(SCHED_FIFO); 
  prio_Min = sched_get_priority_min(SCHED_FIFO); 
 
  set_thread_attr();

  /* Create the services and the semaphores related to them */
	pthread_t service_camera_cap, service_shape_detect, service_shape_verify, service_audio_output, service_led_mat;
  sem_init(&service_camera_cap_sig,0,0);
  sem_init(&service_shape_detect_sig,0,0);
  sem_init(&service_shape_verify_sig,0,0);
  sem_init(&service_audio_sig,0,0);
  sem_init(&service_led_mat_sig,0,0);
  
  if (wiringPiSetup () == -1) exit (1) ;
  pinMode(DATA, OUTPUT);  
  pinMode(CLOCK, OUTPUT);
  pinMode(LOAD, OUTPUT);  
 /* Init. the TTS library */
  //int heap_size = 210000; 
  //int load_init_files = 1; 
  //festival_initialize(load_init_files, heap_size); 
  //festival_eval_command("(voice_kal_diphone)"); 
  //festival_say_text("hello world, creating threads...");
  //festival_say_file("./sampletextfile.txt"); 

  setup_led();

  /* Create the threads */
#if 0
  pthread_create(&service_camera_cap, NULL, camera_capture, NULL);
	pthread_create(&service_shape_detect, NULL, shape_detection, NULL);
	pthread_create(&service_shape_verify, NULL, shape_verify, NULL);
	pthread_create(&service_audio_output, NULL, audio_output, NULL);
  pthread_create(&service_led_mat, NULL, led_matrix, NULL);
  sem_post(&service_camera_cap_sig);
#endif
#if 1 
  pthread_create(&service_camera_cap, &camera_thread_attr, camera_capture, NULL);
	pthread_create(&service_shape_detect, &shape_detect_attr, shape_detection, NULL);
	pthread_create(&service_shape_verify, &shape_verify_attr, shape_verify, NULL);
	pthread_create(&service_audio_output, &audio_attr, audio_output, NULL);
  pthread_create(&service_led_mat, &led_mat_attr, led_matrix, NULL);
  sem_post(&service_camera_cap_sig);
#endif

  /* Wait for the threads to expire */
  pthread_join(service_camera_cap,NULL);
  pthread_join(service_shape_detect,NULL);
  pthread_join(service_shape_verify,NULL);
  pthread_join(service_audio_output,NULL);
  pthread_join(service_led_mat, NULL);
	
  /* calcuate averages */ 
  avg_execution_camera_capture = total_execution_camera_capture / 20; 
  avg_pjitter_camera_capture = total_pjitter_camera_capture / count_pjitter_camera_capture;
  avg_njitter_camera_capture= total_njitter_camera_capture / count_njitter_camera_capture;
  avg_jitter_camera_capture = total_jitter_camera_capture / 20; 

  avg_execution_shape_detect = total_execution_shape_detect / 20; 
  avg_pjitter_shape_detect = total_pjitter_shape_detect / count_pjitter_shape_detect;
  avg_njitter_shape_detect= total_njitter_shape_detect / count_njitter_shape_detect;
  avg_jitter_shape_detect = total_jitter_shape_detect / 20; 
  
  avg_execution_shape_verify = total_execution_shape_verify / 20; 
  avg_pjitter_shape_verify = total_pjitter_shape_verify / count_pjitter_shape_verify;
  avg_njitter_shape_verify= total_njitter_shape_verify / count_njitter_shape_verify;
  avg_jitter_shape_verify = total_jitter_shape_verify / 20; 
  
  avg_execution_audio = total_execution_audio / 20; 
  avg_pjitter_audio = total_pjitter_audio / count_pjitter_audio;
  avg_njitter_audio= total_njitter_audio / count_njitter_audio;
  avg_jitter_audio = total_jitter_audio / 20; 
  
  avg_execution_led_mat = total_execution_led_mat / 20; 
  avg_pjitter_led_mat = total_pjitter_led_mat / count_pjitter_led_mat;
  avg_njitter_led_mat= total_njitter_led_mat / count_njitter_led_mat;
  avg_jitter_led_mat = total_jitter_led_mat / 20; 
  /* print jitter analysis results */     	
    printf("\n\n JITTER ANALYSIS FOR %d x %d RESOLUTION FOR CANNY TRANSFORM, HOUGH ELLIPTICAL TRANSFORM AND HOUGH INTERACTIVE TRANSFORM\n\n", HRES, VRES); 
    printf("---------------------------------------------------------\n");
    printf("Frame No | Camera Service | Shape Detection Service | Shape Verification Service | Audio Service | Led Matrix Serivce\n");  
    for( uint8_t l = 0; l<20; l++)
    {
      printf(" %d       | %0.2f                       | %0.2f                       | %0.2f                  |%0.2f                 |%0.2f\n", l, camera_capture_jitter[l], shape_detect_jitter[l], shape_verify_jitter[l], audio_jitter[l], led_mat_jitter[l]); 
    }

    printf("\n\n---------------------------------------------------------\n");
    printf("Frame No | Camera Service | Shape Detection Service | Shape Verification Service | Audio Service | Led Matrix Serivce\n");  

    printf("Avg Execution Time |%0.2f                       |%0.2f                        |%0.2f                |%0.2f                |%0.2f\nAvg Jitter         |%0.2f                        |%0.2f                       |%0.2f                |%0.2f                |%0.2f\nAvg Positive Jitter|%0.2f                        |%0.2f                       |%0.2f                |%0.2f            |%0.2f\nAvg Negative Jitter|%0.2f                        |%0.2f                       |%0.2f          |%0.2f              |%0.2f\n", avg_execution_camera_capture, avg_execution_shape_detect, avg_execution_shape_verify, avg_execution_audio, avg_execution_led_mat, avg_jitter_camera_capture, avg_jitter_shape_detect, avg_jitter_shape_verify, avg_jitter_audio, avg_jitter_led_mat, avg_pjitter_camera_capture, avg_pjitter_shape_detect, avg_pjitter_shape_verify, avg_pjitter_audio,avg_pjitter_led_mat, avg_njitter_camera_capture, avg_njitter_shape_detect, avg_njitter_shape_verify, avg_njitter_audio, avg_njitter_led_mat); 
   
  /* Destory the created semaphores */
  sem_destroy(&service_camera_cap_sig);
  sem_destroy(&service_shape_detect_sig);
  sem_destroy(&service_shape_verify_sig);
  sem_destroy(&service_audio_sig);
  sem_destroy(&service_led_mat_sig);
  return 0;
}
