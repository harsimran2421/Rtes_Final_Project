/** 
 * @file: shape_detect_profiling.h
 * @Authors: Arundhathi Swami, Harsimransingh Bindra & Vidur Sarin
 * @Version 0.1
 *
 * \copyright  
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * @section DESCRIPTION
 *
 * This is the basic header file for the shape detection and profiling program.
 * It contain all the global variables and arrays used by the program along with prototype 
 * declarations for all the functions.
 *
 * @section REFERENCES
 *  Shape Detection: https://github.com/bsdnoobz/opencv-code/blob/master/shape-detect.cpp
 *  GPIO and LED Matrix Wiring: https://github.com/WiringPi/WiringPi/blob/master/wiringPi/wiringPiSPI.c
 *
 */
#include "header_files.h"

using namespace std;
using namespace cv;

/**
 * @brief Semaphore Declarations
 * initialising the semaphores
 */
sem_t service_shape_detect_sig, service_camera_cap_sig, service_shape_verify_sig, service_audio_sig, service_led_mat_sig;

/**
 * @brief Global OpenCV Declarations
 * gloabal variables for image capturing and processing
 */
cv::Mat imgOriginal;
raspicam::RaspiCam_Cv Camera;
cv::Mat src;
cv::Mat bw;

/**
 * @brief LED Matrix Array
 * 2D array for the led matrix 
 */
unsigned char disp1[10][8]={
  {0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x3C},  /*0*/
  {0x10,0x30,0x50,0x10,0x10,0x10,0x10,0x10},  /*1*/
  {0x7E,0x2,0x2,0x7E,0x40,0x40,0x40,0x7E},    /*2*/
  {0x3E,0x2,0x2,0x3E,0x2,0x2,0x3E,0x0},       /*3*/
  {0x8,0x18,0x28,0x48,0xFE,0x8,0x8,0x8},      /*4*/
  {0x3C,0x20,0x20,0x3C,0x4,0x4,0x3C,0x0},     /*5*/
  {0x3C,0x20,0x20,0x3C,0x24,0x24,0x3C,0x0},   /*6*/
  {0x3E,0x22,0x4,0x8,0x8,0x8,0x8,0x8},        /*7*/
  {0x0,0x3E,0x22,0x22,0x3E,0x22,0x22,0x3E},   /*8*/
  {0x3E,0x22,0x22,0x3E,0x2,0x2,0x2,0x3E},     /*9*/
};

/** 
 * @brief
 * global variable to pass the number of sides to led_mat thread 
 */
int sides = 0;
int frame_count = 0;
int prio_Max = 0;
int prio_Min = 0;

/** 
 * @brief
 * Global var that holds the shape detected 
 */
uint8_t shape_order = 0;

/**
 * @brief
 * Arrays for storing jitter   
 */
float camera_capture_jitter[100]; 
float shape_detect_jitter[100]; 
float shape_verify_jitter[100]; 
float audio_jitter[100]; 
float led_mat_jitter[100]; 

/**
 * @brief
 * jitter analysis variables for all three transforms
 */
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

/**
 * @brief
 * shape_verify profiling variables
 */ 
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

/**
 * @brief
 * shape_detect profiling variables 
 */
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

/**
 * @brief
 * Audio profiling variables
 */
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

/**
 * @brief
 * led matrix profiling variables
 */
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

/**
 * @brief
 * variables used for profiling of frame captures
 */
struct timespec start_time = {0,0};
struct timespec end_time = {0,0};
struct timespec delta_t = {0,0};
long milliseconds_time = 0; 
uint8_t noofframes = 20; 


/**
 * @brief
 * attributes for all the threads and services 
 */
pthread_attr_t camera_thread_attr, shape_detect_attr, shape_verify_attr, audio_attr, led_mat_attr; 

/**
 * @brief
 * scheduling parameters for all the threads and services
 */
struct sched_param camera_thread_param, shape_detect_thread_param, shape_verify_thread_param,\
                     audio_thread_param, led_mat_thread_param; 

/**
 * @brief
 * This function can be used to calculate the difference between the start and stop times for a service. 
 * It is a fundamental function when it comes to profiling services using the gettimeofdata or the clock_gettime functions.
 *
 * @Authors
 * Sam Siewert
 */
int delta_time(struct timespec* stop, struct timespec* start, struct timespec* delta);

/**
 * @brief
 * This function can be used to set overlays on the shape detected by OpenCV. 
 */
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);

/**
 * @brief
 * This function can be used to send 2 bytes to the SPI interface used for the LED Matrix. 
 * This is especially useful when we want to put data into an address space on the SPI driver
 */
static void Send16bits (unsigned short output);

/**
 * @brief
 * This function can be used to put data into a particular register of the SPI interface.
 */
static void MAX7219Send (unsigned char reg_number, unsigned char dataout);

/**
 * @brief
 * Function to setup the LED and the basic initialization registers that are required for its successful operation 
 */
void setup_led();

/**
 * @brief
 * This function prints the number on the Matrix and is responsible for turning on the correct LEDs
 */
void printnumber(int i);

/**
 * @brief
 * This thread is responsible for capturing the camera images and frames. This is the first thread run right after setup 
 * and has the highest rate.
 */
void *camera_capture(void *args);

/**
 * @brief
 * Responsible for running the transform algos on the captured frame
*/
 void *shape_detection(void *args);

/**
 * @brief
 * Verify that the frame is correct and is part of the list of valid frames for the system.
*/
void *shape_verify(void *params);

/**
 * @brief
 * This task is responsible for bit banging the correct data to the SPI driver and displaying the correct number of sides on the led device.
*/
void *led_matrix(void *params);

/**
 * @brief
 * Responsible for the calling the correct audio libraries and functions to announce the word or the detected shape
*/
void *audio_output(void *params);

/**
 * @brief
 * Responsible for detecting the shape of the device. Runs the contour and hough algos to do that.
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
