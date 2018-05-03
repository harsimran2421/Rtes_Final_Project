/** 
 * @file: header_file.h
 * @Author Arundhathi Swami, Harsimransingh Bindra & Vidur Sarin
 * @Version 0.1
 *
 * @copyright
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * @section DESCRIPTION
 *
 * This is the basic header file for the shape detection and profiling program.
 * It contain all the included headers(.h files) and basic extern declarations for all the variables used.
 *
 * @section REFERENCES
 *  Shape Detection: https://github.com/bsdnoobz/opencv-code/blob/master/shape-detect.cpp
 *  GPIO and LED Matrix Wiring: https://github.com/WiringPi/WiringPi/blob/master/wiringPi/wiringPiSPI.c
 *
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

/*header file for wiring pi*/
#include <wiringPi.h>


#define DATA        0 // GPIO 17 (WiringPi pin num 0)   header pin 11
#define CLOCK       3 // GPIO 22 (WiringPi pin num 3)   header pin 15
#define LOAD        4 // GPIO 23 (WiringPi pin num 4)   header pin 16


using namespace std;
using namespace cv;

/** 
 * @brief
 * Basic enum to list the valid shapes that are detected by the system
 */
typedef enum __detect_shape_e {
  SHAPE_RECT = 10,
  SHAPE_TRI = 20,
  SHAPE_CIRCLE = 30,
  SHAPE_PENTA = 40,
  SHAPE_HEXA = 50,
  DEFAULT_SHAPE = 60
} detect_shape_e;

/** 
 * @brief
 * Global variable that holds the shape detected 
 */
detect_shape_e shape = SHAPE_CIRCLE;

/**
 * @brief
 * Initialising the Semaphores for all the tasks
 */
sem_t service_shape_detect_sig, service_camera_cap_sig, service_shape_verify_sig, service_audio_sig, service_led_mat_sig;

/**
 * @brief
 * Gloabal variables for image capturing and processing
 */
cv::Mat imgOriginal;
raspicam::RaspiCam_Cv Camera;
cv::Mat src;
cv::Mat bw;

/**
 * @brief
 * 2d array for led matrix
 */
unsigned char disp1[10][8]={
  {0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x3C},  ///<0
  {0x10,0x30,0x50,0x10,0x10,0x10,0x10,0x10},  ///<1
  {0x7E,0x2,0x2,0x7E,0x40,0x40,0x40,0x7E},    ///<2
  {0x3E,0x2,0x2,0x3E,0x2,0x2,0x3E,0x0},       ///<3
  {0x8,0x18,0x28,0x48,0xFE,0x8,0x8,0x8},      ///<4
  {0x3C,0x20,0x20,0x3C,0x4,0x4,0x3C,0x0},     ///<5
  {0x3C,0x20,0x20,0x3C,0x24,0x24,0x3C,0x0},   ///<6
  {0x3E,0x22,0x4,0x8,0x8,0x8,0x8,0x8},        ///<7
  {0x0,0x3E,0x22,0x22,0x3E,0x22,0x22,0x3E},   ///<8
  {0x3E,0x22,0x22,0x3E,0x2,0x2,0x2,0x3E},     ///<9
};

/*global variable to pass the number of sides to led_mat thread*/
int sides = 2;
int prio_Max = 0;
int prio_Min = 0;
bool verified = false;

/** 
 * @brief
 * A global var to hold the present shape detected and to help match it to the expected order 
 */
uint8_t shape_order = 0;

/** 
 * @brief
 * This function is used to determine the angles detected in the image. This is critical to detect the 
 * correct shape in the captured frame.
 * @param pt1 one point of reference
 * @param pt2 another point of ref.
 * @param pt3 third ref point to complete the triangle.
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);

/**  
 * @brief
 * This function can be used to display an overlay label on the shape detected. It makes sure that the 
 * label is centered on the detected shape.
 * @param label string that you want to display at the center of the detected shape
 * @param &contour pass the contour for the detected shape 
 */
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);

/** 
 * @brief
 * This function can be used to send 2 bytes of data via the SPI interface to the LED Matrix. 
 * Generally, the first byte in this case is the address and the second byte is the data that you want to store in that address space.
 *
 * @param the 16bit number that you want to send to the SPI peripheral
 */
static void Send16bits (unsigned short output);

/** 
 * @brief
 * This function can be used to send a single byte to the peripheral via the SPI driver 
 *
 * @param reg_number the register number 
 * @param dataout the data to be entered into the register
 */
static void MAX7219Send (unsigned char reg_number, unsigned char dataout);

/** 
 * @brief
 * This is the intialization function for the LED Matrix 
 * @param void
 */
void setup_led(void);

/** 
 * @brief
 *  This function will print the passes number on the led matrix after picking up the correct seq. from the array
 * @param number to be printed.
 */

void printnumber(int i);

/** 
 * @brief
 * This is the camera capture thread that is responsible for frame capture and conversion to greyscale. 
 * @param NULL
 */
void *camera_capture(void *args);

/** 
 * @brief
 * This function will detect the correct shape after running canny and contour finding algorithms.
 * @param NULL
 */
void *shape_detection(void *args);

/** 
 * @brief
 * this function will verify the shape detected by the detection algorithm and will only continue if the correct shape has been verified.
 * Else, it recalls the previous service.
 * @param NULL
 */
void *shape_verify(void *params);

/** 
 * @brief
 * this function is responsible for the display on the LED matrix. 
 * @param NULL
 */
void *led_matrix(void *params);

/** 
 * @brief
 * this is the service responsible for the audio output. It calls the library functions for the festival audio library.
 * @param NULL
 */
void *audio_output(void *params);

/** 
 * @brief
 * This function configures all the threads as required. It configs all the threads to run in SCHED_FIFO and assigns default priority to them.
 * @param NULL
 */
void set_thread_attr(void);


