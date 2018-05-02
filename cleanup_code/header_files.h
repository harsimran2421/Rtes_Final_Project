/** 
 * @file: header_file.h
 * @Authors: Arundhathi Swami, Harsimransingh Bindra & Vidur Sarin
 * @Version 0.1
 *
 * @section LICENSE 
 *  
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * @section DESCRIPTION
 *
 * This is the basic header file for the shape detection and profiling program.
 * It contain all the included headers(.h files) and basic extern declarations for all the variables used.
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

#define NUM_PROFILE_COUNTS 20

extern int sides;
extern int frame_count;
extern int prio_Max;
extern int prio_Min;
extern unsigned char disp1[10][8];
extern uint8_t noofframes;
extern struct timespec start_time;
extern struct timespec end_time;
extern struct timespec delta_t;

//detect_shape_e shape;

extern sem_t service_shape_detect_sig, service_camera_cap_sig, service_shape_verify_sig, service_audio_sig, service_led_mat_sig;

/*camera_capture*/ 
extern float avg_execution_camera_capture; 
extern float total_execution_camera_capture;
extern float avg_jitter_camera_capture; 
extern float total_jitter_camera_capture; 
extern float avg_pjitter_camera_capture; 
extern float total_pjitter_camera_capture; 
extern float avg_njitter_camera_capture; 
extern float total_njitter_camera_capture; 
extern uint32_t count_pjitter_camera_capture; 
extern uint32_t count_njitter_camera_capture; 

/*shape_verify*/ 
extern float avg_execution_shape_verify; 
extern float total_execution_shape_verify; 
extern float avg_jitter_shape_verify; 
extern float total_jitter_shape_verify; 
extern float avg_pjitter_shape_verify; 
extern float total_njitter_shape_verify; 
extern float avg_njitter_shape_verify; 
extern float total_pjitter_shape_verify; 
extern uint32_t count_pjitter_shape_verify; 
extern uint32_t count_njitter_shape_verify; 

/*shape_detect*/
extern float avg_execution_shape_detect; 
extern float total_execution_shape_detect; 
extern float avg_jitter_shape_detect; 
extern float total_jitter_shape_detect; 
extern float avg_pjitter_shape_detect; 
extern float total_pjitter_shape_detect; 
extern float avg_njitter_shape_detect; 
extern float total_njitter_shape_detect; 
extern uint32_t count_pjitter_shape_detect; 
extern uint32_t count_njitter_shape_detect; 

/*Audio*/
extern float avg_execution_audio; 
extern float total_execution_audio; 
extern float avg_jitter_audio; 
extern float total_jitter_audio; 
extern float avg_pjitter_audio; 
extern float total_pjitter_audio; 
extern float avg_njitter_audio; 
extern float total_njitter_audio; 
extern uint32_t count_pjitter_audio; 
extern uint32_t count_njitter_audio; 

/*led matrix*/
extern float avg_execution_led_mat; 
extern float total_execution_led_mat; 
extern float avg_jitter_led_mat; 
extern float total_jitter_led_mat; 
extern float avg_pjitter_led_mat; 
extern float total_pjitter_led_mat; 
extern float avg_njitter_led_mat; 
extern float total_njitter_led_mat; 
extern uint32_t count_pjitter_led_mat; 
extern uint32_t count_njitter_led_mat; 


extern cv::Mat imgOriginal;
extern raspicam::RaspiCam_Cv Camera;
extern cv::Mat src;
extern cv::Mat bw;

extern float camera_capture_jitter[100]; 
extern float shape_detect_jitter[100]; 
extern float shape_verify_jitter[100]; 
extern float audio_jitter[100]; 
extern float led_mat_jitter[100]; 

extern uint8_t shape_order;

extern pthread_attr_t camera_thread_attr, shape_detect_attr, shape_verify_attr, audio_attr, led_mat_attr; 

extern struct sched_param camera_thread_param, shape_detect_thread_param, shape_verify_thread_param,\
                     audio_thread_param, led_mat_thread_param; 
