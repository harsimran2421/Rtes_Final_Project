/** 
 * @file: profiling_functions.c 
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
 * This file contains the definitions of most of the functions that are critical to profile the 
 * program and all the services that are a part of it. Moreover, it also contains definitions for the following:
 *  - Thread specific settings and functions 
 *  - Definitions for all the threads and their computations 
 *  - Audio and LED Matrix specific functions and tasks
 *
 * @section REFERENCES
 *  Shape Detection: https://github.com/bsdnoobz/opencv-code/blob/master/shape-detect.cpp
 *  GPIO and LED Matrix Wiring: https://github.com/WiringPi/WiringPi/blob/master/wiringPi/wiringPiSPI.c
 *
 */
#include "header_files.h"

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
detect_shape_e shape = SHAPE_CIRCLE;

bool verified = false;

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

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;

  return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
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
  while(frame_count < noofframes) { 
    sem_wait(&service_camera_cap_sig);
  
	  clock_gettime(CLOCK_REALTIME, &start_time); 
    
    bool bSuccess = Camera.grab(); // read a new frame from video

    if (!bSuccess) //if not success, break loop
    {
  if (wiringPiSetup () == -1) exit (1) ;
      cout << "Cannot read a frame from video stream" << endl;
    }

    Camera.retrieve(imgOriginal);
    imshow("image",imgOriginal);
    string imagename;
    imagename = "/home/pi/Desktop/test2/assests/basic_shapes2.jpg";	
    
    imgOriginal.copyTo(src);
    if (src.empty())
    {		
      cout << "ended " << endl;
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


  while(frame_count < noofframes) { 
    sem_wait(&service_shape_detect_sig);
    clock_gettime(CLOCK_REALTIME, &start_time); 

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
        } else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27) {
          shape = SHAPE_PENTA;
          setLabel(dst, "PENTA", contours[i]);
        } else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45) {
          shape = SHAPE_HEXA;
          setLabel(dst, "HEXA", contours[i]);
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
        }
      }
    }

    cv::imshow("dst", dst);
    cv::waitKey(333);
   

    /* Increment the shapes detected by one only! */ 
    shape_order++;
    
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

void *shape_verify(void *params)
{
  static uint8_t int_order = 10;
  struct timeval start;
  struct timeval stop;
  unsigned long int time_executed_us = 0;
  double current_time;


  while(frame_count < noofframes) { 
    sem_wait(&service_shape_verify_sig);
    
    clock_gettime(CLOCK_REALTIME, &start_time); 
    
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
  while(frame_count < noofframes) { 
    sem_wait(&service_led_mat_sig);
    delay(600);
    
    clock_gettime(CLOCK_REALTIME, &start_time); 
    
    printnumber(sides);
    
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
  while(frame_count < noofframes) { 
    sem_wait(&service_audio_sig);
    clock_gettime(CLOCK_REALTIME, &start_time); 

    if(shape==SHAPE_RECT && verified == true)
      festival_say_text("rectangle");
    else if(shape==SHAPE_TRI && verified == true)
      festival_say_text("triangle");
    else if(shape==SHAPE_CIRCLE && verified == true)
      festival_say_text("circle");
    else if(shape==SHAPE_PENTA && verified == true)
      festival_say_text("pentagon");
    else if(shape==SHAPE_HEXA && verified == true)
      festival_say_text("hexagon");
    else if(verified == false)
      festival_say_text("error");

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

    sem_post(&service_camera_cap_sig);
  }
  return NULL;
}

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
    audio_thread_param.sched_priority =  prio_Max; 
    pthread_attr_setschedparam(&audio_attr, &audio_thread_param); 
    
    pthread_attr_init(&led_mat_attr); 
    pthread_attr_setinheritsched(&led_mat_attr, PTHREAD_EXPLICIT_SCHED); 
    pthread_attr_setschedpolicy(&led_mat_attr, SCHED_FIFO); 
    led_mat_thread_param.sched_priority = prio_Max; 
    pthread_attr_setschedparam(&led_mat_attr, &led_mat_thread_param); 
}
