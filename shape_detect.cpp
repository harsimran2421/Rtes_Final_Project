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

#include<signal.h>

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
sem_t service_shape_detect_sig, service_camera_cap_sig, service_shape_verify_sig, service_audio_sig;

/*gloabal variables for image capturing and processing*/
cv::Mat imgOriginal;
raspicam::RaspiCam_Cv Camera;
cv::Mat src;
cv::Mat bw;

char *error_string = "Wrong Shape, Try Again!";

/* Global var that holds the shape detected */
detect_shape_e shape = DEFAULT_SHAPE;
uint8_t shape_order = 0;

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

void *camera_capture(void *args)
{
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
  
  while(1) { 
    sem_wait(&service_camera_cap_sig);

    bool bSuccess = Camera.grab(); // read a new frame from video

    if (!bSuccess) //if not success, break loop
    {
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
    cv::waitKey(333);
    
    sem_post(&service_shape_detect_sig);

  }
}

void *shape_detection(void *args)
{
  while(1) {
    sem_wait(&service_shape_detect_sig);

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

    //cv::imshow("src", src);
    cv::imshow("dst", dst);
    cv::waitKey(333);
   

    /* Increment the shapes detected by one only! */ 
    shape_order++;
    sem_post(&service_shape_verify_sig);
  }
}
bool verified = false;

void *shape_verify(void *params)
{
  static uint8_t int_order = 10;
  while(1) 
  {
    sem_wait(&service_shape_verify_sig);
    //printf("In threads: %s\n", __func__);
    if(int_order == shape) {
      printf("int_order: %d; shape: %d\n", int_order, shape);
      verified = true;
      int_order+= 10;
      //exit(0);
      /* post and exit */
    } else {
      /* 1. Do an audio output indicating an error
       * 2. Call the camera cap functions again! 
       */
      printf("int_order: %d; shape: %d\n", int_order, shape);
      verified = false;
    }
    
      sem_post(&service_audio_sig);
      //festival_say_text("In function shape verify, valid condition");
  }
    return NULL;
}


void *audio_output(void *params)
{   
    int heap_size = 210000; 
    int load_init_files = 3; 
    festival_initialize(2, heap_size); 
    festival_eval_command("(voice_kal_diphone)");  
  while(1)
  {
    sem_wait(&service_audio_sig);
    
    //if(shape==SHAPE_RECT)
    if(shape==SHAPE_TRI && verified == true)
      festival_say_file("./audiopart1.txt"); 
    else if(shape==SHAPE_TRI && verified == true)
      festival_say_file("./audiopart2.txt"); 
    else if(shape==SHAPE_CIRCLE && verified == true)
      festival_say_file("./audiopart3.txt"); 
    else if(shape==SHAPE_PENTA && verified == true)
      festival_say_file("./audiopart4.txt"); 
    else if(verified == false)
      festival_say_text(error_string);
  
    sem_post(&service_camera_cap_sig);
  }
  return NULL;
}

void intHandler(int dummy) {
  cout<<"Signal Handler Caught!"<<endl;  
  exit(0);
}


int main(int argc, char **argv)
{

  signal(SIGINT, intHandler);

  /* Create the services and the semaphores related to them */
	pthread_t service_camera_cap, service_shape_detect, service_shape_verify, service_audio_output;
  sem_init(&service_camera_cap_sig,0,0);
  sem_init(&service_shape_detect_sig,0,0);
  sem_init(&service_shape_verify_sig,0,0);
  sem_init(&service_audio_sig,0,0);
  /* Init. the TTS library */
  //int heap_size = 210000; 
  //int load_init_files = 1; 
  //festival_initialize(load_init_files, heap_size); 
  //festival_eval_command("(voice_kal_diphone)"); 
  //festival_say_text("hello world, creating threads...");
  //festival_say_file("./sampletextfile.txt"); 

  /* Create the threads */
	pthread_create(&service_camera_cap, NULL, camera_capture, NULL);
	pthread_create(&service_shape_detect, NULL, shape_detection, NULL);
	pthread_create(&service_shape_verify, NULL, shape_verify, NULL);
	pthread_create(&service_audio_output, NULL, audio_output, NULL);
  sem_post(&service_camera_cap_sig);

  /* Wait for the threads to expire */
  pthread_join(service_camera_cap,NULL);
  pthread_join(service_shape_detect,NULL);
  pthread_join(service_shape_verify,NULL);
  pthread_join(service_audio_output,NULL);
	
  /* Destory the created semaphores */
  sem_destroy(&service_camera_cap_sig);
  sem_destroy(&service_shape_detect_sig);
  sem_destroy(&service_shape_verify_sig);
  sem_destroy(&service_audio_sig);
  return 0;
}
