/**
 * Simple shape detector program.
 * It loads an image and tries to find simple shapes (rectangle, triangle, circle, etc) in it.
 * This program is a modified version of `squares.cpp` found in the OpenCV sample dir.
 */

#include "shape_detect_profiling.h"

#if 0

#endif


void intHandler(int dummy) {
  cout<<"Signal Handler Caught!"<<endl;  
  exit(0);
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

  setup_led();

  pthread_create(&service_camera_cap, &camera_thread_attr, camera_capture, NULL);
	pthread_create(&service_shape_detect, &shape_detect_attr, shape_detection, NULL);
	pthread_create(&service_shape_verify, &shape_verify_attr, shape_verify, NULL);
	pthread_create(&service_audio_output, &audio_attr, audio_output, NULL);
  pthread_create(&service_led_mat, &led_mat_attr, led_matrix, NULL);
  sem_post(&service_camera_cap_sig);

  /* Wait for the threads to expire */
  pthread_join(service_camera_cap,NULL);
  pthread_join(service_shape_detect,NULL);
  pthread_join(service_shape_verify,NULL);
  pthread_join(service_audio_output,NULL);
  pthread_join(service_led_mat, NULL);
	
  /* calcuate averages */ 
  avg_execution_camera_capture = total_execution_camera_capture / NUM_PROFILE_COUNTS; 
  avg_pjitter_camera_capture = total_pjitter_camera_capture / count_pjitter_camera_capture;
  avg_njitter_camera_capture= total_njitter_camera_capture / count_njitter_camera_capture;
  avg_jitter_camera_capture = total_jitter_camera_capture / NUM_PROFILE_COUNTS; 

  avg_execution_shape_detect = total_execution_shape_detect / NUM_PROFILE_COUNTS; 
  avg_pjitter_shape_detect = total_pjitter_shape_detect / count_pjitter_shape_detect;
  avg_njitter_shape_detect= total_njitter_shape_detect / count_njitter_shape_detect;
  avg_jitter_shape_detect = total_jitter_shape_detect / NUM_PROFILE_COUNTS; 
  
  avg_execution_shape_verify = total_execution_shape_verify / NUM_PROFILE_COUNTS; 
  avg_pjitter_shape_verify = total_pjitter_shape_verify / count_pjitter_shape_verify;
  avg_njitter_shape_verify= total_njitter_shape_verify / count_njitter_shape_verify;
  avg_jitter_shape_verify = total_jitter_shape_verify / NUM_PROFILE_COUNTS; 
  
  avg_execution_audio = total_execution_audio / NUM_PROFILE_COUNTS; 
  avg_pjitter_audio = total_pjitter_audio / count_pjitter_audio;
  avg_njitter_audio= total_njitter_audio / count_njitter_audio;
  avg_jitter_audio = total_jitter_audio / NUM_PROFILE_COUNTS; 
  
  avg_execution_led_mat = total_execution_led_mat / NUM_PROFILE_COUNTS; 
  avg_pjitter_led_mat = total_pjitter_led_mat / count_pjitter_led_mat;
  avg_njitter_led_mat= total_njitter_led_mat / count_njitter_led_mat;
  avg_jitter_led_mat = total_jitter_led_mat / NUM_PROFILE_COUNTS; 
  
  /* print jitter analysis results */     	
    printf("\n\n JITTER ANALYSIS FOR %d x %d RESOLUTION FOR CANNY TRANSFORM, HOUGH ELLIPTICAL TRANSFORM AND HOUGH INTERACTIVE TRANSFORM\n\n", HRES, VRES); 
    printf("---------------------------------------------------------\n");
    printf("Frame No | Camera Service | Shape Detection Service | Shape Verification Service | Audio Service | Led Matrix Serivce\n");  
    for( uint8_t l = 0; l<20; l++)
    {
      printf(" %d       | %0.2f              | %0.2f             | %0.2f              |%0.2f             |%0.2f\n", l, camera_capture_jitter[l], shape_detect_jitter[l], shape_verify_jitter[l], audio_jitter[l], led_mat_jitter[l]); 
    }

    printf("\n\n---------------------------------------------------------\n");
    printf("Frame No | Camera Service | Shape Detection Service | Shape Verification Service | Audio Service | Led Matrix Serivce\n");  

    printf("Avg Execution Time |%0.2f          |%0.2f             |%0.2f              |%0.2f            |%0.2f\nAvg Jitter         |%0.2f                |%0.2f                |%0.2f                |%0.2f            |%0.2f\nAvg Positive Jitter|%0.2f               |%0.2f                |%0.2f                |%0.2f            |%0.2f\nAvg Negative Jitter|%0.2f                |%0.2f                |%0.2f             |%0.2f              |%0.2f\n", avg_execution_camera_capture, avg_execution_shape_detect, avg_execution_shape_verify, avg_execution_audio, avg_execution_led_mat, avg_jitter_camera_capture, avg_jitter_shape_detect, avg_jitter_shape_verify, avg_jitter_audio, avg_jitter_led_mat, avg_pjitter_camera_capture, avg_pjitter_shape_detect, avg_pjitter_shape_verify, avg_pjitter_audio,avg_pjitter_led_mat, avg_njitter_camera_capture, avg_njitter_shape_detect, avg_njitter_shape_verify, avg_njitter_audio, avg_njitter_led_mat); 
   
  /* Destory the created semaphores */
  sem_destroy(&service_camera_cap_sig);
  sem_destroy(&service_shape_detect_sig);
  sem_destroy(&service_shape_verify_sig);
  sem_destroy(&service_audio_sig);
  sem_destroy(&service_led_mat_sig);
  return 0;
}
