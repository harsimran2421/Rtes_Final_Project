#include "header_files.h"

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
