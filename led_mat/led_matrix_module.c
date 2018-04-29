/*
   max7219.c
   Raspberry Pi driving the Max7219
   to compile : gcc max7219.c -o max7219 -lwiringPi
 */
     
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/time.h> 
#include <semaphore.h>
     // define our pins :

#define DATA        0 // GPIO 17 (WiringPi pin num 0)  header pin 11
#define CLOCK       3 // GPIO 22 (WiringPi pin num 3)   header pin 15
#define LOAD        4 // GPIO 23 (WiringPi pin num 4)   header pin 16


// The Max7219 Registers :

#define DECODE_MODE   0x09                       
#define INTENSITY     0x0a                        
#define SCAN_LIMIT    0x0b                        
#define SHUTDOWN      0x0c                        
#define DISPLAY_TEST  0x0f                         

sem_t service;

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


unsigned char disp1[11][8]={
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
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//9
};

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
  for(int j = 1; j<9 ;j++)
  {
    MAX7219Send(j, disp1[i][j-1]);
  }
  delay(50);
}

int k = 0;
void *led_mat(void *params)
{
  while(1)
  {
    sem_wait(&service);
    printnumber(k);
    //printnumber(11);
  }
  return 0;
}
int main (void)
{

  pthread_t service_led_mat;
  sem_init(&service,0,0);
  printf ("\n\nRaspberry Pi Max7219 Test using WiringPi\n\n");

  if (wiringPiSetup () == -1) exit (1) ;

  //We need 3 output pins to control the Max7219: Data, Clock and Load

  pinMode(DATA, OUTPUT);  
  pinMode(CLOCK, OUTPUT);
  pinMode(LOAD, OUTPUT);  

  //MAX7219Send(SCAN_LIMIT, 7);     // set up to scan all eight digits


  /* 
     BCD decode mode off : data bits correspond to the segments (A-G and DP) of the seven segment display.
     BCD mode on :  0 to 15 =  0 to 9, -, E, H, L, P, and ' '

  */
  
  setup_led();
  pthread_create(&service_led_mat, 0 , led_mat, 0);
  for(k = 0; k <9 ;k++)
  {
    sem_post(&service);
    delay(100);
  }
    pthread_join(service_led_mat,NULL);
  //MAX7219Send(1,6);      // displays the number 6 on digit 1
  return 0;
}
