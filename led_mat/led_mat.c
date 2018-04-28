#include<stdio.h>
#include<stdlib.h>
#include<stdint.h>
#include<string.h>
#include<stdbool.h>
//#include<unistd.h>
#include <wiringPi.h>
//#include <pigpio.h>

// bcm pin 22  or wiring pi pin 3 fo rclock
// bcm pin 27 or wiring pi pin 2 is cs
//bcm pin 17 wiring pi pin 0 is Din
//
//
const int din = 12; 

const int cs = 10; 
const int clk = 14;

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

void write_byte(unsigned char data)
{
  digitalWrite(cs,LOW);
  for(int i = 8; i>=1; i--)
  {
    /*gpioWrite(22,0);
    gpioWrite(17,data & 0x80);
    data = data<<1;
    gpioWrite(22,1);*/
  digitalWrite(clk,LOW);
  digitalWrite(din,data & 0x80);
  data = data <<1;
  digitalWrite(clk,HIGH);
  } 
}

void write(unsigned char address, unsigned char data)
{

  digitalWrite(cs,LOW);
  write_byte(address);
  write_byte(data);
  digitalWrite(cs,HIGH);
}


void setup_led()
{
  write(0x09,0x01);
  write(0x0a, 0x01);
  write(0x0b,0x07 );
  write(0x0c, 0x01);
  write(0x0f, 0x00);
}

void printnumber(int i)
{
  for(int j = 1; j<9 ;j++)
  {
    write(j, disp1[i][j-1]);
  }
  delay(5000);
}

int main()
{

 /* if (gpioInitialise() < 0)
  {
    fprintf(stderr, "pigpio initialisation failed\n");
    return 1;
  }*/
  wiringPiSetupGpio();
  /*gpioSetMode(22,PI_OUTPUT);
  gpioSetMode(27,PI_OUTPUT);
  gpioSetMode(17,PI_OUTPUT);*/
  pinMode(din,OUTPUT);
  pinMode(cs,OUTPUT);
  pinMode(clk,OUTPUT);
  char data =0x88;
  //gpioWrite(27,0);
  setup_led();
  printnumber(5);
 return 0;
}
