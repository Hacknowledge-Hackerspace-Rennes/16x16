/*
 * lin_serial_con.c
 *
 *  Created on: 9 juin 2011
 *      Author: matlo

 *  License: GPLv3
 */

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>

#include <sys/time.h>

struct timeval t0, t1;

/*
 * The serial connection.
 */
int serial;

/*
 * The baud rate in bps.
 */
#define BAUDRATE B230400

/*
 * Connect to a serial port.
 */
int lin_serial_connect(char* portname)
{
  struct termios options;
  int ret = 0;

  printf("connecting to %s\n", portname);

  if ((serial = open(portname, O_RDWR | O_NOCTTY/* | O_NDELAY*/)) < 0)
  {
    printf("can't connect to %s\n", portname);
    ret = -1;
  }
  else
  {
    tcgetattr(serial, &options);
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_oflag &= ~OPOST;
    if(tcsetattr(serial, TCSANOW, &options) < 0)
    {
      printf("can't set serial port options\n");
      ret = -1;
    }
    else
    {
      printf("connected\n");
    }
  }

  return ret;
}

/*
 * Send a usb report to the serial port.
 */
int lin_serial_send(void* pdata, unsigned int size)
{
  return write(serial, (unsigned char*)pdata, size);
}

void lin_serial_close()
{
  close(serial);
}


/*
int main(int argc, char* argv[])
{
  lin_serial_connect("/dev/ttyUSB0");
  
  unsigned char c[32];
  unsigned char i=0, j;
  unsigned char cpt;

  gettimeofday(&t0, NULL);

  while(1)
  {
    //c = rand() % 0xFF;
    for(j=0; j<32; ++j)
    {
      //c[j] = i;
      //c[j] = rand() % 0xFF;
      c[j] = 0;
    }
    c[i/8] = 1<<(i%8);
    ++i;
    //i%=32;
    lin_serial_send(c, sizeof(c));

    int tdiff;

    cpt++;
    gettimeofday(&t1, NULL);

    tdiff = (t1.tv_sec * 1000000 + t1.tv_usec) - (t0.tv_sec * 1000000 + t0.tv_usec);
    if(tdiff > 1000000)
    {
      printf("%d\n", cpt);
      gettimeofday(&t0, NULL);
      cpt = 0;
    }

    //usleep(10000);
  }
  
  lin_serial_close();
  return 0;
}
*/
