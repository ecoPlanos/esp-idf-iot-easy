#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

#define CMD_MSG_LEN 7

int set_interface_attribs (int fd, int speed, int parity)
{
  struct termios tty;
  if (tcgetattr (fd, &tty) != 0)
  {
    // error_message ("error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
                                  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
  {
    // error_message ("error %d from tcsetattr", errno);
    return -1;
  }
  return 0;
}

void set_blocking (int fd, int should_block){
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
  {
    // error_message ("error %d from tggetattr", errno);
    return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0) printf("error %d setting term attributes", errno);
    // error_message ("error %d setting term attributes", errno);
}

// request: 42 4d e1 00 00 01 70 -> resp: 42 4d 00 04 e1 00 01 74
// request: 42 4d e1 00 01 01 71 -> resp: 42 4d 00 04 e1 01 01 75
// request: 42 4d e4 00 00 01 73 -> resp: 42 4d 00 04 e4 00 01 77
// request: 42 4d e4 00 01 01 74 -> resp: measurement
void main(void) {
  char *portname = "/dev/ttyUSB2";
  uint8_t resp[32] = {0x42,0x4d,0x00,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xab};
  uint8_t in_buff [CMD_MSG_LEN];
  uint8_t c;
  int read_len;
  uint8_t i;
  uint16_t checksum;
  uint8_t start[2] = {0,0};

  printf("Opening serial port to read and write...\n");
  int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0){
    // error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
    return;
  }

  printf("Setting serial port attributes...\n");
  set_interface_attribs (fd, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)
  printf("Setting serial blocking...\n");
  set_blocking (fd, 0);                // set no blocking
  printf("Going to loop!\n");

  while(1){
    printf("Waiting for command...\n");
    start[0] = 0;
    start[1] = 0;
    while(read(fd, &c, 1)) {
      printf("Received hex: 0x%x\n",c);
      start[1] = (c == 0x4d);
      in_buff[1] = c;
      if(start[1]&start[0]) {
        // read_len = read(fd, (in_buff+2), 5);
        read_len=2;
        while(read(fd, &c, 1)){
          printf("Received hex: 0x%x\n",c);
          in_buff[read_len]=c;
          read_len++;
          if(read_len==CMD_MSG_LEN) break;
        }
        if(read_len < 5){
          // error_message ("Error on message! Requested %u bytes and received %d",5,read_len);
          printf("Error on message! Requested %u bytes and received %d\n",5,read_len);
          start[0] = 0;
          start[1] = 0;
        } else {
          checksum=0;
          for(i=0;i<5;i++){
            printf("in_buff[%u]: 0x%2x",i, in_buff[i]);
            checksum+= in_buff[i];
          }
          printf("Calculated checksum: 0x%4x\n",checksum);
          printf("Received checksum High: 0x%2x\n", in_buff[5]);
          printf("Received checksum Low: 0x%2x\n", in_buff[6]);
          if(((checksum>>8)&0xff) ==  in_buff[5] && (checksum&0xff) ==  in_buff[6]){
            printf("Deteceted valid command!\n");
            for(i=0;i<CMD_MSG_LEN;i++) printf("command[%u]: 0x%2x\n", in_buff[i]);
            usleep (100000);
            write (fd, resp, 32);
          }
        }
      }
      start[0] = (c == 0x42);
       in_buff[0] = c;
    }
    usleep (500000);
  }
}
