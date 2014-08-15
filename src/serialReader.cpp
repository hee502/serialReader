///////////////////////////////////////////////////////////////////////////////
//Source for talker node to send basic commands to AR-Drone		     					 //
//v1.0									     																								 //
//First creation							    																					 //
//Huseyin Emre Erdem																										     //
//04.08.2014																														     //
///////////////////////////////////////////////////////////////////////////////
     
    #include "ros/ros.h"
    #include "std_msgs/String.h"
    #include "std_msgs/Empty.h"
     
    #include <sstream>
    #include "std_msgs/Duration.h"
     
    #include <iostream>
    #include <stdio.h>
    #include <string.h>
    #include <fcntl.h>
    #include <errno.h>
    #include <termios.h>
    #include <unistd.h>
     
    int set_interface_attribs (int fd, int speed, int parity);
     
    int main(int argc, char **argv)
    {
      /*Variables*/
      char data[15];
      uint8_t sonar[3];
	  uint8_t loopCounter[10];
      char *dataP = &data[0];
      int port = open("/dev/rfcomm0", O_RDWR | O_NOCTTY | O_NDELAY);
      //int port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
     
            /*Objects*/
      ros::init(argc, argv, "serialListener"); //Create node called talker
      ros::NodeHandle n;//Create nodehandle to alter node parameters
      ros::Duration d = ros::Duration(1,0);//Duration object to pause node
     
      /*Opening port*/
      if(port == -1){
              ROS_INFO("Error opening the port");//Inform user on the terminal
      }
      else{
              ROS_INFO("Serial port is open");//Inform user on the terminal
      }
            set_interface_attribs (port, B115200, 0);
           
            while(ros::ok){
     
                    /*Send request byte and read incoming data*/
                    /*while(read(port,dataP,1) == 1){//Empty buffer
                    }*/
                    tcflush(port, TCIFLUSH);
                    write(port,"s",1);
                    ROS_INFO("Data request sent");//Inform user on the terminal
     
                    //d.sleep(); //Wait for 2 seconds to be published
                    usleep(500000);
     
                    read(port,dataP,15);
     
                    /*Show user the data read*/
                    for(int i=0;i<3;i++){
                            sonar[i] = data[i];
                printf("Sonar %d:",i+1);
                printf(" %u\t",sonar[i]);
            }
            printf("\n\r");  // IR sensor reading converting from char to float
            int j=3;
            for(int i=0;i<6;i++){
                            float d = data[j];
                            d += data[j+1] / 100.00;            
                printf("Ir %u:",i+1);
                printf(" %.2f\t",d);
                j=j+2;
            }
            printf("\n\r");
			ros::spinOnce();
            }
     
            /*Close port and exit*/
            close(port);
     
      return 0;
    }
     
    int set_interface_attribs (int fd, int speed, int parity)
    {
            struct termios tty;
            memset (&tty, 0, sizeof tty);
            if (tcgetattr (fd, &tty) != 0)
            {
                    printf("error from tcgetattr", errno);
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
                    printf ("error from tcsetattr", errno);
                    return -1;
            }
            return 0;
    }


