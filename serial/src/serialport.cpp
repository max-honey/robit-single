#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/serial.h>
#include "ros/ros.h"
#include <serial/speedcmd.h>
#include <serial/shootcmd.h>
#include <serial/feedback.h>
//有待补充名字 serial::serialport
void splitint(int a,unsigned char *c)
{
	int tmp = a;
  c[1] = (char)(tmp & 0x00ff);
  tmp >>= 8;
  c[0] = (char)(tmp & 0x00ff);
  return;
}

void initSerial(int fd)
{
  //设置阻塞等文件标志
  fcntl(fd, F_SETFL, O_NONBLOCK);  

  termios options;
  tcgetattr(fd, &options);
  //波特率（并没有使用这个）
  cfsetispeed(&options, B38400);
  cfsetospeed(&options, B38400);
  //本地模式，可读
  options.c_cflag |= (CLOCAL | CREAD);
  //设置数据位
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~INPCK;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  //原始数据输出
  options.c_oflag &= ~OPOST;
  //输入控制
  options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  //原始输入模式
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_lflag &= ~(ECHONL | IEXTEN);
  //设置超时
  options.c_cc[VTIME] = 1;
  options.c_cc[VMIN] = 0;
  //立刻刷新控制
  tcflush(fd,TCIFLUSH);
  if(tcsetattr(fd,TCSANOW,&options) != 0)
  {
    printf("setattr error\n");
    throw 0;
  }

  //特殊波特率设置
  serial_struct ss;
  int baud = 2000000;
  if(ioctl(fd, TIOCGSERIAL, &ss) < 0)
  {
    printf("ioctl error\n");
    throw 0;
  }
  ss.flags = ASYNC_SPD_CUST;
  ss.custom_divisor = ss.baud_base / baud;

  if(ioctl(fd, TIOCSSERIAL, &ss) < 0)
  {
    printf("ioctl S error\n");
    throw 0;
  }
}

int sendSerial(int fd, unsigned char* buf, unsigned long datalen)
{
  int len;
  len = write(fd, (char*)buf, datalen);
  if(len != datalen)
  {
    printf("datalen=%lu len=%d\n", datalen, len);
  }
  tcflush(fd,TCOFLUSH); //必须是NONBLOCK，而且必须是立刻flush，否则输出缓冲区会被占满，直到强制结束进程才能完成发送
  return len;
}
void generateSpeedCmd(unsigned char *buf,const serial::speedcmd cmd)
{
  buf[0] = 0x55;
  buf[1] = 0xaa;
  buf[2] = 0x38;
  buf[3] = 0x0a;
  buf[4] = 0x08;
  buf[5] = 0x70;
  int i = 0, sum = 0;
  for (i=0;i<5;i++)
  {
	  splitint(cmd.speed[i], buf + 6 + 2 * i);
  }
  for (i = 0; i <= 15; i++)
	  sum += buf[i];
  buf[16] = sum % 0x100;
  return;
}
void generateShootCmd(unsigned char *buf, int power)
{
	buf[0] = 0x55;
	buf[1] = 0xaa;
	buf[2] = 0x38;
	buf[3] = 0x02;
	buf[4] = 0x09;
	buf[5] = 0x70;
	splitint(power, buf + 6);
	int i = 0, sum = 0;
	for (i = 0; i < 8; i++)
		sum += buf[i];
	buf[8] = sum % 0x100;
	return;
}

void genCmdBuf(unsigned char* buf)
{
  buf[0] = 0x55;
  buf[1] = 0xaa;
  buf[2] = 0x38;
  buf[3] = 0x0a;
  buf[4] = 0x08;
  buf[5] = 0x70;

  buf[6] = buf[8] = buf[10] = buf[12] = buf[14] = 0x02;
  buf[7] = buf[9] = buf[11] = buf[13] = buf[15] = 0x11;

  int i = 0;
  unsigned char sum = 0;
  for(; i <= 15; i ++)
  {
    sum += buf[i];
  }
  buf[16] = sum;
}


void readSerial(int fd, unsigned char* buf, int* datalen)
{
  int pos, len, ret = 1;
  pos = 0;
  len = 0;
  while(ret > 0)
  {
    ret = read(fd, buf + pos, 1);
    len += ret;
    pos += ret;
  }
  *datalen = len;
}

void showRead(unsigned char* buf, int datalen)
{
  int i;
  if(datalen > 0)
  {
    printf("len:%2d:", datalen);
    for(i = 0; i < datalen; i ++)
    {
      printf(" 0x%02x", buf[i]);
    }
    printf("\n");
  }
}
/*考虑在发送方加入生成命令的函数，直接发送加工好的命令
其中速度命令用generateSpeedCmd生成*/
/*void chatterCallBack(const std_msgs::String::ConstPtr& msg)//考虑用一个中间节点生成命令
{
	
}*/
void decodeFeedBack(unsigned char *buf,serial::feedback* f)
{
	int pos[3] = { 0 };
	int current[3] = { 0 };
	int i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 4; j++)
		{
			pos[i] <<= 8;
			pos[i] += buf[7 + 6 * i + j];//7-10 13-16 19-22
		}
		for (j = 0; j < 2; j++)
		{
			current[i] <<= 8;
			current[i] += buf[11 + j + 6 * i];//11 12 17 18 23 24
		}
	}
	for (i = 0; i < 3; i++)
	{
		f->pos.push_back(pos[i]);
		f->current.push_back(current[i]);
	}
	return;
}
class SubscribeAndPublish
{
public:
		int fd;
	SubscribeAndPublish()
	{
		fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
		if (fd == -1)
		{
			printf("Failed to open serial.");
			
		}
		initSerial(fd);
		//发布反馈回来的消息，似乎要加几个
		pub_ = n_.advertise<serial::feedback>("SerialPort", 1000);

		//订阅两种消息
		sub_ = n_.subscribe("speedcontroller", 1, &SubscribeAndPublish::callback, this);
		subb_ = n_.subscribe("shootcontroller", 1, &SubscribeAndPublish::callback2, this);
	}

	void callback(const serial::speedcmd input)
	{
		//ROS_INFO("Received: [%s]", msg->data.c_str());
		//处理消息，主体放进来
		unsigned char buf[255];
		int siz, len = 17;
		generateSpeedCmd(buf, input);
		//genCmdBuf(buf);
		sendSerial(fd, buf, len);
		//readSerial(fd, buf, &siz);
		//showRead(buf, siz);
		//usleep(20);
		//serial::feedback output;
		//.... do something with the input and generate the output...
		//decodeFeedBack(buf, &output);
		//pub_.publish(output);
	}
	void callback2(const serial::shootcmd input)
	{
		unsigned char buf[255];
		int siz, len;
		generateShootCmd(buf, input.power);
		//genCmdBuf(buf);
		sendSerial(fd, buf, len);
		//readSerial(fd, buf, &siz);
		//showRead(buf, siz);
		//usleep(20);
		//.... do something with the input and generate the output...
		//serial::feedback output;
		//decodeFeedBack(buf, &output);
		//pub_.publish(output);
	}
private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
	ros::Subscriber subb_;

};//End of class SubscribeAndPublish
int main(int argc,char **argv)
{
	
  ros::init(argc,argv,"SerialPort");
  SubscribeAndPublish sp;
  /*从controller传入的数据格式：
  1.五个电机的速度/射门力度
  */
  ros::spin();
  return 0;
}
