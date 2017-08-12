#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <sys/time.h>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include "std_msgs/Int16MultiArray.h"
#include "emg_protocol_main.h"

#define READ_BUF_SIZE 33 
#define DATA_SIZE	33

using namespace std;

pthread_t pub_thread, tcp_thread; 
int pub_thread_id, tcp_thread_id;


void error(const char *msg) 
{
    perror(msg);
    exit(1);
}

int main (int argc, char** argv)
{
	int sockfd, newsockfd, portno; //Socket file descriptors and port numbelr
	socklen_t clilen; //object clilen of type socklen_t
	char buffer[READ_BUF_SIZE+1]; //buffer array of size 256
	char portno_c[16];
	
	struct sockaddr_in cli_addr; ///two objects to store client and server address
	struct addrinfo hints, *result;

	// TimeOut 체크를 위한 timeval, fd_set
	struct timeval tv;
	fd_set readfds;
	int sel_status;

	int n;


	// ROS노드 시작
	ros::init(argc, argv, "emg_node");
	ros::NodeHandle nh;
	ros::Publisher server_pub = nh.advertise<std_msgs::Int16MultiArray>("rawEMG", 100);
	std_msgs::Int16MultiArray pub_msg;

	FILE *fp = fopen("emgdata.txt","w");

	// sampling rate 설정
	ros::Rate loop_rate(512); 

	// publishing 되는 데이터의 규격 설정
	pub_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	pub_msg.layout.dim[0].size = CHANNEL_EMG;
	pub_msg.layout.dim[0].stride = 1;
	pub_msg.layout.dim[0].label = "rawEMG";

	//Network init
	{
		// 네트워크 설정 시작
		// 포트번호 받아옴.
		nh.getParam("/emg_node/port_number",portno);
		sprintf(portno_c,"%d",portno);

		// 입력받은 포트로 들어오는 호스트의 조건 설정
		memset(&hints,0 ,sizeof(struct addrinfo));
		hints.ai_family = AF_INET;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_flags = AI_PASSIVE;
		hints.ai_protocol = 0;
		hints.ai_canonname = NULL;
		hints.ai_addr = NULL;
		hints.ai_next = NULL;

		ROS_INFO("PORT : %d (%s)\n",portno,portno_c);

		// 입력 포트의 (클라이언트)주소정보 획득 	
		if( getaddrinfo(NULL,portno_c,&hints,&result) < 0 )
			error("getaddrinfo() failed");

		// 소켓 생성
		sockfd = socket(result->ai_family, result->ai_socktype, result->ai_protocol);

		if (sockfd < 0)
			error("ERROR opening socket");

		// 소켓 옵션 설정( REUSEADDR : 소켓의 연결이 끊어졌다 이어져도, 이전 주소를 계속 사용 )
		int enable = 1;

		if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
			error("setsockopt(SO_REUSEADDR)");
	
		// 소켓에 주소값 연결
		if (bind(sockfd, result->ai_addr, result->ai_addrlen) < 0)
			error("ERROR on binding");

		// 수신대기
		listen(sockfd,5);

		// 클라이언트측에서 connect 시도시, accept. (cli_addr에 클라이언트 정보 반환)
		clilen = sizeof(cli_addr);
		newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);

		if (newsockfd < 0)
			error("ERROR on accept");

		// 모든 연결과정이 마무리 되었으므로 주소정보 free	
		freeaddrinfo(result);

		ROS_INFO("EMG Connection Established!");
	}

	int cntt = 0;
	double max_sec = 0;
	while( ros::ok() )
	{
ros::Time begin = ros::Time::now();
		FD_ZERO(&readfds);
		FD_SET(newsockfd,&readfds);
		tv.tv_sec = 0;
		tv.tv_usec = 0;

		if( select(newsockfd+1,&readfds,NULL,NULL, &tv) > 0 )
		{
			

			bzero(buffer,READ_BUF_SIZE);

			// TCP/IP에서 데이터 읽어옴
			n = read(newsockfd,buffer,READ_BUF_SIZE);

			if (n < 0) 
				error("ERROR reading from socket");

			// 데이터 분류&큐에 입력
			if( EmgDataUnmarshal(buffer,n) )
			{
				pub_msg.data.clear();
				pub_msg.data.assign(emgBuf.data, emgBuf.data + CHANNEL_EMG);

				server_pub.publish( pub_msg );	
			}

		}

		loop_rate.sleep();
ros::Time end = ros::Time::now();
			ros::Duration d = end - begin;
			double secs =  d.toSec();
			max_sec = (max_sec<secs)?secs:max_sec;
			ROS_INFO("delay : %.6lf / buffer : %c",max_sec,buffer[0]);
	}

	close(sockfd);
	close(newsockfd);
	fclose(fp);

	return 0;
}
