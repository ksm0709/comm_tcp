#pragma once

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
#include <fcntl.h>
#include <queue>
#include "std_msgs/Int16MultiArray.h"

#define CHANNEL_EMG		16

#define HEADER_SIZE     8
#define TIMESTAMP_SIZE  4
#define EMG_DATA_SIZE	32

#define EMG_DATA_ADDR	HEADER_SIZE + TIMESTAMP_SIZE
#define PACKET_SIZE		HEADER_SIZE + TIMESTAMP_SIZE + EMG_DATA_SIZE
#define READ_BUF_SIZE   4400

#define HEADER		0xC691199927021942

typedef std_msgs::Int16MultiArray DType; 

pthread_t pub_thread, tcp_thread; 
pthread_mutex_t emgQue_mutex;
int pub_thread_id, tcp_thread_id;

typedef struct
{
	uint64_t header;
	uint32_t timestamp;
	uint16_t data[CHANNEL_EMG];
} emgData;

std::queue<emgData> emgQue;

void error(const char *msg) 
{
    perror(msg);
    exit(1);
}

bool EmgDataUnmarshal(const char *data, int size)
{
	emgData *emg_p;
	emgData emgBuf;
	int idx = 0;
	int cnt = 0;

	while( true )
	{
		if( size - idx >= PACKET_SIZE )
			emg_p = (emgData*)(data + idx);
		else
		{
			break;
		}

		if( emg_p->header == HEADER )
		{	
			cnt++;

			ROS_INFO("cnt : %d / time : %d / quesize : %d", cnt, emg_p->timestamp, emgQue.size());

			emgBuf.timestamp = emg_p->timestamp;

			for(int i = 0; i < CHANNEL_EMG ; i++ )
			{
				emgBuf.data[i] = emg_p->data[i];
			}

			pthread_mutex_lock(&emgQue_mutex); 
				if (emgQue.size() > 100)
					emgQue.pop();
				emgQue.push(emgBuf);
			pthread_mutex_unlock(&emgQue_mutex);

			idx += PACKET_SIZE;
		}
		else
			idx++;
	}

	return true;
}

void* pub_thread_func(void* par)
{
	ros::NodeHandle* nh= (ros::NodeHandle*)par;

	ros::Publisher server_pub = nh->advertise<DType>("rawEMG", 10);
	DType pub_msg;

	FILE *fp = fopen("emgdata.txt","w");

	// sampling rate 설정
	ros::Rate loop_rate(1024); 

	// publishing 되는 데이터의 규격 설정
	pub_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	pub_msg.layout.dim[0].size = CHANNEL_EMG;
	pub_msg.layout.dim[0].stride = 1;
	pub_msg.layout.dim[0].label = "rawEMG";

	// emgQue는 송,수신부에서 모두 사용하므로 mutex로 한쪽에서 작업할 때 다른쪽에서 건드릴 수 없게 
	pthread_mutex_init(&emgQue_mutex, NULL);

	while(ros::ok())
	{
		if( emgQue.size() > 0 )
		{
			pub_msg.data.clear();

			pthread_mutex_lock(&emgQue_mutex);
				//pub_msg.data.assign(emgQue.front().data, emgQue.front().data + CHANNEL_EMG);
				emgData front = emgQue.front();
				for(int i = 0; i < CHANNEL_EMG ; i++ )
				{
					pub_msg.data.push_back((int16_t)(front.data[i] - 32727));
				}
				emgQue.pop();
			pthread_mutex_unlock(&emgQue_mutex);
			
			fprintf(fp,"%d\n", pub_msg.data[0]);
			server_pub.publish( pub_msg );	

//			ROS_INFO( "que size : %d",emgQue.size());
		}

		loop_rate.sleep();
	}
	fclose(fp);
}

void* tcp_thread_func(void* par)
{
	ros::NodeHandle *nh = (ros::NodeHandle*)par;
	ros::Rate loop_rate(20); 

	int sockfd, newsockfd, portno; //Socket file descriptors and port numbelr
	socklen_t clilen; //object clilen of type socklen_t
	char buffer[READ_BUF_SIZE+1]; //buffer array of size 256
	char portno_c[16];
	
	struct sockaddr_in cli_addr; ///two objects to store client and server address
	struct addrinfo hints, *result;

	// TimeOut 체크를 위한 timeval, fd_set
	struct timeval tv;
	fd_set readfds,tempfds;
	int sel_status;	
	int flag;
	int n;

	//Network init
	{
		// 네트워크 설정 시작
		// 포트번호 받아옴.
		nh->getParam("/emg_node/port_number",portno);
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

		ROS_INFO("EMG Board Connection Established!");
	}
	ROS_INFO("socket fd : %d",newsockfd);
	// Nonblocking socket으로 설정
//	flag = fcntl( newsockfd, F_GETFL, 0 );
//	fcntl( newsockfd, F_SETFL, flag | O_NONBLOCK );
	
	FD_ZERO(&readfds);
	FD_SET(newsockfd,&readfds);

	while(ros::ok()) 
	{
		tempfds = readfds;

		tv.tv_sec = 0;
		tv.tv_usec = 10;

		sel_status = select(newsockfd+1,&tempfds,NULL,NULL, &tv);

		if( sel_status > 0 && FD_ISSET(newsockfd,&tempfds) )
		{
			bzero(buffer,READ_BUF_SIZE);

			// TCP/IP에서 데이터 읽어옴
			n = read(newsockfd,buffer,READ_BUF_SIZE);

			if (n < 0) 
			{
				ROS_INFO("EMG Board Disconnected!");	
				newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
				
				FD_ZERO(&readfds);
				FD_SET(newsockfd,&readfds);
		
				continue;			
			}

			// 데이터 분류&큐에 입력
			EmgDataUnmarshal(buffer,n);
		}
		loop_rate.sleep();
		
	}

	close(sockfd);
	close(newsockfd);
}