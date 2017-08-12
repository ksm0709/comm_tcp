 /*************************************************************************
 * Author: Abhinav Jain
 * Contact: abhinavjain241@gmail.com, abhinav.jain@heig-vd.ch
 * Date: 28/06/2016
 *
 * This file contains source code to the server node of the ROS package
 * comm_tcp developed at LaRA (Laboratory of Robotics and Automation)
 * as part of my project during an internship from May 2016 - July 2016.
 *
 * (C) All rights reserved. LaRA, HEIG-VD, 2016 (http://lara.populus.ch/)
 ***************************************************************************/
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#include "std_msgs/Int8MultiArray.h"
#include "glove_protocol.h"

#define READ_BUF_SIZE 	GLOVE_DATA_SIZE+1

using namespace std;

void error(const char *msg) 
{
    perror(msg);
    exit(1);
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "glove_node");
	ros::NodeHandle nh;
	
	ros::Publisher server_pub = nh.advertise<std_msgs::Int8MultiArray>("rawGLOVE", 10);
	std_msgs::Int8MultiArray pub_msg;

	//Testing package is working fine
	int sockfd, newsockfd, portno; //Socket file descriptors and port numbelr
	socklen_t clilen; //object clilen of type socklen_t
	char buffer[READ_BUF_SIZE]; //buffer array of size 256
	char portno_c[16];
	
	struct sockaddr_in cli_addr; ///two objects to store client and server address
	struct addrinfo hints, *result;

	int n,i;

	GLOVE_DATA gloveData;
	GLOVE_DATA* pgloveData;

	// publishing 되는 데이터의 규격 설정
	pub_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	pub_msg.layout.dim[0].size = GLOVE_DATA_SIZE;
	pub_msg.layout.dim[0].stride = 1;
	pub_msg.layout.dim[0].label = "rawGLOVE";

	// sampling rate 설정
	ros::Rate loop_rate(200); // 100Hz

	// 네트워크 설정 시작
	// 포트번호 받아옴.
	nh.getParam("/glove_node/port_number",portno);
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
	else
		ROS_INFO("Glove Connection Established!");

	// 모든 연결과정이 마무리 되었으므로 주소정보 free	
	freeaddrinfo(result);
    
	while(ros::ok()) 
	{

		bzero(buffer,READ_BUF_SIZE);

		// TCP/IP에서 데이터 읽어옴
		n = read(newsockfd,buffer,READ_BUF_SIZE);

		if (n < 0) 
		{
			ROS_INFO("Glove Disconnected!");
			newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	
		}
		
		// 전달받은 데이터 가공
		if((pgloveData = GloveDataUnmarshal(buffer,n)) != NULL)
		{
			pub_msg.data.clear();

			for(i=0;i<5;i++)
				pub_msg.data.push_back((int8_t)pgloveData->s.finger[i].pip);	

			server_pub.publish( pub_msg );	
		}

		loop_rate.sleep();
	}
	close(sockfd);
	close(newsockfd);

	return 0;
}
