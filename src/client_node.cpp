 /*************************************************************************
 * Author: Abhinav Jain
 * Contact: abhinavjain241@gmail.com, abhinav.jain@heig-vd.ch
 * Date: 28/06/2016
 *
 * This file contains source code to the client node of the ROS package
 * comm_tcp developed at LaRA (Laboratory of Robotics and Automation)
 * as part of my project during an internship from May 2016 - July 2016.
 *
 * (C) All rights reserved. LaRA, HEIG-VD, 2016 (http://lara.populus.ch/)
 ***************************************************************************/
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include "std_msgs/String.h"
#include "std_msgs/UInt16MultiArray.h"

#define MESSAGE_FREQ 1

void error(const char *msg) {
    perror(msg);
    exit(0);
}

class Listener {
private:
    char topic_message[256];
public:
    void callback(const std_msgs::String::ConstPtr& msg);
    char* getMessageValue();
};

void Listener::callback(const std_msgs::String::ConstPtr& msg) {
    memset(topic_message, 0, 256);
    strcpy(topic_message, msg->data.c_str());
    ROS_INFO("I heard:[%s]", msg->data.c_str());
}

char* Listener::getMessageValue() {
    return topic_message;
}

int main(int argc, char *argv[]) {

	int sockfd, portno, n, choice = 1;
    	struct sockaddr_in serv_addr;
    	struct hostent *server;
    	char buffer[256];
    	bool echoMode = false;
	std::string server_add;

	ros::init(argc, argv, "client_node");
	ros::NodeHandle nh;
	Listener listener;

	ROS_INFO("CLIENT_NODE Started!\n");
	
    ros::Subscriber client_sub = nh.subscribe("/client_messages", 1, &Listener::callback, &listener);
	ros::Publisher client_pub = nh.advertise<std_msgs::UInt16MultiArray>("/rawEMG",10);	
    ros::Rate loop_rate(MESSAGE_FREQ); // set the rate as defined in the macro MESSAGE_FREQ

	nh.getParam("port_number", portno);
	
	ROS_INFO("PORT_NUMBER : %d\n",portno);

	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	if (sockfd < 0) error("ERROR opening socket");	
	
	nh.getParam("server_address", server_add);
	server = gethostbyname(server_add.c_str());
	
	ROS_INFO("SERVER_ADD : %s\n",server_add.c_str());
	ROS_INFO("AF_INET : %d\nSOCK_STREAM : %d\n",AF_INET,SOCK_STREAM);

	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;

	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(portno);
    	
	if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) error("ERROR connecting");

	while(ros::ok()) 
	{
/*		// Write 
        	bzero(buffer,256);

            	strcpy(buffer, listener.getMessageValue());

		n = write(sockfd,buffer,strlen(buffer));
	    
	    	if (n < 0) 
	         	error("ERROR writing to socket");
*/
		// Read
		bzero(buffer, 256);
		    
		n = read(sockfd,buffer,255);
		   
		if (n < 0)
			error("ERROR reading reply");
		ROS_INFO("%s\n", buffer);

            	loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
