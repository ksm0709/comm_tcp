#include <ros/ros.h>

#include "emg_protocol.h"

using namespace std;

int main (int argc, char** argv)
{
	pthread_attr_t thread_attrs;
	struct sched_param param;

	// ROS노드 시작
	ros::init(argc, argv, "emg_node");
	ros::NodeHandle nh;

	// 스레드 우선순위 및 기타 설정
	pthread_attr_init(&thread_attrs);
	pthread_attr_setdetachstate(&thread_attrs, PTHREAD_CREATE_DETACHED);

	// publisher 스레드 생성
	pub_thread_id = pthread_create(&pub_thread,&thread_attrs,pub_thread_func,(void*)&nh);
	
	// tcp 스레드 생성
	tcp_thread_id = pthread_create(&tcp_thread,&thread_attrs,tcp_thread_func,(void*)&nh);

	ros::spin();

	pthread_attr_destroy(&thread_attrs);
	return 0;
}
