#include "pacman_wander.h"

Behavior_Wander::Behavior_Wander() {
	this->pub_arbiter = this->nh.advertise<package1::behavior>("behavior/wander",1);
}

char Behavior_Wander::getch() {
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if(rv == -1)
		ROS_ERROR("select");
	else if(rv == 0)
		ROS_DEBUG("no_key_pressed");
	else
		read(filedesc, &buff, len );

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}

void Behavior_Wander::process_behavior() {
	char c = getch();
	package1::behavior msg_move;
	msg_move.active = false;
    msg_move.vel_fw = 0;
    msg_move.vel_turn = 0;

	if (c=='w' || c=='W') {
		ROS_DEBUG("Performing Action: MOVE_FORWARD");
		msg_move.active = true;
        msg_move.vel_fw = WANDER_VELOCITY;
	} else if (c=='a' || c=='A') {
		ROS_DEBUG("Performing Action: TURN_LEFT");
		msg_move.active = true;
		msg_move.vel_fw = WANDER_VELOCITY-0.75;
        msg_move.vel_turn = WANDER_VELOCITY;
	} else if(c=='d' || c=='D') {
		ROS_DEBUG("Performing Action: TURN_RIGHT");
		msg_move.active = true;
		msg_move.vel_fw = WANDER_VELOCITY-0.75;
        msg_move.vel_turn = -WANDER_VELOCITY;
	} else if (c=='s' || c=='S') {
		ROS_DEBUG("Performing Action: MOVE_BACKWARD");
		msg_move.active = true;
        msg_move.vel_fw = -WANDER_VELOCITY;
	}
	this->pub_arbiter.publish(msg_move);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "wander");
	ros::NodeHandle nh;
	ROS_INFO("Starting wander node...");
	ros::Rate loop_rate(BH_WANDER_RATE);

	// Create wander
	Behavior_Wander bh_wander;

	while(ros::ok()){
		bh_wander.process_behavior();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}