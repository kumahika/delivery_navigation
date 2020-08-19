#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "cmath"


#define TURN_DECAY_CONST 0.5 //旋回時の速度の係数

float gain = 0.5;
bool Locking_flag = true;
float x_vel,w_vel,joy_line,joy_ang = 0.0;

geometry_msgs::Twist cmd_vel;

void joy_callback(const sensor_msgs::Joy& joy_msg)
{
  // printf("recieve_joymsg");
	joy_line = joy_msg.axes[1];
	joy_ang = joy_msg.axes[2];

	if(joy_msg.buttons[0]!=0) //disarm mode
  {
	   Locking_flag = 1;
	}
	else if(joy_msg.buttons[3]!=0)//armmode
  {
     Locking_flag = 0;
	}

  else if(joy_msg.buttons[10]!=0)//gain
  {
		if (gain>0){gain -= 0.05;}
		else {gain = 0;}

	}
  else if(joy_msg.buttons[11]!=0)//gain
  {
     gain += 0.05;
  }
	else if(joy_msg.buttons[12]!=0)//Stop
	{
		 joy_line = 0;
		 joy_ang = 0;
	}
	ROS_INFO("Speed(Max) %0.3f [m/s]", gain);
	x_vel = gain*joy_line;
	w_vel = gain*joy_ang*TURN_DECAY_CONST;

}

int main(int argc, char **argv){
	ros::init(argc, argv, "basic_twist_publisher");
	ros::NodeHandle n;

	//publish
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	//subscriibe
	ros::Subscriber joy_sub   = n.subscribe("joy", 10, joy_callback);

	ros::Rate loop_rate(100);
	while (ros::ok())
  {
		if (Locking_flag != true)
    {
      cmd_vel.linear.x = x_vel;
      cmd_vel.angular.z = w_vel;
    }

		cmd_pub.publish(cmd_vel);
		ros::spinOnce();
		loop_rate.sleep();

	}
  return 0;
}
