#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//#include "Twowheels_robot/Wheel_velocity.h"
#include "sensor_msgs/Joy.h"
#include "cmath"

/* change log
*
* 2018/09/02 ver.1.1(水野) duty指令から回転数指令に変更 旋回時のrpmを減らす
* 2018/09/04 ver.1.2(熊本) パブリッシュするトピック名を/cmd_veから/cmd_velに変更
*
*/

#define TURN_DECAY_CONST 0.2 //旋回時の速度の係数

//以下の値を調整することで、ロボットの最大速度を調整することが可能。500：0.1m/s、5000:1m/s
const float Max_rpm = 100.0;
bool Locking_flag = true;
float Pre_rpm1,Pre_rpm2,Outside_vel,Inside_vel,Rotation_vel,joy_line,joy_ang = 0.0;

geometry_msgs::Twist cmd_vel;

//Acceleration controll funtion
float Func_Accele(float Speed,float Pre_rpm)
{
 	if(Speed - Pre_rpm > 0){
		Pre_rpm += Max_rpm*2.0/256;
	}

	else if(Speed - Pre_rpm < 0){
		Pre_rpm -= Max_rpm*2.0/256;
	}
  return Pre_rpm;
}

void joy_callback(const sensor_msgs::Joy& joy_msg){
  // printf("recieve_joymsg");
	joy_line = joy_msg.axes[1];
	joy_ang = joy_msg.axes[2];

	if(joy_msg.buttons[0]!=0){ //disarm mode
  printf("disarm");
	Locking_flag = 1;
	}
	else if(joy_msg.buttons[1]!=0){ //arm mode
  printf("arm");
  Locking_flag = 0;
	}
	Outside_vel = Max_rpm*joy_line;
	Inside_vel = Outside_vel*(1-fabs(joy_ang));
	Rotation_vel = Max_rpm*joy_ang*TURN_DECAY_CONST;

}

int main(int argc, char **argv){
	ros::init(argc, argv, "basic_twist_publisher");
	ros::NodeHandle n;

	//publish
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	//subscriibe
	ros::Subscriber joy_sub   = n.subscribe("joy", 10, joy_callback);

	ros::Rate loop_rate(10);
	while (ros::ok()){

		if (Locking_flag != true){

			//rightside pivot&Speed turn
			if(joy_line != 0 && joy_ang >= 0){
				cmd_vel.linear.x = Func_Accele(Outside_vel,Pre_rpm1);
				cmd_vel.linear.y = Func_Accele(Inside_vel,Pre_rpm2);
			}


			//leftside pivot&Speed turn
			else if(joy_line != 0 && joy_ang <= 0){
				cmd_vel.linear.x = Func_Accele(Inside_vel,Pre_rpm1);
				cmd_vel.linear.y = Func_Accele(Outside_vel,Pre_rpm2);
			}

			//spin turn
			else{
				cmd_vel.linear.x = Func_Accele(Rotation_vel,Pre_rpm1);
				cmd_vel.linear.y = Func_Accele(-Rotation_vel,Pre_rpm2);
			}

		}

		else{
			cmd_vel.linear.x = Func_Accele(0,Pre_rpm1);
			cmd_vel.linear.y = Func_Accele(0,Pre_rpm2);
		}

		Pre_rpm1 = cmd_vel.linear.x;
		Pre_rpm2 = cmd_vel.linear.y;

		cmd_pub.publish(cmd_vel);
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}
