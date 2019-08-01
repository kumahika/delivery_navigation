#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

float x_axis = 0.0;
float y_axis = 0.0;
float yaw_ang = 0.0;
int call_flag = 0;
std::string usr1 = "kuma";
std::string usr2 = "eric";

void chatter_Callback(const std_msgs::String& msg)
{
  // ROS_INFO("to connect chat!");
  int result1 = usr1.compare(msg.data.c_str());
  int result2 = usr2.compare(msg.data.c_str());
  printf("result1 = %d, result2 = %d\n", result1, result2);
  if(result1 == 0)
  {
    x_axis = 8.846;
    y_axis = -3.230;
    yaw_ang = 0.0;
    call_flag = 1;
  }
  else if(result2 == 0)
  {
    x_axis = 3.7608;
    y_axis = -8.191;
    yaw_ang = 0.875;
    call_flag = 1;
  }
  else
  {
    x_axis = 0.0;
    y_axis = 0.0;
    yaw_ang = 0.0;
    call_flag = 1;
  }
}

class delivery_from_web_node
{
private:
  ros::Publisher pub_goal;
  // ros::Subscriber sub_usrname;
	tf::TransformListener tfl;

	std::list<geometry_msgs::PoseStamped> goals;
	geometry_msgs::PoseStamped current_goal;

public:
    delivery_from_web_node()
    {
        ros::NodeHandle nh("~");
        pub_goal = nh.advertise<geometry_msgs::PoseStamped>(
                    "/move_base_simple/goal", 5, true);
    }

	void add_goal(const float x, const float y, const float yaw)
	{
		geometry_msgs::PoseStamped goal;
		goal.header.frame_id = "map";
		goal.pose.position.x = x;
		goal.pose.position.y = y;
		goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		goals.push_back(goal);
	}

	bool pop_goal()
	{
		if(goals.size() == 0) return false;

		current_goal = goals.front();
		goals.pop_front();
		ROS_INFO("Applying goal %0.3f %0.3f %0.3f", current_goal.pose.position.x, current_goal.pose.position.y, tf::getYaw(current_goal.pose.orientation));
		pub_goal.publish(current_goal);

		return true;
	}

    void mainloop()
	{
		// if(!pop_goal())
		// {
		// 	ROS_ERROR("No goal specified");
		// 	return;
		// }

		ros::Rate rate(10.0);
		while(ros::ok())
		{
			rate.sleep();
			ros::spinOnce();

			float x, y, yaw;

			try
			{
				tf::StampedTransform trans;
				tfl.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(0.5));
				tfl.lookupTransform("map", "base_link", ros::Time(0), trans);
				x = trans.getOrigin().x();
				y = trans.getOrigin().y();
				yaw = tf::getYaw(trans.getRotation());
			}

			catch(tf::TransformException &e)
			{
				ROS_WARN("%s", e.what());
				continue;
			}

			float yaw_goal = tf::getYaw(current_goal.pose.orientation);
			float yaw_error = yaw - yaw_goal;
			if(yaw > M_PI) yaw -= 2.0 * M_PI;
			else if(yaw < -M_PI) yaw += 2.0 * M_PI;

			if(hypotf(x - current_goal.pose.position.x, y - current_goal.pose.position.y) < 0.4)
			{
				if(!pop_goal())
				{
					ROS_INFO("waypoint reached!");
					return;
				}

				ROS_INFO("Next goal applied");
        //rate.sleep();
			}
		}
	}
};

int main(int argc, char *argv[])
{
  ROS_INFO("Hello ROS!");
   //　ROSを初期化し、ノード名をつける
    ros::init(argc, argv, "delivery_from_web_node");
    ros::NodeHandle n;
    ros::Subscriber sub_usrname = n.subscribe("chatter", 10, chatter_Callback);

   //　作成したdelivery_from_web_nodeクラスを実体化、ここでは、user_addressと名前をつける
    delivery_from_web_node user_address;

	// 目的地点を指定(複数),user_addressのメンバ関数を呼び出す。
  if (call_flag == 1)
  {
    user_address.add_goal(x_axis, y_axis, yaw_ang); // start to goal
    user_address.add_goal(0.0, 0.0, 3.14); // back home
  }

  user_address.mainloop(); // back home
}
