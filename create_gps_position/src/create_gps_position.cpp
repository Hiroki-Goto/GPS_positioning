#include<ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/NavSatFix.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>
#include <fstream>
#include <string>


class Get_gps_tf {
	public:
		Get_gps_tf() : filename("gps_data.yaml"){
			gps_sub = nh.subscribe("/gps/fix",10,&Get_gps_tf::gpsCallback,this);
			tf_sub = nh.subscribe("/tf",1,&Get_gps_tf::tfCallback,this);
			finish_pose_sub = nh.subscribe("finish_pose",1, &Get_gps_tf::finishPoseCallback,this);
	
			ros::NodeHandle private_nh("~");	
			private_nh.param("filename", filename, filename);
			private_nh.param("robot_frame", robot_frame, std::string("/base_link"));
			private_nh.param("world_frame", world_frame, std::string("/map"));
			gps_data_num = 0;
		}
		
		void gpsCallback(const sensor_msgs::NavSatFixConstPtr &fix);
		void tfCallback(const tf2_msgs::TFMessage &tf);
		void finishPoseCallback(const geometry_msgs::PoseStamped &msg);
		void save();

	private:
		ros::Subscriber gps_sub;
		ros::Subscriber tf_sub;

		ros::Subscriber re_point_sub;
		ros::Subscriber finish_pose_sub;

		tf::TransformListener tf_listener;

		std::string filename;
		std::string world_frame;
		std::string robot_frame;
		geometry_msgs::PointStamped point;
		geometry_msgs::PoseStamped finish_pose;
		ros::NodeHandle nh;
		int gps_data_num;
	
	struct Data{
		double x;
		double y;
		double z;
		double lat;
		double lon;
	}Data[10000];
};


void Get_gps_tf::gpsCallback(const sensor_msgs::NavSatFixConstPtr &fix){
	static ros::Time saved_time(0.0);
	if( (ros::Time::now() - saved_time).toSec() > 5.0){
			Data[gps_data_num].x = point.point.x;
			Data[gps_data_num].y = point.point.y;
			Data[gps_data_num].z = point.point.z;
			Data[gps_data_num].lat = fix->latitude;
			Data[gps_data_num].lon = fix->longitude;
			saved_time = ros::Time::now();
			gps_data_num++;
	}
}

void Get_gps_tf::tfCallback(const tf2_msgs::TFMessage &tf){
		tf::StampedTransform robot_gl;
		try{
		//target-source-time-tranceformの引数で座標変換を行う
		tf_listener.lookupTransform(world_frame, robot_frame, ros::Time(0.0), robot_gl);
		point.point.x = robot_gl.getOrigin().x();
		point.point.y = robot_gl.getOrigin().y();
		point.point.z = robot_gl.getOrigin().z();
		}catch(tf::TransformException &e){
			ROS_WARN_STREAM( e.what());
		}
}


void Get_gps_tf::finishPoseCallback(const geometry_msgs::PoseStamped &msg){
	finish_pose = msg;
	save();
}

void Get_gps_tf::save(){
	std::ofstream ofs(filename.c_str(), std::ios::out);
	ofs << "rviz_point_and_gps_point:" << std::endl;
	for(int i=1; i<gps_data_num; i++ ){
		ofs << "    " << "- point:" << std::endl;
	   	ofs << "         x : " << Data[i].x << std::endl;
		ofs << "         y : " << Data[i].y << std::endl;
		ofs << "         z : " << Data[i].z << std::endl;
		ofs << "		lat: " << std::setprecision(10) << Data[i].lat << std::endl;
		ofs << "		lon: " << std::setprecision(10) << Data[i].lon << std::endl;
		ofs << std::endl;
	}

	ofs << "finish_pose:"           << std::endl;
	ofs << "    header:"            << std::endl;
	ofs << "        seq: "          << finish_pose.header.seq << std::endl;
	ofs << "        stamp: "        << finish_pose.header.stamp << std::endl;
	ofs << "        frame_id: "     << finish_pose.header.frame_id << std::endl;
	ofs << "    pose:"              << std::endl;
	ofs << "        position:"      << std::endl;
	ofs << "            x: "        << finish_pose.pose.position.x << std::endl;
	ofs << "            y: "        << finish_pose.pose.position.y << std::endl;
	ofs << "            z: "        << finish_pose.pose.position.z << std::endl;
	ofs << "        orientation:"   << std::endl;
	ofs << "            x: "        << finish_pose.pose.orientation.x << std::endl;
	ofs << "            y: "        << finish_pose.pose.orientation.y << std::endl;
	ofs << "            z: "        << finish_pose.pose.orientation.z << std::endl;
	ofs << "            w: "        << finish_pose.pose.orientation.w << std::endl;

	ofs.close();

	ROS_INFO("write success");
}

int main(int argc , char *argv[]){
	ros::init(argc, argv, "create_gps_position");
	Get_gps_tf GPS;
	
	ros::spin();
}

