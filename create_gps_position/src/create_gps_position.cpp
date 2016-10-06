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

		struct GpsData{
			geometry_msgs::Point RvizPoint;
			sensor_msgs::NavSatFix GpsPoint;
		};
		std::vector<GpsData> g_waypoints_;
};



void Get_gps_tf::gpsCallback(const sensor_msgs::NavSatFixConstPtr &fix){
	static ros::Time saved_time(0.0);
	if(fix -> position_covariance_type == 1 && (ros::Time::now() - saved_time).toSec() > 5.0){
//		if( (ros::Time::now() - saved_time).toSec() > 5.0){
			GpsData g_data;
			g_data.RvizPoint.x = point.point.x;
			g_data.RvizPoint.y = point.point.y;
			g_data.RvizPoint.z = point.point.z;
			g_data.GpsPoint.latitude = fix -> latitude;
			g_data.GpsPoint.longitude = fix -> longitude;

			g_waypoints_.push_back(g_data);
			saved_time = ros::Time::now();
//		}
	}

	//条件２　fix解が連続して何秒以上
	/*
	int solution;
	if(fix -> position_covariance_type ==1){
		solution++;
		//5秒以上fix解が得られたとき
		if(solution >= 5){
			GpsData g_data;
			g_data.RvizPoint.x = point.point.x;
			g_data.RvizPoint.y = point.point.y;
			g_data.RvizPoint.z = point.point.z;
			g_data.GpsPoint.latitude = fix -> latitude;
			g_data.GpsPoint.longitude = fix -> longitude;

			g_waypoints_.push_back(g_data);
		}
	}else{
	//時間初期化
	solution = 0;
	}
	*/
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
	ofs << "gps_position:" << std::endl;
	for(int i=1; i < g_waypoints_.size(); i++ ){
		ofs << "    " << "- point:" << std::endl;
	  ofs << "         x: " << g_waypoints_[i].RvizPoint.x << std::endl;
		ofs << "         y: " << g_waypoints_[i].RvizPoint.y << std::endl;
		ofs << "         z: " << g_waypoints_[i].RvizPoint.z << std::endl;
		ofs << "         lat: " << std::setprecision(10) << g_waypoints_[i].GpsPoint.latitude << std::endl;
		ofs << "         lon: " << std::setprecision(10) << g_waypoints_[i].GpsPoint.longitude << std::endl;
	}
	ofs.close();
	ROS_INFO("write success");
}

int main(int argc , char *argv[]){
	ros::init(argc, argv, "create_gps_position");
	Get_gps_tf GPS;

	ros::spin();
}
