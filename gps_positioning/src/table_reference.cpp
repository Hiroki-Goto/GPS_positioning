#include<ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <fstream>
#include <math.h>

#ifdef NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i){
    i = node.as<T>();
}
#endif

using namespace visualization_msgs;

class reference{
public:
    //ファイル読み込み
    reference():filename_(""), fp_flag_(false), rate_(10){
        ros::NodeHandle nh;
        GPS_Sub = nh.subscribe("/gps/fix",10,&reference::GPSCallback,this);         //飛んでくるトピックに修正を行う
        marker_description_pub_ = nh.advertise<visualization_msgs::MarkerArray>("GPS_waypoint",1);

        ros::NodeHandle private_nh("~");
        private_nh.param("world_frame",world_frame_,std::string("map"));
        private_nh.param("filename",filename_,filename_);
        if(filename_ != ""){
            ROS_INFO_STREAM("Read GPS waypoints data from" << filename_);
            if(readFile(filename_)){
                fp_flag_ = true;
                //makeMarker();
            }else{
                ROS_ERROR("Failed loading GPS waypoint data file");
            }
        }else{
            ROS_ERROR("GPS data file doesn't have name");
        }
    }
    //GPSデータのサブスクライバー

    void GPSCallback(const sensor_msgs::NavSatFixConstPtr &fix);
    void run(){
        while(ros::ok()){
            rate_.sleep();
            ros::spinOnce();
            publishMarkerDescription();
        }
    }

private:
    void gauss();                       //ガウス分布する関数
    std::string filename_;
    std::string world_frame_;
    void publishMarkerDescription();    //GPSwaypointの表示用
    bool fp_flag_;
    //ふレーム関係
    bool readFile(const std::string &filename);
    struct GPS_Data{
        geometry_msgs::Point RvizPoint;
        sensor_msgs::NavSatFix GpsPoint;
    };
    std::vector<GPS_Data> g_waypoints_;

    ros::Rate rate_;
    visualization_msgs::MarkerArray marker_description_;

    //pub,sub関係
    ros::Publisher marker_description_pub_;
    ros::Subscriber GPS_Sub;
};

bool reference::readFile(const std::string &filename){
	g_waypoints_.clear();
	try{
		std::ifstream ifs(filename.c_str(), std::ifstream::in);
		if(ifs.good() == false){
			return false;
		}
		YAML::Node node;
		#ifdef NEW_YAMLCPP
			node = YAML::Load(ifs);
		#else
			YAML::Parser parser(ifs);
			parser.GetNextDocument(node);
		#endif

		#ifdef NEW_YAMLCPP
			const YAML::Node &gp_node_tmp = node["gps_position"];
			const YAML::Node *gp_node = gp_node_tmp ? &gp_node_tmp : NULL;
		#else
			const YAML::Node *gp_node = node.FindValue("gps_position");
		#endif

		if(gp_node != NULL){
			GPS_Data g_data;
			for(int i=0; i < gp_node->size(); i++){
				(*gp_node)[i]["point"]["x"] >> g_data.RvizPoint.x;
				(*gp_node)[i]["point"]["y"] >> g_data.RvizPoint.y;
				(*gp_node)[i]["point"]["z"] >> g_data.RvizPoint.z;
				(*gp_node)[i]["point"]["lat"] >> g_data.GpsPoint.latitude;
				(*gp_node)[i]["point"]["lon"] >> g_data.GpsPoint.longitude;
			   g_waypoints_.push_back(g_data);
			}
		}else{
			return false;
		}
	}catch(YAML::ParserException &e){
		ROS_WARN_STREAM( e.what() );
		return false;
	}catch(YAML::RepresentationException &e){
		ROS_WARN_STREAM( e.what() );
		return false;
	}
    return true;
}

void reference::GPSCallback(const sensor_msgs::NavSatFixConstPtr &fix){
    if(fix->position_covariance_type == 1){
        //fix解での処理
        //最近の２点間を調べる
        //そこから２点間からの距離を出す
        //トピックを配信する
    }else{
        //それ以外の処理
    }
}

void reference::publishMarkerDescription(){
	for (int i=0; i < g_waypoints_.size(); i++){
		 Marker marker;
		 marker.type = Marker::SPHERE;
		 marker.text = std::to_string(i);
		 marker.header.frame_id = world_frame_;
		 marker.header.stamp = ros::Time::now();
		 std::stringstream name;
		 name << "GPS_waypoint";
		 marker.ns = name.str();
		 marker.id = i;
		 marker.pose.position.x = g_waypoints_[i].RvizPoint.x;
		 marker.pose.position.y = g_waypoints_[i].RvizPoint.y;
		 marker.pose.position.z = 0.0;
		 marker.scale.x = 0.5f;
		 marker.scale.y = 0.5f;
		 marker.scale.z = 0.5f;

		 marker.color.r = 1.0f;
		 marker.color.g = 0.0f;
		 marker.color.b = 0.0f;
		 marker.color.a = 1.0;
		 marker.action = visualization_msgs::Marker::ADD;
		 marker_description_.markers.push_back(marker);
	}
	marker_description_pub_.publish(marker_description_);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"table reference");
    ros::Time::init();
    reference reference;
    reference.run();

    return 0;
}
