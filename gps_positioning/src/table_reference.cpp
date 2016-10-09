#include<ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
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
        //ros::NodeHandle nh;
        GPS_Sub = nh.subscribe("/gps/fix",10,&reference::GPSCallback,this);         //飛んでくるトピックに修正を行う
        marker_description_pub_ = nh.advertise<visualization_msgs::MarkerArray>("GPS_waypoint",1);

    //    GPS_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("solution_data",1);
        position_GPS_pub_ = nh.advertise<geometry_msgs::PointStamped>("position_GPS",1);

        ros::NodeHandle private_nh("~");
        private_nh.param("world_frame",world_frame_,std::string("map"));
        private_nh.param("filename",filename_,filename_);
        if(filename_ != ""){
            ROS_INFO_STREAM("Read GPS waypoints data from" << filename_);
            if(readFile(filename_)){
                fp_flag_ = true;
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
    ros::NodeHandle nh;
    void gps_position(double latitude, double longitude);                       //選ばれた3点から推定座標を求める
    std::string filename_;
    std::string world_frame_;
    void publishMarkerDescription();    //GPSwaypointの表示用
    void publishGPSMarker(double solution_x,double solution_y);   //計測点の表示
    bool fp_flag_;

    bool readFile(const std::string &filename);
    struct GPS_Data{
        geometry_msgs::Point RvizPoint;
        sensor_msgs::NavSatFix GpsPoint;
    };
    std::vector<GPS_Data> g_waypoints_;
    std::vector<geometry_msgs::PointStamped> select_points_;

    ros::Rate rate_;
    visualization_msgs::MarkerArray marker_description_;
    visualization_msgs::MarkerArray marker_GPS_;
    //pub,sub関係
    ros::Publisher marker_description_pub_;
    ros::Subscriber GPS_Sub;
    //ros::Publisher GPS_marker_pub_;
    ros::Publisher position_GPS_pub_;
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
    double solution_x,solution_y;
    if(fix->position_covariance_type == 1){
        //fix解での処理
        ROS_INFO("latitude:%lf  longitude:%lf",fix->latitude,fix->longitude);
        //最近の3点間を調べる
        //そこから3点間からの距離を出す
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

//経度から距離への変換式の追加を行う
void reference::gps_position(double latitude, double longitude){
    double lat[select_points_.size()],lon[select_points_.size()];
    geometry_msgs::PointStamped result;
    //double result_x,result_y;
    for (int i=0; i < select_points_.size(); i++){
        //latitude->y
        result.point.y += (lat[i] - select_points_[i].point.x);
        //longitude->x
        result.point.x += (lat[i] - select_points_[i].point.y);
    }
    result.point.x /= select_points_.size();
    result.point.y /= select_points_.size();
    result.point.z = 0;

    position_GPS_pub_.publish(result);

}

/*
void reference::publishGPSMarker(double solution_x, double solution_y){
    Marker m_GPS;
    m_GPS.type = Marker::SPHERE;
    m_GPS.text = std::string("GPS_solution");
    m_GPS.header.frame_id = world_frame_;
    m_GPS.header.stamp = ros::Time::now();
    std::stringstream name;
    name << "GPS solution";
    m_GPS.ns = name.str();
    m_GPS.id = 1;
    m_GPS.pose.position.x = solution_x;
    m_GPS.pose.position.y = solution_y;
    m_GPS.pose.position.z = 0.0;
    m_GPS.scale.x = 0.5f;
    m_GPS.scale.y = 0.5f;
    m_GPS.scale.z = 0.5f;

    m_GPS.color.r = 0.0f;
    m_GPS.color.g = 0.0f;
    m_GPS.color.b = 1.0f;
    m_GPS.color.a = 1.0;
    m_GPS.action = visualization_msgs::Marker::ADD;
    marker_GPS_.markers.push_back(m_GPS);

    GPS_marker_pub_.publish(marker_GPS_);
}
*/
int main(int argc, char **argv){
    ros::init(argc,argv,"table reference");
    ros::Time::init();
    reference reference;
    reference.run();

    return 0;
}
