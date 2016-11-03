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
    reference():filename_(""), fp_flag_(false), rate_(1){
        GPS_Sub = nh.subscribe("/gps_solution",1,&reference::GPSCallback,this);
        marker_description_pub_ = nh.advertise<visualization_msgs::MarkerArray>("GPS_waypoint",1);

        GPS_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("measurement_point",1);
        position_GPS_pub_ = nh.advertise<sensor_msgs::NavSatFix>("position_GPS",1);

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
    void find_gps_position();
    std::string filename_;
    std::string world_frame_;
    void publishMarkerDescription();    //GPSwaypointの表示用
    void publishGPSMarker(double solution_x,double solution_y,double solution_z);   //計測点の表示
    bool fp_flag_;
    double saved_gps_data[2];
    bool readFile(const std::string &filename);
    struct GPS_Data{
        geometry_msgs::Point RvizPoint;
        sensor_msgs::NavSatFix GpsPoint;
    };
    std::vector<GPS_Data> g_waypoints_;
    struct select_GPS_Data{
        geometry_msgs::Point RvizPoint;
        double sl_lat;
        double sl_lon;
    };
    struct select_Rviz_Data{
        double sl_x;
        double sl_y;
    };
    std::vector<select_GPS_Data> select_gps_points_;
    std::vector<select_Rviz_Data> select_rviz_points_;

    ros::Rate rate_;
    visualization_msgs::MarkerArray marker_description_;
    visualization_msgs::MarkerArray marker_GPS_;
    ros::Publisher marker_description_pub_;
    ros::Subscriber GPS_Sub;
    ros::Publisher GPS_marker_pub_;
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

    double cor_lat = 110946.163901;      //緯度1度あたりの距離[m]
    double cor_lon = 89955.271505;      //経度1度あたりの距離[m]
    double re_lat = fix -> latitude;
    double re_lon = fix -> longitude;
    double S=0;
    double x_g=0;
    double y_g=0;

    //fix解or　float解の識別
    if(fix->position_covariance_type == 3
        || fix->position_covariance_type == 2){

        //急に飛んできたデータの削除を行う
    	double dis_lat =  (re_lat - saved_gps_data[0]) * cor_lat;
    	double dis_lon =  (re_lon - saved_gps_data[1]) * cor_lon;
        if(fabs(dis_lat) > 0.65 || fabs(dis_lon) > 0.65){
            ROS_ERROR("nya?");
        }else{
            geometry_msgs::PointStamped result_point_;
            select_GPS_Data s_g_data;
            select_Rviz_Data s_r_data;
            sensor_msgs::NavSatFix GPSRvizPoints_;
            select_gps_points_.clear();
            select_rviz_points_.clear();

            //近傍点の探索
            for(int i=0; i< g_waypoints_.size(); i++){
                s_g_data.sl_lat = (re_lat - g_waypoints_[i].GpsPoint.latitude) * cor_lat;
                s_g_data.sl_lon = (re_lon - g_waypoints_[i].GpsPoint.longitude) * cor_lon;
                if(fabs(s_g_data.sl_lat) < 3 && fabs(s_g_data.sl_lon) < 3){
                    s_g_data.RvizPoint.x = g_waypoints_[i].RvizPoint.x;
                    s_g_data.RvizPoint.y = g_waypoints_[i].RvizPoint.y;
                    select_gps_points_.push_back(s_g_data);
                }

            }
            if(select_gps_points_.size() == 0){
                ROS_ERROR("nyanto!!");
            }
            //近傍点からの距離から計測点の座標を求める
            for(int i=0; i<select_gps_points_.size(); i++){
                s_r_data.sl_x = (select_gps_points_[i].RvizPoint.x + select_gps_points_[i].sl_lon);
                s_r_data.sl_y = (select_gps_points_[i].RvizPoint.y + select_gps_points_[i].sl_lat);
                select_rviz_points_.push_back(s_r_data);
            }

            for(int i=0;i<select_rviz_points_.size();i++){
                x_g += select_rviz_points_[i].sl_x;
                y_g += select_rviz_points_[i].sl_y;
            }

            //latitude=x    longitude=y;    測位解
            GPSRvizPoints_.latitude                 = x_g /select_rviz_points_.size();
            GPSRvizPoints_.longitude                = y_g /select_rviz_points_.size();
            GPSRvizPoints_.position_covariance_type = fix->position_covariance_type;
            /*
            //近傍店から求められた座標から重心座標を求める
            //座標から面積を求めるS ＝ (1/2)Σ(i＝1,n){Xi*Y(i+1)-X(i+1)*Yi}
            for(int i=0; i<select_rviz_points_.size(); i++ ){
                if(i == (select_rviz_points_.size()-1)){
                    select_rviz_points_[i+1].sl_x = select_rviz_points_[0].sl_x;
                    select_rviz_points_[i+1].sl_y = select_rviz_points_[0].sl_y;
                }
                S += (select_rviz_points_[i].sl_x*select_rviz_points_[i+1].sl_y - select_rviz_points_[i+1].sl_x*select_rviz_points_[i].sl_y);
            }
            S /= 2;
            ROS_INFO("S:%lf",S);
            //重心座標を求める
            //Xg ＝ {1/(6*S)}*Σ(i=1,n){Xi+X(i+1)}*{Xi*Y(i+1)-X(i+1)*Yi}．
            //Yg ＝ {1/(6*S)}*Σ(i=1,n){Yi+Y(i+1)}*{Xi*Y(i+1)-X(i+1)*Yi}
            for(int i=0; i<select_rviz_points_.size(); i++){
                if(i == (select_rviz_points_.size()-1)){
                    select_rviz_points_[i+1].sl_x = select_rviz_points_[0].sl_x;
                    select_rviz_points_[i+1].sl_y = select_rviz_points_[0].sl_y;
                }
                x_g += ( (select_rviz_points_[i].sl_x + select_rviz_points_[i+1].sl_y) * (select_rviz_points_[i].sl_x*select_rviz_points_[i+1].sl_y - select_rviz_points_[i+1].sl_x*select_rviz_points_[i].sl_y) );
                y_g += ( (select_rviz_points_[i].sl_y + select_rviz_points_[i+1].sl_y) * (select_rviz_points_[i].sl_x*select_rviz_points_[i+1].sl_y - select_rviz_points_[i+1].sl_x*select_rviz_points_[i].sl_y) );
            }
            result_point_.point.x = x_g / (6*S);
            result_point_.point.y = y_g / (6*S);
            */
            /*for(int i=0; i < select_points_.size(); i++){
                x_g += (g_waypoints_[i].RvizPoint.x - select_points_[i].sl_lon);
                y_g += (g_waypoints_[i].RvizPoint.y + select_points_[i].sl_lat);
                ROS_INFO("%lf %lf",x_g,y_g);
            }
            result_point_.point.x = x_g / select_points_.size();
            result_point_.point.y = y_g / select_points_.size();
            */
            ROS_INFO("result:%lf  %lf",GPSRvizPoints_.latitude,GPSRvizPoints_.longitude);
            publishGPSMarker(GPSRvizPoints_.latitude,GPSRvizPoints_.longitude,0);
            position_GPS_pub_.publish(GPSRvizPoints_);

        }
    }else{
        //それ以外の処理　何もしない
        ROS_ERROR("hoge");
    }
    saved_gps_data[0] = re_lat;
    saved_gps_data[1] = re_lon;
}

void reference::publishMarkerDescription(){
	for (int i=0; i < g_waypoints_.size(); i++){
		 Marker marker;
		 marker.type = Marker::SPHERE;
		 marker.text = std::to_string(i);
		 marker.header.frame_id = world_frame_;
		 marker.header.stamp = ros::Time::now();
		 std::stringstream name;
		 name << "GpsWaypoint";
		 marker.ns = name.str();
		 marker.id = i;
		 marker.pose.position.x = g_waypoints_[i].RvizPoint.x;
		 marker.pose.position.y = g_waypoints_[i].RvizPoint.y;
		 marker.pose.position.z = 0.0;
		 marker.scale.x = 0.3f;
		 marker.scale.y = 0.3f;
		 marker.scale.z = 0.3f;

		 marker.color.r = 1.0f;
		 marker.color.g = 0.0f;
		 marker.color.b = 0.0f;
		 marker.color.a = 1.0;
		 marker.action = visualization_msgs::Marker::ADD;
		 marker_description_.markers.push_back(marker);
	}
	marker_description_pub_.publish(marker_description_);
}

//ポイントのマーカー表示
void reference::publishGPSMarker(double solution_x, double solution_y, double solution_z){
    Marker m_GPS;
    m_GPS.type = Marker::SPHERE;
    m_GPS.text = std::string("measurement_point");
    m_GPS.header.frame_id = world_frame_;
    m_GPS.header.stamp = ros::Time::now();
    std::stringstream name;
    name << "point";
    m_GPS.ns = name.str();
    m_GPS.id = 1;
    m_GPS.pose.position.x = solution_x;
    m_GPS.pose.position.y = solution_y;
    m_GPS.pose.position.z = solution_z;
    m_GPS.scale.x = 0.5f;
    m_GPS.scale.y = 0.5f;
    m_GPS.scale.z = 0.5f;

    m_GPS.color.r = 0.0f;
    m_GPS.color.g = 1.0f;
    m_GPS.color.b = 0.0f;
    m_GPS.color.a = 1.0;
    m_GPS.action = visualization_msgs::Marker::ADD;
    marker_GPS_.markers.push_back(m_GPS);

    GPS_marker_pub_.publish(marker_GPS_);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"table reference");
    ros::Time::init();
    reference reference;
    reference.run();

    return 0;
}
