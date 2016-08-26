#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>

#include <std_srvs/Trigger.h>

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

class GpsPointEditor{
public:
    GpsPointEditor() :
        filename_(""), fp_flag_(false), rate_(10)
    {
        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;
        marker_description_pub_ = nh.advertise<visualization_msgs::MarkerArray>("marker_descriptions",1);
        private_nh.param("filename", filename_, filename_);
        if(filename_ != ""){
            ROS_INFO_STREAM("Read waypoints data from " << filename_);
            if(readFile(filename_)) {
                fp_flag_ = true;
             } else {
                ROS_ERROR("Failed loading gps data file");
            }
        } else {
            ROS_ERROR("gps data file doesn't have name");
        }
    }

	bool readFile(const std::string &filename);
	void publishMarkerDescription();	
    void run() {
        while(ros::ok()){
            rate_.sleep();
            ros::spinOnce();
        }
    }

private:
    ros::Publisher marker_description_pub_;
    visualization_msgs::MarkerArray marker_description_;
    std::string filename_;
    bool fp_flag_;
    ros::Rate rate_;
	struct GpsData{
		geometry_msgs::Point RvizPoint;
		sensor_msgs::NavSatFix GpsPoint;
	};
    std::vector<GpsData> g_waypoints_;
};


bool GpsPointEditor::readFile(const std::string &filename){
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
			const YAML::Node *gp_node = node.FindValue("gps_positionss");
		#endif
		
		if(gp_node != NULL){
			GpsData g_data;
			for(int i=0; i < gp_node->size(); i++){
				//geometry_msgs::Point point;
				(*gp_node)[i]["point"]["x"] >> g_data.RvizPoint.x;
				(*gp_node)[i]["point"]["y"] >> g_data.RvizPoint.y;
				(*gp_node)[i]["point"]["z"] >> g_data.RvizPoint.z;
				(*gp_node)[i]["point"]["lat"] >> g_data.GpsPoint.latitude;
				(*gp_node)[i]["point"]["lon"] >> g_data.GpsPoint.longitude;
			   g_waypoints_.push_back(g_data );
			}
			/*for(int i=0; i< g_waypoints_.size(); i++){
				//pushback確認用
			  	ROS_INFO("%lf",g_waypoints_[i].RvizPoint.x);
			}*/
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

void GpsPointEditor::publishMarkerDescription(){
	for (int i=0; i!=g_waypoints_.size(); i++){
    	 Marker marker;
		 marker.type = Marker::TEXT_VIEW_FACING;
		// marker.text = std::to_string(i);
		 marker.header.stamp = ros::Time::now();
		 std::stringstream name;
		 name << "gps_point";
		 marker.ns = name.str();
		 marker.id = i;
		// marker.pose.position = g_waypoints_.at(i);
		 marker.pose.position.z = 3.0;
		 marker.scale.z = 2.0;
		 marker.color.r = 0.0;
		 marker.color.g = 0.0;
		 marker.color.b = 0.0;
		 marker.color.a = 1.0;
		 marker.action = visualization_msgs::Marker::ADD;
		 marker_description_.markers.push_back(marker);
	}
	marker_description_pub_.publish(marker_description_);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pose_marker");
  ros::Time::init();
  GpsPointEditor g_edit;
  g_edit.run();

  return 0;
}
