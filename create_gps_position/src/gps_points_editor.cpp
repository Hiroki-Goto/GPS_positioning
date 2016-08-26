#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <sensor_msgs/Joy.h>
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
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif
using namespace visualization_msgs;

class WaypointsEditor{
public:
    WaypointsEditor() :
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
                ROS_ERROR("Failed loading waypoints file");
            }
        } else {
            ROS_ERROR("waypoints file doesn't have name");
        }
    }

	void publishMarkerDescription(){

        for (int i=0; i!=waypoints_.size(); i++){
            Marker marker;
            marker.type = Marker::TEXT_VIEW_FACING;
            marker.text = std::to_string(i);
            marker.header.stamp = ros::Time::now();
            std::stringstream name;
            name << "waypoint";
            marker.ns = name.str();
            marker.id = i;
            marker.pose.position = waypoints_.at(i);
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

    bool readFile(const std::string &filename){
        waypoints_.clear();
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
                for(int i=0; i < gp_node->size(); i++){
                    geometry_msgs::Point point;

                    (*gp_node)[i]["point"]["x"] >> point.x;
                    (*gp_node)[i]["point"]["y"] >> point.y;
                    (*gp_node)[i]["point"]["z"] >> point.z;

                    waypoints_.push_back(point);
                }
            }else{
				ROS_WARN("nya");
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

    void run() {
        while(ros::ok()){
            rate_.sleep();
            ros::spinOnce();
        }
    }

private:
    ros::Publisher marker_description_pub_;
    std::vector<geometry_msgs::Point> waypoints_;
    visualization_msgs::MarkerArray marker_description_;
    std::string filename_;
    bool fp_flag_;
    ros::Rate rate_;
};


int main(int argc, char** argv){
  ros::init(argc, argv, "pose_marker");
  ros::Time::init();
  WaypointsEditor w_edit;
  w_edit.run();

  return 0;
}
