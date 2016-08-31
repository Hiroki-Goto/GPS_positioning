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
        private_nh.param("world_frame", world_frame_, std::string("map"));
		
		server.reset(new interactive_markers::InteractiveMarkerServer("gps_waypoints_marker_server", "", false));
		initMenu();

		ros::NodeHandle nh;
        marker_description_pub_ = nh.advertise<visualization_msgs::MarkerArray>("marker_descriptions",1);
		finish_pose_sub_ = nh.subscribe("finish_pose",1,&GpsPointEditor::finishPoseCallback,this);

		private_nh.param("filename", filename_, filename_);
        if(filename_ != ""){
            ROS_INFO_STREAM("Read waypoints data from " << filename_);
            if(readFile(filename_)) {
                fp_flag_ = true;
		makeMarker();
             } else {
                ROS_ERROR("Failed loading gps data file");
            }
        } else {
            ROS_ERROR("gps data file doesn't have name");
        }
    }

	bool save_gps_WaypointsCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
	void finishPoseCallback(const geometry_msgs::PoseStamped &msg);
	bool readFile(const std::string &filename);
	void publishMarkerDescription();
	void initMenu();
	void fpDeleteCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void wpDeleteCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void resetMarkerDescription();
	void makeWaypointsMarker();
	void applyMenu(){
			for (int i=0; i < g_waypoints_.size(); i++){
				wp_menu_handler_.apply(*server, "waypoint"+std::to_string(i));
			}
			server->applyChanges();
	}

	void run() {
        while(ros::ok()){
            rate_.sleep();
            ros::spinOnce();
			publishMarkerDescription();
        }
    }

private:
	void makeMarker();
	void save();
	ros::Subscriber finish_pose_sub_;
    ros::Publisher marker_description_pub_;
    visualization_msgs::MarkerArray marker_description_;
    std::string filename_;
	std::string world_frame_;
    bool fp_flag_;
    ros::Rate rate_;
	struct GpsData{
		geometry_msgs::Point RvizPoint;
		sensor_msgs::NavSatFix GpsPoint;
	};
    std::vector<GpsData> g_waypoints_;

	boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
	interactive_markers::MenuHandler wp_menu_handler_;
	interactive_markers::MenuHandler fp_menu_handler_;

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
			const YAML::Node *gp_node = node.FindValue("gps_position");
		#endif
		
		if(gp_node != NULL){
			GpsData g_data;
			for(int i=0; i < gp_node->size(); i++){
				(*gp_node)[i]["point"]["x"] >> g_data.RvizPoint.x;
				(*gp_node)[i]["point"]["y"] >> g_data.RvizPoint.y;
				(*gp_node)[i]["point"]["z"] >> g_data.RvizPoint.z;
				(*gp_node)[i]["point"]["lat"] >> g_data.GpsPoint.latitude;
				(*gp_node)[i]["point"]["lon"] >> g_data.GpsPoint.longitude;
			   g_waypoints_.push_back(g_data );
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

void GpsPointEditor::publishMarkerDescription(){
	for (int i=0; i < g_waypoints_.size(); i++){
		 Marker marker;
		 marker.type = Marker::TEXT_VIEW_FACING;
		 marker.text = std::to_string(i);
		 marker.header.frame_id = world_frame_;
		 marker.header.stamp = ros::Time::now();
		 std::stringstream name;
		 name << "waypoint";
		 marker.ns = name.str();
		 marker.id = i;
		 marker.pose.position.x = g_waypoints_[i].RvizPoint.x;
		 marker.pose.position.y = g_waypoints_[i].RvizPoint.y;
		 marker.pose.position.z = 3.0;
		 marker.scale.x = 2.5f;
		 marker.scale.y = 2.5f;
		 marker.scale.z = 2.0f;

		 marker.color.r = 1.0f;
		 marker.color.g = 0.0f;
		 marker.color.b = 0.0f;
		 marker.color.a = 1.0;
		 marker.action = visualization_msgs::Marker::ADD;
		 marker_description_.markers.push_back(marker);
	}
	marker_description_pub_.publish(marker_description_);
}

void GpsPointEditor::finishPoseCallback(const geometry_msgs::PoseStamped &msg){
	save();
}

void GpsPointEditor::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
        std::ostringstream s;
        s << "Feedback from marker '" << feedback->marker_name << "'";

		std::ostringstream mouse_point_ss;

		switch(feedback->event_type){
			case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
				ROS_INFO_STREAM(s.str() << ":button click" << mouse_point_ss.str() << ".");
		   	break;
    
			case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
	            ROS_INFO_STREAM(s.str() << ":menu item" << feedback->menu_entry_id << "clicked" << mouse_point_ss.str() << ".");
	        break;
		}

		server->applyChanges();
}

void GpsPointEditor::initMenu(){
		interactive_markers::MenuHandler::EntryHandle wp_delete_menu_handler = wp_menu_handler_.insert("delete", boost::bind(&GpsPointEditor::wpDeleteCb, this, _1));
		interactive_markers::MenuHandler::EntryHandle fp_delete_menu_handler = fp_menu_handler_.insert("delete", boost::bind(&GpsPointEditor::fpDeleteCb, this, _1));
}

void GpsPointEditor::wpDeleteCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        std::string str_wp_num = feedback->marker_name;
        ROS_INFO_STREAM("delete : " << feedback->marker_name);
		resetMarkerDescription();
		g_waypoints_.erase(g_waypoints_.begin() + std::stoi(str_wp_num.substr(8)));
		makeMarker();
}


void GpsPointEditor::fpDeleteCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        std::string str_wp_num = feedback->marker_name;
        ROS_INFO_STREAM("delete : " << feedback->marker_name);
        resetMarkerDescription();
        fp_flag_ = false;
        makeMarker();
}

void GpsPointEditor::makeMarker(){
        server->clear();
		server->applyChanges();
		makeWaypointsMarker();
        applyMenu();
        server->applyChanges();
}

void GpsPointEditor::resetMarkerDescription(){
		for (int i=0; i < g_waypoints_.size(); i++){
				Marker marker;
			    marker.type = Marker::TEXT_VIEW_FACING;
			    marker.text = std::to_string(i);
				marker.header.frame_id = world_frame_;
				marker.header.stamp = ros::Time::now();
				std::stringstream name;
				name << "waypoint";
				marker.ns = name.str();
	            marker.id = i;
	            uint8_t DELETEALL = 3;
	            marker.action = DELETEALL;
	            marker_description_.markers.push_back(marker);
        }
		marker_description_pub_.publish(marker_description_);
}


void GpsPointEditor::makeWaypointsMarker(){
        for (int i=0; i!=g_waypoints_.size(); i++){
            InteractiveMarker int_marker;
            int_marker.header.frame_id = world_frame_;
            int_marker.pose.position.x = g_waypoints_[i].RvizPoint.x;
	    	int_marker.pose.position.y = g_waypoints_[i].RvizPoint.y;
	    	int_marker.scale = 1;
            int_marker.name = "waypoint"+std::to_string(i);
            int_marker.description = "waypoint"+std::to_string(i);

			
            InteractiveMarkerControl control;
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.interaction_mode = InteractiveMarkerControl::MENU;
            int_marker.controls.push_back(control);
    		
            Marker marker;
            marker.type = Marker::SPHERE;
            marker.scale.x = 0.8;
            marker.scale.y = 0.8;
            marker.scale.z = 0.8;
            marker.color.r = 0.08;
            marker.color.g = 0.0;
            marker.color.b = 0.8;
            marker.color.a = 0.5;
            control.markers.push_back(marker);
            
            control.always_visible = true;
            int_marker.controls.push_back(control);
    
            server->insert(int_marker);
            server->setCallback(int_marker.name, boost::bind(&GpsPointEditor::processFeedback, this, _1));
        }
}

void GpsPointEditor::save(){
	std::ofstream ofs(filename_.c_str(), std::ios::out);
	ofs << "gps_position:" << std::endl;
	for(int i=0; i < g_waypoints_.size(); i++){
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

int main(int argc, char** argv){
  ros::init(argc, argv, "pose_marker");
  ros::Time::init();
  GpsPointEditor g_edit;
  g_edit.run();

  return 0;
}
