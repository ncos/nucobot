#include <vector>
#include <cmath>
#include <cstring>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


ros::Subscriber sub_gazbo;
ros::Subscriber sub_target_obj;
ros::Publisher  pub_markr;
ros::Publisher  pub_cloud;
ros::Publisher  pub_obj;

ros::ServiceClient clear_map_srv;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

std_msgs::String target_obj;
bool target_obj_changed;
std::vector<std::string> untracked_objs;

void gen_flatline(double x1, double y1, double x2, double y2)
{
    double step = 0.04;
    double ln = std::sqrt(std::pow(x2-x1, 2) + std::pow(y2-y1, 2));
    double dlx = step*(x2 - x1)/ln;
	double dly = step*(y2 - y1)/ln;
    for (int i = 0; i < ln/step; ++i) {
    	cloud->push_back(pcl::PointXYZ(x1 + i*dlx, y1 + i*dly, 0));//->points.push_back(pcl::PointXYZ(x1 + i*dlx, y1 + i*dly, 0));
    }
};

void gen_flatrect(double x1, double y1, double x2, double y2)
{
    double step = 0.02;
    for (int i = 0; i < (x2 - x1)/step; ++i)
        for (int j = 0; j < (y2 - y1)/step; ++j)
        	cloud->push_back(pcl::PointXYZ(x1 + i*step, y1 + j*step, 0));
};

void gen_flatcircle(double r, double x, double y)
{
    double step = 2 * M_PI / 12; // 12 points
	double phi = 0;
    while (phi < 6.3) {
    	cloud->push_back(pcl::PointXYZ(x + r*std::sin(phi), y + r*std::cos(phi), 0));
        phi += step;
    }
};


void target_obj_callback(const std_msgs::String::ConstPtr &obj)
{
    if (target_obj.data != obj->data) {
        target_obj_changed = true;
        target_obj.data = obj->data;
        if (!(std::find(untracked_objs.begin(), untracked_objs.end(), obj->data) != untracked_objs.end()))
            untracked_objs.push_back(obj->data);
    }
}

void map_callback(const gazebo_msgs::ModelStates::ConstPtr &data)
{
	visualization_msgs::MarkerArray ma;
	cloud->header.frame_id = "map";

	bool need_add_to_map = true;

    for (int i = 0; i < data->name.size(); ++i) {
	    visualization_msgs::Marker marker, text_marker;
        marker.header.frame_id = "map";
        marker.id = 0;
        marker.ns = data->name[i];
        marker.header.stamp = ros::Time();
        marker.lifetime = ros::Duration(0.3);
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = data->pose[i];

        text_marker.header.frame_id = "map";
        text_marker.id = 1;
        text_marker.ns = data->name[i];
        text_marker.header.stamp = ros::Time();
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.lifetime = ros::Duration(0.3);
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose = data->pose[i];
        text_marker.text = data->name[i];
        text_marker.scale.x = 0.10;
        text_marker.scale.y = 0.10;
        text_marker.scale.z = 0.05;
        text_marker.color.a = 0.90;
        text_marker.color.r = 1.00;
        text_marker.color.g = 1.00;
        text_marker.color.b = 1.00;


        need_add_to_map = !(std::find(untracked_objs.begin(), untracked_objs.end(), data->name[i]) != untracked_objs.end());

        std_srvs::Empty srv;

        if (!need_add_to_map && target_obj_changed) {
            ROS_ERROR("UntrackedObj\n==========================");
            for (int j = 0; j < untracked_objs.size(); ++j)
                ROS_ERROR("%s", untracked_objs.at(j).c_str());
            ROS_ERROR("========================== %d", target_obj_changed);
            if (clear_map_srv.call(srv))
                target_obj_changed = false;
            else
                ROS_ERROR("Fake_towermap: Failed to call service clear_map_srv");
        }


        if (data->name[i].find("cylinder") != std::string::npos) {
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.scale.x = 0.060;
            marker.scale.y = 0.060;
            marker.scale.z = 0.070;
            marker.color.a = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            ma.markers.push_back(marker);
            ma.markers.push_back(text_marker);
            if(need_add_to_map)
                gen_flatcircle(marker.scale.x / 2.0, marker.pose.position.x, marker.pose.position.y);
        }

        if (data->name[i].find("tennis") != std::string::npos) {
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.064;
            marker.scale.y = 0.064;
            marker.scale.z = 0.064;
            marker.color.a = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            ma.markers.push_back(marker);
            ma.markers.push_back(text_marker);
            if(need_add_to_map)
                gen_flatcircle(marker.scale.x / 2.0, marker.pose.position.x, marker.pose.position.y);
        }

        if (data->name[i].find("pop_corn") != std::string::npos) {
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.040;
            marker.scale.y = 0.040;
            marker.scale.z = 0.040;
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            ma.markers.push_back(marker);
            ma.markers.push_back(text_marker);
            if(need_add_to_map)
                gen_flatcircle(marker.scale.x / 2.0, marker.pose.position.x, marker.pose.position.y);
        }

        if (data->name[i].find("cup") != std::string::npos) {
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.pose.position.z += 0.075;
            marker.scale.x = 0.095;
            marker.scale.y = 0.095;
            marker.scale.z = 0.150;
            marker.color.a = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            ma.markers.push_back(marker);
            ma.markers.push_back(text_marker);
            if(need_add_to_map)
                gen_flatcircle(marker.scale.x / 2.0, marker.pose.position.x, marker.pose.position.y);
        }
    }

    pub_cloud.publish(cloud);
	pub_markr.publish(ma);
	pub_obj.publish(data);
	cloud->clear();
};



int main( int argc, char** argv)
{
    ros::init(argc, argv, "fake_towermap");
    ros::NodeHandle nh;

    sub_gazbo      = nh.subscribe<gazebo_msgs::ModelStates>  ("/gazebo/model_states", 1, map_callback);
    sub_target_obj = nh.subscribe<std_msgs::String> ("action_server/target_objects", 1, target_obj_callback);
    pub_markr      = nh.advertise<visualization_msgs::MarkerArray> ("visualization/fake_towermap/objects", 1);
	pub_cloud      = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("fake_towermap/pointcloud", 1);
	pub_obj        = nh.advertise<gazebo_msgs::ModelStates>  ("fake_towermap/objects", 1);

	clear_map_srv  = nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");

	target_obj_changed = false;

    ros::spin ();
    return 0;
};
