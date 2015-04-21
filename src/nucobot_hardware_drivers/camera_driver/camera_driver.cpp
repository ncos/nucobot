#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>


pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);


void cloudshow (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    ROS_INFO("Cloud size = %ld", cloud->size());
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ()){}
}

void image_preprocess (cv::Mat &image) {

}

void mat_to_cloud (cv::Mat &image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    uint8_t* pixelPtr = (uint8_t*)image.data;
    int channels = image.channels();
    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            cloud->points[i*image.rows+j].x = i;
            cloud->points[i*image.rows+j].y = j;
            cloud->points[i*image.rows+j].z = 0;
            cloud->points[i*image.rows+j].r = pixelPtr[i*image.cols*channels + j*channels + 2];
            cloud->points[i*image.rows+j].g = pixelPtr[i*image.cols*channels + j*channels + 1];
            cloud->points[i*image.rows+j].b = pixelPtr[i*image.cols*channels + j*channels + 0];
        }
    }
}


int main(int, char**)
{
    ROS_INFO("Using OpenCV %s", CV_VERSION);

    cv::Mat image;
    image = cv::imread("/home/ncos/y_1.jpg", CV_LOAD_IMAGE_COLOR);
    if(! image.data ) {
        ROS_ERROR("Could not open or find the image");
        return -1;
    }

    cv::resize(image, image, cv::Size(320,240));

    cloud->width  = image.rows*image.cols;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    image_preprocess(image);
    mat_to_cloud(image, cloud);

    //cloudshow(cloud);

    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", image);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (5);
    reg.setMinClusterSize (600);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud (colored_cloud);
    while (!viewer.wasStopped ())
    {
      //boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
    return 0;
}





