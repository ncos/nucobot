#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



#include <aruco.h>
#include <cvdrawingutils.h>

using namespace cv;
using namespace aruco;

int main(int argc,char **argv)
{
    ROS_INFO("Using OpenCV %s", CV_VERSION);

    VideoCapture cap(1); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    try {
        namedWindow("Output",1);
        MarkerDetector MDetector;

        for(;;)
        {
            vector<Marker> Markers;
            Mat frame;
            cap >> frame; // get a new frame from camera
            MDetector.detect(frame, Markers);
            //cvtColor(frame, edges, COLOR_BGR2GRAY);
            for (unsigned int i=0; i<Markers.size(); ++i) {
                //cout<<Markers[i]<<endl;
                Markers[i].draw(frame, Scalar(0,0,255), 2);
            }

            imshow("Output", frame);
            if(waitKey(30) >= 0) break;
        }

    } catch (std::exception &ex) {
        cout<<"Exception :"<<ex.what()<<endl;
    }
}
