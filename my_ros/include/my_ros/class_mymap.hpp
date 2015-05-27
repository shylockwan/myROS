#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace myros {
class mymap {
public:
	mymap(ros::NodeHandle n):nh_(n) {
		last_pt.x=last_pt.y=500;
		cur_pt=last_pt;
		last_ptobj=last_pt;
		cur_ptobj=last_pt;
		map = cv::Mat_<cv::Vec3b>::zeros(1000, 1000);
		objmap = cv::Mat_<cv::Vec3b>::zeros(1000, 1000);
		cv::circle(map, cur_pt, 1, cv::Scalar(0, 0, 255), 1);
		cv::namedWindow("map_robot/cm", CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
		cv::namedWindow("map_obj/cm", CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
		//cv::setMouseCallback("map_obj/cm", &(this->mouseCallback), 0);
	};
	/*void mouseCallback(int event, int x, int y, int flags, void* ustc) {
			if (event == CV_EVENT_LBUTTONDOWN) {
				CvPoint pt = cvPoint(x, y);
				char temp[16];
				sprintf(temp, "(%d,%d)", pt.x, pt.y);
				cv::putText(objmap, temp, pt, cv::FONT_HERSHEY_SIMPLEX, 2,
						cv::Scalar(0, 0, 255), 1);
				cv::circle(objmap, pt, 1, cv::Scalar(255, 255, 255), 1);

			}
		}*/
	void updateMap(const cv::Point& pt_robot,const cv::Point& pt_obj ) {

		cur_pt.x = (-pt_robot.x ) + 500;
		cur_pt.y = 500 - (pt_robot.y );
		cur_ptobj.x = (-pt_obj.x) + 500;
		cur_ptobj.y = 500 - (pt_obj.y );
		cv::circle(map, cur_pt, 1, cv::Scalar(0, 0, 255), 1);
		cv::line(map, last_pt, cur_pt, cv::Scalar(255, 255, 0), 1);
		cv::circle(objmap, cur_ptobj, 1, cv::Scalar(0, 0, 255), 1);
		cv::line(objmap, last_ptobj, cur_ptobj, cv::Scalar(255, 255, 0), 1);
		cv::imshow("map_robot/cm", map);
	    cv::imshow("map_obj/cm", objmap);
		cv::waitKey(30);
		last_pt = cur_pt;
		last_ptobj = cur_ptobj;
		ROS_INFO("pt_robot__x:   %d,   pt_robot__y:  %d", pt_robot.x,pt_robot.y);
		ROS_INFO("pt_obj__x:   %d,   pt_obj__y:  %d", pt_obj.x,pt_obj.y);
	}

private:
	cv::Point last_pt;
	cv::Point cur_pt;
	cv::Point last_ptobj;
	cv::Point cur_ptobj;
	ros::NodeHandle nh_;
	cv::Mat_<cv::Vec3b> map;
	cv::Mat_<cv::Vec3b> objmap;
};
}

