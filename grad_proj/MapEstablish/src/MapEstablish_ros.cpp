#include <MapEstablish_ros.hpp>

namespace establishmap {

EstablishMap_ros::EstablishMap_ros(ros::NodeHandle& nh) :
		nh_(nh), update_rate(5), process_rate(1), flag_enable(0), it_(nh), img_(
				NULL) {
	map_ = NULL;
	ready_se = NULL;
	waitloc_se = NULL;
	waitauto_se = NULL;
	logmap_se = NULL;
	qrscan_se=NULL;
	se_ = ready_se;
	rs_.g_Fuyang = 0.45;
	rs_.g_Chuizhi = 0.262;
	rs_.g_Shuiping = 0.34;
	rs_.g_Height = 32.5;
	rs_.m_PtRobot = cv::Point_<float>(0.0, 0.0);
	rs_.m_ThetaRobot = 0.0;
	e_.cur_state = EstablishMapEvent::ready;
	e_.next_state = EstablishMapEvent::ready;
	time1 = 0, time2 = 0, dtime = 0, leftv1 = 0, leftv2 = 0, rightv1 = 0, rightv2 =
			0; //encode_param_Init
}
;

EstablishMap_ros::~EstablishMap_ros() {
	delete ready_se;
	delete waitloc_se;
	delete waitauto_se;
	delete logmap_se;
	delete map_;
	se_ = NULL;
	thread.join();
}
;
bool EstablishMap_ros::onInit() {

	map_ = new whw::whwMap();
	ready_se = new StateReady();
	waitloc_se = new StateWait_SelfLoc(nh_);
	waitauto_se = new StateWait_AutoPatr(nh_);
	logmap_se = new StateLogMap(map_, nh_);
	qrscan_se=new StateQrScan(map_, nh_);

	se_ = ready_se;
	odom_sub = nh_.subscribe<nav_msgs::Odometry>("/odom", 1,
			&EstablishMap_ros::odomCallback, this);
	sub_img = it_.subscribe("/usb_cam/image_raw", 1, &EstablishMap_ros::imageCb,
			this);
	encoder_sub = nh_.subscribe("/mobile_base/sensors/core", 1,
			&EstablishMap_ros::encoderCallback, this);
	thread.start(&EstablishMap_ros::processEvent, *this);
	return 1;
}
;

void EstablishMap_ros::stateUpdate() {
	if (ros::ok()) {
		if (flag_enable && e_.flag_stateswitch) {
			ROS_INFO("stateUpdating.......");
			ros::param::get("fuyang", rs_.g_Fuyang);
			ros::param::get("chuizhi", rs_.g_Chuizhi);
			ros::param::get("shuiping", rs_.g_Shuiping);
			ros::param::get("height", rs_.g_Height);
			switch (e_.next_state) {
			case EstablishMapEvent::ready:
				if (e_.cur_state != EstablishMapEvent::ready) { //stateChange
					se_ = this->ready_se;
					e_.cur_state = EstablishMapEvent::ready;
					ROS_INFO("state transform to ready_______________________");
				}
				//else do nothing and spin as before
				break;
			case EstablishMapEvent::logmap:
				if (e_.cur_state != EstablishMapEvent::logmap) { //stateChange
					se_ = this->logmap_se;
					e_.cur_state = EstablishMapEvent::logmap;
					ROS_INFO(
							"state transform to logmap_______________________");
				}
				//else do nothing and spin as before
				break;
			case EstablishMapEvent::wait_selfloc:
				if (e_.cur_state != EstablishMapEvent::wait_selfloc) { //stateChange
					se_ = this->waitloc_se;
					e_.cur_state = EstablishMapEvent::wait_selfloc;
					ROS_INFO(
							"state transform to wait_selfloc_______________________");
				}
				//else do nothing and spin as before
				break;
			case EstablishMapEvent::wait_ap:
				if (e_.cur_state != EstablishMapEvent::wait_ap) { //stateChange
					se_ = this->waitauto_se;
					e_.cur_state = EstablishMapEvent::wait_ap;
					ROS_INFO(
							"state transform to wait_ap_______________________");
				}
				//else do nothing and spin as before
				break;
			case EstablishMapEvent::qrscan:
					if (e_.cur_state != EstablishMapEvent::qrscan) { //stateChange
						se_ = this->qrscan_se;
						StateQrScan* temp_ptr =dynamic_cast<StateQrScan*>(qrscan_se);
						temp_ptr->getImg(img_);
						e_.cur_state = EstablishMapEvent::qrscan;
						ROS_INFO(
								"state transform to qr_scan_______________________");
					}
					//else do nothing and spin as before
					break;

			default:
				ROS_INFO("what the fuck you wanna do");
				break;
			}
			e_.flag_stateswitch = 0;
		}
	}
};
void EstablishMap_ros::processEvent() {
	while (ros::ok()) {
		if (flag_enable && !e_.flag_stateswitch) {
			if (se_ != NULL && se_->onInit()) {
				se_->stateHandle(e_, rs_);
			}
		} else {
			std::cout << "无图像或状态子初始化失败" << std::endl;
			e_.report_ = 4;
		}

		if (e_.report_ != 0)
			this->disable();

		process_rate.sleep();
	}
}
;

void EstablishMap_ros::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	//ROS_INFO("(%f ,%f)  ,%f",msg->linear.x,msg->linear.y,msg->angular.z);
	if (0 == flag_enable)
		return;
	geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
	rs_.m_PtRobot.y = 100 * msg->pose.pose.position.x; //单位 m->cm
	rs_.m_PtRobot.x = -100 * msg->pose.pose.position.y;
	rs_.m_ThetaRobot = tf::getYaw(quat); //单位 rad
}

void EstablishMap_ros::encoderCallback(const kobuki_msgs::SensorState &msg) {
	int le_ = 0, re_ = 0;
//std::cout<<"encoder----------callback  "<<rs_.l_encoder<<"  "<<rs_.r_encoder<<std::endl;
	if (time1 == 0) {
		time1 = msg.time_stamp;
		time2 = time1;
		leftv2 = leftv1 = 0;
		rightv2 = rightv1 = 0;

	} else {
		time2 = time1;
		time1 = msg.time_stamp;
		dtime = time1 - time2;
		leftv1 = msg.left_encoder;
		rightv1 = msg.right_encoder;
		le_ = leftv1 >= leftv2 ?
				(leftv1 - leftv2) : ((65535 - leftv2) + leftv1);
		re_ = rightv1 >= rightv2 ?
				(rightv1 - rightv2) : ((65535 - rightv2) + rightv1);
		leftv2 = leftv1;
		rightv2 = rightv1;
	}
	rs_.l_encoder = rs_.l_encoder + le_;
	rs_.r_encoder = rs_.l_encoder + re_;
}
void EstablishMap_ros::imageCb(const sensor_msgs::ImageConstPtr& msg) {
	if (0 == flag_enable)
		return;

	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (img_ == NULL) {
		//	ROS_INFO("new img");
		img_ = new cv::Mat(cv_ptr->image);
	} else
		//	ROS_INFO("update img");
		cv_ptr->image.copyTo(*img_);

	cv::waitKey(20);
}
;
void EstablishMap_ros::Report() {
	switch (e_.report_) {
	case 0:
		//std::cout << std::endl;
		break;
	case 1:
		ROS_INFO("________FINISH__________");
		break;

	case 2:
		ROS_INFO("________AP_ERROR__________");
		break;
	case 3:
		ROS_INFO("__________LOC_ERROR__________");
		break;
	case 4:
		ROS_INFO("________QR_SCAN_FAIL__________");
		break;
	default:
		break;
	}
}
;
void EstablishMap_ros::resetMap() {
	se_ = ready_se;
	e_.cur_state = EstablishMapEvent::ready;
	e_.next_state = EstablishMapEvent::ready;
	e_.map_state = 0;
	time1 = 0;
	//map_->reset();
}
;
void EstablishMap_ros::enable() {
	flag_enable = 1;
	e_.report_ = 0;
	e_.flag_stateswitch = 1;
}
;
void EstablishMap_ros::disable() {
	flag_enable = 0;

	e_.flag_stateswitch = 1;

}
;
bool EstablishMap_ros::getState() {
	return flag_enable;
}
;
//
}
