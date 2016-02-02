#include <autopatrol_ros.hpp>
#include <autopatrol.hpp>
namespace autopatrol {
AutoPatrol_ros::AutoPatrol_ros(ros::NodeHandle& nh) :
		 nh_(nh) ,it_(nh),process_rate(5),
		img_(NULL),flag_enable(0)
{
	se_=NULL;//Base
	ready_se=NULL;
	ap_se=NULL;
	uturn_se=NULL;;
	turnangle_se=NULL;
	qrcover_se=NULL;
	qrloc_se=NULL;
	flag_enable=0;
	event_.cur_state=AutoPatrolEvent::ready;
	event_.next_state=AutoPatrolEvent::ready;
	event_.flag_stateswitch=1;//允许状态切换可能

	event_.report_=0;
	cmd_from=0;
	repo_srv.request.id=1;
	flag_qr=0;
	};

AutoPatrol_ros::~AutoPatrol_ros(){
	delete ready_se;
	delete ap_se;
	delete uturn_se;
	delete turnangle_se;
	delete qrloc_se;
	delete qrcover_se;
	se_=NULL;//Base
	ready_se=NULL;
	ap_se=NULL;
	uturn_se=NULL;;
	turnangle_se=NULL;
	event_thread.join();
};

bool AutoPatrol_ros::onInit() {
	ready_se = new StateReady();
	ap_se = new StateAP();
	uturn_se=new StateUturn();
	turnangle_se=new StateTurnAngle();
	qrloc_se=new StateQrloc();
	qrcover_se=new StateQrcover();
	se_ = ready_se;
	rs_.g_Fuyang = 0.45;
	rs_.g_Chuizhi = 0.262;
	rs_.g_Shuiping = 0.34;
	rs_.g_Height = 32.5;
	rs_.m_PtRobot = cv::Point_<float>(0.0, 0.0);
	rs_.m_ThetaRobot = 0.0;
	//repot_clientss;
	vel_pub = nh_.advertise<geometry_msgs::Twist>(
			"/mobile_base/commands/velocity", 1);

	img_sub = it_.subscribe("/usb_cam/image_raw", 1, &AutoPatrol_ros::imageCb,
			this);
	odom_sub = nh_.subscribe<nav_msgs::Odometry>("/odom", 1,
			&AutoPatrol_ros::odomCallback, this);
	//repo_client = nh_.serviceClient("/linedetect/report");
	event_thread.start(&AutoPatrol_ros::processEvent, *this);
	std::cout<<"initial finish"<<std::endl;
	return 1;
};

void AutoPatrol_ros::stateUpdate() {
	if (ros::ok()) {
		ROS_INFO("stateUpdating.......");
		ros::param::get("fuyang", rs_.g_Fuyang);
		ros::param::get("chuizhi", rs_.g_Chuizhi);
		ros::param::get("shuiping", rs_.g_Shuiping);
		ros::param::get("height", rs_.g_Height);
		if (event_.flag_stateswitch == 1 && flag_enable == 1) {
			switch (event_.next_state) {
			case AutoPatrolEvent::ready:
				if (event_.cur_state != AutoPatrolEvent::ready) { //stateChange
					se_ = this->ready_se;
					event_.cur_state = AutoPatrolEvent::ready;
					ROS_INFO("state transform to ready_______________________");
				}
				break;
			case AutoPatrolEvent::autopatrol:
				if (event_.cur_state != AutoPatrolEvent::autopatrol) { //stateChange
					se_ = this->ap_se;
					event_.cur_state = AutoPatrolEvent::autopatrol;
					ROS_INFO("state transform to autopatrol__________________");
				}
				break;
			case AutoPatrolEvent::qrcover:
				if (event_.cur_state != AutoPatrolEvent::qrcover) { //stateChange
					se_ = this->qrcover_se;
					event_.cur_state = AutoPatrolEvent::qrcover;
					ROS_INFO("state transform to qrcover__________________");
				}
				break;
			case AutoPatrolEvent::qrloc:
				if (event_.cur_state != AutoPatrolEvent::qrloc) { //stateChange
					se_ = this->qrloc_se;
					event_.cur_state = AutoPatrolEvent::qrloc;
					ROS_INFO("state transform to qrloc__________________");
				}
				break;
			case AutoPatrolEvent::angleturn:
				if (event_.cur_state != AutoPatrolEvent::angleturn) { //stateChange
					se_ = this->turnangle_se;
					event_.cur_state = AutoPatrolEvent::angleturn;
					ROS_INFO("state transform to turnangle__________________");
				}
				break;
			case AutoPatrolEvent::uturn:
				if (event_.cur_state != AutoPatrolEvent::uturn) { //stateChange
					se_ = this->uturn_se;
					event_.cur_state = AutoPatrolEvent::uturn;
					ROS_INFO("state transform to uturn__________________");
				}
				break;
			default:
				ROS_INFO("Wrong state transform ");
				break;
			}
			event_.flag_stateswitch = 0;
		}

	}

};

void AutoPatrol_ros::processEvent(){
	while(ros::ok())
	{
		if(flag_enable&&event_.flag_stateswitch==0)
		{
			if (se_ != NULL && se_->onInit()&&img_!=NULL) {
				mu_.lock();
				cv::Mat src=cv::Mat(*img_);
				mu_.unlock();
				se_->stateHandle(src,event_,rs_);
				geometry_msgs::Twist t = se_->getTwist();
				//std::cout<<std::endl<<"linear.x:  "<<t.linear.x<<"   angular.z"<<t.angular.z<<std::endl;
				vel_pub.publish(t);
			}
			else
			{
				std::cout<<"无图像或状态子初始化失败"<<std::endl;
				event_.report_=4;
			}
		}
		if(event_.report_!=0)
			this->disable();

		process_rate.sleep();
	}

};

void AutoPatrol_ros::imageCb(const sensor_msgs::ImageConstPtr& msg){

	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	if(img_==NULL)
	{
	//	ROS_INFO("new img");
		img_=new cv::Mat(cv_ptr->image);
	}

	if (0 == flag_enable)
		return;
	else {
		mu_.lock();
		cv_ptr->image.copyTo(*img_);
		mu_.unlock();
	}
    cv::waitKey(30);
};

void AutoPatrol_ros::enable(){
	std::cout<<"________________________Node Enable________________________"<<std::endl;
	event_.cur_state=AutoPatrolEvent::ready;
	flag_enable=1;
	event_.report_=0;
	event_.flag_stateswitch=1;
};

void AutoPatrol_ros::disable(){
	std::cout<<"________________________Node Disable________________________"<<std::endl;
	flag_enable=0;
	event_.flag_stateswitch=1;
	event_.cur_state=AutoPatrolEvent::ready;
	event_.next_state=AutoPatrolEvent::ready;
	uturn_se->reset();
	turnangle_se->reset();
	Report(); //向上报道模块状态
	cmd_from=0;
	event_.report_=0;
	flag_qr=0;
};

void AutoPatrol_ros::Report(){
	switch(event_.report_)
	{
	case 0:
		//std::cout << std::endl;success
		break;
	case 1:
		ROS_INFO("__________CUR_SERVICE FINISHED__________");
		repo_srv.request.srv_result=0;
		break;
	case 2:
		ROS_INFO("__________NO_OBJ_POINT__________");
		repo_srv.request.srv_result=1;
			break;
	case 3:
		ROS_INFO("__________AP_ERROR__________");
		repo_srv.request.srv_result=1;
		break;
	case 5:
		ROS_INFO("________TurnAngle_ERROR__________");
		repo_srv.request.srv_result=1;
		break;
	case 6:
		ROS_INFO("________Uturn_ERROR__________");
		repo_srv.request.srv_result=1;
		break;
	case 4:
		ROS_INFO("________NO INPUT IMG OR STATEIDE ERROR__________");
		repo_srv.request.srv_result=1;
		break;
	default:
		ROS_INFO("what the fuck you wanna do");
		repo_srv.request.srv_result=1;
		break;
	}
	//1=autopatrol
	//2=establishmap
	//999=testport
	switch (cmd_from) {
	case 1:
		break;
	case 2:
		callForReport("/autotrack_map/report");
		std::cout << "/autotrack_map/report" << std::endl;
		break;
	case 999:
		callForReport("/autotrack_test/report");
		std::cout << "__________CMD FROM TEST PORT__________" << std::endl;
		break;
	default:
		break;
	}
};

void AutoPatrol_ros::getRequestID(const int& id){
	cmd_from=id;
};

void AutoPatrol_ros::getPathInfo(const PathLink& route){
	event_.next_route->angle_from=route.angle_from;
	event_.next_route->angle_to=route.angle_to;
	event_.next_route->flag_uturn=route.flag_uturn;
	event_.next_route->id_from=route.id_from;
	event_.next_route->id_to=route.id_to;
};

void AutoPatrol_ros::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	//ROS_INFO("(%f ,%f)  ,%f",msg->linear.x,msg->linear.y,msg->angular.z);
	if (0 == flag_enable)
		return;
	geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
	rs_.m_PtRobot.y = 100 * msg->pose.pose.position.x; //单位 m->cm
	rs_.m_PtRobot.x = -100 * msg->pose.pose.position.y;
	rs_.m_ThetaRobot = tf::getYaw(quat); //单位 rad
};

bool AutoPatrol_ros::getState(){
	return flag_enable;
};

void AutoPatrol_ros::setQrAngle(const float& angle){
	flag_qr=1;
	rs_.qr_angle=angle;
};

void AutoPatrol_ros::callForReport(const std::string& srvname)
{
	repo_client= nh_.serviceClient<libpkg::report>(srvname);
	repo_client.call(repo_srv);
	return;
	}

}
