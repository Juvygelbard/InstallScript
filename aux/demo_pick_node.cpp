


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <moveit/move_group_interface/move_group.h>

typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace cv;

bool debug_vision=false;

move_base_msgs::MoveBaseGoal move_to_table();
move_base_msgs::MoveBaseGoal move_to_button();
move_base_msgs::MoveBaseGoal get_pre_pick_pose();
bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA> cloud,Point3d *obj);
void go(tf::Transform  dest);
bool gripper_cmd(double gap,double effort);
bool arm_cmd(geometry_msgs::PoseStamped target_pose1);
geometry_msgs::PoseStamped move_to_object();
geometry_msgs::PoseStamped lift_arm();
bool base_cmd(move_base_msgs::MoveBaseGoal goal);
void pick_go_cb(std_msgs::Empty);
void button_go_cb(std_msgs::Empty);
void look_down();

GripperClient *gripperClient_ptr;
MoveBaseClient *moveBaseClient_ptr;
moveit::planning_interface::MoveGroup *moveit_ptr;
tf::TransformListener *listener_ptr;

tf::StampedTransform kinect_obj_transform,base_obj_transform;


bool have_goal=false;
//bool obj_tf_ok=false;
int image_w=0,image_h=0;
bool moving=false;


image_transport::Publisher result_image_pub;
image_transport::Publisher object_image_pub;
image_transport::Publisher bw_image_pub;

image_transport::Subscriber image_sub;

ros::Publisher planning_scene_diff_publisher;
ros::Publisher goal_pub;
ros::Publisher arm_pub;
ros::Publisher nav_pub;

//bgumodo
ros::Publisher object_pub;

ros::Publisher pan_tilt_pub;

double distance_from_object=0.55;
geometry_msgs::PoseStamped moveit_goal;
//red
int minH=3,maxH=160;
int minS=70,maxS=255;
int minV=70,maxV=255;
int minA=5000,maxA=50000;
int gaussian_ksize=0;
int gaussian_sigma=0;
int morph_size=0;


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat bgr=cv_ptr->image;

    if (image_w==0) image_w=bgr.cols;
    if (image_h==0) image_h=bgr.rows;

}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

    if ((image_w==0)||(image_h==0)) return;

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromROSMsg (*input, cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

    Mat result;

    result = Mat(image_h, image_w, CV_8UC3);

    if (!cloudp->empty()) {
        for (int h=0; h<image_h; h++) {
            for (int w=0; w<image_w; w++) {
                int pcl_index = (h*image_w) + w;
                pcl::PointXYZRGBA point = cloudp->at(pcl_index);
                if (point.z>0.1) {
                    // Eigen::Vector3i rgb = point.getRGBVector3i();
                    result.at<cv::Vec3b>(h,w)[0] = point.b;
                    result.at<cv::Vec3b>(h,w)[1] = point.g;
                    result.at<cv::Vec3b>(h,w)[2] = point.r;
                }
                else {
                    result.at<cv::Vec3b>(h,w)[0]=0;
                    result.at<cv::Vec3b>(h,w)[1]=0;
                    result.at<cv::Vec3b>(h,w)[2]=0;
                }
            }
        }

    }
    else {
        ROS_WARN("empty cloud");
        return;
    }


    Point3d obj;
    bool have_object= find_object(result,cloud,&obj);

    waitKey(1);

    if (have_object) {

        // obj_tf_ok=have_object;

        kinect_obj_transform.setOrigin( tf::Vector3(obj.x,obj.y,obj.z) );
        tf::Quaternion q;
        q.setRPY(0.0, 0, 0);
        kinect_obj_transform.setRotation(q);
        kinect_obj_transform.frame_id_="kinect2_depth_optical_frame";
        kinect_obj_transform.child_frame_id_="red_object";
        kinect_obj_transform.stamp_=ros::Time::now();

        // obj_tf_ok=true;
    }
}

bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA> cloud,Point3d *pr) {

    Mat hsv,filtered,bw;

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp=ros::Time::now();
    out_msg.header.frame_id="kinect2_depth_optical_frame";

    cvtColor(input,hsv,CV_BGR2HSV);


    // Threshold the HSV image, keep only the red pixels
    Mat lower_red_hue_range;
    Mat upper_red_hue_range;
    inRange(hsv, cv::Scalar(0, minS, minV), cv::Scalar(minH, maxS, maxV), lower_red_hue_range);
    inRange(hsv, cv::Scalar(maxH, minS, minV), cv::Scalar(179, maxS, maxV), upper_red_hue_range);
    // Combine the above two images
    Mat red_hue_image;
    addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

    //if not red use:
    // inRange(hsv,Scalar(minH,minS,minV),Scalar(maxH,maxS,maxV),red_hue_image);

    hsv.copyTo(filtered,red_hue_image);
    cvtColor(filtered,filtered,CV_HSV2BGR);

    out_msg.image    = filtered;
    out_msg.encoding = "bgr8";
    object_image_pub.publish(out_msg.toImageMsg());

    red_hue_image.copyTo(bw);
    if (gaussian_ksize>0) {
        if (gaussian_ksize % 2 == 0) gaussian_ksize++;
        GaussianBlur( bw, bw, Size(gaussian_ksize,gaussian_ksize), gaussian_sigma , 0);
    }


    if (morph_size>0) {
        Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
        morphologyEx( bw, bw, MORPH_CLOSE, element, Point(-1,-1), 1 );
    }



    out_msg.image    = bw;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    bw_image_pub.publish(out_msg.toImageMsg());


    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(bw, contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    double largest_area=0;
    int largest_contour_index=0;
    for( int i = 0; i< contours.size(); i++ )
    {
        double area0 = abs(contourArea(contours[i]));
        if(area0>largest_area){
            largest_area=area0;
            largest_contour_index=i;
        }
    }
    bool ok=false;
    if ((largest_area>minA)&&(largest_area<maxA)) {

        //
        drawContours(input, contours, (int)largest_contour_index,  Scalar(255,0,0), 3, 8, hierarchy, 0);
        Moments mu=moments( contours[largest_contour_index], true );
        Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
        circle( input, mc, 4, Scalar(0,0,255), -1, 8, 0 );
        int pcl_index = ((int)(mc.y)*input.cols) + (int)(mc.x);
        circle( input, mc, 8, Scalar(0,255,0), -1, 8, 0 );

        pr->x=cloud[pcl_index].x;
        pr->y=cloud[pcl_index].y;
        pr->z=cloud[pcl_index].z;
        char str[100];
        if (isnan (pr->x) || isnan (pr->y) || isnan (pr->z) ) sprintf(str,"NaN");
        else sprintf(str,"[%.3f,%.3f,%.3f] A=%lf",pr->x,pr->y,pr->z,largest_area);
        putText( input, str, mc, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255,255,255), 1, 8);
        ok=true;
    }


    out_msg.image    = input;
    out_msg.encoding = "bgr8";
    result_image_pub.publish(out_msg.toImageMsg());

    return ok;
}

void on_trackbar( int, void* ){

}

move_base_msgs::MoveBaseGoal move_to_button(){

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x=-9.50;
    goal.target_pose.pose.position.y=4.108188;
    goal.target_pose.pose.position.z=0;
    goal.target_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,CV_PI );
    return goal;
}

move_base_msgs::MoveBaseGoal move_to_table(){

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x=-9.0;
    goal.target_pose.pose.position.y=7.0;
    goal.target_pose.pose.position.z=0;
    goal.target_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,CV_PI);
    return goal;
}

move_base_msgs::MoveBaseGoal move_to_goal(){

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x=-3.5;
    goal.target_pose.pose.position.y=-1.0;
    goal.target_pose.pose.position.z=0;
    goal.target_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,1.57);
    return goal;
}

move_base_msgs::MoveBaseGoal move_to_door2(){

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x=-9.0;
    goal.target_pose.pose.position.y=6.0;
    goal.target_pose.pose.position.z=0;
    goal.target_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,-1.57 );
    return goal;
}

geometry_msgs::PoseStamped lift_arm(){
    geometry_msgs::PoseStamped target_pose1;

    target_pose1.header.frame_id="base_link";
    target_pose1.header.stamp=ros::Time::now();
    target_pose1.pose.position.x = 0.4;
    target_pose1.pose.position.y =  0.0;
    target_pose1.pose.position.z =  0.845;
    target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,CV_PI/2.0,0 );
    return target_pose1;
}

move_base_msgs::MoveBaseGoal move_away() {

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x=-5.47;
    goal.target_pose.pose.position.y=6.62;
    goal.target_pose.pose.position.z=0;
    goal.target_pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,CV_PI/2.0 );

    return goal;
}


move_base_msgs::MoveBaseGoal get_pre_pick_pose() {
    tf::Transform dest_transform;
    move_base_msgs::MoveBaseGoal goal;
    try{

        tf::StampedTransform transform_obj,transform_base;
        listener_ptr->lookupTransform("map", "red_object", ros::Time(0), transform_obj);

        listener_ptr->lookupTransform("map", "base_link", ros::Time(0), transform_base);
        tf::Vector3 v_obj =transform_obj.getOrigin();
        tf::Vector3 v_base =transform_base.getOrigin();

        tf::Vector3 v=v_obj-v_base;
        double yaw=atan2(v.y(),v.x());
        double away=distance_from_object/sqrt(v.x()*v.x()+v.y()*v.y());
        tf::Vector3 dest=v_base+v*(1-away);
        dest.setZ(0);
        dest_transform.setOrigin( dest );
        tf::Quaternion q;
        q.setRPY(0.0, 0, yaw);
        dest_transform.setRotation(q);
    }

    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return goal;
    }

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x=dest_transform.getOrigin().x();
    goal.target_pose.pose.position.y=dest_transform.getOrigin().y();
    goal.target_pose.pose.position.z=0;
    goal.target_pose.pose.orientation.x=dest_transform.getRotation().x();
    goal.target_pose.pose.orientation.y=dest_transform.getRotation().y();
    goal.target_pose.pose.orientation.z=dest_transform.getRotation().z();
    goal.target_pose.pose.orientation.w=dest_transform.getRotation().w();
    return goal;
}

void look_down() {
    std_msgs::Float64MultiArray multiArray;
    multiArray.data.push_back(0.0); // pan
    multiArray.data.push_back(0.4); // tilt
    pan_tilt_pub.publish(multiArray);
}

void button_go_cb(std_msgs::Empty) {

    //navigate to the button


    if (base_cmd(move_to_button())) {

        ROS_INFO("Reached button");

        if (base_cmd(move_to_table())) {
            ROS_INFO("Reached table, asking for coke...");

            ros::param::set("/move_base/TrajectoryPlannerROS/max_vel_x", 0.1);

            char sys_cmd[]="espeak -s 150 -v en-uk 'May I have a coke please?'";
            FILE *process=popen(sys_cmd,"r");
            pclose(process);
            ros::Duration w(5);
            w.sleep();
            char sys_cmd1[]="espeak -s 150 -v female3 'Here you go'; rosrun gazebo_ros spawn_model -database coke_can_slim -sdf -model coke_can_slim -y 7.211008 -x -10.722695 -z 0.736 -Y -0.475620";
            FILE *process1=popen(sys_cmd1,"r");
            pclose(process1);
            ros::Duration w1(5);
            w1.sleep(); //wait for detection

            pick_go_cb(std_msgs::Empty());
        }
    }
}

void pick_go_cb(std_msgs::Empty) {

    if (!moving) moving=true;
     ROS_INFO("Looking down...");
 look_down();

 /*char sys_cmd1[]="espeak -s 150 -v female3 'Here you go'; rosrun gazebo_ros spawn_model -database coke_can_slim -sdf -model coke_can_slim -y 7.211008 -x -10.722695 -z 0.736 -Y -0.475620";
    FILE *process1=popen(sys_cmd1,"r");
    pclose(process1)
    ros::Duration w1(5);
    w1.sleep();
;*/
    if (base_cmd(get_pre_pick_pose())) {
        ROS_INFO("Reached pre-picking position, openning gripper...");
        if(gripper_cmd(0.14,0.0)) {
            ROS_INFO("Gripper is oppend, planning for pre-grasping..");
            ros::Duration w2(5);
            w2.sleep(); //wait for re-detection
            if (arm_cmd(move_to_object())) {
                ROS_INFO("Arm planning is done, moving arm..");
                if(moveit_ptr->move()) {
                    ROS_INFO("Ready to grasp");
                    if(gripper_cmd(0.01,0.0)) {
                        ROS_INFO("Grasping is done");
                        ros::Duration w(5);
                        w.sleep(); //wait for attach
                        ROS_INFO("Lifting object...");
                        if (arm_cmd(lift_arm())) {
                            ROS_INFO("Arm planning is done, moving arm..");
                            if (moveit_ptr->move()) {
                                ROS_INFO("Arm is up");
                                if (base_cmd(move_away())) {
                                    ROS_INFO("All done!!!");
                                }

                            }
                        }
                    }
                }
            }
        }
    }
}

geometry_msgs::PoseStamped move_close_to_object(){
    geometry_msgs::PoseStamped target_pose1;

    tf::StampedTransform transform_base_obj = base_obj_transform;

 /*   try{

        listener_ptr->lookupTransform("base_link", "red_object", ros::Time(0), transform_base_obj);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return target_pose1;
    }
    */
    tf::Vector3 v= transform_base_obj.getOrigin();

    double 	yaw=atan2(v.y(),v.x());

    target_pose1.header.frame_id="base_link";
    target_pose1.header.stamp=ros::Time::now();

    /*float away=0.05/sqrt(v.x()*v.x()+v.y()*v.y());
    tf::Vector3 dest=v*(1-away);*/

    target_pose1.pose.position.x =  v.x()+0.15;
    target_pose1.pose.position.y =  v.y();
    target_pose1.pose.position.z =  v.z();
    target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,CV_PI/2.0,0 );

    return target_pose1;

}
geometry_msgs::PoseStamped move_to_object(){
    geometry_msgs::PoseStamped target_pose1;

    tf::StampedTransform transform_base_obj = base_obj_transform;

 /*   try{

        listener_ptr->lookupTransform("base_link", "red_object", ros::Time(0), transform_base_obj);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return target_pose1;
    }
    */
    tf::Vector3 v= transform_base_obj.getOrigin();

    double 	yaw=atan2(v.y(),v.x());

    target_pose1.header.frame_id="base_link";
    target_pose1.header.stamp=ros::Time::now();

    float away=0.05/sqrt(v.x()*v.x()+v.y()*v.y());
    tf::Vector3 dest=v*(1-away);

    target_pose1.pose.position.x =  dest.x();
    target_pose1.pose.position.y =  v.y();
    target_pose1.pose.position.z =  v.z();
    target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(-yaw,CV_PI/2.0,0 );

    return target_pose1;

}

bool arm_cmd( geometry_msgs::PoseStamped target_pose1) {


    moveit_ptr->setStartStateToCurrentState();

    if (!have_goal) have_goal=true;

    moveit::planning_interface::MoveGroup::Plan my_plan;
    double d[]={0, 0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
    double z=target_pose1.pose.position.z;
    for (int i=0;i<sizeof(d)/sizeof(double);i++) {
        target_pose1.pose.position.z=z+d[i];
        goal_pub.publish(target_pose1);
        moveit_goal=target_pose1;
        moveit_ptr->setPoseTarget(target_pose1);
        bool success = moveit_ptr->plan(my_plan);
        ROS_INFO("Moveit plan %s",success?"SUCCESS":"FAILED");
        if (success) return true;
    }
    return false;
}


bool base_cmd(move_base_msgs::MoveBaseGoal goal) {


    ROS_INFO("[%s]: Sending goal", ros::this_node::getName().c_str());

    moveBaseClient_ptr->sendGoal(goal);
    moveBaseClient_ptr->waitForResult();

    if(moveBaseClient_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        return true;
    }
    else {
        ROS_ERROR("[%s]: Navigation failed ", ros::this_node::getName().c_str());
        return false;
    }
}

bool gripper_cmd(double gap,double effort) {

    control_msgs::GripperCommandGoal openGoal;

    openGoal.command.position = gap;
    openGoal.command.max_effort = effort;
    gripperClient_ptr->sendGoal(openGoal);
    ROS_INFO("Sent gripper goal");
    gripperClient_ptr->waitForResult();

    if(gripperClient_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Gripper done");
        return true;
    }
    else {
        ROS_ERROR("Gripper fault");
        // return false;
    }
    return false;
}

// BGUMODO START
void nav_to_cb(const std_msgs::String::ConstPtr& msg) {
    
    ROS_INFO(msg->data.c_str());
    
//    std::string btn("button");
    if (msg->data == "door_zone_1") {
	    ROS_INFO("got button msg");  
        if (base_cmd(move_to_button())) {
    		ROS_INFO("got button msg");
		std_msgs::String status;
		status.data = "Success";
            nav_pub.publish(status);
            

        }
 
   }
	else if (msg->data == "room_4") {
		ROS_INFO("got room4 msg");  
        if (base_cmd(move_to_table())) {
    		ROS_INFO("got room4 msg");		
		std_msgs::String status;
		status.data = "Success";
            nav_pub.publish(status);
            
        }
    }
	else if (msg->data == "room_2") {
		ROS_INFO("got room2 msg");  
        if (base_cmd(move_to_goal())) {
    		ROS_INFO("got room2 msg");
            	std_msgs::String status;
		status.data = "Success";
            nav_pub.publish(status);
        }
    }
	else if (msg->data == "door_zone_2") {
		ROS_INFO("got room2 msg");  
        if (base_cmd(move_to_door2())) {
    		ROS_INFO("got room2 msg");
            	std_msgs::String status;
		status.data = "Success";
            nav_pub.publish(status);
        }
    }
    else {
	std_msgs::String status;
	status.data = "Failed";
	nav_pub.publish(status);
    }
}


void arm_to_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	// ROS_INFO_STREAM("msg");
 //    ROS_INFO_STREAM(msg->pose);
 //    ROS_INFO_STREAM(msg->header); 
 //    geometry_msgs::PoseStamped pose = *msg;
 //    arm_cmd(pose);
 //    moveit_ptr->move();
    bool all_done = false;


      if (!moving) moving=true;
     ROS_INFO("Looking down...");
 look_down();

 /*char sys_cmd1[]="espeak -s 150 -v female3 'Here you go'; rosrun gazebo_ros spawn_model -database coke_can_slim -sdf -model coke_can_slim -y 7.211008 -x -10.722695 -z 0.736 -Y -0.475620";
    FILE *process1=popen(sys_cmd1,"r");
    pclose(process1)
    ros::Duration w1(5);
    w1.sleep();
;*/
    if (base_cmd(get_pre_pick_pose())) {
        ROS_INFO("Reached pre-picking position, openning gripper...");
	if (base_cmd(get_pre_pick_pose())) {
		if(gripper_cmd(0.14,0.0)) {
		    ROS_INFO("Gripper is oppend, planning for pre-grasping..");
		    ros::Duration w2(5);
		    w2.sleep(); //wait for re-detection
		    /*if (arm_cmd(move_close_to_object())) {
		        ROS_INFO("Arm planning is done, moving arm toward the object..");
		        if(moveit_ptr->move()) {
		            ROS_INFO("Ready to grasp");*/
			    if (arm_cmd(move_to_object())) {
				ROS_INFO("Arm planning is done, moving arm..");
				if(moveit_ptr->move()) {
				    ROS_INFO("Starting to grasp");
				    if(gripper_cmd(0.01,0.0)) {
				        ROS_INFO("Grasping is done");
				        ros::Duration w(5);
				        w.sleep(); //wait for attach
				        ROS_INFO("Lifting object...");
				        if (arm_cmd(lift_arm())) {
				            ROS_INFO("Arm planning is done, moving arm..");
				            if (moveit_ptr->move()) {
				                ROS_INFO("Arm is up");
                                //if (all_done = base_cmd(move_away())) {
                                    ROS_INFO("All done!!!");
					std_msgs::String status;
					status.data = "Success";
					arm_pub.publish(status);
                                //}
				//else {
				//	std_msgs::String status;
				//	status.data = "Failed";
       				//	arm_pub.publish(status);				
				//}

                            }
				else {
					std_msgs::String status;
					status.data = "Success";
       					arm_pub.publish(status);
				}
			      }
		 	 //  }
                        }
                    }
                }
            }
        }
    }
/*    if (all_done){
		std_msgs::String status;
		status.data = "Success";
        arm_pub.publish(status);
    }
    else {
		std_msgs::String status;
		status.data = "Failed";
        arm_pub.publish(status);
}*/

}


void pose_red_object_cb(const std_msgs::String::ConstPtr& msg) {
	if (msg->data == "Start") {
		geometry_msgs::PoseStamped object_pose;
		object_pose= move_to_object();
		object_pub.publish(object_pose);
	}
}




int main(int argc, char **argv) {

    ros::init(argc, argv, "demo_pick_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle n;
    ROS_INFO("Hello");
    // n.getParam("distance_from_object", distance_from_object);

    image_transport::ImageTransport it_(n);

    image_sub = it_.subscribe("kinect2/hd/image_color", 1,imageCb);
    result_image_pub = it_.advertise("demo_pick_node/result", 1);
    object_image_pub = it_.advertise("demo_pick_node/hsv_filterd", 1);
    bw_image_pub = it_.advertise("demo_pick_node/bw", 1);

    GripperClient gripperClient("/gripper_controller/gripper_cmd", true);
    //wait for the gripper action server to come up
    while (!gripperClient.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the /gripper_controller/gripper_cmd action server to come up");
    }

    gripperClient_ptr=&gripperClient;

    MoveBaseClient moveBaseClient("move_base", true);
    //wait for the action server to come up
    while(!moveBaseClient.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    moveBaseClient_ptr=&moveBaseClient;

    ROS_INFO("Waiting for the moveit action server to come up");
    moveit::planning_interface::MoveGroup group("arm");

    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    group.allowReplanning(true);
    group.setPlanningTime(8.0);
    group.setNumPlanningAttempts(5);
    group.setPlannerId("RRTConnectkConfigDefault");
    //group.setPlannerId("KPIECEkConfigDefault");
    group.setMaxAccelerationScalingFactor(0.02);
    group.setMaxVelocityScalingFactor(0.02);

    moveit_ptr=&group;
    ros::Subscriber pcl_sub = n.subscribe("kinect2/hd/points", 1, cloud_cb);
    
    ros::Subscriber pick_sub = n.subscribe("pick_go", 1, pick_go_cb);
    ros::Subscriber button_sub = n.subscribe("button_go",1,button_go_cb);

    // bgumodo start here
    ros::Subscriber nav_sub = n.subscribe("nav_to", 1, nav_to_cb);
    ros::Subscriber arm_sub = n.subscribe("arm_to", 1, arm_to_cb);
    ros::Subscriber vision_sub = n.subscribe("pose_red_object", 1, pose_red_object_cb);
    
    
    // end of bgumodo

    goal_pub=n.advertise<geometry_msgs::PoseStamped>("pick_moveit_goal", 2, true);

	object_pub=n.advertise<geometry_msgs::PoseStamped>("pose_red_object_res", 2, true);
    nav_pub=n.advertise<std_msgs::String>("nav_to_res", 2, true);
    arm_pub=n.advertise<std_msgs::String>("arm_to_res", 2, true);

    ros::Publisher wr_pub=n.advertise<geometry_msgs::PoseStamped>("wrist", 2, true);
    pan_tilt_pub = n.advertise<std_msgs::Float64MultiArray>("pan_tilt_controller/command", 10);


    if (debug_vision) {
        namedWindow("Trackbars",CV_WINDOW_AUTOSIZE);              // trackbars window
        createTrackbar( "H min", "Trackbars", &minH, 180, on_trackbar );
        createTrackbar( "H max", "Trackbars", &maxH, 180, on_trackbar );
        createTrackbar( "S min", "Trackbars", &minS, 255, on_trackbar );
        createTrackbar( "S max", "Trackbars", &maxS, 255, on_trackbar );
        createTrackbar( "V min", "Trackbars", &minV, 255, on_trackbar );
        createTrackbar( "V max", "Trackbars", &maxV, 255, on_trackbar );
        createTrackbar( "gaussian_ksize", "Trackbars", &gaussian_ksize, 255, on_trackbar );
        createTrackbar( "gaussian_sigma", "Trackbars", &gaussian_sigma, 255, on_trackbar );
        createTrackbar( "A min", "Trackbars", &minA, 50000, on_trackbar );
        createTrackbar( "A max", "Trackbars", &maxA, 50000, on_trackbar );
        createTrackbar( "morph_size", "Trackbars", &morph_size, 50, on_trackbar );
    }
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    listener_ptr=&listener;

    ros::Rate r(50); // 100 hz

    ROS_INFO("Ready!");

    tf::Quaternion q;
    q.setRPY(0.0, 0, 0);
    kinect_obj_transform.setRotation(q);
    kinect_obj_transform.frame_id_="kinect2_depth_optical_frame";
    kinect_obj_transform.child_frame_id_="red_object";


    while (ros::ok())
    {

        //  if (obj_tf_ok) {

        kinect_obj_transform.stamp_=ros::Time::now();
        br.sendTransform(kinect_obj_transform);
        try{
            listener_ptr->lookupTransform("base_link", "red_object", ros::Time(0), base_obj_transform);
        }
        catch (tf::TransformException ex){
           //  ROS_ERROR("%s",ex.what());
        }
        // }
        if (have_goal) {
            tf::StampedTransform wr_goal;
            tf::Quaternion q;
            tf::quaternionMsgToTF(moveit_goal.pose.orientation,q);
            wr_goal.setOrigin(tf::Vector3(moveit_goal.pose.position.x,moveit_goal.pose.position.y,moveit_goal.pose.position.z));
            wr_goal.setRotation((q));
            wr_goal.stamp_=ros::Time::now();
            wr_goal.child_frame_id_="moveit_goal";
            wr_goal.frame_id_="base_link";
            br.sendTransform(wr_goal);

        }


        tf::StampedTransform wr;
        try{
            listener_ptr->lookupTransform("base_link", "wrist_link", ros::Time(0), wr);
            double roll,pitch,yaw;
            tf::Quaternion quat=wr.getRotation();
            tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);

            geometry_msgs::PoseStamped target_pose2;
            target_pose2.header.frame_id="base_link";
            target_pose2.header.stamp=ros::Time::now();
            target_pose2.pose.position.x = wr.getOrigin().x();
            target_pose2.pose.position.y = wr.getOrigin().y();
            target_pose2.pose.position.z = wr.getOrigin().z();
            tf::quaternionTFToMsg( wr.getRotation(), target_pose2.pose.orientation);
            wr_pub.publish(target_pose2);

        }
        catch (tf::TransformException ex){
            // ROS_ERROR("%s",ex.what());
        }

        r.sleep();
    }

    return 0;
}

