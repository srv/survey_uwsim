#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

class SurveyMaker{
public:

	enum State // not implemented yet
	{
		ACCELERATING,
		CONSTANT_SPEED,
		DEACCELERATING
	};

	SurveyMaker(ros::NodeHandle nh, ros::NodeHandle nhp);

	int update(void);

	double freq;

private:
	double x_;
	double y_;
	double z_;
	double yaw_;
	double long_x_;
	double long_y_;
	double linear_acceleration_; // not implemented yet
	double linear_speed_; // not implemented yet
	double actual_linear_speed_; // not implemented yet
	double angular_acceleration_; // not implemented yet
	double angular_speed_; // not implemented yet
	double actual_angular_speed_; // not implemented yet
	double acc_distance_; // not implemented yet
	int nx_;
	int ny_;
	bool clockwise_turns_;
	bool positive_direction_;
	bool first_run_;
	std::string pose_topic_;
	std::string control_topic_;

	int forward_movement_state_; // not implemented yet
	int rotation_state_; // not implemented yet

	std::vector<tf::Transform> trajectory_;
	tf::Transform current_pose_;
	tf::Transform vehicle_pose_;
	size_t trajectory_index_;
	bool waypoint_achieved_;

	ros::Time previous_update_time_; // not implemented yet

	ros::NodeHandle nh_;
	ros::NodeHandle nhp_;
	ros::Publisher position_pub_;
	ros::Subscriber position_sub_;

	void generateWaypoints(void);

	void vehiclePoseCallback(const nav_msgs::Odometry& odom);

};

SurveyMaker::SurveyMaker(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp){
	nhp_.getParam("x",x_);
  nhp_.getParam("y",y_);
  nhp_.getParam("z",z_);
  nhp_.getParam("yaw",yaw_);
  nhp_.getParam("long_x",long_x_);
  nhp_.getParam("long_y",long_y_);
  nhp_.getParam("nx",nx_);
  nhp_.getParam("ny",ny_);

  nhp_.getParam("freq",freq);

  nhp_.param("clockwise_turns",clockwise_turns_,false);
  nhp_.param("positive_direction",positive_direction_,true);

  nhp_.param("linear_acceleration",linear_acceleration_,1.0);
  nhp_.param("linear_speed",linear_speed_,1.0);
  nhp_.param("angular_acceleration",angular_acceleration_,0.1);
  nhp_.param("angular_speed",angular_speed_,0.1);

  pose_topic_ = "pose_topic";
  control_topic_ = "control_topic";

  ROS_INFO_STREAM("[SurveyMaker]: Instantiating a SurveyMaker with params \n" << 
  	"\t* x:\t\t\t" 								<< x_ << "\n" <<
  	"\t* y:\t\t\t" 								<< y_ << "\n" <<
  	"\t* z:\t\t\t" 								<< z_ << "\n" <<
  	"\t* yaw:\t\t\t" 							<< yaw_ << "\n" <<
  	"\t* long_x:\t\t" 						<< long_x_ << "\n" <<
  	"\t* long_y:\t\t" 						<< long_y_ << "\n" <<
  	"\t* nx:\t\t\t" 							<< nx_ << "\n" <<
  	"\t* ny:\t\t\t" 							<< ny_ << "\n" <<
  	"\t* freq:\t\t\t" 					  << freq << "\n" <<
  	"\t* clockwise_turns:\t" 			<< clockwise_turns_ << "\n" <<
  	"\t* positive_direction:\t" 	<< positive_direction_ << "\n" <<
  	"\t* linear_acceleration:\t" 	<< linear_acceleration_ << "\n" <<
  	"\t* linear_speed:\t\t" 			<< linear_speed_ << "\n" <<
  	"\t* angular_acceleration:\t" << angular_acceleration_ << "\n" <<
  	"\t* angular_speed:\t" 				<< angular_speed_ << "\n" );

  position_pub_ = nh_.advertise<nav_msgs::Odometry>(control_topic_, 1);
  position_sub_ = nh_.subscribe(pose_topic_, 1, &SurveyMaker::vehiclePoseCallback, this);

  generateWaypoints();

  trajectory_index_ = 0;
  waypoint_achieved_ = false;

  first_run_ = true;

  forward_movement_state_ = ACCELERATING;
  rotation_state_ = ACCELERATING;
  actual_linear_speed_ = 0;
  actual_angular_speed_ = 0;
}

int SurveyMaker::update(void){

	if(first_run_){
		ROS_INFO_STREAM_ONCE("[SurveyMaker]: Waiting for topics to be published.");
		return 0;
	}

	tf::Vector3 current_position = current_pose_.getOrigin();
	tf::Quaternion current_rotation = current_pose_.getRotation();

	if (trajectory_index_ == trajectory_.size()){
		ROS_INFO_STREAM("[SurveyMaker]: Survey finished!");
		nhp_.shutdown();
		nh_.shutdown();
		return 1;
	}

	tf::Transform waypoint = trajectory_[trajectory_index_];
	tf::Vector3 waypoint_position = waypoint.getOrigin();
	tf::Quaternion waypoint_rotation = waypoint.getRotation();

	tf::Vector3 vT = (waypoint_position-current_position)*0.15;
	double vScale = (vT.length()>0.1) ? 0.1/vT.length() : 1.0;

	double yaw  = tf::getYaw(current_rotation);
	double waypoint_yaw  = tf::getYaw(waypoint_rotation);
	double diff_yaw = current_rotation.angleShortestPath(waypoint_rotation); //TODO solve bad turn directions
	int rotation_dir = waypoint_yaw - yaw > 0 ? 1 : -1;

	double yawT = diff_yaw * 0.07;
  double yawScale = (fabs(yawT) > 0.1) ? fabs(0.005 / yawT) : 0.05;

	double x = current_position.x() + vT.x()*vScale;
	double y = current_position.y() + vT.y()*vScale;
	double z = current_position.z() + vT.z()*vScale;

	yaw += rotation_dir * yawT * yawScale;

	current_position = tf::Vector3(x,y,z);
	current_rotation.setRPY(0,0, yaw);

	ROS_DEBUG_STREAM("[SurveyMaker]: Pose (x,y,z,yaw) = (\t" << x << ",\t" << y << ",\t" << z << ",\t" << yaw << ")");
	ROS_DEBUG_STREAM("[SurveyMaker]: Wayp (x,y,z,yaw) = (\t" << waypoint_position.x() << ",\t" << waypoint_position.y() << ",\t" << waypoint_position.z() << ",\t" << waypoint_yaw << ")");

	// update our position
	//current_pose_ = tf::Transform(current_rotation, current_position);

	// Publish odometry message
	nav_msgs::Odometry odom;
  odom.pose.pose.position.x = current_position.x();
  odom.pose.pose.position.y = current_position.y();
  odom.pose.pose.position.z = current_position.z();
  odom.pose.pose.orientation.x = current_rotation.x();
  odom.pose.pose.orientation.y = current_rotation.y();
  odom.pose.pose.orientation.z = current_rotation.z();
  odom.pose.pose.orientation.w = current_rotation.w();
  position_pub_.publish(odom);

  return 0;
}

void SurveyMaker::vehiclePoseCallback(const nav_msgs::Odometry& odom) {
	tf::Vector3 v(odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z);
	tf::Quaternion q(odom.pose.pose.orientation.x,
	                 odom.pose.pose.orientation.y,
	                 odom.pose.pose.orientation.z,
	                 odom.pose.pose.orientation.w);
	vehicle_pose_ = tf::Transform(q,v);

	if(first_run_){
		first_run_ = false;
		
		previous_update_time_ = ros::Time::now();

	  nav_msgs::Odometry init_pose;
	  tf::Vector3 v_init(x_,y_,z_);
	  tf::Quaternion q_init;
	  q_init.setRPY(0,0, yaw_);

	  init_pose.pose.pose.position.x = v_init.x();
	  init_pose.pose.pose.position.y = v_init.y();
	  init_pose.pose.pose.position.z = v_init.z();
	  init_pose.pose.pose.orientation.x = q_init.x();
	  init_pose.pose.pose.orientation.y = q_init.y();
	  init_pose.pose.pose.orientation.z = q_init.z();
	  init_pose.pose.pose.orientation.w = q_init.w();

	  position_pub_.publish(init_pose);

	  current_pose_ = tf::Transform(q_init,v_init);

		ROS_INFO_STREAM_ONCE("[SurveyMaker]: Suscription working.");
	}

	current_pose_ = vehicle_pose_;

	update();

	tf::Transform waypoint = trajectory_[trajectory_index_];
	tf::Vector3 waypoint_position = waypoint.getOrigin();
	tf::Quaternion waypoint_rotation = waypoint.getRotation();

	double pos_distance = tf::tfDistance(waypoint_position,v);
	double ang_distance = std::abs(q.angleShortestPath(waypoint_rotation));
	double eps = 0.001;
	double ang_eps = 0.01;

	if( (pos_distance < eps) && (ang_distance < ang_eps) ){
		waypoint_achieved_ = true;
	//}
	//if(waypoint_achieved_){
		ROS_INFO_STREAM("[SurveyMaker]: Waypoint " << trajectory_index_ << " achieved.");
		trajectory_index_ += 1;
		waypoint_achieved_ = false;
	}
}

void SurveyMaker::generateWaypoints(void){
	double step_x = long_x_ / nx_;
  double step_y = long_y_ / ny_;

  int x_turn_dir = clockwise_turns_ ? 1 : -1;
  int x_move_dir = positive_direction_ ? 1 : -1;
	int y_turn_dir = clockwise_turns_ ? -1 : 1; // NOT(clockwise_turns_)
  int y_move_dir = (clockwise_turns_ ^ positive_direction_) ? 1 : -1; // XOR(clockwise_turns_, positive_direction_)
  int y_steps_dir = (clockwise_turns_ ^ positive_direction_) ? -1 : 1; // NOT(XOR(clockwise_turns_, positive_direction_))
  int x_steps_dir = positive_direction_ ? -1 : 1; // NOT(positive_direction_)

  tf::Transform t;
  tf::Quaternion q;
  tf::Quaternion qaux;

  // first point of the trajectory is the original point provided by the user
  t.setOrigin(tf::Vector3(x_,y_,z_));
  q.setRPY(0, 0, yaw_);
  t.setRotation(q.normalized());
  trajectory_.push_back(t);

  // second point is the first translation in x
  t.setOrigin( t.getOrigin() + tf::Vector3(x_move_dir * long_x_, 0, 0));
  trajectory_.push_back(t);

  // loop the new positions in zigzag
  for (int i = 0; i < ny_; i++){

  	// 3. turn 90 degrees 
  	qaux.setRPY(0, 0, x_turn_dir * M_PI_2);
  	q = q * qaux;
  	t.setRotation(q.normalized());
  	trajectory_.push_back(t);

  	// 4. move in Y
  	t.setOrigin( t.getOrigin() + tf::Vector3(0, y_steps_dir * step_y, 0));
  	trajectory_.push_back(t);

  	// 5. turn 90 degrees again
  	qaux.setRPY(0, 0, x_turn_dir * M_PI_2);
  	q = q * qaux;
  	t.setRotation(q.normalized());
  	trajectory_.push_back(t);

  	// 6. return in X
  	t.setOrigin( t.getOrigin() + tf::Vector3( - x_move_dir * long_x_, 0, 0));
  	trajectory_.push_back(t);

  	// change the turn direction
  	x_turn_dir *= -1; 
  	x_move_dir *= -1;
  }

  // turn 90 degrees 
	qaux.setRPY(0,0,y_turn_dir * M_PI_2);
	q = q * qaux;
	t.setRotation(q.normalized());
	trajectory_.push_back(t);

	// move in Y all the way down
	t.setOrigin( t.getOrigin() + tf::Vector3(0, y_move_dir * long_y_,0));
  trajectory_.push_back(t);

  for (int i = 0; i < nx_; i++){
  	// turn 90 degrees 
  	qaux.setRPY(0,0,y_turn_dir * M_PI_2);
  	q = q * qaux;
  	t.setRotation(q.normalized());
  	trajectory_.push_back(t);

  	// move in X
  	t.setOrigin( t.getOrigin() + tf::Vector3(x_steps_dir * step_x,0,0));
  	trajectory_.push_back(t);

  	// turn 90 degrees again
  	qaux.setRPY(0,0,y_turn_dir * M_PI_2);
  	q = q * qaux;
  	t.setRotation(q.normalized());
  	trajectory_.push_back(t);

  	// return in Y
  	t.setOrigin( t.getOrigin() + tf::Vector3(0, - y_move_dir * long_y_,0));
  	trajectory_.push_back(t);

  	// change the turn direction
  	y_turn_dir *= -1; 
  	y_move_dir *= -1;
  }

  ROS_INFO_STREAM("[SurveyMaker]: Trajectory generated with " << trajectory_.size() << " waypoints.");
  std::cout << "\t    Waypoint \t X\t Y\tZ\t  YAW" << std::endl;
  for(size_t i = 0; i < trajectory_.size(); i++){
  	std::cout << "\t\t" << i+1 << "\t" <<
  	trajectory_[i].getOrigin().x() << "\t" <<
  	trajectory_[i].getOrigin().y() << "\t" <<
  	trajectory_[i].getOrigin().z() << "\t" <<
  	tf::getYaw(trajectory_[i].getRotation()) << std::endl;
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "make_survey");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  SurveyMaker sm(nh,nhp);

  ros::Rate r(sm.freq);
  while (ros::ok()) {
  	//if(sm.update())
  	//	break;
  	ros::spinOnce();
    r.sleep();
  }
  return 0;
}