/* Move a vehicle with a velocity proportional to the distance 
 * between the current pose and a desired absolute pose 
*/

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ros/ros.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <osg/Quat>
#include <osg/Vec3d>
#include <osg/Matrix>
#include <iostream>

#define VRATE 0.15
#define YAWRATE 0.05

bool firstpass=false;
osg::Quat initialQ, goalQ, currentQ;
osg::Vec3d initialT, goalT, currentT;
double totalDistance, currentDistance;
double initialYaw, currentYaw, goalYaw, currentAngleDiff, totalAngleDiff;

struct _position{
  double x;
  double y;
  double z;
  
  double roll,pitch,yaw;
};

double quat2Yaw(double z, double w) {
  return angles::normalize_angle(atan2(z,w)*2.0);
}

void getWayPoints(double initX, double initY, double initZ, double longX, double longY, int nX, int nY, std::vector<_position> &points) {
  double stepX = longX/nX;
  double stepY = longY/nY;
  double dir;
  int i;
  struct _position auxPoint;
  
  //The first position is the init
  auxPoint.x = initX;
  auxPoint.y = initY;
  auxPoint.z = initZ;
  auxPoint.roll = auxPoint.pitch = auxPoint.yaw =0.0;
  points.push_back(auxPoint);
  //Next position
  auxPoint.x = initX + longX;
  auxPoint.y = initY;
  
  points.push_back(auxPoint);

  dir = (-1.0);
  for (i=0; i<nY; i++) {
     auxPoint.yaw = auxPoint.yaw - dir * M_PI_2;
     points.push_back(auxPoint);
    
     auxPoint.y = auxPoint.y + stepY;
     points.push_back(auxPoint);
     
     auxPoint.yaw = auxPoint.yaw - dir * M_PI_2;
     points.push_back(auxPoint);
     
     auxPoint.x = auxPoint.x + (dir * longX);
     points.push_back(auxPoint);
     dir = dir * (-1.0);
  }
  
    //Next position

  auxPoint.yaw = auxPoint.yaw - M_PI_2;
  points.push_back(auxPoint);

  auxPoint.y = auxPoint.y - longY;
  points.push_back(auxPoint);
  
  dir = (1.0);
  for (i=0; i<nY; i++) {
     auxPoint.yaw = auxPoint.yaw - dir * M_PI_2;
     points.push_back(auxPoint);
    
     auxPoint.x = auxPoint.x - stepX;
     points.push_back(auxPoint);
     
     auxPoint.yaw = auxPoint.yaw - dir * M_PI_2;
     points.push_back(auxPoint);
     
     auxPoint.y = auxPoint.y + (dir * longY);
     points.push_back(auxPoint);
     dir = dir * (-1.0);
  }
}

void vehiclePoseCallback(const nav_msgs::Odometry& odom) {
	if (!firstpass) {
	        
		initialT.set(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
		initialQ.set(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
		totalDistance=(goalT-initialT).length();
		currentYaw = initialYaw = quat2Yaw(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
		std::cout << " Callback: currentYaw=" << currentYaw << " initialYaw=" << initialYaw << " odomPose=" << odom.pose.pose.orientation.z << std::endl;
		totalAngleDiff = angles::shortest_angular_distance(initialYaw, goalYaw);
		
		currentT.set(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
	        currentDistance=(goalT-currentT).length();
		currentAngleDiff = angles::shortest_angular_distance(goalYaw, currentYaw);
		firstpass=true;
	} else {
	  currentYaw = quat2Yaw(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
	  currentAngleDiff = angles::shortest_angular_distance(currentYaw, goalYaw);
	  std::cout << " Callback2: currentYaw=" << currentYaw << " initialYaw=" << initialYaw << " odomPose=" << odom.pose.pose.orientation.z << std::endl;
	  currentT.set(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
	  currentDistance=(goalT-currentT).length();
	 
	}
}

int main(int argc, char **argv) {


	if (argc!=13) {
		std::cerr << "USAGE: " << argv[0] << " <vehiclePoseTopic> <vehicleControlTopic> <x> <y> <z> <roll> <pitch> <yaw> <longX> <longY> <nx> <ny>" << std::endl;
		std::cerr << "units are meters and radians." << std::endl;
		return 0;
	}	

	std::string poseTopic(argv[1]);
	std::string controlTopic(argv[2]);
	double x=atof(argv[3]);
	double y=atof(argv[4]);
	double z=atof(argv[5]);
	double roll=atof(argv[6]);
	double pitch=atof(argv[7]);
	double yaw=atof(argv[8]);
	double longX=atof(argv[9]);
	double longY=atof(argv[10]);
	int nx=atof(argv[11]);
	int ny=atof(argv[12]);
	
	osg::Vec3d myX;
	double a,b;
	
	std::vector<_position> trajectory;
	
	std::string nodeName="trajectory_node";
	ros::init(argc, argv, nodeName);
	ros::NodeHandle nh;

	osg::Matrixd T, Rx, Ry, Rz, transform;
	
        //getWaitPoints(x, y, z, 5.5, 3.0, 4, 4, trajectory);
	
	getWayPoints(x, y, z, longX, longY, ny, nx, trajectory);
	
	/*for (int i=0;i<trajectory.size();i++) {
	  std::cout << i << " "<< trajectory[i].x << ", " << trajectory[i].y << ", " << trajectory[i].z << std::endl;
	}*/
	
	ros::Publisher position_pub=nh.advertise<nav_msgs::Odometry>(controlTopic,1);
	ros::Subscriber position_sub = nh.subscribe(poseTopic, 1, vehiclePoseCallback);

        ros::Rate r(30);
	
        while (ros::ok()) {
	for (int i=0;i<trajectory.size();i++) {
	  std::cout << i << " "<< trajectory[i].x << ", " << trajectory[i].y << ", " << trajectory[i].z << " Dist=" << currentDistance << std::endl;
	  T.makeTranslate(trajectory[i].x,trajectory[i].y,trajectory[i].z);
	  Rx.makeRotate(trajectory[i].roll,1,0,0);
	  Ry.makeRotate(trajectory[i].pitch,0,1,0);
	  Rz.makeRotate(trajectory[i].yaw,0,0,1);
	  transform=Rz*Ry*Rx*T;
	  goalT=transform.getTrans();
	  goalQ=transform.getRotate();
	  goalYaw = angles::normalize_angle (trajectory[i].yaw);
          firstpass = false;
	  
	  
	  osg::Quat quatAux; ///Used as an auxiliar variable for getting the vehicle orientation
	  while (ros::ok() && ((!firstpass) || (currentDistance >= 0.05) || (fabs(currentAngleDiff) >=0.05))) {
	    if (firstpass) {
		  osg::Vec3d vT=(goalT-currentT);//*0.15;
		  
		  //double vScale=(vT.length()>0.1) ? 0.1/vT.length() : 1;
		  double vScale = 0.0;
                  if (totalDistance >= 0.05) {
		    b = (4.0*0.05)/(totalDistance);
		    a = (-b/totalDistance);
		    vScale = a * vT.length() * vT.length() + b * vT.length() + 0.03;
		  }
		  std::cout << "VScale=" << vScale << " vT.length()=" << vT.length() << " totalDistance=" << totalDistance << std::endl;
		  //if (vScale <= 0.02) vScale = 0.02;
		  
		  //End new
		  
		  
		  double yawT = angles::shortest_angular_distance(currentYaw, goalYaw)*0.07;
		  double yawScale = (fabs(yawT)>0.1)? fabs(YAWRATE/yawT) : 1.0;
		  
		  //std::cout << " currentYaw="<< currentYaw << " goalYaw=" << goalYaw << " currentAngleDiff=" << currentAngleDiff << " totalAngleDiff=" << totalAngleDiff << std::endl;
		  //std::cout << " yawT=" <<  yawT << "" << std::endl;
		  //currentQ.slerp(1-currentDistance/totalDistance,initialQ, goalQ);
		  /*if (fabs(totalAngleDiff)>=001)
		    currentQ.slerp(1.0-fabs(currentAngleDiff/totalAngleDiff),initialQ, goalQ);
		  else
		    currentQ.slerp(0.0,initialQ, goalQ);*/
		  
		  T.makeTranslate(currentT.x(),currentT.y(),currentT.z());
		  //Rz.makeRotate(currentYaw + yawT * yawScale,0,0,1)*Ry*Rx*T).getRotate();
		  Rz.makeRotate(currentYaw + yawT * yawScale,0,0,1);
		  transform = Rz*Ry*Rx*T;
		  currentQ = transform.getRotate();
		  
		  nav_msgs::Odometry odom;
		  /*odom.pose.pose.position.x=currentT.x()+vT.x()*vScale;
		  odom.pose.pose.position.y=currentT.y()+vT.y()*vScale;
		  odom.pose.pose.position.z=currentT.z()+vT.z()*vScale;*/
		  vScale = vScale * VRATE;
		  if (totalDistance > 0.05) {
		    odom.pose.pose.position.x=currentT.x()+ vScale*(goalT.x()-initialT.x())/totalDistance;
		    odom.pose.pose.position.y=currentT.y()+vScale*(goalT.y()-initialT.y())/totalDistance;
		    odom.pose.pose.position.z=currentT.z()+vScale*(goalT.z()-initialT.z())/totalDistance;
		  } else {
		    odom.pose.pose.position.x=currentT.x();
		    odom.pose.pose.position.y=currentT.y();
		    odom.pose.pose.position.z=currentT.z();
		  }
		  
		  odom.pose.pose.orientation.x=currentQ.x();
		  odom.pose.pose.orientation.y=currentQ.y();
		  odom.pose.pose.orientation.z=currentQ.z();
		  odom.pose.pose.orientation.w=currentQ.w();

		  odom.twist.twist.linear.x=0;
		  odom.twist.twist.linear.y=0;
		  odom.twist.twist.linear.z=0;
		  odom.twist.twist.angular.x=0;
		  odom.twist.twist.angular.y=0;
		  odom.twist.twist.angular.z=0;
		  for (int i=0; i<36; i++) {
			  odom.twist.covariance[i]=0;
			  odom.pose.covariance[i]=0;
		  }
		  position_pub.publish(odom);
	    }
	    ros::spinOnce();
	    r.sleep();
	  }
	}
	  //if (currentDistance <= 0.01) break;
	}

	return 0;
}
