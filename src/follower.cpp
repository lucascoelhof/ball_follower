#include <cstdio>
#include <stdlib.h>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmvision/Blobs.h>
#include <cmvision/Blob.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

class orangeBallTracker
{
  private:
    geometry_msgs::Twist trackedObject;
    geometry_msgs::Twist imageCenter;
    
    geometry_msgs::Twist speed;
    
    sensor_msgs::PointCloud2 depth;
    pcl::PointCloud < pcl::PointXYZ > pcl_cloud;

    tf::Transform orangeBallTF;

    tf::TransformListener listenerTF;
    tf::TransformBroadcaster br;

    ros::Publisher velPub;

    ros::NodeHandle *n;
    
    int thresholdArea;

    double min_z; 
    double max_z;
    double desired_z;

    double tol_z, tol_ang;

    double pGain;
    double pGainRot;

    double maxSpeed;
    double maxRot;

    double zError;
    double pError, iError, dError;

    double angError;
    double maxAngError;
    double pAngError, iAngError, dAngError;

    std::string cameralink, objectName;

  public:

    int squareMod(unsigned int left, unsigned int right, unsigned int top, unsigned int bottom)
    {
      int width, height;
      width =  abs(right - left);
      height = abs(bottom - top);

      return abs(width - height);
    }

    void blobCallback(const cmvision::Blobs::ConstPtr& msg)
    {
      cmvision::Blob orangeBall;
      cmvision::Blob blob;

      /*** A blob containing a orange ball needs to be a big blob        ***/
      /*** A blob containing a orange ball is aproximately square shaped ***/

      n->param("/follower/thresholdArea", thresholdArea, thresholdArea);

      if(msg->blob_count==0)
      {
        ROS_INFO("No blobs detected.");
        speed.linear.x = 0;
        speed.angular.z = 0;
        velPub.publish(speed);
        return;
      }

      blob = msg->blobs[0];

      if(blob.area < thresholdArea)
      {
        ROS_INFO("Detected blobs are too small. Area = %u\n", blob.area);
        speed.linear.x = 0;
        speed.angular.z = 0;
        velPub.publish(speed);
        return; 
      }
      orangeBall = blob;

      //printf("x: %u, y: %u, area: %u\n", blob.x, blob.y, blob.area);

      trackedObject.linear = getPose(blob.x, blob.y);
      /*printf("Position of the tracked object >>> x: %f; y: %f; z: %f\n", 
                trackedObject.linear.x, trackedObject.linear.y, trackedObject.linear.z);*/
      tfPublisher();
      speedPublisher();
    }

    geometry_msgs::Vector3 getPose(int x, int y)
    {
      geometry_msgs::Vector3 v;
      pcl::PointXYZ point = pcl_cloud.at(x, y);
      v.x = point.x;
      v.y = point.y;
      v.z = point.z;
      return v;
    }

    double getX(int x, int y)
    {
      return pcl_cloud.at(x, y).x;
    }

    double getY(int x, int y)
    {
      return pcl_cloud.at(x, y).y;
    }

    void tfPublisher()
    {
      orangeBallTF.setOrigin( tf::Vector3(trackedObject.linear.z, -trackedObject.linear.x, -trackedObject.linear.y));
      orangeBallTF.setRotation( tf::Quaternion(0, 0, 0) );

      n->param<std::string>("camera_link", cameralink, "camera_link");
      n->param<std::string>("object_name", objectName, "orange_ball");
      br.sendTransform(tf::StampedTransform(orangeBallTF, ros::Time::now(), cameralink, objectName));
    }

    void kinectDepthCallback(const sensor_msgs::PointCloud2::ConstPtr msg)
    {
      depth = *msg; 

      //std::cout<<msg->height<<" ";
      pcl::PCLPointCloud2 pcl_pc2;
      //pcl_conversions::toPCL((sensor_msgs::PointCloud2&)msg, pcl_pc2);
      pcl_conversions::toPCL(*msg, pcl_pc2);
       //Actually convert the PointCloud2 message into a type we can reason about
      pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);

      imageCenter.linear.x = depth.width/2;
      imageCenter.linear.y = depth.height/2;
    }

    void speedPublisher()
    {
      static double iError = 0;
      static double iErrorRot = 0;

      double pError=0, dError=0, iErrorIncrement=0;
      double pErrorRot=0, dErrorRot=0, iErrorIncrementRot=0;

      static bool isSpeedSaturated = 0;
      static bool isRotSaturated = 0;

      double speedPID = 0;
      ros::Duration delTime;
      static ros::Time lastCall = ros::Time::now();

      double zError=0;

      ros::Time timeNow = ros::Time::now();

      delTime = timeNow - lastCall;
      
      n->param<double>("/follower/min_z", min_z, min_z);
      n->param<double>("/follower/max_z", max_z, max_z);
      n->param<double>("/follower/desired_z", desired_z, desired_z);
      n->param<double>("/follower/tol_z", tol_z, tol_z);
      n->param<double>("/follower/tol_ang", tol_ang, tol_ang);
      
      n->param<double>("/follower/pGain", pGain, pGain);
      n->param<double>("/follower/pGainRot", pGainRot, pGainRot);

      n->param<double>("/follower/maxSpeed", maxSpeed, maxSpeed);
      n->param<double>("/follower/maxRot", maxRot, maxRot);

      n->param<double>("/follower/maxAngError", maxAngError, maxAngError); 


      if (trackedObject.linear.z > max_z || trackedObject.linear.z < min_z)
      {
        speed.linear.x = 0;
        speed.angular.z = 0;
      }
      else
      {
        /*
        tf::StampedTransform transform;
        try{
          listenerTF.lookupTransform(objectName, cameralink,
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }
        */
        zError =  trackedObject.linear.z - desired_z;

        /*zError = sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));*/
        pError = zError * pGain;
        
        speedPID = pError;

        if(fabs(zError) < tol_z)
          speed.linear.x=0;
        else
          speed.linear.x = speedPID;

        if(fabs(speed.linear.x) > maxSpeed)
          speed.linear.x=maxSpeed*fabs(speed.linear.x)/speed.linear.x;


        //angError =  (320 -  trackedObject.linear.x)/ 640;
        angError = -atan2(trackedObject.linear.x, trackedObject.linear.y);

        /*angError = sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));*/
        if(fabs(angError) < tol_ang)
          speed.angular.z = 0;
        else
          speed.angular.z = angError*pGainRot;

        /*if(fabs(angError) > maxAngError)
          speed.linear.x = 0;*/


        printf("zError: %f angError: %f speed: %f rot: %f abs(zError): %f\n", 
                zError, angError, speed.linear.x, speed.angular.z, fabs(zError));
        

        /*if(fabs(angError) > maxAngError)
          speed.linear.x=0;
        */
        /* PID Controller
        pError = zError * pGain;
        iErrorIncrement = zError * iGain * delTime.toSec();
        //iError=0;
        
        if(!isSpeedSaturated)
        {
          iError += iErrorIncrement;
          if(iError>1)
            iError=1;
          else if (iError<-1)
            iError=-1;
        }
        //dError = dGain *zError / delTime.toSec();
        dError =0;
        speed.linear.x = pError + iError + dError;

        printf("--- Linear --- \ndt: %f\t\tP: %f\tI: %f\tD: %f\t \nError: %f\tP: %f\tI: %f\tD: %f\tSpeed: %f\n", 
                    delTime.toSec(), pGain, iGain, dGain, zError, pError, iError, dError, speed.linear.x);

        if(fabs(speed.linear.x) > maxSpeed)
        { 
          speed.linear.x =  maxSpeed * fabs(speed.linear.x)/speed.linear.x;
          isSpeedSaturated = 1;
        }
        else
          isSpeedSaturated = 0; 

        angError =  (320 -  trackedObject.linear.x)/ 640;
        if( fabs(angError) > maxAngError )
        {
          speed.linear.x = 0;
          isSpeedSaturated = 1;
        }

        pErrorRot = angError * pGainRot;
        iErrorIncrementRot = angError * iGainRot * delTime.toSec();
        //iErrorRot=0;
        if(!isRotSaturated)
        {
          iErrorRot += iErrorIncrementRot;
          if(iErrorRot>1)
            iErrorRot=1;
          else if (iErrorRot<-1)
            iErrorRot=-1;
        }
        //iErrorRot=0;
        //dErrorRot = angError * dGainRot / delTime.toSec();
        dErrorRot=0;
        speed.angular.z = pErrorRot + iErrorRot + dErrorRot;

        printf("Angular - Error: %f; P: %f; I: %f; D: %f; Speed: %f\n\n", 
          angError, pErrorRot, iErrorRot, dErrorRot, speed.angular.z);

        if( fabs(speed.angular.z) > maxRot )
        {  
          speed.angular.z = maxRot * fabs(speed.angular.z)/speed.angular.z;
          isRotSaturated = 1;
        }
        else
        {
          isRotSaturated=0;
        }
        */
      }
      velPub.publish(speed);
      lastCall = ros::Time::now();

    }

    void setPublisher(ros::NodeHandle* n, std::string topic, int buffersize)
    {
      velPub = n->advertise<geometry_msgs::Twist>(topic, buffersize);
    }

    void setNodeHandle(ros::NodeHandle* nh)
    {
      n = nh;
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n; 
  orangeBallTracker ballTracker;
  //ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, chatterCallback);
  ballTracker.setNodeHandle(&n);

  std::string velTopic, depthTopic, blobsTopic;

  n.param<std::string>("velTopic", velTopic, "/cmd_vel_mux/input/navi");
  n.param<std::string>("depthTopic", depthTopic, "/camera/depth/points");
  n.param<std::string>("blobsTopic", blobsTopic, "/blobs");

  ballTracker.setPublisher(&n, velTopic, 100);
  ros::Subscriber kinectsub = n.subscribe(depthTopic, 1000, &orangeBallTracker::kinectDepthCallback, &ballTracker);
  ros::Subscriber blobSub = n.subscribe(blobsTopic, 100, &orangeBallTracker::blobCallback, &ballTracker);

  ros::spin();
}
