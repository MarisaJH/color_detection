#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"
#include "color_detection/pixelTo3dPoint.h"
#include "color_detection/points.h"
#include <vector>
#include <iostream>

geometry_msgs::Point pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v);

sensor_msgs::PointCloud2ConstPtr POINTCLOUD = boost::shared_ptr<const sensor_msgs::PointCloud2_<std::allocator<void> > >(); //nullptr;

void callback(const sensor_msgs::PointCloud2ConstPtr& pc) {
  //std::cout << "In pointcloud callback" << std::endl;
  POINTCLOUD = pc;
  //std::cout << "Pointcloud received" << std::endl;
}

bool getPixels(color_detection::pixelTo3dPoint::Request  &req,
               color_detection::pixelTo3dPoint::Response &res) {

  std::vector<geometry_msgs::Point> points3d;

  if (POINTCLOUD == boost::shared_ptr<const sensor_msgs::PointCloud2_<std::allocator<void> > >()) {  //nullptr) { compiler says this is wrong on turtlebot?
    //std::cout << "pointcloud is nullptr" << std::endl;

    geometry_msgs::Point point;
    point.x = -99.0; point.y = -99.0; point.z = -99.0;
    points3d.push_back(point);
    res.conePos = points3d;
    return true;
  }

  //const sensor_msgs::PointCloud2 pc2 = *POINTCLOUD;
  //loop through request and get all points for one segment, then avg and add to vector
  for (int i = 0; i < req.pixels.size(); i++) {
    //std::cout << "In first for loop" << std::endl;
    double xSum = 0;
    double ySum = 0;
    double zSum = 0;
    int count = 0;

    for (int j = 0; j < req.pixels.at(i).points.size(); j+= 2) {
      //std::cout << "Inner loop" << std::endl;

      int v = req.pixels.at(i).points.at(j);
      int u = req.pixels.at(i).points.at(j+1);
      geometry_msgs::Point point = pixelTo3DPoint(*POINTCLOUD, u, v);

      //std::cout << "Converted pixels" << std::endl;

      if (!isnan(point.x) && !isnan(point.y) && !isnan(point.z)) {
        xSum += point.x; ySum += point.y; zSum += point.z;
        count++;
      }
    }

    // don't count segments that only have a few pixels
    if (count > 20) {
      double xAvg = xSum/count; double yAvg = ySum/count; double zAvg = zSum/count;
      geometry_msgs::Point avgPoint;
      avgPoint.x = xAvg; avgPoint.y = yAvg; avgPoint.z = zAvg;
      points3d.push_back(avgPoint);
    }
    else {
      continue;
    }
  }
  //std::cout << "Num segments: " << points3d.size() << std::endl;
  res.conePos = points3d;
  return true;
}

/*
https://answers.ros.org/question/191265/pointcloud2-access-data/
*/
geometry_msgs::Point pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v) //, geometry_msgs::Point &p)
    {

      // get width and height of 2D point cloud data
      int width = pCloud.width;
      int height = pCloud.height;

      // Convert from u (column / width), v (row/height) to position in array
      // where X,Y,Z data starts
      int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

      // compute position in array where x,y,z data start
      int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
      int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
      int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8
      //std::cout << "array position x,y,z " << arrayPosX << " " << arrayPosY << " " << arrayPosZ << std::endl;

      float X = 0.0;
      float Y = 0.0;
      float Z = 0.0;

      memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
      memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
      memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

     // put data into the point p
     geometry_msgs::Point p;
      p.x = X;
      p.y = Y;
      p.z = Z;

      return p;
    }

int main(int argc, char **argv)
{
  // initialize node
  ros::init(argc, argv, "pointcloudSub", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  //color_detection::pixelTo3dPoint srv;

  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1, callback);
  ros::ServiceServer service = n.advertiseService("pixelTo3dPoint", getPixels);

  ros::spin();

  return 0;
}
