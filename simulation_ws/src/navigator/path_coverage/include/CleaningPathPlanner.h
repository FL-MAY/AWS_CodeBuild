/***
 * @brief: cleaning robot path planning
 * @author: Tony
 * @date: 201909
***/

#ifndef CLEANINGPATHPLANNING_H
#define CLEANINGPATHPLANNING_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "trajectory_generator_waypoint.h"

#define ACCLIMIT 0.8
#define DEVORDER 3
#define VELLIMIT 0.8
#define ZERO 1E-8

using namespace cv;
using namespace Eigen;
using namespace std;

constexpr double PI = 3.14159;

struct cellIndex {
	int    row;
	int    col;
	double theta;
};

struct Astar {
	int row;
	int col;
	int father_row;
	int father_col;
	int F;
	int G;
	int H;
};

/*************************************************
 *
 * 读取栅格地图并根据占据信息获取其对应的空闲（可行走）空间，
 * 按照遍历算法规划行走路线。
 *
 * **********************************************/
class CleaningPathPlanning {
public:
	CleaningPathPlanning(costmap_2d::Costmap2DROS* costmap2d_ros, nav_msgs::Path polygon);

	~CleaningPathPlanning();

	vector<nav_msgs::Path> GetPathInROS();

	bool boundingJudge(int a, int b);

private:
	bool initializeMats();

	void getCellMatAndFreeSpaceInPolygon(Mat srcImg, Mat& cellMat, vector<cellIndex>& freeSpaceVec);

	void initializeNeuralMat(Mat cellMat, Mat neuralizedMat);

	void mainPlanningLoop();

	vector<cellIndex> GetPathInCV();

	bool isInPolygon(cellIndex cell, vector<cellIndex> polygon);

	bool isInLine(cellIndex cell, int& x0, int& y0, int& x1, int& y1);

	void findStartPoint(nav_msgs::Path polygon, double& x, double& y);

	bool isPolygonNarrow(nav_msgs::Path polygon);

	nav_msgs::Path polygonFilter(nav_msgs::Path polygon, double tolerance, unsigned int rotate_size);

	int modYaw(double yaw);

	void rotateCostmap(double angle);

	nav_msgs::Path pathOptimize(nav_msgs::Path& path);

	VectorXd timeAllocation(MatrixXd way_points, double vel_limit, double acc_limit);

	Vector3d getPosPoly(MatrixXd polyCoeff, int k, double t);


	bool                               initialized_;
	Mat                                srcMap_;
	Mat                                cellMat_;
	Mat                                neuralizedMat_;
	vector<cellIndex>                  freeSpaceVec_;
	vector<cellIndex>                  pathVec_;
	vector<geometry_msgs::PoseStamped> pathVecInROS_;
	vector<nav_msgs::Path>             pathCleaning_;
	vector<nav_msgs::Path>             pathOptimized_;
	nav_msgs::Path                     planning_polygon_;

	double                  resolution_;

	costmap_2d::Costmap2D   * costmap2d_;
	costmap_2d::Costmap2DROS* costmap2d_ros_;
	costmap_2d::Costmap2D   * origin_costmap_;

	cv::Point2f center_;
	double angle_;
	int rotated_start_index_;

	int SIZE_OF_CELL; //must be odd number.
};

#endif // CLEANINGPATHPLANNING_H
