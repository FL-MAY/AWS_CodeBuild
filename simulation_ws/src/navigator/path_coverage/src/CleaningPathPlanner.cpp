#include "CleaningPathPlanner.h"
#include <costmap_2d/cost_values.h>

CleaningPathPlanning::CleaningPathPlanning(costmap_2d::Costmap2DROS* costmap2d_ros, nav_msgs::Path polygon) {
	costmap2d_ros_ = costmap2d_ros;
//	costmap2d_ros_->updateMap();
	origin_costmap_ = costmap2d_ros->getCostmap();
	sleep(1);
//	origin_costmap_->saveMap("/home/rock-trl/temp.pgm");

//	planning_polygon_ = polygon;
	planning_polygon_ = polygonFilter(polygon, 0.5, 4);

//	cout << "before rotate, planning polygon is" << endl;
//	for (auto& pose: planning_polygon_.poses) {
//		cout << "[" << pose.pose.position.x << ", " << pose.pose.position.y << "]" << endl;
//	}

	if (angle_ == 0.0) {
		costmap2d_ = origin_costmap_;
	} else {
		rotateCostmap(angle_);
		ROS_INFO("Rotate info: center: (%.2f, %.2f), angle: %.2f", center_.x, center_.y, angle_);
		auto cos_a = cos(angle_);
		auto sin_a = sin(angle_);
		//rotate polygon
		for (auto& pose: planning_polygon_.poses) {
			auto temp_x = pose.pose.position.x;
			auto temp_y = pose.pose.position.y;
			pose.pose.position.x = center_.x + (temp_x - center_.x) * cos_a - (temp_y - center_.y) * sin_a;
			pose.pose.position.y = center_.y + (temp_x - center_.x) * sin_a + (temp_y - center_.y) * cos_a;
		}
//		costmap2d_->saveMap("/home/rock-trl/rotated.pgm");
//		cout << "after rotate, planning polygon is" << endl;
//		for (auto& pose: planning_polygon_.poses) {
//			cout << "[" << pose.pose.position.x << ", " << pose.pose.position.y << "]" << endl;
//		}
	}
//	costmap2d_->saveMap("/home/rock-trl/rotated.pgm");

	ros::NodeHandle private_nh("~/cleaning_plan_nodehandle");

	string sizeOfCellString;
	SIZE_OF_CELL = 3;
	if (private_nh.searchParam("size_of_cell", sizeOfCellString)) private_nh.param("size_of_cell", SIZE_OF_CELL, 3);

	int sizex = costmap2d_->getSizeInCellsX();
	int sizey = costmap2d_->getSizeInCellsY();
	resolution_ = costmap2d_->getResolution();
	ROS_INFO("The size of map is [%d, %d] with resolution %.2f", sizex, sizey, resolution_);

	srcMap_ = Mat(sizey, sizex, CV_8U);
	for (auto r = 0; r < sizey; r++) {
		for (auto c = 0; c < sizex; c++) {
			//caution: costmap's origin is at left bottom ,while opencv's pic's origin is at left-top.
			srcMap_.at<uchar>(r, c) = costmap2d_->getCost(c, sizey - r - 1);
		}
	}

	initializeMats();
	//imwrite("debug_srcmap.jpg",srcMap_);

	if (!srcMap_.empty()) initialized_ = true;
	else initialized_ = false;
}

CleaningPathPlanning::~CleaningPathPlanning() {
}

vector<nav_msgs::Path> CleaningPathPlanning::GetPathInROS() {
	auto start_secs = ros::Time::now();
	if (!pathCleaning_.empty()) pathCleaning_.clear();

	nav_msgs::Path path;
	auto dist_threshold = 2.5 * resolution_ * SIZE_OF_CELL;
//	cout << "Distance threshold is " << dist_threshold << endl;
	dist_threshold = pow(dist_threshold, 2);

	geometry_msgs::PoseStamped posestamped;
	geometry_msgs::Pose        pose;
	vector<cellIndex>          cellvec;

	cellvec = GetPathInCV();
	if(cellvec.empty())
		return pathCleaning_;

	/**trasnsform**/
	vector<cellIndex>::iterator iter;
	auto sizey = cellMat_.rows;

	for (iter = cellvec.begin(); iter != cellvec.end(); iter++) {
		costmap2d_->mapToWorld((*iter).col * SIZE_OF_CELL + SIZE_OF_CELL / 2,
		                       (sizey - (*iter).row - 1) * SIZE_OF_CELL + SIZE_OF_CELL / 2,
		                       pose.position.x, pose.position.y);
		pose.orientation.w          = 1;
		pose.orientation.x          = 0;
		pose.orientation.y          = 0;
		pose.orientation.z          = 0;
		posestamped.header.stamp    = ros::Time::now();
		posestamped.header.frame_id = "map";
		posestamped.pose            = pose;

		if (!path.poses.empty()) {
			if ((pow(posestamped.pose.position.x - path.poses.back().pose.position.x, 2)
			     + pow(posestamped.pose.position.y - path.poses.back().pose.position.y, 2)) > dist_threshold) {
				pathCleaning_.push_back(path);
				path.poses.clear();
				path.poses.push_back(posestamped);
				continue;
			}
			path.poses.push_back(posestamped);
		} else { //Note by Tony: first point
			path.poses.push_back(posestamped);
		}
	}
	pathCleaning_.push_back(path); //Note by Tony: last point

	/**cut down way points**/
	for (auto& path: pathCleaning_) {
		auto it = path.poses.begin();
		for (; it < path.poses.end() - 1; it++) {
			(*it).pose.orientation.z = atan2((*(it + 1)).pose.position.y - (*it).pose.position.y,
			                                 (*(it + 1)).pose.position.x - (*it).pose.position.x);
		}
		(*it).pose.orientation.z = (*(it - 1)).pose.orientation.z;
	}

	#if 0
	bool erase_flag = true;
	for (auto& path: pathCleaning_) {
		auto it = path.poses.begin() + 1;
		for (; it < path.poses.end() - 1;) {
			if ((*it).pose.orientation.z == (*(it - 1)).pose.orientation.z) {
				if (erase_flag) {
					it = path.poses.erase(it);
				} else {
					++it;
				}
				erase_flag = !erase_flag;
			} else {
				++it;
				erase_flag = true;
			}
		}
	}
	#endif

	for (auto& path: pathCleaning_) {
		auto it = path.poses.begin() + 2;
		for (; it < path.poses.end() - 1;) { //Note by Tony: use CSC style, not CS style
			if ((*(it - 2)).pose.orientation.z != (*(it - 1)).pose.orientation.z &&
			    (*(it - 1)).pose.orientation.z == (*it).pose.orientation.z &&
				(*it).pose.orientation.z != (*(it + 1)).pose.orientation.z) {
				it = path.poses.erase(it);
			} else {
				++it;
			}
		}
	}

//	auto cv_secs = ros::Time::now();
//	ROS_WARN("Get way points time is %.2f secs", (cv_secs - start_secs).toSec());

	/**rotate back path**/
	if (angle_ != 0.0) {
		auto cos_an = cos(-angle_);
		auto sin_an = sin(-angle_);
		for (auto& path: pathCleaning_) {
			for (auto& pose: path.poses) {
				auto temp_x = pose.pose.position.x;
				auto temp_y = pose.pose.position.y;
				pose.pose.position.x = center_.x + (temp_x - center_.x) * cos_an - (temp_y - center_.y) * sin_an;
				pose.pose.position.y = center_.y + (temp_x - center_.x) * sin_an + (temp_y - center_.y) * cos_an;
//				cout << pose.pose.position.x << ", " << pose.pose.position.y << endl;
			}
		}
	}
//	auto rotate_secs = ros::Time::now();
//	ROS_WARN("Rotate way points time is %.2f secs", (rotate_secs - cv_secs).toSec());

	/**optimize**/
	if (!pathOptimized_.empty()) pathOptimized_.clear();
	for (auto i = 0; i < pathCleaning_.size(); i++) {
		auto om_path = pathOptimize(pathCleaning_[i]);
		if (om_path.poses.size() == 1 && pathCleaning_.size() > 1) {
			if (i == pathCleaning_.size() - 1) {
				auto theta = atan2(om_path.poses[0].pose.position.y - pathCleaning_[i - 1].poses.back().pose.position.y,
						om_path.poses[0].pose.position.x - pathCleaning_[i - 1].poses.back().pose.position.x);
				om_path.poses[0].pose.orientation.w = cos(theta / 2);
				om_path.poses[0].pose.orientation.z = sin(theta / 2);
			} else {
				auto theta = atan2(pathCleaning_[i + 1].poses.front().pose.position.y - om_path.poses[0].pose.position.y,
				                   pathCleaning_[i + 1].poses.front().pose.position.x - om_path.poses[0].pose.position.x);
				om_path.poses[0].pose.orientation.w = cos(theta / 2);
				om_path.poses[0].pose.orientation.z = sin(theta / 2);
			}
		}
		pathOptimized_.push_back(om_path);
	}

	auto optimize_secs = ros::Time::now();
//	ROS_WARN("Optimize way points time is %.2f secs", (optimize_secs - rotate_secs).toSec());
	ROS_WARN("Total time is %.2f secs", (optimize_secs - start_secs).toSec());
	ROS_INFO("Generate %d paths.", pathOptimized_.size());

	return pathOptimized_;
}

vector<cellIndex> CleaningPathPlanning::GetPathInCV() {
	mainPlanningLoop();
	return this->pathVec_;
}

bool CleaningPathPlanning::initializeMats() {
	//initialize the member variables.
	if (srcMap_.empty())
		return false;

	getCellMatAndFreeSpaceInPolygon(srcMap_, cellMat_, freeSpaceVec_);
	neuralizedMat_ = Mat(cellMat_.rows, cellMat_.cols, CV_32F);
	initializeNeuralMat(cellMat_, neuralizedMat_);
	return true;
}

void CleaningPathPlanning::getCellMatAndFreeSpaceInPolygon(Mat srcImg, Mat& cellMat, vector<cellIndex>& freeSpaceVec){
	cellMat = Mat(srcImg.rows / SIZE_OF_CELL, srcImg.cols / SIZE_OF_CELL, srcImg.type());
	vector<cellIndex> polygon;
	unsigned int a, b;
	bool duplicated = false;
	for (auto& pose: planning_polygon_.poses) {
		duplicated = false;
		costmap2d_->worldToMap(pose.pose.position.x, pose.pose.position.y, a, b);
		for (auto& point: polygon) {
			if (point.row == cellMat.rows - b / SIZE_OF_CELL - 1 && point.col == a / SIZE_OF_CELL) {
				duplicated = true;
			}
		}

		if (!duplicated)
			polygon.push_back(cellIndex{cellMat.rows - b / SIZE_OF_CELL - 1, a / SIZE_OF_CELL, 0});
	}

	for (auto& point: polygon) {
		ROS_INFO("[%d, %d]  ", point.row, point.col);
	}

	if (polygon.size() < 3) {
		ROS_ERROR("Got a polygon zone with less than 3 points. Planning at whole map.");
	}

	freeSpaceVec.clear();
	bool isFree   = true;
	int  obsPiece = 0;
	int  r = 0, c = 0, i = 0, j = 0;
	for (r = 0; r < cellMat.rows; r++) {
		for (c = 0; c < cellMat.cols; c++) {
			isFree   = true;
			obsPiece = 0;
			for (i = 0; i < SIZE_OF_CELL; i++) {
				for (j = 0; j < SIZE_OF_CELL; j++) {
					if (srcImg.at<uchar>(r * SIZE_OF_CELL + i, c * SIZE_OF_CELL + j) != costmap_2d::FREE_SPACE) {
						++obsPiece;
						if (obsPiece > 4) {
							isFree = false;
							i      = SIZE_OF_CELL;
							break;
						}
					}
				}
			}

			if (isFree && isInPolygon(cellIndex{r, c, 0}, polygon)) {
//				cout << "cell: [" << r << ", " << c << "]" << endl;
				cellIndex ci;
				ci.row   = r;
				ci.col   = c;
				ci.theta = 0;
				freeSpaceVec.push_back(ci);
				cellMat.at<uchar>(r, c) = costmap_2d::FREE_SPACE;
			} else {
				cellMat.at<uchar>(r, c) = costmap_2d::LETHAL_OBSTACLE;
			}
		}
	}

	ROS_INFO("free space size is %d", freeSpaceVec.size());
//	imwrite("/home/rock-trl/cellMat.jpg",cellMat);
}

void CleaningPathPlanning::initializeNeuralMat(Mat cellMat, Mat neuralizedMat) {
	int i = 0, j = 0;
	for (i = 0; i < neuralizedMat.rows; i++) {
		for (j = 0; j < neuralizedMat.cols; j++) {
			if (cellMat.at<uchar>(i, j) == costmap_2d::LETHAL_OBSTACLE) neuralizedMat.at<float>(i, j) = -100000.0;
			else neuralizedMat.at<float>(i, j) = 50.0 - j / neuralizedMat.cols;
		}
	}
}

void CleaningPathPlanning::mainPlanningLoop() {
	cellIndex initPoint, nextPoint, currentPoint, lastPoint;
	unsigned int mx, my;
	double wx, wy;

	if (freeSpaceVec_.empty()) {
		ROS_ERROR("no free space!");
		return;
	}

	if (planning_polygon_.poses.empty()) {
		ROS_ERROR("No zone peak!");
		return;
	}

	//TODO by Tony: change with different rect
	if (rotated_start_index_ == -1) {
		if (!isPolygonNarrow(planning_polygon_)) {
			initPoint.theta = 0;
		} else {
			initPoint.theta = 90;
		}
	} else {
		initPoint.theta = 0;
	}

	findStartPoint(planning_polygon_, wx, wy);
	ROS_INFO("start inittheta is %.2f, start idx is %d", initPoint.theta, rotated_start_index_);

	bool getmapcoor = costmap2d_->worldToMap(wx, wy, mx, my);
	if (!getmapcoor) {
		ROS_ERROR("Failed to set initial pose in map!");
		return;
	}

	initPoint.row = cellMat_.rows - my / SIZE_OF_CELL - 1;
	initPoint.col = mx / SIZE_OF_CELL;
	currentPoint = initPoint;
	pathVec_.clear();
	pathVec_.push_back(initPoint);

	bool          jump_point = false, change_weight = false;
	int           tiny_weight_down = -1, tiny_weight_right = -1;
	float         initTheta = initPoint.theta;
	const float   c_0 = 50;
	float         e   = 0.0, v = 0.0, v_1 = 0.0, deltaTheta = 0.0, lasttheta = initTheta, lasttheta_2 = lasttheta;
	vector<float> thetaVec  = {0, 45, 90, 135, 180, 225, 270, 315};

	/**main planning loop**/
	for (int loop = 0; loop < 9000; loop++) {
		vector<cellIndex>::iterator it;
		int   maxIndex = 0;
		float max_v    = -300;
		//Note by Tony: -250.0 means never search again
		neuralizedMat_.at<float>(currentPoint.row, currentPoint.col) = -250.0;
		float th = currentPoint.theta;
		for (int id = 0; id < 8; id++) {
			deltaTheta = max(thetaVec[id], th) - min(thetaVec[id], th);
			if (deltaTheta > 180) deltaTheta = 360 - deltaTheta;
			e = 1 - abs(deltaTheta) / 180;
			switch (id) {
				case 0:
					if (currentPoint.col == neuralizedMat_.cols - 1) {
						v = -100000;
						break;
					}

					v = neuralizedMat_.at<float>(currentPoint.row, currentPoint.col + 1) + c_0 * e;
					break;
				case 1:
					if (currentPoint.col == neuralizedMat_.cols - 1 || currentPoint.row == 0) {
						v = -100000;
						break;
					}

					v = neuralizedMat_.at<float>(currentPoint.row - 1, currentPoint.col + 1) + c_0 * e - 200;
					break;
				case 2:
					if (currentPoint.row == 0) {
						v = -100000;
						break;
					}

					v = neuralizedMat_.at<float>(currentPoint.row - 1, currentPoint.col) + c_0 * e;
					break;
				case 3:
					if (currentPoint.col == 0 || currentPoint.row == 0) {
						v = -100000;
						break;
					}

					v = neuralizedMat_.at<float>(currentPoint.row - 1, currentPoint.col - 1) + c_0 * e - 200;
					break;
				case 4:
					if (currentPoint.col == 0) {
						v = -100000;
						break;
					}
					//DO by Tony: Auto change weight through current direction
					v = neuralizedMat_.at<float>(currentPoint.row, currentPoint.col - 1) + c_0 * e + tiny_weight_down;
					break;
				case 5:
					if (currentPoint.col == 0 || currentPoint.row == neuralizedMat_.rows - 1) {
						v = -100000;
						break;
					}

					v = neuralizedMat_.at<float>(currentPoint.row + 1, currentPoint.col - 1) + c_0 * e - 200;
					break;
				case 6:
					if (currentPoint.row == neuralizedMat_.rows - 1) {
						v = -100000;
						break;
					}
					//DO by Tony: Auto change weight through current direction
					v = neuralizedMat_.at<float>(currentPoint.row + 1, currentPoint.col) + c_0 * e + tiny_weight_right;
					break;
				case 7:
					if (currentPoint.col == neuralizedMat_.cols - 1 || currentPoint.row == neuralizedMat_.rows - 1) {
						v = -100000;
						break;
					}

					v = neuralizedMat_.at<float>(currentPoint.row + 1, currentPoint.col + 1) + c_0 * e - 200;
					break;
				default:
					break;
			}

			if (v > max_v || v == max_v && id > maxIndex) {
				max_v = v;
				maxIndex = id;
			}
		}

		if (max_v <= 0) {
			float dist = 0.0, min_dist = 100000;
			int ii = 0, min_index = -1;
			for (it = freeSpaceVec_.begin(); it != freeSpaceVec_.end(); it++) {
				if (neuralizedMat_.at<float>((*it).row, (*it).col) > 0) {
					//Note by Tony: whether or not plan jump area
//					if (boundingJudge((*it).row, (*it).col)) {
						dist = sqrt((currentPoint.row - (*it).row) * (currentPoint.row - (*it).row)
						            + (currentPoint.col - (*it).col) * (currentPoint.col - (*it).col));
						if (dist < min_dist) {
							min_dist  = dist;
							min_index = ii;
						}
//					}
				}
				ii++;
			}

			if (min_index != -1 && min_dist != 100000) {
//				cout << "next point index: " << min_index << endl;
//				cout << "distance: " << min_dist << endl;
				nextPoint    = freeSpaceVec_[min_index];
//				cout << "current point: " << currentPoint.row << ", " << currentPoint.col << endl;
//				cout << "next point: " << nextPoint.row << ", " << nextPoint.col << endl << endl;
				lastPoint = currentPoint;
				currentPoint = nextPoint;
				pathVec_.push_back(nextPoint);
				//Note by Tony: bug#159 Remove jump strategy
//				jump_point = true;
				continue;
			} else {
//				ROS_INFO("The program has been dead because of the self-locking");
				ROS_WARN("The program has gone through %d steps", pathVec_.size());
				break;
			}
		}

		/**jump point strategy, renew initTheta**/
		if (jump_point && static_cast<int>(currentPoint.theta / 45) % 2 == 0) {
			jump_point = false;
			change_weight = true;
			if (static_cast<int>(currentPoint.theta / 90) % 2 == 0) {
				initTheta = 0;
				cout << "inittheta -> 0, current point: " << currentPoint.row << ", " << currentPoint.col << ", " << currentPoint.theta << endl;
			} else {
				initTheta = 90;
				cout << "inittheta -> 90, current point: " << currentPoint.row << ", " << currentPoint.col << ", "
				     << currentPoint.theta << endl;
			}
		}

		if (change_weight) {
			if (initTheta == 0) {
				if (currentPoint.theta != 0 && currentPoint.theta != 180) {
					if (currentPoint.theta < 180) {
						tiny_weight_right = -1;
					} else {
						tiny_weight_right = 1;
					}
					change_weight = false;
				}
			} else if (initTheta == 90) {
				if (currentPoint.theta != 90 && currentPoint.theta != 270) {
					if (currentPoint.theta < 90 || currentPoint.theta > 270) {
						tiny_weight_down = -1;
					} else {
						tiny_weight_down = 1;
					}
					change_weight = false;
				}
			} else {
				change_weight = false;
			}
		}

		if (initTheta == 0) {
			if ((currentPoint.theta != 0 && currentPoint.theta != 180) && currentPoint.theta == lasttheta && 0 == lasttheta_2) {
//				ROS_INFO("change line: up -> down");
//				cout << "current point: " << currentPoint.row << ", " << currentPoint.col << endl;
				maxIndex = 4;
				neuralizedMat_.at<float>(lastPoint.row, lastPoint.col) = 50.0 - lastPoint.col / neuralizedMat_.cols;
			}
			if ((currentPoint.theta != 0 && currentPoint.theta != 180) && currentPoint.theta == lasttheta && 180 == lasttheta_2) {
//				ROS_INFO("change line: down -> up");
//				cout << "current point: " << currentPoint.row << ", " << currentPoint.col << endl;
				maxIndex = 0;
				neuralizedMat_.at<float>(lastPoint.row, lastPoint.col) = 50.0 - lastPoint.col / neuralizedMat_.cols;
			}
		} else if (initTheta == 90) {
			if ((currentPoint.theta != 90 && currentPoint.theta != 270) && currentPoint.theta == lasttheta && 90 == lasttheta_2) {
//				ROS_INFO("change line: left -> right");
//				cout << "current point: " << currentPoint.row << ", " << currentPoint.col << endl;
				maxIndex = 6;
				neuralizedMat_.at<float>(lastPoint.row, lastPoint.col) = 50.0 - lastPoint.col / neuralizedMat_.cols;
			}
			if ((currentPoint.theta != 90 && currentPoint.theta != 270) && currentPoint.theta == lasttheta && 270 == lasttheta_2) {
//				ROS_INFO("change line: right -> left");
//				cout << "current point: " << currentPoint.row << ", " << currentPoint.col << endl;
				maxIndex = 2;
				neuralizedMat_.at<float>(lastPoint.row, lastPoint.col) = 50.0 - lastPoint.col / neuralizedMat_.cols;
			}
		}

		lasttheta_2 = lasttheta;
		lasttheta = currentPoint.theta;

		/**next point**/
		switch (maxIndex) {
			case 0:
				nextPoint.row = currentPoint.row;
				nextPoint.col = currentPoint.col + 1;
				break;
			case 1:
				nextPoint.row = currentPoint.row - 1;
				nextPoint.col = currentPoint.col + 1;
				break;
			case 2:
				nextPoint.row = currentPoint.row - 1;
				nextPoint.col = currentPoint.col;
				break;
			case 3:
				nextPoint.row = currentPoint.row - 1;
				nextPoint.col = currentPoint.col - 1;
				break;
			case 4:
				nextPoint.row = currentPoint.row;
				nextPoint.col = currentPoint.col - 1;
				break;
			case 5:
				nextPoint.row = currentPoint.row + 1;
				nextPoint.col = currentPoint.col - 1;
				break;
			case 6:
				nextPoint.row = currentPoint.row + 1;
				nextPoint.col = currentPoint.col;
				break;
			case 7:
				nextPoint.row = currentPoint.row + 1;
				nextPoint.col = currentPoint.col + 1;
				break;
			default:
				break;
		}

		nextPoint.theta = thetaVec[maxIndex];
		lastPoint = currentPoint;
		currentPoint = nextPoint;
		pathVec_.push_back(nextPoint);
	}
}

bool CleaningPathPlanning::boundingJudge(int a, int b) {
	int num = 0;
	for (int i = -1; i <= 1; i++) {
		for (int m = -1; m <= 1; m++) {
			if (i == 0 && m == 0) continue;
			if (neuralizedMat_.at<float>((a + i), (b + m)) == -250.0) num++;
		}
	}

	return num != 0;
}

//DO by Tony: support concave shape
bool CleaningPathPlanning::isInPolygon(cellIndex cell, vector<cellIndex> polygon) {
	auto size = polygon.size();
	if (size < 3) {
//		ROS_ERROR("Got a polygon zone with less than 3 points. Planning at whole map.");
		return true;
	}

	auto p_x = cell.row;
	auto p_y = cell.col;
	auto counter = 0;
	double xinters;
	int p1_x, p1_y, p2_x,p2_y;
	p1_x = polygon.back().row;
	p1_y = polygon.back().col;

	for (auto i = 0; i < size; i++) {
		p2_x = polygon[i].row;
		p2_y = polygon[i].col;

		if (isInLine(cell, p1_x, p1_y, p2_x, p2_y)) {
			return true;
		}

		if (p1_y == p2_y) {
			p1_x = p2_x;
			continue;
		}

		if (p_y >= min(p1_y, p2_y) && p_y < max(p1_y, p2_y) && p_x <= max(p1_x, p2_x)) {
			xinters = double((p_y - p1_y) * (p2_x - p1_x)) / (p2_y - p1_y) + p1_x;
			if (p1_x == p2_x || p_x <= xinters) {
				counter++;
			}
		}

		p1_x = p2_x;
		p1_y = p2_y;
	}

	return counter % 2 != 0;
}

bool CleaningPathPlanning::isInLine(cellIndex cell, int& x0, int& y0, int& x1, int& y1) {
	auto px = cell.row;
	auto py = cell.col;

	if (px > max(x0, x1) || px < min(x0, x1) || py > max(y0, y1) || py < min(y0, y1))
		return false;

	auto side = (py - y0) * (x1 - x0) - (px - x0) * (y1 - y0);

	return side == 0;
}

void CleaningPathPlanning::findStartPoint(nav_msgs::Path polygon, double& x, double& y) {
	auto min_x = polygon.poses.front().pose.position.x;
	auto min_y = polygon.poses.front().pose.position.y;
	auto min_index = 0;
	if (isPolygonNarrow(polygon)) {
		for (auto i = 1; i < polygon.poses.size(); i++) {
			if (polygon.poses[i].pose.position.x <= min_x) {
				min_x = polygon.poses[i].pose.position.x;
				min_y = polygon.poses[i].pose.position.y;
			}
		}

		for (auto i = 0; i < polygon.poses.size(); i++) {
			if (abs(polygon.poses[i].pose.position.x - min_x) <= ZERO) {
				if (polygon.poses[i].pose.position.y <= min_y) {
					min_y     = polygon.poses[i].pose.position.y;
					min_index = i;
				}
			}
		}
	} else {
		ROS_INFO("unnarrow!");
		for (auto i = 1; i < polygon.poses.size(); i++) {
			if (polygon.poses[i].pose.position.y <= min_y) {
				min_x = polygon.poses[i].pose.position.x;
				min_y = polygon.poses[i].pose.position.y;
			}
		}

		for (auto i = 0; i < polygon.poses.size(); i++) {
			if (abs(polygon.poses[i].pose.position.y - min_y) <= ZERO) {
				if (polygon.poses[i].pose.position.x <= min_x) {
					min_x     = polygon.poses[i].pose.position.x;
					min_index = i;
				}
			}
		}
	}

	x = planning_polygon_.poses[min_index].pose.position.x;
	y = planning_polygon_.poses[min_index].pose.position.y;
	//DO: Judge if idx is free
	unsigned int mx, my;
	if (!costmap2d_->worldToMap(x, y, mx, my))
		return;

	if (costmap2d_->getCost(mx, my) == costmap_2d::FREE_SPACE)
		return;

	int r0 = cellMat_.rows - my / SIZE_OF_CELL - 1;
	int c0 = mx / SIZE_OF_CELL;
	int min_dist = 100000;
	int r_min, c_min;
	for (auto& cell: freeSpaceVec_) {
		if ((abs(cell.row - r0) + abs(cell.col - c0)) < min_dist) {
			min_dist = abs(cell.row - r0) + abs(cell.col - c0);
			r_min = cell.row;
			c_min = cell.col;
		}
	}

	costmap2d_->mapToWorld(c_min * SIZE_OF_CELL + SIZE_OF_CELL / 2, (cellMat_.rows - r_min - 1) * SIZE_OF_CELL + SIZE_OF_CELL / 2, x, y);
	ROS_INFO("Start peak is not in free space, start point set form [%.2f, %.2f] to [%.2f, %.2f]",
	         planning_polygon_.poses[min_index].pose.position.x, planning_polygon_.poses[min_index].pose.position.y, x, y);
}

bool CleaningPathPlanning::isPolygonNarrow(nav_msgs::Path polygon) {
	auto min_x = polygon.poses.front().pose.position.x;
	auto max_x = min_x;
	auto min_y = polygon.poses.front().pose.position.y;
	auto max_y = min_y;
	for (auto i = 1; i < polygon.poses.size(); i++) {
		if (polygon.poses[i].pose.position.x < min_x) {
			min_x = polygon.poses[i].pose.position.x;
		}
		if (polygon.poses[i].pose.position.y < min_y) {
			min_y = polygon.poses[i].pose.position.y;
		}
		if (polygon.poses[i].pose.position.x >= max_x) {
			max_x = polygon.poses[i].pose.position.x;
		}
		if (polygon.poses[i].pose.position.y >= max_y) {
			max_y = polygon.poses[i].pose.position.y;
		}
	}

	auto length = max_x - min_x;
	auto height = max_y - min_y;

	return (height - length) > 0;
}

nav_msgs::Path CleaningPathPlanning::polygonFilter(nav_msgs::Path polygon, double tolerance, unsigned int rotate_size) {
	nav_msgs::Path filtered_polygon;

	/**get first last_yaw**/
	tf::Quaternion ql(polygon.poses.back().pose.orientation.x, polygon.poses.back().pose.orientation.y,
	                  polygon.poses.back().pose.orientation.z, polygon.poses.back().pose.orientation.w);
	tf::Matrix3x3 ml(ql);
	double r, p, y;
	ml.getRPY(r, p, y);
	auto last_yaw = modYaw(y);
	auto last_point = polygon.poses.back();
	filtered_polygon.poses.push_back(last_point);

	/**get corner loop**/
	for (auto& pose: polygon.poses) {
		tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y,
		                 pose.pose.orientation.z, pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double rr, pp, yy;
		m.getRPY(rr, pp, yy);
		auto yaw = modYaw(yy);
		auto dist = sqrt(pow(pose.pose.position.x - last_point.pose.position.x, 2)
				+ pow(pose.pose.position.y - last_point.pose.position.y, 2));

		if (yaw != last_yaw || dist > tolerance) {
			if (filtered_polygon.poses.front().pose.position.x != pose.pose.position.x ||
			    filtered_polygon.poses.front().pose.position.y != pose.pose.position.y) {
				filtered_polygon.poses.push_back(pose);
			}
		}

		last_yaw = yaw;
		last_point = pose;
	}

	ROS_INFO("filtered size is %d", filtered_polygon.poses.size());
	for (auto& pose: filtered_polygon.poses) {
		cout << "[" << pose.pose.position.x << ", " << pose.pose.position.y << "]" << endl;
	}

	/**rotate**/
	if (filtered_polygon.poses.size() > rotate_size || filtered_polygon.poses.size() < 3) {
		angle_ = 0.0;
		rotated_start_index_ = -1;
	} else {
		auto longest = 0.0;
		auto idx = 0;
		auto last = filtered_polygon.poses.back();
		//Find longest edge
		for (auto i = 0; i < filtered_polygon.poses.size(); i++) {
			auto dist = sqrt(pow(filtered_polygon.poses[i].pose.position.x - last.pose.position.x, 2)
			                 + pow(filtered_polygon.poses[i].pose.position.y - last.pose.position.y, 2));
			last = filtered_polygon.poses[i];
			if (dist > longest) {
				longest = dist;
				idx = (i == 0) ? (filtered_polygon.poses.size() - 1) : (i - 1);
			}
		}
		ROS_INFO("Find longest edge = %.2f", longest);

		//Get angle
		double sx, sy, ex, ey;
		sx = filtered_polygon.poses[idx].pose.position.x;
		sy = filtered_polygon.poses[idx].pose.position.y;
		if (idx == filtered_polygon.poses.size() - 1) {
			ex = filtered_polygon.poses[0].pose.position.x;
			ey = filtered_polygon.poses[0].pose.position.y;
		} else {
			ex = filtered_polygon.poses[idx + 1].pose.position.x;
			ey = filtered_polygon.poses[idx + 1].pose.position.y;
		}
		if (ex > sx) {
			angle_ = -atan2(ey - sy, ex - sx);
			rotated_start_index_ = idx;
		} else {
			angle_ = -atan2(sy - ey, sx - ex);
			rotated_start_index_ = (idx == filtered_polygon.poses.size() - 1) ? 0 : (idx + 1);
		}
		ROS_INFO("Angle = %.2f deg, start idx = %d", angle_ * 180.0 / 3.1415, rotated_start_index_);
	}

	return filtered_polygon;
}

int CleaningPathPlanning::modYaw(double yaw) {
	double theta = yaw * 180.0 / 3.14159;
	if (theta < 0)
		theta += 360.0;
	if (theta > 180.0)
		theta -= 180.0;
	if (theta >= 157.5 || theta < 22.5) {
		return 0;
	} else if (theta >= 22.5 && theta < 67.5) {
		return 1;
	} else if (theta >= 67.5 && theta < 112.5) {
		return 2;
	} else if (theta >= 112.5 && theta < 157.5) {
		return 3;
	} else {
		return 0;
	}
}

void CleaningPathPlanning::rotateCostmap(double angle) {
	auto cos_p = cos(angle);
	auto sin_p = sin(angle);
	auto cos_n = cos_p;
	auto sin_n = -sin_p;
	ROS_INFO("angle: %.2f, cos: %.2f, sin: %.2f", angle, cos_p, sin_p);

	auto width_cell_old  = origin_costmap_->getSizeInCellsX();
	auto height_cell_old = origin_costmap_->getSizeInCellsY();
	auto width_cell  = (unsigned int)(abs(width_cell_old * cos_p) + abs(height_cell_old * sin_p));
	auto height_cell = (unsigned int)(abs(width_cell_old * sin_p) + abs(height_cell_old * cos_p));
	auto width_old  = width_cell_old * origin_costmap_->getResolution();
	auto height_old = height_cell_old * origin_costmap_->getResolution();
	auto width  = width_cell * origin_costmap_->getResolution();
	auto height = height_cell * origin_costmap_->getResolution();
	ROS_INFO("old: (%.2f, %.2f); new: (%.2f, %.2f)", width_old, height_old, width, height);

	auto center_x = origin_costmap_->getOriginX() + width_old / 2;
	auto center_y = origin_costmap_->getOriginY() + height_old / 2;
	center_.x = center_x;
	center_.y = center_y;
//	unsigned int ix, iy;
//	origin_costmap_->worldToMap(center_x, center_y, ix, iy);
//	ROS_INFO("center index: (%d, %d); size: (%d, %d)", ix, iy, origin_costmap_->getSizeInCellsX(), origin_costmap_->getSizeInCellsY());

	auto origin_x = center_x - width / 2;
	auto origin_y = center_y - height / 2;
	ROS_INFO("both center: (%.2f, %.2f); old origin: (%.2f, %.2f); new origin: (%.2f, %.2f)",
	         center_x, center_y, origin_costmap_->getOriginX(), origin_costmap_->getOriginY(), origin_x, origin_y);

	costmap2d_ = new costmap_2d::Costmap2D(width_cell, height_cell, origin_costmap_->getResolution(), origin_x, origin_y);

//	ROS_INFO("after costmap build");

	for (auto i = 0; i < width_cell; i++) {
		for (auto j = 0; j < height_cell; j++) {
			double wx, wy;
			costmap2d_->mapToWorld(i, j, wx, wy);
			auto wx_old = center_x + (wx - center_x) * cos_n - (wy - center_y) * sin_n;
			auto wy_old = center_y + (wx - center_x) * sin_n + (wy - center_y) * cos_n;
			if (wx_old < origin_costmap_->getOriginX() || wx_old > origin_costmap_->getOriginX() + width_old ||
			    wy_old < origin_costmap_->getOriginY() || wy_old > origin_costmap_->getOriginY() + height_old) {
				costmap2d_->setCost(i, j, costmap_2d::NO_INFORMATION);
			} else {
				unsigned int ii, jj;
				origin_costmap_->worldToMap(wx_old, wy_old, ii, jj);
				costmap2d_->setCost(i, j, origin_costmap_->getCost(ii, jj));
			}
		}
	}
//	costmap2d_->saveMap("/home/rock-trl/rotatedfunc.pgm");
}

nav_msgs::Path CleaningPathPlanning::pathOptimize(nav_msgs::Path& path) {
	if (path.poses.size() < 2) {
		return path;
	}

	MatrixXd way_points(path.poses.size(), 3);
	for (auto i = 0; i < path.poses.size(); i++) {
		Vector3d pt(path.poses[i].pose.position.x, path.poses[i].pose.position.y, 0.0);
		way_points.row(i) = pt;
	}

	TrajectoryGeneratorWaypoint  trajectoryGeneratorWaypoint;
	MatrixXd vel = MatrixXd::Zero(2, 3);
	MatrixXd acc = MatrixXd::Zero(2, 3);

	auto poly_time = timeAllocation(way_points, VELLIMIT, ACCLIMIT);
	auto poly_coeff = trajectoryGeneratorWaypoint.PolyQPGeneration(DEVORDER, way_points, vel, acc, poly_time);

	nav_msgs::Path optimized_path;
	geometry_msgs::PoseStamped posestamped;

	/**position**/
	for (auto i = 0; i < poly_time.size(); i++) {
		for (double t = 0.0; t < poly_time(i); t += 0.05) {
			auto pos = getPosPoly(poly_coeff, i, t);
			posestamped.pose.position.x = pos(0);
			posestamped.pose.position.y = pos(1);
			posestamped.pose.position.z = 0.0;
			posestamped.pose.orientation.w = 1;
			posestamped.pose.orientation.x = 0;
			posestamped.pose.orientation.y = 0;
			posestamped.pose.orientation.z = 0;
			posestamped.header.stamp = ros::Time::now();
			posestamped.header.frame_id = "map";
			optimized_path.poses.push_back(posestamped);
		}
	}
	/**orientation**/
	auto it = optimized_path.poses.begin();
	for (; it < optimized_path.poses.end() - 1; it++) {
		auto theta = atan2((*(it + 1)).pose.position.y - (*it).pose.position.y,
		                   (*(it + 1)).pose.position.x - (*it).pose.position.x);
		(*it).pose.orientation.w = cos(theta / 2);
		(*it).pose.orientation.z = sin(theta / 2);
	}
	(*it).pose.orientation = (*(it - 1)).pose.orientation;

	return optimized_path;
}

VectorXd CleaningPathPlanning::timeAllocation(MatrixXd way_points, double vel_limit, double acc_limit) {
	VectorXd time(way_points.rows() - 1);
	auto lamda = 1.0;

	for (auto i = 0; i < (int)time.size(); i++) {
		auto dist = 0.0;
		for (auto m = 0; m < 2; m++) {
			dist += pow(way_points(i, m) - way_points(i + 1, m), 2);
		}
		dist = sqrt(dist);

		/**trapezoidal planning**/
		if (dist < vel_limit * vel_limit / acc_limit) {
			time(i) = sqrt(dist / acc_limit);
		} else {
			time(i) = vel_limit / acc_limit + dist / vel_limit;
		}
		time(i) *= lamda;
	}

	return time;
}

Vector3d CleaningPathPlanning::getPosPoly(MatrixXd polyCoeff, int k, double t) {
	Vector3d ret;

	for (int dim = 0; dim < 3; dim++) {
		VectorXd coeff = (polyCoeff.row(k)).segment(dim * 2 * DEVORDER, 2 * DEVORDER);
		VectorXd time  = VectorXd::Zero(2 * DEVORDER);

		for(int j = 0; j < 2 * DEVORDER; j++)
			if(j == 0)
				time(j) = 1.0;
			else
				time(j) = pow(t, j);

		ret(dim) = coeff.dot(time);
	}

	return ret;
}
