//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "scrubber/scrubber_util.h"

namespace rock::scrubber::scrubber {

	bool onSegment(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r) {
		return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
		       q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);

	}

	int orientation(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r) {
		double val = (q.y - p.y) * (r.x - q.x) -
		             (q.x - p.x) * (r.y - q.y);

		if (val == 0) return 0;  // colinear

		return (val > 0) ? 1 : 2; // clock or counterclock wise
	}

	bool intersect(geometry_msgs::Point p1, geometry_msgs::Point q1,
	               geometry_msgs::Point p2, geometry_msgs::Point q2) {
		// Find the four orientations needed for general and
		// special cases
		int o1 = orientation(p1, q1, p2);
		int o2 = orientation(p1, q1, q2);
		int o3 = orientation(p2, q2, p1);
		int o4 = orientation(p2, q2, q1);

		// General case
		if (o1 != o2 && o3 != o4)
			return true;

		// Special Cases
		// p1, q1 and p2 are colinear and p2 lies on segment p1q1
		if (o1 == 0 && onSegment(p1, p2, q1)) return true;

		// p1, q1 and q2 are colinear and q2 lies on segment p1q1
		if (o2 == 0 && onSegment(p1, q2, q1)) return true;

		// p2, q2 and p1 are colinear and p1 lies on segment p2q2
		if (o3 == 0 && onSegment(p2, p1, q2)) return true;

		// p2, q2 and q1 are colinear and q1 lies on segment p2q2
		return o4 == 0 && onSegment(p2, q1, q2); // Doesn't fall in any of the above cases
	}

	nav_msgs::Path pointsToPath(std::vector<geometry_msgs::Point>& points) {
		nav_msgs::Path             path;
		geometry_msgs::PoseStamped pose;
		path.header.frame_id    = "map";
		pose.header.frame_id    = "map";
		pose.pose.orientation.z = 1;
		for (auto& point: points) {
			pose.pose.position.x = point.x;
			pose.pose.position.y = point.y;
			path.poses.push_back(pose);
		}

		return path;
	}

	bool rectEqual(const roborock_app::Rect& rec1, const roborock_app::Rect& rec2) {
		return std::fabs(rec1.first.x - rec2.first.x) < 0.05
		       && std::fabs(rec1.first.y - rec2.first.y) < 0.05
		       && std::fabs(rec1.second.x - rec2.second.x) < 0.05
		       && std::fabs(rec1.second.y - rec2.second.y) < 0.05
		       && std::fabs(rec1.third.x - rec2.third.x) < 0.05
		       && std::fabs(rec1.third.y - rec2.third.y) < 0.05
		       && std::fabs(rec1.fourth.x - rec2.fourth.x) < 0.05
		       && std::fabs(rec1.fourth.y - rec2.fourth.y) < 0.05;
	}

	int gzCompress(const char* src, int srcLen, char* dest, int destLen) {
		z_stream c_stream;
		uint8_t  err           = 0;
		uint8_t  windowBits    = 15;
		uint8_t  GZIP_ENCODING = 16;

		if (src && srcLen > 0) {
			c_stream.zalloc = (alloc_func) 0;
			c_stream.zfree  = (free_func) 0;
			c_stream.opaque = (voidpf) 0;
			if (deflateInit2(&c_stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED,
			                 windowBits | GZIP_ENCODING, 8, Z_DEFAULT_STRATEGY) != Z_OK)
				return -1;
			c_stream.next_in   = (Bytef*) src;
			c_stream.avail_in  = srcLen;
			c_stream.next_out  = (Bytef*) dest;
			c_stream.avail_out = destLen;
			while (c_stream.avail_in != 0 && c_stream.total_out < destLen) {
				if (deflate(&c_stream, Z_NO_FLUSH) != Z_OK) return -1;
			}
			if (c_stream.avail_in != 0) return c_stream.avail_in;
			for (;;) {
				if ((err = deflate(&c_stream, Z_FINISH)) == Z_STREAM_END) break;
				if (err != Z_OK) return -1;
			}
			if (deflateEnd(&c_stream) != Z_OK) return -1;
			return c_stream.total_out;
		}
		return -1;
	}

	int gzDecompress(const char* src, int srcLen, const char* dst, int dstLen) {
		z_stream strm;
		strm.zalloc = NULL;
		strm.zfree  = NULL;
		strm.opaque = NULL;

		strm.avail_in  = srcLen;
		strm.avail_out = dstLen;
		strm.next_in   = (Bytef*) src;
		strm.next_out  = (Bytef*) dst;

		int err = inflateInit2(&strm, MAX_WBITS + 16);
		if (err == Z_OK) {
			err = inflate(&strm, Z_FINISH);
			if (err == Z_STREAM_END) {
				err = strm.total_out;
			} else {
				inflateEnd(&strm);
				return err;
			}
		} else {
			inflateEnd(&strm);
			return err;
		}

		inflateEnd(&strm);
		return err;
	}

	std::vector<geometry_msgs::Point> rectToPointArray(const roborock_app::Rect& rect) {
		std::vector<geometry_msgs::Point> PointArray;
		PointArray.push_back(rect.first);
		PointArray.push_back(rect.second);
		PointArray.push_back(rect.third);
		PointArray.push_back(rect.fourth);
		return PointArray;
	}

	/*
	 * p0(x0, y0), p1(x1, y1) define a path l(p0->p1), candidate pose is px,py
	 * Return -1 if p on the right side of l, 0 p on l, 1 p on left side of l
	 */
	uint8_t sideOf(double& x0, double& y0, double& x1, double& y1, double& px, double& py) {

		double side = (py - y0) * (x1 - x0) - (px - x0) * (y1 - y0);
		if (side == 0) return 0;

		return side < 0 ? -1 : 1;
	}

	uint8_t TypeZoneToappMkpType(uint8_t& type_zone) {
		if (type_zone == db_msgs::TypeZone::REFILL) { //Refill clean water
			return roborock_app::MarkPointType::WATER;
		} else if (type_zone == db_msgs::TypeZone::DUMP) { //Dump dirty water
			return roborock_app::MarkPointType::DRAIN;
		} else if (type_zone == db_msgs::TypeZone::CHARGING) { //Recharge
			return roborock_app::MarkPointType::CHARGE;
		} else if (type_zone == db_msgs::TypeZone::PARKING) { //Parking lot
			return roborock_app::MarkPointType::PARK;
		}

		return 0;
	}

	/*
	 * App Mark Point type to database zone type
	 */
	uint8_t appMkpTypeToTypeZone(uint8_t& app_type) {
		if (app_type == roborock_app::MarkPointType::WATER) { //Refill clean water
			return db_msgs::TypeZone::REFILL;
		} else if (app_type == roborock_app::MarkPointType::DRAIN) { //Dump dirty water
			return db_msgs::TypeZone::DUMP;
		} else if (app_type == roborock_app::MarkPointType::CHARGE) { //Recharge
			return db_msgs::TypeZone::CHARGING;
		} else if (app_type == roborock_app::MarkPointType::PARK) { //Parking lot
			return db_msgs::TypeZone::PARKING;
		}

		return 0;
	}

	int8_t appSpeedTodbSpeed(uint8_t& app_speed, bool vel) {
		int offset = vel ? 3 : 0;
		switch (app_speed) {
			case roborock_app::Speed::STOP:
				return db_msgs::ConfigValueID::OFF;
			case roborock_app::Speed::SLOW:
				return db_msgs::ConfigValueID::CLEAN_LOW + offset;
			case roborock_app::Speed::MIDDLE:
				return db_msgs::ConfigValueID::CLEAN_MEDIUM + offset;
			case roborock_app::Speed::FAST:
				return db_msgs::ConfigValueID::CLEAN_HIGH + offset;
			default:
				return -1;
		}
	}

	int8_t dbSpeedToAppSpeed(uint8_t& db_speed) {
		switch (db_speed) {
			case db_msgs::ConfigValueID::CLEAN_LOW:
			case db_msgs::ConfigValueID::SPEED_LOW:
				return roborock_app::Speed::SLOW;
			case db_msgs::ConfigValueID::CLEAN_MEDIUM:
			case db_msgs::ConfigValueID::SPEED_MEDIUM:
				return roborock_app::Speed::MIDDLE;
			case db_msgs::ConfigValueID::CLEAN_HIGH:
			case db_msgs::ConfigValueID::SPEED_HIGH:
				return roborock_app::Speed::FAST;
			case db_msgs::ConfigValueID::OFF: //OFF
				return roborock_app::Speed::STOP;
			default:
				return -1;
		}
	}

	int8_t cfgModeToappCfgIndex(uint8_t cfgMode) {
		switch (cfgMode) {
			case db_msgs::ConfigMode::VACUUM:
				return roborock_app::ConfigIndex::VACCUM_MODE;
			case db_msgs::ConfigMode::ECU_WASH:
				return roborock_app::ConfigIndex::ECU_WASH_MODE;
			case db_msgs::ConfigMode::STANDARD_WASH:
				return roborock_app::ConfigIndex::STANDARD_WASH_MODE;
			case db_msgs::ConfigMode::CUSTOM:
				return roborock_app::ConfigIndex::CUSTOM_MODE;
			default:
				return -1;
		}
	}

	int8_t appCfgIndexToCfgMode(uint8_t app_index) {
		switch (app_index) {
			case roborock_app::ConfigIndex::VACCUM_MODE:
				return db_msgs::ConfigMode::VACUUM;
			case roborock_app::ConfigIndex::ECU_WASH_MODE:
				return db_msgs::ConfigMode::ECU_WASH;
			case roborock_app::ConfigIndex::STANDARD_WASH_MODE:
				return db_msgs::ConfigMode::STANDARD_WASH;
			case roborock_app::ConfigIndex::CUSTOM_MODE:
				return db_msgs::ConfigMode::CUSTOM;
			default:
				return -1;
		}
	}

	nav_msgs::Path rectToPath(roborock_app::Rect& rect) {
		nav_msgs::Path             path;
		geometry_msgs::PoseStamped pose;
		pose.pose.orientation.w = 1;
		pose.pose.position.x    = rect.first.x;
		pose.pose.position.y    = rect.first.y;
		path.poses.push_back(pose);
		pose.pose.position.x = rect.second.x;
		pose.pose.position.y = rect.second.y;
		path.poses.push_back(pose);
		pose.pose.position.x = rect.third.x;
		pose.pose.position.y = rect.third.y;
		path.poses.push_back(pose);
		pose.pose.position.x = rect.fourth.x;
		pose.pose.position.y = rect.fourth.y;
		path.poses.push_back(pose);
		return path;
	}

	roborock_app::Rect pathToRect(nav_msgs::Path& path) {
		roborock_app::Rect rect;
		if (path.poses.size() < 4) return rect;
		rect.first.x  = path.poses[0].pose.position.x;
		rect.first.y  = path.poses[0].pose.position.y;
		rect.second.x = path.poses[1].pose.position.x;
		rect.second.y = path.poses[1].pose.position.y;
		rect.third.x  = path.poses[2].pose.position.x;
		rect.third.y  = path.poses[2].pose.position.y;
		rect.fourth.x = path.poses[3].pose.position.x;
		rect.fourth.y = path.poses[3].pose.position.y;
		return rect;
	}

	void diagonal(std::vector<geometry_msgs::Point>& zone,
	              geometry_msgs::Point& min, geometry_msgs::Point& max) {
		if (zone.empty()) return;
		min.x = min.y = INT_MAX;
		max.x = max.y = INT_MIN;

		for (auto& point: zone) {
			if (point.x < min.x) {
				min.x = point.x;
			}

			if (point.y < min.y) {
				min.y = point.y;
			}

			if (point.x > max.x) {
				max.x = point.x;
			}

			if (point.y > max.y) {
				max.y = point.y;
			}
		}
	}

	/*
	 * return whether point inside zone
	 */
	bool inZone(std::vector<geometry_msgs::Point>& zone, geometry_msgs::Point& point) {
		uint16_t size = zone.size();
		if (size < 3) {
			ROS_ERROR("Got a polygon zone with less than 3 points");
			return false;
		}

		//Count how many time a ray start at (p_x, p_y) point to x dir intersects with the polygon
		//Even->outside  Odd->inside
		//Robot pose
		double p_x     = point.x;
		double p_y     = point.y;
		//Counter and other variable
		int    counter = 0;
		double xinters;
		//Initial polygon point
		double p1_x, p1_y, p2_x, p2_y;
		p1_x = zone.back().x;
		p1_y = zone.back().y;

		for (int i = 0; i < size; i++) {
			p2_x = zone[i].x;
			p2_y = zone[i].y;
			if (p1_y == p2_y) {
				p1_x = p2_x;//Update p1
				continue;
			}

			if (p_y > std::min(p1_y, p2_y) && p_y <= std::max(p1_y, p2_y)
			    && p_x <= std::max(p1_x, p2_x)) {
				xinters = (p_y - p1_y) * (p2_x - p1_x) / (p2_y - p1_y) + p1_x;
				if (p1_x == p2_x || p_x <= xinters) {
					counter++;
				}
			}

			//update p1
			p1_x = p2_x;
			p1_y = p2_y;
		}

		return counter % 2 != 0;
	}

	bool overlap(std::vector<geometry_msgs::Point>& one, std::vector<geometry_msgs::Point>& two) {
		//Check if diagnal overlap
		geometry_msgs::Point min_one, max_one, min_two, max_two;
		diagonal(one, min_one, max_one);
		diagonal(two, min_two, max_two);
		if (intersect(min_one, max_one, min_two, max_two)) {
			ROS_INFO("Found diagonal intersect");
			return true;
		}

		//Check whether any one's point in two
		for (auto& point: one) {
			if (inZone(two, point)) {
				return true;
			}
		}

		//Check whether any two's point in one
		for (auto& point: two) {
			if (inZone(one, point)) {
				return true;
			}
		}

		return false;
	}

	/**
	 *                     up
	 *
	 * *********************************************
	 *                                             *
	 * left mid                           right mid*    square bump
	 *                                             *
	 * *********************************************
	 *
	 *                     down
	 * @param zone
	 * @return
	 */
	std::vector<geometry_msgs::Point> crossRoute(std::vector<geometry_msgs::Point>& zone) {
		std::vector<geometry_msgs::Point> waypoints;
		geometry_msgs::Point left_mid, right_mid, mid, up, down;
		double               sin_theta, cos_theta, mid_dis;
		double               d = 1.0; //
		if (zone.size() == 4) { //Rectangle
			if (distance(zone[0], zone[1]) > distance(zone[1], zone[2])) {
				left_mid.x  = (zone[0].x + zone[3].x) / 2;
				left_mid.y  = (zone[0].y + zone[3].y) / 2;
				right_mid.x = (zone[1].x + zone[2].x) / 2;
				right_mid.y = (zone[1].y + zone[2].y) / 2;
				d +=  distance(zone[1], zone[2]) / 2;
			} else {
				left_mid.x  = (zone[0].x + zone[1].x) / 2;
				left_mid.y  = (zone[0].y + zone[1].y) / 2;
				right_mid.x = (zone[2].x + zone[3].x) / 2;
				right_mid.y = (zone[2].y + zone[3].y) / 2;
				d +=  distance(zone[0], zone[1]) / 2;
			}

			if (right_mid.x < left_mid.x) {
				geometry_msgs::Point tmp = left_mid;
				left_mid  = right_mid;
				right_mid = tmp;
			}

			mid_dis = distance(left_mid, right_mid);
			mid.x = (left_mid.x + right_mid.x) / 2;
			mid.y = (left_mid.y + right_mid.y) / 2;
			sin_theta = (right_mid.y - left_mid.y) / mid_dis;
			cos_theta = (right_mid.x - left_mid.x) / mid_dis;
			down.x = mid.x + d*sin_theta;
			down.y = mid.y - d*cos_theta;
			up.x   = mid.x - d*sin_theta;
			up.y   = mid.y + d*cos_theta;
			waypoints.push_back(up);
			waypoints.push_back(down);
		}

		return waypoints;
	}

	double distance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
		return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
	}

	bool compressMap(nav_msgs::OccupancyGrid& raw_map, nav_msgs::OccupancyGrid& compressed_map) {
		compressed_map.info   = raw_map.info;
		compressed_map.header = raw_map.header;
		int  data_size        = raw_map.info.height * raw_map.info.width;
		char compressed[data_size];
		int  c_len            = gzCompress((const char*) raw_map.data.data(), data_size, compressed, data_size);
		if (c_len > 0) {
			compressed_map.data = {(int8_t*) compressed, (int8_t*) compressed + c_len};
//			ROS_WARN("Map compressed, original size %d, compressed size %d",
//			         data_size, c_len);
//          DECOMPRESS FOR DEBUG ONLY
//			c_len = gzDecompress((const char*)app_map_info_.map.data.data(),
//					c_len, (const char*)decompressed, data_size);
//			ROS_WARN("Map decompressed, decompressed size %d", c_len);
//			if(c_len > 0) {
//				app_map_info_.map.data = {(int8_t*) decompressed, (int8_t*) decompressed + c_len};
//			}
			return true;
		}

		compressed_map = raw_map;
		ROS_ERROR("Failed to compress error code: %d", c_len);
		return false;
	}
}
