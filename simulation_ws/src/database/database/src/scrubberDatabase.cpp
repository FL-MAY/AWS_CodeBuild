//
// Created by longyue on 19-7-31.
//

#include "database/DatabaseHelper.h"


int main(int argc, char **argv){
	ros::init(argc, argv, "scrubber_database");
	rock::scrubber::database::DatabaseHelper helper;
	ros::spin();
	return 0;
}
