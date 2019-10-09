//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************
#include "scrubber/Scrubber.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "scrubber");
	rock::scrubber::scrubber::Scrubber scrubber_node;
	ros::spin();
	return 0;
}

