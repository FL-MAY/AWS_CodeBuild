//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************
#include <launcher_state_machine/StateMachine.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "launcher");
	rock::scrubber::launcher::StateMachine stateMachine;
	ros::spin();
	return 0;
}