#include <pc_tool/pc_tool.h>
using namespace pc_tool;


int main(int argc, char** argv) {

	ros::init(argc, argv, "action_client");
	ros::NodeHandle node;
	ros::Rate r(1);

	ActionClient* client = new ActionClient();
	int ch;
	while (node.ok()) {
		uint8_t state = client->getStates();
		if (state == States::IDLE) {
			ROS_INFO("Now state is IDLE \n MainManual: \n 1.AUTOMATIC 1 \n 2.MAPPING 2 \n 3.Tracking 3 \n 4.Manual");
			ROS_INFO("Input your chosen num: ");
			std::cin >> ch;
			if (ch == 1) {
				client->targetState(States::AUTOMATIC);
			} else if (ch == 2) {
				client->targetState(States::MAPPING);
			} else if (ch == 3) {
				client->targetState(States::TRACKING);
			} else if (ch == 4) {
                                client->targetState(States::MANUAL);
                        }else {
				ROS_ERROR("Please input the current num");
			}
		} else if (state == States::AUTOMATIC) {
			ROS_INFO("Now state is GOTO \n MainManual: \n 1.START \n 2.PAUSE \n 3.CANCEL \n 4.BACK \n");
			ROS_INFO("Input your chosen num: ");
			std::cin >> ch;

			if (ch == 1) {
				client->Action_Goal(GoalActions::AUTOMATIC, 10);
				client->start();
			} else if (ch == 2) {
				client->pause();
					//continue;
			} else if (ch == 3) {
				client->cancel();
			} else if (ch == 4) {
				client->back();
			} else {
				ROS_ERROR("Please input current num");
			}
		} else if (state == States::MAPPING) {
			ROS_INFO("Now state is MAPPING \n MainManual: \n 1.START \n");
			ROS_INFO("Input your chosen num: ");
			std::cin >> ch;

			if (ch == 1) {
				client->Action_Goal(GoalActions::MAPPING, 2);
				client->start();
				ROS_INFO("Here we go! \n SecManual: \n 1.PAUSE \n 2.CANCEL \n 3.BACK \n");
				ROS_INFO("Input your chosen num: ");
				std::cin >> ch;
				if (ch == 1) {
					client->pause();
				} else if (ch == 2) {
					client->cancel();
				} else if (ch == 3) {
					client->back();
				} else {
					ROS_ERROR("Please input the current num");
				}
			} else {
				ROS_ERROR("Please first start your mission");
			}
		} else if (state == States::TRACKING) {
			ROS_INFO("Now state is TRACKING \n MainManual: \n 1.START \n");
			ROS_INFO("Input your chosen num: ");
			std::cin >> ch;

			if (ch == 1) {
				client->Action_Goal(GoalActions::TRACKING, 3);
				client->start();
				ROS_INFO("Here we go! \n SecdManual: \n 1.PAUSE \n 2.CANCEL \n 3.BACK \n");
				ROS_INFO("Input your chosen num: ");
				std::cin >> ch;
				if (ch == 1) {
					client->pause();
				} else if (ch == 2) {
					client->cancel();
				} else if (ch == 3) {
					client->back();
				} else {
					ROS_ERROR("Please input the current num");
				}
			} else {
				ROS_ERROR("Please first start your mission");
			}
		} else if (state == States::MANUAL) {
			ROS_INFO("Now state is MANUAL \n MainManual: \n 1.START \n");
			ROS_INFO("Input your chosen num: ");
			std::cin >> ch;
			if (ch == 1) {
				client->Action_Goal(GoalActions::MANUAL, 3);
				client->start();
				ROS_INFO("Here we go! \n SecdManual: \n 1.PAUSE \n 2.CANCEL \n 3.BACK \n");
				ROS_INFO("Input your chosen num: ");
				std::cin >> ch;
				if (ch == 1) {
					client->pause();
				} else if (ch == 2) {
					client->cancel();
				} else if (ch == 3) {
					client->back();
				} else {
					ROS_ERROR("Please input the current num");
				}
			} else {
				ROS_ERROR("Please first start your mission");
			}
		} else if (state == States::PAUSE) {
			ROS_INFO("Now state is IDLE \n ThdManual: \n 1.RESUME \n 2.CANCEL \n 3.BACK \n");
			ROS_INFO("Input your chosen num: ");
			std::cin >> ch;
			
			if (ch == 1) {
				client->resume();
			} else if (ch == 2) {
				client->cancel();
			} else if (ch == 3) {
				client->back();
			} else {
				ROS_ERROR("Please input the current num");
			}
		}else if(state == States::PREPARING){
			ROS_INFO("Preparing.......wait");
		}else{
			ROS_INFO("Unknown state!");
		}
		ros::Duration(1).sleep();
		ros::spinOnce();
		//std::cin.ignore();//TODO:
	}
	//ros::spin();
	return 0;
}
