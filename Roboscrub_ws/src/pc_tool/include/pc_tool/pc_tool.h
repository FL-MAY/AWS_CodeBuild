//
// Created by yang on 7/18/19.
//

#ifndef PC_TOOL_PC_TOOL_H
#define PC_TOOL_PC_TOOL_H


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <launcher_msgs/LauncherAction.h>
#include <launcher_msgs/PrepareNode.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt8.h>

namespace pc_tool {

    typedef actionlib::SimpleActionClient<launcher_msgs::LauncherAction> LauncherActionClient;
    typedef launcher_msgs::MissionResult States;
    typedef launcher_msgs::MissionGoal GoalActions;

    class ActionClient {
    private:
        actionlib::SimpleActionClient<launcher_msgs::LauncherAction> *action_client;
        uint8_t curr_states;
	    ros::Subscriber subscriber_;
        ros::NodeHandle n;

    public:
        ActionClient();
        ~ActionClient();

        void Action_Goal(uint8_t type, int ID);

        void doneCb(const actionlib::SimpleClientGoalState &state,
                    const launcher_msgs::LauncherResultConstPtr &result);

        void feedbackCb(const launcher_msgs::LauncherFeedbackConstPtr &fdbk_msg);

        void stateCallback(const std_msgs::UInt8::ConstPtr &state);

        void activeCb();

        void start();

        void pause();

        void resume();

        void cancel();

        void back();

	    void targetState(const uint8_t state_);

        bool g_active;

        uint8_t g_result;
        uint8_t getStates(){return curr_states;}
        bool serverExists_;
    };
}
#endif //PC_TOOL_PC_TOOL_H
