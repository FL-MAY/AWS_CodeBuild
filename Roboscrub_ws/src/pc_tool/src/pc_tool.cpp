#include <pc_tool/pc_tool.h>
namespace pc_tool {

    ActionClient::ActionClient() : action_client(nullptr)
    {

        ros::Rate r(10.0);

        action_client = new LauncherActionClient("LauncherServer", true);

        ROS_INFO("attempting to connect to server");

        serverExists_ = action_client->waitForServer(ros::Duration(10.0));

        while (!serverExists_){
            ROS_WARN("could not connect to server; retrying...");
            serverExists_ = action_client->waitForServer(ros::Duration(10.0));
        }
        ROS_INFO("connected to action server");
        ros::NodeHandle nh_;
	    subscriber_ = nh_.subscribe("/launcher/scrubber_state", 1, &ActionClient::stateCallback, this);

        //curr_states=States::IDLE;
    }

    void ActionClient::Action_Goal(uint8_t type, int ID) {
        launcher_msgs::LauncherGoal goal_;
        goal_.mission.type = type;
        goal_.mission.id = ID;
        action_client->sendGoal(goal_,
                                boost::bind(&ActionClient::doneCb, this, _1, _2),
                                boost::bind(&ActionClient::activeCb, this),
                                boost::bind(&ActionClient::feedbackCb, this, _1)
                                );
    }

    void ActionClient::targetState(const uint8_t state_) {
	    ros::ServiceClient target_state = n.serviceClient<launcher_msgs::PrepareNode>("launcher/prepare");
	    launcher_msgs::PrepareNode srv;
	    srv.request.target_state = state_;
	    target_state.call(srv);
    }

	void ActionClient::stateCallback(const std_msgs::UInt8::ConstPtr& state){
    	 curr_states = state->data;
    	 ROS_INFO("i get curr_states");
    }

	void ActionClient::cancel(){
        action_client->cancelGoal();
    }

    void ActionClient::start(){
    	ros::ServiceClient start = n.serviceClient<std_srvs::Empty>("launcher/start");
    	std_srvs::Empty srv;
    	start.call(srv);
    }

    void ActionClient::pause(){
        ros::ServiceClient pause = n.serviceClient<std_srvs::Empty>("launcher/pause");
        std_srvs::Empty srv;
        pause.call(srv);
    }

    void ActionClient::resume(){
        ros::ServiceClient resume = n.serviceClient<std_srvs::Empty>("launcher/resume");
        std_srvs::Empty srv;
        resume.call(srv);
    }

    void ActionClient::back(){
    	action_client->cancelGoal();
    	targetState(States::IDLE);
    }

    void ActionClient::feedbackCb(const launcher_msgs::LauncherFeedbackConstPtr &fdbk_msg) {
        if (!fdbk_msg) {
            ROS_INFO("RECEIVE A EMPTY FEEDBACK");
            return;
        }
        //curr_states = fdbk_msg->state_feedback.state;
        //ROS_INFO("Got Feedback is %d", curr_states);
    }

    void ActionClient::activeCb() {
        //ROS_INFO("Goal just went active");
        g_active = true;
    }

    void ActionClient::doneCb(const actionlib::SimpleClientGoalState &state,
                              const launcher_msgs::LauncherResultConstPtr &result) {
        ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
        ROS_INFO("got result output = %d'", result->result.state);
        if (!result) {
            ROS_INFO("RECEIVE A EMPTY RESULT");
            return;
        }
        g_result = result->result.state;
        ROS_INFO("Receive result is %d", g_result);
	curr_states = g_result;
        g_active = false;
    }
}
