# Database

### Terms
* Plan
> A plan has a name and plan_id, assoiciate with a series of ordered tasks
* Task
> A task for now associate with a clean config, task type(For now just Follow Path), and a path to follow
* Path
> Array of pose, if it only have one single pose, then the pose is a target to navigate 
### Providing Service

* GetPLan
> Request : plan_id or plan_name
Respond: array of task
* AddPlan
> Request: name of new plan and array of task
Respond: None
* DelPlan
> Request: name of plan need to remove
Respond: None
* RenamePlan
> Request: old name and new name
Respond: None
* UpdatePlan
> Request: name of plan need to update and array of tasks
Respond: None
Note: this will rewrite the old plan
* GetTask
> Request: plan_id
Respond: array of tasks associate with this plan