##Usage: Not used in any topic, but should be used in every action integration related action.

##Description: a generic action result that is expected and required to be part of each ActionResult so that the action handlers can respond to the caller the kind of error or interruption faced.
bool success
uint64 millis_passed
string status  #Use the constants of ActionStatusConstants in the status field
moveit_msgs/MotionPlanResponse plan