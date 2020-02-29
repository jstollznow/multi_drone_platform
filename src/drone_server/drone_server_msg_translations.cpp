
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/GetPlan.h"
#include "tf2_msgs/FrameGraph.h"

namespace mdp {

class id {
    private:
        bool owner = false;
        std_msgs::Header* data;

    public:
        id() {owner = true; data = new std_msgs::Header;}
        id(std_msgs::Header* header) {data = header;}
        ~id() {if (owner) {delete data;}}

        uint32_t& numeric_id() {return data->stamp.sec;}

        std_msgs::Header get_data() {return *data;}
};

class input_msg {
    private:
        geometry_msgs::TransformStamped* data;

    public:
        input_msg(geometry_msgs::TransformStamped* transform) {data = transform;}
        ~input_msg() {}

        id drone_id() { return id(&data->header); }
        std::string& msg_type() { return data->child_frame_id; }
        geometry_msgs::Vector3& pos_vel() { return data->transform.translation; }
        double& relative()  { return data->transform.rotation.x; }
        double& target()    { return data->transform.rotation.y; }
        double& yaw_rate()  { return data->transform.rotation.z; }
        double& yaw()       { return data->transform.rotation.z; }
        double& duration()  { return data->transform.rotation.w; }

};

class drone_feedback_srv {
    private:
        nav_msgs::GetPlan* data;
        
    public:
        drone_feedback_srv(nav_msgs::GetPlan* input) {data = input;}
        ~drone_feedback_srv() {}

        id drone_id() {return id(&data->request.start.header);}
        std::string& msg_type() {return data->request.goal.header.frame_id;}
        
        geometry_msgs::Point& vec3() {return data->response.plan.poses[0].pose.position;}
        double& forward_x() {return data->response.plan.poses[0].pose.orientation.x;}
        double& forward_y() {return data->response.plan.poses[0].pose.orientation.y;}
        double& yaw_rate()  {return data->response.plan.poses[0].pose.orientation.z;}
};

class drone_feedback_srv_req {
    private:
        nav_msgs::GetPlan::Request* req;
    public:
        drone_feedback_srv_req(nav_msgs::GetPlan::Request* data) {req = data;}
        ~drone_feedback_srv_req() {}

        id drone_id() {return id(&req->start.header);}
        std::string& msgType() {return req->goal.header.frame_id;}

};

class drone_feedback_srv_res {
    private:
        nav_msgs::GetPlan::Response* res;
    public:
        drone_feedback_srv_res(nav_msgs::GetPlan::Response* data) {res = data; res->plan.poses.push_back({});}
        ~drone_feedback_srv_res() {}

        std_msgs::Header& time_header() {return res->plan.poses[0].header;}
        geometry_msgs::Point& vec3() {return res->plan.poses[0].pose.position;}
        double& forward_x() {return res->plan.poses[0].pose.orientation.x;}
        double& forward_y() {return res->plan.poses[0].pose.orientation.y;}
        double& yaw_rate()  {return res->plan.poses[0].pose.orientation.z;}
};

}