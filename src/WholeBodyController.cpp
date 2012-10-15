#include "WholeBodyController.h"

WholeBodyController::WholeBodyController() {

    initialize();

}

WholeBodyController::~WholeBodyController() {

    ROS_INFO("Shutting down whole body controller");
    measured_torso_position_sub_.shutdown();
    measured_left_arm_position_sub_.shutdown();
    measured_right_arm_position_sub_.shutdown();
    measured_head_pan_sub_.shutdown();
    measured_head_tilt_sub_.shutdown();
    torso_pub_.shutdown();
    left_arm_pub_.shutdown();
    right_arm_pub_.shutdown();
    head_pub_.shutdown();
    /*CIleft_.~CartesianImpedance();
    CIright_.~CartesianImpedance();
    ComputeJacobian_.~ComputeJacobian();
    AdmitCont_.~AdmittanceController();
    ComputeNullspace_.~ComputeNullspace();
    JointLimitAvoidance_.~JointLimitAvoidance();*/

}

bool WholeBodyController::initialize() {

    // ToDo: Parameterize
    Ts = 0.1;

    // Get node handle
    ros::NodeHandle n("~");
    //ROS_INFO("Nodehandle %s",n.getNamespace().c_str());
    ROS_INFO("Initializing whole body controller");

    // Fill in component description map. This should be parsed from parameter file
    component_description_map_["torso"].root_name = "base";
    component_description_map_["torso"].tip_names.push_back("shoulder_mount_left");
    component_description_map_["torso"].tip_names.push_back("shoulder_mount_right");
    component_description_map_["torso"].number_of_joints = 0;
    component_description_map_["torso"].q.resize(component_description_map_["torso"].number_of_joints);

    component_description_map_["left_arm"].root_name = "shoulder_mount_left";
    component_description_map_["left_arm"].tip_names.push_back("grippoint_left");
    component_description_map_["left_arm"].number_of_joints = 0;
    component_description_map_["left_arm"].q.resize(component_description_map_["left_arm"].number_of_joints);

    component_description_map_["right_arm"].root_name = "shoulder_mount_right";
    component_description_map_["right_arm"].tip_names.push_back("grippoint_right");
    component_description_map_["right_arm"].number_of_joints = 0;
    component_description_map_["right_arm"].q.resize(component_description_map_["right_arm"].number_of_joints);
    /*struct component_description {
            std::string root_name;
            std::string tip_name;
            int number_of_joints;
            int start_index;
            int end_index;*/


    // Implement Jacobian matrix
    // Needs to be done before defining the subscribers, since the callback functions depend on
    // the mapping that results from this initialization
    ComputeJacobian_.Initialize(component_description_map_);
    ///ROS_INFO("Number of torso joints equals %i", component_description_map_["torso"].number_of_joints);
    ///ROS_INFO("Number of left arm joints equals %i", component_description_map_["left_arm"].number_of_joints);
    ///ROS_INFO("Number of right arm joints equals %i", component_description_map_["right_arm"].number_of_joints);

    // Initialize subscribers etc.
    setTopics();

    // Implement left Cartesian Impedance
    CIleft_.initialize(std::string("/grippoint_left"), 0);

    // Implement right Cartesian Impedance
    CIright_.initialize(std::string("/grippoint_right"), 6);

    // Initialize admittance controller
    AdmitCont_.initialize(ComputeJacobian_.q_min_, ComputeJacobian_.q_max_);

    // Initialize nullspace calculator
    // ToDo: Make this variable
    Eigen::MatrixXd A;
    A.resize(ComputeJacobian_.num_joints,ComputeJacobian_.num_joints);
    for (uint i = 0; i < ComputeJacobian_.num_joints; i++) {
        for (uint j = 0; j<ComputeJacobian_.num_joints; j++) {
            if (i==j) A(i,j) = 1;
            else A(i,j) = 0;
        }
    }
    ComputeNullspace_.initialize(ComputeJacobian_.num_joints, A);
    N_.resize(ComputeJacobian_.num_joints,ComputeJacobian_.num_joints);

    // Initialize Joint Limit Avoidance
    // ToDo: make variable
    std::vector<double> JLA_gain(ComputeJacobian_.num_joints);      // Gain for the joint limit avoidance
    std::vector<double> JLA_workspace(ComputeJacobian_.num_joints); // Workspace: joint gets 'pushed' back when it's outside of this part of the center of the workspace
    for (uint i = 0; i < ComputeJacobian_.num_joints; i++) {
        JLA_gain[i] = 1;
        JLA_workspace[i] = 0.9;
    }
    JointLimitAvoidance_.initialize(ComputeJacobian_.q_min_, ComputeJacobian_.q_max_, JLA_gain, JLA_workspace);
    ROS_INFO("Joint limit avoidance initialized");

    // Initialize Posture Controller
    // ToDo: make variable
    std::vector<double> posture_gain(ComputeJacobian_.num_joints);
    for (uint i = 0; i < ComputeJacobian_.num_joints; i++) posture_gain[i] = 1;
    PostureControl_.initialize(ComputeJacobian_.q_min_,ComputeJacobian_.q_max_,posture_gain);
    ROS_INFO("Posture Control initialized");

    // Resize additional variables
    F_task_.resize(0);
    q_current_.resize(ComputeJacobian_.num_joints);
    qdot_reference_.resize(ComputeJacobian_.num_joints);
    q_reference_.resize(ComputeJacobian_.num_joints);
    Jacobian_.resize(0,ComputeJacobian_.num_joints);                // Jacobian is resized to zero rows, number of rows depends on number of tasks
    tau_.resize(ComputeJacobian_.num_joints);
    tau_nullspace_.resize(ComputeJacobian_.num_joints);
    isactive_vector_.resize(2);
    previous_num_active_tasks_ = 0;

    ROS_INFO("Whole Body Controller Initialized");

    return true;

}

bool WholeBodyController::update() {

    // Set some variables to zero
    ///for (int i = 0; i<F_task_.rows(); i++) F_task_(i) = 0
    tau_.setZero();
    tau_nullspace_.setZero();
    uint force_vector_index = 0;
    uint num_active_tasks = 0;

    // Checking the number of active chains
    // ToDo: make variable
    if (CIleft_.is_active_) {
        isactive_vector_[0] = true;
        num_active_tasks++;
    }
    else isactive_vector_[0] = false;
    if (CIright_.is_active_) {
        isactive_vector_[1] = true;
        num_active_tasks++;
    }
    else isactive_vector_[1] = false;

    // Only resize F_task_ when number of active tasks has chaned
    if (num_active_tasks != previous_num_active_tasks_) {
        F_task_.resize(6*num_active_tasks);
        F_task_.setZero();
    }

    // Compute new Jacobian matrix
    ///ROS_INFO("Updating wholebodycontroller");
    ComputeJacobian_.Update(component_description_map_, isactive_vector_, Jacobian_);
    if (isactive_vector_[0] || isactive_vector_[1]) {
        uint show_column = 6;
        uint show_row = 0;
        ROS_INFO("Row %i, column %i of computed Jacobian is \n%f\n%f\n%f\n%f\n%f\n%f",show_row+1,show_column+1,Jacobian_(6*show_row+0,show_column),Jacobian_(6*show_row+1,show_column),Jacobian_(6*show_row+2,show_column),Jacobian_(6*show_row+3,show_column),Jacobian_(6*show_row+4,show_column),Jacobian_(6*show_row+5,show_column));
        ROS_INFO("Jacobian (x,q4) = %f, (y,q4) = %f, (z,q4) = %f, (z,torso) = %f", Jacobian_(0,3), Jacobian_(1,3), Jacobian_(2,3), Jacobian_(2,7));
    }
    ROS_INFO("Jacobian updated");

    // Update the torque output
    // Currently only one torque 'source', when multiple ones a smarter solution has to be found
    // Plan of attack: set tau to zero --> add in every update loop.
    // Note that nullspace solution should be included in various subproblems
    // Not sure if this is the right approach: summing the taus up and then doing nullspace projection may result in smoother transitions and is probably quicker
    CIleft_.update(F_task_, force_vector_index);
    CIright_.update(F_task_, force_vector_index);
    ///if (num_active_tasks == 2) ROS_INFO("F_task\n0x = %f\t0y = %f0z = %f\n0x = %f\t0y = %f\t0z = %f\n1x = %f\t1y = %f1z = %f\n1x = %f\t1y = %f\t1z = %f",F_task_(0),F_task_(1),F_task_(2),F_task_(3),F_task_(4),F_task_(5),F_task_(6),F_task_(7),F_task_(8),F_task_(9),F_task_(10),F_task_(11));

    // Compute torques (only compute tau_ if there is a task active)
    // ToDo: include this somehow in class
    if (isactive_vector_[0] || isactive_vector_[1]) {
        ROS_INFO("Update tau: Jacobian = [%i,%i], F_task = [%i,%i]",Jacobian_.rows(),Jacobian_.cols(),F_task_.rows(),F_task_.cols());
        tau_ = Jacobian_.transpose() * F_task_;
    }

    ///ROS_INFO("tau %f %f %f %f", tau_(0),  tau_(1),  tau_(2),  tau_(3));
    //ROS_INFO("FSpindle %f", tau_(7));

    ComputeNullspace_.update(Jacobian_, N_);
    //ROS_INFO("Nullspace updated");

    JointLimitAvoidance_.update(q_current_, tau_nullspace_);
    ROS_INFO("Joint limit avoidance updated");

    PostureControl_.update(q_current_, tau_nullspace_);
    ROS_INFO("Posture control updated");

    tau_ += N_ * tau_nullspace_;

    AdmitCont_.update(tau_,qdot_reference_, q_current_, q_reference_);

    ///ROS_INFO("qdr = %f %f %f %f", qdot_reference_(0), qdot_reference_(1), qdot_reference_(2), qdot_reference_(3));
    ROS_INFO("qdrspindle = %f", qdot_reference_(7));

    publishReferences();

    previous_num_active_tasks_ = num_active_tasks;

    return true;

}

void WholeBodyController::setTopics() {

    // Get node handle
    ros::NodeHandle n("~");

    std::vector<double> temp_vector;
    // ToDo: standardize data types joint positions
    // ToDo: make temp_vector.resize and map arguments variable
    // ToDo: fill in odom topic and make base publisher
    //odom_sub_ = ;
    measured_torso_position_sub_ = n.subscribe<std_msgs::Float64>("/spindle_position", 1, &WholeBodyController::callbackMeasuredTorsoPosition, this);
    temp_vector.resize(1);
    q_current_map_["spindle_joint"] = temp_vector;

    measured_left_arm_position_sub_ = n.subscribe<amigo_msgs::arm_joints>("/arm_left_controller/joint_measurements", 1, &WholeBodyController::callbackMeasuredLeftArmPosition, this);
    temp_vector.resize(7);
    q_current_map_["shoulder_yaw_joint_left"] = temp_vector;

    measured_right_arm_position_sub_ = n.subscribe<amigo_msgs::arm_joints>("/arm_right_controller/joint_measurements", 1, &WholeBodyController::callbackMeasuredRightArmPosition, this);
    temp_vector.resize(7);
    q_current_map_["shoulder_yaw_joint_right"] = temp_vector;

    measured_head_pan_sub_ = n.subscribe<std_msgs::Float64>("/head_pan_angle", 1, &WholeBodyController::callbackMeasuredHeadPan, this);
    temp_vector.resize(1);
    q_current_map_["pan_joint"] = temp_vector; //NOT_CORRECT

    measured_head_tilt_sub_ = n.subscribe<std_msgs::Float64>("/head_tilt_angle", 1, &WholeBodyController::callbackMeasuredHeadTilt, this);
    temp_vector.resize(1);
    q_current_map_["tilt_joint"] = temp_vector; //NOT CORRECT

    //ros::Publisher torso_pub_, left_arm_pub_, right_arm_pub_, head_pub_;
    torso_pub_ = n.advertise<amigo_msgs::spindle_setpoint>("/spindle_controller/spindle_coordinates", 10);
    left_arm_pub_ = n.advertise<amigo_msgs::arm_joints>("/arm_left_controller/joint_references", 10);
    right_arm_pub_ = n.advertise<amigo_msgs::arm_joints>("/arm_right_controller/joint_references", 10);
    head_pub_ = n.advertise<amigo_msgs::head_ref>("/head_controller/set_Head", 10);

}

void WholeBodyController::callbackMeasuredTorsoPosition(const std_msgs::Float64::ConstPtr& msg) {

    //TODO: Get rid of hardcoded root_joint

    std::string component_name = "torso";
    for (int i = 0; i<component_description_map_[component_name].number_of_joints; i++) {
        component_description_map_[component_name].q[i] = msg->data;
        q_current_(i+component_description_map_[component_name].start_index) = msg->data;
        //ROS_INFO("%s joint %i = %f",component_name.c_str(),i+1,component_description_map_[component_name].q[i]);
    }

}

void WholeBodyController::callbackMeasuredLeftArmPosition(const amigo_msgs::arm_joints::ConstPtr& msg) {

    //TODO: Get rid of hardcoded root_joint

    std::string component_name = "left_arm";
    uint num_comp_joints = component_description_map_[component_name].number_of_joints;
    for (uint i = 0; i<num_comp_joints; i++) {
        component_description_map_[component_name].q[i] = msg->pos[i].data;
        q_current_(i+component_description_map_[component_name].start_index) = msg->pos[i].data;

        // I guess we need to reverse this vector --> probably not
        ///component_description_map_[component_name].q[i] = msg->pos[num_comp_joints-1-i].data;
        //ROS_INFO("%s joint %i = %f",component_name.c_str(),i+1,component_description_map_[component_name].q[i]);
    }
}

void WholeBodyController::callbackMeasuredRightArmPosition(const amigo_msgs::arm_joints::ConstPtr& msg) {

    //TODO: Get rid of hardcoded root_joint

    std::string component_name = "right_arm";
    uint num_comp_joints = component_description_map_[component_name].number_of_joints;
    for (uint i = 0; i<num_comp_joints; i++) {
        component_description_map_[component_name].q[i] = msg->pos[i].data;
        q_current_(i+component_description_map_[component_name].start_index) = msg->pos[i].data;

        // I guess we need to reverse this vector --> probably not
        ///component_description_map_[component_name].q[i] = msg->pos[num_comp_joints-1-i].data;
        //ROS_INFO("%s joint %i = %f",component_name.c_str(),i+1,component_description_map_[component_name].q[i]);
    }
}

void WholeBodyController::callbackMeasuredHeadPan(const std_msgs::Float64::ConstPtr& msg) {

}

void WholeBodyController::callbackMeasuredHeadTilt(const std_msgs::Float64::ConstPtr& msg) {

}

void WholeBodyController::publishReferences() {

    amigo_msgs::spindle_setpoint torso_msg;
    amigo_msgs::arm_joints arm_msg;
    // Base

    // Torso
    torso_msg.stop = 0;
    std::string component_name("torso");
    /*for (int i = 0; i<component_description_map_[component_name].number_of_joints; i++) {
        torso_msg.pos = component_description_map_[component_name].q[i] +
                        qdot_reference_(component_description_map_[component_name].start_index+i)*Ts;
    }*/
    for (int i = 0; i<component_description_map_[component_name].number_of_joints; i++) {
        torso_msg.pos = q_reference_(component_description_map_[component_name].start_index + i);
    }
    torso_pub_.publish(torso_msg);
    ///ROS_INFO("Torso position = %f, reference = %f",component_description_map_[component_name].q[0],torso_msg.pos);

    // Left Arm
    component_name = "left_arm";
    /*for (int i = 0; i<component_description_map_[component_name].number_of_joints; i++) {
        arm_msg.pos[i].data = component_description_map_[component_name].q[i] +
                              qdot_reference_(component_description_map_[component_name].start_index+i)*Ts;
    }*/
    for (int i = 0; i<component_description_map_[component_name].number_of_joints; i++) {
        arm_msg.pos[i].data = q_reference_(component_description_map_[component_name].start_index + i);
    }
    left_arm_pub_.publish(arm_msg);

    // Right Arm
    component_name = "right_arm";
    /*for (int i = 0; i<component_description_map_[component_name].number_of_joints; i++) {
        arm_msg.pos[i].data = component_description_map_[component_name].q[i] +
                              qdot_reference_(component_description_map_[component_name].start_index+i)*Ts;
    }*/
    for (int i = 0; i<component_description_map_[component_name].number_of_joints; i++) {
        arm_msg.pos[i].data = q_reference_(component_description_map_[component_name].start_index + i);
    }
    right_arm_pub_.publish(arm_msg);

    // Head

}
