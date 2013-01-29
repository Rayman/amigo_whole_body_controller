/*!
 * \author Paul Metsemakers
 * \date January, 2013
 * \version 0.1
 */

#ifndef WBC_OBSTACLEAVOIDANCE_H_
#define WBC_OBSTACLEAVOIDANCE_H_

// ROS
#include "ros/ros.h"

// tf
#include <tf/transform_listener.h>

// Eigen
#include <Eigen/Core>

#include "MotionObjective.h"

/////

class ObstacleAvoidance : public MotionObjective {

    struct Box {

        Box(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
            : min_(min), max_(max) {
        }

        Eigen::Vector3d min_;
        Eigen::Vector3d max_;
    };

public:

    //ToDo: make configure, start- and stophook. Components can be started/stopped in an actionlib kind of fashion

    /**
     * Constructor
     */
    ObstacleAvoidance(const std::string& end_effector_frame, tf::TransformListener* tf_listener);

    /**
     * Deconstructor
     */
    virtual ~ObstacleAvoidance();

    /**
     * Initialize function
     */
    bool initialize();


    bool isActive();

    void apply(const RobotState& robotstate);

    void visualize(const Eigen::Vector3d& end_effector_pos, const Eigen::VectorXd& wrench) const;

protected:

    Chain* chain_;

    tf::TransformListener& listener_;

    std::string end_effector_frame_;

    std::vector<Box*> boxes_;

    ros::Publisher pub_marker_;

    geometry_msgs::PoseStamped end_effector_msg_;
    tf::Stamped<tf::Pose> tf_end_effector_pose_;
    tf::Stamped<tf::Pose> tf_end_effector_pose_MAP_;

    //void getposeRPY(geometry_msgs::PoseStamped& pose, Eigen::Vector3d& RPY) ;

};

#endif
