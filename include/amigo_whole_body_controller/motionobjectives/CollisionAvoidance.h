/*!
 * \author Paul Metsemakers
 * \date January, 2013
 * \version 0.1
 */

#ifndef WBC_OBSTACLEAVOIDANCE_H_
#define WBC_OBSTACLEAVOIDANCE_H_

#include <Eigen/Core>
#include <octomap/OcTreeStamped.h>

#include "MotionObjective.h"
#include "ChainParser.h"
#include "Chain.h"
#include "Tree.h"
#include "amigo_whole_body_controller/worldclient.h"
#include "amigo_whole_body_controller/Tracing.hpp"


#ifdef USE_BULLET
// Bullet GJK Closest Point calculation
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h>
#include <Bullet-C-Api.h>
#endif

#ifdef USE_FCL
// FCL closest point
#include <fcl/distance.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#endif

#include <profiling/StatsPublisher.h>

// ToDo: why not make total wrenches and total distances member variables? Now they're passed on from function to function

namespace wbc {

/**
 * @brief Data structure to wrap around RobotState::CollisionBody's for use in FCL
 *
 * The userData member of FCL Object's contains a void*. In this project
 * the userData is always a CollisionGeometryData* which can point back to it's RobotState::CollisionBody.
 *
 * A union is used (following the moveit convention) to provide easy future extension with for example a WorldObject::Body.
 */
struct CollisionGeometryData
{
  CollisionGeometryData(RobotState::CollisionBody *link)
  {
    ptr.link = link;
  }

  /*
  const std::string& getID() const
  {
    switch (type)
    {
    case BodyTypes::ROBOT_LINK:
      return ptr.link->getName();
    case BodyTypes::ROBOT_ATTACHED:
      return ptr.ab->getName();
    default:
      break;
    }
    return ptr.obj->id_;
  }

  std::string getTypeString() const
  {
    switch (type)
    {
    case BodyTypes::ROBOT_LINK:
      return "Robot link";
    case BodyTypes::ROBOT_ATTACHED:
      return "Robot attached";
    default:
      break;
    }
    return "Object";
  }
  */

  /** \brief Check if two CollisionGeometryData objects point to the same source object */
  bool sameObject(const CollisionGeometryData &other) const
  {
    return ptr.raw == other.ptr.raw;
  }

  union
  {
/*  const robot_model::LinkModel    *link;
    const robot_state::AttachedBody *ab;
    const World::Object             *obj; */
    const RobotState::CollisionBody *link;
    const void                      *raw;
  } ptr;

};

class CollisionAvoidance : public MotionObjective
{
public:

    /// Define the type of OctoMap as timestamped
    typedef octomap::OcTreeStamped OctreeType;

    struct collisionAvoidanceParameters
    {
        struct Parameters
        {
            double f_max;
            double f_min_percent;
            double d_threshold;
            int order;
            double octomap_resolution;
            double visualization_force_factor;
        } ;
        Parameters self_collision;
        Parameters environment_collision;
    } ca_param_;

    //ToDo: make configure, start- and stophook. Components can be started/stopped in an actionlib kind of fashion

    /**
     * Constructor
     */
    CollisionAvoidance(collisionAvoidanceParameters &parameters, const double Ts);

    /**
     * Deconstructor
     */
    ~CollisionAvoidance();

    /**
     * Initialize function
     */
    bool initialize(RobotState &robotstate);

    void apply(RobotState& robotstate);

    void setCollisionWorld(WorldClient *world_client);

    void setOctoMap(octomap::OcTreeStamped* octree);

    void removeOctomapBBX(const geometry_msgs::Point& goal, const std::string& root);

    /** Adds object collisionmodel to robot collisionmodel
     * @param frame_id: frame where the object is added to */
    void addObjectCollisionModel(const std::string& frame_id);

    /** Remove object collision model
     */
    void removeObjectCollisionModel();


protected:

    //! Sampling time
    double Ts_;

    Tree tree_;

    RobotState* robot_state_;

    ros::Publisher pub_model_marker_;
    ros::Publisher pub_model_marker_fcl_;
    ros::Publisher pub_forces_marker_;
    ros::Publisher pub_forces_marker_fcl_;
    ros::Publisher pub_bbx_marker_;

    struct Voxel {
        KDL::Frame center_point;
        double size_voxel;
    };
    struct Distance {
        std::string frame_id;
        btPointCollector bt_distance;
    } ;
    struct Distance2 {
        std::string frame_id;
#ifdef USE_FCL
        fcl::DistanceResult result;
#endif
    };

    struct RepulsiveForce {
        std::string frame_id;
        Eigen::Vector3d pointOnA;
        Eigen::Vector3d direction;
        float amplitude;
    };

    // provide << printing functionality
    friend std::ostream& operator << (std::ostream &o, const RepulsiveForce &r)
    {
        o << r.frame_id << " ";
        o << r.pointOnA[0]  << " " << r.pointOnA[1]  << " " << r.pointOnA[2]  << " ";
        o << r.direction[0] << " " << r.direction[1] << " " << r.direction[2] << " ";
        o << r.amplitude;
        return o;
    }

    /** Matrix containing the combined Jacobian matrix of this objective */
    Eigen::MatrixXd jacobian_pre_alloc_;
    /** Vector containing all relevant wrench entries */
    Eigen::VectorXd wrenches_pre_alloc_;

    KDL::Frame no_fix_;

    /**
     * @brief The current WorldClient that is in use
     *
     * This can be various implementations that inherit from WorldClient,
     * for example an OctomapWorld or an EdWorld
     */
    WorldClient *world_client_;

    octomap::OcTreeStamped* octomap_;

#ifdef USE_FCL
    fcl::DynamicAABBTreeCollisionManager selfCollisionManager;
#endif

    btConvexPenetrationDepthSolver*	depthSolver;
    btSimplexSolverInterface* simplexSolver;

    // Minimum and maximum point of the BBX
    std::vector<octomath::Vector3> min_;
    std::vector<octomath::Vector3> max_;

    /**
     * @brief Calculate the repulsive forces as a result of self collision avoidance
     * @param Output: Vector with the minimum distances between robot collision bodies, vector with the repulsive forces
     */
    void selfCollision(std::vector<Distance> &min_distances);
    void selfCollisionFast(std::vector<Distance2> &min_distances);

    /**
     * @brief Calculate the repulsive forces as a result of environment collision avoidance
     * @param Output: Vector with the minimum distances to the environment, vector with the repulsive forces
     */
    void environmentCollision(std::vector<Distance>  &min_distances);
#ifdef USE_FCL
    void environmentCollisionVWM(std::vector<Distance2> &min_distances);
#endif

    /**
     * @brief Construct the collision bodies
     * @param Input: The robot state
     */
    void initializeCollisionModel(RobotState &robotstate);

    /**
     * @brief Calculate the pose of the collision bodies
     */
    void calculateTransform();

    /**
     * @brief Calculate the Bullet shape pose from the corresponding FK pose using a correction from this frame pose
     * @param Input: Pose of the KDL frame and the fix for the collision bodies, Output: Bullet transform
     */
#ifdef USE_BULLET
    void setTransform(const KDL::Frame &fkPose, const KDL::Frame& fixPose, btTransform &transform_out);
#endif
#ifdef USE_FCL
    void setTransform(const KDL::Frame &fkPose, const KDL::Frame& fixPose, fcl::Transform3f &transform_out);
#endif

    /**
     * @brief Calculate the closest distance between two collision bodies
     * @param Input: The two collision bodies and their poses, output: The closest points with the corresponding distance en normal vector between them
     */
#ifdef USE_BULLET
    void distanceCalculation(btConvexShape &shapeA, btConvexShape &shapeB, btTransform& transformA, btTransform& transformB, btPointCollector& distance_out);
#endif
#ifdef USE_FCL
    static boost::shared_ptr<fcl::CollisionGeometry> shapeToMesh(const fcl::CollisionGeometry &shape, const fcl::Transform3f pose = fcl::Transform3f());
    void distanceCalculation(const fcl::CollisionObject* o1, const fcl::CollisionObject* o2, fcl::DistanceResult& result);
#endif

    /**
     * @brief Select the minimal closest distance
     * @param Vector with all closest distances from a collision body, Vector with the minimal closest distances
     */
#ifdef USE_BULLET
    void pickMinimumDistance(std::vector<Distance> &calculatedDistances, std::vector<Distance> &minimumDistances);
#endif
#ifdef USE_FCL
    void pickMinimumDistance(std::vector<Distance2> &calculatedDistances, std::vector<Distance2> &minimumDistances);
#endif
    /**
     * @brief Calculate the amplitude of the repulsive forces
     * @param Vector with the minimal closest distances, Vector with the repulsive forces
     */
    // ToDo: use membervariablefor collisionAvoidanceParameters
    // ToDo: use only 1 implementation for calculateRepulsiveForce
#ifdef USE_BULLET
    void calculateRepulsiveForce(const std::vector<Distance>  &minimumDistances, std::vector<RepulsiveForce> &repulsiveForces, const collisionAvoidanceParameters::Parameters &param);
#endif
#ifdef USE_FCL
    void calculateRepulsiveForce(const std::vector<Distance2> &minimumDistances, std::vector<RepulsiveForce> &repulsiveForces, const collisionAvoidanceParameters::Parameters &param);
#endif

    /**
     * @brief Calculate the wrenches as a function of the repulsive forces
     * @param Input: Vector with the repulsive forces, Output: Vector with the wrenches
     */
    void calculateWrenches(const std::vector<RepulsiveForce> &repulsive_forces);

    /**
     * @brief Visualize the collision avoidance in RVIZ
     * @param Vector with minimal distances
     */
    void visualize(std::vector<Distance> &min_distances) const;
    void visualizeRepulsiveForces(std::vector<Distance2> &min_distances) const;

    /**
     * @brief Construct the visualization markers to visualize the collision model in RVIZ
     * @param Collision body information
     */
#ifdef USE_BULLET
    void visualizeCollisionModel(RobotState::CollisionBody collisionBody,int id)  const;
#endif
#ifdef USE_FCL
    void visualizeCollisionModelFCL(RobotState::CollisionBody collisionBody,int id)  const;
#endif

    /**
     * @brief Construct the visualization markers to visualize the repulsive forces in RVIZ
     * @param Collision vector
     */
    void visualizeRepulsiveForce(Distance &d_min, int id) const;
    void visualizeRepulsiveForce(Distance2 &d_min,int id) const;

    /**
     * @brief Construct the visualization markers to visualize the bounding box in RVIZ
     * @param Minimum and maximum point of the bounding box
     */
    void visualizeBBX(octomath::Vector3 min, octomath::Vector3 max, int id) const;

    /**
     * @brief Find the outer points of the collision models for the bounding box construction in /map frame
     * @param Input: The collision body, Output: The minimum and maximum point of the collision body in /map frame
     */
    void findOuterPoints(RobotState::CollisionBody& collisionBody, btVector3 &min, btVector3 &max);

    /** Tracing object */
    Tracing tracer_;

    std::vector<Distance2>::const_iterator findMinimumDistance(const std::vector<Distance2> &distances, std::string link);

    std::vector<CollisionAvoidance::RepulsiveForce>::const_iterator findMaxRepulsiveForce(const std::vector<RepulsiveForce> &forces, std::string link);

    StatsPublisher statsPublisher_;
};

} // namespace

#endif
