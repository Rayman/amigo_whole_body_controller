#include "vwm/vwmclient.h"

#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"

#include <wire_volume/EntityHandle.h>
#include <wire_volume/WorldModel.h>

#include "profiling/Profiler.h"

namespace vwm_tools {

vwmClient::vwmClient()
    : cache(), world_objects()
{
}

vwmClient::~vwmClient()
{
}

void vwmClient::update()
{
    Timer updateTimer;
    updateTimer.start();

    vwm::WorldModelConstPtr world = client.getWorld();

    if (!world) {
        ROS_WARN_ONCE("The world model is not running! Environment collision will not be available");
        return;
    }

    const std::map<vwm::UUID, vwm::EntityHandle>& entities = world->getEntities();

    std::vector<fcl::CollisionObject*> objects;
    for(std::map<vwm::UUID, vwm::EntityHandle>::const_iterator it = entities.begin(); it != entities.end(); ++it) {
        vwm::EntityHandle e = it->second;

        fcl::CollisionObject* obj = cache.getCollisionObject(e);

        if (obj) {
            objects.push_back(obj);
        }
    }

    world_objects = objects;

    updateTimer.stop();
    ROS_INFO("VWM update took %f ms, (%li/%li) collision bodies found", updateTimer.getElapsedTimeInMilliSec(), objects.size(), entities.size());
}

std::vector<fcl::CollisionObject*> vwmClient::getWorldObjects()
{
    return world_objects;
}

} // namespace