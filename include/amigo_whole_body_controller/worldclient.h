#ifndef WORLDCLIENT_H
#define WORLDCLIENT_H

#include "world.h"

namespace wbc {

class WorldClient
{
public:
    WorldClient();

    virtual void initialize() {}

    virtual WorldPtr getWorld() = 0;

    virtual void start() = 0;

    virtual void setIgnoredEntities(std::vector<std::string> entities) {}
};

} // namespace

#endif // WORLDCLIENT_H
