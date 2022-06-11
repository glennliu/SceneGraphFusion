#include "instance.h"

using namespace PSLAM;

Instance::Instance(int id):
    instance_id(id),valid(false)
{
    nodes.emplace(id);
    
}

void Instance::setLabel(std::string type)
{
    label = type;
}

void Instance::addNode(int node_id)
{
    nodes.emplace(node_id);
}



