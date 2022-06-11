#pragma once
#include <set>
#include <memory>

namespace PSLAM
{
    typedef std::set<int> NodeList;
    
    class Instance
    {
        public:
        Instance(int id);

        void addNode(int node_id);
        // {
        //     nodes.emplace(node_id);
        // }

        // void removeNode(int node_id){
        //     nodes.erase(node_id);
        // }

        void setLabel(std::string type);

        const NodeList *getNodeList()const{
            return &nodes;
        }
        
        private:
            int instance_id;
            NodeList nodes;
            std::string label;
            bool valid;

    };

    typedef std::shared_ptr<Instance> InstancePtr;
}