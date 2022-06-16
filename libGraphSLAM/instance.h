#pragma once
#include <set>
#include <vector>
#include <memory>
#include "node.h"

namespace PSLAM
{
    typedef std::set<int> NodeList;
    
    class Instance
    {
        public:
        struct TimeStamp{
            size_t created;
            size_t lastest_viewed;
        };

        Instance(const NodePtr &node_ptr);//(int id);

        void addNode(const NodePtr &node_ptr);

        void setLabel(std::string type);

        void updateAttributes();

        // void setInValid();

        void reInitiate();    // Re-init instance

        const int getId()const{return instance_id;}

        const NodeList *getNodeList()const{
            return &nodelist;
        }

        const std::vector<ConstNodePtr> *getNodesPtr()const{
            return &nodes;
        }

        cv::Rect2f get2DBox()const;

        // const bool isValid()const{return active;}

        public:
            //TODO: ensure the unit is consistent
            Eigen::Vector3f centroid;   // in mm
            Eigen::Vector3f normal;
            Eigen::Vector3f bboxmin, bboxmax;
            TimeStamp time_stamp;
            unsigned int num_sfs;
            std::string label;
            std::unordered_set<size_t> neighbors;

            bool stable; // Set false for instances that are too small
            bool parent; // Set false if merged to other instance
            bool active_flag;
        private:
            int instance_id;
            NodeList nodelist;
            std::vector<ConstNodePtr> nodes;
            
    };

    typedef std::shared_ptr<Instance> InstancePtr;
}