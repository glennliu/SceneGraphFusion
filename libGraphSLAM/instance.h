#pragma once
#include <set>
#include <vector>
#include <memory>
#include "node.h"
#include "config.h"
// #include "corner.h"

namespace PSLAM
{
    class Corner
    {
    public:
        Corner() =default;
        std::pair<int,int> idxs;
        std::pair<std::string,std::string> types;
        Eigen::Vector2f distances;
        Eigen::Vector3f cross_vec;
        float cos_theta;
        float sin_theta;
    };
    typedef std::shared_ptr<Corner> CornerPtr;
    typedef std::set<int> NodeList;

    class Instance
    {
        public:
        struct TimeStamp{
            size_t created;
            size_t lastest_viewed;
        };
        struct Thresholds
        {
            int min_surfels=5000;
            float min_class_weight=0.5;
        };
        
        Instance(const NodePtr &node_ptr,float position_scale_, 
            // Thresholds threshold_params,
            ConfigPSLAM::StableInstance stable_threshold);

        void addNode(const NodePtr &node_ptr);

        void setLabel(std::string type);

        void updateAttributes();

        void removeNode(const int id);

        void checkOverlappedEdge(float max_cos_theta=0.9);

        // void setInValid();

        void reInitiate();    // Re-init instance

        const int getId()const{return instance_id;}

        const NodeList *getNodeList()const{
            return &nodelist;
        }

        const std::vector<ConstNodePtr> *getNodesPtr()const{
            return &nodes;
        }

        const std::vector<int> getNeighborList()const{
            std::vector<int> neighbor_vector;
            for(auto nb:neighbors){
                neighbor_vector.emplace_back(nb);
            }
            return neighbor_vector;
        }

        cv::Rect2f get2DBox()const;


        public:
            //TODO: ensure the unit is consistent
            Eigen::Vector3f centroid;   // in meters
            Eigen::Vector3f normal;
            Eigen::Vector3f normalstd;
            Eigen::Vector3f bboxmin, bboxmax;   // in meters
            float height;   // TODO: height relative to floor
            TimeStamp time_stamp;
            unsigned int num_sfs;
            std::string label;
            float clss_prb;
            std::unordered_set<size_t> neighbors;
            std::unordered_map<size_t,EdgePtr> edges;   // Directed edge
            std::vector<CornerPtr> corners; 
            std::vector<Eigen::Vector3i> rWalkDes; // Random walk descriptor
            std::vector<Eigen::Vector3i> rWalkRoute; // Random walk route

            bool stable; // Set false for instances that are too small
            bool parent; // Set false if merged to other instance
            bool active_flag;
        private:
            int instance_id;
            float position_scale;
            ConfigPSLAM::StableInstance thresh_params;
            // unsigned int min_surfels;  // Threshold to mark stable
            NodeList nodelist;
            std::vector<ConstNodePtr> nodes;
            
    };

    typedef std::shared_ptr<Instance> InstancePtr;
}