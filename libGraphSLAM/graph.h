#ifndef _H_INC_INSEG_MAP_SEGMENTS_IMPL_
#define _H_INC_INSEG_MAP_SEGMENTS_IMPL_
#include <unordered_set>
#include <unordered_map>
#include <mutex>
#include <iostream>

#include <Eigen/Dense>
#include <inseg_lib/surfel.h>
#include <inseg_lib/map_segments.h>
#include <ORUtils/thread_pool.hpp>

#include "config.h"
#include "node.h"
#include "edge.h"
#include "instance.h"

namespace PSLAM {

class Graph: public inseg_lib::SegmentsInterface {
public:
    struct TimeStampData{
        size_t time_created;
        size_t time_viewed;
    }; 

    ~Graph() {}
    Graph(const ConfigPSLAM *configPslam, bool useThread);

    int Add(const SurfelPtr &surfel) override;
    /// Merge segment idx_from to idx_to
    void Merge(const int seg_idx_to, const int seg_idx_from) override;
    void Clear() override;
    void Update(const SurfelPtr surfel,
                const Eigen::Vector3f& pos,
                const Eigen::Vector3f& normal) override;
    void CheckConnectivity(float margin);
    /**
     * Update the label of a surfel
     * This is updated during inseg
     * Remove surfel from node[surfel->label_old], chagne surfel->label to label and add it to node[label]
     * @param surfel target surfel
     * @param label new label
     * @return the index of the surfel in the graph
     */
    int UpdateLabel(const SurfelPtr surfel, const int label) override;
    int AddEdge(int from, int to, const std::shared_ptr<Edge> &edge);
    int AddEdge(const std::shared_ptr<Edge> &edge);

    void updateTimeStamp(const size_t timestamp_);

    // Update each node properties.
    void UpdateSelectedNodes(
        const std::unordered_set<int> &filtered_selected_nodes, 
        const size_t time, const bool force);

    void updateSelectedInstances(bool enable_init_new=false);

    void createInstances(const size_t time, float position_scale=0.001f);
        // const std::unordered_set<int> &nodes_toadd, 

    // For the nodes are updated, recrod their timestamp
    void RecordUpdateTime(const size_t &timestamp);
    
    //
    void RemoveInactiveNodes(const std::set<int> &nodes_to_remove);

    void RemoveInactiveInstances(const std::set<int> &instances_to_remove);

    void Wait();

    void labelNodes(const std::map<int,std::string> &instanceid_to_semantic);

    void labelTimestamp(const std::map<int,TimeStampData> &instance_timestamp);

    Instance *findInstance(int id);

    void transformNodes(const Eigen::Matrix4f T_);

    void transformEntireGraph(const Eigen::Matrix4f T_);

    // ===
    // thread
    // ===
    std::map<int, NodePtr> nodes;
    std::map<int, InstancePtr> instances;  
    std::map<std::pair<int,int>, EdgePtr > edges; // Directed edge
    std::map<std::pair<int,int>, EdgePtr > edges_ud; // Undirected edges; Just for visualization in loop detection

    std::set<int> nodes_to_update;
    std::set<int> instances_to_update;
    std::mutex mMutNode; // when access nodes
    std::mutex mMutEdge; // when access edges
    std::mutex mMutThis;

    // SLAM
    //TODO:
    std::map<std::string,std::vector<int>> nodes_group;
private:
    const ConfigPSLAM *mConfigPslam;
    void RemoveSurfelFromNode(const SurfelPtr &surfel);
    void RemoveNode(int idx);
    void RemoveEdge(const std::pair<int,int> &pair);
    bool mbThread;
    size_t timestamp_latest;

    std::unique_ptr<tools::TaskThreadPool> mPools;
};

}  // namespace inseg_lib

#endif  // _H_INC_INSEG_MAP_SEGMENTS_IMPL_
