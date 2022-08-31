#include "instance.h"

using namespace PSLAM;

Instance::Instance(const NodePtr &node_ptr,float position_scale_, 
    ConfigPSLAM::StableInstance stable_threshold):
    instance_id(node_ptr->idx),parent(true),active_flag(true),
    position_scale(position_scale_),thresh_params(stable_threshold)
{
    nodelist.emplace(node_ptr->idx);
    nodes.emplace_back(node_ptr);

    {   
        label = node_ptr->GetLabel();
        height = 0.0f;
        if(label!=node_ptr->Unknown())
            clss_prb = node_ptr->mClsProb[label];
        else clss_prb = 0.0;
        if(label=="desk") label="table";
        centroid = position_scale * node_ptr->centroid;
        normal = node_ptr->sNormal;
        normalstd = node_ptr->sNormalStd;
        bboxmin = position_scale * node_ptr->bbox_min;
        bboxmax = position_scale * node_ptr->bbox_max;
        time_stamp.created = node_ptr->time_stamp_active;
        time_stamp.lastest_viewed = node_ptr->time_stamp_viewed;
        num_sfs = node_ptr->surfels.size();
        closet_object_type = "none";
        // if(label=="wall"&&std::abs(normal[2])>0.3) stable=false;
        if(num_sfs>thresh_params.min_surfels && clss_prb>thresh_params.min_class_weight
            && label!="none" && label!="otherfurniture" && label!="unknown")
            stable = true;
        // else if (label=="window"&&num_sfs>3000) stable=true;
        else stable = false;
    }
}

void Instance::updateAttributes()
{
    if(!parent)return;
    centroid.setZero();
    normal.setZero();
    num_sfs = 0;    
    int largest_node_size = 0;
    // size_t largest_node_create_time = 0;

    for(auto node_ptr:nodes){
        if(!node_ptr) continue;
        // if(!node_ptr->validFlag) continue;
        int node_sf_num = node_ptr->surfels.size();
        centroid += node_ptr->pos_sum;
        normal += node_ptr->Normal_sum;
        num_sfs += node_sf_num;
        for(int dim=0;dim<3;dim++){
            bboxmin[dim] = std::min(bboxmin[dim],position_scale* node_ptr->bbox_min[dim]);
            bboxmax[dim] = std::max(bboxmax[dim],position_scale* node_ptr->bbox_max[dim]);
        }
        if(node_sf_num>largest_node_size) time_stamp.created = node_ptr->time_stamp_active;
        // time_stamp.created = std::max(node_ptr->time_stamp_active,time_stamp.created);
        time_stamp.lastest_viewed = std::max(node_ptr->time_stamp_viewed,time_stamp.lastest_viewed);
    }
    centroid /=num_sfs;
    centroid *=position_scale;
    normal /=num_sfs;

    if(num_sfs>thresh_params.min_surfels && clss_prb>thresh_params.min_class_weight
        && label!="none" && label!="otherfurniture" && label!="unknown")
        stable = true;
    // else if(label=="window"&&num_sfs>3000) stable=true;
    else stable = false;
}

void Instance::addNode(const NodePtr &node_ptr)
{
    auto ret = nodelist.emplace(node_ptr->idx);
    nodes.emplace_back(node_ptr);
    // Eigen::Vector3f centroid_old = centroid;
    // Eigen::Vector3f normal_old = normal;
    // int num_sfs_old = num_sfs;
    // if(ret.second) {
    //     nodes.emplace_back(node_ptr);
    //     int num_sfs_add = node_ptr->surfels.size();
    //     num_sfs +=node_ptr->surfels.size();
    //     centroid = (num_sfs_old* centroid_old + num_sfs_add* node_ptr->centroid)/num_sfs;
    //     normal = (num_sfs_old*normal_old + node_ptr->Normal_sum) / num_sfs;
    // }
}

void Instance::checkOverlappedEdge(float max_cos_theta)
{
    if(label=="floor"||label=="wall") return;
    std::vector<int> neighbors_vec = getNeighborList();
    const int N=neighbors_vec.size();
    if(N<2) return;

    for(int i=0;i<N-1;i++){
        auto ed0_ptr = edges[neighbors_vec[i]];
        std::string ed0_label = ed0_ptr->node_to_label;
        if(ed0_label=="floor"||ed0_label=="wall") continue; 
        Eigen::Vector3f vec0 = ed0_ptr->edge_vec;
        
        for(int j=i+1;j<N;j++){
            auto ed1_ptr = edges[neighbors_vec[j]];
            std::string ed1_label = ed1_ptr->node_to_label;
            if(ed1_label=="floor"||ed1_label=="wall") continue;
            Eigen::Vector3f vec1 = ed1_ptr->edge_vec;
            float cos_theta = vec0.dot(vec1) / (vec0.norm()*vec1.norm());
            if(cos_theta>max_cos_theta){
                if(vec0.norm()<vec1.norm()) ed0_ptr->valid = false;
                else ed1_ptr->valid = false;
            }
        }
    }
}

// Not tested yet
void Instance::removeNode(const int id)
{
    auto ret_id = nodelist.find(id);
    if(ret_id!=nodelist.end()) nodelist.erase(ret_id);
}

void Instance::setLabel(std::string type)
{
    label = type;
    if(label=="desk") label = "table";
}

cv::Rect2f Instance::get2DBox()const
{
    return cv::Rect2f(bboxmin(0),bboxmin(1),
        bboxmax(0)-bboxmin(0),bboxmax(1)-bboxmin(1));
}

void Instance::reInitiate()
{
    parent = true;
    nodelist.clear();
    nodelist.emplace(instance_id);
}

bool Instance::isNeighborExist(const int nb_idx)
{
    if(neighbors.find(nb_idx)==neighbors.end()) return false;
    else return true;
}