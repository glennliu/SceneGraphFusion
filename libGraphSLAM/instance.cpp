#include "instance.h"

using namespace PSLAM;

Instance::Instance(const NodePtr &node_ptr,float position_scale_, int min_surfels_):
    instance_id(node_ptr->idx),parent(true),active_flag(true),
    position_scale(position_scale_), min_surfels(min_surfels_)
{
    nodelist.emplace(node_ptr->idx);
    nodes.emplace_back(node_ptr);

    {   
        label = node_ptr->GetLabel();
        centroid = position_scale * node_ptr->centroid;
        normal = node_ptr->sNormal;
        bboxmin = position_scale * node_ptr->bbox_min;
        bboxmax = position_scale * node_ptr->bbox_max;
        time_stamp.created = node_ptr->time_stamp_active;
        time_stamp.lastest_viewed = node_ptr->time_stamp_viewed;
        num_sfs = node_ptr->surfels.size();
        if(num_sfs>min_surfels 
            && label!="none" && label!="otherfurniture" && label!="unknown")
            stable = true;
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

    if(num_sfs>min_surfels 
        && label!="none" && label!="otherfurniture" && label!="unknown")
        stable = true;
    else stable = false;
}

void Instance::addNode(const NodePtr &node_ptr)
{
    auto ret = nodelist.emplace(node_ptr->idx);
    if(ret.second) {
        nodes.emplace_back(node_ptr);
    }
}

void Instance::setLabel(std::string type)
{
    label = type;
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

