//
// Created by sc on 8/21/20.
//

#include "GraphSLAM.h"
#include "../libGraphSLAMGUI/Label_NYU40.h"
#include <ORUtils/LogUtil.h>
#ifdef COMPILE_WITH_TINYPLY
#include <tinyply.h>
#endif
#include <memory>

using namespace PSLAM;

GraphSLAM::GraphSLAM(ConfigPSLAM *config, const CameraParameters &camParamD):mConfig(config){
    mGraph = std::make_shared<Graph>(config, config->use_thread);//(config->use_thread);
    inactive_mGraph = std::make_shared<Graph>(config,config->use_thread);
    inseg_ = std::make_shared<inseg_lib::InSegLib>(mGraph, config->inseg_config, config->main_config, config->map_config, config->segmentation_config);
    // std::cout<<"Set depth edge thre:"<<inseg_->config().depth_edge_threshold<<"!\n";
    pose_.setIdentity();
    if(mConfig->graph_predict) {
#ifdef COMPILE_WITH_GRAPHPRED
        if(LoadPredictModel())
            mpGraphPredictor->RUN(mGraph.get());
        else
            mConfig->graph_predict = false;
#else
        SCLOG(WARNING) << "Did not compile with graph prediction.";
#endif
    }
    Initialize(camParamD);
};
GraphSLAM::~GraphSLAM(){
#ifdef COMPILE_WITH_GRAPHPRED
    if(mpGraphPredictor)mpGraphPredictor->Stop();
#endif
}

bool GraphSLAM::Initialize(const CameraParameters &camParamD){
    mbInitMap = false;
    Eigen::Matrix3d intrinsics;
    intrinsics(0,0) = camParamD.fx;
    intrinsics(1,1) = camParamD.fy;
    intrinsics(0,2) = camParamD.cx; 
    intrinsics(1,2) = camParamD.cy;
    inseg_->InitializeCamera(intrinsics, camParamD.width, camParamD.height);
    return true;
}

bool GraphSLAM::LoadPredictModel() {
#ifdef COMPILE_WITH_GRAPHPRED
    try {
        mpGraphPredictor = MakeGraphPredictor(mConfig);
        return true;
    } catch (...) {
        SCLOG(WARNING) << "Loading model from " << mConfig->pth_model << " failed.";
    }
#else
    SCLOG(WARNING) << "Did not compile with graph prediction.";
#endif
    return false;
}

void GraphSLAM::SaveModel(const std::string &output_folder) const {
    if (!inseg_) {
        printf("unable to save model. inseg is not initialized.\n");
        return ;
    }
    inseg_->map().SaveModel(output_folder+"/inseg.ply");
}

void GraphSLAM::ProcessFrame(int idx, const cv::Mat &colorImage, const cv::Mat &depthImage,const Eigen::Matrix4f *pose) {
    TicToc timer;
    SCLOG(INFO)<<"Processing frame "<< idx;
    mTimeStamp = idx;
    mGraph->updateTimeStamp(mTimeStamp);
    inactive_mGraph->updateTimeStamp(mTimeStamp);

    if(pose){
        pose_ = *pose;
        inseg_->set_pose(pose_);
        // std::cout<<"Input pose:\n"<<pose_.inverse()<<"\n";
    }

    // SCLOG(VERBOSE) <<"Edges:"<<mGraph->edges.size();
    CTICK("[SLAM][ProcessFrame]1.ProcessFrame");
    inseg_->ProcessFrame(depthImage, colorImage);   
    CTOCK("[SLAM][ProcessFrame]1.ProcessFrame");

    pose_ = inseg_->pose();

    if (!mbInitMap) {
        inseg_->InitializeMap(pose_);
        mbInitMap = true;
    }

    // std::cout<<"Output pose:\n"<<pose_.inverse()<<"\n";

    CTICK("[SLAM][ProcessFrame]2.ChechConnectivity");
    mGraph->CheckConnectivity(mConfig->neighbor_margin);

    mLastUpdatedSegments = mGraph->nodes_to_update;
    CTOCK("[SLAM][ProcessFrame]2.ChechConnectivity");

    mGraph->RecordUpdateTime(mTimeStamp);

    if(!mConfig->graph_predict) return;
    CTICK("[SLAM][ProcessFrame]4.SSCPrediction");
    AddSelectedNodeToUpdate(idx);
    CTOCK("[SLAM][ProcessFrame]4.SSCPrediction");

#ifdef COMPILE_WITH_GRAPHPRED
    if(mpGraphPredictor->Pin()){
        SCLOG(ERROR) << "prediction thread is dead.";
    }
#endif

    transitInactiveNodes(mTimeStamp);
    std::cout<<"Active graph: "
        << mGraph->instances.size() <<" Instances, "
        << mGraph->nodes.size() <<" Nodes, "
        << inseg_->map().surfels.size() <<" Surfels\n";
    std::cout<<"Processed in "<<timer.toc_ms()<<" ms\n";

}

void GraphSLAM::transitInactiveNodes(const size_t &timestamp)
{
    std::set<int> instances_tomove;
    std::set<int> nodes_tomove; 
    std::unordered_set<int> nodes_toadd;
    // std::map<int,std::string> instanceid_to_semantic;
    // std::map<int,Graph::TimeStampData> instance_timestamp;

    if(mGraph->instances.empty()) return;
    SCLOG(INFO)<<"Removing instances...";

    for(const auto &instance_itr:mGraph->instances){
        const int inactive_frame_count = (int)timestamp - instance_itr.second->time_stamp.lastest_viewed;
        const int active_frame_count   = (int)timestamp - instance_itr.second->time_stamp.created;

        if(inactive_frame_count > mConfig->inactive_frames_threshold ||
            active_frame_count > mConfig->active_frames_threshold)     
        { // Select inactive instances
            if(instance_itr.second->parent && instance_itr.second->stable)
            {
                instances_tomove.emplace(instance_itr.first);
                // Graph::TimeStampData node_timestamp;
                std::map<std::string, float> pd;
                std::map<std::string, std::pair<size_t, size_t>> sizeAndEdge;
                // size_t dominant_node_createtime(0);
                // size_t dominant_node_size(0);

                // Transmit surfels
                for(auto node_id:*instance_itr.second->getNodeList()){
                    auto node_ptr = mGraph->nodes.find(node_id);
                    if(node_ptr==mGraph->nodes.end()){
                        SCLOG(WARNING)<<"Instance "<<instance_itr.first
                            <<" is trying to transmit a non-exist node "<<node_id;
                        continue;
                    }

                    auto ret = nodes_tomove.emplace(node_id);
                    if(!ret.second){
                        SCLOG(WARNING)<<"Transmiting a node "<<node_id
                            << " belong to multiple instance."
                            << " inactive frames: "<<inactive_frame_count
                            << " active frames: "<<active_frame_count;
                        continue;
                    }
                    
                    for(auto &pair:node_ptr->second->surfels){
                        auto &surfel = pair.second;

                        // Move surfels of nodes to inactive graph
                        if( surfel->is_valid && surfel->is_stable){
                            SurfelPtr inactive_sf = std::make_shared<inseg_lib::Surfel>();

                            inactive_sf->pos = surfel->pos;
                            inactive_sf->normal = surfel->normal;
                            inactive_sf->color = surfel->color;
                            inactive_sf->radius = surfel->radius;
                            inactive_sf->SetLabel(instance_itr.first);
                            inactive_sf->is_valid = true;
                            inactive_sf->is_stable = true;

                            inactive_mGraph->Add(inactive_sf);
                        }
                        surfel->is_valid = false;
                        // inseg_->map().SetInvalid(surfel->label);
                    }
                    
                    if(node_id == instance_itr.first){
                        pd = node_ptr->second->mClsProb;
                        sizeAndEdge = node_ptr->second->mSizeAndEdge;
                    }
                    // dominant_node_createtime =std::min(dominant_node_createtime, node_ptr->second->time_stamp_active);
                }

                nodes_toadd.emplace(instance_itr.first);

                // Prediction and time stamp
                auto node_ptr_inactive = inactive_mGraph->nodes.find(instance_itr.first);
                if(node_ptr_inactive!=inactive_mGraph->nodes.end()){
                    node_ptr_inactive->second->UpdatePrediction(pd,sizeAndEdge,true);
                    node_ptr_inactive->second->time_stamp_active = instance_itr.second->time_stamp.created;
                    // std::min(
                    //     node_ptr_inactive->second->time_stamp_active,instance_itr.second->time_stamp.created);
                    node_ptr_inactive->second->time_stamp_viewed = instance_itr.second->time_stamp.lastest_viewed;
                }
                // Edges
                // for(const auto &ed:instance_itr.second->edges) inactive_mGraph->AddEdge(ed);
            }  
            else if(instance_itr.second->parent && instance_itr.second->getNodeList()->size()==1){  
                //Parent but unstable instance
                instances_tomove.emplace(instance_itr.first);
                auto node_ptr = mGraph->nodes.find(instance_itr.first);
                if(node_ptr!=mGraph->nodes.end()&&node_ptr->second->instance_idx==instance_itr.first){
                    auto ret = nodes_tomove.emplace(instance_itr.first);
                    node_ptr->second->DeactivateSurfels();
                }
            } 
        }
    }
    // std::cout<<"\n";

    for(auto node:mGraph->nodes){
        if(node.second->GetLabel() == Node::Unknown() && node.second->instance_idx == node.first){
            if(timestamp - node.second->time_stamp_viewed > (size_t)mConfig->inactive_frames_threshold
                || timestamp-node.second->time_stamp_active >(size_t) mConfig->active_frames_threshold){
                nodes_tomove.emplace(node.first);      
                node.second->DeactivateSurfels();     
            }
        }
    }

    SCLOG(INFO)<<"Selected nodes and instances to transmit";
    
    mGraph->RemoveInactiveNodes(nodes_tomove);
    mGraph->RemoveInactiveInstances(instances_tomove);
    inactive_mGraph->createInstances(timestamp, nodes_toadd,0.001f);

    std::cout<<"InactiveGraph: "
        <<inactive_mGraph->instances.size()<<" instances, "
        <<inactive_mGraph->nodes.size()<<" nodes\n";
}

void GraphSLAM::AddSelectedNodeToUpdate(int idx){
#ifdef COMPILE_WITH_ONNX
    if (!mpGraphPredictor) {
        SCLOG(WARNING) << "predictor is not initialized!";
        return;
    }
    SCLOG(VERBOSE) << "=== filter and extract graph ===";
    // SCLOG(VERBOSE) << "Nodes:"<<mGraph->nodes.size();
    CTICK("[SLAM][SemanticClassification]1.DataPreparation");
    std::unordered_set<int> filtered_selected_nodes;
    for (auto node_idx : mGraph->nodes_to_update)
        if (node_idx != 0 && int(mGraph->nodes.at(node_idx)->surfels.size()) > mConfig->filter_num_node)
            filtered_selected_nodes.insert(node_idx);
    SCLOG(VERBOSE) << "Update selected surfels.";
    mGraph->UpdateSelectedNodes(filtered_selected_nodes, idx, false);
    mGraph->updateSelectedInstances();

    mGraph->nodes_to_update.clear();

    if(!mConfig->use_thread) mpGraphPredictor->RUN(mGraph.get()); // if not use thread, call it every run
#else
    SCLOG(WARNING) << "Did not compile with onnx!";
#endif
}

void GraphSLAM::RunFullPrediction(){
#ifdef COMPILE_WITH_GRAPHPRED
    if (!mpGraphPredictor) {
        SCLOG(WARNING) << "predictor is not initialized!";
        return;
    }
    mpGraphPredictor->Stop();

    std::unordered_set<int> selected_nodes;
    for(const auto &pair: mGraph->nodes)
        if(pair.second->surfels.size() > (size_t)mConfig->filter_num_node)
            selected_nodes.insert(pair.first);
    mGraph->UpdateSelectedNodes(selected_nodes, 0, true);
    mGraph->Wait();
    mpGraphPredictor->RUN(mGraph.get());
#endif
}

void GraphSLAM::Start() {
#ifdef COMPILE_WITH_GRAPHPRED
    if(!mpGraphPredictor) return;
    if(!mConfig->use_thread)return;
    mpGraphPredictor->RUN(mGraph.get());
#endif
}

void GraphSLAM::Stop() {
#ifdef COMPILE_WITH_GRAPHPRED
    if(!mpGraphPredictor) return;
    if(!mConfig->use_thread)return;
    mpGraphPredictor->Stop();
#endif
}

std::vector<std::shared_ptr<inseg_lib::Surfel>> GraphSLAM::GetUpdatedSurfels(){
    const auto& surfels = inseg_->map().surfels;
    const auto& updated_segment_ids = mGraph->nodes_to_update;
    std::vector<std::shared_ptr<inseg_lib::Surfel>> filtered_surfels;
    filtered_surfels.reserve(surfels.size());
    for(const auto& surfel : surfels){
        if ((!surfel->is_valid || !surfel->is_stable) || (surfel->label == 0))
            continue;
        if(updated_segment_ids.find(surfel->label) == updated_segment_ids.end())
            continue;
        filtered_surfels.push_back(surfel);
    }
    SCLOG(VERBOSE)<< "Get clustered surfels \n";
    return filtered_surfels;
}

std::vector<std::shared_ptr<inseg_lib::Surfel>> GraphSLAM::FilterSegment(
        int segment_filter,  const std::vector<std::shared_ptr<inseg_lib::Surfel>> &surfels){
    if(segment_filter<=0) return surfels;
    std::map<unsigned short, size_t> counter;
    std::set<unsigned short> outliers;
    for (const auto &surfel:surfels){
        counter[surfel->label]++;
    }
    for(auto pair:counter){
        if (int(pair.second) < segment_filter) outliers.insert(pair.first);
    }
    std::vector<std::shared_ptr<inseg_lib::Surfel>> filtered;
    filtered.reserve(surfels.size());
    for (const auto &surfel:surfels){
        counter[surfel->label]++;
    }
    for (const auto &surfel:surfels){
        auto label = surfel->label;
        if(outliers.find(label) == outliers.end()){
            filtered.push_back(surfel);
        }
    }
    return filtered;
}

void GraphSLAM::SaveSurfelsToPLY(int segment_filter, const std::string &output_folder, const std::string &output_name, bool binary) {
    auto filtered_surfels = FilterSegment(segment_filter,inseg_->map().surfels);
    SaveSurfelsToPLY(output_folder, output_name, filtered_surfels, binary);
}

template<typename T, typename T2>
void write_binary (std::ostream& os, T2 v) {
    T vv = static_cast<T>(v);
    os.write((char*)&vv,sizeof(T));
}

void GraphSLAM::SaveNodesToPLY(int segment_filter, 
    const std::string &output_dir, SAVECOLORMODE saveColorMode, 
    bool binary, bool inactive_graph_) {
    size_t counter = 0;

    std::shared_ptr<Graph> target_graph;
    std::string path;
    if(inactive_graph_) {
        target_graph = std::shared_ptr<Graph>(inactive_mGraph);
        path = output_dir +"inactive_node";
    }
    else {
        target_graph = std::shared_ptr<Graph>(mGraph);
        path = output_dir +"active_node";
    }
    for (const auto &node : target_graph->nodes) {
        if (node.first == 0 ) continue;
        if (segment_filter > 0)
            if (node.second->surfels.size() < size_t(segment_filter)) continue;
        for(const auto &s : node.second->surfels)
            if(s.second->is_valid && s.second->is_stable) counter++;
    }
    if (counter == 0) {
        printf("No valid points found. return.\n");
        return;
    }

    // std::string path = output_folder + "/node";
    switch (saveColorMode) {
        case GraphSLAM::SAVECOLORMODE_RGB:
            path += "_RGB";
            break;
        case GraphSLAM::SAVECOLORMODE_SEGMENT:
            path += "_segment";
            break;
        case GraphSLAM::SAVECOLORMODE_INSTANCE:
            path += "_instance";
            break;
        case GraphSLAM::SAVECOLORMODE_SEMANTIC:
            path += "_semantic";
            break;
        case GraphSLAM::SAVECOLORMODEL_PANOPTIC:
            path += "_panoptic";
            break;
    }
    path += ".ply";
    std::fstream file;
    if(!binary)
        file.open(path, std::ios::out);
    else
        file.open(path, std::ios::out | std::ios::binary);
    assert(file.is_open());
    file << "ply\n";
    if(!binary)
        file << "format ascii 1.0\n";
    else
        file << "format binary_little_endian 1.0\n";
    file << "element vertex " + std::to_string(counter) + "\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property ushort label\n"
            "property ushort semantic\n"
            "property ushort instance\n"
            "property float nx\n"
            "property float ny\n"
            "property float nz\n"
            "property float curvature\n"
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "property uchar alpha\n"
            "property float quality\n"
            "property float radius\n"
            "end_header\n";

//    if(binary) {
//        file.close();
//        file.open(path, std::ios::out | std::ios::app | std::ios::binary);
//    }

    for (const auto &node: target_graph->nodes) {
        if (node.first == 0 ) continue;
        if (segment_filter > 0)
            if (node.second->surfels.size() < size_t(segment_filter)) continue;

//        bool print=true;
        for (const auto &pair :node.second->surfels) {
            if(!pair.second->is_valid || !pair.second->is_stable) continue;
            const auto &surfel = pair.second;
            // pos
            if(!binary) {
                for (size_t i = 0; i < 3; ++i) file << surfel->pos[i] / 1000.0f << " ";
            } else {
                for (size_t i = 0; i < 3; ++i) {
                    float v = surfel->pos[i] / 1000.0f;
                    file.write((char*)&v,sizeof(float));
                }
            }

            // label (segment label)
            if(!binary) file << node.first << " ";
            else {
                unsigned short value = node.first;
                file.write((char*)&value, sizeof(unsigned short));
            }

            // semantic
            {

                unsigned short value;
#ifdef COMPILE_WITH_GRAPHPRED
                if(node.second->GetLabel() != Node::Unknown())
                    value = mpGraphPredictor->mLabelsName2Idx.at(node.second->GetLabel()) + 1;
                else
                    value = 0;
#else
                value = 0;
#endif
                if(!binary) file << value << " ";
                else file.write((char*)&value,sizeof(unsigned short));
            }

            // instance
            if(!binary) file << node.second->instance_idx << " ";
            else {
                unsigned short value = node.second->instance_idx;
                file.write((char*)&value,sizeof(unsigned short));
            }

            // normal
            if(!binary) {
                for (size_t i = 0; i < 3; ++i) file << surfel->normal[i] << " ";
                file << "1 ";
            } else {
                for (size_t i = 0; i < 3; ++i) file.write((char*)&surfel->normal[i],sizeof(float));
                float value = 1.f;
                file.write((char*)&value,sizeof(float));
            }

            // color
            switch (saveColorMode) {
                case SAVECOLORMODE_RGB: {
                    if(!binary)
                        for (size_t i = 0; i < 3; ++i) file << int(surfel->color[2 - i]) << " ";
                    else {
                        for (size_t i = 0; i < 3; ++i) {
                            unsigned char value = surfel->color[2 - i];
                            file.write((char*)&value,sizeof(unsigned char));
                        }
                    }
                    break;
                }
                case SAVECOLORMODE_SEGMENT: {
                    const cv::Vec3b &color = inseg_lib::CalculateLabelColor(node.first);
                    if(!binary)
                        for (size_t i = 0; i < 3; ++i) file << int(color[i]) << " ";
                    else{
                        for (size_t i = 0; i < 3; ++i) {
                            unsigned char value = color[i];
                            file.write((char*)&value,sizeof(unsigned char));
                        }
                    }
                    break;
                }
                case SAVECOLORMODE_INSTANCE: {
                    const cv::Vec3b &color = inseg_lib::CalculateLabelColor(node.second->instance_idx);
                    if(!binary)
                        for (size_t i = 0; i < 3; ++i) file << int(color[i]) << " ";
                    else {
                        for (size_t i = 0; i < 3; ++i) {
                            unsigned char value = color[i];
                            file.write((char*)&value,sizeof(unsigned char));
                        }
                    }
                    break;
                }
                case SAVECOLORMODE_SEMANTIC: {
                    auto label = node.second->GetLabel();
#ifdef COMPILE_WITH_GRAPHPRED
                    if (label != Node::Unknown()) {
                        if (mpGraphPredictor->GetParams().at("label_type").string_value() == "NYU40" ||
                            mpGraphPredictor->GetParams().at("label_type").string_value() == "ScanNet20") {
                            auto &name = label;
                            auto idx = NYU40Name2Labels.at(name);
                            const auto &color_ = NYU40LabelColors.at(idx);
                            if(!binary)
                                for (size_t i = 0; i < 3; ++i) file << int(color_[i]) << " ";
                            else {
                                for (size_t i = 0; i < 3; ++i) {
                                    unsigned char value = color_[i];
                                    file.write((char*)&value,sizeof(unsigned char));
                                }
                            }
                        } else {
                            const cv::Vec3b &color = inseg_lib::CalculateLabelColor(mpGraphPredictor->mLabelsName2Idx.at(label));
                            if(!binary)
                                for (size_t i = 0; i < 3; ++i) file << int(color[2 - i]) << " ";
                            else {
                                for (size_t i = 0; i < 3; ++i) {
                                    unsigned char value = color[2 - i];
                                    file.write((char*)&value,sizeof(unsigned char));
                                }
                            }
                        }
                    } else
#endif
                    {
                        if (!binary)
                            for (size_t i = 0; i < 3; ++i) file << "0 ";
                        else {
                            for (size_t i = 0; i < 3; ++i) {
                                unsigned char value = 0;
                                file.write((char*)&value,sizeof(unsigned char));
                            }
                        }
                    }
                    break;
                }

                case SAVECOLORMODEL_PANOPTIC: {
                    const auto &name = node.second->GetLabel();
                    cv::Vec3b color;
#ifdef COMPILE_WITH_GRAPHPRED
                    if (name != Node::Unknown() && (name == "wall" || name == "floor")) {
                        auto idx = NYU40Name2Labels.at(name);
                        auto color_ = NYU40LabelColors.at(idx);
                        color[0] = color_[0];
                        color[1] = color_[1];
                        color[2] = color_[2];
                    } else {
                        auto inst = node.second->instance_idx.load();
                        if (inst < (int)mpGraphPredictor->mLabels.size()) {
                            const auto & name2 = mpGraphPredictor->mLabels.at(inst);
                            if (name2 == "wall" || name2 == "floor") {
                                inst+=mpGraphPredictor->mLabels.size();
                            }
                        }
                        auto color_ = inseg_lib::CalculateLabelColor(inst);
                        for(auto i=0;i<3;++i)
                            color[i] = color_[2-i];
                    }
#else
                    color = {0,0,0};
#endif

                    if(!binary)
                        for (size_t i = 0; i < 3; ++i) file << int(color[i]) << " ";
                    else {
                        for (size_t i = 0; i < 3; ++i) {
                            unsigned char value = color[i];
                            file.write((char*)&value,sizeof(unsigned char));
                        }
                    }
                    break;
                }

            }
            if (!binary)
                file << "255 ";
            else {
                unsigned char value = 255;
                file.write((char*)&value,sizeof(unsigned char));
            }

            // quality
            if(!binary)
                file << surfel->confidence << " ";
            else
                file.write((char*)&surfel->confidence,sizeof(float));
            // radius
            if(!binary)
                file << surfel->radius << "\n";
            else
                file.write((char*)&surfel->radius,sizeof(float));

        }

    }

    file.close();
}

void GraphSLAM::SaveSurfelsToPLY(const std::string &output_dir, const std::string &output_name, const std::vector<std::shared_ptr<inseg_lib::Surfel>> &surfels,
        bool binary) {
#ifdef WITH_TINYPLY
    struct TINYPLY_SURFEL {
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> pts;
        std::vector<unsigned short> labels;
        std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> normals;
        std::vector<std::array<unsigned char, 4>> colors;
        std::vector<float> quality;
        std::vector<float> radius;
    };
    TINYPLY_SURFEL tinysurfels;
#endif

    // calculate labeled surfels
    size_t counter = 0;
    for(const auto &surfel:surfels)
        if(surfel->label>0 && surfel->is_valid && surfel->is_stable)counter++;

    std::fstream file;
    std::string output_file_dir = output_dir + output_name;
    std::cout<<"saving to "<< output_file_dir<<"\n"
        <<"nodes: "<<mGraph->nodes.size()<<"\n";
        // <<"edges: "<<mGraph->edges.size()<<"\n";
    for(auto node_ptr:mGraph->nodes){
        std::cout<<node_ptr.first<<",";
    }
    std::cout<<"\n";
    if(!binary)
        file.open(output_dir+output_name, std::ios::out);
    else
        file.open(output_dir+output_name, std::ios::out | std::ios::binary);
    assert(file.is_open());

    file << "ply\n";
    if(!binary)
        file << "format ascii 1.0\n";
    else
        file << "format binary_little_endian 1.0\n";

    file << "element vertex " + std::to_string(counter) + "\n"
                                                          "property float x\n"
                                                          "property float y\n"
                                                          "property float z\n"
                                                          "property ushort label\n"
                                                          "property float nx\n"
                                                          "property float ny\n"
                                                          "property float nz\n"
                                                          "property float curvature\n"
                                                          "property uchar red\n"
                                                          "property uchar green\n"
                                                          "property uchar blue\n"
                                                          "property uchar alpha\n"
                                                          "property float quality\n"
                                                          "property float radius\n"
                                                          "end_header\n";


    for (const auto &surfel:surfels){
        if(surfel->label==0 || !surfel->is_valid || !surfel->is_stable) continue;
        // pos
        if(!binary)
            for(size_t i=0;i<3;++i) file << surfel->pos[i]/1000.0f << " ";
        else {
            for(size_t i=0;i<3;++i) {
                float v = surfel->pos[i] / 1000.0f;
                write_binary<float>(file, v);
            }
        }
        // label
        if(!binary) file << surfel->label << " ";
        else write_binary<unsigned short>(file,surfel->label);
        // normal
        if(!binary) {
            for (size_t i = 0; i < 3; ++i) file << surfel->normal[i] << " ";
            file << "1 ";
        } else {
            for (size_t i = 0; i < 3; ++i) write_binary<float>(file, surfel->normal[i]);
            write_binary<float>(file,1.f);
        }
        // color
        const bool segment_color = false;
        if(!binary) {
            if (segment_color) {
                const cv::Vec3b &color = inseg_lib::CalculateLabelColor(surfel->label);
                for (size_t i = 0; i < 3; ++i) file << int(color[i]) << " ";
            } else {
                for (size_t i = 0; i < 3; ++i) file << int(surfel->color[2 - i]) << " ";
            }
            file << "255 ";
        } else {
            if (segment_color) {
                const cv::Vec3b &color = inseg_lib::CalculateLabelColor(surfel->label);
                for (size_t i = 0; i < 3; ++i) write_binary<unsigned char>(file,color[i]);
            } else {
                for (size_t i = 0; i < 3; ++i) write_binary<unsigned char>(file,surfel->color[2-i]);
            }
            write_binary<unsigned char>(file,255);
        }

        // quality
        if(!binary) file << surfel->confidence << " ";
        else write_binary<float>(file,surfel->confidence);
        // radius
        if(!binary)file << surfel->radius << "\n";
        else write_binary<float>(file,surfel->radius);

#ifdef WITH_TINYPLY
        tinysurfels.pts.push_back(surfel->pos);
        tinysurfels.labels.push_back(surfel->label);
        Eigen::Vector4f n;
        n.head<3>() = surfel->normal; n.w() =1;
        tinysurfels.normals.emplace_back(n);
        std::array<unsigned char,4> c = {surfel->color[2],surfel->color[1],surfel->color[0],255};
        tinysurfels.colors.push_back(c);
        tinysurfels.quality.push_back(surfel->confidence);
        tinysurfels.radius.push_back(surfel->radius);
#endif
    }

    file.close();

#ifdef WITH_TINYPLY
    std::filebuf fb_binary;
    auto filename = output_folder+"/inseg_tinyply.ply";
    fb_binary.open(filename, std::ios::out | std::ios::binary);
    std::ostream outstream_binary(&fb_binary);
    if (outstream_binary.fail()) throw std::runtime_error("failed to open " + filename);
    tinyply::PlyFile plyFile;
    plyFile.add_properties_to_element("vertex", { "x", "y", "z" },
                                      tinyply::Type::FLOAT32, tinysurfels.pts.size(), reinterpret_cast<uint8_t*>(tinysurfels.pts.data()), tinyply::Type::INVALID, 0);

    plyFile.add_properties_to_element("vertex", { "label" },
                                      tinyply::Type::UINT16, tinysurfels.labels.size(), reinterpret_cast<uint8_t*>(tinysurfels.labels.data()), tinyply::Type::INVALID, 0);

    plyFile.add_properties_to_element("vertex", { "nx", "ny", "nz", "curvature" },
                                      tinyply::Type::FLOAT32, tinysurfels.normals.size(), reinterpret_cast<uint8_t*>(tinysurfels.normals.data()), tinyply::Type::INVALID, 0);

    plyFile.add_properties_to_element("vertex", { "red", "green", "blue", "alpha" },
                                      tinyply::Type::UINT8, tinysurfels.colors.size(), reinterpret_cast<uint8_t*>(tinysurfels.colors.data()), tinyply::Type::INVALID, 0);

    plyFile.add_properties_to_element("vertex", { "quality" },
                                      tinyply::Type::FLOAT32, tinysurfels.quality.size(), reinterpret_cast<uint8_t*>(tinysurfels.quality.data()), tinyply::Type::INVALID, 0);

    plyFile.add_properties_to_element("vertex", { "radius" },
                                      tinyply::Type::FLOAT32, tinysurfels.radius.size(), reinterpret_cast<uint8_t*>(tinysurfels.radius.data()), tinyply::Type::INVALID, 0);
    plyFile.get_comments().push_back("generated by tinyply 2.3");

    plyFile.write(outstream_binary, true);
#endif

}

#ifdef COMPILE_WITH_JSON
json11::Json GraphSLAM::GetSceneGraph(
    bool full,bool export_src_graph){
    std::shared_ptr<Graph> graph_ptr;
    if(export_src_graph) graph_ptr= mGraph;
    else graph_ptr = inactive_mGraph;
    std::unique_lock<std::mutex> lock(graph_ptr->mMutNode);
    json11::Json::object nodes;
    json11::Json::object edges;
    json11::Json::object colors;
    json11::Json::object kfs;
    json11::Json::object prediction;
    if(!full) {
        for (const auto &node : graph_ptr->nodes) {
            if (node.second->GetLabel() == Node::Unknown()) continue;
            nodes[std::to_string(node.first)] = node.second->GetLabel();
        }

        std::map<std::string, json11::Json::array> tmp_edges;
        for (const auto &edge : graph_ptr->edges){
            if (edge.second->GetLabel() == Edge::Same())
                if (graph_ptr->nodes.at(edge.second->nodeFrom)->GetLabel() !=
                    graph_ptr->nodes.at(edge.second->nodeTo)->GetLabel())
                    continue;
            std::string name = std::to_string(edge.second->nodeFrom) + "_" + std::to_string(edge.second->nodeTo);
//            if(edge.second->label == Edge::None());
            if (tmp_edges.find(name) == tmp_edges.end()) tmp_edges[name] = json11::Json::array();
            tmp_edges.at(name).push_back(edge.second->GetLabel());
        }
        for(auto &pair:tmp_edges){
            edges[pair.first] = pair.second;
        }
    } else {
        for (const auto &node : graph_ptr->nodes) {
            if (node.second->GetLabel() == Node::Unknown()) continue;
            json11::Json::object probs, node_json;
            if (node.second->mClsProb.empty()) continue;
            for(const auto &pair:node.second->mClsProb) {
//                if(std::isinf(log(pair.second)))
//                    SCLOG(ERROR) << "get inf from " << pair.second << " of them " << pair.first;
                probs[pair.first] = pair.second==0?0 :log(pair.second); // softmax -> logsoftmax
            }
            node_json["label"] = probs;
            node_json["time_created"] = (int)node.second->time_stamp_active;
            node_json["time_viewed"] = (int)node.second->time_stamp_viewed;
            nodes[std::to_string(node.second->instance_idx)] = node_json;
            // nodes[std::to_string(node.first)]["label"] = probs;
        }

        for(const auto &edge : graph_ptr->edges) {
            if (edge.second->GetLabel() == Edge::Same())
                if (graph_ptr->nodes.at(edge.second->nodeFrom)->GetLabel() !=
                    graph_ptr->nodes.at(edge.second->nodeTo)->GetLabel())
                    continue;
            std::string name = std::to_string(edge.second->nodeFrom) + "_" + std::to_string(edge.second->nodeTo);
            json11::Json::object probs;
            if (edge.second->labelProp.empty()) continue;
            for(auto &pair:edge.second->labelProp) {
//                if(std::isinf(log(pair.second)))
//                    SCLOG(ERROR) << "get inf from " << pair.second << " of them " << pair.first;
                probs[pair.first] = pair.second==0? 0 : log(pair.second);// softmax -> logsoftmax
            }
            edges[name] = probs;
        }
    }
    prediction["nodes"] = nodes;
    prediction["edges"] = edges;
    prediction["kfs"] = kfs;
    return prediction;
}
#endif

void GraphSLAM::SaveGraph(const std::string &output_folder, bool fullProb) {
#ifdef COMPILE_WITH_JSON
    auto prediction = GetSceneGraph(fullProb);
    std::string prediction_path = "/home/uav/prediction.json";
    std::fstream test_file(prediction_path,std::fstream::out);
    if(!test_file.is_open()) std::cout<< prediction_path <<" cannot be opened!";
    else std::cout<< prediction_path <<"opened correct!";
    test_file.close();
    

    ORUtils::JsonUtils::Dump(prediction,prediction_path);
#else
    SCLOG(WARNING) << "Did not compile with json.";
#endif
}