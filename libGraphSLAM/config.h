//
// Created by sc on 10/9/20.
//

#ifndef GRAPHSLAM_CONFIG_H
#define GRAPHSLAM_CONFIG_H
#include <string>
#include <fstream>
#include <sstream>
#include <inseg_lib/inseg_config.h>
#include <inseg_lib/main_config.h>
#include <inseg_lib/map_config.h>
#include <inseg_lib/segmentation_config.h>
//#include <inseg_lib/segmentation.h>
namespace PSLAM {
    struct ConfigPSLAM {
        ConfigPSLAM();
        explicit ConfigPSLAM(const std::string &path);
        ~ConfigPSLAM();
        bool use_thread;

        bool use_fusion;

        /// Use graph predict or not
        bool graph_predict;

        /// model path
        std::string pth_model;

        int filter_num_node;

//        int predict_when_have_at_least_n_node;
        size_t n_pts;

        float neighbor_margin; // mm

        /// Only update the information of a node if the node has size change to N percent
        float update_thres_node_size;
        /// Update a node if its time stamp is older than n
        int update_thres_time_stamp;
        /// Set the node as inactive if its timestamp is older than n
        int inactive_frames_threshold;
        /// Set the node as inactive if it has been labelled as active larger than the threshold
        int active_frames_threshold;

        std::string pth;


        inseg_lib::InSegConfig inseg_config;
        inseg_lib::MainConfig main_config;
        inseg_lib::MapConfig map_config;
        inseg_lib::SegmentationConfig segmentation_config;
    
        std::string print(){
            std::stringstream str;
            str<<"=========== PSLAM Parameters ============\n"
                <<"ConfigPSLAM\n"
                <<"--pth_model: "<<pth_model<<"\n"
                <<"--use_thread: "<<use_thread<<"\n"
                <<"--use_fusion: "<<use_fusion<<"\n"
                <<"--graph_predict: "<<graph_predict<<"\n"
                <<"--filter_num_node: "<<filter_num_node<<"\n"
                <<"--n_pts: "<<n_pts<<"\n"
                <<"--neighbor_margin: "<<neighbor_margin<<"\n"
                <<"--update_thres_time_stamp: "<<update_thres_time_stamp<<"\n"
                <<"--inactive_frames_threshold: "<<inactive_frames_threshold<<"\n"
                <<"--active_frames_threshold: "<<active_frames_threshold<<"\n"
                <<"--pth: "<<pth<<"\n";
            str<<"MainConfig\n"
                <<"--min_pyr_level: "<<main_config.min_pyr_level<<"\n"
                <<"--max_pyr_level: "<<main_config.max_pyr_level<<"\n";
            str<<"InsegLib Params\n"
                <<"--depth_edge_threshold: "<<inseg_config.depth_edge_threshold<<"\n";

            return str.str();
        }
    };


}

#endif //GRAPHSLAM_CONFIG_H
