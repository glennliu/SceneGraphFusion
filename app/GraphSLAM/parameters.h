#include <iostream>

#include "../../Utils/parser.hpp"
#include "../../libGraphSLAM/config.h"

struct Params{
    std::string pth_in;
    std::string pth_out = "/home/uav/lch_ws/code_ws/SceneGraph/3RScan/output";
    std::string dataset_dir;
    std::string scan_name;
    std::string pth_model;
    std::string save_name = "inseg.ply";
    int min_pyr_level=2;
    int inactive_frames = 200;

    float depth_edge_threshold = -1; // -1: use default.

    /// Use rendered view from a given mesh (for ScanNet)
    bool use_render=true;
    bool save=true;
    bool save_graph=true;
    bool save_graph_ply = true;
    bool save_surfel_ply = true;
    bool save_time = true;

    /// Predict semantic scene graph
    bool use_predict=true;
    bool full_prop=true;
    bool binary=false;
    bool fusion=true;
    ///
    bool thread=true;


    bool verbose=false;
    int segment_filter=512;
};

void ParseCommondLine(int argc, char **argv, Params &params) {
    tools::Parser parser(argc,argv);
    parser.addOption(pkgcname("pth_in", &params.pth_in), "", false);
    parser.addOption(pkgcname("dataset_dir",&params.dataset_dir),"",true);
    parser.addOption(pkgcname("scan_name",&params.scan_name),"",true);
    parser.addOption(pkgcname("pth_out", &params.pth_out),"");
    parser.addOption(pkgcname("pth_model", &params.pth_model),"path to the folder contains models",false);

    parser.addOption(pkgcname("save_name", &params.save_name),"");

    parser.addOption(pkgcname("min_pyr_level", &params.min_pyr_level),
                     "which level to run InSeg");
    parser.addOption(pkgcname("segment_filter", &params.segment_filter),
                     "Filter out segment that has not enough surfels.");
    parser.addOption(pkgcname("depth_edge_threshold", &params.depth_edge_threshold),
                     "depth_edge_threshold for InSeg");
    parser.addOption(pkgcname("inactive_thre", &params.inactive_frames),"Select inactive nodes");

    parser.addOption(pkgcname("rendered", &params.use_render), "use rendered depth");
    parser.addOption(pkgcname("full_prop", &params.full_prop), "return full prop or just the maximum.");
    parser.addOption(pkgcname("fusion", &params.fusion), "use fusion.");
    parser.addOption(pkgcname("binary", &params.binary), "save ply to binary.");
    parser.addOption(pkgcname("thread", &params.thread), "use thread on scene graph prediction.");
    parser.addOption(pkgcname("verbose", &params.verbose), "verbose.");
    parser.addOption(pkgcname("prediction", &params.use_predict), "do graph prediction.");

    parser.addOption(pkgcname("save", &params.save), "");
    parser.addOption(pkgcname("save_graph", &params.save_graph), "");
    parser.addOption(pkgcname("save_graph_ply", &params.save_graph_ply), "output graph as ply (with semantic id)");
    parser.addOption(pkgcname("save_surfel_ply", &params.save_surfel_ply), "output surfels to ply (inseg output)");
    parser.addOption(pkgcname("save_time", &params.save_time), "");

    auto status = parser.showMsg(params.verbose);
    if(status<1)
        exit(-1);
    if(status==2)
        exit(0);
}

PSLAM::ConfigPSLAM getConfig(const Params &params)
{
    PSLAM::ConfigPSLAM configPslam;
    /// Adjust configurations according to dataset and level
    if(params.pth_model.empty())
        configPslam.graph_predict = false;
    else
    {
        configPslam.pth_model = params.pth_model;
        configPslam.use_fusion = params.fusion;
        configPslam.use_thread = params.thread;
        configPslam.filter_num_node = params.segment_filter;
        configPslam.graph_predict=params.use_predict;
        configPslam.main_config.max_pyr_level = 4;
        configPslam.main_config.min_pyr_level = params.min_pyr_level;
        configPslam.inactive_frames_threshold = params.inactive_frames;

        if(params.depth_edge_threshold == -1) {
            std::cout<< "use predefined depth edge threshold: " << params.depth_edge_threshold;
            if (configPslam.main_config.min_pyr_level == 1)
                configPslam.inseg_config.depth_edge_threshold = 0.99f;
            else if (configPslam.main_config.min_pyr_level == 2)
                configPslam.inseg_config.depth_edge_threshold = 0.99f;
            else if (configPslam.main_config.min_pyr_level == 3)
                configPslam.inseg_config.depth_edge_threshold = 0.98f;
        } else {
            std::cout << "use custom depth edge threshold: " << params.depth_edge_threshold;
            configPslam.inseg_config.depth_edge_threshold = params.depth_edge_threshold;
        }
    }
    return configPslam;
}

