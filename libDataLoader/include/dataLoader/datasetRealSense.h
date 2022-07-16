#ifndef LIBSURFELRECONSTRUCTION_DATASETRS_H
#define LIBSURFELRECONSTRUCTION_DATASETRS_H

#include "dataset_base.h"

namespace PSLAM
{

class RealsenseDataset: public DatasetDefinitionBase
{
public:
    RealsenseDataset(INPUTE_TYPE type, const std::string &path){
            datasetType = type;
            folder = path;
            // frame_index_counter = 1;
            number_length = 1;
            prefix_pose = "/pose_orb/camera_pose_";
            prefix_depth = "/depth/depth";
            prefix_rgb = "/rgb/image";

            suffix_depth = ".png";
            suffix_rgb = ".png";
            suffix_pose = ".txt";

            min_pyr_level = 3;
            number_pose = 6;
            number_length = 5;
    }
    
};

}

#endif