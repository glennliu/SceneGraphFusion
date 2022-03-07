//
// Created by sc on 1/13/21.
//

#ifndef LIBSURFELRECONSTRUCTION_DATASETSCENENN_H
#define LIBSURFELRECONSTRUCTION_DATASETSCENENN_H

#include "dataset_base.h"

namespace PSLAM {
    class SceneNNDataset : public DatasetDefinitionBase {
    public:
        SceneNNDataset(INPUTE_TYPE type, const std::string &path)  {
            datasetType = type;
            folder = path;
            frame_index_counter = 1;
            number_length = 1;
            prefix_pose = "/pose/camera_pose_";
            prefix_depth = "/depth/depth";
            prefix_rgb = "/image/image";

            suffix_depth = ".png";
            suffix_rgb = ".png";
            suffix_pose = ".txt";
            if (rotate_pose_img) {
                suffix_depth = ".png";
                suffix_pose = ".txt";
            }

            min_pyr_level = 3;
            number_pose = 6;
            number_length = 5;
        }
        bool use_aligned_pose = true;
    };
}

#endif //LIBSURFELRECONSTRUCTION_DATASETSCENENN_H
