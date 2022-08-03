#ifndef LIBSURFELRECONSTRUCTION_DATALOADER_REALSENSE_H
#define LIBSURFELRECONSTRUCTION_DATALOADER_REALSENSE_H

#include <map>
#include "dataset_loader.h"

namespace PSLAM
{
class Datasetloader_Realsense:public DatasetLoader_base
{
public:
    Datasetloader_Realsense(std::shared_ptr<DatasetDefinitionBase> dataset);
    void Reset() override;
    bool Retrieve() override;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    struct FrameData{
        double ts;
        std::string rgb_filename;
        std::string depth_filename;
    };

    bool loadIntrinsic(
        const std::string file_dir);
        // CameraParameters &rgb_intrinsics,CameraParameters &depth_intrinsics);

    bool loadAssociateFile(const std::string &file_dir);

    bool loadCameraPose(const std::string &file_dir);

    const std::string GetFileName(const std::string &folder,
                                    const std::string &subfolder,
                                    const std::string &prefix,
                                    const std::string &suffix,
                                    int number_length) const;
    std::vector<FrameData> associated_frames;
    std::vector<Eigen::Matrix4f> camera_poses;
    std::map<std::string,FrameData> associated_frames_map;
    

};

}

#endif