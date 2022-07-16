#ifndef LIBSURFELRECONSTRUCTION_DATALOADER_REALSENSE_H
#define LIBSURFELRECONSTRUCTION_DATALOADER_REALSENSE_H

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
    bool loadIntrinsic(
        const std::string file_dir, CameraParameters &intrinsics);


    const std::string GetFileName(const std::string &folder,
                                    const std::string &subfolder,
                                    const std::string &prefix,
                                    const std::string &suffix,
                                    int number_length) const;

};

}

#endif