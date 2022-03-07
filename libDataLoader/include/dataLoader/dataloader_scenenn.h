//
// Created by sc on 1/13/21.
//

#ifndef LIBSURFELRECONSTRUCTION_DATALOADER_SCENENN_H
#define LIBSURFELRECONSTRUCTION_DATALOADER_SCENENN_H
#include "dataset_loader.h"
namespace PSLAM {
    class DatasetLoader_SceneNN : public DatasetLoader_base {
    public:
        DatasetLoader_SceneNN(std::shared_ptr<DatasetDefinitionBase> dataset);
        void Reset() override;
        bool Retrieve() override;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        std::string pose_file_name_ = "";

        Eigen::Matrix<float, 4, 4> rotation_matrix_Z(const float rot = M_PI);

        const std::string GetFileName(const std::string &folder,
                                      const std::string &subfolder,
                                      const std::string &prefix,
                                      const std::string &suffix,
                                      int number_length) const;
        bool IsV2() const;
    };
}

#endif //LIBSURFELRECONSTRUCTION_DATALOADER_SCENENN_H
