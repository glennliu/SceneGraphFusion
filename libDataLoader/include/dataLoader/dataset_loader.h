#ifndef _H_INC_DATASET_LOADER_
#define _H_INC_DATASET_LOADER_

#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <string>
#include "dataset_base.h"
#include "CameraParameters.h"

namespace PSLAM {
    class DatasetLoader_base {
    public:
        explicit DatasetLoader_base(std::shared_ptr<DatasetDefinitionBase> dataset):
                m_dataset(std::move(dataset)){
        }
        virtual ~DatasetLoader_base() = default;

        virtual void Reset() = 0;
        virtual bool Retrieve() = 0;


        [[nodiscard]] const cv::Mat& GetRGBImage() const { return m_rgb; }
        [[nodiscard]] const cv::Mat& GetDepthImage() const { return m_d; }
        [[nodiscard]] const Eigen::Matrix4f& GetPose() const { return m_pose; }
        [[nodiscard]] const CameraParameters &GetCamParamDepth() const { return m_cam_param_d; }
        [[nodiscard]] const CameraParameters &GetCamParamRGB() const { return m_cam_param_rgb; }
        [[nodiscard]] CameraParameters &GetCamParamDepth()  { return m_cam_param_d; }
        [[nodiscard]] CameraParameters &GetCamParamRGB()  { return m_cam_param_rgb; }
        const DatasetDefinitionBase * GetDataBase(){return m_dataset.get();}
        const Eigen::Matrix4f GetDriftedPose() const { 
            return accumulated_drift * m_pose; 
        }
        const cv::Mat& getRenderDepth()const{
            return m_d_render;
        }
        const bool isFinished()const{
            if(frame_index<frame_index_max)return false;
            else return true;
        }

        void SetFrameIndex(int idx) {frame_index = idx;}

        // Sigma yaw is in degree unit.
        void setFramewiseDrift(const float sigma_xy, const float sigma_z,
            const float sigma_yaw){
            const float sigma_yaw_radian = sigma_yaw * M_PI /180;
            drift<<
                cos(sigma_yaw_radian),-sin(sigma_yaw_radian),0,sigma_xy,
                sin(sigma_yaw_radian),cos(sigma_yaw_radian),0,sigma_xy,
                0,0,1,sigma_z,
                0,0,0,1;
            std::cout<<"Framewise pose drift is set to \n"<< drift<<"\n";
        }

        void printCamParam(bool depth = true){
            CameraParameters *cam_to_print;
            if(depth){
                cam_to_print = &m_cam_param_d;
                std::cout<<"DepthParams\n";
                }
            else {
                std::cout<<"RgbParams\n";
                cam_to_print = &m_cam_param_rgb;
            }
            std::cout
                <<"--width: "<<cam_to_print->width<<"\n"
                <<"--height: "<<cam_to_print->height<<"\n"
                <<"--fx: "<<cam_to_print->fx<<"\n"
                <<"--fy: "<<cam_to_print->fy<<"\n"
                <<"--cx: "<<cam_to_print->cx<<"\n"
                <<"--cy: "<<cam_to_print->cy<<"\n";
                // <<"--scale: "<<cam_to_print->scale<<"\n";
        }

        const int& GetFrameIndex() const { return frame_index; }

        const int& GetFinalIndex() const { return frame_index_max; }

        const DatasetDefinitionBase* GetBase() const {return m_dataset.get(); }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
        std::shared_ptr<DatasetDefinitionBase> m_dataset;
        int frame_index = 0, frame_index_max = 0;
        // int total_frames_num = 0;
        cv::Mat m_rgb, m_d;
        cv::Mat m_d_render;
        Eigen::Matrix4f m_pose;
        CameraParameters m_cam_param_rgb, m_cam_param_d;

        // 4Dof drift [x y height yaw]; 
        // x,y,z are in unit of mm; yaw is in the unit of degree.
        // In each incoming keyframe, the drift is incorporated.
        Eigen::Matrix4f drift = Eigen::Matrix4f::Identity();

        Eigen::Matrix4f accumulated_drift = Eigen::Matrix4f::Identity(); 

    };





}
#endif
