#define STB_IMAGE_IMPLEMENTATION

#include <dataLoader/dataset_loader_facotry.h>
#include <ORUtils/Logging.h>
#include <ORUtils/PathTool.hpp>
#include "parameters.h"

#include "../../Utils/EvaluationHelper.h"
#include "../../libGraphSLAMGUI/GraphSLAMGUI.h"

using namespace std;

class Renderer : SC::GUI3D {
public:
    Renderer(int width, int height, const std::string &path, bool align):SC::GUI3D("render",width,height,false){
        std::string folder, scan_id;
        PSLAM::MeshRenderType type;
        if(path.find("scene") != std::string::npos) {
            auto parent_folder = tools::PathTool::find_parent_folder(path, 1);
            scan_id = tools::PathTool::getFileName(parent_folder);
            folder =  tools::PathTool::find_parent_folder(parent_folder, 1);
            type = PSLAM::MeshRenderType_ScanNet;
        } else {
            auto seq_folder = tools::PathTool::find_parent_folder(path,1);
            scan_id = tools::PathTool::getFileName(seq_folder);
            folder = tools::PathTool::find_parent_folder(seq_folder,1);
            type = PSLAM::MeshRenderType_3RScan;
        }
        mMeshRenderer.reset( PSLAM::MakeMeshRenderer(width, height, folder,scan_id,type,align) );
    }

    void Render(cv::Mat &rgb, cv::Mat &depth, const Eigen::Matrix4f &pose, const PSLAM::CameraParameters &params){
        auto view_pose = glUtil::GetViewMatrix(pose);
        auto proj = glUtil::Perspective<float>(params.fx,params.fy,
                                               params.cx,params.cy,
                                               params.width,params.height,
                                               glCam->projection_control_->near,glCam->projection_control_->far);
        mMeshRenderer->Render(proj,view_pose,glCam->projection_control_->near,glCam->projection_control_->far);
        depth = mMeshRenderer->GetDepth();
    }
private:
    std::unique_ptr<PSLAM::MeshRendererInterface> mMeshRenderer;
};

int main(int argc, char** argv) 
{
    Params params;
    string path;
    PSLAM::ConfigPSLAM config;
    shared_ptr<PSLAM::DatasetLoader_base> dataset_loader_;
    cv::Mat depth_img, rgb_img;

    string src_model_dir;

    {
        ParseCommondLine(argc, argv, params);

        config = getConfig(params);
        path = params.pth_in;
        src_model_dir = tools::PathTool::find_parent_folder(path,1)+"/output/node_RGB.ply";
        cout<<"Source model: "<< src_model_dir<<"\n";

        dataset_loader_.reset(PSLAM::DataLoaderFactory::Make(path));
        dataset_loader_->Reset();
        if (path.find("scene") == std::string::npos) dataset_loader_->GetCamParamDepth() = dataset_loader_->GetCamParamRGB();

        // std::unique_ptr<Renderer> renderer;
        // if(params.use_render) {
        //     SCLOG(INFO) << "Building renderer...";
        //     renderer = std::make_unique<Renderer>(dataset_loader_->GetCamParamDepth().width,
        //                                         dataset_loader_->GetCamParamDepth().height,
        //                                         path, true
        //     );
        // }

        // depth_img = dataset_loader_->GetDepthImage();
        // rgb_img = dataset_loader_->GetRGBImage();
        
        // auto camDepth = dataset_loader_->GetCamParamDepth();
        // std::cout
        //     <<camDepth.width<<","<<camDepth.height<<","
        //     <<camDepth.cx<<","<<camDepth.cy<<"\n";
    }

    SCLOG(INFO) << "Building framework...";

    PSLAM::GraphSLAM graphSlam(&config, dataset_loader_->GetCamParamDepth());

    SCLOG(INFO) << "LAUNCH gui...";
    PSLAM::GraphSLAMGUI gui(&graphSlam, dataset_loader_.get(),true);
    gui.setGraphMatchMode(src_model_dir,src_model_dir);
    gui.SetRender(dataset_loader_->GetCamParamDepth().width,dataset_loader_->GetCamParamDepth().height,path, true);
    gui.run();

    SCLOG(INFO) << "Processing start ...";

    return 0;
}