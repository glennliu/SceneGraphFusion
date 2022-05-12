#define STB_IMAGE_IMPLEMENTATION
#ifdef __APPLE__
/* Defined before OpenGL and GLUT includes to avoid deprecation messages */
#define GL_SILENCE_DEPRECATION
#endif
#include <iostream>
#include <dataLoader/dataset_loader_facotry.h>
#include "../../Utils/parser.hpp"
#include <ORUtils/Logging.h>
#include <ORUtils/PathTool.hpp>
#include <chrono>
#include "parameters.h"

#ifdef COMPILE_WITH_PSLAM_GUI
#include "../../libGraphSLAMGUI/GraphSLAMGUI.h"
#include "../../Utils/EvaluationHelper.h"
#else
#include "../../libGraphSLAM/GraphSLAM.h"
#include "../Utils/EvaluationHelper.h"
#ifdef COMPILE_WITH_ASSIMP
#include "../libGUI3D/libGUI3D/GUI3D.h"
#include "../../renderer/RendererFactory.h"
#endif
#endif



class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc_ms()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        start = std::chrono::system_clock::now();
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

#ifndef COMPILE_WITH_PSLAM_GUI
#ifdef COMPILE_WITH_ASSIMP
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
#else
class Renderer{
public:

    Renderer(int width, int height, const std::string &path){
        throw std::runtime_error("did not compile with assimp");
    }

    void Render(cv::Mat &rgb, cv::Mat &depth, const Eigen::Matrix4f &pose, const PSLAM::CameraParameters &params){
        throw std::runtime_error("did not compile with assimp");
    }
};
#endif
#endif

int main(int argc, char** argv) {
    Params params;
    ParseCommondLine(argc, argv, params);
#ifdef NDEBUG
    if(params.verbose) SCLOG_ON(VERBOSE);
    else SCLOG_ON(INFO);
#else
    SCLOG_ON(VERBOSE);
#endif

    if (params.save) tools::PathTool::create_folder(params.pth_out);
    auto path = params.dataset_dir+"/"+params.scan_name+"/sequence";//params.pth_in;
    if (path.find(".txt") != std::string::npos) {
        std::ifstream file(path);
        assert(file.is_open());
        std::getline(file, path, '\n');
    }
    SCLOG(INFO) << "Buliding data loader...";
    std::shared_ptr<PSLAM::DatasetLoader_base> dataset_loader_;
    dataset_loader_.reset(PSLAM::DataLoaderFactory::Make(path));
    dataset_loader_->Reset();

    if(params.use_render) {
        if (path.find("scene") == std::string::npos) dataset_loader_->GetCamParamDepth() = dataset_loader_->GetCamParamRGB();
    }

#ifndef COMPILE_WITH_PSLAM_GUI
    std::unique_ptr<Renderer> renderer;
    if(params.use_render) {
        SCLOG(INFO) << "Building renderer...";
        renderer = std::make_unique<Renderer>(dataset_loader_->GetCamParamDepth().width,
                                              dataset_loader_->GetCamParamDepth().height,
                                              path, true
        );
    }
#endif

    auto depth_img = dataset_loader_->GetDepthImage();
    auto rgb_img = dataset_loader_->GetRGBImage();
    
    SCLOG(INFO) << "Building framework...";
    PSLAM::ConfigPSLAM configPslam;

    
    configPslam = getConfig(params);
    PSLAM::GraphSLAM graphSlam(&configPslam, dataset_loader_->GetCamParamDepth());


#ifdef COMPILE_WITH_PSLAM_GUI
    SCLOG(INFO) << "start gui...";
    PSLAM::GraphSLAMGUI gui(&graphSlam, dataset_loader_.get());
    // std::cout<<dataset_loader_->get();
    if(params.use_render) gui.SetRender(dataset_loader_->GetCamParamDepth().width,dataset_loader_->GetCamParamDepth().height,path, true);
    gui.run();
#else
    SCLOG(INFO) << "start processing frames...";
    while (true) {
        if (!dataset_loader_->Retrieve())break;
        TicToc timer;
        SCLOG(VERBOSE) << "process frame: " << dataset_loader_->GetFrameIndex();
        const Eigen::Matrix4f pose = dataset_loader_->GetPose().inverse();
        auto rgb = dataset_loader_->GetRGBImage();
        auto d   = dataset_loader_->GetDepthImage();
        std::cout<<"loader "<< timer.toc_ms()<<"ms, ";

        if(renderer) {
            auto t_p = dataset_loader_->GetPose();
            t_p.topRightCorner<3, 1>() /= 1000.f;
            t_p.transposeInPlace();
            cv::Mat t_rgb;
            renderer->Render(t_rgb,d,t_p,dataset_loader_->GetCamParamDepth());
        }
        std::cout<<"render "<< timer.toc_ms()<<"ms, ";

        CTICK("[ALL]0.all");
        graphSlam.ProcessFrame(dataset_loader_->GetFrameIndex(), rgb, d, &pose);
        CTOCK("[ALL]0.all");
        std::cout<<"slam "<< timer.toc_ms()<<"ms\n";


    }
    SCLOG(INFO) << "frame processing finished.";
#endif
    if(params.save) {
        graphSlam.Stop();

        SCLOG(INFO) << "saving results.";
        if(params.save_graph_ply) {
            auto colorMode = params.use_predict? PSLAM::GraphSLAM::SAVECOLORMODE_SEMANTIC : PSLAM::GraphSLAM::SAVECOLORMODE_RGB;
            if(params.use_predict)
                graphSlam.SaveNodesToPLY(params.segment_filter, params.pth_out, colorMode,
                                     params.binary);
        }

        if(params.save_surfel_ply)
            graphSlam.SaveSurfelsToPLY(params.pth_out,params.save_name,graphSlam.GetInSeg()->map().surfels,params.binary);

        if(params.save_graph) {
            auto scan_id = tools::PathTool::getFileName(tools::PathTool::find_parent_folder(path, 1));
            auto predictions = graphSlam.GetSceneGraph(params.full_prop);
            json11::Json::object json;
            json[scan_id] = predictions;
            ORUtils::JsonUtils::Dump(json, params.pth_out + "/predictions.json");
        }

        if(params.save_time) {
            SCSLAM::EVALUATION::Logging::printResultToFile(params.pth_out, "times.txt");
#ifdef COMPILE_WITH_GRAPHPRED
            if(graphSlam.GetGraphPred()) {
                const auto &times = graphSlam.GetGraphPred()->GetTimes();
                for (const auto &pair: times) {
                    std::fstream file(params.pth_out + "/times_graph_" + pair.first + ".txt", std::ios::out);
                    file << pair.first << "\n";
                    for (const auto &pp : pair.second)
                        file << pp.first << "," << pp.second << "\n";
                    file << "\n";
                    file.close();
                }
            }
#endif
        }
    }


    return 0;
}
