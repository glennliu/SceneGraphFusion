//
// Created by sc on 8/21/20.
//

#ifndef GRAPHSLAM_GRAPHSLAM_H
#define GRAPHSLAM_GRAPHSLAM_H
#include "config.h"
#include "disjSet.h"
#include "graph.h"
#include "../Objects/Camera/CameraParameters.h"
#include <inseg_lib/lib.h>
#include <ORUtils/Logging.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <future>
#include <chrono>

#ifdef COMPILE_WITH_GRAPHPRED
#include "GraphPredictor.h"
#else
#include <ORUtils/JsonUtil.h>
#endif
namespace PSLAM {

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

    class GraphSLAM {
    public:
        explicit GraphSLAM(ConfigPSLAM *config, const CameraParameters &camParamD);
        ~GraphSLAM();

        //////////
        /// Frameprocessing
        //////////
        bool Initialize(const CameraParameters &camParamD);

        void ProcessFrame(int idx, const cv::Mat &colorImage, const cv::Mat &depthImage, const Eigen::Matrix4f *pose);

        bool &UseThread(){return mConfig->use_thread;}

        //////////
        /// Access
        //////////
        inseg_lib::InSegLib *GetInSeg() { return inseg_.get(); }

        Graph *GetGraph() { return mGraph.get(); }

        Graph *GetInactiveGraph() {return inactive_mGraph.get();}

        ConfigPSLAM *GetConfig(){return mConfig;}


#ifdef COMPILE_WITH_GRAPHPRED
        GraphPredictor *GetGraphPred() { return mpGraphPredictor.get(); }
#endif

#ifdef COMPILE_WITH_JSON
        json11::Json GetSceneGraph(bool full, bool export_src_graph = true);
#endif

        //////////
        /// IO
        //////////
        void SaveModel(const std::string &output_folder) const;

        void SaveGraph(const std::string &output_folder, bool fullProb);

        void SaveSurfelsToPLY(int segment_filter, const std::string &output_dir, const std::string &output_name, bool binary);

        enum SAVECOLORMODE {
            SAVECOLORMODE_RGB, SAVECOLORMODE_SEGMENT, SAVECOLORMODE_INSTANCE, SAVECOLORMODE_SEMANTIC, SAVECOLORMODEL_PANOPTIC
        };

        void SaveNodesToPLY(int segment_filter, const std::string &output_dir, 
                SAVECOLORMODE saveColorMode, 
                bool binary=false, bool inactive_graph_=false);

        void SaveSurfelsToPLY(const std::string &output_folder, const std::string &output_name,
                              const std::vector<std::shared_ptr<inseg_lib::Surfel>> &surfels, bool binary);
        
        //////////
        /// Graph
        //////////
        void RunFullPrediction();
        /// Start backend
        void Start();
        /// Stop backend
        void Stop();

        // Update the selected nodes
        void AddSelectedNodeToUpdate(int idx);

        // void removeInactiveNodes(const int &idx);

        bool LoadPredictModel();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::set<int> mLastUpdatedSegments{};
    private:
        // system
        size_t mTimeStamp=0;
        bool mbInitMap = false;

        Eigen::Matrix4f pose_;
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> trajectory_{};

        // Inseg Standard configuration.
        std::shared_ptr<inseg_lib::InSegLib> inseg_;

        // Graph
        ConfigPSLAM *mConfig;
        std::shared_ptr<Graph> mGraph;
        std::shared_ptr<Graph> inactive_mGraph;
        // std::shared_ptr<Graph> activeGraph;
#ifdef COMPILE_WITH_GRAPHPRED
        GraphPredictorPtr mpGraphPredictor;
#endif

        std::vector<std::shared_ptr<inseg_lib::Surfel>> GetUpdatedSurfels();

        std::vector<std::shared_ptr<inseg_lib::Surfel>> FilterSegment(
                int segment_filter, const std::vector<std::shared_ptr<inseg_lib::Surfel>> &surfels);

        int debug_index = 0;

        void transitInactiveNodes(const size_t &timestamp);

    };
}

#endif //GRAPHSLAM_GRAPHSLAM_H
