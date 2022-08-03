#pragma once
#ifdef COMPILE_WITH_GRAPHPRED
#include "MemoryBlock.h"
#endif
#include <memory>
#include <map>
#include <mutex>
#include <Eigen/Dense>

namespace PSLAM {
    class Edge { // Directed Edge
    public:
        inline static std::string None() {return "none";}
        inline static std::string Same() {return "same part";}
        Edge() = default;
        Edge(Edge &edge);

        int nodeFrom{}, nodeTo{};
        /// Shared
        std::map<std::string, float> labelProp;
        std::map<std::string, float> mClsWeight;
        /// Thread_SG
#ifdef COMPILE_WITH_GRAPHPRED
        std::map<std::string, std::shared_ptr<MemoryBlock>> mFeatures;
#endif
        mutable std::mutex mMutex, mMutLabel;

        std::string GetLabel() const;
        void UpdatePrediction(const std::map<std::string, float> &prop, bool fusion);

        std::string label = None();

        bool consistent = false;
        std::string node_to_label;
        Eigen::Vector3f edge_vec;
        float distance = 0.0f;
    };
    typedef std::shared_ptr<Edge> EdgePtr;
    static inline EdgePtr MakeEdgePtr(){return std::make_shared<Edge>();}
}
