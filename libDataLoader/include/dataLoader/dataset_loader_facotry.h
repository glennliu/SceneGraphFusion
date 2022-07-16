#pragma once
#include "dataset3RScan.h"
#include "datasetScanNet.h"
#include "datasetSceneNN.h"
#include "datasetRealSense.h"
#include "dataloader_3rscan.h"
#include "dataloader_scannet.h"
#include "dataloader_scenenn.h"
#include "dataloader_realsense.h"

namespace PSLAM {
    struct  DataLoaderFactory {
        static DatasetLoader_base *Make(
                const std::string &pth, INPUTE_TYPE inputeType = DATASET_DETECT) {
            DatasetLoader_base *output = nullptr;
            // detect datatype by checking file name

            if(inputeType == DATASET_DETECT){
                std::cerr << "detect data type: ";
                // if(pth.find(".sens") != std::string::npos || pth.find("scene") != std::string::npos) {
                if(pth.find("ScanNet") != std::string::npos) {
                    inputeType = DATASET_SCANNET;
                    std::cerr << "ScanNet\n";
                }
                else if(pth.find("realsense")!=std::string::npos){
                    inputeType = DATASET_REALSENSE;
                    std::cout<<"data type: Realsense!\n";
                } 
                else {
                    inputeType = DATASET_3RSCAN;
                    std::cerr << "3RScan";
                }
            }
                

            // Create dataloader
            switch (inputeType) {
                case DATASET_3RSCAN: {
                    auto path = pth.back() == '/'? pth : pth+"/";
                    auto database = std::make_shared<Scan3RDataset>(inputeType, path);
                    output = new DatasetLoader_3RScan(database);
                    break;
                }
                case DATASET_SCANNET: {
                    auto database = std::make_shared<ScanNetDataset>(inputeType, pth);
                    output = new DatasetLoader_ScanNet(database);
                    break;
                }
                case DATASET_SCENENN:{
                    auto database = std::make_shared<SceneNNDataset>(inputeType,pth);
                    output = new DatasetLoader_SceneNN(database);
                    break;
                }
                case DATASET_REALSENSE:{
                    auto database = std::make_shared<RealsenseDataset>(inputeType,pth);
                    output = new Datasetloader_Realsense(database);
                    break;
                }
                case DATASET_DETECT:
                    break;

            }
            return output;
        }
    };
}