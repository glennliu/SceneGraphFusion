//
// Created by sc on 1/13/21.
//
#include "../include/dataLoader/dataloader_scenenn.h"
#include "../include/dataLoader/util.h"
#include "../include/dataLoader/Scan3R_json_loader.h"
#include <ORUtils/Logging.h>

#include <utility>
// #include <dataLoader/dataset3RScan.h>
#include <ORUtils/PathTool.hpp>

using namespace PSLAM;

static const std::vector<std::string> split(const std::string s, const std::string delim) {
    std::vector<std::string> list;
    auto start = 0U;
    auto end = s.find(delim);
    while (true) {
        list.push_back(s.substr(start, end - start));
        if (end == std::string::npos)
            break;
        start = end + delim.length();
        end = s.find(delim, start);
    }
    return list;
}

static bool LoadInfoIntrinsics(const std::string& filename,
                                //   const bool depth_intrinsics,
                        CameraParameters& intrinsics) {
    // const std::string search_tag = ;//depth_intrinsics ? "m_calibrationDepthIntrinsic" : "m_calibrationColorIntrinsic";
    const std::string search_tag_w = "depth_width";//depth_intrinsics? "m_depthWidth":"m_colorWidth";
    const std::string search_tag_h = "depth_height";//depth_intrinsics? "m_depthHeight":"m_colorHeight";
    const std::string search_tag_fx = "fx";
    const std::string search_tag_fy = "fy";
    const std::string search_tag_cx = "cx";
    const std::string search_tag_cy = "cy";


    std::string line{""};
    std::ifstream file(filename);
    int width,height;
    float fx,fy,cx,cy;
    if (file.is_open()) {
        while (std::getline(file,line)) {
            if (line.rfind(search_tag_w, 0) == 0){
                const auto parts = split(line," ");
                width = std::stoi(parts[1]);
            }
            else if (line.rfind(search_tag_h, 0) == 0){
                const auto parts = split(line," ");
                height = std::stoi(parts[1]);
            }
            else if(line.rfind(search_tag_fx,0) == 0){
                const auto parts = split(line," ");
                fx = std::stoi(parts[1]);
            }
            else if(line.rfind(search_tag_fy,0) == 0){
                const auto parts = split(line," ");
                fy = std::stoi(parts[1]);
            }
            else if(line.rfind(search_tag_cx,0) == 0){
                const auto parts = split(line," ");
                cx = std::stoi(parts[1]);
            }
            else if(line.rfind(search_tag_cy,0) == 0){
                const auto parts = split(line," ");
                cy = std::stoi(parts[1]);
            }
        }

        file.close();
        intrinsics.Set(width,height,fx,fy,cx,cy,1.f);
        std::cout<<"=============Intrinsics=============\n"
            <<"width:"<<width<<"\n"
            <<"height:"<<height<<"\n"
            <<"fx:"<<fx<<","
            <<"fy:"<<fy<<","
            <<"cx:"<<cx<<","
            <<"cy:"<<cy<<"\n";

        return true;
    }

    return false;
}

DatasetLoader_SceneNN::DatasetLoader_SceneNN(std::shared_ptr<DatasetDefinitionBase> dataset):
        DatasetLoader_base(std::move(dataset)) {


    if(!LoadInfoIntrinsics(m_dataset->folder+"/intrinsic.txt",m_cam_param_d))
        throw std::runtime_error("unable to open _info file");

    // frame_index = 1;
   /*
    if(reinterpret_cast<PSLAM::Scan3RDataset*>(m_dataset.get())->use_aligned_pose) {
        auto seq_folder = tools::PathTool::find_parent_folder(m_dataset->folder,1);
        auto seq_name = tools::PathTool::getFileName(seq_folder);
        auto data_folder = tools::PathTool::find_parent_folder(seq_folder,1);
        auto scan3rLoader = PSLAM::io::Scan3RLoader(data_folder+"/3RScan.json");
        if(scan3rLoader.IsRescan(seq_name)) { // find ref scan ID
            auto ref_id = scan3rLoader.rescanToReference.at(seq_name);
            m_poseTransform = scan3rLoader.scaninfos.at(ref_id)->rescans.at(seq_name)->transformation;
            m_poseTransform.topRightCorner<3,1>()*=1e3;
        }
    }
    */
    
    // std::cout<<GetFileName()<<"\n";
//    return;
}

const std::string DatasetLoader_SceneNN::GetFileName(const std::string& folder,
                                                    const std::string& subfolder,
                                                    const std::string& prefix,
                                                    const std::string& suffix,
                                                    int number_length = -1) const {
    std::stringstream filename;
    const std::string path = folder + subfolder + prefix;
                            // (folder == "/" ? "" : folder) +
                            //  (subfolder == "/" ? "" : subfolder) +
                            //  (prefix == "/" ? "" : prefix);
    if (number_length < 0)
        filename << path << (suffix == "/" ? "" : suffix);
    else
        filename << path << std::setfill('0') << std::setw(number_length) << frame_index+1 << (suffix == "/" ? "" : suffix);
    std::string s(filename.str());
    return s;
}

bool DatasetLoader_SceneNN::IsV2() const {
    return  m_dataset->rotate_pose_img;
}

bool DatasetLoader_SceneNN::Retrieve() {
    // if(frame_index >300) return false;
    std::string depthFilename = GetFileName(m_dataset->folder,
                                            m_dataset->folder_depth,
                                            m_dataset->prefix_depth,
                                            m_dataset->suffix_depth,
                                            m_dataset->number_length);
    std::string colorFilename = GetFileName(m_dataset->folder,
                                            m_dataset->folder_rgb,
                                            m_dataset->prefix_rgb,
                                            m_dataset->suffix_rgb,
                                            m_dataset->number_length);

    std::string pose_file_name_ = GetFileName(m_dataset->folder,
                                  m_dataset->folder_pose,
                                  m_dataset->prefix_pose,
                                  m_dataset->suffix_pose,
                                  m_dataset->number_length);
    std::cout<< depthFilename <<"\n"
        <<pose_file_name_<<"\n";
        // << colorFilename <<"\n";                         
    bool isExist = isFileExist(depthFilename);
    if (!isExist) {
        frame_index = 0;
        SCLOG(VERBOSE) << "Cannot find path:\n" << depthFilename << "\n" << colorFilename;

        return false;
    }
    m_d = cv::imread(depthFilename, -1);

    // std::cout<<m_d.type()<<","
    //     <<m_d.channels()<<","
    //     <<m_d.depth()<<"\n";

    // cv::imshow("debug",m_d);
    // cv::waitKey(1);

    // mask depth
    for(size_t c=0;c<(size_t)m_d.cols;++c){
        for(size_t r=0;r<(size_t)m_d.rows;++r){
            if(m_d.at<unsigned short>(r,c)>=m_dataset->max_depth)
                m_d.at<unsigned short>(r,c) = 0;
        }
    }

    if (isFileExist(colorFilename.c_str())) {
        m_rgb = cv::imread(colorFilename, -1);
    }
    if (m_dataset->rotate_pose_img) {
        cv::rotate(m_d, m_d, cv::ROTATE_90_COUNTERCLOCKWISE);
    }
    LoadPose(m_pose, pose_file_name_,m_dataset->rotate_pose_img);
    // std::cout<<m_pose<<"\n";
    frame_index += m_dataset->frame_index_counter;
    return true;
}



void DatasetLoader_SceneNN::Reset() {
    frame_index = 0;
}

Eigen::Matrix<float,4,4> DatasetLoader_SceneNN::rotation_matrix_Z(const float rot) {
    Eigen::Matrix<float,4,4> res = Eigen::Matrix<float,4,4>::Identity();
    res << cos(rot), -sin(rot), 0, 0,
            sin(rot), cos(rot), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    return res;
}