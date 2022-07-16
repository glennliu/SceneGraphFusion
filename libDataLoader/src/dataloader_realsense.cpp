#include "dataLoader/util.h"
#include "dataLoader/dataloader_realsense.h"
#include "dataLoader/datasetRealSense.h"

using namespace PSLAM;

Datasetloader_Realsense::Datasetloader_Realsense(
    std::shared_ptr<DatasetDefinitionBase> dataset):DatasetLoader_base(std::move(dataset))
{
    std::cout<<"Initiating realsense dataloader...\n";
    if(!loadIntrinsic(m_dataset->folder+"/intrinsic.txt",m_cam_param_d))
        throw std::runtime_error("unable to open _info file");
    m_cam_param_rgb = m_cam_param_d;
}

bool Datasetloader_Realsense::Retrieve()
{
    std::string depthFilename = GetFileName(m_dataset->folder,
                                        m_dataset->folder_depth,
                                        m_dataset->prefix_depth,
                                        m_dataset->suffix_depth,
                                        m_dataset->number_length);
    std::string rgbFilename = GetFileName(m_dataset->folder,
                                        m_dataset->folder_rgb,
                                        m_dataset->prefix_rgb,
                                        m_dataset->suffix_rgb,
                                        m_dataset->number_length);
                                        
    std::string poseFilename = GetFileName(m_dataset->folder,
                                        m_dataset->folder_pose,
                                        m_dataset->prefix_pose,
                                        m_dataset->suffix_pose,
                                        m_dataset->number_length);
    std::cout<<"Depth file "<<depthFilename<<"\n"
        <<rgbFilename<<"\n"
        <<poseFilename<<"\n";
    if(!isFileExist(depthFilename)){
        frame_index = 0;
        std::cout<<"Cannot find "<<depthFilename<<"\n";
        return false;
    }
    if(!isFileExist(rgbFilename)){
        std::cout<<"Cannot find "<<rgbFilename<<"\n";
        return false;
    }
    if(!isFileExist(poseFilename)){
        std::cout<<"Cannot find "<<poseFilename<<"\n";
        return false;
    }
    m_d = cv::imread(depthFilename,-1);
    std::cout<<"depth type: "<<m_d.type()<<"\n";
    for(size_t c=0;c<(size_t)m_d.cols;++c){
        for(size_t r=0;r<(size_t)m_d.rows;++r){
            if(m_d.at<unsigned short>(r,c)>=m_dataset->max_depth)
                m_d.at<unsigned short>(r,c) = 0;
        }
    }

    m_rgb = cv::imread(rgbFilename, -1);

    LoadPose(m_pose, poseFilename, false);

    frame_index += 1;
    return true;
}

bool Datasetloader_Realsense::loadIntrinsic(
    const std::string file_dir, CameraParameters &intrinsics)
{
    const std::string search_tag_w = "depth_width";
    const std::string search_tag_h = "depth_height";
    std::string line{""};
    std::ifstream file(file_dir);
    std::cout<<"Loading intrinsic from "<<file_dir<<"...\n";
    int width,height;
    float fx,fy,cx,cy;
    if (file.is_open()) {
        while (std::getline(file,line)) {
            if (line.rfind(search_tag_w, 0) == 0)
                width = std::stoi(line.substr(line.find("= ")+2, std::string::npos));
            else if (line.rfind(search_tag_h, 0) == 0)
                height = std::stoi(line.substr(line.find("= ")+2, std::string::npos));
            else if (line.rfind("fx", 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                fx = std::stof(model);
            }
            else if (line.rfind("fy", 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                fy = std::stof(model);
            }
            else if (line.rfind("cx", 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                cx = std::stof(model);
            }            
            else if (line.rfind("cy", 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                cy = std::stof(model);
            }
        }
        file.close();
        intrinsics.Set(width,height,fx,fy,cx,cy,1.f);
        return true;
    }
    return false;
}

const std::string Datasetloader_Realsense::GetFileName(const std::string& folder,
                                                    const std::string& subfolder,
                                                    const std::string& prefix,
                                                    const std::string& suffix,
                                                    int number_length = -1) const {
    std::stringstream filename;
    const std::string path = (folder == "/" ? "" : folder) +
                             (subfolder == "/" ? "" : subfolder) +
                             (prefix == "/" ? "" : prefix);
    if (number_length < 0)
        filename << path << (suffix == "/" ? "" : suffix);
    else
        filename << path << std::setfill('0') << std::setw(number_length) << frame_index << (suffix == "/" ? "" : suffix);
    std::string s(filename.str());
    return s;
}

void Datasetloader_Realsense::Reset()
{

}


