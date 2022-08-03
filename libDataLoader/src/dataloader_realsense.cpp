#include "dataLoader/util.h"
#include "dataLoader/dataloader_realsense.h"
#include "dataLoader/datasetRealSense.h"

using namespace PSLAM;

Datasetloader_Realsense::Datasetloader_Realsense(
    std::shared_ptr<DatasetDefinitionBase> dataset):DatasetLoader_base(std::move(dataset))
{
    std::cout<<"Initiating realsense dataloader...\n";
    if(!loadIntrinsic(m_dataset->folder+"/intrinsic.txt"))
        throw std::runtime_error("unable to open intrinsic file");

    if(!loadAssociateFile(m_dataset->folder+"/association.txt"))
        throw std::runtime_error("unable to open association file");
    
    if(!loadCameraPose(m_dataset->folder+"/CameraTrajectory.txt"))
        throw std::runtime_error("unable to open trajectory file");

    if(associated_frames.size()!=camera_poses.size())
        throw std::runtime_error("Image frames and camera pose are not aligned");

}

bool Datasetloader_Realsense::loadIntrinsic(
    const std::string file_dir)
    // CameraParameters &rgb_intrinsics,CameraParameters &depth_intrinsics)
{
    // const std::string search_tag_w = "depth_width";
    // const std::string search_tag_h = "depth_height";
    std::string line{""};
    std::ifstream file(file_dir);
    std::cout<<"Loading intrinsic from "<<file_dir<<"...\n";
    struct CamParams{
        int width,height;
        float fx,fy,cx,cy;
    }rgb_param, depth_param;

    if (file.is_open()) {
        while (std::getline(file,line)) {
            if (line.rfind("depth_width", 0) == 0)
                depth_param.width = std::stoi(line.substr(line.find("= ")+2, std::string::npos));
            else if (line.rfind("depth_height", 0) == 0)
                depth_param.height = std::stoi(line.substr(line.find("= ")+2, std::string::npos));
            else if (line.rfind("depth_fx", 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                depth_param.fx = std::stof(model);
            }
            else if (line.rfind("depth_fy", 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                depth_param.fy = std::stof(model);
            }
            else if (line.rfind("depth_cx", 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                depth_param.cx = std::stof(model);
            }            
            else if (line.rfind("depth_cy", 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                depth_param.cy = std::stof(model);
            }
            else if (line.rfind("color_width", 0) == 0)
                rgb_param.width = std::stoi(line.substr(line.find("= ")+2, std::string::npos));
            else if (line.rfind("color_height", 0) == 0)
                rgb_param.height = std::stoi(line.substr(line.find("= ")+2, std::string::npos));
            else if (line.rfind("color_fx", 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                rgb_param.fx = std::stof(model);
            }
            else if (line.rfind("color_fy", 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                rgb_param.fy = std::stof(model);
            }
            else if (line.rfind("color_cx", 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                rgb_param.cx = std::stof(model);
            }            
            else if (line.rfind("color_cy", 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                rgb_param.cy = std::stof(model);
            }

        }
        file.close();
        m_cam_param_d.Set(depth_param.width,depth_param.height,depth_param.fx,depth_param.fy,depth_param.cx,depth_param.cy,1.f);
        m_cam_param_rgb.Set(rgb_param.width,rgb_param.height,rgb_param.fx,rgb_param.fy,rgb_param.cx,rgb_param.cy,1.f);

        return true;
    }
    return false;
}

bool Datasetloader_Realsense::loadAssociateFile(const std::string &file_dir)
{
    std::string line{""};
    std::ifstream file(file_dir);
    std::cout<<"Loading associate from "<<file_dir<<"...\n";
    if(file.is_open()){
        while(std::getline(file,line)){
            auto parts = split_str(line," ");
            FrameData frame_data;
            frame_data.ts = std::stod(parts[2]);
            frame_data.rgb_filename = parts[1];
            frame_data.depth_filename = "depth/"+parts[2]+".png";//parts[3];
            associated_frames.emplace_back(frame_data);
            associated_frames_map.emplace(parts[0],frame_data);
        }
        std::cout<<associated_frames.size()<<" frames of data are loaded\n";
        return true;
    }
    else return false;
}

bool Datasetloader_Realsense::loadCameraPose(const std::string &file_dir)
{
    std::string line{""};
    std::ifstream file(file_dir);
    std::cout<<"Loading camera poses from "<<file_dir<<"...\n";
    Eigen::Matrix4f T_wl;
    T_wl<<-0.0165065, -0.0583131, 0.998162, 0.0364988,
        -0.999754, -0.0138208, -0.0173403, -0.127461,
        0.0148066, -0.998203, -0.0580706, 0.199308,
        0.0, 0.0, 0.0, 1.0;

    // {   // For Azure Kinect data collected by Lin.
    //     T_wl.setIdentity();
    //     associated_frames.clear();
    // }

    if(file.is_open()){
        while(std::getline(file,line)){
            auto parts = split_str(line," ");
            Eigen::Quaternionf q_;
            Eigen::Matrix3f rot_;
            Eigen::Matrix4f T_wc, T_lc;
            T_lc.setIdentity();
            T_lc(0,3) = std::stof(parts[1]);
            T_lc(1,3) = std::stof(parts[2]);
            T_lc(2,3) = std::stof(parts[3]);
            q_.x() = std::stof(parts[4]);
            q_.y() = std::stof(parts[5]);
            q_.z() = std::stof(parts[6]);
            q_.w() = std::stof(parts[7]);

            rot_ = q_;
            T_lc.topLeftCorner(3,3) = rot_;
            T_wc = T_wl * T_lc;
            T_wc.block<3,1>(0,3) *=1000.0f;
            camera_poses.emplace_back(T_wc);

            // associated_frames.emplace_back(associated_frames_map[parts[0]]);
        }
        std::cout<<camera_poses.size()<<" poses are loaded\n";
        return true;
    }
    else return false;

}

bool Datasetloader_Realsense::Retrieve()
{
    /*
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
                                        */
    
    FrameData frame_info = associated_frames[frame_index];
    std::string rgbFilename = m_dataset->folder+"/"+frame_info.rgb_filename;
    std::string depthFilename = m_dataset->folder+"/"+frame_info.depth_filename;

    std::cout<<"Depth file: "<<depthFilename<<"\n"
        <<"Rgb file: "<<rgbFilename<<"\n";
        // <<poseFilename<<"\n";
    if(!isFileExist(depthFilename)){
        frame_index = 0;
        std::cout<<"Cannot find "<<depthFilename<<"\n";
        return false;
    }
    if(!isFileExist(rgbFilename)){
        std::cout<<"Cannot find "<<rgbFilename<<"\n";
        return false;
    }

    m_d = cv::imread(depthFilename,-1);
    // m_d_scaled.convertTo(m_d_scaled,CV_32F);
    // m_d = 0.2 * m_d_scaled;
    // m_d.convertTo(m_d,CV_16U);
    
    for(size_t c=0;c<(size_t)m_d.cols;++c){
        for(size_t r=0;r<(size_t)m_d.rows;++r){
            if(m_d.at<unsigned short>(r,c)>=m_dataset->max_depth)
                m_d.at<unsigned short>(r,c) = 0;
        }
    }

    m_rgb = cv::imread(rgbFilename, -1);

    // LoadPose(m_pose, poseFilename, false);
    m_pose = camera_poses[frame_index];

    frame_index += 1;
    return true;
}


const std::string Datasetloader_Realsense::GetFileName(const std::string& folder,
                                                    const std::string& subfolder,
                                                    const std::string& prefix,
                                                    const std::string& suffix,
                                                    int number_length = -1) const 
{
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


