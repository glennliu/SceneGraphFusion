//
// Created by sc on 1/13/21.
//

#ifndef LIBSURFELRECONSTRUCTION_UTIL_H
#define LIBSURFELRECONSTRUCTION_UTIL_H
#include <string>
#include <fstream>
#include <Eigen/Core>
namespace PSLAM {
    static inline bool isFileExist(const std::string &filename) {
        std::ifstream file_tmp(filename);
        if (!file_tmp.is_open()) {
            return false;
        }
        file_tmp.close();
        return true;
    }

    static inline void LoadPose(Eigen::Matrix4f &pose, const std::string &path, bool rotate) {
        pose.setIdentity();
        std::ifstream file(path);
        assert(file.is_open());
        if (file.is_open()) {
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    file >> pose(i, j);
            pose.block<3, 1>(0, 3) *= 1000.0f;
            file.close();
        }
//        std::cout << "pose\n"<< pose << "\n";
    }

    static const inline std::vector<std::string> split_str(const std::string s, const std::string delim) {
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

}



#endif //LIBSURFELRECONSTRUCTION_UTIL_H
