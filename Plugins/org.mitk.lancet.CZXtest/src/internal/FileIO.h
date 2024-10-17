#pragma once
#include <filesystem>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include<sstream> 
#include <eigen3/Eigen/Dense>
#include <vtkMatrix4x4.h>
#include <vtkSTLWriter.h>
#include <nlohmann/json.hpp>
#include <mitkSurface.h>
#include <mitkIOUtil.h>

#include "CalculationHelper.h"

    class FileIO
    {
    public:
        template<typename... Args>
        static std::filesystem::path CombinePath(const std::string& first, const Args&... args)
        {
            std::filesystem::path fullPath(first);
            (fullPath /= ... /= args);
            return fullPath.lexically_normal();
        }

        static std::vector<std::vector<double>> ReadTextFileAsTwoDarray(std::string path);

        static std::vector<Eigen::Vector3d> ReadTextFileAsPoints(std::string path);

        static void ReadTextFileAsvtkMatrix(std::string path, vtkMatrix4x4* matrix);

        static std::string GetLastNParts(const std::string& path, std::size_t n);

        static std::vector<std::string> GetPathFilesWithFileType(std::filesystem::path path, std::string fileType);

        static std::vector<std::string> GetListSubdirectories(std::string path);

        static void ReadPKAImplantJson(std::filesystem::path filePath);

        static std::string GetLastCharaters(const std::filesystem::path path, size_t n);

        static std::string getFileNameWithoutExtension(const std::string& filePath);

        static void SaveMatrix2File(std::string filePath, vtkMatrix4x4* matrix);

        static void WritePolyDataAsSTL(vtkPolyData* polyData, std::string fileName);

        static void SaveDataNodeAsSTL(mitk::DataNode* aDataNode, std::string aFileName);
    };

