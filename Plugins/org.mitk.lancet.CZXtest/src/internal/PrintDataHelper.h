#pragma once
#include <mitkPoint.h>
#include <QTextBrowser>
#include <vtkMatrix4x4.h>
#include <eigen3/Eigen/Dense>

    class  PrintDataHelper
    {
    public:
        static void AppendTextBrowserMatrix(QTextBrowser* browser, const char* matrixName, const double* matrix);
        static void AppendTextBrowserMatrix(QTextBrowser* browser, const char* matrixName, const vtkMatrix4x4* matrix);
        static void PrintMatrix(const char* matrixName, const double* matrix);
        static void PrintMatrix(const char* matrixName, const vtkMatrix4x4* matrix);
        static void CoutMatrix(const char* matrixName, const double* matrix);
        static void CoutMatrix(const char* matrixName, const vtkMatrix4x4* matrix);
        static void AppendTextBrowserArray(QTextBrowser* browser, const char* arrayName, int size, const double* array);
        static void AppendTextBrowserArray(QTextBrowser* browser, const char* arrayName, const std::vector<double> array);
        static void AppendTextBrowserArray(QTextBrowser* browser, const Eigen::Vector3d array, const char* arrayName);
        static void CoutArray(const Eigen::Vector3d array, const char* arrayName);
        static void CoutArray(const double* array, int size, const char* arrayName);

        static void AppentTextBrowserVector(QTextBrowser* browser, const char* vecName, const std::vector<double> vec);
        static void CoutVector(const std::vector<double> vec, const char* vecName);

        template <std::size_t N>
        static void AppendTextBrowserArray(QTextBrowser* browser, const char* arrayName, const std::array<double, N>& arr)
        {
            QString str;
            for (const auto& element : arr)
            {
                str += QString::number(element) + " ";
            }
            browser->append(arrayName + str);
        }

        template <typename TPoint>
        static void CoutMitkPoint(const TPoint& aPoint, std::string aPointName)
        {
            std::cout << aPointName + " :(";
            for (unsigned int i = 0; i < aPoint.GetPointDimension(); ++i) {
                std::cout << aPoint[i];
                if (i < aPoint.GetPointDimension() - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << ")" << std::endl;
        }
    };


