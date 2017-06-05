// SpaceResection.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "SpaceResectionDLL.h"
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;

int main(size_t argc, char** argv)
{
    /*********
    * 创建对象
    **********/
    if (argc < 2)
    {
        cout << "没有内方位元素文件" << endl;
        return -1;
    }
    double x0, y0, f, pixelsize, m;
    ifstream fin_iee(argv[1]);
    fin_iee >> x0 >> y0 >> f >> pixelsize >> m;
    //ISpaceResection* spaceResection = ISpaceResection::create(x0 / pixelsize, y0 / pixelsize, f, m, false, 0); // x0=y0=0, f=153.24mm
    ISpaceResection* spaceResection = ISpaceResection::create(x0 / pixelsize, y0 / pixelsize, f, m, true, 2); // x0=y0=0, f=153.24mm
    fin_iee.close();
    /***********
    * 添加控制点
    ************/
    if (argc < 3)
    {
        cout << "没有控制点文件" << endl;
        return -1;
    }
    ifstream fin_gcp(argv[2]);
    if (!fin_gcp.is_open())
    {
        cout << "打开控制点文件失败" << endl;
        return -1;
    }
    int nGcpNum = 0; // 控制点数量
    fin_gcp >> nGcpNum;
    vector<size_t> vPointId;
    for (size_t i = 0; i < nGcpNum && !fin_gcp.eof(); i++)
    {
        size_t gcp_id;
        double gcp_coord[5];
        fin_gcp >> gcp_id >> gcp_coord[0] >> gcp_coord[1] >> gcp_coord[2] >> gcp_coord[3] >> gcp_coord[4];
        vPointId.push_back(gcp_id);
        spaceResection->AddGcp(gcp_coord[0] * pixelsize, gcp_coord[1] * pixelsize, gcp_coord[2], gcp_coord[3], gcp_coord[4]);
    }
    fin_gcp.close();
    /*************
    * 后方交会解算
    **************/
    int results = RESECTION_SUCCESS;
    //results = spaceResection->CalcMeasuringScale();
    results = spaceResection->InitExteriorElements(0.0, 0.0, 0.0);
    results = spaceResection->CalcSpaceResection();
    if (results != RESECTION_SUCCESS)
    {
        switch (results)
        {
            case RESECTION_INDEX_OUT_OF_RANGE :
                printf_s("索引超过数组范围！");
                break;
            case RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED :
                printf_s("外方位元素没有初始化！");
                break;
            case RESECTION_GCP_NOT_ENOUGH :
                printf_s("控制点数量不足！");
                break;
            case RESECTION_ROTATE_MATRIX_IS_NOT_3_3_MATRIX :
                printf_s("旋转矩阵不是3×3的矩阵！");
                break;
            case RESECTION_MEASURE_SCALE_NOT_CALCULATED :
                printf_s("比例尺没有计算！");
                break;
            case RESECTION_CALC_A_OUT_MATRIX_IS_EMPTY :
                printf_s("输出数组为空！");
                break;
            case RESECTION_CALC_A_OUT_MATRIX_IS_NOT_A_2_6_MATRIX :
                printf_s("输出的A数组不是2×6数组！");
                break;
            case RESECTION_CALC_A_ROTATE_MATRIX_IS_EMPTY :
                printf_s("旋转矩阵为空！");
                break;
            case RESECTION_CALC_l_OUT_MATRIX_IS_EMPTY :
                printf_s("输出l数组为空！");
                break;
            case RESECTION_CALC_l_OUT_MATRIX_IS_NOT_A_2_1_MATRIX :
                printf_s("输出l数组不是2×1矩阵！");
                break;
            case RESECTION_CALC_l_ROTATE_MATRIX_IS_EMPTY :
                printf_s("旋转矩阵为空！");
                break;
            case RESECTION_CORRECT_EXTERIOR_CORRECTING_NUMBERS_NOT_ENOUGH :
                printf_s("外方位元素改正数不足！");
                break;
            default:
                printf_s("未知错误！");
                break;
        }
        system("pause");
        return -1;
    }
    /***************
    * 获取外方位元素
    ****************/
    ExteriorElements exterior = spaceResection->GetExteriroElements();
    double* R = spaceResection->GetRotateMatrix();
    /*************
    * 输出计算结果
    **************/
    cout << "单像空间后方交会结果" << endl << endl;
    // 像点中误差
    //printf_s("精度：%.8lf\n", spaceResection->AssessPrecision());
    cout << right << fixed << setw(12) << setprecision(8) 
        << spaceResection->AssessPrecision() / pixelsize << " 像素" << endl;
    cout << endl;
    // 内方位元素
    InneriorElements innerior = spaceResection->GetInneriorElements();
    cout << "内方位元素" << endl;
    cout << setw(2) << right << "f" << setw(10) << setprecision(3) << innerior.f << " mm" << endl;
    cout << setw(2) << right << "x0" << setw(10) << setprecision(3) << innerior.x0 / pixelsize << " 像素" << endl;
    cout << setw(2) << right << "y0" << setw(10) << setprecision(3) << innerior.y0 / pixelsize << " 像素" << endl;
    cout << endl;
    // 外方位元素
    printf_s("单像空间后方交会计算结果：\n");
    printf_s("Xs = %8.2lf\n", exterior.Xs);
    printf_s("Ys = %8.2lf\n", exterior.Ys);
    printf_s("Zs = %8.2lf\n", exterior.Zs);
    printf_s("R=\n");
    printf_s("%9.5lf %9.5lf %9.5lf\n", R[0], R[1], R[2]);
    printf_s("%9.5lf %9.5lf %9.5lf\n", R[3], R[4], R[5]);
    printf_s("%9.5lf %9.5lf %9.5lf\n", R[6], R[7], R[8]);
    // 基本系数
    double calibParams[2] = { 0.0 };
    spaceResection->GetCalibParams(calibParams);
    cout << endl << "畸变系数" << endl;
    for (size_t i = 0; i < 2; i++)
    {
        cout << "k" << i << right << scientific << setw(12) << setprecision(3) << calibParams[i] << endl;
    }
    cout << resetiosflags(ios::scientific) << endl;
    // 像点残差中误差
    double* residual = new double[vPointId.size()];
    spaceResection->CalcImagePointResidual(residual);
    cout << "像点残差中误差：" << endl << setiosflags(ios::right) << setiosflags(ios::fixed);
    for (size_t i = 0; i < vPointId.size(); i++)
    {
        cout << vPointId.at(i) << ":" 
            << setw(10) << setprecision(6) << residual[i] 
            << '\t' << "约" 
            << setw(6) << setprecision(3) << residual[i] / pixelsize << " 像元" << endl;
    }
    //system("Pause");
    /*********
    * 释放对象
    **********/
    //delete spaceResection;
    ISpaceResection::release(spaceResection);
    return 0;
}

