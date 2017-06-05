#pragma once
#ifdef SPACERESECTIONDLL_EXPORTS
#define SPACERESECTIONDLL_API __declspec(dllexport)
#else
#define SPACERESECTIONDLL_API __declspec(dllimport)
#endif
#include "stdafx.h"
#include "SpaceResectionDLL.h"
#include "ISpaceResection.h"

using namespace std;
using namespace cv;


template class SPACERESECTIONDLL_API vector<ImageGcp>;
template class SPACERESECTIONDLL_API vector<GroundGcp>;
class SPACERESECTIONDLL_API cv::Mat;

// 此类是从 SpaceResectionDLL.dll 导出的
/**
 * 单像空间后方交会计算类，提供计算影像外方位元素的方法。
 * 使用时，先利用 内方位元素 创建对象，
 * 再输入控制点对的坐标，
 * 然后使用方法计算 摄影比例尺
 * 接着使用方法初始化 外方位元素
 * 最后使用方法对 外方位元素 进行 平差计算
 * 接下来就可以输出 外方位元素 或者 精度评定
 */
class SPACERESECTIONDLL_API CSpaceResection : public ISpaceResection {
private: 
    int m_nGcpNum; // 控制点对的数量
    vector<ImageGcp> m_vImageGcp; // 影像控制点数组
    vector<GroundGcp> m_vGroundGcp; // 地面控制点数组
    // 以下为 6 个外方位元素
    double Xs; // Xs: 摄影中心在地面摄影测量坐标系中的 X 坐标
    double Ys; // Ys: 摄影中心在地面摄影测量坐标系中的 Y 坐标
    double Zs; // Zs: 摄影中心在地面摄影测量坐标系中的 Z 坐标
    double varphi; // varphi: 像空间坐标系沿 Y 轴的旋转角
    double omega; // omega: 像空间坐标系沿 Y 轴的旋转角
    double kappa; // kappa: 像空间坐标系沿 Z 轴的旋转角
    // 以下为 3 个内方位元素
    double x0; // x0: 像主点在像空间坐标系中的 x 坐标
    double y0; // y0: 像主点在像空间坐标系中的 y 坐标
    double f; // f: 航摄仪的焦距
    double m; // m: 摄影比例尺
    Mat R; // 旋转矩阵
    // 以下为畸变参数
    size_t nCalibParams;
    double* k;
    // 以下为计算矩阵
    Mat ATA; // 法方程式系数矩阵
    Mat ATl; // 法方程式常数项矩阵
    Mat dx; // 改正数
    // 其他辅助变量
    size_t nUnknownNum;
    bool isInneriorApprox; // 内方位元素是否为近似值
    bool isSelfCalibNeed;  // 是否需要计算畸变系数
    bool isExteriorInited; // 外方位元素是否已计算初值
    bool isMeasuringScaleCalced; // 比例尺是否被计算

public:
    /**
    * 空间后方交会计算类构造函数
    * lf_x0: 已知内方位元素 x0
    * lf_y0: 已知内方位元素 y0
    * lf_f: 已知焦距 f
    * lf_m: 已知航摄比例尺 m
    */
	CSpaceResection(double lf_x0, double lf_y0, double lf_f, double lf_m, bool inneriorApprox = false, size_t calibParamNums = 0);
    /*
    * 析构函数
    */
    ~CSpaceResection();
	/**
    * 获取控制点的数量
    */
    virtual const int getGcpNum() { return m_nGcpNum; }
    /**
    * 添加控制点对
    * xi: 影像控制点在像平面坐标系中的 x 坐标，单位：mm
    * yi: 影像控制点在像平面坐标系中的 y 坐标，单位：mm
    * Xi: 地面控制点在地面摄影测量坐标系重的 X 坐标
    * Yi: 地面控制点在地面摄影测量坐标系重的 Y 坐标
    * Zi: 地面控制点在地面摄影测量坐标系重的 Z 坐标
    */
    virtual void AddGcp(double xi, double yi, double Xi, double Yi, double Zi);
    /**
    * 删除指定位置的控制点对
    * i: 指定删除的位置
    */
    virtual BOOL DeleteGcp(int i);
    /**
    * 修改指定位置的控制点坐标
    * i: 指定的控制点的位置
    * xi: 影像控制点在像平面坐标系中的 x 坐标，单位：mm
    * yi: 影像控制点在像平面坐标系中的 y 坐标，单位：mm
    * Xi: 地面控制点在地面摄影测量坐标系重的 X 坐标
    * Yi: 地面控制点在地面摄影测量坐标系重的 Y 坐标
    * Zi: 地面控制点在地面摄影测量坐标系重的 Z 坐标
    */
    virtual BOOL UpdateGcp(int i, double xi, double yi, double Xi, double Yi, double Zi);
    /**
    * 检查控制点是否符合要求
    */
    virtual BOOL CheckGcps();
    /**
     * 计算摄影比例尺（近似垂直航空摄影）
     * [RESECTION_SUCCESS] 成功计算摄影比例尺
     * [RESECTION_GCP_NOT_ENOUGH] 控制点数量不足
     */
    virtual BOOL CalcMeasuringScale();
    /**
    * 计算外方位元素的初始值
    * lf_varphi: 指定的 varphi 的值
    * lf_omega: 指定的 omega 的值
    * lf_kappa: 指定的 kappa 的值
    */
    virtual BOOL InitExteriorElements(double lf_varphi = 0.0, double lf_omega = 0.0, double lf_kappa = 0.0);
    /**
    * 迭代计算外方位元素
    */
    virtual BOOL CalcExteriorElements();
    /**
    * 单像空间后方交会解算
    */
    virtual BOOL CalcSpaceResection(double lf_kappa = 0.0);
    /**
    * 精度评定。
    */
    virtual double AssessPrecision();
    /**
    * 输出计算结果
    */
    virtual ExteriorElements GetExteriroElements();
    /**
    * 输出旋转矩阵
    */
    virtual double* GetRotateMatrix() { return (double*)R.data; }
    /**
    * 输出内方位元素
    */
    virtual InneriorElements GetInneriorElements()
    {
        InneriorElements inneriorElements = { f, x0, y0 };
        return inneriorElements;
    }
    /**
    * 输出畸变系数
    **/
    virtual void GetCalibParams(double* calibParams)
    {
        for (size_t i = 0; i < nCalibParams; i++)
        {
            calibParams[i] = k[i];
        }
    }
    /**
    * 输出像点残差
    **/
    void CalcImagePointResidual(double* pResidual);
private: 
    /**
    * 检查影像控制点是否在同一条直线上
    */
    BOOL InLineCheck();
    BOOL InCylindricalSurface();
    /**
    * 计算旋转矩阵
    * [RESECTION_SUCCESS] 成功计算旋转矩阵
    * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] 错误：外方位元素没有初始化
    * [RESECTION_ROTATE_MATRIX_IS_NOT_3_3_MATRIX] 错误：旋转矩阵为空或旋转矩阵不是3*3矩阵
    */
    BOOL Calc_R_Matrix();
    /**
    * 计算某对控制点的法方程系数矩阵 A
    * i: 要计算 A 矩阵的控制点对索引
    * A: 存储结果的 A 矩阵
    * [RESECTION_SUCCESS] 返回：成功计算
    * [RESECTION_INDEX_OUT_OF_RANGE] 错误：索引超出数组范围
    * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] 错误：外方位元素没有计算初值
    */
    BOOL Calc_A_Matrix(int i, Mat& A);
    /**
    * 计算某对控制点的常数项系数矩阵 l
    * i: 要计算 A 矩阵的控制点对索引
    * l: 存储结果的 l 矩阵
    * [RESECTION_SUCCESS] 返回：成功计算
    * [RESECTION_INDEX_OUT_OF_RANGE] 错误：索引超出数组范围
    * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] 错误：外方位元素没有计算初值
    */
    BOOL Calc_l_Matrix(int i, Mat& l);
    /**
    * 逐点法化法计算 ATA 矩阵
    * [RESECTION_SUCCESS] 返回：成功计算 ATA 矩阵
    * [RESECTION_CALC_ATA_GCP_NOT_ENOUGH] 错误：控制点数量不足
    */
    //BOOL Calc_ATA_Matrix();
    /**
    * 逐点法化法计算 ATl 矩阵
    * [RESECTION_SUCCESS] 返回：成功计算 ATl 矩阵
    * [RESECTION_CALC_ATA_GCP_NOT_ENOUGH] 错误：控制点数量不足
    */
    //BOOL Calc_ATl_Matrix();
    /**
    * 逐点法化法计算 ATA ATl 矩阵
    * [SUCCESS] 返回：成功计算 ATl 矩阵
    * [RESECTION_CALC_ATA_GCP_NOT_ENOUGH] 错误：控制点数量不足
    */
    BOOL Calc_ATA_ATl_Matrix();
    /**
    * 计算相片旋偏角
    */
    double CalcKappa();
    /**
    * 计算相片旋角 kappa
    * imageGcp1: 第1个影像控制点
    * imageGcp2: 第2个影像控制点
    * groundGcp1: 第1个地面控制点
    * groundGcp2: 第2个地面控制点
    */
    double CalcKappa(ImageGcp imageGcp1, ImageGcp imageGcp2, GroundGcp groundGcp1, GroundGcp groundGcp2);
    /**
    * 根据计算出来的外方位元素改正值改正外方位元素
    * [RESECTION_SUCCESS] 返回：成功
    * [RESECTION_CORRECT_EXTERIOR_CORRECTING_NUMBERS_NOT_ENOUGH] 错误：改正数个数不足
    * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] 错误：外方位线元素没有初始化
    */
    BOOL CorrectExteriorElements(const Mat& delta_x);
    /**
    * 初始化外方位线元素
    */
    void InitExteriorLineElements(); 
    /**
    * 判断是否满足要求
    * deltaX: 计算出来的改正数
    */
    bool isReachDemand(const Mat& deltaX)
    {
        // 将限差的角度制值转换为弧度制
        const double pi = 3.1415926535897932384626433832795, varepsilon = 0.1 / 60 / 180 * pi;
        // 角元素改正值限差
        double deltaVarphi = abs(deltaX.at<double>(3, 0)),
            deltaOmega = abs(deltaX.at<double>(4, 0)),
            deltaKappa = abs(deltaX.at<double>(5, 0));
        if ((deltaVarphi < varepsilon) && (deltaOmega < varepsilon) && (deltaKappa < varepsilon)) return true;
        return false;
    }
};
