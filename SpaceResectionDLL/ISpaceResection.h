#pragma once

#include "DataTypes.h"

class SPACERESECTIONDLL_API ISpaceResection
{
// **********
// 构造与析构
// **********
public:
    /**
    * 空间后方交会计算类构造函数
    * lf_x0: 已知内方位元素 x0
    * lf_y0: 已知内方位元素 y0
    * lf_f: 已知焦距 f
    * lf_m: 已知航摄比例尺 m
    */
    static ISpaceResection* create(double lf_x0, double lf_y0, double lf_f, double lf_m, bool inneriorApprox = false, size_t calibParamNums = 0);
    /*
    * 析构函数
    */
    void operator delete(void* instance);
// ****
// 方法
// ****
public:
    /**
    * 获取控制点的数量
    */
    virtual const int getGcpNum() = 0;
    /**
    * 添加控制点对
    * xi: 影像控制点在像平面坐标系中的 x 坐标，单位：mm
    * yi: 影像控制点在像平面坐标系中的 y 坐标，单位：mm
    * Xi: 地面控制点在地面摄影测量坐标系重的 X 坐标
    * Yi: 地面控制点在地面摄影测量坐标系重的 Y 坐标
    * Zi: 地面控制点在地面摄影测量坐标系重的 Z 坐标
    */
    virtual void AddGcp(double xi, double yi, double Xi, double Yi, double Zi) = 0;
    /**
    * 删除指定位置的控制点对
    * i: 指定删除的位置
    */
    virtual int DeleteGcp(int i) = 0;
    /**
    * 修改指定位置的控制点坐标
    * i: 指定的控制点的位置
    * xi: 影像控制点在像平面坐标系中的 x 坐标，单位：mm
    * yi: 影像控制点在像平面坐标系中的 y 坐标，单位：mm
    * Xi: 地面控制点在地面摄影测量坐标系重的 X 坐标
    * Yi: 地面控制点在地面摄影测量坐标系重的 Y 坐标
    * Zi: 地面控制点在地面摄影测量坐标系重的 Z 坐标
    */
    virtual int UpdateGcp(int i, double xi, double yi, double Xi, double Yi, double Zi) = 0;
    /**
    * 检查控制点是否符合要求
    */
    virtual int CheckGcps() = 0;
    /**
    * 计算摄影比例尺（近似垂直航空摄影）
    * [RESECTION_SUCCESS] 成功计算摄影比例尺
    * [RESECTION_GCP_NOT_ENOUGH] 控制点数量不足
    */
    virtual int CalcMeasuringScale() = 0;
    /**
    * 计算外方位元素的初始值
    * lf_varphi: 指定的 varphi 的值
    * lf_omega: 指定的 omega 的值
    * lf_kappa: 指定的 kappa 的值
    */
    virtual int InitExteriorElements(double lf_varphi, double lf_omega, double lf_kappa) = 0;
    /**
    * 迭代计算外方位元素
    */
    virtual int CalcExteriorElements() = 0;
    /**
    * 单像空间后方交会解算
    */
    virtual int CalcSpaceResection(double lf_kappa = 0.0) = 0;
    /**
    * 精度评定。
    */
    virtual double AssessPrecision() = 0;
    /**
    * 输出计算结果
    */
    virtual ExteriorElements GetExteriroElements() = 0;
    /**
    * 输出旋转矩阵
    */
    virtual double* GetRotateMatrix() = 0;
};

