#pragma once

// 下列 ifdef 块是创建使从 DLL 导出更简单的
// 宏的标准方法。此 DLL 中的所有文件都是用命令行上定义的 SPACERESECTIONDLL_EXPORTS
// 符号编译的。在使用此 DLL 的
// 任何其他项目上不应定义此符号。这样，源文件中包含此文件的任何其他项目都会将
// SPACERESECTIONDLL_API 函数视为是从 DLL 导入的，而此 DLL 则将用此宏定义的
// 符号视为是被导出的。
#ifdef SPACERESECTIONDLL_EXPORTS
#define SPACERESECTIONDLL_API __declspec(dllexport)
#else
#define SPACERESECTIONDLL_API __declspec(dllimport)
#endif

// 外方位元素结构
struct SPACERESECTIONDLL_API ExteriorElements
{
    double Xs; // Xs: 摄影中心在地面摄影测量坐标系中的 X 坐标
    double Ys; // Ys: 摄影中心在地面摄影测量坐标系中的 Y 坐标
    double Zs; // Zs: 摄影中心在地面摄影测量坐标系中的 Z 坐标
    double varphi; // varphi: 像空间坐标系沿 Y 轴的旋转角
    double omega; // omega: 像空间坐标系沿 Y 轴的旋转角
    double kappa; // kappa: 像空间坐标系沿 Z 轴的旋转角
};


/**
* 影像控制点结构。
* x: 像空间坐标系的 x 坐标
* y: 像空间坐标洗的 y 坐标
*/
struct ImageGcp
{
    double  x; // 像空间坐标系的 x 坐标
    double  y; // 像空间坐标系的 y 坐标
};


/**
* 地面控制点结构
* X: 地面摄影测量坐标系的 X 坐标
* Y: 地面摄影测量坐标系的 Y 坐标
* Z: 地面摄影测量坐标系的 Z 坐标
*/
struct GroundGcp
{
    double X; // 地面摄影测量坐标系的 X 坐标
    double Y; // 地面摄影测量坐标系的 Y 坐标
    double Z; // 地面摄影测量坐标系的 Z 坐标
};


/**
* 影像控制点对结构。
* x: 像空间坐标系的 x 坐标
* y: 像空间坐标洗的 y 坐标
* X: 地面摄影测量坐标系的 X 坐标
* Y: 地面摄影测量坐标系的 Y 坐标
* Z: 地面摄影测量坐标系的 Z 坐标
*/
struct ControlPiar
{
    size_t id;
    ImageGcp imageCoord;
    GroundGcp groundCoord;
};