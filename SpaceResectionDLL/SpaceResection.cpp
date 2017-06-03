// SpaceResectionDLL.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "SpaceResection.h"


// 这是导出变量的一个示例
//SPACERESECTIONDLL_API int nSpaceResectionDLL=0;

// 这是导出函数的一个示例。
//SPACERESECTIONDLL_API int fnSpaceResectionDLL(void)
//{
//    return 42;
//}


/**
 * 空间后方交会计算类构造函数
 * lf_x0: 已知内方位元素 x0
 * lf_y0: 已知内方位元素 y0
 * lf_f: 已知焦距 f
 * lf_m: 已知航摄比例尺 m
 */
CSpaceResection::CSpaceResection(double lf_x0, double lf_y0, double lf_f, double lf_m, bool inneriorApprox, size_t calibParamNums)
    : m_vGroundGcp()
    , m_vImageGcp()
    , R(3, 3, CV_64FC1, 0.0)
    , nCalibParams(calibParamNums)
    , isInneriorApprox(inneriorApprox)
{
    m_nGcpNum = 0;
    Xs = Ys = Zs = varphi = omega = kappa = 0;
    x0 = lf_x0;
    y0 = lf_y0;
    f = lf_f;
    m = lf_m;
    k = new double[calibParamNums];
    memset(k, 0, calibParamNums * sizeof(double));
    isExteriorInited = false;
    isMeasuringScaleCalced = true;
    nUnknownNum = 6 + calibParamNums;
    if (isInneriorApprox) nUnknownNum += 3;
    if (!ATA.empty()) ATA.release();
    ATA.create((int)nUnknownNum, (int)nUnknownNum, CV_64FC1);
    if (!ATl.empty()) ATl.release();
    ATl.create((int)nUnknownNum, (int)nUnknownNum, CV_64FC1);
}

CSpaceResection::~CSpaceResection()
{
    //if (m_vImageGcp.size() > 0) m_vImageGcp.clear();
    //if (m_vGroundGcp.size() > 0) m_vGroundGcp.clear();
    R.release();
    ATA.release();
    ATl.release();
    dx.release();
    if (k) delete k;
    k = nullptr;
}


/**
 * 添加控制点对
 * xi: 影像控制点在像平面坐标系中的 x 坐标
 * yi: 影像控制点在像平面坐标系中的 y 坐标
 * Xi: 地面控制点在地面摄影测量坐标系重的 X 坐标
 * Yi: 地面控制点在地面摄影测量坐标系重的 Y 坐标
 * Zi: 地面控制点在地面摄影测量坐标系重的 Z 坐标
 */
void CSpaceResection::AddGcp(double xi, double yi, double Xi, double Yi, double Zi)
{
    // 添加控制点对
    ImageGcp imageGcp = { xi, yi };
    GroundGcp groundGcp = { Xi, Yi, Zi };
    m_vImageGcp.push_back(imageGcp);
    m_vGroundGcp.push_back(groundGcp);
    m_nGcpNum++;
}


/**
 * 删除指定位置的控制点对
 * i: 指定删除的位置
 * [RESECTION_SUCCESS] 返回：成功
 * [RESECTION_INDEX_OUT_OF_RANGE] 错误：索引超过数组范围
 */
BOOL CSpaceResection::DeleteGcp(int i)
{
    if (i >= m_nGcpNum) return RESECTION_INDEX_OUT_OF_RANGE;

    m_vImageGcp.erase(m_vImageGcp.begin() + i);
    m_vGroundGcp.erase(m_vGroundGcp.begin() + i);

    m_nGcpNum--;

    return RESECTION_SUCCESS;
}


/**
 * 修改指定位置的控制点坐标
 * i: 指定的控制点的位置
 * xi: 影像控制点在像平面坐标系中的 x 坐标
 * yi: 影像控制点在像平面坐标系中的 y 坐标
 * Xi: 地面控制点在地面摄影测量坐标系重的 X 坐标
 * Yi: 地面控制点在地面摄影测量坐标系重的 Y 坐标
 * Zi: 地面控制点在地面摄影测量坐标系重的 Z 坐标
 * [RESECTION_SUCCESS] 返回：成功
 * [RESECTION_INDEX_OUT_OF_RANGE] 错误：索引超过数组范围
 */
BOOL CSpaceResection::UpdateGcp(int i, double xi, double yi, double Xi, double Yi, double Zi)
{
    if (i >= m_nGcpNum) return RESECTION_INDEX_OUT_OF_RANGE;

    m_vImageGcp.at(i).x = xi;
    m_vImageGcp.at(i).y = yi;
    m_vGroundGcp.at(i).X = Xi;
    m_vGroundGcp.at(i).Y = Yi;
    m_vGroundGcp.at(i).Z = Zi;

    return RESECTION_SUCCESS;
}


/**
 * 检查控制点是否符合要求
 * 影像控制点不能在同一条直线上
 * 地面控制点不能再同一个圆柱面上
 */
BOOL CSpaceResection::CheckGcps()
{
    // 如果控制点的数量不足四对，返回错误
    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
    
    // 影像控制点不能在同一条直线上检查
    BOOL InLineCheckResult = InLineCheck();
    if (InLineCheckResult != RESECTION_SUCCESS)
    {
        return InLineCheckResult;
    }
    BOOL InCylindricalSurfaceCheck = InCylindricalSurface();
    if (InCylindricalSurfaceCheck != RESECTION_SUCCESS)
    {
        return InCylindricalSurfaceCheck;
    }

    return RESECTION_SUCCESS;
}


/**
 * 检查影像控制点是否在同一条直线上
 * [RESECTION_SUCCESS] 成功
 * [RESECTION_SOME_GCP_IN_ONE_LINE] 有三点共线
 */
BOOL CSpaceResection::InLineCheck()
{
    ImageGcp* vImageGcp = m_vImageGcp.data();
    for (int i = 0; i < m_nGcpNum - 2; i++)
    {
        ImageGcp* imageGcp1 = vImageGcp + i;
        for (int j = i + 1; j < m_nGcpNum - 1; j++)
        {
            ImageGcp* imageGcp2 = vImageGcp + j;
            double k1 = (imageGcp2->y - imageGcp1->y) / (imageGcp2->x - imageGcp1->x);
            for (int n = j + 1; n < m_nGcpNum; n++)
            {
                ImageGcp* imageGcp3 = vImageGcp + n;
                double k2 = (imageGcp3->y - imageGcp2->y) / (imageGcp3->x - imageGcp2->x);
                if (k2 == k1)
                {
                    return RESECTION_SOME_GCP_IN_ONE_LINE;
                }
            }
        }
    }
    return RESECTION_SUCCESS;
}


/**
 * 创建凸多边形
 * groundGcp: 构成四边形的四个地面点。
 * result: 结果数组，存储顺序。
 */
bool makeConvexQuadrangle(GroundGcp* groundGcp, int* result)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (j == i) continue;
            for (int m = 0; m < 4; m++)
            {
                if (m == j || m == i) continue;
                
                int n = 6 - i - j - m;
                double x1 = groundGcp[i].X, x2 = groundGcp[j].X, x3 = groundGcp[m].X, x4 = groundGcp[n].X;
                double y1 = groundGcp[i].Y, y2 = groundGcp[j].Y, y3 = groundGcp[m].Y, y4 = groundGcp[n].Y;
                double cross1 = (x2 - x1)*(y3 - y2) - (x3 - x2)*(y2 - y1);
                double cross2 = (x3 - x2)*(y4 - y3) - (x4 - x3)*(y3 - y2);
                double cross3 = (x4 - x3)*(y1 - y4) - (x1 - x4)*(y4 - y3);
                double cross4 = (x1 - x4)*(y2 - y1) - (x2 - x1)*(y1 - y4);

                if (cross1 > 0)
                {
                    if ((cross2 > 0) && (cross3 > 0) && (cross4 > 0))
                    {
                        result[0] = i; result[1] = j; result[2] = m; result[3] = n;
                        return true;
                    }
                }
                else if (cross1 < 0)
                {
                    if ((cross2 < 0) && (cross3 < 0) && (cross4 < 0))
                    {
                        result[0] = i; result[1] = j; result[2] = m; result[3] = n;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}


/**
 * 托勒密定理验证
 * groundGcp: 构成四边形的四个地面点。
 * arrangement: 排列顺序
 */
double CalcPtolemy(GroundGcp* groundGcp, int* arrangement)
{
    double x1 = groundGcp[arrangement[0]].X, x2 = groundGcp[arrangement[1]].X, x3 = groundGcp[arrangement[2]].X, x4 = groundGcp[arrangement[3]].X;
    double y1 = groundGcp[arrangement[0]].Y, y2 = groundGcp[arrangement[1]].Y, y3 = groundGcp[arrangement[2]].Y, y4 = groundGcp[arrangement[3]].Y;
    double len12 = sqrt((y1 - y2)*(y1 - y2) + (x1 - x2)*(x1 - x2));
    double len23 = sqrt((y2 - y3)*(y2 - y3) + (x2 - x3)*(x2 - x3));
    double len34 = sqrt((y3 - y4)*(y3 - y4) + (x3 - x4)*(x3 - x4));
    double len41 = sqrt((y4 - y1)*(y4 - y1) + (x4 - x1)*(x4 - x1));
    double len13 = sqrt((y1 - y3)*(y1 - y3) + (x1 - x3)*(x1 - x3));
    double len24 = sqrt((y2 - y4)*(y2 - y4) + (x2 - x4)*(x2 - x4));

    return abs(len12*len34 + len23*len41 - len13*len24);
}

/**
 * 检查地面控制点是否在同一个圆柱面上
 * 根据托勒密定理
 * 圆内接凸四边形两对对边的乘积之和等于对角线的乘积
 */
BOOL CSpaceResection::InCylindricalSurface()
{
    GroundGcp* vGroundGcp = m_vGroundGcp.data();
    // 选取四个点四个点进行组合
    for (int i = 0; i < m_nGcpNum - 3; i++)
    {
        for (int j = i + 1; j < m_nGcpNum - 2; j++)
        {
            for (int m = j + 1; m < m_nGcpNum - 1; m++)
            {
                for (int n = m + 1; n < m_nGcpNum; n++)
                {
                    // 构造四边形数组
                    GroundGcp groundGcp[4] = {vGroundGcp[i], vGroundGcp[j], vGroundGcp[m], vGroundGcp[n]};
                    // 开辟结果存储数组
                    int arrangement[4] = { 0 };
                    // 构造凸四边形
                    if (makeConvexQuadrangle(groundGcp, arrangement))
                    {
                        // 托勒密定理验证
                        if ( CalcPtolemy(groundGcp, arrangement) < 1.0e-12)
                        {
                            return RESECTION_SOME_GCP_IN_ONE_CYLINDRICAL_SURFACE;
                        }
                    }
                    
                }
            }
        }
    }
    return RESECTION_SUCCESS;
}


/**
 * 计算摄影比例尺（近似垂直航空摄影）
 * [RESECTION_SUCCESS] 成功计算摄影比例尺
 * [RESECTION_GCP_NOT_ENOUGH] 控制点数量不足
 */
BOOL CSpaceResection::CalcMeasuringScale()
{
    // 入口处检验
    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
    
    // 提取一对影像控制点和地面控制点
    ImageGcp imageGcp1 = m_vImageGcp.at(0), imageGcp2 = m_vImageGcp.at(1);
    GroundGcp groundGcp1 = m_vGroundGcp.at(0), groundGcp2 = m_vGroundGcp.at(1);

    double imageLength = sqrt((imageGcp1.x - imageGcp2.x) * (imageGcp1.x - imageGcp2.x) + (imageGcp1.y - imageGcp2.y)*(imageGcp1.y - imageGcp2.y));
    double groundLength = sqrt((groundGcp1.X - groundGcp2.X) * (groundGcp1.X - groundGcp2.X) + (groundGcp1.Y - groundGcp2.Y)*(groundGcp1.Y - groundGcp2.Y));

    m = groundLength / imageLength;
    //m = 50000;

    isMeasuringScaleCalced = true;

    return RESECTION_SUCCESS;
}


/**
 * 计算旋转矩阵
 * [RESECTION_SUCCESS] 成功计算旋转矩阵
 * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] 错误：外方位元素没有初始化
 * [RESECTION_ROTATE_MATRIX_IS_NOT_3_3_MATRIX] 错误：旋转矩阵为空或旋转矩阵不是3*3矩阵
 */
BOOL CSpaceResection::Calc_R_Matrix()
{
    // 入口处检验
    // 检验外方位元素是否已经初始化
    if (isExteriorInited == false) return RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED;
    if ((R.empty()) || (R.rows != 3) || (R.cols != 3)) return RESECTION_ROTATE_MATRIX_IS_NOT_3_3_MATRIX;

    // 预先计算好数据
    double sinVarphi = sin(varphi); double sinOmega = sin(omega); double sinKappa = sin(kappa);
    double cosVarphi = cos(varphi); double cosOmega = cos(omega); double cosKappa = cos(kappa);
    // 计算旋转矩阵
    R.at<double>(0, 0) = cosVarphi*cosKappa - sinVarphi*sinOmega*sinKappa;
    R.at<double>(0, 1) = -cosVarphi*sinKappa - sinVarphi*sinOmega*cosKappa;
    R.at<double>(0, 2) = -sinVarphi*cosOmega;
    R.at<double>(1, 0) = cosOmega*sinKappa;
    R.at<double>(1, 1) = cosOmega*cosKappa;
    R.at<double>(1, 2) = -sinOmega;
    R.at<double>(2, 0) = sinVarphi*cosKappa + cosVarphi*sinOmega*sinKappa;
    R.at<double>(2, 1) = -sinVarphi*sinKappa + cosVarphi*sinOmega*cosKappa;
    R.at<double>(2, 2) = cosVarphi*cosOmega;

    return RESECTION_SUCCESS;
}


/**
 * 计算某对控制点的法方程系数矩阵 A
 * i: 要计算 A 矩阵的控制点对索引
 * A: 存储结果的 A 矩阵
 * [RESECTION_SUCCESS] 返回：成功计算
 * [RESECTION_INDEX_OUT_OF_RANGE] 错误：索引超出数组范围
 * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] 错误：外方位元素没有计算初值
 */
BOOL CSpaceResection::Calc_A_Matrix(int i, Mat& A)
{
    /**************
    /* 获取一些数据
    /**************/
    // 获取参数个数
    const size_t params = A.cols;
    // 提取控制点，并用 const 保护。
    const ImageGcp iImageGcp = m_vImageGcp.at(i);
    const GroundGcp iGroundGcp = m_vGroundGcp.at(i);
    // 提取地面控制点坐标
    const double x = iImageGcp.x;
    const double y = iImageGcp.y;
    // 提取旋转矩阵中的元素
    const double 
        a1 = R.at<double>(0, 0), a2 = R.at<double>(0, 1), a3 = R.at<double>(0, 2),
        b1 = R.at<double>(1, 0), b2 = R.at<double>(1, 1), b3 = R.at<double>(1, 2),
        c1 = R.at<double>(2, 0), c2 = R.at<double>(2, 1), c3 = R.at<double>(2, 2);
    // 计算 Z_bar
    double Z_bar = a3*(iGroundGcp.X - Xs) + b3 * (iGroundGcp.Y - Ys) + c3* (iGroundGcp.Z - Zs);
    // 预先计算好数据
    double Dx = x - x0, Dy = y - y0;
    double r2 = Dx*Dx + Dy*Dy;
    double sinOmega = sin(omega); double sinKappa = sin(kappa);
    double cosOmega = cos(omega); double cosKappa = cos(kappa);
    /***************************
    ** 生成6个外方位元素的偏导数
    ****************************/
    // 计算6个外方位线元素的偏导数
    A.at<double>(0, 0) = (a1*f + a3*Dx) / Z_bar;
    A.at<double>(0, 1) = (b1*f + b3*Dx) / Z_bar;
    A.at<double>(0, 2) = (c1*f + c3*Dx) / Z_bar;
    A.at<double>(1, 0) = (a2*f + a3*Dy) / Z_bar;
    A.at<double>(1, 1) = (b2*f + b3*Dy) / Z_bar;
    A.at<double>(1, 2) = (c2*f + c3*Dy) / Z_bar;
    // 计算6个外方位角元素的偏导数
    A.at<double>(0, 3) = Dy * sinOmega - (Dx*(Dx*cosKappa - Dy*sinKappa) / f + f*cosKappa)*cosOmega;
    A.at<double>(0, 4) = -f*sinKappa - Dx*(Dx*sinKappa + Dy*cosKappa) / f;
    A.at<double>(0, 5) = Dy;
    A.at<double>(1, 3) = -Dx*sinOmega - (Dy*(Dx*cosKappa - Dy*sinKappa) / f - f*sinKappa)*cosOmega;
    A.at<double>(1, 4) = -f*cosKappa - Dy*(Dx*sinKappa + Dy*cosKappa) / f;
    A.at<double>(1, 5) = -Dx;
    /****************
    ** 计算其他偏导数
    *****************/
    switch (params)
    {
        case 11:
            // 计算二阶畸变参数的偏导数
            A.at<double>(0, 10) = (Dx)*r2;
            A.at<double>(1, 10) = (Dy)*r2;
        case 10:
            // 计算一阶畸变参数偏导数
            A.at<double>(0, 9) = (Dx)*sqrt(r2);
            A.at<double>(1, 9) = (Dy)*sqrt(r2);
        case 9:
            // 计算3个内方位元素的偏导数
            A.at<double>(0, 6) = (Dx / f);
            A.at<double>(0, 7) = 1;
            A.at<double>(0, 8) = 0;
            A.at<double>(1, 6) = (Dy / f);
            A.at<double>(1, 7) = 0;
            A.at<double>(1, 8) = 1;
        default:
            break;
    }

    // 返回成功报告
    return RESECTION_SUCCESS;
}


/**
 * 计算某对控制点的常数项系数矩阵 l
 * i: 要计算 A 矩阵的控制点对索引
 * l: 存储结果的 l 矩阵
 * [RESECTION_SUCCESS] 返回：成功计算
 * [RESECTION_INDEX_OUT_OF_RANGE] 错误：索引超出数组范围
 * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] 错误：外方位元素没有计算初值
 */
BOOL CSpaceResection::Calc_l_Matrix(int i, Mat & l)
{
    // 提取控制点，并用 const 保护。
    const ImageGcp iImageGcp = m_vImageGcp.at(i);
    const GroundGcp iGroundGcp = m_vGroundGcp.at(i);
    // 提取控制点坐标
    const double x = iImageGcp.x; const double y = iImageGcp.y;
    const double X = iGroundGcp.X; const double Y = iGroundGcp.Y; const double Z = iGroundGcp.Z;
    // 提取旋转矩阵中的元素
    const double a1 = R.at<double>(0, 0), a2 = R.at<double>(0, 1), a3 = R.at<double>(0, 2);
    const double b1 = R.at<double>(1, 0), b2 = R.at<double>(1, 1), b3 = R.at<double>(1, 2);
    const double c1 = R.at<double>(2, 0), c2 = R.at<double>(2, 1), c3 = R.at<double>(2, 2);

    // 计算X_bar, Y_Bar, Z_bar矩阵
    double X_bar = a1*(X - Xs) + b1 * (Y - Ys) + c1* (Z - Zs);
    double Y_bar = a2*(X - Xs) + b2 * (Y - Ys) + c2* (Z - Zs);
    double Z_bar = a3*(X - Xs) + b3 * (Y - Ys) + c3* (Z - Zs);

    // 计算 像点坐标 初始值
    double x_origin = -f * X_bar / Z_bar;
    double y_origin = -f * Y_bar / Z_bar;
    double r2 = x_origin*x_origin + y_origin*y_origin;
    double dx = 0, dy = 0;
    switch (nCalibParams)
    {
        case 2:
            dx += x_origin*r2*r2*k[1];
            dy += y_origin*r2*r2*k[1];
        case 1:
            dx += x_origin*r2*k[0];
            dy += y_origin*r2*k[0];
            break;
        default:
            break;
    }
    // 计算 l 矩阵
    l.at<double>(0, 0) = x - (x_origin + x0 + dx);
    l.at<double>(1, 0) = y - (y_origin + y0 + dy);

    return RESECTION_SUCCESS;
}


/**
 * 逐点法化法计算 ATA 矩阵
 * [RESECTION_SUCCESS] 返回：成功计算 ATA 矩阵
 * [RESECTION_CALC_ATA_GCP_NOT_ENOUGH] 错误：控制点数量不足
 */
//BOOL CSpaceResection::Calc_ATA_Matrix()
//{
//    // 入口处验证
//    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
//
//    Mat tempATA(6, 6, CV_64FC1, 0.0);
//    Mat Ai(2, 6, CV_64FC1, 0.0);
//    // 逐点法化
//    for (int i = 0; i < m_nGcpNum; i++)
//    {
//        // 计算某点的法方程系数
//        BOOL calcAResult = Calc_A_Matrix(i, Ai);
//        if (calcAResult != RESECTION_SUCCESS)
//        {
//            return calcAResult;
//        }
//        // 加到 ATA矩阵中
//        ATA = ATA + (Ai.t() * Ai);
//    }
//
//    return RESECTION_SUCCESS;
//}


/**
 * 逐点法化法计算 ATl 矩阵
 * [RESECTION_SUCCESS] 返回：成功计算 ATl 矩阵
 * [RESECTION_CALC_ATA_GCP_NOT_ENOUGH] 错误：控制点数量不足
 */
//BOOL CSpaceResection::Calc_ATl_Matrix()
//{
//    // 入口处验证
//    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
//
//    Mat tempATA(6, 6, CV_64FC1, 0.0);
//    Mat tempATl(6, 1, CV_64FC1, 0.0);
//    Mat Ai(2, 6, CV_64FC1, 0.0);
//    Mat li(2, 1, CV_64FC1, 0.0);
//    // 逐点法化
//    for (int i = 0; i < m_nGcpNum; i++)
//    {
//        // 计算某点的法方程系数
//        BOOL calcAResult = Calc_A_Matrix(i, Ai);
//        if (calcAResult != RESECTION_SUCCESS)
//        {
//            return calcAResult;
//        }
//        BOOL calclResult = Calc_l_Matrix(i, li);
//        // 加到 ATl 矩阵中
//        ATl = ATl + (Ai.t() * li);
//    }
//
//    return RESECTION_SUCCESS;
//}


/**
* 逐点法化法计算 ATA ATl 矩阵
* [SUCCESS] 返回：成功计算 ATl 矩阵
* [RESECTION_CALC_ATA_GCP_NOT_ENOUGH] 错误：控制点数量不足
*/
BOOL CSpaceResection::Calc_ATA_ATl_Matrix()
{
    // 入口处验证
    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
    if (isMeasuringScaleCalced == false) return RESECTION_MEASURE_SCALE_NOT_CALCULATED;

    Mat tempATA(nUnknownNum, nUnknownNum, CV_64FC1, 0.0);
    Mat tempATl(nUnknownNum, 1, CV_64FC1, 0.0);
    Mat Ai(2, nUnknownNum, CV_64FC1, 0.0);
    Mat li(2, 1, CV_64FC1, 0.0);
    // 逐点法化
    for (int i = 0; i < m_nGcpNum; i++)
    {
        // 计算某点的法方程系数
        BOOL calcAResult = Calc_A_Matrix(i, Ai);
        if (calcAResult != RESECTION_SUCCESS)
        {
            return calcAResult;
        }
        BOOL calclResult = Calc_l_Matrix(i, li);
        // 加到 ATA 和 ATl 矩阵中
        tempATA = tempATA + (Ai.t() * Ai);
        tempATl = tempATl + (Ai.t() * li);
    }

    ATA = tempATA;
    ATl = tempATl;

    return RESECTION_SUCCESS;
}


/**
 * 计算外方位元素的初始值
 * lf_varphi: 指定的 varphi 的值
 * lf_omega: 指定的 omega 的值
 * lf_kappa: 指定的 kappa 的值
 */
BOOL CSpaceResection::InitExteriorElements(double lf_varphi, double lf_omega, double lf_kappa)
{
    // 入口处检验
    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
    
    // 计算外方位线元素的初始值
    InitExteriorLineElements();

    // 计算外方位角元素
    varphi = lf_varphi;
    omega = lf_omega;
    kappa = (lf_kappa == 0.0) ? CalcKappa() : lf_kappa;
    // 设置外方位元素初始值已计算标志
    isExteriorInited = true;

    // 计算旋转矩阵
    Calc_R_Matrix();

    return RESECTION_SUCCESS;
}


/**
 * 单像空间后方交会
 */
BOOL CSpaceResection::CalcExteriorElements()
{
    // 入口处检验
    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
    if (isExteriorInited == false) return RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED;
    
    bool isOk = false; // 记录精度
    Mat deltaX(nUnknownNum, 1, CV_64FC1, 0.0); // 记录外方位元素改正数

    while (!isOk)
    {
        // 求 ATA Atl 矩阵
        BOOL calcATAResult = Calc_ATA_ATl_Matrix();
        if (calcATAResult != RESECTION_SUCCESS) return calcATAResult;
        // 计算 deltaX
        solve(ATA, ATl, deltaX, CV_LU);
        //deltaX = ATA.inv() * ATl;
        isOk = isReachDemand(deltaX);
        // 修正外方位元素
        BOOL correctExteriorResults = CorrectExteriorElements(deltaX);
        if (correctExteriorResults != RESECTION_SUCCESS) return correctExteriorResults;
    }
    dx = deltaX;

    return RESECTION_SUCCESS;
}


/**
* 单像空间后方交会解算函数
*/
BOOL CSpaceResection::CalcSpaceResection(double lf_kappa)
{
    BOOL result = CheckGcps();
    if (result != RESECTION_SUCCESS)
    {
        return result;
    }
    result = (isExteriorInited) ? InitExteriorElements(0, 0, lf_kappa) : RESECTION_SUCCESS;
    if (result != RESECTION_SUCCESS)
    {
        return result;
    }
    result = CalcExteriorElements();
    if (result != RESECTION_SUCCESS)
    {
        return result;
    }
    return RESECTION_SUCCESS;
}


/**
 * 精度评定
 */
double CSpaceResection::AssessPrecision()
{
    double lTl = 0;
    
    // 分别计算所有的常数项矩阵
    Mat li(2, 1, CV_64FC1, 0.0);
    for (int i = 0; i < m_nGcpNum; i++)
    {
        BOOL CalcLResult = Calc_l_Matrix(i, li);
        if (CalcLResult == RESECTION_SUCCESS)
        {
            double l1 = li.at<double>(0, 0), l2 = li.at<double>(1, 0);
            lTl += (l1 * l1 + l2 * l2);
        }
        else return CalcLResult;
    }

    // 计算评定的精度
    double result = (lTl - ATl.at<double>(0, 0) * dx.at<double>(0, 0));
    return result;
}

/**
 * 输出计算结果。
 * 注意：使用完结果之后需要即使释放内存！
 */
ExteriorElements CSpaceResection::GetExteriroElements()
{
    ExteriorElements exteriorElements = { Xs, Ys, Zs, varphi, omega, kappa };

    return exteriorElements;
}


/**
* 初始化外方位线元素
*/
void CSpaceResection::InitExteriorLineElements()
{
    double lf_SumXs = 0;
    double lf_SumYs = 0;
    double lf_SumZs = 0;
    for (int i = 0; i < m_nGcpNum; i++)
    {
        GroundGcp iGroundGcp = m_vGroundGcp.at(i);
        lf_SumXs += iGroundGcp.X;
        lf_SumYs += iGroundGcp.Y;
        lf_SumZs += iGroundGcp.Z;
    }
    Xs = lf_SumXs / m_nGcpNum;
    Ys = lf_SumYs / m_nGcpNum;
    Zs = lf_SumZs / m_nGcpNum + f * m;
    //Zs = f * m;
}


/**
 * 计算相片旋角 kappa
 * imageGcp1: 第1个影像控制点
 * imageGcp2: 第2个影像控制点
 * groundGcp1: 第1个地面控制点
 * groundGcp2: 第2个地面控制点
 */
double CSpaceResection::CalcKappa(const ImageGcp imageGcp1, const ImageGcp imageGcp2, const GroundGcp groundGcp1, const GroundGcp groundGcp2)
{
    double delta_Y = groundGcp1.Y - groundGcp2.Y;
    double delta_X = groundGcp1.X - groundGcp2.X;
    double delta_y = imageGcp1.y - imageGcp2.y;
    double delta_x = imageGcp1.x - imageGcp2.x;

    double alpha_ground = atan2(delta_Y, delta_X);
    double alpha_image = atan2(delta_y, delta_x);
    
    return (alpha_ground - alpha_image);
}


/**
* 计算相片旋角 kappa
*/
double CSpaceResection::CalcKappa()
{
    double iKappa = 0.0;
    for (int i = 0; i < m_nGcpNum; i++)
    {
        int j = (i + 1) % m_nGcpNum;
        iKappa += CalcKappa(m_vImageGcp.at(i), m_vImageGcp.at(j), m_vGroundGcp.at(i), m_vGroundGcp.at(j));
    }
    return (iKappa / m_nGcpNum);
}


/**
 * 根据计算出来的外方位元素改正值改正外方位元素
 * [RESECTION_SUCCESS] 返回：成功
 * [RESECTION_CORRECT_EXTERIOR_CORRECTING_NUMBERS_NOT_ENOUGH] 错误：改正数个数不足
 * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] 错误：外方位线元素没有初始化
 */
BOOL CSpaceResection::CorrectExteriorElements(const Mat & delta_x)
{
    // 入口处检验
    if (delta_x.rows < 6) return RESECTION_CORRECT_EXTERIOR_CORRECTING_NUMBERS_NOT_ENOUGH;
    if (isExteriorInited == false) return RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED;

    Xs += delta_x.at<double>(0, 0);
    Ys += delta_x.at<double>(1, 0);
    Zs += delta_x.at<double>(2, 0);
    varphi += delta_x.at<double>(3, 0);
    omega += delta_x.at<double>(4, 0);
    kappa += delta_x.at<double>(5, 0);

    switch (nUnknownNum)
    {
        case 11:
            k[1] += delta_x.at<double>(10, 0);
        case 10:
            k[0] += delta_x.at<double>(9, 0);
        case 9:
            f += delta_x.at<double>(6, 0);
            x0 += delta_x.at<double>(7, 0);
            y0 += delta_x.at<double>(8, 0);
        default:
            break;
    }

    BOOL calcRResult = Calc_R_Matrix();
    if (calcRResult != RESECTION_SUCCESS) return calcRResult;

    return RESECTION_SUCCESS;
}

