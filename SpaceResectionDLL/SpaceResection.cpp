// SpaceResectionDLL.cpp : ���� DLL Ӧ�ó���ĵ���������
//

#include "stdafx.h"
#include "SpaceResection.h"


// ���ǵ���������һ��ʾ��
//SPACERESECTIONDLL_API int nSpaceResectionDLL=0;

// ���ǵ���������һ��ʾ����
//SPACERESECTIONDLL_API int fnSpaceResectionDLL(void)
//{
//    return 42;
//}


/**
 * �ռ�󷽽�������๹�캯��
 * lf_x0: ��֪�ڷ�λԪ�� x0
 * lf_y0: ��֪�ڷ�λԪ�� y0
 * lf_f: ��֪���� f
 * lf_m: ��֪��������� m
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
 * ��ӿ��Ƶ��
 * xi: Ӱ����Ƶ�����ƽ������ϵ�е� x ����
 * yi: Ӱ����Ƶ�����ƽ������ϵ�е� y ����
 * Xi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� X ����
 * Yi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� Y ����
 * Zi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� Z ����
 */
void CSpaceResection::AddGcp(double xi, double yi, double Xi, double Yi, double Zi)
{
    // ��ӿ��Ƶ��
    ImageGcp imageGcp = { xi, yi };
    GroundGcp groundGcp = { Xi, Yi, Zi };
    m_vImageGcp.push_back(imageGcp);
    m_vGroundGcp.push_back(groundGcp);
    m_nGcpNum++;
}


/**
 * ɾ��ָ��λ�õĿ��Ƶ��
 * i: ָ��ɾ����λ��
 * [RESECTION_SUCCESS] ���أ��ɹ�
 * [RESECTION_INDEX_OUT_OF_RANGE] ���������������鷶Χ
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
 * �޸�ָ��λ�õĿ��Ƶ�����
 * i: ָ���Ŀ��Ƶ��λ��
 * xi: Ӱ����Ƶ�����ƽ������ϵ�е� x ����
 * yi: Ӱ����Ƶ�����ƽ������ϵ�е� y ����
 * Xi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� X ����
 * Yi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� Y ����
 * Zi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� Z ����
 * [RESECTION_SUCCESS] ���أ��ɹ�
 * [RESECTION_INDEX_OUT_OF_RANGE] ���������������鷶Χ
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
 * �����Ƶ��Ƿ����Ҫ��
 * Ӱ����Ƶ㲻����ͬһ��ֱ����
 * ������Ƶ㲻����ͬһ��Բ������
 */
BOOL CSpaceResection::CheckGcps()
{
    // ������Ƶ�����������Ķԣ����ش���
    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
    
    // Ӱ����Ƶ㲻����ͬһ��ֱ���ϼ��
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
 * ���Ӱ����Ƶ��Ƿ���ͬһ��ֱ����
 * [RESECTION_SUCCESS] �ɹ�
 * [RESECTION_SOME_GCP_IN_ONE_LINE] �����㹲��
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
 * ����͹�����
 * groundGcp: �����ı��ε��ĸ�����㡣
 * result: ������飬�洢˳��
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
 * �����ܶ�����֤
 * groundGcp: �����ı��ε��ĸ�����㡣
 * arrangement: ����˳��
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
 * ��������Ƶ��Ƿ���ͬһ��Բ������
 * ���������ܶ���
 * Բ�ڽ�͹�ı������ԶԱߵĳ˻�֮�͵��ڶԽ��ߵĳ˻�
 */
BOOL CSpaceResection::InCylindricalSurface()
{
    GroundGcp* vGroundGcp = m_vGroundGcp.data();
    // ѡȡ�ĸ����ĸ���������
    for (int i = 0; i < m_nGcpNum - 3; i++)
    {
        for (int j = i + 1; j < m_nGcpNum - 2; j++)
        {
            for (int m = j + 1; m < m_nGcpNum - 1; m++)
            {
                for (int n = m + 1; n < m_nGcpNum; n++)
                {
                    // �����ı�������
                    GroundGcp groundGcp[4] = {vGroundGcp[i], vGroundGcp[j], vGroundGcp[m], vGroundGcp[n]};
                    // ���ٽ���洢����
                    int arrangement[4] = { 0 };
                    // ����͹�ı���
                    if (makeConvexQuadrangle(groundGcp, arrangement))
                    {
                        // �����ܶ�����֤
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
 * ������Ӱ�����ߣ����ƴ�ֱ������Ӱ��
 * [RESECTION_SUCCESS] �ɹ�������Ӱ������
 * [RESECTION_GCP_NOT_ENOUGH] ���Ƶ���������
 */
BOOL CSpaceResection::CalcMeasuringScale()
{
    // ��ڴ�����
    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
    
    // ��ȡһ��Ӱ����Ƶ�͵�����Ƶ�
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
 * ������ת����
 * [RESECTION_SUCCESS] �ɹ�������ת����
 * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] �����ⷽλԪ��û�г�ʼ��
 * [RESECTION_ROTATE_MATRIX_IS_NOT_3_3_MATRIX] ������ת����Ϊ�ջ���ת������3*3����
 */
BOOL CSpaceResection::Calc_R_Matrix()
{
    // ��ڴ�����
    // �����ⷽλԪ���Ƿ��Ѿ���ʼ��
    if (isExteriorInited == false) return RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED;
    if ((R.empty()) || (R.rows != 3) || (R.cols != 3)) return RESECTION_ROTATE_MATRIX_IS_NOT_3_3_MATRIX;

    // Ԥ�ȼ��������
    double sinVarphi = sin(varphi); double sinOmega = sin(omega); double sinKappa = sin(kappa);
    double cosVarphi = cos(varphi); double cosOmega = cos(omega); double cosKappa = cos(kappa);
    // ������ת����
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
 * ����ĳ�Կ��Ƶ�ķ�����ϵ������ A
 * i: Ҫ���� A ����Ŀ��Ƶ������
 * A: �洢����� A ����
 * [RESECTION_SUCCESS] ���أ��ɹ�����
 * [RESECTION_INDEX_OUT_OF_RANGE] ���������������鷶Χ
 * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] �����ⷽλԪ��û�м����ֵ
 */
BOOL CSpaceResection::Calc_A_Matrix(int i, Mat& A)
{
    /**************
    /* ��ȡһЩ����
    /**************/
    // ��ȡ��������
    const size_t params = A.cols;
    // ��ȡ���Ƶ㣬���� const ������
    const ImageGcp iImageGcp = m_vImageGcp.at(i);
    const GroundGcp iGroundGcp = m_vGroundGcp.at(i);
    // ��ȡ������Ƶ�����
    const double x = iImageGcp.x;
    const double y = iImageGcp.y;
    // ��ȡ��ת�����е�Ԫ��
    const double 
        a1 = R.at<double>(0, 0), a2 = R.at<double>(0, 1), a3 = R.at<double>(0, 2),
        b1 = R.at<double>(1, 0), b2 = R.at<double>(1, 1), b3 = R.at<double>(1, 2),
        c1 = R.at<double>(2, 0), c2 = R.at<double>(2, 1), c3 = R.at<double>(2, 2);
    // ���� Z_bar
    double Z_bar = a3*(iGroundGcp.X - Xs) + b3 * (iGroundGcp.Y - Ys) + c3* (iGroundGcp.Z - Zs);
    // Ԥ�ȼ��������
    double Dx = x - x0, Dy = y - y0;
    double r2 = Dx*Dx + Dy*Dy;
    double sinOmega = sin(omega); double sinKappa = sin(kappa);
    double cosOmega = cos(omega); double cosKappa = cos(kappa);
    /***************************
    ** ����6���ⷽλԪ�ص�ƫ����
    ****************************/
    // ����6���ⷽλ��Ԫ�ص�ƫ����
    A.at<double>(0, 0) = (a1*f + a3*Dx) / Z_bar;
    A.at<double>(0, 1) = (b1*f + b3*Dx) / Z_bar;
    A.at<double>(0, 2) = (c1*f + c3*Dx) / Z_bar;
    A.at<double>(1, 0) = (a2*f + a3*Dy) / Z_bar;
    A.at<double>(1, 1) = (b2*f + b3*Dy) / Z_bar;
    A.at<double>(1, 2) = (c2*f + c3*Dy) / Z_bar;
    // ����6���ⷽλ��Ԫ�ص�ƫ����
    A.at<double>(0, 3) = Dy * sinOmega - (Dx*(Dx*cosKappa - Dy*sinKappa) / f + f*cosKappa)*cosOmega;
    A.at<double>(0, 4) = -f*sinKappa - Dx*(Dx*sinKappa + Dy*cosKappa) / f;
    A.at<double>(0, 5) = Dy;
    A.at<double>(1, 3) = -Dx*sinOmega - (Dy*(Dx*cosKappa - Dy*sinKappa) / f - f*sinKappa)*cosOmega;
    A.at<double>(1, 4) = -f*cosKappa - Dy*(Dx*sinKappa + Dy*cosKappa) / f;
    A.at<double>(1, 5) = -Dx;
    /****************
    ** ��������ƫ����
    *****************/
    switch (params)
    {
        case 11:
            // ������׻��������ƫ����
            A.at<double>(0, 10) = (Dx)*r2;
            A.at<double>(1, 10) = (Dy)*r2;
        case 10:
            // ����һ�׻������ƫ����
            A.at<double>(0, 9) = (Dx)*sqrt(r2);
            A.at<double>(1, 9) = (Dy)*sqrt(r2);
        case 9:
            // ����3���ڷ�λԪ�ص�ƫ����
            A.at<double>(0, 6) = (Dx / f);
            A.at<double>(0, 7) = 1;
            A.at<double>(0, 8) = 0;
            A.at<double>(1, 6) = (Dy / f);
            A.at<double>(1, 7) = 0;
            A.at<double>(1, 8) = 1;
        default:
            break;
    }

    // ���سɹ�����
    return RESECTION_SUCCESS;
}


/**
 * ����ĳ�Կ��Ƶ�ĳ�����ϵ������ l
 * i: Ҫ���� A ����Ŀ��Ƶ������
 * l: �洢����� l ����
 * [RESECTION_SUCCESS] ���أ��ɹ�����
 * [RESECTION_INDEX_OUT_OF_RANGE] ���������������鷶Χ
 * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] �����ⷽλԪ��û�м����ֵ
 */
BOOL CSpaceResection::Calc_l_Matrix(int i, Mat & l)
{
    // ��ȡ���Ƶ㣬���� const ������
    const ImageGcp iImageGcp = m_vImageGcp.at(i);
    const GroundGcp iGroundGcp = m_vGroundGcp.at(i);
    // ��ȡ���Ƶ�����
    const double x = iImageGcp.x; const double y = iImageGcp.y;
    const double X = iGroundGcp.X; const double Y = iGroundGcp.Y; const double Z = iGroundGcp.Z;
    // ��ȡ��ת�����е�Ԫ��
    const double a1 = R.at<double>(0, 0), a2 = R.at<double>(0, 1), a3 = R.at<double>(0, 2);
    const double b1 = R.at<double>(1, 0), b2 = R.at<double>(1, 1), b3 = R.at<double>(1, 2);
    const double c1 = R.at<double>(2, 0), c2 = R.at<double>(2, 1), c3 = R.at<double>(2, 2);

    // ����X_bar, Y_Bar, Z_bar����
    double X_bar = a1*(X - Xs) + b1 * (Y - Ys) + c1* (Z - Zs);
    double Y_bar = a2*(X - Xs) + b2 * (Y - Ys) + c2* (Z - Zs);
    double Z_bar = a3*(X - Xs) + b3 * (Y - Ys) + c3* (Z - Zs);

    // ���� ������� ��ʼֵ
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
    // ���� l ����
    l.at<double>(0, 0) = x - (x_origin + x0 + dx);
    l.at<double>(1, 0) = y - (y_origin + y0 + dy);

    return RESECTION_SUCCESS;
}


/**
 * ��㷨�������� ATA ����
 * [RESECTION_SUCCESS] ���أ��ɹ����� ATA ����
 * [RESECTION_CALC_ATA_GCP_NOT_ENOUGH] ���󣺿��Ƶ���������
 */
//BOOL CSpaceResection::Calc_ATA_Matrix()
//{
//    // ��ڴ���֤
//    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
//
//    Mat tempATA(6, 6, CV_64FC1, 0.0);
//    Mat Ai(2, 6, CV_64FC1, 0.0);
//    // ��㷨��
//    for (int i = 0; i < m_nGcpNum; i++)
//    {
//        // ����ĳ��ķ�����ϵ��
//        BOOL calcAResult = Calc_A_Matrix(i, Ai);
//        if (calcAResult != RESECTION_SUCCESS)
//        {
//            return calcAResult;
//        }
//        // �ӵ� ATA������
//        ATA = ATA + (Ai.t() * Ai);
//    }
//
//    return RESECTION_SUCCESS;
//}


/**
 * ��㷨�������� ATl ����
 * [RESECTION_SUCCESS] ���أ��ɹ����� ATl ����
 * [RESECTION_CALC_ATA_GCP_NOT_ENOUGH] ���󣺿��Ƶ���������
 */
//BOOL CSpaceResection::Calc_ATl_Matrix()
//{
//    // ��ڴ���֤
//    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
//
//    Mat tempATA(6, 6, CV_64FC1, 0.0);
//    Mat tempATl(6, 1, CV_64FC1, 0.0);
//    Mat Ai(2, 6, CV_64FC1, 0.0);
//    Mat li(2, 1, CV_64FC1, 0.0);
//    // ��㷨��
//    for (int i = 0; i < m_nGcpNum; i++)
//    {
//        // ����ĳ��ķ�����ϵ��
//        BOOL calcAResult = Calc_A_Matrix(i, Ai);
//        if (calcAResult != RESECTION_SUCCESS)
//        {
//            return calcAResult;
//        }
//        BOOL calclResult = Calc_l_Matrix(i, li);
//        // �ӵ� ATl ������
//        ATl = ATl + (Ai.t() * li);
//    }
//
//    return RESECTION_SUCCESS;
//}


/**
* ��㷨�������� ATA ATl ����
* [SUCCESS] ���أ��ɹ����� ATl ����
* [RESECTION_CALC_ATA_GCP_NOT_ENOUGH] ���󣺿��Ƶ���������
*/
BOOL CSpaceResection::Calc_ATA_ATl_Matrix()
{
    // ��ڴ���֤
    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
    if (isMeasuringScaleCalced == false) return RESECTION_MEASURE_SCALE_NOT_CALCULATED;

    Mat tempATA(nUnknownNum, nUnknownNum, CV_64FC1, 0.0);
    Mat tempATl(nUnknownNum, 1, CV_64FC1, 0.0);
    Mat Ai(2, nUnknownNum, CV_64FC1, 0.0);
    Mat li(2, 1, CV_64FC1, 0.0);
    // ��㷨��
    for (int i = 0; i < m_nGcpNum; i++)
    {
        // ����ĳ��ķ�����ϵ��
        BOOL calcAResult = Calc_A_Matrix(i, Ai);
        if (calcAResult != RESECTION_SUCCESS)
        {
            return calcAResult;
        }
        BOOL calclResult = Calc_l_Matrix(i, li);
        // �ӵ� ATA �� ATl ������
        tempATA = tempATA + (Ai.t() * Ai);
        tempATl = tempATl + (Ai.t() * li);
    }

    ATA = tempATA;
    ATl = tempATl;

    return RESECTION_SUCCESS;
}


/**
 * �����ⷽλԪ�صĳ�ʼֵ
 * lf_varphi: ָ���� varphi ��ֵ
 * lf_omega: ָ���� omega ��ֵ
 * lf_kappa: ָ���� kappa ��ֵ
 */
BOOL CSpaceResection::InitExteriorElements(double lf_varphi, double lf_omega, double lf_kappa)
{
    // ��ڴ�����
    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
    
    // �����ⷽλ��Ԫ�صĳ�ʼֵ
    InitExteriorLineElements();

    // �����ⷽλ��Ԫ��
    varphi = lf_varphi;
    omega = lf_omega;
    kappa = (lf_kappa == 0.0) ? CalcKappa() : lf_kappa;
    // �����ⷽλԪ�س�ʼֵ�Ѽ����־
    isExteriorInited = true;

    // ������ת����
    Calc_R_Matrix();

    return RESECTION_SUCCESS;
}


/**
 * ����ռ�󷽽���
 */
BOOL CSpaceResection::CalcExteriorElements()
{
    // ��ڴ�����
    if (m_nGcpNum < 4) return RESECTION_GCP_NOT_ENOUGH;
    if (isExteriorInited == false) return RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED;
    
    bool isOk = false; // ��¼����
    Mat deltaX(nUnknownNum, 1, CV_64FC1, 0.0); // ��¼�ⷽλԪ�ظ�����

    while (!isOk)
    {
        // �� ATA Atl ����
        BOOL calcATAResult = Calc_ATA_ATl_Matrix();
        if (calcATAResult != RESECTION_SUCCESS) return calcATAResult;
        // ���� deltaX
        solve(ATA, ATl, deltaX, CV_LU);
        //deltaX = ATA.inv() * ATl;
        isOk = isReachDemand(deltaX);
        // �����ⷽλԪ��
        BOOL correctExteriorResults = CorrectExteriorElements(deltaX);
        if (correctExteriorResults != RESECTION_SUCCESS) return correctExteriorResults;
    }
    dx = deltaX;

    return RESECTION_SUCCESS;
}


/**
* ����ռ�󷽽�����㺯��
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
 * ��������
 */
double CSpaceResection::AssessPrecision()
{
    double lTl = 0;
    
    // �ֱ�������еĳ��������
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

    // ���������ľ���
    double result = (lTl - ATl.at<double>(0, 0) * dx.at<double>(0, 0));
    return result;
}

/**
 * �����������
 * ע�⣺ʹ������֮����Ҫ��ʹ�ͷ��ڴ棡
 */
ExteriorElements CSpaceResection::GetExteriroElements()
{
    ExteriorElements exteriorElements = { Xs, Ys, Zs, varphi, omega, kappa };

    return exteriorElements;
}


/**
* ��ʼ���ⷽλ��Ԫ��
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
 * ������Ƭ���� kappa
 * imageGcp1: ��1��Ӱ����Ƶ�
 * imageGcp2: ��2��Ӱ����Ƶ�
 * groundGcp1: ��1��������Ƶ�
 * groundGcp2: ��2��������Ƶ�
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
* ������Ƭ���� kappa
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
 * ���ݼ���������ⷽλԪ�ظ���ֵ�����ⷽλԪ��
 * [RESECTION_SUCCESS] ���أ��ɹ�
 * [RESECTION_CORRECT_EXTERIOR_CORRECTING_NUMBERS_NOT_ENOUGH] ���󣺸�������������
 * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] �����ⷽλ��Ԫ��û�г�ʼ��
 */
BOOL CSpaceResection::CorrectExteriorElements(const Mat & delta_x)
{
    // ��ڴ�����
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

