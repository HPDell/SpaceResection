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

// �����Ǵ� SpaceResectionDLL.dll ������
/**
 * ����ռ�󷽽�������࣬�ṩ����Ӱ���ⷽλԪ�صķ�����
 * ʹ��ʱ�������� �ڷ�λԪ�� ��������
 * ��������Ƶ�Ե����꣬
 * Ȼ��ʹ�÷������� ��Ӱ������
 * ����ʹ�÷�����ʼ�� �ⷽλԪ��
 * ���ʹ�÷����� �ⷽλԪ�� ���� ƽ�����
 * �������Ϳ������ �ⷽλԪ�� ���� ��������
 */
class SPACERESECTIONDLL_API CSpaceResection : public ISpaceResection {
private: 
    int m_nGcpNum; // ���Ƶ�Ե�����
    vector<ImageGcp> m_vImageGcp; // Ӱ����Ƶ�����
    vector<GroundGcp> m_vGroundGcp; // ������Ƶ�����
    // ����Ϊ 6 ���ⷽλԪ��
    double Xs; // Xs: ��Ӱ�����ڵ�����Ӱ��������ϵ�е� X ����
    double Ys; // Ys: ��Ӱ�����ڵ�����Ӱ��������ϵ�е� Y ����
    double Zs; // Zs: ��Ӱ�����ڵ�����Ӱ��������ϵ�е� Z ����
    double varphi; // varphi: ��ռ�����ϵ�� Y �����ת��
    double omega; // omega: ��ռ�����ϵ�� Y �����ת��
    double kappa; // kappa: ��ռ�����ϵ�� Z �����ת��
    // ����Ϊ 3 ���ڷ�λԪ��
    double x0; // x0: ����������ռ�����ϵ�е� x ����
    double y0; // y0: ����������ռ�����ϵ�е� y ����
    double f; // f: �����ǵĽ���
    double m; // m: ��Ӱ������
    Mat R; // ��ת����
    // ����Ϊ�������
    size_t nCalibParams;
    double* k;
    // ����Ϊ�������
    Mat ATA; // ������ʽϵ������
    Mat ATl; // ������ʽ���������
    Mat dx; // ������
    // ������������
    size_t nUnknownNum;
    bool isInneriorApprox; // �ڷ�λԪ���Ƿ�Ϊ����ֵ
    bool isSelfCalibNeed;  // �Ƿ���Ҫ�������ϵ��
    bool isExteriorInited; // �ⷽλԪ���Ƿ��Ѽ����ֵ
    bool isMeasuringScaleCalced; // �������Ƿ񱻼���

public:
    /**
    * �ռ�󷽽�������๹�캯��
    * lf_x0: ��֪�ڷ�λԪ�� x0
    * lf_y0: ��֪�ڷ�λԪ�� y0
    * lf_f: ��֪���� f
    * lf_m: ��֪��������� m
    */
	CSpaceResection(double lf_x0, double lf_y0, double lf_f, double lf_m, bool inneriorApprox = false, size_t calibParamNums = 0);
    /*
    * ��������
    */
    ~CSpaceResection();
	/**
    * ��ȡ���Ƶ������
    */
    virtual const int getGcpNum() { return m_nGcpNum; }
    /**
    * ��ӿ��Ƶ��
    * xi: Ӱ����Ƶ�����ƽ������ϵ�е� x ���꣬��λ��mm
    * yi: Ӱ����Ƶ�����ƽ������ϵ�е� y ���꣬��λ��mm
    * Xi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� X ����
    * Yi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� Y ����
    * Zi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� Z ����
    */
    virtual void AddGcp(double xi, double yi, double Xi, double Yi, double Zi);
    /**
    * ɾ��ָ��λ�õĿ��Ƶ��
    * i: ָ��ɾ����λ��
    */
    virtual BOOL DeleteGcp(int i);
    /**
    * �޸�ָ��λ�õĿ��Ƶ�����
    * i: ָ���Ŀ��Ƶ��λ��
    * xi: Ӱ����Ƶ�����ƽ������ϵ�е� x ���꣬��λ��mm
    * yi: Ӱ����Ƶ�����ƽ������ϵ�е� y ���꣬��λ��mm
    * Xi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� X ����
    * Yi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� Y ����
    * Zi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� Z ����
    */
    virtual BOOL UpdateGcp(int i, double xi, double yi, double Xi, double Yi, double Zi);
    /**
    * �����Ƶ��Ƿ����Ҫ��
    */
    virtual BOOL CheckGcps();
    /**
     * ������Ӱ�����ߣ����ƴ�ֱ������Ӱ��
     * [RESECTION_SUCCESS] �ɹ�������Ӱ������
     * [RESECTION_GCP_NOT_ENOUGH] ���Ƶ���������
     */
    virtual BOOL CalcMeasuringScale();
    /**
    * �����ⷽλԪ�صĳ�ʼֵ
    * lf_varphi: ָ���� varphi ��ֵ
    * lf_omega: ָ���� omega ��ֵ
    * lf_kappa: ָ���� kappa ��ֵ
    */
    virtual BOOL InitExteriorElements(double lf_varphi = 0.0, double lf_omega = 0.0, double lf_kappa = 0.0);
    /**
    * ���������ⷽλԪ��
    */
    virtual BOOL CalcExteriorElements();
    /**
    * ����ռ�󷽽������
    */
    virtual BOOL CalcSpaceResection(double lf_kappa = 0.0);
    /**
    * ����������
    */
    virtual double AssessPrecision();
    /**
    * ���������
    */
    virtual ExteriorElements GetExteriroElements();
    /**
    * �����ת����
    */
    virtual double* GetRotateMatrix() { return (double*)R.data; }
    /**
    * ����ڷ�λԪ��
    */
    virtual InneriorElements GetInneriorElements()
    {
        InneriorElements inneriorElements = { f, x0, y0 };
        return inneriorElements;
    }
    /**
    * �������ϵ��
    **/
    virtual void GetCalibParams(double* calibParams)
    {
        for (size_t i = 0; i < nCalibParams; i++)
        {
            calibParams[i] = k[i];
        }
    }
    /**
    * ������в�
    **/
    void CalcImagePointResidual(double* pResidual);
private: 
    /**
    * ���Ӱ����Ƶ��Ƿ���ͬһ��ֱ����
    */
    BOOL InLineCheck();
    BOOL InCylindricalSurface();
    /**
    * ������ת����
    * [RESECTION_SUCCESS] �ɹ�������ת����
    * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] �����ⷽλԪ��û�г�ʼ��
    * [RESECTION_ROTATE_MATRIX_IS_NOT_3_3_MATRIX] ������ת����Ϊ�ջ���ת������3*3����
    */
    BOOL Calc_R_Matrix();
    /**
    * ����ĳ�Կ��Ƶ�ķ�����ϵ������ A
    * i: Ҫ���� A ����Ŀ��Ƶ������
    * A: �洢����� A ����
    * [RESECTION_SUCCESS] ���أ��ɹ�����
    * [RESECTION_INDEX_OUT_OF_RANGE] ���������������鷶Χ
    * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] �����ⷽλԪ��û�м����ֵ
    */
    BOOL Calc_A_Matrix(int i, Mat& A);
    /**
    * ����ĳ�Կ��Ƶ�ĳ�����ϵ������ l
    * i: Ҫ���� A ����Ŀ��Ƶ������
    * l: �洢����� l ����
    * [RESECTION_SUCCESS] ���أ��ɹ�����
    * [RESECTION_INDEX_OUT_OF_RANGE] ���������������鷶Χ
    * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] �����ⷽλԪ��û�м����ֵ
    */
    BOOL Calc_l_Matrix(int i, Mat& l);
    /**
    * ��㷨�������� ATA ����
    * [RESECTION_SUCCESS] ���أ��ɹ����� ATA ����
    * [RESECTION_CALC_ATA_GCP_NOT_ENOUGH] ���󣺿��Ƶ���������
    */
    //BOOL Calc_ATA_Matrix();
    /**
    * ��㷨�������� ATl ����
    * [RESECTION_SUCCESS] ���أ��ɹ����� ATl ����
    * [RESECTION_CALC_ATA_GCP_NOT_ENOUGH] ���󣺿��Ƶ���������
    */
    //BOOL Calc_ATl_Matrix();
    /**
    * ��㷨�������� ATA ATl ����
    * [SUCCESS] ���أ��ɹ����� ATl ����
    * [RESECTION_CALC_ATA_GCP_NOT_ENOUGH] ���󣺿��Ƶ���������
    */
    BOOL Calc_ATA_ATl_Matrix();
    /**
    * ������Ƭ��ƫ��
    */
    double CalcKappa();
    /**
    * ������Ƭ���� kappa
    * imageGcp1: ��1��Ӱ����Ƶ�
    * imageGcp2: ��2��Ӱ����Ƶ�
    * groundGcp1: ��1��������Ƶ�
    * groundGcp2: ��2��������Ƶ�
    */
    double CalcKappa(ImageGcp imageGcp1, ImageGcp imageGcp2, GroundGcp groundGcp1, GroundGcp groundGcp2);
    /**
    * ���ݼ���������ⷽλԪ�ظ���ֵ�����ⷽλԪ��
    * [RESECTION_SUCCESS] ���أ��ɹ�
    * [RESECTION_CORRECT_EXTERIOR_CORRECTING_NUMBERS_NOT_ENOUGH] ���󣺸�������������
    * [RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED] �����ⷽλ��Ԫ��û�г�ʼ��
    */
    BOOL CorrectExteriorElements(const Mat& delta_x);
    /**
    * ��ʼ���ⷽλ��Ԫ��
    */
    void InitExteriorLineElements(); 
    /**
    * �ж��Ƿ�����Ҫ��
    * deltaX: ��������ĸ�����
    */
    bool isReachDemand(const Mat& deltaX)
    {
        // ���޲�ĽǶ���ֵת��Ϊ������
        const double pi = 3.1415926535897932384626433832795, varepsilon = 0.1 / 60 / 180 * pi;
        // ��Ԫ�ظ���ֵ�޲�
        double deltaVarphi = abs(deltaX.at<double>(3, 0)),
            deltaOmega = abs(deltaX.at<double>(4, 0)),
            deltaKappa = abs(deltaX.at<double>(5, 0));
        if ((deltaVarphi < varepsilon) && (deltaOmega < varepsilon) && (deltaKappa < varepsilon)) return true;
        return false;
    }
};
