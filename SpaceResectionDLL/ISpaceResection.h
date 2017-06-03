#pragma once

#include "DataTypes.h"

class SPACERESECTIONDLL_API ISpaceResection
{
// **********
// ����������
// **********
public:
    /**
    * �ռ�󷽽�������๹�캯��
    * lf_x0: ��֪�ڷ�λԪ�� x0
    * lf_y0: ��֪�ڷ�λԪ�� y0
    * lf_f: ��֪���� f
    * lf_m: ��֪��������� m
    */
    static ISpaceResection* create(double lf_x0, double lf_y0, double lf_f, double lf_m, bool inneriorApprox = false, size_t calibParamNums = 0);
    /*
    * ��������
    */
    void operator delete(void* instance);
// ****
// ����
// ****
public:
    /**
    * ��ȡ���Ƶ������
    */
    virtual const int getGcpNum() = 0;
    /**
    * ��ӿ��Ƶ��
    * xi: Ӱ����Ƶ�����ƽ������ϵ�е� x ���꣬��λ��mm
    * yi: Ӱ����Ƶ�����ƽ������ϵ�е� y ���꣬��λ��mm
    * Xi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� X ����
    * Yi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� Y ����
    * Zi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� Z ����
    */
    virtual void AddGcp(double xi, double yi, double Xi, double Yi, double Zi) = 0;
    /**
    * ɾ��ָ��λ�õĿ��Ƶ��
    * i: ָ��ɾ����λ��
    */
    virtual int DeleteGcp(int i) = 0;
    /**
    * �޸�ָ��λ�õĿ��Ƶ�����
    * i: ָ���Ŀ��Ƶ��λ��
    * xi: Ӱ����Ƶ�����ƽ������ϵ�е� x ���꣬��λ��mm
    * yi: Ӱ����Ƶ�����ƽ������ϵ�е� y ���꣬��λ��mm
    * Xi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� X ����
    * Yi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� Y ����
    * Zi: ������Ƶ��ڵ�����Ӱ��������ϵ�ص� Z ����
    */
    virtual int UpdateGcp(int i, double xi, double yi, double Xi, double Yi, double Zi) = 0;
    /**
    * �����Ƶ��Ƿ����Ҫ��
    */
    virtual int CheckGcps() = 0;
    /**
    * ������Ӱ�����ߣ����ƴ�ֱ������Ӱ��
    * [RESECTION_SUCCESS] �ɹ�������Ӱ������
    * [RESECTION_GCP_NOT_ENOUGH] ���Ƶ���������
    */
    virtual int CalcMeasuringScale() = 0;
    /**
    * �����ⷽλԪ�صĳ�ʼֵ
    * lf_varphi: ָ���� varphi ��ֵ
    * lf_omega: ָ���� omega ��ֵ
    * lf_kappa: ָ���� kappa ��ֵ
    */
    virtual int InitExteriorElements(double lf_varphi, double lf_omega, double lf_kappa) = 0;
    /**
    * ���������ⷽλԪ��
    */
    virtual int CalcExteriorElements() = 0;
    /**
    * ����ռ�󷽽������
    */
    virtual int CalcSpaceResection(double lf_kappa = 0.0) = 0;
    /**
    * ����������
    */
    virtual double AssessPrecision() = 0;
    /**
    * ���������
    */
    virtual ExteriorElements GetExteriroElements() = 0;
    /**
    * �����ת����
    */
    virtual double* GetRotateMatrix() = 0;
};

