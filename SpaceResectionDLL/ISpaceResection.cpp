#include "stdafx.h"
#include "ISpaceResection.h"

#include "SpaceResection.h"
/**
* �ռ�󷽽�������๹�캯��
* lf_x0: ��֪�ڷ�λԪ�� x0
* lf_y0: ��֪�ڷ�λԪ�� y0
* lf_f: ��֪���� f
* lf_m: ��֪��������� m
*/
ISpaceResection * ISpaceResection::create(double lf_x0, double lf_y0, double lf_f, double lf_m, bool inneriorApprox, size_t calibParamNums)
{
    return (ISpaceResection*)(new CSpaceResection(lf_x0, lf_y0, lf_f, lf_m, inneriorApprox, calibParamNums));
}

//void ISpaceResection::operator delete(void * instance)
//{
//    delete ((CSpaceResection*)instance);
//}

void ISpaceResection::release(ISpaceResection * instance)
{
    delete ((CSpaceResection*)instance);
}
