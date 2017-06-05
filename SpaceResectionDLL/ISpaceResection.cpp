#include "stdafx.h"
#include "ISpaceResection.h"

#include "SpaceResection.h"
/**
* 空间后方交会计算类构造函数
* lf_x0: 已知内方位元素 x0
* lf_y0: 已知内方位元素 y0
* lf_f: 已知焦距 f
* lf_m: 已知航摄比例尺 m
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
