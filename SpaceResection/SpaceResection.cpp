// SpaceResection.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include "SpaceResectionDLL.h"
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;

int main(size_t argc, char** argv)
{
    /*********
    * ��������
    **********/
    if (argc < 2)
    {
        cout << "û���ڷ�λԪ���ļ�" << endl;
        return -1;
    }
    double x0, y0, f, pixelsize, m;
    ifstream fin_iee(argv[1]);
    fin_iee >> x0 >> y0 >> f >> pixelsize >> m;
    ISpaceResection* spaceResection = ISpaceResection::create(x0 / pixelsize, y0 / pixelsize, f, m, true, 2); // x0=y0=0, f=153.24mm
    fin_iee.close();
    /***********
    * ��ӿ��Ƶ�
    ************/
    if (argc < 3)
    {
        cout << "û�п��Ƶ��ļ�" << endl;
        return -1;
    }
    ifstream fin_gcp(argv[2]);
    if (!fin_gcp.is_open())
    {
        cout << "�򿪿��Ƶ��ļ�ʧ��" << endl;
        return -1;
    }
    int nGcpNum = 0; // ���Ƶ�����
    fin_gcp >> nGcpNum;
    for (size_t i = 0; i < nGcpNum && !fin_gcp.eof(); i++)
    {
        size_t gcp_id;
        double gcp_coord[5];
        fin_gcp >> gcp_id >> gcp_coord[0] >> gcp_coord[1] >> gcp_coord[2] >> gcp_coord[3] >> gcp_coord[4];
        spaceResection->AddGcp(gcp_coord[0] / pixelsize, gcp_coord[1] / pixelsize, gcp_coord[2], gcp_coord[3], gcp_coord[4]);
    }
    fin_gcp.close();
    /*************
    * �󷽽������
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
                printf_s("�����������鷶Χ��");
                break;
            case RESECTION_EXTERIOR_ELEMENTS_IS_NOT_INITED :
                printf_s("�ⷽλԪ��û�г�ʼ����");
                break;
            case RESECTION_GCP_NOT_ENOUGH :
                printf_s("���Ƶ��������㣡");
                break;
            case RESECTION_ROTATE_MATRIX_IS_NOT_3_3_MATRIX :
                printf_s("��ת������3��3�ľ���");
                break;
            case RESECTION_MEASURE_SCALE_NOT_CALCULATED :
                printf_s("������û�м��㣡");
                break;
            case RESECTION_CALC_A_OUT_MATRIX_IS_EMPTY :
                printf_s("�������Ϊ�գ�");
                break;
            case RESECTION_CALC_A_OUT_MATRIX_IS_NOT_A_2_6_MATRIX :
                printf_s("�����A���鲻��2��6���飡");
                break;
            case RESECTION_CALC_A_ROTATE_MATRIX_IS_EMPTY :
                printf_s("��ת����Ϊ�գ�");
                break;
            case RESECTION_CALC_l_OUT_MATRIX_IS_EMPTY :
                printf_s("���l����Ϊ�գ�");
                break;
            case RESECTION_CALC_l_OUT_MATRIX_IS_NOT_A_2_1_MATRIX :
                printf_s("���l���鲻��2��1����");
                break;
            case RESECTION_CALC_l_ROTATE_MATRIX_IS_EMPTY :
                printf_s("��ת����Ϊ�գ�");
                break;
            case RESECTION_CORRECT_EXTERIOR_CORRECTING_NUMBERS_NOT_ENOUGH :
                printf_s("�ⷽλԪ�ظ��������㣡");
                break;
            default:
                printf_s("δ֪����");
                break;
        }
        system("pause");
        return -1;
    }
    /***************
    * ��ȡ�ⷽλԪ��
    ****************/
    ExteriorElements exterior = spaceResection->GetExteriroElements();
    double* R = spaceResection->GetRotateMatrix();
    /*************
    * ���������
    **************/
    cout << "����ռ�󷽽�����" << endl << endl;
    cout << "���в�����" << endl;

    printf_s("����ռ�󷽽����������\n");
    printf_s("Xs = %8.2lf\n", exterior.Xs);
    printf_s("Ys = %8.2lf\n", exterior.Ys);
    printf_s("Zs = %8.2lf\n", exterior.Zs);
    printf_s("R=\n");
    printf_s("\t%9.5lf\t%9.5lf\t%9.5lf\n", R[0], R[1], R[2]);
    printf_s("\t%9.5lf\t%9.5lf\t%9.5lf\n", R[3], R[4], R[5]);
    printf_s("\t%9.5lf\t%9.5lf\t%9.5lf\n", R[6], R[7], R[8]);
    printf_s("���ȣ�%e\n", spaceResection->AssessPrecision());
    system("Pause");
    /*********
    * �ͷŶ���
    **********/
    delete spaceResection;
    return 0;
}

