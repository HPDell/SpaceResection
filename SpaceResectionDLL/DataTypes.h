#pragma once

// ���� ifdef ���Ǵ���ʹ�� DLL �������򵥵�
// ��ı�׼�������� DLL �е������ļ��������������϶���� SPACERESECTIONDLL_EXPORTS
// ���ű���ġ���ʹ�ô� DLL ��
// �κ�������Ŀ�ϲ�Ӧ����˷��š�������Դ�ļ��а������ļ����κ�������Ŀ���Ὣ
// SPACERESECTIONDLL_API ������Ϊ�Ǵ� DLL ����ģ����� DLL ���ô˺궨���
// ������Ϊ�Ǳ������ġ�
#ifdef SPACERESECTIONDLL_EXPORTS
#define SPACERESECTIONDLL_API __declspec(dllexport)
#else
#define SPACERESECTIONDLL_API __declspec(dllimport)
#endif

// �ⷽλԪ�ؽṹ
struct SPACERESECTIONDLL_API ExteriorElements
{
    double Xs; // Xs: ��Ӱ�����ڵ�����Ӱ��������ϵ�е� X ����
    double Ys; // Ys: ��Ӱ�����ڵ�����Ӱ��������ϵ�е� Y ����
    double Zs; // Zs: ��Ӱ�����ڵ�����Ӱ��������ϵ�е� Z ����
    double varphi; // varphi: ��ռ�����ϵ�� Y �����ת��
    double omega; // omega: ��ռ�����ϵ�� Y �����ת��
    double kappa; // kappa: ��ռ�����ϵ�� Z �����ת��
};


/**
* Ӱ����Ƶ�ṹ��
* x: ��ռ�����ϵ�� x ����
* y: ��ռ�����ϴ�� y ����
*/
struct ImageGcp
{
    double  x; // ��ռ�����ϵ�� x ����
    double  y; // ��ռ�����ϵ�� y ����
};


/**
* ������Ƶ�ṹ
* X: ������Ӱ��������ϵ�� X ����
* Y: ������Ӱ��������ϵ�� Y ����
* Z: ������Ӱ��������ϵ�� Z ����
*/
struct GroundGcp
{
    double X; // ������Ӱ��������ϵ�� X ����
    double Y; // ������Ӱ��������ϵ�� Y ����
    double Z; // ������Ӱ��������ϵ�� Z ����
};


/**
* Ӱ����Ƶ�Խṹ��
* x: ��ռ�����ϵ�� x ����
* y: ��ռ�����ϴ�� y ����
* X: ������Ӱ��������ϵ�� X ����
* Y: ������Ӱ��������ϵ�� Y ����
* Z: ������Ӱ��������ϵ�� Z ����
*/
struct ControlPiar
{
    size_t id;
    ImageGcp imageCoord;
    GroundGcp groundCoord;
};