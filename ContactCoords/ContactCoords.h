// ���� ifdef ���Ǵ���ʹ�� DLL �������򵥵�
// ��ı�׼�������� DLL �е������ļ��������������϶���� CONTACTCOORDS_EXPORTS
// ���ű���ġ���ʹ�ô� DLL ��
// �κ�������Ŀ�ϲ�Ӧ����˷��š�������Դ�ļ��а������ļ����κ�������Ŀ���Ὣ
// CONTACTCOORDS_API ������Ϊ�Ǵ� DLL ����ģ����� DLL ���ô˺궨���
// ������Ϊ�Ǳ������ġ�
#ifdef CONTACTCOORDS_EXPORTS
#define CONTACTCOORDS_API __declspec(dllexport)
#else
#define CONTACTCOORDS_API __declspec(dllimport)
#endif

#include "../SpaceResectionDLL/SpaceResectionDLL.h"

// �����Ǵ� ContactCoords.dll ������
class CONTACTCOORDS_API CContactCoords {
public:
	//CContactCoords(void);
	// TODO:  �ڴ�������ķ�����
    static int Contact(ControlPiar* pControlPair, const char* pGCPfile);
};