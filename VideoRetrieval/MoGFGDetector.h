// MoGFGDetector.h: interface for the MoGFGDetector class.
//ǰ�����ģ��
//////////////////////////////////////////////////////////////////////
#ifndef MOGFGDETECTOR_H
#define MOGFGDETECTOR_H

#include "Object.h"
//parameters for shadow removal
#define __f0 3.14/180*2.5   //�н�
#define __r1 1.0            //���ȱ�������
#define __r2 1.6f           //���ȱ�������

//MoG parameters
#define GK 3                //��˹ģ����Ŀ
#define VInitial 10 * 10    //��ʼ����
#define VLowLimit 5 * 5     //��С����
#define matchT 2.5f         //��������

#define weightAlpha 0.05f   //��������ϵ��
#define valueAlpha 0.05f

#define priorWeight 0.05    //��ʼȨ��
#define weightT 0.5       

//λ����
#define set(a,b,c)		(a |= (c << b))
#define clear(a,b)		(a &= (~(1 << b)))
#define test(a,b)		((a & (1<<b)) > 0)

//���ǰ�������С��С���� �����ͼ���С
#define areaLimit 0.0001

class MoGFGDetector
{
public:
	MoGFGDetector();
	virtual ~MoGFGDetector();

	void initialize(int iw, int ih, int stride, int frameStart);
	void process(IplImage* pFrame, long ilhandle);      //����һ֡������
	void update(IplImage* pTrackLabel, vector<int>* pUnStableLabel);        //���±���ģ�� �ο����ٵ����
	//��ȡ����Ľӿ�
	IplImage* getFG();
	IplImage* getBG();
	vector<myBlob>* getBlobList();
	//�Ƿ���ɳ�ʼ��ģ�����Կ�ʼ������
	bool ifStable();
	void reset();

private:
	//��˹ģ�͵Ĳ�����
	//��ֵ ƥ���¼
	unsigned char * u[GK], *gMatch;
	//����
	float * Q[GK];
	//Ȩ��
	float * W[GK];
	//ǰ�� ����  ͼƬ
	IplImage *pFG, *pBG;
	//��ǰһ֡�͵�ǰ֡�Ŀ���
	unsigned char *frOld, *frNew;
	//ǰ��������� ��Ӱ�� copy
	unsigned char *foreground, *shadow, *foregroundCopy;
	//��bit�洢2ֵ�������ǰ�����
	unsigned int *foregroundInBits;
	//��̬ѧ���� ��Ҫ��buffer
	unsigned int *morBuffer, *morBuffer2;

	int FrameStart;  //ѵ����֡��
	//ת��Ϊ��bit�洢2ֵ�������ǰ����������Լ���������̬ѧ�������ٶ�	
	void getForegroundInBits(unsigned char* input);
	//��̬ѧ����֮��ת������
	void retrieveForeground(unsigned char* output);
	//����	
	void dilate();
	//��ʴ	
	void erode();
	//������
	void openProcess(int iSize);
	//�մ���
	void closeProcess(int iSize);
	void killSeperatePoint();
	//�ߴ� stride��ʾ ͼ��һ���е��ֽ��� ��Ϊdibͼ����Ҫ���ֽ��� �� 4�ı����� �����ܵ���ͼ����
	int iW, iH, iStride, iLen;
	//֡����
	int frameCounter;
	//��¼ƽ������
	double lightBuffer;
	//������ǰ��������
	vector<myBlob>	m_NewBlobList;
	//��¼ÿ��ǰ��������
	int				area[1000];
};

#endif