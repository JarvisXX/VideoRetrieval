// MoGFGDetector.h: interface for the MoGFGDetector class.
//前景检测模块
//////////////////////////////////////////////////////////////////////
#ifndef MOGFGDETECTOR_H
#define MOGFGDETECTOR_H

#include "Object.h"
//parameters for shadow removal
#define __f0 3.14/180*2.5   //夹角
#define __r1 1.0            //亮度比例下限
#define __r2 1.6f           //亮度比例上限

//MoG parameters
#define GK 3                //高斯模型数目
#define VInitial 10 * 10    //初始方差
#define VLowLimit 5 * 5     //最小方差
#define matchT 2.5f         //相似门限

#define weightAlpha 0.05f   //背景更新系数
#define valueAlpha 0.05f

#define priorWeight 0.05    //初始权重
#define weightT 0.5       

//位操作
#define set(a,b,c)		(a |= (c << b))
#define clear(a,b)		(a &= (~(1 << b)))
#define test(a,b)		((a & (1<<b)) > 0)

//检出前景块的最小大小比例 相对于图像大小
#define areaLimit 0.0001

class MoGFGDetector
{
public:
	MoGFGDetector();
	virtual ~MoGFGDetector();

	void initialize(int iw, int ih, int stride, int frameStart);
	void process(IplImage* pFrame, long ilhandle);      //对新一帧做处理
	void update(IplImage* pTrackLabel, vector<int>* pUnStableLabel);        //更新背景模型 参考跟踪的情况
	//获取结果的接口
	IplImage* getFG();
	IplImage* getBG();
	vector<myBlob>* getBlobList();
	//是否完成初始建模，可以开始检测跟踪
	bool ifStable();
	void reset();

private:
	//高斯模型的参数：
	//均值 匹配记录
	unsigned char * u[GK], *gMatch;
	//方差
	float * Q[GK];
	//权重
	float * W[GK];
	//前景 背景  图片
	IplImage *pFG, *pBG;
	//对前一帧和当前帧的拷贝
	unsigned char *frOld, *frNew;
	//前景检测结果， 阴影， copy
	unsigned char *foreground, *shadow, *foregroundCopy;
	//以bit存储2值化结果的前景结果
	unsigned int *foregroundInBits;
	//形态学处理 需要的buffer
	unsigned int *morBuffer, *morBuffer2;

	int FrameStart;  //训练的帧数
	//转化为以bit存储2值化结果的前景结果，可以极大的提高形态学操作的速度	
	void getForegroundInBits(unsigned char* input);
	//形态学操作之后转换回来
	void retrieveForeground(unsigned char* output);
	//膨胀	
	void dilate();
	//腐蚀	
	void erode();
	//开处理
	void openProcess(int iSize);
	//闭处理
	void closeProcess(int iSize);
	void killSeperatePoint();
	//尺寸 stride表示 图像一行中的字节数 因为dib图像是要求字节数 是 4的倍数的 不是总等于图像宽度
	int iW, iH, iStride, iLen;
	//帧计数
	int frameCounter;
	//记录平均亮度
	double lightBuffer;
	//检测出的前景块序列
	vector<myBlob>	m_NewBlobList;
	//记录每个前景快的面积
	int				area[1000];
};

#endif