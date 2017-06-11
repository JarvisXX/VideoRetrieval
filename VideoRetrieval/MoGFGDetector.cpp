
#include "clude.h"
#include "MoGFGDetector.h"
#include "stdlib.h"
#include "math.h"
#include "Rule.h"

 rule *EntityRule1;
 rule *EntityRule[5];
 extern float resizeFactor;

void ucArr2Img(IplImage* img, unsigned char* data, int iW)
{
	int i, stride, w, h, nchannel;
	w = img->width;
	h = img->height;
	nchannel = img->nChannels;
	stride = img->widthStep;
	for (i = 0; i < h; i++)
	{
		memcpy((void*)(img->imageData + i * stride), (void*)(data + i * iW), iW);
	}
}

void Img2ucArr(IplImage* img, unsigned char* data, int iW)
{
	int i, stride, w, h, nchannel;
	w = img->width;
	h = img->height;
	nchannel = img->nChannels;
	stride = img->widthStep;
	for (i = 0; i < h; i++)
	{
		memcpy((void*)(data + i * iW), (void*)(img->imageData + i * stride), iW);
	}
}
MoGFGDetector::MoGFGDetector()
{
	for (int i = 0; i < GK; i++)
	{
		u[i] = NULL;
		Q[i] = NULL;
		W[i] = NULL;
	}
	foregroundInBits = NULL;
	morBuffer = NULL;
	morBuffer2 = NULL;
	foreground = NULL;
	foregroundCopy = NULL;
	shadow = NULL;
	frOld = NULL;
	gMatch = NULL;
	pFG = NULL;
	pBG = NULL;
}

MoGFGDetector::~MoGFGDetector()   //��Ҫ�Ǵ���洢�ռ���ͷ�
{
	for (int i = 0; i < GK; i++)
	{
		if (u[i] != NULL)
		{
			delete u[i];
		}
		if (Q[i] != NULL)
		{
			delete Q[i];
		}
		if (W[i] != NULL)
		{
			delete W[i];
		}
	}
	if (foregroundInBits != NULL)
		free(foregroundInBits);
	if (morBuffer2 != NULL)
		free(morBuffer2);
	if (morBuffer != NULL)
		free(morBuffer);
	if (foreground != NULL)
		free(foreground);
	if (foregroundCopy != NULL)
		free(foregroundCopy);
	if (shadow != NULL)
		free(shadow);
	if (frOld != NULL)
		free(frOld);
	if (gMatch != NULL)
		free(gMatch);
	if (pFG != NULL)
		cvReleaseImage(&pFG);
	if (pBG != NULL)
		cvReleaseImage(&pBG);
}

void MoGFGDetector::initialize(int iw, int ih, int stride, int frameStart)
{
	//��ʼ��
	FrameStart = frameStart;
	iW = iw; iH = ih; iStride = stride;
	iLen = iStride * iH;
	frameCounter = 0;
	//��ʼ����˹ģ��
	for (int i = 0; i < GK; i++)
	{
		u[i] = new unsigned char[iLen];
		Q[i] = new float[iW * iH];
		W[i] = new float[iW * iH];
	}
	//��̬ѧ������Ҫ���м����
	foregroundInBits = (unsigned int*)malloc(((iW / 32) + 1) * iH * 4);
	morBuffer = (unsigned int*)malloc(((iW / 32) + 1) * iH * 4);
	morBuffer2 = (unsigned int*)malloc(((iW / 32) + 1) * iH * 4);
	//ǰ��
	foreground = (unsigned char*)malloc(iW * iH);
	foregroundCopy = (unsigned char*)malloc(iW * iH);
	//��Ӱ
	shadow = (unsigned char*)malloc(iW * iH);
	//����1֡
	frOld = (unsigned char*)malloc(iLen);
	//��¼��ǰ֡��ģ�͵�ƥ�����������ʱ��Ҫ
	gMatch = (unsigned char*)malloc(iW * iH);

	//Iplimage�࣬������ģ���ȡ
	pFG = cvCreateImage(cvSize(iW, iH), IPL_DEPTH_8U, 1);
	pBG = cvCreateImage(cvSize(iW, iH), IPL_DEPTH_8U, 3);
}
//ʹ��opencv�ľ��ຯ���ϲ��ӽ���blobʱ ��Ҫ�ıȽϺ���  Դ��opencv Visual Surveillance�ο�����
static int CompareContour(const void* a, const void* b, void*)
{
	float           dx, dy;
	float           h, w, ht, wt;
	CvPoint2D32f    pa, pb;
	CvRect          ra, rb;
	CvSeq*          pCA = *(CvSeq**)a;
	CvSeq*          pCB = *(CvSeq**)b;
	ra = ((CvContour*)pCA)->rect;
	rb = ((CvContour*)pCB)->rect;
	pa.x = ra.x + ra.width*0.5f;
	pa.y = ra.y + ra.height*0.5f;
	pb.x = rb.x + rb.width*0.5f;
	pb.y = rb.y + rb.height*0.5f;
	w = (ra.width + rb.width)*0.5f;
	h = (ra.height + rb.height)*0.5f;

	dx = (float)(fabs(pa.x - pb.x) - w);
	dy = (float)(fabs(pa.y - pb.y) - h);

	//wt = MAX(ra.width,rb.width)*0.1f;
	wt = 0;
	ht = MAX(ra.height, rb.height)*0.1f;
	if (dx < wt && dy < ht) return 1;
	return 0;
}

void MoGFGDetector::process(IplImage* pFrame, long ilhandle)
{
	int i, j, k, placeBmp, place;
	float sd;
	bool flagShadow = false;
	float Sweight, Sweight2;
	float fBuff;
	unsigned char ucBuff;
	bool flagMatch = false;
	double Ip, f, cosf, h, Bp, r, cosfb, fb, f0 = __f0, r1 = __r1, r2 = __r2;
	frNew = (unsigned char*)pFrame->imageDataOrigin;


	if (frameCounter == 0) 		//��һ֡�Ļ� ��ʼ����˹ģ�͵ĸ��ֲ���
	{
		memcpy(u[0], frNew, iLen);
		place = 0;
		placeBmp = 0;
		for (i = 0; i < iH; i++)
		{
			for (j = 0; j < iW; j++)
			{
				for (k = 0; k < GK; k++)
				{
					*(Q[k] + place) = VInitial;
					*(W[k] + place) = 0;
				}
				*(W[0] + place) = 1;
				place++;
				placeBmp += 3;
			}
			placeBmp += (iStride - 3 * iW);
		}
		memcpy(frOld, frNew, iLen);
	}
	else
	{
		//���տ���
		/*
		����ȫ��ƽ���Ҷȣ�����һ֡�ĸ�˹ģ��ƽ���Ҷ���ȣ�����нϴ�仯���򽫱䶯�����ڽ�С��Χ�ࡣ
		�򵥵�˵����ǯ�Ʊ仯���ٶȣ�������������Եļ��͸��½��
		ȫ�ֻ���ͳ�ƺͼӼ����� ���Ƿǳ���������Ч���п�
		*/
		//ͳ��ƽ��ֵ
		double newAvg = 0;
		int dLight = 0;
		placeBmp = 0;
		place = 0;
		for (i = 0; i < iH; i++)
		{
			for (j = 0; j < iW; j++)
			{
				newAvg += (double(*(frNew + placeBmp) + *(frNew + placeBmp + 1) + *(frNew + placeBmp + 2))) / 3;
				placeBmp += 3;
			}
			placeBmp += (iStride - 3 * iW);
		}
		newAvg /= (iW*iH);

		if (fabs(newAvg - lightBuffer) > 4)
		{
			//����仯�󣬼�С���ձ仯
			dLight = int(newAvg - lightBuffer);
			placeBmp = 0;
			for (i = 0; i < iH; i++)
			{
				for (j = 0; j < iW; j++)
				{
					if (dLight > 0)
					{
						*(frNew + placeBmp) = *(frNew + placeBmp) + 1 - dLight >= 0 ? *(frNew + placeBmp) - dLight + 1 : 0;
						*(frNew + placeBmp + 1) = *(frNew + placeBmp + 1) + 1 - dLight >= 0 ? *(frNew + placeBmp + 1) - dLight + 1 : 0;
						*(frNew + placeBmp + 2) = *(frNew + placeBmp + 2) + 1 - dLight >= 0 ? *(frNew + placeBmp + 2) - dLight + 1 : 0;
					}
					else
					{
						*(frNew + placeBmp) = *(frNew + placeBmp) - dLight <= 256 ? *(frNew + placeBmp) - dLight - 1 : 255;
						*(frNew + placeBmp + 1) = *(frNew + placeBmp + 1) - dLight <= 256 ? *(frNew + placeBmp + 1) - dLight - 1 : 255;
						*(frNew + placeBmp + 2) = *(frNew + placeBmp + 2) - dLight <= 256 ? *(frNew + placeBmp + 2) - dLight - 1 : 255;
					}
					placeBmp += 3;
				}
				placeBmp += (iStride - 3 * iW);
			}
		}
		lightBuffer = 0;
		//ǰ����ȡ

		placeBmp = 0;   //��¼3ͨ��ͼ���е�offset
		place = 0;      //��¼��ͨ��ͼ���е�offset
		{
			for (i = 0; i < iH; i++)
			{
				for (j = 0; j < iW; j++)
				{
					unsigned char &newB = *(frNew + placeBmp), &newG = *(frNew + placeBmp + 1), &newR = *(frNew + placeBmp + 2);
					flagMatch = false;
					flagShadow = false;
					*(gMatch + place) = 0;
					Sweight = 0;
					for (k = 0; k < GK; k++)
					{
						float &q = *(Q[k] + place), &weight = *(W[k] + place);
						unsigned char &uB = *(u[k] + placeBmp), &uG = *(u[k] + placeBmp + 1), &uR = *(u[k] + placeBmp + 2);
						sd = sqrt(q);
						if (!flagMatch && abs(uB - newB)< matchT * sd
							&& abs(uG - newG) < matchT * sd
							&& abs(uR - newR) < matchT * sd)
						{
							*(gMatch + place) = k + 1;    //��¼ƥ��
							flagMatch = true;
						}
						else
						{
							//��ƥ�䣬���ж��ǲ�����Ӱ����
							if (!flagMatch && Sweight < weightT)
							{
								Sweight += weight;
								Ip = sqrt((float(frNew[placeBmp]))*(float(frNew[placeBmp]))
									+ (float(frNew[placeBmp + 1]))*(float(frNew[placeBmp + 1]))
									+ (float(frNew[placeBmp + 2]))*(float(frNew[placeBmp + 2])));

								Bp = sqrt((float(*(u[k] + placeBmp)))*(float(*(u[k] + placeBmp)))
									+ (float(*(u[k] + placeBmp + 1)))*(float(*(u[k] + placeBmp + 1)))
									+ (float(*(u[k] + placeBmp + 2)))*(float(*(u[k] + placeBmp + 2))));

								cosf = float(frNew[placeBmp] * *(u[k] + placeBmp)
									+ frNew[placeBmp + 1] * *(u[k] + placeBmp + 1)
									+ frNew[placeBmp + 2] * *(u[k] + placeBmp + 2)) / (Ip*Bp);
								f = acos(cosf);

								cosfb = float(255 * (*(u[k] + placeBmp))
									+ 255 * (*(u[k] + placeBmp + 1))
									+ 255 * (*(u[k] + placeBmp + 2))) / (Ip*Bp);
								fb = acos(cosfb);

								h = Ip * cosf;
								r = Bp / h;

								if (f < f0 && r > r1 && r < r2)
								{
									flagShadow = true;
								}
							}
						}
					}
					if (!flagMatch || Sweight > weightT)
					{
						//���û�кͱ���ƥ���ϻ��� �Ѿ�����һ�����ʲŵõ�ƥ�� ���ǰ������Ӱ����
						foreground[place] = flagShadow ? 0 : 255;
						shadow[place] = flagShadow ? 255 : 0;
					}
					else
					{
						foreground[place] = 0;
						shadow[place] = 0;
					}
					lightBuffer += (double(*(u[0] + placeBmp) + *(u[0] + placeBmp + 1) + *(u[0] + placeBmp + 2))) / 3;   //ͳ�Ƹ�˹ģ���е�ƽ���Ҷ�
					placeBmp += 3;
					place++;
				}
				placeBmp += (iStride - 3 * iW);
			}
		}
		lightBuffer /= (iW * iH);

		//��������
		memcpy(frOld, frNew, iLen);

		//��Ӱ���˶���ȡ,(׼����ʾ����)
		if (frameCounter > FrameStart)
		{
			//��̬ѧ���� 
			//��ͼ��ת���� ÿλ����һ���صĴ�����ݣ��ӿ���̬ѧ�����ٶ�
			getForegroundInBits(foreground);
			killSeperatePoint();     //ȥ�����ĵ�
			retrieveForeground(foreground);    //��ԭ�õ��ַ��ʹ洢��ǰ��ģʽ
			placeBmp = 0;
			place = 0;

			ucArr2Img(pFG, foreground, iW);    //��ǰ��ת��Ϊͼ���ʽ
			m_NewBlobList.erase(m_NewBlobList.begin(), m_NewBlobList.end());    //��ռ���BLOB�б�
			IplImage*       pIB = cvCreateImage(cvSize(iW, iH), IPL_DEPTH_8U, 1);
			CvMemStorage*   storage = cvCreateMemStorage();
			CvSeq*          cnt = NULL;    //��ȡ������
			CvSeq*          cnt_list = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvSeq*), storage);
			CvSeq*          clasters = NULL;
			int             claster_cur, claster_num;
			CvSize			S = cvSize(pFG->width, pFG->height);
			float m_HMin = 0.05, m_WMin = 0.02;
			//float m_HMin = 0.1, m_WMin = 0.05;

			ucArr2Img(pIB, foreground, iW);    //��ǰ��ת��Ϊͼ���ʽ
			cvFindContours(pIB, storage, &cnt, sizeof(CvContour), CV_RETR_EXTERNAL);
			cvReleaseImage(&pIB);

			for (; cnt; cnt = cnt->h_next)
			{
				cvSeqPush(cnt_list, &cnt);    //���õ�����������ջ��
			}
			//���ӽ�����������
			claster_num = cvSeqPartition(cnt_list, storage, &clasters, CompareContour, NULL);

			for (claster_cur = 0; claster_cur<claster_num; ++claster_cur)
			{
				//��ÿһ������������һ��blob
				int         cnt_cur;
				myBlob		NewBlob;
				double      M00, X, Y, XX, YY;
				CvMoments   m;
				CvRect      rect_res = cvRect(-1, -1, -1, -1);
				CvMat       mat;

				for (cnt_cur = 0; cnt_cur<clasters->total; ++cnt_cur)
				{
					CvRect  rect;
					CvSeq*  cnt;
					int k = *(int*)cvGetSeqElem(clasters, cnt_cur);
					if (k != claster_cur) continue;
					cnt = *(CvSeq**)cvGetSeqElem(cnt_list, cnt_cur);
					rect = ((CvContour*)cnt)->rect;

					if (rect_res.height<0)
					{
						rect_res = rect;
					}
					else
					{
						int x0, x1, y0, y1;
						x0 = MIN(rect_res.x, rect.x);
						y0 = MIN(rect_res.y, rect.y);
						x1 = MAX(rect_res.x + rect_res.width, rect.x + rect.width);
						y1 = MAX(rect_res.y + rect_res.height, rect.y + rect.height);
						rect_res.x = x0;
						rect_res.y = y0;
						rect_res.width = x1 - x0;
						rect_res.height = y1 - y0;
					}
				}
				//if(rect_res.height < S.height*m_HMin || rect_res.width < S.width*m_WMin) continue;		//too small
				//if (EntityRule[ilhandle]->Rule_invasion.invasion_flag == 1)
				//{
				//	if (rect_res.height * resizeFactor <EntityRule[ilhandle]->Rule_invasion.Rule_foregroundYsize || rect_res.width * resizeFactor < EntityRule[ilhandle]->Rule_invasion.Rule_foregroundXsize) continue;
				//}

				//else
				//{
				//	if (rect_res.height < S.height*m_HMin || rect_res.width < S.width*m_WMin) continue;		//too small
				//}
				if (rect_res.x + rect_res.width / 2 < 3 || rect_res.x + rect_res.width / 2 > S.width - 3
					|| rect_res.y + rect_res.height / 2 < 3 || rect_res.y + rect_res.height / 2 > S.height - 3) continue;		//too close to border
				if (rect_res.height < 1 || rect_res.width < 1)
				{
					X = 0;
					Y = 0;
					XX = 0;
					YY = 0;
					continue;
				}
				else
				{
					//����2�׾����õ����ʵ�bounding box
					cvMoments(cvGetSubRect(pFG, &mat, rect_res), &m, 0);
					M00 = cvGetSpatialMoment(&m, 0, 0);
					if (M00 <= 0) continue;
					X = cvGetSpatialMoment(&m, 1, 0) / M00;
					Y = cvGetSpatialMoment(&m, 0, 1) / M00;
					XX = (cvGetSpatialMoment(&m, 2, 0) / M00) - X*X;
					YY = (cvGetSpatialMoment(&m, 0, 2) / M00) - Y*Y;
				}
				NewBlob.ID = 0;
				//outRect�Ǳ߽��
				NewBlob.outRect = rect_res;
				//���ݾصõ��Ŀ�Centroid��ΪĿ������ĵ�
				NewBlob.monRect =
					cvRect(rect_res.x + (float)X - (float)(2 * sqrt(XX)),
					rect_res.y + (float)Y - (float)(2 * sqrt(YY)), (float)(4 * sqrt(XX)), (float)(4 * sqrt(YY)));
				NewBlob.monRect.x = max(NewBlob.monRect.x, NewBlob.outRect.x);
				NewBlob.monRect.y = max(NewBlob.monRect.y, NewBlob.outRect.y);
				NewBlob.monRect.width = min(NewBlob.monRect.width, NewBlob.outRect.x + NewBlob.outRect.width - NewBlob.monRect.x);
				NewBlob.monRect.height = min(NewBlob.monRect.height, NewBlob.outRect.y + NewBlob.outRect.height - NewBlob.monRect.y);
				//���ƾؿ򲻳����߽�򣬱���һ��С����ķ���
				CvRect& rect = NewBlob.monRect;
				rect.x = rect.x >= 0 ? rect.x : 0; rect.x = rect.x<iW ? rect.x : iW - 1;
				rect.y = rect.y >= 0 ? rect.y : 0; rect.y = rect.y<iH ? rect.y : iH - 1;
				rect.width = rect.x + rect.width < iW ? rect.width : iW - rect.x - 1;
				rect.height = rect.y + rect.height < iH ? rect.height : iH - rect.y - 1;

				NewBlob.medianX = NewBlob.monRect.x + NewBlob.monRect.width / 2;
				NewBlob.medianY = NewBlob.monRect.y + NewBlob.monRect.height / 2;
				//���뵽new blob list
				m_NewBlobList.push_back(NewBlob);
			}
			//��������N��blob
			{
				int N = m_NewBlobList.size();
				int i, j;
				for (i = 1; i<MIN(N, 12); ++i)
				{
					for (j = i; j>0; --j)
					{
						myBlob  temp;
						float   AreaP, AreaN;
						myBlob* pP = &m_NewBlobList[j - 1];
						myBlob* pN = &m_NewBlobList[j];
						AreaP = pP->monRect.width*pP->monRect.height;
						AreaN = pN->monRect.width*pN->monRect.height;
						if (AreaN < AreaP)break;
						temp = pN[0];
						pN[0] = pP[0];
						pP[0] = temp;
					}
				}

				for (i = N; i>12; --i)
				{
					m_NewBlobList.erase(m_NewBlobList.begin() + i - 1);
				}
			}/* sort blobs by size */
			cvReleaseMemStorage(&storage);
		}
	}
	frameCounter++;
}

vector<myBlob>* MoGFGDetector::getBlobList()
{
	return &m_NewBlobList;
}


IplImage* MoGFGDetector::getFG()
{
	return pFG;
}

IplImage* MoGFGDetector::getBG()
{
	return pBG;
}

bool MoGFGDetector::ifStable()
{
	return frameCounter > FrameStart;
}
//��ͼ��ת���� ÿλ����һ���صĴ�����ݣ��ӿ���̬ѧ�����ٶ�
void MoGFGDetector::getForegroundInBits(unsigned char* input)
{
	int i, j, placeInByte = 0;
	unsigned char* fgbytes = input;
	unsigned int* fgbits = foregroundInBits;     //��ȡǰ��  ��1λ����ͼ����һ���� 
	memset(fgbits, 0, (((iW / 32) + 1) * iH * 4)); //iW*iH��Ӧ��ȫ����ʲô���ӵ�   1��UINT��Ӧ32λ,��memset�ǰ��ֽ���0������*4
	for (i = 0; i < iH; i++)
	{
		placeInByte = 0;     //ƫ����
		for (j = 0; j < iW; j++)
		{
			set(*fgbits, placeInByte, (*(fgbytes++) > 0));
			if (++placeInByte == 32)
			{
				fgbits++;
				placeInByte = 0;
			}
		}
		fgbits++;
	}
}
//��̬ѧ����֮��õ��ַ��͵�ǰ��
void MoGFGDetector::retrieveForeground(unsigned char* output)
{
	int i, j, placeInByte = 0;
	unsigned char* fgbytes = output;
	unsigned int* fgbits = foregroundInBits;
	for (i = 0; i < iH; i++)
	{
		placeInByte = 0;
		for (j = 0; j < iW; j++)
		{
			if (test(*fgbits, placeInByte))
			{
				*(fgbytes++) = 255;
			}
			else
			{
				*(fgbytes++) = 0;
			}
			if (++placeInByte == 32)
			{
				fgbits++;
				placeInByte = 0;
			}
		}
		fgbits++;
	}
}
//����
void MoGFGDetector::dilate()
{
	int i = 0, j = 0, stride = (iW / 32) + 1, place = 0;
	memcpy(morBuffer, foregroundInBits, stride * iH * 4);
	for (i = 0; i < iH; i++)
	{
		place = i * stride;
		foregroundInBits[place] |= ((morBuffer[place] >> 1) + (morBuffer[place + 1] << 31));
		foregroundInBits[place] |= ((morBuffer[place] << 1) + 0);
		place++;
		for (j = 1; j< stride - 1; j++)
		{
			foregroundInBits[place] |= ((morBuffer[place] >> 1) + (morBuffer[place + 1] << 31));
			foregroundInBits[place] |= ((morBuffer[place] << 1) + (morBuffer[place - 1] >> 31));
			place++;
		}
		foregroundInBits[place] |= ((morBuffer[place] >> 1) + 0);
		foregroundInBits[place] |= ((morBuffer[place] << 1) + (morBuffer[place - 1] >> 31));
	}
	place = 0;
	for (j = 0; j< stride; j++)
	{
		foregroundInBits[place] |= (morBuffer[place + stride]);
		place++;
	}
	for (i = 1; i < iH - 1; i++)
	{
		for (j = 0; j< stride; j++)
		{
			foregroundInBits[place] |= (morBuffer[place + stride]);
			foregroundInBits[place] |= (morBuffer[place - stride]);
			place++;
		}
	}
	for (j = 0; j< stride; j++)
	{
		foregroundInBits[place] |= (morBuffer[place - stride]);
		place++;
	}
}
//��ʴ
void MoGFGDetector::erode()
{
	int i = 0, j = 0, stride = (iW / 32) + 1, place = 0;
	memcpy(morBuffer, foregroundInBits, stride * iH * 4);
	for (i = 0; i < iH; i++)
	{
		place = i * stride;
		foregroundInBits[place] &= ((morBuffer[place] >> 1) + (morBuffer[place + 1] << 31));
		foregroundInBits[place] &= ((morBuffer[place] << 1) + (1 >> 31));
		place++;
		for (j = 1; j< stride - 1; j++)
		{
			foregroundInBits[place] &= ((morBuffer[place] >> 1) + (morBuffer[place + 1] << 31));
			foregroundInBits[place] &= ((morBuffer[place] << 1) + (morBuffer[place - 1] >> 31));
			place++;
		}
		foregroundInBits[place] &= ((morBuffer[place] >> 1) + (1 << 31));
		foregroundInBits[place] &= ((morBuffer[place] << 1) + (morBuffer[place - 1] >> 31));
	}
	place = 0;
	for (j = 0; j< stride; j++)
	{
		foregroundInBits[place] &= (morBuffer[place + stride]);
		place++;
	}
	for (i = 1; i < iH - 1; i++)
	{
		for (j = 0; j< stride; j++)
		{
			foregroundInBits[place] &= (morBuffer[place + stride]);
			foregroundInBits[place] &= (morBuffer[place - stride]);
			place++;
		}
	}
	for (j = 0; j< stride; j++)
	{
		foregroundInBits[place] &= (morBuffer[place - stride]);
		place++;
	}
}
//��
void MoGFGDetector::openProcess(int iSize)
{
	int i;
	for (i = 0; i < iSize; i++)
		erode();
	for (i = 0; i < iSize; i++)
		dilate();
}
//��
void MoGFGDetector::closeProcess(int iSize)
{
	int i;
	for (i = 0; i < iSize; i++)
		dilate();
	for (i = 0; i < iSize; i++)
		erode();
}

//ȥ��������  ///˳��ȥ��������
void MoGFGDetector::killSeperatePoint()
{
	int i = 0, j = 0, stride = (iW / 32) + 1, place = 0;
	memcpy(morBuffer2, foregroundInBits, stride * iH * 4);
	erode();
	dilate(); dilate(); dilate(); dilate();
	for (i = 0; i < iH; i++)
	{
		place = i * stride;
		for (j = 0; j< stride; j++)
		{
			foregroundInBits[place] &= morBuffer2[place];
			place++;
		}
	}
}
//���±���ģ��
void MoGFGDetector::update(IplImage* pTrackLabel, vector<int>* pUnStableLabel)
{
	int i, j, k, placeBmp, place;
	bool flagShadow = false;
	float Sweight, Sweight2;
	float fBuff;
	unsigned char ucBuff;
	bool flagMatch = false;
	uchar* pLabel;

	//��TrackLabel�����Ը��ٽ����ת��Ϊforeground
	Img2ucArr(pTrackLabel, foreground, iW);
	//pUnStableLabel �еı���� ���ȶ�Ŀ�꣬Ҳ���ǲ����ٸ��½׶����Ա���������
	if (pUnStableLabel != NULL)
	{
		for (i = 0; i < iH; i++)
		{
			pLabel = foreground + i * iW;
			for (j = 0; j < iW; j++)
			{
				if (*pLabel != 0)
				{
					for (int k = 0; k < int(pUnStableLabel->size()); k++)
					{
						if (*pLabel == (*pUnStableLabel)[k])
						{
							*pLabel = 0;
							break;
						}
					}
				}
				pLabel++;

			}
		}
	}

	getForegroundInBits(foreground);
	dilate(); dilate(); dilate(); dilate(); dilate(); dilate(); dilate();
	retrieveForeground(foreground);

	if (frameCounter < FrameStart || frameCounter % 10 == 0)
	{
		//ÿ10֡һ����
		placeBmp = 0;
		place = 0;
		for (i = 0; i < iH; i++)
		{
			pLabel = foreground + i * iW;
			for (j = 0; j < iW; j++)
			{
				bool flagContinue;
				if (*pLabel == 0)          //���ʹ�ô��жϣ���ʾ���������ڸ�������ռ�ݵ�����
				{
					//����gMatch�洢��ƥ����Ϣ�����±���ģ��
					unsigned char &newB = *(frOld + placeBmp), &newG = *(frOld + placeBmp + 1), &newR = *(frOld + placeBmp + 2);
					flagMatch = false;
					flagShadow = false;
					Sweight = 0;
					Sweight2 = 0;
					//����Ȩ������
					for (k = 0; k < GK; k++)
					{
						float &weight = *(W[k] + place);
						Sweight2 += weight;
						if (!flagMatch && *(gMatch + place) == k + 1)
						{
							float &q = *(Q[k] + place);
							unsigned char &uB = *(u[k] + placeBmp), &uG = *(u[k] + placeBmp + 1), &uR = *(u[k] + placeBmp + 2);
							uB = (1 - valueAlpha) * uB + valueAlpha * newB + 0.5;
							uG = (1 - valueAlpha) * uG + valueAlpha * newG + 0.5;
							uR = (1 - valueAlpha) * uR + valueAlpha * newR + 0.5;
							fBuff = (uB - newB) * (uB - newB)
								+ (uG - newG) * (uG - newG)
								+ (uR - newR) * (uR - newR);
							q = max((1 - valueAlpha) * q + valueAlpha * fBuff, (float)VLowLimit);
							weight = float(1 - weightAlpha) * weight + float(weightAlpha) * 0.95;
							flagMatch = true;
						}
						else
						{
							weight *= float(1 - weightAlpha);
						}
					}
					for (k = 0; k < GK; k++)
					{
						*(W[k] + place) /= Sweight2;
					}
					for (k = GK - 1; k > 0; k--)
					{
						float &weight = *(W[k] + place), &weight2 = *(W[k - 1] + place);
						if (weight / *(Q[k] + place) > weight2 / *(Q[k - 1] + place))
						{
							fBuff = weight; weight = weight2; weight2 = fBuff;
							fBuff = *(Q[k] + place); *(Q[k] + place) = *(Q[k - 1] + place); *(Q[k - 1] + place) = fBuff;
							ucBuff = *(u[k] + placeBmp); *(u[k] + placeBmp) = *(u[k - 1] + placeBmp); *(u[k - 1] + placeBmp) = ucBuff;
							ucBuff = *(u[k] + placeBmp + 1); *(u[k] + placeBmp + 1) = *(u[k - 1] + placeBmp + 1); *(u[k - 1] + placeBmp + 1) = ucBuff;
							ucBuff = *(u[k] + placeBmp + 2); *(u[k] + placeBmp + 2) = *(u[k - 1] + placeBmp + 2); *(u[k - 1] + placeBmp + 2) = ucBuff;
						}
					}
					for (k = 1; k < GK - 1; k++)
					{
						float &weight = *(W[k] + place), &weight2 = *(W[k + 1] + place);
						if (weight / *(Q[k] + place)  < weight2 / *(Q[k + 1] + place))
						{
							fBuff = weight; weight = weight2; weight2 = fBuff;
							fBuff = *(Q[k] + place); *(Q[k] + place) = *(Q[k + 1] + place); *(Q[k + 1] + place) = fBuff;
							ucBuff = *(u[k] + placeBmp); *(u[k] + placeBmp) = *(u[k + 1] + placeBmp); *(u[k + 1] + placeBmp) = ucBuff;
							ucBuff = *(u[k] + placeBmp + 1); *(u[k] + placeBmp + 1) = *(u[k + 1] + placeBmp + 1); *(u[k + 1] + placeBmp + 1) = ucBuff;
							ucBuff = *(u[k] + placeBmp + 2); *(u[k] + placeBmp + 2) = *(u[k + 1] + placeBmp + 2); *(u[k + 1] + placeBmp + 2) = ucBuff;
						}
					}
					if (flagMatch == false)
					{
						W[0][place] += W[GK - 1][place];
						W[GK - 1][place] = float(priorWeight);
						Q[GK - 1][place] = float(VInitial);
						u[GK - 1][placeBmp] = newB;
						u[GK - 1][placeBmp + 1] = newG;
						u[GK - 1][placeBmp + 2] = newR;
					}
				}
				pLabel++;
				placeBmp += 3;
				place++;
			}
			placeBmp += (iStride - 3 * iW);
		}
	}
	//��ñ���ͼƬ
	ucArr2Img(pBG, u[0], iStride);
}

void MoGFGDetector::reset()
{
	frameCounter = 0;
}