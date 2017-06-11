#include"clude.h"

#ifndef ITEM_H
#define ITEM_H

class item
{
public:
	int id; //���
	int category; //���� 0-vehicle  1-pedstrain
	int video_id; //��Ƶ���
	int color_list[10];
	int color_t1, color_t2;
	/*
	0 - red(5)	1 - orange	2 - yellow	3 - green	4 - violet
	5 - red		6 - white	7 - grey	8 - black	9 - blue*/
	int color; //��ɫ
	/*
	1 - ��ɫ	2 - ��ɫ	3 - ��ɫ	4 - ��ɫ	5 - ��ɫ
	6 - ��ɫ	7 - ��ɫ	8 - ��ɫ	9 - ��ɫ	0 - ��ɫ*/
	string place; //�ص�
	int start_frame; //��ʼ֡
	int end_frame; //����֡
	int is_abnormal_v; //�ٶ��Ƿ��쳣:
	int is_abnormal_traj; //�켣�Ƿ��쳣
	int is_violent; //�Ƿ��б����¼�

	item() {
		id = 0;
		category = 0;
		video_id = 0;
		memset(color_list, 0, sizeof(color_list));
		color_t1 = 0;
		color_t2 = 0;
		color = 1;
		place = "";
		start_frame = 0;
		end_frame = 0;
		is_abnormal_v = 0;
		is_abnormal_traj = 0;
		is_violent = 0;
	}
	~item() {
	}

private:

};

#endif