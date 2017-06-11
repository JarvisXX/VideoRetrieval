#include"clude.h"

#ifndef ITEM_H
#define ITEM_H

class item
{
public:
	int id; //编号
	int category; //语义 0-vehicle  1-pedstrain
	int video_id; //视频编号
	int color_list[10];
	int color_t1, color_t2;
	/*
	0 - red(5)	1 - orange	2 - yellow	3 - green	4 - violet
	5 - red		6 - white	7 - grey	8 - black	9 - blue*/
	int color; //颜色
	/*
	1 - 黑色	2 - 灰色	3 - 白色	4 - 红色	5 - 橙色
	6 - 黄色	7 - 绿色	8 - 青色	9 - 蓝色	0 - 紫色*/
	string place; //地点
	int start_frame; //起始帧
	int end_frame; //结束帧
	int is_abnormal_v; //速度是否异常:
	int is_abnormal_traj; //轨迹是否异常
	int is_violent; //是否有暴力事件

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