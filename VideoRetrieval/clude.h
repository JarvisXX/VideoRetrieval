#include"cv.h"
#include "cvaux.h"

//#include <windows.h>
#include "highgui.h"
#include "cxcore.h"
#include "cvaux.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <locale.h>
#include <ml.h> 
#include <string.h>
#include <vector>
#include <queue>
#include <list>

using namespace cv;
using namespace std;



//#include <afx.h>


#ifndef GLOBAL_H
#define GLOBAL_H



void ucArr2Img(IplImage* img, unsigned char* data, int iW);
void Img2ucArr(IplImage* img, unsigned char* data, int iW);

#endif
