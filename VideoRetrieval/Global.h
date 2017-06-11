#include"Rule.h"
#include"item.h"
rule *Rule;
void screnere_train();
string screnere_test(Mat frametest);
CvSVM svm;
int item_id = 0;
int video_id = 0;
int item_sum = 0;
vector<item> item_list;
bool is_new_video = TRUE;
#define PedHalfSearchWidth 16