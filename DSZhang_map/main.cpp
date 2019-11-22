#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <sstream>
#include <string.h>

using namespace std;
struct Point_map {
    int x;
    int y;
};

//static int *split_num(char * str,char *split,int count)
//{
//    if(strlen(str)==0 || strlen(split)==0)
//        return NULL;
//    if(count <=1) return NULL;
//    int * ints=new int[count];
//    memset(ints,0x0,count*sizeof(int));
//
//    char split_str[48];
//    int index_length=0;
//
//    snprintf(split_str,48,"%%d%s",split);
//    for (int i=0;i<count;i++){
//        sscanf(str+index_length,split_str,&ints[i]);
//        char num_str[20]={0};
//        //sprintf(num_str,"%d",ints[i]);
//        index_length+=strlen(num_str)+strlen(split);
//    }
//    return ints;
//}
int Grial(char *&dist, char *str) {
    //判断表达式是否合理，此处我就不用栈了，
    //栈还挺浪费空间的,直接上一个标识flags=0，
    //遇到'(',flags++,遇到')',flags--,if(flags==0)
    //则这个表达式正确，否则-1返回 。
    int flags = 0;
    dist = new char[strlen(str) + 1];
    char *q = dist;
    const char *p = str;
    while (*p != '\0') {
        if (*p == '(')
            flags++;
        if (*p == ')')
            flags--;
        p++;
    }
    if (flags != 0)return -1;
    p = str;
    while (*p != '\0') {
        if (*p == '(' || *p == ')') {
            p++;
            continue;
        } else {
            *q++ = *p++;
        }
    }
    *q = '\0';
    return 0;
}

//int Grial_space(char *&dist,char *str)
//{
//    //判断表达式是否合理，此处我就不用栈了，
//    //栈还挺浪费空间的,直接上一个标识flags=0，
//    //遇到'(',flags++,遇到')',flags--,if(flags==0)
//    //则这个表达式正确，否则-1返回 。
//    int flags = 0;
//    dist = new char[strlen(str)+1];
//    char *q = dist;
//    const char *p = str;
//    while (*p!='\0')
//    {
//        if (*p == ' ')
//            flags++;
//        if (*p == ' ')
//            flags--;
//        p++;
//    }
//    if (flags != 0)return -1;
//    p = str;
//    while (*p != '\0')
//    {
//        if (*p == ' ' || *p == ' ')
//        {
//            p++;
//            continue;
//        }
//        else
//        {
//            *q++ = *p++;
//        }
//    }
//    *q = '\0';
//    return 0;
//}
void part(string _str, vector<int> &_num) {
    int sum = 0;
    int order = 0;;
    unsigned int i = 0;
    int neg_flag = 0;
    while (i < _str.length()) {
        if ('0' <= _str.at(i) && _str.at(i) <= '9') {
            //还原连续的数字
            sum = sum * 10 + (_str.at(i) - '0');
        } else if ('-' == _str.at(i)) {
            neg_flag = 1;
        } else {
            if (sum != 0) {
                if (neg_flag == 0) {
                    _num[order++] = sum;
                    sum = 0;
                } else {
                    sum = sum * -1;
                    _num[order++] = sum;
                    neg_flag = 0;
                    sum = 0;
                }
            }
        }

        i++;
    }
}

int y_search(vector<vector<int> > cross_info_all, int cross_num, int cross_id, vector<int> &x_point,
             vector<int> &y_point)//返回新的cross_id
{
    for (int i = 0; i < cross_num && i != cross_id; i++) {
        if (cross_info_all[cross_id][1] == cross_info_all[i][3]) {
            y_point[i] = y_point[cross_id] + 1;
            x_point[i] = x_point[cross_id];
            //cout<<"changed ---------"<<endl;
            cross_id = i;
        }
    }
    if (cross_info_all[cross_id][1] != -1) {
        return cross_id;
    } else {
        return -1;
    }
}

int
x_search(vector<vector<int> > cross_info_all, int cross_num, int cross_id, vector<int> &x_point, vector<int> &y_point) {
    for (int i = 0; i < cross_num && i != cross_id; i++) {
        if (cross_info_all[cross_id][2] == cross_info_all[i][4]) {
            x_point[i] = x_point[cross_id] + 1;
            y_point[i] = y_point[cross_id];
            // cout<<"already changed ---------"<<endl;
            cross_id = i;
        }
    }
    return cross_id;
}

int main() {
    ifstream crossfile("SDK/SDK_C++/config_2/cross.txt");
    string info_temp;
    int cross_num;
    vector<int> cross_info(5);
    vector<vector<int> > cross_info_all;
    getline(crossfile, info_temp);
    while (getline(crossfile, info_temp)) {
        cout<< info_temp << endl;
//         char *s_input = (char *)info_temp.c_str();
//        char *out_put;
//        Grial(out_put,s_input);
//        Grial_space(s_input,out_put);
        part(info_temp, cross_info);
         for(int i = 0; i < 5; i++)
         {
             cout << cross_info[i] << "  ";
         }
         cout << endl;
        cross_info_all.push_back(cross_info);
    }
    cross_num = cross_info_all.size();
    cout<<"size ======"<<cross_num<<endl;
//    int cross_id = 0;
//    int cross_id_new = 0;
//    int cross_id_original = 0;
//    int loop = 0;
//    int out_flag = 0; //判断x是否已经到达最右
//    vector<int> x_point(cross_num, 0);
//    vector<int> y_point(cross_num, 0);
//    for (int i = 0; i < 36; i++) {
//        x_point.push_back(0);
//        y_point.push_back(0);
//    }
//    if (cross_id == 0) {
//        x_point[cross_id] = 0;
//        y_point[cross_id] = 0;
//    }
//    while (out_flag == 0) {
//        cross_id_original = cross_id;
//        while (cross_id >= 0) {
//            cross_id_new = y_search(cross_info_all, cross_num, cross_id, x_point, y_point);
//            //cout<<"y_research"<<endl;
//            cross_id = cross_id_new;
//        }
//        if (cross_info_all[cross_id_original][2] != -1) {
//            cross_id_new = x_search(cross_info_all, cross_num, cross_id_original, x_point, y_point);
//            cross_id = cross_id_new;
//        } else {
//            out_flag = 1;
//        }
//    }
//    for (int i = 0; i < cross_num; i++) {
//        cout << x_point[i] << "     " << y_point[i] << endl;
//    }
    return 0;

}