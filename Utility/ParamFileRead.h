
#ifndef PARAMFILEREAD_H_
#define PARAMFILEREAD_H_

#include "parameter.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <stdlib.h>

using namespace std;

#define LIST_MAX	(50)

typedef struct file_data{
    string dataName = "unknown";
    bool IsFloat = false;
    int int_data = 0;
    float float_data = 0.0;
}FILE_DATA;

class ParamFileRead {
public:
	int fileRead(void);			// Fileからデータを読み出す
	int setParameters(void);	// Fileのデータをparameter.cの変数にセットする
    int GetFileData_int(string dataName, int * retData);	// Fileにあるデータを取得する（int版）
    int GetFileData_float(string dataName, float * retData);	// Fileにあるデータを取得する（float版）
private:
    FILE_DATA file_data_list[LIST_MAX];
	vector<string> split(const string &str, char sep);
};


#endif // !PARAMFILEREAD_H_
