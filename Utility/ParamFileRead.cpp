#include "ev3api.h"
#include "ParamFileRead.h"


/* Fileから読み出したいパラメータを以下に追加する========================================================================= */
int ParamFileRead::setParameters(void){
    //intデータ
    //GetFileData_int("XXXXX", &XXXXX);
    GetFileData_int("TAIL_ANGLE_STAND_UP", &TAIL_ANGLE_STAND_UP);
    GetFileData_int("TAIL_ANGLE_DANSA", &TAIL_ANGLE_DANSA);
    GetFileData_int("TAIL_ANGLE_LUG", &TAIL_ANGLE_LUG);
    GetFileData_int("TAIL_ANGLE_GARAGE", &TAIL_ANGLE_GARAGE);
    GetFileData_int("STOP_POS_FROM_LUG", &STOP_POS_FROM_LUG);
    GetFileData_int("STOP_POS_APP_LUG", &STOP_POS_APP_LUG);

    //floatデータ
    //GetFileData_float("XXXXX", &XXXXX);
    GetFileData_float("APPROACH_TO_1st_LUG", &APPROACH_TO_1st_LUG);
    GetFileData_float("APPROACH_TO_2nd_LUG", &APPROACH_TO_2nd_LUG);
    GetFileData_float("APPROACH_TO_3rd_LUG", &APPROACH_TO_3rd_LUG);
    GetFileData_float("LUG_1st_STOP", &LUG_1st_STOP);
    GetFileData_float("LUG_2nd_STOP", &LUG_2nd_STOP);
    GetFileData_float("LUG_3rd_STOP", &LUG_3rd_STOP);
    GetFileData_float("GARAGE_OFFSET_ANGLE", &GARAGE_OFFSET_ANGLE);
    return 0;
}

/*==以下は制御用なので変更不要===========================================================================================*/
int ParamFileRead::fileRead(void){
    ifstream ifs("parameter.csv");
    //  char str[256] = {'\0'};
    string str; 
    int i = 0;
    if (ifs.fail())
    {
        ev3_lcd_draw_string("error", 0, 32);
        return -1;
    }
    while(getline(ifs,str))
    {
        if(i >= LIST_MAX){
            return -1;
        }
        vector<string> strList;
        strList = split(str, ',');
        file_data_list[i].dataName = strList[1];
        if(strList[0] == "int"){
            file_data_list[i].IsFloat = false;
            file_data_list[i].int_data = atoi(strList[2].c_str());
        }else if(strList[0] == "float"){
            file_data_list[i].IsFloat = true;
            file_data_list[i].float_data = atof(strList[2].c_str());
        }else{
            file_data_list[i].dataName = "unknown";
        }
#if 0
        static char buf[256];
        if(file_data_list[i].IsFloat==false){
            sprintf(buf, " %s = %d\n", file_data_list[i].dataName.c_str(), file_data_list[i].int_data);
        }else{
            sprintf(buf, " %s = %f\n", file_data_list[i].dataName.c_str(), file_data_list[i].float_data);
        }

        ev3_lcd_draw_string(buf, 0, i*10);
#endif
        i++;
    }
    i = 0;
    return 0;
}


//split 
vector<string> ParamFileRead::split(const string &str, char sep)
{
    vector<string> v;
    stringstream ss(str);
    string buffer;
    while( getline(ss, buffer, sep) ) {
        v.push_back(buffer);
    }
    return v;
}

int ParamFileRead::GetFileData_int(string dataName, int * retData){
    if( retData == nullptr ) return -1;
    for(int i=0; i < LIST_MAX; i++){
        if(file_data_list[i].dataName == "unknown") return -1;
        if((file_data_list[i].IsFloat==false)&&(dataName == file_data_list[i].dataName)){
            *retData = file_data_list[i].int_data;
            return 0;
        }
    }
    return -1;
}

int ParamFileRead::GetFileData_float(string dataName, float * retData){
    if( retData == nullptr ) return -1;
    for(int i=0; i < LIST_MAX; i++){
        if(file_data_list[i].dataName == "unknown") return -1;
        if((file_data_list[i].IsFloat==true)&&(dataName == file_data_list[i].dataName)){
            *retData = file_data_list[i].float_data;
            return 0;
        }
    }
    return -1;
}
