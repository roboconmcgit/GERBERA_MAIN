#include "ev3api.h"
#include "ParamFileRead.h"


/* Fileから読み出したいパラメータを以下に追加する========================================================================= */
int ParamFileRead::setParameters(void){
    //intデータ
    //GetFileData_int("XXXXX", &XXXXX);

    GetFileData_int("SONAR_DIST", &SONAR_DIST);
    GetFileData_int("GARAGE_LIT_DIST", &GARAGE_LIT_DIST);
#if 0
    GetFileData_int("SONAR_ALERT_LOOKUPGATE", &SONAR_ALERT_LOOKUPGATE);

    GetFileData_int("TAIL_ANGLE_STAND_UP", &TAIL_ANGLE_STAND_UP);
    GetFileData_int("TAIL_ANGLE_RUN", &TAIL_ANGLE_RUN);
    GetFileData_int("TAIL_ANGLE_DANSA", &TAIL_ANGLE_DANSA);
    GetFileData_int("TAIL_ANGLE_LUG", &TAIL_ANGLE_LUG);
    GetFileData_int("TAIL_ANGLE_GARAGE", &TAIL_ANGLE_GARAGE);
    GetFileData_int("RoboTread", &RoboTread);
#endif
    //floatデータ
#if 0
    //GetFileData_float("XXXXX", &XXXXX);
    GetFileData_float("FINAL_STRAIGHT_LENGTH", &FINAL_STRAIGHT_LENGTH);
    GetFileData_float("DEAD_ZONE_LENGTH", &DEAD_ZONE_LENGTH);
    GetFileData_float("STEP_TO_GARAGE_LENGTH", &STEP_TO_GARAGE_LENGTH);
    GetFileData_float("LOOKUPGATE_CORNER_LENGTH", &LOOKUPGATE_CORNER_LENGTH);
    GetFileData_float("dT_100ms", &dT_100ms);
    GetFileData_float("dT_4ms", &dT_4ms);
    GetFileData_float("PAI", &PAI);
    GetFileData_float("RAD_1_DEG", &RAD_1_DEG);
    GetFileData_float("RAD_5_DEG", &RAD_5_DEG);
    GetFileData_float("RAD_15_DEG", &RAD_15_DEG);
    GetFileData_float("RAD_30_DEG", &RAD_30_DEG);
    GetFileData_float("MINUS_RAD_5_DEG", &MINUS_RAD_5_DEG);
    GetFileData_float("MINUS_RAD_15_DEG", &MINUS_RAD_15_DEG);
    GetFileData_float("MINUS_RAD_30_DEG", &MINUS_RAD_30_DEG);
    GetFileData_float("RAD_90_DEG", &RAD_90_DEG);
    GetFileData_float("RAD_120_DEG", &RAD_120_DEG);
    GetFileData_float("RAD_315_DEG", &RAD_315_DEG);
    GetFileData_float("RAD_345_DEG", &RAD_345_DEG);
    GetFileData_float("RAD_360_DEG", &RAD_360_DEG);
    GetFileData_float("RAD_450_DEG", &RAD_450_DEG);
    GetFileData_float("WheelDiameter", &WheelDiameter);
    GetFileData_float("WHEEL_R", &WHEEL_R);
#endif
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
