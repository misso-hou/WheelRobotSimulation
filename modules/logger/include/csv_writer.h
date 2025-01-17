#pragma once

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

namespace modules {
namespace logger {

/*
 * @定义csv文件写入与读取类定义
 */
class CSVWandR {
 private:
  ofstream csvfile_;  // csv写入时文件操作符
  /*模板函数：字符串转数字*/
  template <class Type>
  Type StringToNum(string& str);

 public:
  /*清空CSV文件，防止数据重合*/
  void CSVcreate(string file_name);

  /*
   * @brief: 在csv文件中按行写入固定个数数据
   * @params:
   *   file_name:文件名
   *   a,b,c,d: CSV文件行元素
   */
  template <typename T1>
  void CSVWriter(string file_name, T1 a, T1 b, T1 c) {
    csvfile_.open(file_name, ios::app);  // 以追加的形式打开csv文件
    csvfile_ << a << "," << b << ',' << c << endl;
    csvfile_.close();
    return;
  }

  /*写入csv文件数据*/
  void CSVwriter(string file_name, float a, float b, float c = 0.0f, float d = 0.0f, float e = 0.0f, float f = 0.0f, float g = 0.0f, float h = 0.0f,
                 float i = 0.0f, float j = 0.0f, float k = 0.0f, float l = 0.0f, float m = 0.0f);

  void RecordWithTime(string file_name, float a, float b, float c = 0.0f, float d = 0.0f, float e = 0.0f, float f = 0.0f, float g = 0.0f,
                      float h = 0.0f, float i = 0.0f, float j = 0.0f, float k = 0.0f, float l = 0.0f, float m = 0.0f, float n = 0.0f, float o = 0.0f);

  /*按行写入数据*/
  void CSVwriteRow(string file_name, const vector<float>& data_in_row);
};
}  // namespace logger
}  // namespace modules
