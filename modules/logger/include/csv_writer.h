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

namespace modules {
namespace logger {

using namespace std;

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
  void CSVwriter(string file_name, const vector<float>& data);
  /*记录带有时间戳的数据*/
  void RecordWithTime(string file_name, const vector<float>& data);
  /*按行写入数据*/
  void CSVwriteRow(string file_name, const vector<float>& data_in_row);
};
}  // namespace logger
}  // namespace modules
