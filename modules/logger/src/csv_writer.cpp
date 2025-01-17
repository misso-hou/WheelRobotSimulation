#include "csv_writer.h"

using namespace std;

namespace modules {
namespace logger {

/*
 * @brief: 模板函数：convert string to int/float/double
 * @param: reference of string
 *
 * @return:
 *    num: 字符串转换后的数据类型
 */
template <class Type>
Type CSVWandR::StringToNum(string& str) {
  istringstream iss(str);
  Type num;
  iss >> num;
  return num;
}

/*
 * @brief: 创建csv文件，并清空文件内部数据
 */
void CSVWandR::CSVcreate(string file_name) {
  csvfile_.open(file_name, ios::out | ios::trunc);
  csvfile_.close();
}

/*
 * @brief: 在csv文件中按行写入固定个数数据
 * @params:
 *	  file_name:文件名
 *    a,b,c,d,e,f,g,h,i,g,h,i,j,k: CSV文件行元素
 */
void CSVWandR::CSVwriter(string file_name, float a, float b, float c, float d, float e, float f, float g, float h, float i, float j, float k, float l,
                         float m) {
  csvfile_.open(file_name, ios::app);  // 以追加的形式打开csv文件
  // 数据保存到小数点后三位
  csvfile_ << fixed << setprecision(3) << a << ',' << b << ',' << c << ',' << d << ',' << e << ',' << f << ',' << g << ',' << h << ',' << i << ','
           << j << ',' << k << ',' << l << ',' << m << endl;
  csvfile_.close();
  return;
}

/*
 * @brief: 在csv文件中按行写入不定个数数据
 * @params:
 *    data_in_row: 行元素
 */
void CSVWandR::CSVwriteRow(string file_name, const vector<float>& data_in_row) {
  csvfile_.open(file_name, ios::app);
  int data_num = data_in_row.size();
  /*有数据写入*/
  if (data_num > 0) {
    for (uint i = 0; i < data_in_row.size() - 1; i++) {
      csvfile_ << fixed << setprecision(3) << data_in_row.at(i) << ',';
    }
    csvfile_ << setprecision(3) << data_in_row.back() << endl;
  } else {
    csvfile_ << 0.0f << endl;
  }
  csvfile_.close();
  return;
}

/*
 * @breif:带有时间戳的数据
 */
void CSVWandR::RecordWithTime(string file_name, float a, float b, float c, float d, float e, float f, float g, float h, float i, float j, float k,
                              float l, float m, float n, float o) {
  csvfile_.open(file_name, ios::app);  // 以追加的形式打开csv文件
  // 数据保存到小数点后三位
  csvfile_ << fixed << setprecision(3) << a << ',' << b << ',' << c << ',' << d << ',' << e << ',' << f << ',' << g << ',' << h << ',' << i << ','
           << j << ',' << k << ',' << l << ',' << m << ',' << n << ',' << o << ',' << endl;
  /*时间戳记录*/
  time_t now = time(NULL);
  struct tm* tm_now = localtime(&now);
  stringstream ss;
  // 毫秒获取
  struct timespec start1;
  int nsec1 = 1;
  if (clock_gettime(CLOCK_REALTIME, &start1) != -1) {
    nsec1 = start1.tv_nsec / 1000000;
  }
  ss << (tm_now->tm_year + 1900) << "-" << (tm_now->tm_mon + 1) << "-" << tm_now->tm_mday << "_";
  ss << tm_now->tm_hour << "_" << tm_now->tm_min << "_" << tm_now->tm_sec << "," << to_string(nsec1);
  csvfile_ << ss.str() << endl;
  csvfile_.close();
  return;
}

}  // namespace logger
}  // namespace modules
