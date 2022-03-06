#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h"

namespace myslam {
class Config {
 private:
  // 声明静态全局智能指针变量 config_ 这里没有初始值
  static std::shared_ptr<Config> config_;
  // 类的成员变量
  cv::FileStorage file_;
  // 声明默认构造函数
  Config(){}  // private constructor makes a singleton
 public:
  
  // 声明析构函数
  ~Config();  // close the file when deconstructing

  // set a new config file
  // 声明 读取参数的函数
  // set a new config file
  // 只要将 new 运算符返回的指针 p 交给一个 shared_ptr 对象“托管”
  // myslam::Config::setParameterFile ( argv[1] );
  static void setParameterFile(const std::string& filename);

  // access the parameter values
  // 这是一个可以获取参数的模板函数
  // 通过模板实现返回指定类型的参数
  // double fx = myslam::Config::get<double> ("Camera.fx");
  template <typename T>
  static T get(const std::string& key) {
    return T(Config::config_->file_[key]);
  }
};
}  // namespace myslam

#endif
