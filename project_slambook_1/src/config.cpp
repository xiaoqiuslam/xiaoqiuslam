#include "myslam/config.h"

/**
 * 一旦定义了命名空间 头文件和源码文件都要放在命名空间中
 * 目的是为了隔离变量的作用域
 */
namespace myslam {
/**
 * @brief 这里我们更加充分理解了 声明 定义 变量在整个项目中的作用
 * 一切都是为了变量 定义 读取参数的函数

 * @param filename
 * myslam::Config::setParameterFile ( argv[1] );这里调用传参数
 */
void Config::setParameterFile(const std::string& filename) {
  // 判断智能指针对象是否存在，不存在才会创建一个 config_ 对象
  // 只要将 new 运算符返回的指针 p 交给一个 shared_ptr 对象“托管”
  if (config_ == nullptr) config_ = shared_ptr<Config>(new Config);
  // 类的成员变量 操作 这里是读取文件
  config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
  if (config_->file_.isOpened() == false) {
    std::cerr << "parameter file " << filename << " does not exist."
              << std::endl;
    config_->file_.release();
    return;
  }
}



// 定义析构函数
Config::~Config() {
  if (file_.isOpened()) file_.release();
}


// 定义静态全局智能指针变量 config_ 就是给出初始值的意思
shared_ptr<Config> Config::config_ = nullptr;

}  // namespace myslam
