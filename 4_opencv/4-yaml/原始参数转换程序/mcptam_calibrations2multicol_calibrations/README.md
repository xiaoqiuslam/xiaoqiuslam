# mcptam_cali2multicol_cali

mcptam calibrations  files transform to  MultiCol_calibrations

need to run twice 

I'm solving the problem.

这个程序是负责参数文件的转化　下面是参数

/home/q/Downloads/mcptam_calibrations2multicol_calibrations/mcptam_calibrations/
/home/q/Downloads/mcptam_calibrations2multicol_calibrations/pol.txt
/home/q/Downloads/mcptam_calibrations2multicol_calibrations/poses.dat
/home/q/Downloads/mcptam_calibrations2multicol_calibrations/MultiCol_calibrations/


截止到2020年9月19日

尝试了　C++ 的　std 标准库　
std::ifstream  Input file stream  读取文件
std::ofstream  Output file stream 写入文件

和　opencv 的　yaml 文件读取方式　但是　感觉不对　
报了错误　OpenCV Error: Bad argument (Key names may only contain alphanumeric characters [a-zA-Z0-9], '-', '_' and ' ') in icvYMLWrite, 

意思是参数文件里面不支持　＂．＂　但是文件参数里面有　＂．＂　真是不知道怎么回事　下次试一下原生的　yaml 库吧　今天实在是没有精力了　累了
cv::FileStorage cv_file_in("cv_file_read.yaml", FileStorage::READ);    读取文件
cv::FileStorage cv_file_out("cv_file_out.yaml", FileStorage::WRITE);　 写入文件
