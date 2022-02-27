cmake_minimum_required(VERSION 2.8)

project(opencv)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

add_executable(orb_cv orb_cv.cpp)
target_link_libraries(orb_cv ${OpenCV_LIBS})

add_executable(orb_self orb_self.cpp)
target_link_libraries(orb_self ${OpenCV_LIBS})

# 创建一张灰度图像
add_executable(0_create_CV_8UC1 0_create_CV_8UC1.cpp)
target_link_libraries(0_create_CV_8UC1 ${OpenCV_LIBS})

# 创建一张彩色图像
add_executable(0_create_CV_8UC3 0_create_CV_8UC3.cpp)
target_link_libraries(0_create_CV_8UC3 ${OpenCV_LIBS})

# 创建一张四通道图像
add_executable(0_create_CV_8UC4 0_create_CV_8UC4.cpp)
target_link_libraries(0_create_CV_8UC4 ${OpenCV_LIBS})

# 绘制基本图形 ellipse circle fillPoly line
add_executable(20_DrawImage 20_DrawImage.cpp)
target_link_libraries(20_DrawImage ${OpenCV_LIBS})

# 鼠标绘制矩形
add_executable(18_UseMouse 18_UseMouse.cpp)
target_link_libraries(18_UseMouse ${OpenCV_LIBS})

# 图像基础操作
add_executable(imageBasics imageBasics.cpp)
target_link_libraries(imageBasics ${OpenCV_LIBS})

# 读取图像
add_executable(1_read 1_read.cpp)
target_link_libraries(1_read ${OpenCV_LIBS})


# 图像重命名
add_executable(rename "rename.cpp")
target_link_libraries(rename ${OpenCV_LIBS})

# 图像　点　矩形框　去畸变
add_executable(undisort undisort.cpp)
target_link_libraries(undisort  ${OpenCV_LIBS})

#图像修补示例
add_executable(78_inpaint 78_inpaint.cpp)
target_link_libraries(78_inpaint ${OpenCV_LIBS})

#H-S二维直方图的绘制
add_executable(79_H-S-Histogram 79_H-S-Histogram.cpp)
target_link_libraries(79_H-S-Histogram ${OpenCV_LIBS})


#一维直方图的绘制
add_executable(80_Histogram2 80_Histogram2.cpp)
target_link_libraries(80_Histogram2 ${OpenCV_LIBS})

#绘制RGB三色分量的直方图
add_executable(81_histogram3 81_histogram3.cpp)
target_link_libraries(81_histogram3 ${OpenCV_LIBS})

# 直方图对比
add_executable(82_compareHist 82_compareHist.cpp)
target_link_libraries(82_compareHist ${OpenCV_LIBS})

#用指针访问像素
add_executable(21_UsePointerAccessPixel 21_UsePointerAccessPixel.cpp)
target_link_libraries(21_UsePointerAccessPixel ${OpenCV_LIBS})

#用迭代器访问像素
add_executable(22_UseIteratorAccessPixel 22_UseIteratorAccessPixel.cpp)
target_link_libraries(22_UseIteratorAccessPixel ${OpenCV_LIBS})

# 用指针 at 访问像素
add_executable(23_UseAtAccessPixel 23_UseAtAccessPixel.cpp)
target_link_libraries(23_UseAtAccessPixel ${OpenCV_LIBS})

add_executable(24_14WaysToAccessPixel 24_14WaysToAccessPixel.cpp)
target_link_libraries(24_14WaysToAccessPixel ${OpenCV_LIBS})

# 图像对比度、亮度值调整
add_executable(27_ChangeContrastAndBright 27_ChangeContrastAndBright.cpp)
target_link_libraries(27_ChangeContrastAndBright ${OpenCV_LIBS})

# 读取视频
add_executable(4_video_capture 4_video_capture.cpp)
target_link_libraries(4_video_capture ${OpenCV_LIBS})

# 打开摄像头
add_executable(5_video_capture 5_video_capture.cpp)
target_link_libraries(5_video_capture ${OpenCV_LIBS})


# 图像叠加
add_executable(16_addWeighted 16_addWeighted.cpp)
target_link_libraries(16_addWeighted ${OpenCV_LIBS})

# 图像叠加 回调函数
add_executable(17_addWeighted 17_addWeighted.cpp)
target_link_libraries(17_addWeighted ${OpenCV_LIBS})

# 图像叠加
add_executable(25_ImageBlending 25_ImageBlending.cpp)
target_link_libraries(25_ImageBlending ${OpenCV_LIBS})

# 图像叠加 三通道分别叠加
add_executable(26_MultiChannelImageBlending 26_MultiChannelImageBlending.cpp)
target_link_libraries(26_MultiChannelImageBlending ${OpenCV_LIBS})

# resize函数用法示例
add_executable(51_resize 51_resize.cpp)
target_link_libraries(51_resize ${OpenCV_LIBS})

# pyrUp函数用法示例
add_executable(52_pyrUp 52_pyrUp.cpp)
target_link_libraries(52_pyrUp ${OpenCV_LIBS})

# pyrDown函数用法示例
add_executable(53_pyrDown 53_pyrDown.cpp)
target_link_libraries(53_pyrDown ${OpenCV_LIBS})

# 图像金字塔和resize综合示例
add_executable(54_PyrAndResize 54_PyrAndResize.cpp)
target_link_libraries(54_PyrAndResize ${OpenCV_LIBS})

# remap函数用法示例
add_executable(65_remap 65_remap.cpp)
target_link_libraries(65_remap ${OpenCV_LIBS})

# 实现多种重映射综合示例
add_executable(66_remap2 66_remap2.cpp)
target_link_libraries(66_remap2 ${OpenCV_LIBS})

# 仿射变换综合示例
add_executable(67_AffineTransform 67_AffineTransform.cpp)
target_link_libraries(67_AffineTransform ${OpenCV_LIBS})

# 直方图均衡化
add_executable(68_equalizeHist 68_equalizeHist.cpp)
target_link_libraries(68_equalizeHist ${OpenCV_LIBS})

# OpenCV基本阈值操作
add_executable(55_threshold 55_threshold.cpp)
target_link_libraries(55_threshold ${OpenCV_LIBS})



#
add_executable(32_blur 32_blur.cpp)
target_link_libraries(32_blur ${OpenCV_LIBS})

add_executable(33_GaussianBlur 33_GaussianBlur.cpp)
target_link_libraries(33_GaussianBlur ${OpenCV_LIBS})

add_executable(34_LinearImageFilter 34_LinearImageFilter.cpp)
target_link_libraries(34_LinearImageFilter ${OpenCV_LIBS})

add_executable(35_medianBlur 35_medianBlur.cpp)
target_link_libraries(35_medianBlur ${OpenCV_LIBS})

add_executable(36_bilateralFilter 36_bilateralFilter.cpp)
target_link_libraries(36_bilateralFilter ${OpenCV_LIBS})

#五种图像滤波综合示例
add_executable(37_ImageFiltering 37_ImageFiltering.cpp)
target_link_libraries(37_ImageFiltering ${OpenCV_LIBS})

add_executable(31_boxFilter 31_boxFilter.cpp)
target_link_libraries(31_boxFilter ${OpenCV_LIBS})

# 膨胀
add_executable(38_dilate 38_dilate.cpp)
target_link_libraries(38_dilate ${OpenCV_LIBS})

# 腐蚀
add_executable(2_erode_blur 2_erode_blur.cpp)
target_link_libraries(2_erode_blur ${OpenCV_LIBS})

add_executable(39_erode 39_erode.cpp)
target_link_libraries(39_erode ${OpenCV_LIBS})

# 40_图像腐蚀与膨胀综合示例
add_executable(40_dilate_And_erode 40_dilate_And_erode.cpp)
target_link_libraries(40_dilate_And_erode ${OpenCV_LIBS})

# 用morphologyEx进行图像膨胀
add_executable(41_DilationEX 41_DilationEX.cpp)
target_link_libraries(41_DilationEX ${OpenCV_LIBS})

add_executable(42_ErodeEX 42_ErodeEX.cpp)
target_link_libraries(42_ErodeEX ${OpenCV_LIBS})

# 用morphologyEx进行图像形态学开运算用法示例
add_executable(43_Opening 43_Opening.cpp)
target_link_libraries(43_Opening ${OpenCV_LIBS})

# 用morphologyEx进行图像闭运算
add_executable(44_Closing 44_Closing.cpp)
target_link_libraries(44_Closing ${OpenCV_LIBS})

# 用morphologyEx进行形态学梯度运算
add_executable(45_Gradient 45_Gradient.cpp)
target_link_libraries(45_Gradient ${OpenCV_LIBS})

# 用morphologyEx进行形态学顶帽运算
add_executable(46_TopHat 46_TopHat.cpp)
target_link_libraries(46_TopHat ${OpenCV_LIBS})

# 用morphologyEx进行形态学黑帽运算
add_executable(47_BlackHat 47_BlackHat.cpp)
target_link_libraries(47_BlackHat ${OpenCV_LIBS})

# 形态学图像处理——开运算、闭运算、形态学梯度、顶帽、黑帽
add_executable(48_Morphology 48_Morphology.cpp)
target_link_libraries(48_Morphology ${OpenCV_LIBS})

# 漫水填充floodFill函数用法示例
add_executable(49_floodFill 49_floodFill.cpp)
target_link_libraries(49_floodFill ${OpenCV_LIBS})

# 漫水填充算法综合示例
add_executable(50_floodFill2 50_floodFill2.cpp)
target_link_libraries(50_floodFill2 ${OpenCV_LIBS})

add_executable(3_canny 3_canny.cpp)
target_link_libraries(3_canny ${OpenCV_LIBS})

# canny函数用法示例
add_executable(56_canny 56_canny.cpp)
target_link_libraries(56_canny ${OpenCV_LIBS})

# 边缘检测综合示例Canny算子,Sobel算子,Laplace算子,Scharr滤波器合辑
add_executable(60_EdgeDetection 60_EdgeDetection.cpp)
target_link_libraries(60_EdgeDetection ${OpenCV_LIBS})


# sobel函数用法示例
add_executable(57_sobel 57_sobel.cpp)
target_link_libraries(57_sobel ${OpenCV_LIBS})

#Laplacian函数用法示例
add_executable(58_Laplacian 58_Laplacian.cpp)
target_link_libraries(58_Laplacian ${OpenCV_LIBS})

#Scharr函数用法示例
add_executable(59_Scharr 59_Scharr.cpp)
target_link_libraries(59_Scharr ${OpenCV_LIBS})

# HoughLines函数用法示例
add_executable(61_HoughLines 61_HoughLines.cpp)
target_link_libraries(61_HoughLines ${OpenCV_LIBS})
# HoughLinesP函数用法示例
add_executable(62_HoughLinesP 62_HoughLinesP.cpp)
target_link_libraries(62_HoughLinesP ${OpenCV_LIBS})

# HoughCircles函数用法示例
add_executable(63_HoughCircles 63_HoughCircles.cpp)
target_link_libraries(63_HoughCircles ${OpenCV_LIBS})

#霍夫线变换综合示例
add_executable(64_HoughLines 64_HoughLines.cpp)
target_link_libraries(64_HoughLines ${OpenCV_LIBS})

#基础轮廓查找——findContours+drawContours
add_executable(69_findContours 69_findContours.cpp)
target_link_libraries(69_findContours ${OpenCV_LIBS})

#查找并绘制轮廓综合示例
add_executable(70_FindAndDrawContours 70_FindAndDrawContours.cpp)
target_link_libraries(70_FindAndDrawContours ${OpenCV_LIBS})

#凸包检测基础
add_executable(71_convexHull 71_convexHull.cpp)
target_link_libraries(71_convexHull ${OpenCV_LIBS})

#创建包围轮廓的矩形边界
add_executable(73_minAreaRect 73_minAreaRect.cpp)
target_link_libraries(73_minAreaRect ${OpenCV_LIBS})

# 创建包围轮廓的圆形边界
add_executable(74_minEnclosingCircle 74_minEnclosingCircle.cpp)
target_link_libraries(74_minEnclosingCircle ${OpenCV_LIBS})

#创建轮廓边界框
add_executable(75_approxPolyDP 75_approxPolyDP.cpp)
target_link_libraries(75_approxPolyDP ${OpenCV_LIBS})

# 查找和绘制图片轮廓矩
add_executable(76_ContourMoment 76_ContourMoment.cpp)
target_link_libraries(76_ContourMoment ${OpenCV_LIBS})

# 寻找和绘制物体的凸包
add_executable(72_convexHull2 72_convexHull2.cpp)
target_link_libraries(72_convexHull2 ${OpenCV_LIBS})

#分水岭算法综合示例
add_executable(77_watershed 77_watershed.cpp)
target_link_libraries(77_watershed ${OpenCV_LIBS})



# 反向投影示例程序
add_executable(83_calcBackProject 83_calcBackProject.cpp)
target_link_libraries(83_calcBackProject ${OpenCV_LIBS})

# 模板匹配
add_executable(84_matchTemplate 84_matchTemplate.cpp)
target_link_libraries(84_matchTemplate ${OpenCV_LIBS})

# cornerHarris函数用法示例
add_executable(85_cornerHarris 85_cornerHarris.cpp)
target_link_libraries(85_cornerHarris ${OpenCV_LIBS})

#Harris角点检测
add_executable(86_cornerHarris 86_cornerHarris.cpp)
target_link_libraries(86_cornerHarris ${OpenCV_LIBS})

#Shi-Tomasi角点检测示例
add_executable(87_goodFeaturesToTrack 87_goodFeaturesToTrack.cpp)
target_link_libraries(87_goodFeaturesToTrack ${OpenCV_LIBS})

# 亚像素级角点检测
add_executable(88_cornerSubPix 88_cornerSubPix.cpp)
target_link_libraries(88_cornerSubPix ${OpenCV_LIBS})


add_executable(8_CamShiftDemo 8_CamShiftDemo.cpp)
target_link_libraries(8_CamShiftDemo ${OpenCV_LIBS})

add_executable(9_OpticalFlow 9_OpticalFlow.cpp)
target_link_libraries(9_OpticalFlow ${OpenCV_LIBS})

# add_executable(10_lkdemo 10_lkdemo.cpp)
# target_link_libraries(10_lkdemo ${OpenCV_LIBS})


add_executable(11_ObjectDetection 11_ObjectDetection.cpp)
target_link_libraries(11_ObjectDetection ${OpenCV_LIBS})


#add_executable(12_SupportVectorMachines 12_SupportVectorMachines.cpp)
#target_link_libraries(12_SupportVectorMachines ${OpenCV_LIBS})



# add_executable(13_SVMForNonLinearlySeparableData 13_SVMForNonLinearlySeparableData.cpp)
# target_link_libraries(13_SVMForNonLinearlySeparableData ${OpenCV_LIBS})



#离散傅里叶变换
add_executable(28_DFT 28_DFT.cpp)
target_link_libraries(28_DFT ${OpenCV_LIBS})


add_executable(29_Write_XML_and_YAML_File 29_Write_XML_and_YAML_File.cpp)
target_link_libraries(29_Write_XML_and_YAML_File ${OpenCV_LIBS})

add_executable(30_Read_XML_and_YAML_File 30_Read_XML_and_YAML_File.cpp)
target_link_libraries(30_Read_XML_and_YAML_File ${OpenCV_LIBS})


