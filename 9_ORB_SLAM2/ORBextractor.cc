/**
* This file is part of ORB-SLAM2.
* This file is based on the file orb.cpp from the OpenCV library (see BSD license below).
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
/**
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>

#include "ORBextractor.h"


using namespace cv;
using namespace std;

namespace ORB_SLAM2
{

const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;
const int EDGE_THRESHOLD = 19;

// keypoint->angle = IC_Angle(image, keypoint->pt, umax);
static float IC_Angle(const Mat& image, Point2f pt,  const vector<int> & u_max)
{
    // cv::imwrite("./IC_Angle.png",image);


    // .at<uchar>(), .ptr<uchar>()获取像素值
    // 一般使用的灰度图像元素是8位的uchar型(注意不是无符号整型, 而是无符号字符型, 元素值0-255）
    const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));
    // std::cout << "image.at<uchar> (cvRound(pt.y), cvRound(pt.x)) " << int(image.at<uchar> (cvRound(pt.y), cvRound(pt.x))) << std::endl;

    // Treat the center line differently, v=0 -> -15 < u < 15
    // 理解 cv::Mat 在内存中的存储和像素的索引规则
    // 特征点所在行的每一个像素值 center[u] center = image.at<uchar> (cvRound(pt.y), cvRound(pt.x))
    // u -15
    // center[u] 88
    // m_10 0
    // m_10 += u * center[u] -1320

    // u -14
    // center[u] 86
    // m_10 -1320
    // m_10 += u * center[u] -2524

    // X乘像素值 求和
    int m_01 = 0;
    // y乘像素值 求和
    int m_10 = 0;
    // 只算 特征点所在一行的 X乘像素值的和
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u){
        // std::cout << "u " << u << std::endl;
        // std::cout << "center[u] " << float(center[u]) << std::endl;
        // std::cout << "m_10 " << m_10 << std::endl;
        m_10 += u * center[u];
        // std::cout << "m_10 += u * center[u] " << m_10 << std::endl;
    }

    // Go line by line in the circular patch 理解 cv::Mat 在内存中的存储和像素的索引规则
    /**
     * 假设图像为 1024*768 感兴趣区域ROI为 400*200
     * 访问这个感兴趣区域的下一行,图像数据指针的步长step应该为 1024 而不是 200
     * |***| CV_8UC3 彩色图 一个像素需要3个字节24位存储
     * |*| CV_8UC1 灰度图 一个像素需要1个字节8位存储
     * CV_8UC1(CV_位数+数据类型+通道数 C1~C4表示对应的通道数，即有1~4个通道)8位无符号单通道->灰度图片
     */
    int step = (int)image.step1();// 790
    // std::cout << "image.cols " << image.cols << endl;// 752
    // std::cout << "image.rows " << image.rows << endl;// 480

    // std::cout << "(int)image.step1() " << image.step1() << endl;// 790
    // std::cout << "(int)image.step " << image.step << endl;// 790

    /**
     *                                  Y
     *                             * * *| * * * umax[15] = 3
     *                       * * * * * *| * * * * * * umax[14] = 6
     *                   * * * * * * * *| * * * * * * * * umax[13] = 8
     *                 * * * * * * * * *| * * * * * * * * * umax[12] = 9
     *               * * * * * * * * * *| * * * * * * * * * * umax[11] = 10
     *             * * * * * * * * * * *| * * * * * * * * * * * umax[10] = 11
     *           * * * * * * * * * * * *| * * * * * * * * * * * * umax[9] = 12
     *         * * * * * * * * * * * * *| * * * * * * * * * * * * * umax[8] = 13
     *         * * * * * * * * * * * * *| * * * * * * * * * * * * * umax[7] = 13
     *       * * * * * * * * * * * * * *| * * * * * * * * * * * * * * umax[6] = 14
     *       * * * * * * * * * * * * * *| * * * * * * * * * * * * * * umax[5] = 14
     *       * * * * * * * * * * * * * *| * * * * * * * * * * * * * * umax[4] = 14
     *     * * * * * * * * * * * * * * *| * * * * * * * * * * * * * * * umax[3] = 15
     *     * * * * * * * * * * * * * * *| * * * * * * * * * * * * * * * umax[2] = 15
     *     * * * * * * * * * * * * * * *| * * * * * * * * * * * * * * * umax[1] = 15
     *     * * * * * * * * * * * * * * *| * * * * * * * * * * * * * * *-->X umax[0] = 15  umax 16个数值依次是：15 15 15 15 14 14 14 13 13 12 11 10 9 8 6 3
     *     * * * * * * * * * * * * * * *| * * * * * * * * * * * * * * *
     *     * * * * * * * * * * * * * * *| * * * * * * * * * * * * * * *
     *     * * * * * * * * * * * * * * *| * * * * * * * * * * * * * * *
     *       * * * * * * * * * * * * * *| * * * * * * * * * * * * * *
     *       * * * * * * * * * * * * * *| * * * * * * * * * * * * * *
     *       * * * * * * * * * * * * * *| * * * * * * * * * * * * * *
     *         * * * * * * * * * * * * *| * * * * * * * * * * * * *
     *         * * * * * * * * * * * * *| * * * * * * * * * * * * *
     *           * * * * * * * * * * * *| * * * * * * * * * * * *
     *             * * * * * * * * * * *| * * * * * * * * * * *
     *               * * * * * * * * * *| * * * * * * * * * *
     *               * * * * * * * * * *| * * * * * * * * * *
     *                 * * * * * * * * *| * * * * * * * * *
     *                 * * * * * * * * *| * * * * * * * * *
     *                   * * * * * * * *| * * * * * * * *
     *                       * * * * * *| * * * * * *
     *                             * * *| * * *
     */
     // 遍历 行
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
    {
        // Proceed over the two lines
        int v_sum = 0;
        int d = u_max[v];
        // 遍历 列 - u_max[v] < u < u_max[v]
        for (int u = -d; u <= d; ++u)
        {
            // 特征点所在行的上面第v行(1 < v < 15)的第u个像素值
            int val_plus = center[u + v*step];
            // 求质量心的 圆形区域内 每个像素坐标u(X)值 乘像素值center[] 求和  m_01
            m_10 += u * val_plus;

            // 特征点所在行的下面第v行(1 < v < 15)的第u个像素值
            int val_minus = center[u - v*step];
            m_10 += u * val_minus;

            // 每次计算的两个点坐标,中心线下方为(x, y), 中心线上方为(x, -y)
            // 每次计算的两个点坐标：m_10 = Σ x*I(x,y) =  x*I(x,y) +  x*I(x,-y) = x*(I(x,y) + I(x,-y))
            // 每次计算的两个点坐标：m_01 = Σ y*I(x,y) =  y*I(x,y) + -y*I(x,-y) = y*(I(x,y) - I(x,-y))
            v_sum += (val_plus - val_minus);
            // y*(I(x,y) - I(x,-y)) + y*(I(x,y) - I(x,-y)) + y*(I(x,y) - I(x,-y));
            // y*( (I(x,y) - I(x,-y)) + (I(x,y) - I(x,-y)) + (I(x,y) - I(x,-y)) );
            // 在这个小循环里面 y 一直是 1 ,可以将y提出来, 然后将 m_01 += v * v_sum; 放到了循环外面
            // 等价写法 m_01 += v * v_sum
        }
        // 求质量心的 圆形区域内 每个像素坐标v(Y)值 乘像素值center[] 求和  m_10
        m_01 += v * v_sum;
    }

    // float fastAtan2(float y,float x)  向量的x坐标 向量的y坐标 输入一个2维向量,计算这个向量的方向,以度为单位范围是0度-360度
    // fastAtan2函数得出的角度是以X轴正方向为0°方向, 然后角度确定按照逆时针方向以360°为终点,角度范围0°- 360°
    return fastAtan2((float)m_01, (float)m_10);
}// 如何求质心 https://blog.csdn.net/qq_21950671/article/details/107092044


// 角度转弧度
const float factorPI = (float)(CV_PI/180.f);
// 全局的静态函数,只能是在本文件内被调用
// 维度为32*8 = 256 bit
// https://zhuanlan.zhihu.com/p/61738607
static void computeOrbDescriptor(const KeyPoint& kpt, const Mat& img, const Point* pattern, uchar* desc)
{
    float angle = (float)kpt.angle*factorPI;
    // 角度的余弦值
    float a = (float)cos(angle);
    // 角度的正弦值
    float b = (float)sin(angle);
    const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
    // 图像的每行的字节数
    const int step = (int)img.step;
    // std::cout << "step " << step << std::endl;// 752

    // pattern 旋转前坐标为(x,y), 旋转后坐标(x',y')
    // [x'] = cos(θ) -sin(θ) [x]
    // [y'] = sin(θ) cos(θ)  [y]
    // x'= cos(θ)x - sin(θ)y
    // y'= sin(θ)x + cos(θ)y
    // y'* step + x'
    // 至此提取了FAST角点,还找出了角点的角度,这个角度可以用来指导描述子的提取,保证每次都在相同的方向上计算描述子,实现角度不变性 Oriented FAST
    #define GET_VALUE(idx) center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + cvRound(pattern[idx].x*a - pattern[idx].y*b)]

    // 16 以像素为圆心,选择半径为3的圆上的16个像素点
    // BEIFE的计算步骤
    // 首先以关键点为中心,选择一个31×31的图像块
    // 然后在这个图像块内按照一定的方法选择N对点,N一般取256(一定的方法会生成256个点对,就是前面转换的那个pattern)
    // "static int bit_pattern_31_[256 * 4] = {" 是一个1024维的数组,数组数据类型是int,是特征点keypoint为中心周围256对点的坐标,强制转换成 cv::Point 类型的 512个点.
    // 一个字节等于8位,所以val是16个字节是128位的二进制数
    for (int i = 0; i < 32; ++i, pattern += 16)// 32 * 16 = 512
    {
        // 0和1编码了关键点附近连个随机像素点的大小关系, 如果第一个点大于第二个点则取1, 如果第一个点小于第二个点则取0,
        int t0, t1, val;
        // 获取像素值
        t0 = GET_VALUE(0);
        t1 = GET_VALUE(1);
        val = t0 < t1;
        // std::cout << "t0=" << t0  << " t1=" << t1 << " val " << val << std::endl;
        // > 检查左操作数的值是否大于右操作数的值,如果是则条件为真val=1, 否则val=0
        //  t0=124 t1=128  -> 1
        //  val =  0000 0001 = 1
        //  0    0   0   0   0  0  0  0
        //  128  64  32  16  8  4  2  1

        t0 = GET_VALUE(2);// 157
        t1 = GET_VALUE(3);
        // > 大于运算符
        // < 小于运算符
        // << 二进制左移运算符,将一个运算对象的各二进制位全部左移若干位(左边的二进制位丢弃右边补0), A << 2 将得到 240, 即为 1111 0000
        // |= 位或运算后赋值
        val |= (t0 < t1) << 1;
        //std::cout << "t0=" << t0  << " t1=" << t1 << " val " << val << std::endl;
        // t0=157 t1=190 -> 1
        // val =  0000 0001
        //        0000 0010
        //        0000 0011 = val 3


        t0 = GET_VALUE(4);
        t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;
        //std::cout << "t0=" << t0  << " t1=" << t1 << " val " << val << std::endl;
        // t0=38 t1=39 -> 1
        // val =  0000 0011
        //        0000 0100
        //        0000 0111 = val 7

        t0 = GET_VALUE(6);
        t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;
        //std::cout << "t0=" << t0  << " t1=" << t1 << " val " << val << std::endl;
        // t0=190 t1=134 -> 0
        // val =  0000 0111
        //        0000 0001
        //        0000 0111 = val 7

        t0 = GET_VALUE(8);
        t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;
        //std::cout << "t0=" << t0  << " t1=" << t1 << " val " << val << std::endl;
        // t0=92 t1=135 -> 1
        // val =  0000 0111
        //        0001 0000
        //        0001 0111 = val 23


        t0 = GET_VALUE(10);
        t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;
        //std::cout << "t0=" << t0  << " t1=" << t1 << " val " << val << std::endl;
        // t0=92 t1=135 -> 1
        // val =  0000 0111
        //        0001 0000
        //        0001 0111 = val 23

        t0 = GET_VALUE(12);
        t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;
        //std::cout << "t0=" << t0  << " t1=" << t1 << " val " << val << std::endl;
        // t0=73 t1=48 -> 0
        // val =  0000 0111
        //        0001 0000
        //        0001 0111 = val 23


        t0 = GET_VALUE(14);
        t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;
        //std::cout << "t0=" << t0  << " t1=" << t1 << " val " << val << std::endl;
        // t0=26 t1=38 -> 1
        // val =  0001 0111
        //        1000 0000
        //        1001 0111 = val 151

        desc[i] = (uchar)val;
        // cout << "desc[" << i << "] = " << int(desc[i]) << std::endl;
        // desc[1] = 173
        // desc[2] = 186
        // desc[3] = 122
        // desc[4] = 159
        // desc[5] = 209
        // desc[6] = 150
        // desc[7] = 233
        // desc[8] = 223
        // desc[9] = 19
        // desc[10] = 245
        // desc[11] = 26
        // desc[12] = 46
        // desc[13] = 216
        // desc[14] = 110
        // desc[15] = 37
        // desc[16] = 233
        // desc[17] = 251
        // desc[18] = 58
        // desc[19] = 83
        // desc[20] = 126
        // desc[21] = 111
        // desc[22] = 138
        // desc[23] = 247
        // desc[24] = 187
        // desc[25] = 94
        // desc[26] = 23
        // desc[27] = 123
        // desc[28] = 81
        // desc[29] = 79
        // desc[30] = 169
        // desc[31] = 142
    }
    #undef GET_VALUE
}


static int bit_pattern_31_[256*4] =
{
    8,-3, 9,5/*mean (0), correlation (0)*/,
    4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
    -11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
    7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
    2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
    1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
    -2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
    -13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
    -13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
    10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
    -13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
    -11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
    7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
    -4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
    -13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
    -9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
    12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
    -3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
    -6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
    11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
    4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
    5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
    3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
    -8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
    -2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
    -13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
    -7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
    -4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
    -10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
    5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
    5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
    1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
    9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
    4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
    2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
    -4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
    -8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
    4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
    0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
    -13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
    -3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
    -6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
    8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
    0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
    7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
    -13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
    10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
    -6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
    10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
    -13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
    -13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
    3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
    5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
    -1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
    3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
    2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
    -13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
    -13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
    -13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
    -7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
    6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
    -9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
    -2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
    -12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
    3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
    -7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
    -3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
    2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
    -11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
    -1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
    5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
    -4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
    -9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
    -12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
    10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
    7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
    -7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
    -4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
    7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
    -7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
    -13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
    -3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
    7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
    -13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
    1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
    2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
    -4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
    -1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
    7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
    1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
    9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
    -1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
    -13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
    7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
    12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
    6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
    5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
    2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
    3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
    2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
    9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
    -8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
    -11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
    1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
    6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
    2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
    6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
    3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
    7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
    -11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
    -10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
    -5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
    -10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
    8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
    4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
    -10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
    4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
    -2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
    -5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
    7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
    -9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
    -5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
    8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
    -9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
    1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
    7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
    -2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
    11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
    -12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
    3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
    5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
    0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
    -9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
    0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
    -1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
    5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
    3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
    -13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
    -5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
    -4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
    6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
    -7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
    -13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
    1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
    4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
    -2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
    2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
    -2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
    4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
    -6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
    -3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
    7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
    4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
    -13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
    7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
    7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
    -7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
    -8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
    -13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
    2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
    10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
    -6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
    8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
    2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
    -11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
    -12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
    -11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
    5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
    -2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
    -1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
    -13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
    -10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
    -3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
    2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
    -9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
    -4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
    -4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
    -6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
    6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
    -13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
    11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
    7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
    -1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
    -4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
    -7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
    -13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
    -7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
    -8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
    -5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
    -13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
    1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
    1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
    9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
    5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
    -1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
    -9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
    -1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
    -13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
    8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
    2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
    7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
    -10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
    -10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
    4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
    3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
    -4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
    5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
    4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
    -9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
    0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
    -12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
    3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
    -10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
    8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
    -8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
    2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
    10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
    6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
    -7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
    -3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
    -1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
    -3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
    -8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
    4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
    2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
    6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
    3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
    11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
    -3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
    4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
    2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
    -10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
    -13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
    -13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
    6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
    0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
    -13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
    -9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
    -13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
    5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
    2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
    -1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
    9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
    11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
    3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
    -1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
    3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
    -13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
    5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
    8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
    7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
    -10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
    7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
    9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
    7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
    -1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
};

/**
 * 1. 加载 Euroc 数据集 -->添加微信 slamshizhanjiaocheng 邀请加入 SLAM实战教程 微信交流群
 * 2. 加载 Euroc 数据集参数 对图像进行去畸变 -->添加微信 slamshizhanjiaocheng 邀请加入 SLAM实战教程 微信交流群
 * 3. 创建 ORB_SLAM2::System 对象, 初始化 System Tracking Frame ORBextractor -->添加微信 slamshizhanjiaocheng 邀请加入 SLAM实战教程 微信交流群
 * 4. 执行 ORBextractor 构造函数 a.金字塔每层图像缩放系数 b.金字塔每层图像提取特征点数量 c.计算umax -->添加微信 slamshizhanjiaocheng 邀请加入 SLAM实战教程 微信交流群
 */
ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels, int _iniThFAST, int _minThFAST):
    nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels), iniThFAST(_iniThFAST), minThFAST(_minThFAST)
{
    mvScaleFactor.resize(nlevels);
    mvLevelSigma2.resize(nlevels);
    // 存储缩放系数的vector
    mvScaleFactor[0]=1.0f;
    // 存储缩放系数平方的vector
    mvLevelSigma2[0]=1.0f;
    /**
     * mvScaleFactor[0]: 1.0
     * mvScaleFactor[1]: 1.2
     * mvScaleFactor[2]: 1.44
     * mvScaleFactor[3]: 1.728
    * mvScaleFactor[4]: 2.0736
    * mvScaleFactor[5]: 2.48832
    * mvScaleFactor[6]: 2.98598
    * mvScaleFactor[7]: 3.58318
    */
    for(int i=1; i<nlevels; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }
    /**
     * mvLevelSigma2[0]: 1.0
     * mvLevelSigma2[1]: 1.44
     * mvLevelSigma2[2]: 2.0736
     * mvLevelSigma2[3]: 2.98598
     * mvLevelSigma2[4]: 4.29982
     * mvLevelSigma2[5]: 6.19174
     * mvLevelSigma2[6]: 8.9161
     * mvLevelSigma2[7]: 12.8392
     */
    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);
    for(int i=0; i<nlevels; i++)
    {
        // 存储缩放系数倒数的vector
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        // 存储缩放系数平方倒数的vector
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }

    mvImagePyramid.resize(nlevels);
    mnFeaturesPerLevel.resize(nlevels);
    // 图像缩放系数的倒数 double scaleFactor;
    float factor = 1.0f / scaleFactor;
    // 第一层：nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels))
    // 第二层：nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels))*factor
    // 第三层：nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels))*factor*factor
    //  .........
    // 第nlevels层：nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels))*q^nlevels
    // 那么前nlevels层的和为总的特征点数量nfeatures等比数列的前n相和
    // 参考大佬讲解
    // https://zhuanlan.zhihu.com/p/61738607
    // https://blog.csdn.net/qq_30356613/article/details/75231440
    /**
     * 层需要提取的特征点数目
     * 第1层 261
     * 第2层 217
     * 第3层 181
     * 第4层 151
     * 第5层 126
     * 第6层 105
     * 第7层 87
     * 第8层 0
     */
    float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));
    int sumFeatures = 0;
    for( int level = 0; level < nlevels-1; level++ )
    {
        // std::vector<int> mnFeaturesPerLevel; 这个vector存储图像金字塔每层需要提取的特征点个数
        // 第1层提取到的特征点数目取整然后存储到vector->mnFeaturesPerLevel
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
        // 特征点数目累加,这个变量的用途是为了计算第8层图像需要提取的特征点数目.
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    // 从给定的两个值中查找最大值,上面只是计算前七层,这句代码是计算第8层需要提取特征点数目.
    mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);

    // https://zhuanlan.zhihu.com/p/61738607 4. 计算Rotation-Aware BRIEF rBRIEF
    // "static int bit_pattern_31_[256 * 4] = {" 是一个1024维的数组,数组数据类型是int,是特征点keypoint为中心周围256对点的坐标
    // 强制转换成 cv::Point 类型的 512个点.
    const int npoints = 512;
    const Point* pattern0 = (const Point*)bit_pattern_31_;
    /**
     * 8, -3, 9,  5
     * 4,  2, 7, -12
     * [8, -3]
     * [9, 5]
     * [4, 2]
     * [7, -12]
     */
    // copy [pattern0, pattern0+npoints] 到 std::vector<cv::Point> pattern
    // pattern0是数据的首地址, 数据的尾地址是首地址加上偏移量.
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));


    // This is for orientation pre-compute the end of a row in a circular patch
    // 以特征点keypoint像素坐标点为中心的patch圆内计算关键点keypoint的方向,直径为PATCH_SIZE=31,半径为HALF_PATCH_SIZE=15
    // 参考大佬讲解 https://blog.csdn.net/liu502617169/article/details/89423494
    // 参考大佬讲解 https://zhuanlan.zhihu.com/p/61738607 3. Oriented FAST,旋转角度计算
    // 参考大佬链接：http://the7.net/news/show-44083.html#!
    // cvRound() 即四舍五入返回跟参数最接近的整数值
    // cvFloor() 即向下取整返回不大于参数的最大整数值
    // cvCeil()  即向上取整返回不小于参数的最小整数值

    // "sqrt(" 返回一个数字的平方根 根号2对应45°圆心角
    // 1. 最大行数vmax初始化为R*sin45°(根号2/2)+1, +1是为了vmax和vmin边界值在遍历的过程中产生交叉,因为做了取整操作防止漏掉.
    // 2. 最大行数vmax单纯是0-45°这个过程的最大行数,而不是这个圆的最大行数.
    // 3. vmin为最小行数向上取整,是45-90°过程中的最小行数
    int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);// 11
    int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
    // 半径R的平方 圆的半径取为15, 整个patch圆是31×31的
    const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
    // std::vector<int> umax;, ".resize(" 调整容器的大小
    umax.resize(HALF_PATCH_SIZE + 1);
    // umax 16个数值依次是：15 15 15 15 14 14 14 13 13 12 11 10 9 8 6 3
    // 图像坐标系,四分之一圆
    /**
     * Y
     * | * * *
     * | * * * * * *
     * | * * * * * * * *
     * | * * * * * * * * *
     * | * * * * * * * * * *
     * | * * * * * * * * * * *
     * | * * * * * * * * * * * *
     * | * * * * * * * * * * * * *
     * | * * * * * * * * * * * * *
     * | * * * * * * * * * * * * * *
     * | * * * * * * * * * * * * * *
     * | * * * * * * * * * * * * * *
     * | * * * * * * * * * * * * * * *
     * | * * * * * * * * * * * * * * *
     * | * * * * * * * * * * * * * * *
     * | * * * * * * * * * * * * * * *
     * |------------------------------------>X
     * 
     */
    for (v = 0; v <= vmax; ++v){
        // 利用三角函数定理计算 umax[v]平方 = HALF_PATCH_SIZE平方- v * v
        umax[v] = cvRound(sqrt(hp2 - v * v));
    }
    // Make sure we are symmetric
    // 计算v = vmin至HALF_PATCH_SIZE时的umax[v],v从11到15之间的值依次是: 10, 9, 8, 6, 3
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }
}

// 每个特征点所在图像区块的每行的边界 u_max 组成的vector
static void computeOrientation(const Mat& image, vector<KeyPoint>& keypoints, const vector<int>& umax)
{
    for (vector<KeyPoint>::iterator keypoint = keypoints.begin(), keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
    {
        keypoint->angle = IC_Angle(image, keypoint->pt, umax);
    }
}

// 这部分代码主要是将节点划分为四部分(四叉树), 并且计算每部分的左上右下点的坐标, 然后根据坐标将特征点存储到对应的节点中
void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4){
    const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);// 180
    const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);// 224

    // Define boundaries of childs
    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x+halfX,UL.y);
    n1.BL = cv::Point2i(UL.x,UL.y+halfY);
    n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
    n1.vKeys.reserve(vKeys.size());
    // std::cout << "n1: UL " << n1.UL << " UR " << n1.UR << " BL " << n1.BR << " BR " << n1.BL << std::endl;


    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x,UL.y+halfY);
    n2.vKeys.reserve(vKeys.size());
    // std::cout << "n2: UL " << n2.UL << " UR " << n2.UR << " BL " << n2.BR << " BR " << n2.BL << std::endl;


    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(n1.BR.x,BL.y);
    n3.vKeys.reserve(vKeys.size());
    // std::cout << "n3: UL " << n3.UL << " UR " << n3.UR << " BL " << n3.BR << " BR " << n3.BL << std::endl;

    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());
    // std::cout << "n4: UL " << n4.UL << " UR " << n4.UR << " BL " << n4.BR << " BR " << n4.BL << std::endl;

    // Associate points to childs
    // 这部分是根据特征点的坐标, 将特征点存储到对应的子节点里面的vKeys中
    // 循环次数是父亲节点存储的特征点数目
    for(size_t i=0; i<vKeys.size(); i++){
        // 定义特征点的引用
        const cv::KeyPoint &kp = vKeys[i];
        if(kp.pt.x<n1.UR.x)
        {
            if(kp.pt.y<n1.BR.y)
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        }
        else if(kp.pt.y<n1.BR.y)
            n2.vKeys.push_back(kp);
        else
            n4.vKeys.push_back(kp);
    }

    // 这里也是预处理, 提前做了一个判断, 并且更新节点中bNoMore变量的值
    if(n1.vKeys.size()==1)
        n1.bNoMore = true;
    if(n2.vKeys.size()==1)
        n2.bNoMore = true;
    if(n3.vKeys.size()==1)
        n3.bNoMore = true;
    if(n4.vKeys.size()==1)
        n4.bNoMore = true;
}

vector<cv::KeyPoint> ORBextractor::DistributeOctTree(const vector<cv::KeyPoint>& vToDistributeKeys, const int &minX, const int &maxX, const int &minY, const int &maxY, const int &N, const int &level){
    // Compute how many initial nodes 类型转换
    // const float width = (maxBorderX-minBorderX);// 原始图像宽752 可提取特征图像宽 752-32=720
    // const float height = (maxBorderY-minBorderY);// 原始图像高480 可提取特征图像宽 480-32=448
    // 初始节点的数目
    const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));// 2
    // std::cout << "nIni " << nIni << std::endl;// 2
    const float hX = static_cast<float>(maxX-minX)/nIni;// 360
    // std::cout << "hX " << hX << std::endl;// 360

    //class ExtractorNode
    //{
    //public:
    //    ExtractorNode():bNoMore(false){}
    //    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);
    //    std::vector<cv::KeyPoint> vKeys;
    //    cv::Point2i UL, UR, BL, BR;
    //    std::list<ExtractorNode>::iterator lit;
    //    bool bNoMore;
    //};
    // 定义 list ExtractorNode 用于存储节点(KeyPoint)
    list<ExtractorNode> lNodes;
    // 定义 vector ExtractorNode* 用于索引节点
    vector<ExtractorNode*> vpIniNodes;
    vpIniNodes.resize(nIni);

    // 这个循环存储每个节点的 起始点坐标和终止点坐标
    for(int i=0; i<nIni; i++){
        // 创建节点
        ExtractorNode ni;
        ni.UL = cv::Point2i(hX*static_cast<float>(i),0);
        ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);
        ni.BL = cv::Point2i(ni.UL.x,maxY-minY);
        ni.BR = cv::Point2i(ni.UR.x,maxY-minY);
        // std::cout << i << " : UL " << ni.UL << " UR " << ni.UR << " BL " << ni.BR << " BR " << ni.BL << std::endl;
        // vToDistributeKeys 是前面一帧图像提取到的所有特征点
        ni.vKeys.reserve(vToDistributeKeys.size());
        lNodes.push_back(ni);
        // list::back() 输出容器中最后一个元素
        vpIniNodes[i] = &lNodes.back();
    }

    // Associate points to childs
    // 这个循环将特征点下存储到上面的节点里面
    for(size_t i=0;i<vToDistributeKeys.size();i++){
        const cv::KeyPoint &kp = vToDistributeKeys[i];
        // std::cout << "kp.pt.x/hX " << kp.pt.x/hX << std::endl;
        // 隐含取整操作 kp.pt.x/hX相比 只有俩个结果, 小于1, 大于1, 小于1都放在第一个节点里面, 大于1都放在第二个节点里面
        vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);
    }

    // 通过迭代器（iterator）可以访问顺序容器和关联容器中的元素
    list<ExtractorNode>::iterator lit = lNodes.begin();
    // 这个循环的功能是节点预处理,如果节点中的特征点数目是零,就在节点列表中将这个节点删除,如果这个节点中特征点数目只有一个那就设置一个标记后面不再分裂
    while(lit!=lNodes.end()){
        // 节点中特征点的数目是否等于1, 等于1 bNoMore 就赋值为 true
        if(lit->vKeys.size()==1){
            lit->bNoMore=true;
            lit++;
        }
        // 节点中特征点的数目是否为空, 为空就删除这个节点
        else if(lit->vKeys.empty())
            lit = lNodes.erase(lit);
        // 迭代器自增
        else
            lit++;
    }


    // 创建一个vector里面存储的数据是一个数据对, int是节点中特征点的数目 ExtractorNode*是节点
    vector<pair<int,ExtractorNode*> > vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size()*4);

    bool bFinish = false;
    int iteration = 0;
    // 这个循环实现了将节点分裂的功能
    while(!bFinish){
        iteration++;
        int prevSize = lNodes.size();
        lit = lNodes.begin();
        int nToExpand = 0;
        vSizeAndPointerToNode.clear();
        // 分裂的结束条件是迭代器指向节点列表末尾 (list::end()函数将迭代器返回到列表最后一个元素之后的元素)
        while(lit!=lNodes.end())
        {
            if(lit->bNoMore)
            {
                // If node only contains one point do not subdivide and continue
                lit++;
                continue;
            }
            else
            {
                // If more than one point, subdivide
                ExtractorNode n1,n2,n3,n4;
                // 四叉树 每次新增四个节点
                lit->DivideNode(n1,n2,n3,n4);

                // Add childs if they contain points
                if(n1.vKeys.size()>0){
                    //  list push_front()函数在列表的开头添加了一个新元素
                    lNodes.push_front(n1);                    
                    if(n1.vKeys.size()>1)
                    {
                        nToExpand++;
                        // list<ExtractorNode> lNodes;  通过front()可以获得list容器中的头部元素, 通过back()可以获得list容器的最后一个元素
                        vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                        // list 迭代器 begin 返回指向起始的迭代器
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n2.vKeys.size()>0)
                {
                    lNodes.push_front(n2);
                    if(n2.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n3.vKeys.size()>0)
                {
                    lNodes.push_front(n3);
                    if(n3.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n4.vKeys.size()>0)
                {
                    lNodes.push_front(n4);
                    if(n4.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                // erase() 该成员函数删除 list 容器中指定位置处的元素
                // 理解为迭代器重新指向了删除节点后的新的list的第一个节点
                lit = lNodes.erase(lit);
                continue;
            }
        }       


        // Finish if there are more nodes than required features or all nodes contain just one point
        // 随时准备提前结束分裂：当前的节点数已经超过了要求的特征点数, 当前所有的节点中都只包含一个特征点
        if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize){
            bFinish = true;
        }
        // 随时准备提前结束分裂 这里借助vector<pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode 在可分裂的前提下, 对可以分裂的节点进行优先分裂,在这个过程中也会随时检测准备退出结束分裂
        else if(((int)lNodes.size()+nToExpand*3)>N)
        {

            while(!bFinish)
            {

                prevSize = lNodes.size();

                vector<pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                // 对需要划分的节点进行排序,对pair对的第一个元素进行排序,默认是从小到大排序, 优先分裂特征点多的节点,使得特征点密集的区域保留更少的特征点
                sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end());
                for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)
                {
                    ExtractorNode n1,n2,n3,n4;
                    // 取出 vPrevSizeAndPointerToNode 中的节点
                    vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                    // Add childs if they contain points
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(n1);
                        if(n1.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                    if((int)lNodes.size()>=N)
                        break;
                }

                if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                    bFinish = true;

            }
        }
    }


    // Retain the best point in each node
    // 节点分裂好了之后, 节点的数目就是要提取特征点的数目, 所以一个节点只保留一个特征点, 特征点的筛选条件是特征点的响应强度
    vector<cv::KeyPoint> vResultKeys;
    vResultKeys.reserve(nfeatures);
    for(list<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
    {
        // 节点特征点的索引
        vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
        // 节点第一个特征点的指针
        cv::KeyPoint* pKP = &vNodeKeys[0];
        // 获取特征点的响应值
        float maxResponse = pKP->response;
        // 遍历特征点寻找响应值大的特征点
        for(size_t k=1;k<vNodeKeys.size();k++)
        {
            // 如果发现响应值大的就替换指针
            if(vNodeKeys[k].response>maxResponse)
            {
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }
        // 将满足条件的特征点存储到vResultKeys
        vResultKeys.push_back(*pKP);
    }

    return vResultKeys;
}


void ORBextractor::ComputeKeyPointsOctTree(vector<vector<KeyPoint> >& allKeypoints)
{
    cv::Mat image;
    allKeypoints.resize(nlevels);
    const float W = 30;
    // for 循环遍历金字塔每层图像, 计算特征点（分块）, 特征点均匀化（四叉树）
    for (int level = 0; level < nlevels; ++level)
    {
//        rectangle(mvImagePyramid[level], Point(19, 19), Point(mvImagePyramid[level].cols-EDGE_THRESHOLD, mvImagePyramid[level].rows-EDGE_THRESHOLD), Scalar(0), 1);
//        cv::imwrite( to_string(level) + "_rectangle_19.png", mvImagePyramid[level]);
        // 3 fast 角点像素半径为3
        const int minBorderX = EDGE_THRESHOLD-3;// 16=19-3
        const int minBorderY = minBorderX;// 16=19-3
        const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD+3;// 736=752-19+3
        const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD+3;// 464=480-19+3
        rectangle(mvImagePyramid[level], Point(minBorderX, minBorderY), Point(maxBorderX, maxBorderY), Scalar(0), 1);
//        cv::imwrite(to_string(level) + "_rectangle_16.png", mvImagePyramid[level]);

        const float width = (maxBorderX-minBorderX);// 原始图像宽752 可提取特征图像宽 752-32=720
        const float height = (maxBorderY-minBorderY);// 原始图像高480 可提取特征图像宽 480-32=448

        // 宽720 分24个图像块
        const int nCols = width/W;
        // 高448 分14个图像块剩余余28个像素 每块高30个像素
        const int nRows = height/W;

        // 每块图像宽720/24=30个像素
        const int wCell = ceil(width/nCols);
        // 每块图像高448/14=32个像素
        const int hCell = ceil(height/nRows);

        // 存储提取到的所有特征点
        vector<cv::KeyPoint> vToDistributeKeys;
        vToDistributeKeys.reserve(nfeatures*10);
        // 双层for循环遍历某层图像的所有小图像块(宽30*高32pixel)
        // 外层for循环遍历行, 内层for循环遍历列
        for(int i=0; i<nRows; i++){
            // 计算每个图像块左上角的iniY坐标
            const float iniY =minBorderY + i*hCell;// iniY=16 = minBorderY=6 + i=0×hCell=32
            // 可以提取特征点的图像范围是从(19, 19)开始的, 然后为了考虑边界像素提取Fast特征点的时候是以特征点为中心半径为3个像素的圆计算的
            // 计算每个图像块左下角的maxY坐标
            float maxY = iniY + 3 + hCell + 3;// maxY=54 = iniY=16 + 3 + hCell=32 + 3
            // 计算到图像块边缘像素坐标和图像边界之间已经不足3个像素时, 就没有计算的必要了, 因为提取提取Fast特征点以特征点为中心半径为3个像素的圆计算的
            if(iniY>=maxBorderY-3)
                continue;
            // 计算到图像块边缘像素坐标大于图像边界时直接赋值为可以提取特征点的图像的边界
            if(maxY>maxBorderY)
                maxY = maxBorderY;

            // 外层for循环遍历行, 内层for循环遍历列
            for(int j=0; j<nCols; j++){
                // 计算每个图像块左上角的iniX坐标
                const float iniX = minBorderX+j*wCell;// iniX=16 = minBorderX=16 + j=0×wCell=30
                // 计算每个图像块右上角的maxX坐标
                float maxX = iniX+3+wCell+3;//maxX=52 = iniX=16 + j=0×wCell=30
                // 计算到图像块边缘像素坐标和图像边界之间已经不足6个像素时,停止当前图像块的计算,继续下一个图像块
                if(iniX>=maxBorderX-6)
                    continue;
                // 计算到图像块边缘像素坐标大于图像边界时直接赋值为可以提取特征点的图像的边界
                if(maxX>maxBorderX)
                    maxX = maxBorderX;

                vector<cv::KeyPoint> vKeysCell;
                // std::cout << "iniX " << iniX << " iniY " << iniY << " maxX " << maxX << " maxY " << maxY << std::endl;
                // 绘制提取特征的小图像块
                rectangle(mvImagePyramid[level], Point(iniX, iniY), Point(maxX, maxY), Scalar(0), 1);
//                cv::imwrite(to_string(level) + to_string(i) + to_string(j) +".png", mvImagePyramid[level]);

                FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX), vKeysCell,iniThFAST,true);
//                cv::imwrite(to_string(level) + to_string(i) + to_string(j) + "_fast.png", mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX));

                if(vKeysCell.empty())
                {
                    FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),vKeysCell,minThFAST,true);
                }

                if(!vKeysCell.empty()){
                    for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++){
                        // 像素相对与图像块的坐标转换到相对与检测特征点全图像的的像素坐标
//                        std::cout << "(*vit).pt.x " << (*vit).pt.x << " (*vit).pt.y " << (*vit).pt.y << std::endl;
                        (*vit).pt.x+=j*wCell;
                        (*vit).pt.y+=i*hCell;
//                        std::cout << "(*vit).pt.x " << (*vit).pt.x << " (*vit).pt.y " << (*vit).pt.y << std::endl;
                        vToDistributeKeys.push_back(*vit);
                        image = mvImagePyramid[level].rowRange(minBorderY,maxBorderY).colRange(minBorderX,maxBorderX);
                        // 绘制提取到的特征点
                        cv::circle(image, (*vit).pt,1,cv::Scalar(0),-1);
                    }
                    // cv::imwrite( to_string(level) + "_keypoint.png", image);
                }

            }
        }


        // 当前金字塔图层 均匀化处理后的特征点vector
        vector<KeyPoint> & keypoints = allKeypoints[level];
        keypoints.reserve(nfeatures);
        keypoints = DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX, minBorderY, maxBorderY,mnFeaturesPerLevel[level], level);

        // PATCH_SIZE 也会随着图层进行缩放 为下一次计算做准备工作
        const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

        // Add border to coordinates and scale information
        const int nkps = keypoints.size();
        for(int i=0; i<nkps ; i++)
        {
            // 将特征点的坐标转换到 包含提取特征点的图像的16个扩充像素的大图中
            keypoints[i].pt.x+=minBorderX;
            keypoints[i].pt.y+=minBorderY;
            // 同时存储特征点是提取自那一个图层
            keypoints[i].octave=level;
            // 同时存储要计算特征点旋转不变性的圆半径
            keypoints[i].size = scaledPatchSize;
        }
    }


    // compute orientations
    // 当提取完所有金字塔图层的特征点之后开始计算旋转不变性
    for (int level = 0; level < nlevels; ++level)
        computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
}


static void computeDescriptors(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors, const vector<Point>& pattern)
{
    // 初始化
    descriptors = Mat::zeros((int)keypoints.size(), 32, CV_8UC1);
    for (size_t i = 0; i < keypoints.size(); i++)
        computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));
}


void ORBextractor::operator()( InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints, OutputArray _descriptors)
{ 
    if(_image.empty())
        return;

    Mat image = _image.getMat();
    assert(image.type() == CV_8UC1);

    // Pre-compute the scale pyramid
    ComputePyramid(image);
//    for (int level = 0; level < nlevels; ++level)// 遍历图像金字塔 0 1 2 3 4 5 6 7
//    {
//        cv::imwrite("show_mvImagePyramid_" + to_string(level) + ".png", mvImagePyramid[level]);
//    }
    vector < vector<KeyPoint> > allKeypoints;
    ComputeKeyPointsOctTree(allKeypoints);
    Mat descriptors;
    int nkeypoints = 0;
    for (int level = 0; level < nlevels; ++level)
        nkeypoints += (int)allKeypoints[level].size();
    if( nkeypoints == 0 )
        // 递减引用计数并释放该矩阵
        _descriptors.release();
    else
    {
        // 分配创建一个矩阵的矩阵体 范围CV_8U 0-255 默认值205
        _descriptors.create(nkeypoints, 32, CV_8U);
        // getMat函数将OutputArray数据类型转换为cv::Mat类型
        descriptors = _descriptors.getMat();
    }
    //  vector有两个参数,一个是size表示当前vector容器内存储的元素个数;一个是capacity表示当前vector在内存中申请的这片区域所能容纳的元素个数
    //  通常capacity会比size大,如果往vector中push_back数据这样就不用重新申请内存和拷贝元素到新内存区域了便于节省时间
    //  所以vector.clear()的真正作用是把size设置成0,capacity不变,并不会把所有元素清零
    _keypoints.clear();
    // reserve的作用是更改vector的容量capacity,使vector至少可以容纳n个元素
    // 如果n大于vector当前的容量,reserve会对vector进行扩容,其他情况下都不会重新分配vector的存储空间
    _keypoints.reserve(nkeypoints);

    int offset = 0;
    // 循环 每层图像金字塔 计算特征点的描述子
    for (int level = 0; level < nlevels; ++level)
    {
        vector<KeyPoint>& keypoints = allKeypoints[level];
        int nkeypointsLevel = (int)keypoints.size();

        if(nkeypointsLevel==0)
            continue;

        // preprocess the resized image
        // mat3.copyTo(mat1); // mat1未被重新分配内存, 通过mat1可以改变mat2的内容
        // mat1 = mat3.clone(); // mat1被重新分配内存, 通过mat1不能改变mat2的内容
        Mat workingMat = mvImagePyramid[level].clone();
        // cv::imwrite("workingMat.png", workingMat);
        // 高斯滤波是一种线性平滑滤波, 本质上是一种数据平滑技术（data smoothing）对于除去高斯噪声有很好的效果
        // 所谓"模糊",可以理解成每一个像素都取周边像素的平均值
        // 通常图像处理软件会提供"模糊"（blur）滤镜,使图片产生模糊的效果
        // "模糊"的算法有很多种,其中有一种叫做"高斯模糊"(Gaussian Blur)它将正态分布(又名"高斯分布")用于图像处理
        // 第一个参数: InputArray类型的src输入图像Mat类的对象该函数对通道是独立处理的且可以处理任意通道数的图像但是待处理的图像深度应该是CV_8U,CV_16U,CV_16S,CV_32F,CV_64F
        // 第二个参数: OutputArray类型的dst目标图像,需要和输入图像有相同的尺寸和类型
        // 第三个参数: Size类型的size,内核的大小,一般用Size(w,h)的写法表示,w和h可以不同,但是必须是正数和奇数,例如Size(3,3)，Size(5,5)。
        // 第四个参数: double类型的sigmaX,表示高斯核函数在X方向上的标准偏差
        // 第五个参数: double类型的sigmaY,表示高斯核函数在Y方向上的标准偏差
        // 如果sigmaY是0,就将它设为sigmaX,如果sigmaX和sigmaY都是0,那么就由ksize.width和ksize.height计算出来,为了结果的正确性最好是将Size,sigmaX,sigmaY全部指定到
        // 第六个参数: int类型的borderType,用于推断图像外部像素的某种边界模式,默认值 BORDER_DEFAULT
        // 边界点的处理: 如果一个点处于边界,周边没有足够的点怎么办,一个变通方法,就是把已有的点拷贝复制到边界的另一侧的对应位置进行扩充
        GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);
        // cv::imwrite("GaussianBlur_workingMat.png", workingMat);

        // Compute the descriptors
        // 遍历是一层一层进行的,但是描述子矩阵是存储整个图像金字塔中特征点的描述子,所以在这里设置了Offset变量来保存"寻址"时的偏移量,offset辅助进行在总描述子mat中的定位
        // nkeypointsLevel为金字塔图像中一层的特征点.
        // Mat的rowRange和colRange函数可以获取某些范围内行或列的指针
        // 理解为将描述子矩阵中的一部分 单独拿出来 用来存储当前层特征点的描述子
        Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
        computeDescriptors(workingMat, keypoints, desc, pattern);

        offset += nkeypointsLevel;

        // Scale keypoint coordinates
        if (level != 0)
        {
            float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
            for (vector<KeyPoint>::iterator keypoint = keypoints.begin(), keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
                // 对非第0层图像中的特征点的坐标恢复到第0层图像（原图像）的坐标系下 特征点乘缩放倍数
                keypoint->pt *= scale;
        }
        // And add the keypoints to the output 将keypoints中内容插入到_keypoints 的末尾
        _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
    }
}


/**
 * 5. 执行 System::TrackStereo Tracking::GrabImageStereo Frame::Frame Frame::ExtractORB ORBextractor::operator() 函数 计算图像金子塔 -->添加微信 slamshizhanjiaocheng 邀请加入 SLAM实战教程 微信交流群
 */
void ORBextractor::ComputePyramid(cv::Mat image)
{
    for (int level = 0; level < nlevels; ++level)// 遍历图像金字塔 0 1 2 3 4 5 6 7
    {
        float scale = mvInvScaleFactor[level];// 图像缩放系数 1 0.83
        Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));// 缩放后图像的宽高
        Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);// 对缩放后图像的加上边框后图像宽高
        Mat temp(wholeSize, image.type()), masktemp;// 创建两个 CV::Mat
        mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));// 给图像金字塔赋值,注意这里只是图像的宽高和像素的类型，还没有为每个像素赋值
        // Compute the resized image
        if( level != 0 )
        {
            resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);
            copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, BORDER_REFLECT_101+BORDER_ISOLATED);
            // cv::imwrite("mvImagePyramid_" + to_string(level) + ".png", mvImagePyramid[level]);
            // cv::imwrite("temp_" + to_string(level) + ".png", temp);
        }
        else
        {
            // 原始图像不需要缩放, 直接进行边界19个像素的对称扩充理解为加边框, 这里生成了第一张金字塔图像
            copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,BORDER_REFLECT_101);
            // cv::imwrite("image.png", image);
            // cv::imwrite("temp.png", temp);
            // cv::imwrite("mvImagePyramid_" + to_string(level) + ".png", mvImagePyramid[level]);
        }
    }

}

} //namespace ORB_SLAM