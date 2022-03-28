#include "Tracking.h"
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
//#include "Eigen/Dense"
#include <opencv/highgui.h>
#include <cmath>
#include <thread>

using namespace std;
using namespace cv;

namespace ORB_SLAM2 {

    bool Tracking::mbInitialComputations = true;
    float Tracking::mnMinX, Tracking::mnMinY, Tracking::mnMaxX, Tracking::mnMaxY;
    float Tracking::mfGridElementWidthInv, Tracking::mfGridElementHeightInv;
    const int Tracking::TH_LOW = 50;
    int tmp_number = 0;


    Tracking::Tracking(System *pSys, const string &strSettingPath){
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fSettings["Camera.fx"];
        K.at<float>(1, 1) = fSettings["Camera.fy"];
        K.at<float>(0, 2) = fSettings["Camera.cx"];
        K.at<float>(1, 2) = fSettings["Camera.cy"];
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0) {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);
        {
            cv::Mat tmp_K_r, tmp_DistCoef_r;
            fSettings["RIGHT.K"] >> tmp_K_r;
            fSettings["RIGHT.D"] >> tmp_DistCoef_r;
            cv::Mat K_r = cv::Mat::eye(3, 3, CV_32F);
            tmp_K_r.convertTo(K_r, CV_32F);

            cv::Mat DistCoef_r(4, 1, CV_32F);
            tmp_DistCoef_r.convertTo(DistCoef_r, CV_32F);

            K_r.copyTo(mK_r);
            DistCoef_r.copyTo(mDistCoef_r);

            cv::Mat tmp_RotationRL, tmp_tlinr;
            fSettings["CAMERA.R"] >> tmp_RotationRL;
            fSettings["CAMERA.T"] >> tmp_tlinr;
            cv::Mat RotationRL(3, 3, CV_32F);
            tmp_RotationRL.convertTo(RotationRL, CV_32F);
            cv::Mat tlinr(3, 1, CV_32F);
            tmp_tlinr.convertTo(tlinr, CV_32F);

            RotationRL.copyTo(mRrl);
            tlinr.copyTo(mtlinr);
        }

        float fps = fSettings["Camera.fps"];
        if (fps == 0) {
            fps = 30;
        }
        if (DistCoef.rows == 5) cout << "- k3: " << DistCoef.at<float>(4) << endl;
        // Eigen::Vector3d euler_angles_mRrl = Tracking::MattoEulerAngle(mRrl);
        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;
        if (mbRGB) {
            //cout << "[Tracking] color order: RGB (ignored if grayscale)" << endl;
        } else {
            //cout << "[Tracking] ncolor order: BGR (ignored if grayscale)" << endl;
        }
        int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];

        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        cout << "Left ORB extractor " << endl;
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        cout << "Right ORB extractor " << endl;
    }

    cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight) {
        mImGray = imRectLeft;
        imGrayRight = imRectRight;
        if (mImGray.channels() == 3) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
            }
        } else if (mImGray.channels() == 4) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
            }
        }

        Frame();
        return cv::Mat::eye(4, 4, CV_32F);
    }

    void Tracking::Frame() {
        // 存储缩放系数平方的vector
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        // 存储缩放系数平方倒数的vector
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
        thread threadLeft(&Tracking::ExtractORB, this, 0, mImGray);
        thread threadRight(&Tracking::ExtractORB, this, 1, imGrayRight);
        threadLeft.join();
        threadRight.join();
        N = mvKeys.size();
        NRight = mvKeysRight.size();
        std::cout << "[Tracking] left camera Keypoint N: " << N << " right camera Keypoints NRight: " << NRight << std::endl;
        if (mvKeys.empty()) {
            return;
        }
        UndistortKeyPoints();
        if (mbInitialComputations) {
            mnMinX = 0.0f;
            mnMaxX = mImGray.cols;
            mnMinY = 0.0f;
            mnMaxY = mImGray.rows;
            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
            //std::cout << "mfGridElementWidthInv " << mfGridElementWidthInv << std::endl;
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);
            //std::cout << "mfGridElementHeightInv " << mfGridElementHeightInv << std::endl;
            mbInitialComputations = false;
        }
        AssignFeaturesToGrid();
        AssignFeaturesToGridRight();
        mImGray.copyTo(imLeft_cp);
        imGrayRight.copyTo(imRight_cp);
    }

    void Tracking::UndistortKeyPoints() {
        if(mDistCoef.at<float>(0)==0.0)
        {
            mvKeysUn=mvKeys;
            return;
        }

        // Fill matrix with points
        cv::Mat mat(N,2,CV_32F);
        for(int i=0; i<N; i++)
        {
            mat.at<float>(i,0)=mvKeys[i].pt.x;
            mat.at<float>(i,1)=mvKeys[i].pt.y;
        }

        // Undistort points
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        // Fill undistorted keypoint vector
        mvKeysUn.resize(N);
        for(int i=0; i<N; i++)
        {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x=mat.at<float>(i,0);
            kp.pt.y=mat.at<float>(i,1);
            mvKeysUn[i]=kp;
        }
    }

    void Tracking::AssignFeaturesToGrid() {
        int nReserve = 0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) {
                mGrid[i][j].clear();
                mGrid[i][j].reserve(nReserve);
            }

        for (int i = 0; i < N; i++) {
            const cv::KeyPoint &kp = mvKeysUn[i];
            int nGridPosX, nGridPosY;
            if (PosInGrid(kp, nGridPosX, nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }
    }


    void Tracking::AssignFeaturesToGridRight() {
        int NR = mvKeysRight.size();
        // cout << "AssignFeaturesToGridRight NR = " << NR << endl;
        int nReserve = 0.5f * NR / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++) {
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) {
                mGridRight[i][j].clear();
                mGridRight[i][j].reserve(nReserve);
            }
        }
        for (int i = 0; i < NR; i++) {
            const cv::KeyPoint &kpright = mvKeysRight[i];
            int nGridPosX, nGridPosY;
            if (PosInGrid(kpright, nGridPosX, nGridPosY)) {// 当前特征点在那个格子里面
                mGridRight[nGridPosX][nGridPosY].push_back(i);// 把当前特征点放到对应格子的容器中
            }
        }
    }

    void Tracking::ExtractORB(int flag, const cv::Mat &im) {
        vector<int> vLapping = {100, 600};
        if (flag == 0)
            monoLeft = (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors, vLapping);
        else
            monoRight = (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight, vLapping);
    }

    vector<size_t> Tracking::GetFeaturesInAreaRight(const float &x, const float &y, const float &r, const int minLevel,
                                                    const int maxLevel) const {
        vector<size_t> vIndices;
        vIndices.reserve(N);
        // 已经将特征点分配到(40/3, 10)的各自里， 总共为48*48个，所以这里主要是计算起始和结束的格子数；
        const int nMinCellX = max(0, (int) floor((x - mnMinX - r) * mfGridElementWidthInv));
        if (nMinCellX >= FRAME_GRID_COLS) return vIndices;

        const int nMaxCellX = min((int) FRAME_GRID_COLS - 1, (int) ceil((x - mnMinX + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0) return vIndices;

        const int nMinCellY =
                max(0, (int) floor((y - mnMinY - r) * mfGridElementHeightInv));
        if (nMinCellY >= FRAME_GRID_ROWS) return vIndices;

        const int nMaxCellY =
                min((int) FRAME_GRID_ROWS - 1,
                    (int) ceil((y - mnMinY + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0) return vIndices;

        const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
                const vector<size_t> vCell = mGridRight[ix][iy];
                if (vCell.empty())  // 当前cell中是都有特征点
                    continue;

                for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
                    const cv::KeyPoint &kpRight = mvKeysRight[vCell[j]];
                    if (bCheckLevels) {
                        if (kpRight.octave < minLevel) continue;
                        if (maxLevel >= 0)
                            if (kpRight.octave > maxLevel) continue;
                    }

                    const float distx = kpRight.pt.x - x;
                    const float disty = kpRight.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r) {
                        vIndices.push_back(vCell[j]);
                    }
                }
            }
        }

        return vIndices;
    }


    bool Tracking::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) {
        posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
        posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);
        if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
            return false;
        return true;
    }

}  // namespace ORB_SLAM2
