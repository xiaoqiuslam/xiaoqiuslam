#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv){
    if(argc != 4){
        cerr << endl << "Usage: ./orb_slam2_extract_feature path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    cv::FileStorage fSettings(argv[1], cv::FileStorage::READ);
    if(!fSettings.isOpened()){
        cerr << "Failed to open settings file at: " << argv[1] << endl;
        exit(-1);
    }

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;

    int nRGB = fSettings["Camera.RGB"];
    bool mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    float mbf;
    mbf = fSettings["Camera.bf"];
    float mThDepth;
    mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
    cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;

    float mDepthMapFactor;
    mDepthMapFactor = fSettings["DepthMapFactor"];
    if(fabs(mDepthMapFactor)<1e-5)
        mDepthMapFactor=1;
    else
        mDepthMapFactor = 1.0f/mDepthMapFactor;

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[3]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    if(vstrImageFilenamesRGB.empty()){
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size()){
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    int nImages = vstrImageFilenamesRGB.size();
    cout << "Images in the sequence: " << nImages << endl << endl;

    cv::Mat imRGB, imD;
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    for(int ni=0; ni<nImages; ni++){
        imRGB = cv::imread(string(argv[2])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        // cout << "Images RGB in the sequence: " << string(argv[2])+"/"+vstrImageFilenamesRGB[ni] << endl << endl;
        imD = cv::imread(string(argv[2])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        // cout << "Images imD in the sequence: " << string(argv[2])+"/"+vstrImageFilenamesD[ni] << endl << endl;
        double tframe = vTimestamps[ni];

        if(imRGB.empty()){
            cerr << endl << "Failed to load image at: " << string(argv[2]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        cv::Mat mImGray;
        mImGray = imRGB;
        cv::Mat imDepth;
        imDepth = imD;

        if(mImGray.channels()==3){
            if(mbRGB)
                cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            else
                cvtColor(mImGray,mImGray,CV_BGR2GRAY);
        }
        else if(mImGray.channels()==4){
            if(mbRGB)
                cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            else
                cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
        }

        if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
            imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);
    }

    return 0;
}


        // ORBextractor* mpORBextractorLeft;
        // mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

        // mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
        // Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
        // :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
        // {
        //     // Frame ID
        //     mnId=nNextId++;

        //     // Scale Level Info
        //     mnScaleLevels = nLevels;
        //     mfScaleFactor = fScaleFactor;
        //     mfLogScaleFactor = log(mfScaleFactor);
        //     mvScaleFactors = mvScaleFactor;
        //     mvInvScaleFactors = mvInvScaleFactor;
        //     mvLevelSigma2 = mvLevelSigma2;
        //     mvInvLevelSigma2 = mvInvLevelSigma2;

        //     // ORB extraction
        //     // ExtractORB(0,imGray);

        //     // Compute the ORB features and descriptors on an image.
        //     // ORB are dispersed on the image using an octree.
        //     // Mask is ignored in the current implementation.
        //     void operator()(cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint> &keypoints, cv::OutputArray descriptors);

        //     (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);

        //     void ORBextractor::operator()( InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints, OutputArray _descriptors)
        //     {
        //         if(_image.empty())
        //             return;

        //         Mat image = _image.getMat();
        //         assert(image.type() == CV_8UC1 );

        //         // Pre-compute the scale pyramid
        //         ComputePyramid(image);

        //         vector < vector<KeyPoint> > allKeypoints;
        //         ComputeKeyPointsOctTree(allKeypoints);
        //         //ComputeKeyPointsOld(allKeypoints);

        //         Mat descriptors;

        //         int nkeypoints = 0;
        //         for (int level = 0; level < nlevels; ++level)
        //             nkeypoints += (int)allKeypoints[level].size();
        //         if( nkeypoints == 0 )
        //             _descriptors.release();
        //         else
        //         {
        //             _descriptors.create(nkeypoints, 32, CV_8U);
        //             descriptors = _descriptors.getMat();
        //         }

        //         _keypoints.clear();
        //         _keypoints.reserve(nkeypoints);

        //         int offset = 0;
        //         for (int level = 0; level < nlevels; ++level)
        //         {
        //             vector<KeyPoint>& keypoints = allKeypoints[level];
        //             int nkeypointsLevel = (int)keypoints.size();

        //             if(nkeypointsLevel==0)
        //                 continue;

        //             // preprocess the resized image
        //             Mat workingMat = mvImagePyramid[level].clone();
        //             GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

        //             // Compute the descriptors
        //             Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
        //             computeDescriptors(workingMat, keypoints, desc, pattern);

        //             offset += nkeypointsLevel;

        //             // Scale keypoint coordinates
        //             if (level != 0)
        //             {
        //                 float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
        //                 for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
        //                              keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
        //                     keypoint->pt *= scale;
        //             }
        //             // And add the keypoints to the output
        //             _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
        //         }
        //     }

        //     N = mvKeys.size();

        //     if(mvKeys.empty())
        //         return;

        //     UndistortKeyPoints();

        //     ComputeStereoFromRGBD(imDepth);

        //     mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
        //     mvbOutlier = vector<bool>(N,false);

        //     // This is done only for the first Frame (or after a change in the calibration)
        //     if(mbInitialComputations)
        //     {
        //         ComputeImageBounds(imGray);

        //         mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        //         mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        //         fx = K.at<float>(0,0);
        //         fy = K.at<float>(1,1);
        //         cx = K.at<float>(0,2);
        //         cy = K.at<float>(1,2);
        //         invfx = 1.0f/fx;
        //         invfy = 1.0f/fy;

        //         mbInitialComputations=false;
        //     }

        //     mb = mbf/fx;

        //     AssignFeaturesToGrid();
        // }




void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}


//
//    // Stop all threads
//    SLAM.Shutdown();
//
//    // Tracking time statistics
//    sort(vTimesTrack.begin(),vTimesTrack.end());
//    float totaltime = 0;
//    for(int ni=0; ni<nImages; ni++)
//    {
//        totaltime+=vTimesTrack[ni];
//    }
//    cout << "-------" << endl << endl;
//    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
//    cout << "mean tracking time: " << totaltime/nImages << endl;
//
//    // Save camera trajectory
//    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
//    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
//
//    return 0;
//}
//
//cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
//{
//
//    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);
//    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
//    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
//    return Tcw;
//}
//
//
//cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
//{
//    mImGray = imRGB;
//    cv::Mat imDepth = imD;
//
//    if(mImGray.channels()==3)
//    {
//        if(mbRGB)
//            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
//        else
//            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
//    }
//    else if(mImGray.channels()==4)
//    {
//        if(mbRGB)
//            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
//        else
//            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
//    }
//
//    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
//        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);
//
//    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
//
//    Track();
//
//    return mCurrentFrame.mTcw.clone();
//}
//
//
//Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
//    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
//     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
//{
//    // Frame ID
//    mnId=nNextId++;
//
//    // Scale Level Info
//    mnScaleLevels = mpORBextractorLeft->GetLevels();
//    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
//    mfLogScaleFactor = log(mfScaleFactor);
//    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
//    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
//    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
//    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
//
//    // ORB extraction
//    ExtractORB(0,imGray);
//    (*mpORBextractorLeft)(imGray,cv::Mat(),mvKeys,mDescriptors);
//
//    N = mvKeys.size();
//
//    if(mvKeys.empty())
//        return;
//
//    UndistortKeyPoints();
//
//    // Set no stereo information
//    mvuRight = vector<float>(N,-1);
//    mvDepth = vector<float>(N,-1);
//
//    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
//    mvbOutlier = vector<bool>(N,false);
//
//    // This is done only for the first Frame (or after a change in the calibration)
//    if(mbInitialComputations)
//    {
//        ComputeImageBounds(imGray);
//
//        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
//        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);
//
//        fx = K.at<float>(0,0);
//        fy = K.at<float>(1,1);
//        cx = K.at<float>(0,2);
//        cy = K.at<float>(1,2);
//        invfx = 1.0f/fx;
//        invfy = 1.0f/fy;
//
//        mbInitialComputations=false;
//    }
//
//    mb = mbf/fx;
//
//    AssignFeaturesToGrid();
//}
//
//void ExtractORB(int flag, const cv::Mat &im)
//{
//    if(flag==0)
//        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
//    else
//        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
//}
//
//ORBextractor(int nfeatures, float scaleFactor, int nlevels,
//                 int iniThFAST, int minThFAST);
//
//void operator()(cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint> &keypoints, cv::OutputArray descriptors);
