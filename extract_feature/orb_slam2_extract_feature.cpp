#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include"ORBextractor.h"

#include <opencv2/opencv.hpp>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[3]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
    if(!fsSettings.isOpened()){
       cerr << "Failed to open settings file at: " << argv[1] << endl;
       exit(-1);
    }

    // Load camera parameters from settings file
    cv::FileStorage fSettings(argv[1], cv::FileStorage::READ);
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
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;

    int nRGB = fSettings["Camera.RGB"];
    bool mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

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

    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    // Main loop
    cv::Mat imRGB, imD;

    ORBextractor* mpORBextractorLeft;
    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    for(int ni=0; ni<nImages; ni++){
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[2])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[2])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();


        // Pass the image to the SLAM system
//        SLAM.TrackRGBD(imRGB,imD,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();


        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();




        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    return 0;
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
