
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // Previous = Source = Query
    // Current = Reference = Train

    std::vector<double> euclideanDists;
    double maxDistanceFromMedian = 1.0;

    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        if (boundingBox.roi.contains(kptsCurr[(*it).trainIdx].pt))
        {
            double euclideanDist = cv::norm(kptsPrev[(*it).queryIdx].pt - kptsCurr[(*it).trainIdx].pt);
            euclideanDists.push_back(euclideanDist);
        }
    }

    std::sort(euclideanDists.begin(), euclideanDists.end());
    long medIndex = floor(euclideanDists.size() / 2.0);
    double medEuclideanDist = euclideanDists.size() % 2 == 0 ? (euclideanDists[medIndex - 1] + euclideanDists[medIndex]) / 2.0 : euclideanDists[medIndex];

    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        if (boundingBox.roi.contains(kptsCurr[(*it).trainIdx].pt))
        {
            double euclideanDist = cv::norm(kptsPrev[(*it).queryIdx].pt - kptsCurr[(*it).trainIdx].pt);

            if ((euclideanDist > (medEuclideanDist - maxDistanceFromMedian)) && (euclideanDist < (medEuclideanDist + maxDistanceFromMedian)))
            {
                boundingBox.kptMatches.push_back(*it);
            }
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame

    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    {
        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        {
            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            // avoid division by zero
            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            {
                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        }
    }

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex];

    double dT = 1.0 / frameRate; // time between two measurements in seconds
    TTC = -dT / (1.0 - medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1.0 / frameRate; // time between two measurements in seconds
    double laneWidth = 4.0;      // assumed width of the ego lane

    // calculate the mean x position (previous and current)
    double avgXPrev = 0.0, avgXCurr = 0.0;

    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        avgXPrev += it->x;
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        avgXCurr += it->x;
    }

    avgXPrev /= lidarPointsPrev.size();
    avgXCurr /= lidarPointsCurr.size();

    // calculate the standard deviation (previous and current)
    double varPrev = 0.0, varCurr = 0.0;

    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        varPrev += (it->x - avgXPrev) * (it->x - avgXPrev);
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        varCurr += (it->x - avgXCurr) * (it->x - avgXCurr);
    }

    varPrev /= lidarPointsPrev.size();
    varCurr /= lidarPointsCurr.size();

    double stdPrev = sqrt(varPrev);
    double stdCurr = sqrt(varCurr);

    // find closest distance to Lidar points within ego lane
    double stdNum = 0.5;
    double minXPrev = 1e9, minXCurr = 1e9;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        // Remove lidar points outside lane or stdNum standard deviations closer to the host than the mean lidar point
        if ((abs(it->y) <= (laneWidth / 2.0)) && (it->x > (avgXPrev - (stdNum * stdPrev))))
        {
            minXPrev = minXPrev > it->x ? it->x : minXPrev;
        }
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        // Remove lidar points outside lane or stdNum standard deviations closer to the host than the mean lidar point
        if ((abs(it->y) <= (laneWidth / 2.0)) && (it->x > (avgXCurr - (stdNum * stdCurr))))
        {
            minXCurr = minXCurr > it->x ? it->x : minXCurr;
        }
    }

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // Previous = Source = Query
    // Current = Reference = Train

    vector<int> matchedIDs;

    // Loop over all bounding boxes in the current frame
    for (std::vector<BoundingBox>::iterator current_it = currFrame.boundingBoxes.begin(); current_it != currFrame.boundingBoxes.end(); ++current_it)
    {
        int bbCurrentID = (*current_it).boxID;
        map<int, int> bbMatches; // <prev boxID, cnt>

        // Loop over all matches
        for (std::vector<cv::DMatch>::iterator matches_it = matches.begin(); matches_it != matches.end(); ++matches_it)
        {
            // Check whether the keypoint match is contained in the bounding box
            if ((*current_it).roi.contains(currFrame.keypoints[(*matches_it).trainIdx].pt))
            {
                // Loop over all bounding boxes in the previous frame
                for (std::vector<BoundingBox>::iterator previous_it = prevFrame.boundingBoxes.begin(); previous_it != prevFrame.boundingBoxes.end(); ++previous_it)
                {
                    // Check whether the keypoint match is contained in the bounding box
                    if ((*previous_it).roi.contains(prevFrame.keypoints[(*matches_it).queryIdx].pt))
                    {
                        if (0 == bbMatches.count((*previous_it).boxID))
                        {
                            bbMatches.insert(pair<int, int>((*previous_it).boxID, 0));
                        }

                        bbMatches.at((*previous_it).boxID)++;
                    }
                }
            }
        }

        int bbPreviousID;
        int maxMatches = 0;

        for (map<int, int>::iterator map_it = bbMatches.begin(); map_it != bbMatches.end(); ++map_it)
        {
            if ((map_it->second > maxMatches) && (find(matchedIDs.begin(), matchedIDs.end(), map_it->first) == matchedIDs.end()))
            {
                maxMatches = map_it->second;
                bbPreviousID = map_it->first;
            }
        }

        if (maxMatches != 0)
        {
            matchedIDs.push_back(bbPreviousID);
            bbBestMatches.insert(pair<int, int>(bbPreviousID, bbCurrentID));
        }
    }
}
