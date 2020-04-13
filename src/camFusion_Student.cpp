
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
    vector<cv::DMatch> kptMatchesInRoi;   // temporal storage
    double mean = 0.0;
    
    // add all matched keypoints that are in the bounding box into kpMatchesInRoi
    for(auto match : kptMatches)
    {
        if( boundingBox.roi.contains( kptsCurr[match.trainIdx].pt) )
            kptMatchesInRoi.push_back(match);
    }
    
    if(kptMatchesInRoi.size() > 0)    // ensures we have any left
    {
        // calculate mean distance value
        for(auto kptmatch : kptMatchesInRoi)
            mean += kptmatch.distance;
        mean /= kptMatchesInRoi.size();
            
        // only copy the ones smaller than the mean into the result
        for(auto kptMatch: kptMatchesInRoi)
        {
            if(kptMatch.distance < mean)
                boundingBox.kptMatches.push_back(kptMatch);
        }
    }
    // cout << __PRETTY_FUNCTION__<< " returns " << boundingBox.kptMatches.size() << " relevant matches" << endl;    
}


static double computeMedian(vector<double> distances)
{
    const size_t size = distances.size();

    sort(distances.begin(), distances.end() );
    if(size % 2)   // odd number of entries
         return (distances[size/2-1] + distances[size/2]) / 2.0;
    
    return distances[size/2];
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    

    // the for loop construct below actually fails in case no matches are delivered!
    // \FIXME this should be done more elegant by fixing the loop itself below.
    if(kptMatches.size() == 0)
    {
        TTC = NAN;
        return;
    }
    
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    // double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();

    double medianDistRatio = computeMedian(distRatios);
    
    double dT = 1 / frameRate;
    TTC = -dT / (1 - medianDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // approach is to ignore the closest 5% of the lidar points but use the next one
    
    // sort the lidar points from current and previous frame
    // uses lamda expression from example here https://en.cppreference.com/w/cpp/algorithm/sort
    std::sort(lidarPointsPrev.begin(), lidarPointsPrev.end(), [](LidarPoint a, LidarPoint b) {
        return a.x > b.x;
    });
    std::sort(lidarPointsCurr.begin(), lidarPointsCurr.end(), [](LidarPoint a, LidarPoint b) {
        return a.x > b.x;
    });

    const int indexPrev = static_cast<int> (lidarPointsPrev.size() * 0.95f);
    const int indexCurr = static_cast<int> (lidarPointsCurr.size() * 0.95f);
    
    const double d0 = lidarPointsPrev[indexPrev].x;    // distance at t0
    const double d1 = lidarPointsCurr[indexCurr].x;    // distance at t1
    
    // from Udacity, Estimating TTC with Lidar: TTC = D1 * dt / (d0-d1)  with dt=1/frameRate
    // =>  TTC = D1 / ((frameRate * (d0-d1)
    TTC = d1 / ((frameRate * (d0-d1)));
}


static int getBBoxId(cv::KeyPoint kp, vector<BoundingBox> &bBoxes)
{
    for(auto bBox: bBoxes)
    {
        if(bBox.roi.contains(kp.pt))
            return bBox.boxID;
    }
    return -1; // boxID -1 indicates no match!
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    int maxPrevFrameBoxId = 0;
    
    std::multimap<int, int> assoc;   // map to store the association between current box ID and previous box ID for each match.
    
   // cout << __PRETTY_FUNCTION__ << ": " << matches.size() << " matches" << endl;

    // for each match find according bounding boxes
    for(auto match: matches)
    {
        const int currBox = getBBoxId( currFrame.keypoints[match.trainIdx], currFrame.boundingBoxes);
        const int prevBox = getBBoxId( prevFrame.keypoints[match.queryIdx], prevFrame.boundingBoxes);
        
        // one entry for keypoints which are boxed in currFrame and prevFrame
        if(currBox!=-1 && prevBox!=-1)
            assoc.insert( { currBox, prevBox } );
        
        maxPrevFrameBoxId = std::max(maxPrevFrameBoxId, prevBox); // required to dimension the histogram vector array later
    }
    
    typedef struct {
        int currBoxId;
        int prevBoxId;
        int matches;
    } BoxAssocScore;
    
    vector<BoxAssocScore> myBoxMatches;
    
    
    // iterate of each bounding boxes of currFrame
    // fill myBoxMatches containing the relevant associations between boxes left
    for(auto bbox : currFrame.boundingBoxes)
    {
        vector<int> hist(maxPrevFrameBoxId+1, 0);   // we generate a histogram to easily find the most frequent prevframe BBox idx
        std::multimap<int, int>::iterator it;
        
        for(it=assoc.equal_range(bbox.boxID).first; it!=assoc.equal_range(bbox.boxID).second; ++it)
        {
            hist[(*it).second]++;
        }

        int max_prevFrameBBIdx = std::max_element(hist.begin(), hist.end()) -hist.begin() ;
        
        if(hist[max_prevFrameBBIdx]>0)
        {
            // bbBestMatches.insert( {max_prevFrameBBIdx, bbox.boxID} );      
            // cout << "   Match " << bbox.boxID << " - " << max_prevFrameBBIdx << " (" << hist[max_prevFrameBBIdx] << ")" << endl;
            myBoxMatches.push_back( {bbox.boxID, max_prevFrameBBIdx, hist[max_prevFrameBBIdx]} );
        }
    }
    
    // myBoxMatches now contains only one prevBoxId for each currBoxId, there might however be more than once the prevBoxId used.
    // so we now search reverse, trying to find for each prevFrame BBox the entry with most matches
    for(auto bbox : prevFrame.boundingBoxes)
    {
        int resCurrBoxId = -1;
        int resCurrBoxIdMatches = 0;
        
        for(auto myBoxMatch: myBoxMatches)
        {
            if( (myBoxMatch.prevBoxId == bbox.boxID) &&
                (myBoxMatch.matches >resCurrBoxIdMatches) )
            {
                resCurrBoxIdMatches = myBoxMatch.matches;
                resCurrBoxId = myBoxMatch.currBoxId;
            }
        }
        if(resCurrBoxId != -1 && resCurrBoxIdMatches > 0)
        {
            // cout << "  *Match " << resCurrBoxId <<  " - " << bbox.boxID << " (" << resCurrBoxIdMatches << ")" << endl;    
            bbBestMatches.insert( {bbox.boxID, resCurrBoxId } );
        }
    }
    // bbBestmatches contains the output of function 
    //cout << __FUNCTION__ << " returns " << bbBestMatches.size() << " matches" << endl;
}
