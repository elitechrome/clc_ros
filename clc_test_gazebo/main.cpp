#if 1
#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <eigen/Eigen>
#include <Eigen/Geometry>

#include "CLC.h"
using namespace cv;
using namespace Eigen;

std::vector<cv::Point2d> keypoints1;
int i = 0;
int j = 0;
Point2f um(512, 384); //image center point

//callback function
void mouseEvent1(int evt, int x, int y, int flags, void* param){
    cv::Mat *src1 = (cv::Mat*)param;
    cv::Point pot;
    //cv::imshow("src1",*src1);
    
    if (evt == CV_EVENT_LBUTTONDOWN && i<4){
        //keypoints1[i].pt.x = x;
        //keypoints1[i].pt.y = y;
        pot = cv::Point(x, y);
        cv::circle(*src1, pot, 5, cv::Scalar(0, 255, 0), 4, 5);
        
        keypoints1.push_back(cv::Point(x, y));
        printf("사각형의 %d번째 꼭지점의 좌표(%d, %d)\n", i + 1, x, y);
        cv::imshow("Image1", *src1);
        i++;
    }
}

int main(void)
{
    cv::Mat original_img = cv::imread("2.png");
    cv::Mat image1;
    if (original_img.empty())
    {
        std::cout << "image open error." << std::endl;
        return -1;
    }
    CLC clc;
    
    while (1)
    {
        keypoints1.clear();
        i = 0;
        original_img.copyTo(image1);
        resize(image1, image1, Size(1024, 768), 0, 0, INTER_CUBIC);
        cv::namedWindow("Image1", CV_WINDOW_AUTOSIZE);
        cv::imshow("Image1", image1);
        cv::setMouseCallback("Image1", mouseEvent1, &image1);
        cv::waitKey();
        if (keypoints1.empty()){
            std::cout << "error, no points are selected.\n";
            return -1;
        }
        clc.SetOffCenteredQuad(keypoints1);
        clc.FindProxyQuadrilateral();
        Vector3d trans; Quaternion<double> q;
        clc.CalcCLC(trans, q);
        clc.Visualization(image1);
        cv::imshow("Image1", image1);
        cv::waitKey();
    }
    return 0;
}
#endif
#if 0
#include <opencv2/opencv.hpp>
#include <eigen/Eigen>
#include <Eigen/Geometry>

#include "CLC.h"

#include <iostream>
#include <math.h>
#include <string.h>

using namespace cv;
using namespace std;
using namespace Eigen;

static void help()
{
    cout <<
    "\nA program using pyramid scaling, Canny, contours, contour simpification and\n"
    "memory storage (it's got it all folks) to find\n"
    "squares in a list of images pic1-6.png\n"
    "Returns sequence of squares detected on the image.\n"
    "the sequence is stored in the specified memory storage\n"
    "Call:\n"
    "./squares\n"
    "Using OpenCV version %s\n" << CV_VERSION << "\n" << endl;
}


int thresh = 50, N = 1;
const char* wndname = "Square Detection Demo";

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
static void findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();
    
    Mat pyr, timg, gray0(image.size(), CV_8U), gray;
    
    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;
    
    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);
        
        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }
            
            // find contours and store them all as a list
            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
            
            vector<Point> approx;
            
            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
                
                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                   fabs(contourArea(Mat(approx))) > 1000 &&
                   isContourConvex(Mat(approx)) )
                {
                    double maxCosine = 0;
                    
                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }
                    
                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.5 )
                        squares.push_back(approx);
                }
            }
        }
    }
}


// the function draws all the squares in the image
static void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,0,255));
    }
    
    imshow(wndname, image);
}


int main(int /*argc*/, char** /*argv*/)
{
    static const char* names[] = { "IMG_2660.jpg", "IMG_2655.JPG","IMG_2663.JPG","IMG_2665.JPG", "photo.jpg","Poster-Board-1.jpg", 0 };
    help();
    namedWindow( wndname, 1 );
    vector<vector<Point> > squares;
    Mat image;
    
    for( int i = 0; names[i] != 0; i++ )
    {
        Mat original_image = imread(names[i], 1);
        if( original_image.empty() )
        {
            cout << "Couldn't load " << names[i] << endl;
            continue;
        }
        
        original_image.copyTo(image);
        resize(image, image, Size(1024, 768), 0, 0, INTER_CUBIC);
        CLC clc;
        findSquares(image, squares);
        drawSquares(image, squares);
        for(int i =0; i < squares.size();i++)
        {
            vector<Point2d> tmp_square;
            tmp_square.push_back(squares[i][0]);
            tmp_square.push_back(squares[i][1]);
            tmp_square.push_back(squares[i][2]);
            tmp_square.push_back(squares[i][3]);
            clc.SetOffCenteredQuad(tmp_square);
            clc.FindProxyQuadrilateral();
            Vector3d trans; Quaternion<double> q;
            if(clc.CalcCLC(trans, q))
                clc.Visualization(image);
            cv::imshow("Image", image);
        }
        
        int c = waitKey();
        if( (char)c == 27 )
            break;
    }
    
    return 0;
}
#endif