#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include "CLC.h"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        std::vector<vector<Point2f> > squares;
        cv::Mat original_image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat image;
        original_image.copyTo(image);
        resize(image, image, Size(640, 480), 0, 0, INTER_LINEAR);
        CLC clc;
        //clc.findSquares(image, squares);
        double l = 0.1818175, phi = 0.61546*2;

        cv::Mat V1 = (cv::Mat_<double>(4, 1) << l, 0, 0.000001, 1);
        cv::Mat V2 = (cv::Mat_<double>(4, 1) << l*cos(phi), l*sin(phi), 0.000001, 1);
        cv::Mat V3 = (cv::Mat_<double>(4, 1) << -l, 0, 0.000001, 1);
        cv::Mat V4 = (cv::Mat_<double>(4, 1) << -l*cos(phi), -l*sin(phi), 0.000001, 1);

        cv::Mat U1 = (cv::Mat_<double>(3, 1) << 0, 0, 0);
        cv::Mat U2 = (cv::Mat_<double>(3, 1) << 0, 0, 0);
        cv::Mat U3 = (cv::Mat_<double>(3, 1) << 0, 0, 0);
        cv::Mat U4 = (cv::Mat_<double>(3, 1) << 0, 0, 0);

        // Intrinsic
        double fx = 283.11208;
        double fy = 283.11208;
        double cx = 320;
        double cy = 240;
        cv::Mat intrinsic = (cv::Mat_<double>(3, 3) <<
                             fx,  0, cx,
                              0, fy, cy,
                              0,  0,  1
                         );

         // Extrinsic
        cv::Mat extrinsic = (cv::Mat_<double>(3, 4) <<
                              1,  0,  0, 0,
                              0,  1,  0, 0,
                              0,  0,  1, 0);
        //Get TF
        tf::TransformListener listener;
        tf::StampedTransform transform;
        try{
            ros::Time now = ros::Time(0);
            listener.waitForTransform("/camera_frame", "/rect", now, ros::Duration(1));
            listener.lookupTransform("/camera_frame", "/rect", now, transform);
            tf::Matrix3x3 rot = transform.getBasis();
            tf::Vector3 trans = transform.getOrigin();


            cv::Mat R = (cv::Mat_<double>(3, 3)    <<
            rot[0][0], rot[0][1], rot[0][2],
            rot[1][0], rot[1][1], rot[1][2],
            rot[2][0], rot[2][1], rot[2][2]
            );

            cv::Mat tvec = (cv::Mat_<double>(3, 1) << trans[0],trans[1],trans[2]);
            R.copyTo(extrinsic.rowRange(0, 3).colRange(0, 3));
            tvec.copyTo(extrinsic.rowRange(0, 3).col(3));
            std::cout << "extrinsic(tf):\n" << extrinsic  <<std::endl;
            //tf::Quaternion rot_qt = transform.getRotation();
            std::cout << "t(tf):\n" << (-R.inv()*tvec).t() <<std::endl;
            //std::cout << "tf_quaternion: " <<rot_qt.getX()<<","<<rot_qt.getY()<<","<<rot_qt.getZ()<< "," <<rot_qt.getW()<< std::endl;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        U1 = intrinsic * extrinsic * V1;
        U2 = intrinsic * extrinsic * V2;
        U3 = intrinsic * extrinsic * V3;
        U4 = intrinsic * extrinsic * V4;
        U1/=U1.at<double>(2, 0);
        U2/=U2.at<double>(2, 0);
        U3/=U3.at<double>(2, 0);
        U4/=U4.at<double>(2, 0);

        vector<Point2f> tmp_square;
        tmp_square.push_back(Point2f(U1.at<double>(0,0),U1.at<double>(1,0)));
        tmp_square.push_back(Point2f(U2.at<double>(0,0),U2.at<double>(1,0)));
        tmp_square.push_back(Point2f(U3.at<double>(0,0),U3.at<double>(1,0)));
        tmp_square.push_back(Point2f(U4.at<double>(0,0),U4.at<double>(1,0)));
        squares.push_back(tmp_square);
//        std::cout << extrinsic  <<std::endl;

        //std::cout<< U1<<std::endl;
        clc.drawSquares(image, squares);
        for(int i =0; i < squares.size();i++)
        {
            vector<Point2f> tmp_square;
            tmp_square.push_back(squares[i][0]);
            tmp_square.push_back(squares[i][1]);
            tmp_square.push_back(squares[i][2]);
            tmp_square.push_back(squares[i][3]);
            clc.SetOffCenteredQuad(tmp_square);
            clc.FindProxyQuadrilateral();
            Vector3d trans_pnp, trans_clc;
            Quaternion<double> q; Matrix3d rot_pnp, rot_clc;

            clc.CalcPnP(trans_pnp, rot_pnp);
            if(clc.CalcCLC(trans_clc, rot_clc)){
                clc.Visualization(image);

            }
            std::cout<<std::endl<<"R_pnp\n"<<rot_pnp<<std::endl;
            std::cout<<"R_CLC\n"<<rot_clc<<std::endl;
            std::cout<<std::endl<<"t_pnp\n"<<trans_pnp<<std::endl;
            std::cout<<"t_CLC\n"<<trans_clc   <<std::endl;
        }

        imshow("view", image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clc_test_gazebo");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/multisense_sl/camera/left/image_raw", 1, imageCallback);

    ros::spin();
    cv::destroyAllWindows();
}
