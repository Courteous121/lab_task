#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace std;

using namespace cv;
vector<Mat> channels;
Mat imgb, imgb1;
vector<Vec4i> hierarchy;

vector<vector<Point>> t1;
Point2f vertices1[4];
// vector <vector<Point3f>> objp;
// objp.clear();
// objp.push_back(Point3f(0,0,0));//顺时针，第一个点在左上
// objp.push_back(Point3f(100,0,0));
// objp.push_back(Point3f(100,110,0));
// objp.push_back(Point3f(0,110,0));

// Mat camera_matrix(3,3,CV_64F);
// Mat distortion_coefficients(1,5,CV_64F);

//  camera_matrix=(Mat_<double>(3,3)<<1.6041191539594568e+03, 0., 6.3983687194220943e+02, 0.,
//         1.6047833493341714e+03, 5.1222951297937527e+02, 0., 0., 1.);
//  distortion_coefficients(Mat_<double>(1,5)<<-6.4910709385278609e-01, 8.6914328787426987e-01,
//        5.1733428362687644e-03, -4.1111054148847371e-03, 0.);

// double a[3][3]={1.6041191539594568e+03, 0., 6.3983687194220943e+02, 0.,1.6047833493341714e+03, 5.1222951297937527e+02, 0., 0., 1.};
// Mat camera_matrix(3,3,CV_64F,a);
// double b[5][1]={-6.4910709385278609e-01, 8.6914328787426987e-01,5.1733428362687644e-03, -4.1111054148847371e-03, 0.};
// Mat distortion_coefficients(5,1,CV_64F,b);

double cameraD[3][3] = {{
                            1.6041191539594568e+03,
                            0.,
                            6.3983687194220943e+02,
                        },
                        {
                            0.,
                            1.6047833493341714e+03,
                            5.1222951297937527e+02,
                        },
                        {
                            0.,
                            0.,
                            1.,
                        }};
double distC[5] = {-6.4910709385278609e-01, 8.6914328787426987e-01, 5.1733428362687644e-03, -4.1111054148847371e-03, 0.};

Mat cameraMatrix(3, 3, cv::DataType<double>::type, cameraD);
Mat distCoeffs(5, 1, cv::DataType<double>::type, distC);

string convertToString(double d)
{
    ostringstream os;
    if (os << d)
        return os.str();
}

class light
{
private:
    vector<vector<Point>> t0;
    RotatedRect rect1;
    Point2f vertices3[4];
    Rect brect;
    vector<RotatedRect> rectbox0;

public:
    light(vector<vector<Point>> t)
    {
        t0 = t;
    }
    vector<RotatedRect> getrectbox();
};

vector<RotatedRect> light::getrectbox()
{
    for (int i = 0; i < t0.size(); i++)
    {
        rect1 = minAreaRect(t0[i]);

        rect1.points(vertices3);
        brect = rect1.boundingRect();
        double m5 = rect1.size.width > rect1.size.height ? rect1.size.width : rect1.size.height;
        double n5 = rect1.size.width < rect1.size.height ? rect1.size.width : rect1.size.height;
        double m = m5 / n5;
        double n = brect.height / brect.width;
        // if (m > 3 && m < 8 && n > 0 && n < 6)

        if (m > 0 && m < 2.5 && rect1.size.area() > 100 && rect1.size.area() < 2000)
        {

            rectbox0.push_back(rect1);
        }
    }
    return rectbox0;
}

int main()
{
    vector<Point3f> objp;
    objp.clear();
    objp.push_back(Point3f(0, 0, 0)); //顺时针，第一个点在左上
    objp.push_back(Point3f(100.0f, 0, 0));
    objp.push_back(Point3f(100.0f, 110.0f, 0));
    objp.push_back(Point3f(0, 110.0f, 0));
    // objp.push_back(Point3f(50.0f,55.0f,0));//顺时针，第一个点在左上
    // objp.push_back(Point3f(50.0f,0,0));
    VideoCapture cap("能量机关.avi");
    Mat img;

    Point c0;
    while (1)
    {
        vector<Point2f> imgp;
        vector<Point> cbox;
        cap >> img;
        split(img, channels);
        imgb = channels.at(0);
        threshold(imgb, imgb1, 170, 255, THRESH_BINARY); //运用了通道分离加二值化，但是调处的灰度图不够明显，后期调试考虑hsv
        findContours(imgb1, t1, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

        vector<RotatedRect> rectbox;

        light s1(t1);

        rectbox = s1.getrectbox();
        for (int i = 0; i < static_cast<int>(rectbox.size()) - 1; i++)
        {

            rectbox[i].points(vertices1);
            double x = (vertices1[0].x + vertices1[2].x) / 2;
            double y = (vertices1[0].y + vertices1[2].y) / 2;
            Point c = Point(x, y);
            if (rectbox[i].size.area() < 1000)
            {
                c0 = c;
                imgp.push_back(vertices1[1]);
                imgp.push_back(vertices1[2]);
                imgp.push_back(vertices1[3]);
                imgp.push_back(vertices1[0]);
                Point2f m1, m2;
                m1.x = (vertices1[1].x + vertices1[3].x) / 2;
                m1.y = (vertices1[1].y + vertices1[3].y) / 2;
                m2.x = (vertices1[1].x + vertices1[3].x) / 2;
                m2.y = vertices1[1].y;
                // imgp.push_back(m1);
                // imgp.push_back(m2);
            }
            else
            {
                cbox.push_back(c);
            }
            circle(img, c, 5, Scalar(255, 255, 255), -1);

            for (int j = 0; j < 4; j++)
            {

                line(img, vertices1[j], vertices1[(j + 1) % 4], Scalar(255, 0, 255), 3, CV_AA);
            }
        }

        if (static_cast<int>(cbox.size()) - 1 >= 0)
            for (int i = 0; i < static_cast<int>(cbox.size()) - 1; i++)
            {
                if ((cbox[i].x - c0.x) != 0)
                {

                    double angle = atan2((cbox[i].y - c0.y), (cbox[i].x - c0.x)); // atan接收一个参数
                    double angle1 = angle * 57.3;
                    // cout<<angle1<<endl;
                    string Angle = convertToString(angle1);
                    cout << angle1 << "   " << Angle << endl;
                    putText(img, Angle, cbox[i], FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2); // angle必须是string数据类型
                }

                else
                {
                    if (cbox[i].y > c0.y)
                        cout << "90" << endl;
                    else
                        cout << "-90" << endl;
                }

                Mat objm, imgm;
                Mat(objp).convertTo(objm, CV_32F);
                // Mat(imgp).convertTo(imgm,CV_32F);
                Mat rvec, tvec;
                // Mat rotM(3,3,CV_64F);
                // Rodrigues(rotM,rvec);
                Mat rvecs = Mat::zeros(3, 1, CV_64FC1);
                Mat tvecs = Mat::zeros(3, 1, CV_64FC1);
                if (imgp.size() == 4)
                {
                    solvePnP(objm, imgp, cameraMatrix, distCoeffs, rvecs, tvecs);
                    double tx = tvecs.at<double>(0, 0);
                    double ty = tvecs.at<double>(1, 0);
                    double tz = tvecs.at<double>(2, 0);
                    double dis = sqrt(tx * tx + ty * ty + tz * tz);
                    cout << "距离：" << dis<<"mm" << endl;
                    string DIS = convertToString(dis);
                    putText(img, DIS, Point(0, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2); // angle必须是string数据类型
                }
                // solvePnP(objp,imgp,camera_matrix,distortion_coefficients,rvec,tvec,SOLVEPNP_EPNP );

                // double tx=tvec.at<double>(0,0);
                // double ty=tvec.at<double>(1,0);
                // double tz=tvec.at<double>(2,0);
                // double dis=sqrt(tx*tx+ty*ty+tz*tz);
                // cout<<"距离："<<dis<<endl;
            }

        imshow("1", img);

        if (waitKey(100) == 27)
            if (waitKey(0) == 27)
                break;
    }
}
