#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace std;

using namespace cv;

vector<Vec4i> hierarchy;

vector<vector<Point>> t1;

Point2f vertices1[4];
Point2f vertices2[4];


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
Mat cameraMatrix(3, 3, cv::DataType<double>::type, cameraD);

double distC[5] = {-6.4910709385278609e-01, 8.6914328787426987e-01, 5.1733428362687644e-03, -4.1111054148847371e-03, 0.};

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
    RotatedRect rect1; // rect1是旋转最小矩形
    Point2f vertices3[4];
    Rect brect;
    vector<RotatedRect> rectbox0; //筛选出的光条会放在里面

public:
    light(vector<vector<Point>> t) // t是装脚点的向量
    {
        t0 = t;
    }
    vector<RotatedRect> getrectbox(); //筛选光条的函数
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
        if (m > 0 && m < 8 && n > 0 && n < 8 && brect.area() > 50 && rect1.size.area() > 50) // m在4左右
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
    objp.push_back(Point3f(240, 0, 0));
    objp.push_back(Point3f(240, 100, 0)); // 2.4  1
    objp.push_back(Point3f(0, 100, 0));

    VideoCapture cap("装甲板.avi");
    vector<Point> prebox;
    while (1)
    {
        Mat img, imghsv, img2;
        cap >> img;

        cvtColor(img, imghsv, COLOR_BGR2HSV);

        inRange(imghsv, Scalar(0, 85, 40), Scalar(35, 255, 255), img2);

        // findContours(img2, t1, hierarchy, RETR_LIST, CV_CHAIN_APPROX_NONE);
        findContours(img2, t1, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

        vector<RotatedRect> rectbox;

        light s1(t1);
        rectbox = s1.getrectbox(); // rectbox里面是筛选出来的光条

        vector<Point2f> imgp;

        for (int i = 0; i < static_cast<int>(rectbox.size()) - 1; i++)
        {
            rectbox[i].points(vertices1);
            rectbox[i + 1].points(vertices2);
          
for(int i=0;i<3;i++)
{
    for(int j=i+1;j<4;j++)
    {
        if(vertices1[i].x>vertices1[j].x)
        {
Point2f temp=vertices1[i];
vertices1[i]=vertices1[j];
vertices1[j]=temp;

        }
    }
}

if(vertices1[0].y>vertices1[1].y)
{
 Point temp=vertices1[0];
 vertices1[0]=vertices1[1];
 vertices1[1]=temp;

}

if(vertices1[2].y>vertices1[3].y)
{
 Point temp=vertices1[2];
 vertices1[2]=vertices1[3];
 vertices1[3]=temp;

}



for(int i=0;i<3;i++)
{
    for(int j=i+1;j<4;j++)
    {
        if(vertices2[i].x>vertices2[j].x)
        {
Point2f temp=vertices2[i];
vertices2[i]=vertices2[j];
vertices2[j]=temp;

        }
    }
}

if(vertices2[0].y>vertices2[1].y)
{
 Point temp=vertices2[0];
 vertices2[0]=vertices2[1];
 vertices2[1]=temp;

}
if(vertices2[2].y>vertices2[3].y)
{
 Point temp=vertices2[2];
 vertices2[2]=vertices2[3];
 vertices2[3]=temp;

}
            Point m1,n1,m2,n2;
            m1.x=(vertices1[0].x+vertices1[2].x)/2;
            m1.y=(vertices1[0].y+vertices1[2].y)/2;
            n1.x=(vertices1[1].x+vertices1[3].x)/2;
            n1.y=(vertices1[1].y+vertices1[3].y)/2;
            m2.x=(vertices2[0].x+vertices2[2].x)/2;
            m2.y=(vertices2[0].y+vertices2[2].y)/2;
            n2.x=(vertices2[1].x+vertices2[3].x)/2;
            n2.y=(vertices2[1].y+vertices2[3].y)/2;

            float m10 = rectbox[i].size.height > rectbox[i].size.width ? rectbox[i].size.height : rectbox[i].size.width;
            float n10 = sqrt(pow((vertices1[3].x - vertices2[3].x), 2) + pow((vertices1[3].y - vertices2[3].y), 2));
            float m = n10 / m10;
            float n = n10 * m10;
            // double m20 = abs(vertices2[3].x - vertices1[1].x);
            // double n20 = abs(vertices2[3].y - vertices1[1].y);
            // double m0 = m20 / n20;
            if (m > 0 && m < 8 && n < 40000 && n > 2000)
            {

                line(img, m1, n2, Scalar(0, 0, 255), 1);
                line(img, m2, n1, Scalar(0, 0, 255), 1);
                double x1 = (m1.x + n2.x) / 2;
                double y1 = (m1.y + n2.y) / 2;
                circle(img, Point(x1, y1), 5, Scalar(0, 255, 255), -1);

                prebox.push_back(Point(x1, y1));
                
                    imgp.push_back(m1);
                    imgp.push_back(m2);
                    imgp.push_back(n2);
                    imgp.push_back(n1);
                
            }
        }
        Point pre;
        int n = prebox.size() - 1;

        if (n - 1 >= 0)
        {
            pre.x = 2 * prebox[n].x - prebox[n - 1].x;
            pre.y = 2 * prebox[n].y - prebox[n - 1].y;
            circle(img, pre, 5, Scalar(255, 255, 255), -1);
        }
        Mat rvecs = Mat::zeros(3, 1, CV_64FC1);
        Mat tvecs = Mat::zeros(3, 1, CV_64FC1);
        Mat objm, imgm;
        Mat(objp).convertTo(objm, CV_32F);
        if (imgp.size() == 4)
        {
            solvePnP(objm, imgp, cameraMatrix, distCoeffs, rvecs, tvecs);
            Mat rotm;
            Rodrigues(rvecs, rotm);

double a1=atan2(rotm.at<double>(1,2),rotm.at<double>(2,2))*57.3;
double c2=sqrt(rotm.at<double>(0,0)*rotm.at<double>(0,0)+rotm.at<double>(0,1)*rotm.at<double>(0,1));
double a2=atan2(-rotm.at<double>(0,2),c2)*57.3;
double a3=atan2(rotm.at<double>(0,1),rotm.at<double>(0,0))*57.3;
    
            string A11 = convertToString(a1);
            string A22 = convertToString(a2);
            string A33 = convertToString(a3);
            
            
            putText(img, A11, Point(0, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 1); // angle必须是string数据类型
            putText(img, A22, Point(0, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 1); // angle必须是string数据类型
            putText(img, A33, Point(0, 90), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 1); // angle必须是string数据类型



double tx = tvecs.at<double>(0, 0);
                    double ty = tvecs.at<double>(1, 0);
                    double tz = tvecs.at<double>(2, 0);
                    double dis = sqrt(tx * tx + ty * ty + tz * tz);
                    cout << "距离：" << dis <<"mm"<< endl;
                    string DIS = convertToString(dis);
                    putText(img, DIS, Point(0, 120), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2); // angle必须是string数据类型
                
        }

        imshow("1", img);
        // imshow("2",img2);

        if (waitKey(50) == 27)
            if (waitKey(0) == 27)
                break;
    }

    return 0;
}
