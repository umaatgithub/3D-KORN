#include <QCoreApplication>
#include "registration.h"

#include <iostream>
using namespace std;

int main()
{



    cout<<"Hello"<<endl;
    const std::string inputPath1 = "D:\U2.Cloud\u2.cloud\Source-Code\RegistrationTest\chair_alb0.ply";
    const std::string inputPath2 = "D:\U2.Cloud\u2.cloud\Source-Code\RegistrationTest\chair_alb1.ply";

    const std::string OutputPath = "D:\U2.Cloud\u2.cloud\Source-Code\RegistrationTest\registered.ply";

    PointCloud::Ptr PointC1 (new PointCloud);
    PointCloud::Ptr PointC2 (new PointCloud);
    PointCloud::Ptr OutputPC (new PointCloud);

    ReadPointC (inputPath1, PointC1);
    ReadPointC (inputPath2, PointC2);

    OutputPC = Register (PointC1 , PointC2);
    Save_PointC(OutputPath , OutputPC);
    return 0;
}
