#include <QCoreApplication>
#include "registration.h"


int main(int argc, char *argv[])
{


    QCoreApplication a(argc, argv);

    cout<<"Hello"<<endl;
    const std::string inputPath1 = "C:\Users\dono_\Documents\1. MAIA LE CREUSOT\Software engineering\Project\G1-3D-Korn\3D-KORN-master\Source-Code\ScanRegistrationDev\chair_alb0.ply";
    const std::string inputPath2 = "C:\Users\dono_\Documents\1. MAIA LE CREUSOT\Software engineering\Project\G1-3D-Korn\3D-KORN-master\Source-Code\ScanRegistrationDev\chair_alb1.ply";

    const std::string OutputPath = "C:\Users\dono_\Documents\1. MAIA LE CREUSOT\Software engineering\Project\G1-3D-Korn\3D-KORN-master\Source-Code\ScanRegistrationDev\registered.ply";

    PointCloud::Ptr PointC1 (new PointCloud);
    PointCloud::Ptr PointC2 (new PointCloud);
    PointCloud::Ptr OutputPC (new PointCloud);

    ReadPointC (inputPath1, *PointC1);
    ReadPointC (inputPath2, *PointC2);

    OutputPC = *(Register (*PointC1 , *PointC2);
    Save_PointC(OutputPath , *OutputPC);
    return a.exec();
}
