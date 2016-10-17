#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <iostream>
#include "openni_grabber.cpp"

using namespace std;
int main ()
{
  SimpleOpenNIProcessor v;
  v.run ();
  return (0);
}

