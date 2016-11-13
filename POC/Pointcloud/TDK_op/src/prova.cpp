for (size_t i = 0; i < cloud_RGB->points.size (); ++i)
 {

     if (minx > cloud_RGB->points[i].x) //check on x

         minx = cloud_RGB->points[i].x;

     if (maxx < cloud_RGB->points[i].x)
         maxx = cloud_RGB->points[i].x;


     if (miny > cloud_RGB->points[i].y) //check on y
         miny = cloud_RGB->points[i].y;

     if (maxy < cloud_RGB->points[i].y)
         maxy = cloud_RGB->points[i].y;

     if (minz > cloud_RGB->points[i].z) //check on z
         minz = cloud_RGB->points[i].z;

     if (maxz < cloud_RGB->points[i].z)
         maxz = cloud_RGB->points[i].z;

 }
//LU: find the distanze between each direction
kx=(maxx-minx)/255+minx;

ky=(maxy-miny)/255+miny;


kz=(maxz-minz)/255+minz;

viewer->addPointcloud_RGB (cloud_RGB, "cloud_RGB");
pSliderValueChanged (2);
viewer->resetCamera ();
ui->qvtkWidget->update ();
