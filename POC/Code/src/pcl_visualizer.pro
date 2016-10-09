#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:24:33
#
#-------------------------------------------------

QT       += core gui
QT += widgets

TARGET = pcl_visualizer
TEMPLATE = app


SOURCES += main.cpp\
        pclviewer.cpp

HEADERS  += pclviewer.h \
    ui_pclviewer.h

FORMS    += pclviewer.ui

#openCV
-lopencv_core240d \
-lopencv_highgui240d  \
-lopencv_imgproc240d  \
-lopencv_features2d240d  \
-lopencv_calib3d240d \
-lopencv_contrib240d \
-lopencv_flann240d \
-lopencv_legacy240d \
-lopencv_ml240d \
-lopencv_objdetect240d \
-lopencv_ts240d \
#-lopencv_gpu240d \
-lopencv_video240d



LIBS += -LC:\\PCL_1_6_0\\3rdParty\\Qhull\\Lib\\ \
-lqhullstatic

LIBS += -LC:\\PCL_1_6_0\\lib\\ \
-lpcl_apps_release \
-lpcl_common_release \
-lpcl_features_release \
-lpcl_filters_release \
-lpcl_io_release \
-lpcl_io_ply_release \
-lpcl_kdtree_release \
-lpcl_keypoints_release \
-lpcl_octree_release \
#-lpcl_range_image_border_extractor_release \
-lpcl_registration_release \
-lpcl_sample_consensus_release \
-lpcl_search_release \
-lpcl_segmentation_release \
-lpcl_surface_release \
-lpcl_tracking_release \
-lpcl_visualization_release

LIBS += -LC:\\PCL_1_6_0\\3rdParty\\VTK\\lib\\vtk-5.8\\ \
-lMapReduceMPI \
-lmpistubs  \
-lQVTK \
-lvtkalglib \
-lvtkCharts \
-lvtkCommon \
-lvtkDICOMParser \
-lvtkexoIIc \
-lvtkexpat \
-lvtkFiltering \
-lvtkfreetype \
-lvtkftgl \
-lvtkGenericFiltering \
-lvtkGeovis \
-lvtkGraphics \
-lvtkhdf5 \
-lvtkHybrid \
-lvtkImaging \
-lvtkInfovis \
-lvtkIO \
-lvtkjpeg \
-lvtklibxml2 \
-lvtkmetaio \
-lvtkNetCDF \
-lvtkNetCDF_cxx \
-lvtkpng \
-lvtkproj4 \
-lvtkRendering \
-lvtksqlite \
-lvtksys \
-lvtktiff \
-lvtkverdict \
-lvtkViews \
-lvtkVolumeRendering \
-lvtkWidgets \
-lvtkzlib

LIBS += -LC:\\PCL_1_6_0\\3rdParty\\FLANN\\lib\\ \
-lflann_cpp_s

LIBS += -LC:\\PCL_1_6_0\\3rdParty\\Boost\\lib\\ \
-llibboost_date_time-vc100-mt-1_49 \
-llibboost_thread-vc100-mt-1_49 \
-llibboost_filesystem-vc100-mt-1_49 \
-llibboost_system-vc100-mt-1_49 \
-llibboost_iostreams-vc100-mt-1_49



LIBS += -LC:\\PCL_1_6_0\\3rdParty\\Qhull\\Lib\\ \
-lqhullstatic

LIBS += -LC:\\PCL_1_6_0\\lib\\ \
-lpcl_apps_release \
-lpcl_common_release \
-lpcl_features_release \
-lpcl_filters_release \
-lpcl_io_release \
-lpcl_io_ply_release \
-lpcl_kdtree_release \
-lpcl_keypoints_release \
-lpcl_octree_release \
#-lpcl_range_image_border_extractor_release \
-lpcl_registration_release \
-lpcl_sample_consensus_release \
-lpcl_search_release \
-lpcl_segmentation_release \
-lpcl_surface_release \
-lpcl_tracking_release \
-lpcl_visualization_release

LIBS += -LC:\\PCL_1_6_0\\3rdParty\\VTK\\lib\\vtk-5.8\\ \
-lMapReduceMPI \
-lmpistubs  \
-lvtkalglib \
-lvtkCharts \
-lvtkCommon \
-lvtkDICOMParser \
-lvtkexoIIc \
-lvtkexpat \
-lvtkFiltering \
-lvtkfreetype \
-lvtkftgl \
-lvtkGenericFiltering \
-lvtkGeovis \
-lvtkGraphics \
-lvtkhdf5 \
-lvtkHybrid \
-lvtkImaging \
-lvtkInfovis \
-lvtkIO \
-lvtkjpeg \
-lvtklibxml2 \
-lvtkmetaio \
-lvtkNetCDF \
-lvtkNetCDF_cxx \
-lvtkpng \
-lvtkproj4 \
-lvtkRendering \
-lvtksqlite \
-lvtksys \
-lvtktiff \
-lvtkverdict \
-lvtkViews \
-lvtkVolumeRendering \
-lvtkWidgets \
-lvtkzlib \
-lvtkViews

LIBS += -LC:\\PCL_1_6_0\\3rdParty\\FLANN\\lib\\ \
-lflann_cpp_s

LIBS += -LC:\\PCL_1_6_0\\3rdParty\\Boost\\lib\\ \
-llibboost_date_time-vc100-mt-1_49 \
-llibboost_thread-vc100-mt-1_49 \
-llibboost_filesystem-vc100-mt-1_49 \
-llibboost_system-vc100-mt-1_49 \
-llibboost_iostreams-vc100-mt-1_49

OTHER_FILES += \
    CMakeLists.txt
