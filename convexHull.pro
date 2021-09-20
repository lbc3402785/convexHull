QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle
OBJECTS_DIR=$${PWD}/build
QMAKE_CXXFLAGS += /bigobj
# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
OPENCV_INCLUDE_DIRS=D:\soft\opencv-3.4.15\build-vs2015\install\include
OPENCV_LIBRARY_DIRS=D:\soft\opencv-3.4.15\build-vs2015\install\x64\vc14\lib
ZLIB_INCLUDE_DIRS=D:\soft\opencv-3.4.15\3rdparty\zlib
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
INCLUDEPATH+=D:\soft\eigen-3.4.0
INCLUDEPATH+="C:\Program Files\dlib_project\include"
INCLUDEPATH+=$$OPENCV_INCLUDE_DIRS
INCLUDEPATH+=$$ZLIB_INCLUDE_DIRS
LIBS+=-L$$OPENCV_LIBRARY_DIRS -lopencv_world3415
LIBS+=-L"C:\Program Files\dlib_project\lib" -ldlib19.22.99_release_64bit_msvc1916

#SOURCES += D:\soft\dlib\dlib\all\source.cpp
SOURCES += main.cpp \
    test.cpp \
    Dlib.cpp

HEADERS += \
    convexhull2d.h \
    convexhull3d.h \
    data.h \
    test.h \
    Dlib.h
