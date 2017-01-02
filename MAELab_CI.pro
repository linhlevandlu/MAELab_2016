TEMPLATE = app
TARGET = 
DEPENDPATH += .
INCLUDEPATH += "/usr/local/include"
LIBS += -L"/usr/lib" -lOpenCL \
	

QT           += opengl
CONFIG       += console
#RESOURCES     = Ipm.qrc
HEADERS += io/*.h io/LibJpeg/*.h imageModel/*.h \
			segmentation/*.h histograms/*.h pht/*.h \
		  pointInterest/*.h correlation/*.h utils/*.h ui/*.h MAELab.h 
	
SOURCES += MAELab_CI.cpp io/*.cpp io/LibJpeg/*.c imageModel/*.cpp \
		segmentation/*.cpp histograms/*.cpp pht/*.cpp \
		pointInterest/*.cpp correlation/*.cpp \
		 utils/*.cpp ui/*.cpp MAELab.cpp 