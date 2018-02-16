#############################################################################
# Makefile for building: MAELab_CI
# Generated by qmake (2.01a) (Qt 4.8.7) on: Fri Feb 16 12:28:30 2018
# Project:  MAELab_CI.pro
# Template: app
# Command: /usr/lib/x86_64-linux-gnu/qt4/bin/qmake -o Makefile MAELab_CI.pro
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = -DQT_NO_DEBUG -DQT_OPENGL_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED
CFLAGS        = -m64 -pipe -fPIC -lpthread -O2 -Wall -W -D_REENTRANT $(DEFINES)
CXXFLAGS      = -m64 -pipe -fPIC -lpthread -O2 -Wall -W -D_REENTRANT $(DEFINES)
INCPATH       = -I/usr/share/qt4/mkspecs/linux-g++-64 -I. -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtOpenGL -I/usr/include/qt4 -I/usr/local/include -I/usr/X11R6/include -I.
LINK          = g++
LFLAGS        = -m64 -Wl,-O1
LIBS          = $(SUBLIBS)  -L/usr/lib/x86_64-linux-gnu -L/usr/X11R6/lib64 -L/usr/lib -lQtOpenGL -lQtGui -lQtCore -lGL -lpthread 
AR            = ar cqs
RANLIB        = 
QMAKE         = /usr/lib/x86_64-linux-gnu/qt4/bin/qmake
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = ./

####### Files

SOURCES       = io/JPEGReader.cpp \
		io/Reader.cpp \
		io/TPSReader.cpp \
		io/LibJpeg/jaricom.c \
		io/LibJpeg/jcapimin.c \
		io/LibJpeg/jcapistd.c \
		io/LibJpeg/jcarith.c \
		io/LibJpeg/jccoefct.c \
		io/LibJpeg/jccolor.c \
		io/LibJpeg/jcdctmgr.c \
		io/LibJpeg/jchuff.c \
		io/LibJpeg/jcinit.c \
		io/LibJpeg/jcmainct.c \
		io/LibJpeg/jcmarker.c \
		io/LibJpeg/jcmaster.c \
		io/LibJpeg/jcomapi.c \
		io/LibJpeg/jcparam.c \
		io/LibJpeg/jcprepct.c \
		io/LibJpeg/jcsample.c \
		io/LibJpeg/jctrans.c \
		io/LibJpeg/jdapimin.c \
		io/LibJpeg/jdapistd.c \
		io/LibJpeg/jdarith.c \
		io/LibJpeg/jdatadst.c \
		io/LibJpeg/jdatasrc.c \
		io/LibJpeg/jdcoefct.c \
		io/LibJpeg/jdcolor.c \
		io/LibJpeg/jddctmgr.c \
		io/LibJpeg/jdhuff.c \
		io/LibJpeg/jdinput.c \
		io/LibJpeg/jdmainct.c \
		io/LibJpeg/jdmarker.c \
		io/LibJpeg/jdmaster.c \
		io/LibJpeg/jdmerge.c \
		io/LibJpeg/jdpostct.c \
		io/LibJpeg/jdsample.c \
		io/LibJpeg/jdtrans.c \
		io/LibJpeg/jerror.c \
		io/LibJpeg/jfdctflt.c \
		io/LibJpeg/jfdctfst.c \
		io/LibJpeg/jfdctint.c \
		io/LibJpeg/jidctflt.c \
		io/LibJpeg/jidctfst.c \
		io/LibJpeg/jidctint.c \
		io/LibJpeg/jmemmgr.c \
		io/LibJpeg/jmemnobs.c \
		io/LibJpeg/jquant1.c \
		io/LibJpeg/jquant2.c \
		io/LibJpeg/jutils.c \
		imageModel/Edge.cpp \
		imageModel/Image.cpp \
		imageModel/Line.cpp \
		imageModel/Matrix.cpp \
		imageModel/Point.cpp \
		segmentation/Canny.cpp \
		segmentation/Filters.cpp \
		segmentation/Projection.cpp \
		segmentation/Suzuki.cpp \
		segmentation/Thresholds.cpp \
		histograms/LBP.cpp \
		histograms/ShapeHistogram.cpp \
		pht/GHTInPoint.cpp \
		pht/PCA.cpp \
		pht/PHoughTransform.cpp \
		pht/PHTEntry.cpp \
		pht/SVD.cpp \
		pointInterest/GeometricHistgoram.cpp \
		pointInterest/LandmarkDetection.cpp \
		pointInterest/ProHoughTransform.cpp \
		pointInterest/Segmentation.cpp \
		pointInterest/Treatments.cpp \
		correlation/CrossCorrelation.cpp \
		correlation/DescriptorDistance.cpp \
		utils/Converter.cpp \
		utils/Drawing.cpp \
		utils/ImageConvert.cpp \
		ui/ImageViewer.cpp \
		MAELab.cpp \
		MAELab_Procrustes4.cpp moc_ImageViewer.cpp
OBJECTS       = JPEGReader.o \
		Reader.o \
		TPSReader.o \
		jaricom.o \
		jcapimin.o \
		jcapistd.o \
		jcarith.o \
		jccoefct.o \
		jccolor.o \
		jcdctmgr.o \
		jchuff.o \
		jcinit.o \
		jcmainct.o \
		jcmarker.o \
		jcmaster.o \
		jcomapi.o \
		jcparam.o \
		jcprepct.o \
		jcsample.o \
		jctrans.o \
		jdapimin.o \
		jdapistd.o \
		jdarith.o \
		jdatadst.o \
		jdatasrc.o \
		jdcoefct.o \
		jdcolor.o \
		jddctmgr.o \
		jdhuff.o \
		jdinput.o \
		jdmainct.o \
		jdmarker.o \
		jdmaster.o \
		jdmerge.o \
		jdpostct.o \
		jdsample.o \
		jdtrans.o \
		jerror.o \
		jfdctflt.o \
		jfdctfst.o \
		jfdctint.o \
		jidctflt.o \
		jidctfst.o \
		jidctint.o \
		jmemmgr.o \
		jmemnobs.o \
		jquant1.o \
		jquant2.o \
		jutils.o \
		Edge.o \
		Image.o \
		Line.o \
		Matrix.o \
		Point.o \
		Canny.o \
		Filters.o \
		Projection.o \
		Suzuki.o \
		Thresholds.o \
		LBP.o \
		ShapeHistogram.o \
		GHTInPoint.o \
		PCA.o \
		PHoughTransform.o \
		PHTEntry.o \
		SVD.o \
		GeometricHistgoram.o \
		LandmarkDetection.o \
		ProHoughTransform.o \
		Segmentation.o \
		Treatments.o \
		CrossCorrelation.o \
		DescriptorDistance.o \
		Converter.o \
		Drawing.o \
		ImageConvert.o \
		ImageViewer.o \
		MAELab.o \
		MAELab_Procrustes4.o \
		moc_ImageViewer.o
DIST          = /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/shared.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/unix/opengl.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		MAELab_CI.pro
QMAKE_TARGET  = MAELab_CI
DESTDIR       = 
TARGET        = MAELab_CI

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

Makefile: MAELab_CI.pro  /usr/share/qt4/mkspecs/linux-g++-64/qmake.conf /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/shared.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/unix/opengl.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		/usr/lib/x86_64-linux-gnu/libQtOpenGL.prl \
		/usr/lib/x86_64-linux-gnu/libQtGui.prl \
		/usr/lib/x86_64-linux-gnu/libQtCore.prl
	$(QMAKE) -o Makefile MAELab_CI.pro
/usr/share/qt4/mkspecs/common/unix.conf:
/usr/share/qt4/mkspecs/common/linux.conf:
/usr/share/qt4/mkspecs/common/gcc-base.conf:
/usr/share/qt4/mkspecs/common/gcc-base-unix.conf:
/usr/share/qt4/mkspecs/common/g++-base.conf:
/usr/share/qt4/mkspecs/common/g++-unix.conf:
/usr/share/qt4/mkspecs/qconfig.pri:
/usr/share/qt4/mkspecs/features/qt_functions.prf:
/usr/share/qt4/mkspecs/features/qt_config.prf:
/usr/share/qt4/mkspecs/features/exclusive_builds.prf:
/usr/share/qt4/mkspecs/features/default_pre.prf:
/usr/share/qt4/mkspecs/features/release.prf:
/usr/share/qt4/mkspecs/features/default_post.prf:
/usr/share/qt4/mkspecs/features/shared.prf:
/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf:
/usr/share/qt4/mkspecs/features/warn_on.prf:
/usr/share/qt4/mkspecs/features/qt.prf:
/usr/share/qt4/mkspecs/features/unix/opengl.prf:
/usr/share/qt4/mkspecs/features/unix/thread.prf:
/usr/share/qt4/mkspecs/features/moc.prf:
/usr/share/qt4/mkspecs/features/resources.prf:
/usr/share/qt4/mkspecs/features/uic.prf:
/usr/share/qt4/mkspecs/features/yacc.prf:
/usr/share/qt4/mkspecs/features/lex.prf:
/usr/share/qt4/mkspecs/features/include_source_dir.prf:
/usr/lib/x86_64-linux-gnu/libQtOpenGL.prl:
/usr/lib/x86_64-linux-gnu/libQtGui.prl:
/usr/lib/x86_64-linux-gnu/libQtCore.prl:
qmake:  FORCE
	@$(QMAKE) -o Makefile MAELab_CI.pro

dist: 
	@$(CHK_DIR_EXISTS) .tmp/MAELab_CI1.0.0 || $(MKDIR) .tmp/MAELab_CI1.0.0 
	$(COPY_FILE) --parents $(SOURCES) $(DIST) .tmp/MAELab_CI1.0.0/ && $(COPY_FILE) --parents io/JPEGReader.h io/Reader.h io/TPSReader.h io/LibJpeg/jconfig.h io/LibJpeg/jdct.h io/LibJpeg/jerror.h io/LibJpeg/jinclude.h io/LibJpeg/jmemsys.h io/LibJpeg/jmorecfg.h io/LibJpeg/jpegint.h io/LibJpeg/jpeglib.h io/LibJpeg/jversion.h imageModel/Edge.h imageModel/Image.h imageModel/Line.h imageModel/Matrix.h imageModel/Point.h segmentation/Canny.h segmentation/Filters.h segmentation/Projection.h segmentation/Suzuki.h segmentation/Thresholds.h histograms/LBP.h histograms/ShapeHistogram.h pht/GHTInPoint.h pht/PCA.h pht/PHoughTransform.h pht/PHTEntry.h pht/SVD.h pointInterest/GeometricHistgoram.h pointInterest/LandmarkDetection.h pointInterest/ProHoughTransform.h pointInterest/Segmentation.h pointInterest/Treatments.h correlation/CrossCorrelation.h correlation/DescriptorDistance.h utils/Converter.h utils/Drawing.h utils/ImageConvert.h ui/ImageViewer.h MAELab.h .tmp/MAELab_CI1.0.0/ && $(COPY_FILE) --parents io/JPEGReader.cpp io/Reader.cpp io/TPSReader.cpp io/LibJpeg/jaricom.c io/LibJpeg/jcapimin.c io/LibJpeg/jcapistd.c io/LibJpeg/jcarith.c io/LibJpeg/jccoefct.c io/LibJpeg/jccolor.c io/LibJpeg/jcdctmgr.c io/LibJpeg/jchuff.c io/LibJpeg/jcinit.c io/LibJpeg/jcmainct.c io/LibJpeg/jcmarker.c io/LibJpeg/jcmaster.c io/LibJpeg/jcomapi.c io/LibJpeg/jcparam.c io/LibJpeg/jcprepct.c io/LibJpeg/jcsample.c io/LibJpeg/jctrans.c io/LibJpeg/jdapimin.c io/LibJpeg/jdapistd.c io/LibJpeg/jdarith.c io/LibJpeg/jdatadst.c io/LibJpeg/jdatasrc.c io/LibJpeg/jdcoefct.c io/LibJpeg/jdcolor.c io/LibJpeg/jddctmgr.c io/LibJpeg/jdhuff.c io/LibJpeg/jdinput.c io/LibJpeg/jdmainct.c io/LibJpeg/jdmarker.c io/LibJpeg/jdmaster.c io/LibJpeg/jdmerge.c io/LibJpeg/jdpostct.c io/LibJpeg/jdsample.c io/LibJpeg/jdtrans.c io/LibJpeg/jerror.c io/LibJpeg/jfdctflt.c io/LibJpeg/jfdctfst.c io/LibJpeg/jfdctint.c io/LibJpeg/jidctflt.c io/LibJpeg/jidctfst.c io/LibJpeg/jidctint.c io/LibJpeg/jmemmgr.c io/LibJpeg/jmemnobs.c io/LibJpeg/jquant1.c io/LibJpeg/jquant2.c io/LibJpeg/jutils.c imageModel/Edge.cpp imageModel/Image.cpp imageModel/Line.cpp imageModel/Matrix.cpp imageModel/Point.cpp segmentation/Canny.cpp segmentation/Filters.cpp segmentation/Projection.cpp segmentation/Suzuki.cpp segmentation/Thresholds.cpp histograms/LBP.cpp histograms/ShapeHistogram.cpp pht/GHTInPoint.cpp pht/PCA.cpp pht/PHoughTransform.cpp pht/PHTEntry.cpp pht/SVD.cpp pointInterest/GeometricHistgoram.cpp pointInterest/LandmarkDetection.cpp pointInterest/ProHoughTransform.cpp pointInterest/Segmentation.cpp pointInterest/Treatments.cpp correlation/CrossCorrelation.cpp correlation/DescriptorDistance.cpp utils/Converter.cpp utils/Drawing.cpp utils/ImageConvert.cpp ui/ImageViewer.cpp MAELab.cpp MAELab_Procrustes4.cpp .tmp/MAELab_CI1.0.0/ && (cd `dirname .tmp/MAELab_CI1.0.0` && $(TAR) MAELab_CI1.0.0.tar MAELab_CI1.0.0 && $(COMPRESS) MAELab_CI1.0.0.tar) && $(MOVE) `dirname .tmp/MAELab_CI1.0.0`/MAELab_CI1.0.0.tar.gz . && $(DEL_FILE) -r .tmp/MAELab_CI1.0.0


clean:compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile


check: first

mocclean: compiler_moc_header_clean compiler_moc_source_clean

mocables: compiler_moc_header_make_all compiler_moc_source_make_all

compiler_moc_header_make_all: moc_ImageViewer.cpp
compiler_moc_header_clean:
	-$(DEL_FILE) moc_ImageViewer.cpp
moc_ImageViewer.cpp: imageModel/Point.h \
		ui/ImageViewer.h
	/usr/lib/x86_64-linux-gnu/qt4/bin/moc $(DEFINES) $(INCPATH) ui/ImageViewer.h -o moc_ImageViewer.cpp

compiler_rcc_make_all:
compiler_rcc_clean:
compiler_image_collection_make_all: qmake_image_collection.cpp
compiler_image_collection_clean:
	-$(DEL_FILE) qmake_image_collection.cpp
compiler_moc_source_make_all:
compiler_moc_source_clean:
compiler_uic_make_all:
compiler_uic_clean:
compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: compiler_moc_header_clean 

####### Compile

JPEGReader.o: io/JPEGReader.cpp io/LibJpeg/jpeglib.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h \
		imageModel/Point.h \
		imageModel/Matrix.h \
		io/JPEGReader.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o JPEGReader.o io/JPEGReader.cpp

Reader.o: io/Reader.cpp imageModel/Point.h \
		imageModel/Matrix.h \
		io/TPSReader.h \
		io/JPEGReader.h \
		io/Reader.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Reader.o io/Reader.cpp

TPSReader.o: io/TPSReader.cpp imageModel/Point.h \
		io/TPSReader.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o TPSReader.o io/TPSReader.cpp

jaricom.o: io/LibJpeg/jaricom.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jaricom.o io/LibJpeg/jaricom.c

jcapimin.o: io/LibJpeg/jcapimin.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jcapimin.o io/LibJpeg/jcapimin.c

jcapistd.o: io/LibJpeg/jcapistd.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jcapistd.o io/LibJpeg/jcapistd.c

jcarith.o: io/LibJpeg/jcarith.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jcarith.o io/LibJpeg/jcarith.c

jccoefct.o: io/LibJpeg/jccoefct.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jccoefct.o io/LibJpeg/jccoefct.c

jccolor.o: io/LibJpeg/jccolor.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jccolor.o io/LibJpeg/jccolor.c

jcdctmgr.o: io/LibJpeg/jcdctmgr.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h \
		io/LibJpeg/jdct.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jcdctmgr.o io/LibJpeg/jcdctmgr.c

jchuff.o: io/LibJpeg/jchuff.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jchuff.o io/LibJpeg/jchuff.c

jcinit.o: io/LibJpeg/jcinit.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jcinit.o io/LibJpeg/jcinit.c

jcmainct.o: io/LibJpeg/jcmainct.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jcmainct.o io/LibJpeg/jcmainct.c

jcmarker.o: io/LibJpeg/jcmarker.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jcmarker.o io/LibJpeg/jcmarker.c

jcmaster.o: io/LibJpeg/jcmaster.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jcmaster.o io/LibJpeg/jcmaster.c

jcomapi.o: io/LibJpeg/jcomapi.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jcomapi.o io/LibJpeg/jcomapi.c

jcparam.o: io/LibJpeg/jcparam.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jcparam.o io/LibJpeg/jcparam.c

jcprepct.o: io/LibJpeg/jcprepct.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jcprepct.o io/LibJpeg/jcprepct.c

jcsample.o: io/LibJpeg/jcsample.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jcsample.o io/LibJpeg/jcsample.c

jctrans.o: io/LibJpeg/jctrans.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jctrans.o io/LibJpeg/jctrans.c

jdapimin.o: io/LibJpeg/jdapimin.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdapimin.o io/LibJpeg/jdapimin.c

jdapistd.o: io/LibJpeg/jdapistd.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdapistd.o io/LibJpeg/jdapistd.c

jdarith.o: io/LibJpeg/jdarith.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdarith.o io/LibJpeg/jdarith.c

jdatadst.o: io/LibJpeg/jdatadst.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdatadst.o io/LibJpeg/jdatadst.c

jdatasrc.o: io/LibJpeg/jdatasrc.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdatasrc.o io/LibJpeg/jdatasrc.c

jdcoefct.o: io/LibJpeg/jdcoefct.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdcoefct.o io/LibJpeg/jdcoefct.c

jdcolor.o: io/LibJpeg/jdcolor.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdcolor.o io/LibJpeg/jdcolor.c

jddctmgr.o: io/LibJpeg/jddctmgr.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h \
		io/LibJpeg/jdct.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jddctmgr.o io/LibJpeg/jddctmgr.c

jdhuff.o: io/LibJpeg/jdhuff.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdhuff.o io/LibJpeg/jdhuff.c

jdinput.o: io/LibJpeg/jdinput.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdinput.o io/LibJpeg/jdinput.c

jdmainct.o: io/LibJpeg/jdmainct.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdmainct.o io/LibJpeg/jdmainct.c

jdmarker.o: io/LibJpeg/jdmarker.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdmarker.o io/LibJpeg/jdmarker.c

jdmaster.o: io/LibJpeg/jdmaster.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdmaster.o io/LibJpeg/jdmaster.c

jdmerge.o: io/LibJpeg/jdmerge.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdmerge.o io/LibJpeg/jdmerge.c

jdpostct.o: io/LibJpeg/jdpostct.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdpostct.o io/LibJpeg/jdpostct.c

jdsample.o: io/LibJpeg/jdsample.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdsample.o io/LibJpeg/jdsample.c

jdtrans.o: io/LibJpeg/jdtrans.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jdtrans.o io/LibJpeg/jdtrans.c

jerror.o: io/LibJpeg/jerror.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h \
		io/LibJpeg/jversion.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jerror.o io/LibJpeg/jerror.c

jfdctflt.o: io/LibJpeg/jfdctflt.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h \
		io/LibJpeg/jdct.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jfdctflt.o io/LibJpeg/jfdctflt.c

jfdctfst.o: io/LibJpeg/jfdctfst.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h \
		io/LibJpeg/jdct.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jfdctfst.o io/LibJpeg/jfdctfst.c

jfdctint.o: io/LibJpeg/jfdctint.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h \
		io/LibJpeg/jdct.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jfdctint.o io/LibJpeg/jfdctint.c

jidctflt.o: io/LibJpeg/jidctflt.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h \
		io/LibJpeg/jdct.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jidctflt.o io/LibJpeg/jidctflt.c

jidctfst.o: io/LibJpeg/jidctfst.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h \
		io/LibJpeg/jdct.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jidctfst.o io/LibJpeg/jidctfst.c

jidctint.o: io/LibJpeg/jidctint.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h \
		io/LibJpeg/jdct.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jidctint.o io/LibJpeg/jidctint.c

jmemmgr.o: io/LibJpeg/jmemmgr.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h \
		io/LibJpeg/jmemsys.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jmemmgr.o io/LibJpeg/jmemmgr.c

jmemnobs.o: io/LibJpeg/jmemnobs.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h \
		io/LibJpeg/jmemsys.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jmemnobs.o io/LibJpeg/jmemnobs.c

jquant1.o: io/LibJpeg/jquant1.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jquant1.o io/LibJpeg/jquant1.c

jquant2.o: io/LibJpeg/jquant2.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jquant2.o io/LibJpeg/jquant2.c

jutils.o: io/LibJpeg/jutils.c io/LibJpeg/jinclude.h \
		io/LibJpeg/jconfig.h \
		io/LibJpeg/jpeglib.h \
		io/LibJpeg/jmorecfg.h \
		io/LibJpeg/jpegint.h \
		io/LibJpeg/jerror.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o jutils.o io/LibJpeg/jutils.c

Edge.o: imageModel/Edge.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Edge.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Edge.o imageModel/Edge.cpp

Image.o: imageModel/Image.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Edge.h \
		imageModel/Matrix.h \
		io/Reader.h \
		segmentation/Thresholds.h \
		segmentation/Canny.h \
		segmentation/Suzuki.h \
		imageModel/Image.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Image.o imageModel/Image.cpp

Line.o: imageModel/Line.cpp imageModel/Point.h \
		imageModel/Line.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Line.o imageModel/Line.cpp

Matrix.o: imageModel/Matrix.cpp imageModel/Point.h \
		imageModel/Matrix.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Matrix.o imageModel/Matrix.cpp

Point.o: imageModel/Point.cpp imageModel/Point.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Point.o imageModel/Point.cpp

Canny.o: segmentation/Canny.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Edge.h \
		imageModel/Matrix.h \
		io/Reader.h \
		utils/Converter.h \
		segmentation/Filters.h \
		segmentation/Thresholds.h \
		segmentation/Canny.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Canny.o segmentation/Canny.cpp

Filters.o: segmentation/Filters.cpp imageModel/Point.h \
		imageModel/Matrix.h \
		segmentation/Canny.h \
		segmentation/Filters.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Filters.o segmentation/Filters.cpp

Projection.o: segmentation/Projection.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Matrix.h \
		io/Reader.h \
		segmentation/Projection.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Projection.o segmentation/Projection.cpp

Suzuki.o: segmentation/Suzuki.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Edge.h \
		imageModel/Matrix.h \
		io/Reader.h \
		segmentation/Thresholds.h \
		segmentation/Suzuki.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Suzuki.o segmentation/Suzuki.cpp

Thresholds.o: segmentation/Thresholds.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Edge.h \
		imageModel/Matrix.h \
		segmentation/Filters.h \
		segmentation/Thresholds.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Thresholds.o segmentation/Thresholds.cpp

LBP.o: histograms/LBP.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		imageModel/Edge.h \
		histograms/LBP.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o LBP.o histograms/LBP.cpp

ShapeHistogram.o: histograms/ShapeHistogram.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		imageModel/Edge.h \
		histograms/ShapeHistogram.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o ShapeHistogram.o histograms/ShapeHistogram.cpp

GHTInPoint.o: pht/GHTInPoint.cpp utils/Converter.h \
		imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		imageModel/Edge.h \
		segmentation/Thresholds.h \
		segmentation/Canny.h \
		pht/PCA.h \
		pht/GHTInPoint.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o GHTInPoint.o pht/GHTInPoint.cpp

PCA.o: pht/PCA.cpp utils/Converter.h \
		imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		imageModel/Edge.h \
		segmentation/Thresholds.h \
		segmentation/Canny.h \
		correlation/CrossCorrelation.h \
		correlation/DescriptorDistance.h \
		utils/Drawing.h \
		io/Reader.h \
		pht/GHTInPoint.h \
		pht/PCA.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o PCA.o pht/PCA.cpp

PHoughTransform.o: pht/PHoughTransform.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		imageModel/Edge.h \
		pht/PHTEntry.h \
		pht/PHoughTransform.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o PHoughTransform.o pht/PHoughTransform.cpp

PHTEntry.o: pht/PHTEntry.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Matrix.h \
		pht/PHTEntry.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o PHTEntry.o pht/PHTEntry.cpp

SVD.o: pht/SVD.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Edge.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		pht/GHTInPoint.h \
		io/Reader.h \
		segmentation/Thresholds.h \
		pht/SVD.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o SVD.o pht/SVD.cpp

GeometricHistgoram.o: pointInterest/GeometricHistgoram.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Edge.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		histograms/ShapeHistogram.h \
		pointInterest/Treatments.h \
		pointInterest/GeometricHistgoram.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o GeometricHistgoram.o pointInterest/GeometricHistgoram.cpp

LandmarkDetection.o: pointInterest/LandmarkDetection.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Edge.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		segmentation/Canny.h \
		segmentation/Thresholds.h \
		histograms/ShapeHistogram.h \
		pht/PHTEntry.h \
		pht/PHoughTransform.h \
		pht/GHTInPoint.h \
		correlation/CrossCorrelation.h \
		correlation/DescriptorDistance.h \
		utils/Drawing.h \
		io/Reader.h \
		pointInterest/Treatments.h \
		pointInterest/Segmentation.h \
		pointInterest/GeometricHistgoram.h \
		pointInterest/ProHoughTransform.h \
		pointInterest/LandmarkDetection.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o LandmarkDetection.o pointInterest/LandmarkDetection.cpp

ProHoughTransform.o: pointInterest/ProHoughTransform.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Edge.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		segmentation/Canny.h \
		segmentation/Thresholds.h \
		pht/PHTEntry.h \
		pht/PHoughTransform.h \
		pht/GHTInPoint.h \
		io/Reader.h \
		utils/Drawing.h \
		pht/PCA.h \
		correlation/DescriptorDistance.h \
		pointInterest/Treatments.h \
		pointInterest/ProHoughTransform.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o ProHoughTransform.o pointInterest/ProHoughTransform.cpp

Segmentation.o: pointInterest/Segmentation.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Edge.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		segmentation/Thresholds.h \
		segmentation/Canny.h \
		segmentation/Suzuki.h \
		histograms/ShapeHistogram.h \
		io/Reader.h \
		segmentation/Filters.h \
		pointInterest/Treatments.h \
		pointInterest/Segmentation.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Segmentation.o pointInterest/Segmentation.cpp

Treatments.o: pointInterest/Treatments.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		imageModel/Edge.h \
		histograms/ShapeHistogram.h \
		pointInterest/Treatments.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Treatments.o pointInterest/Treatments.cpp

CrossCorrelation.o: correlation/CrossCorrelation.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		imageModel/Edge.h \
		correlation/CrossCorrelation.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o CrossCorrelation.o correlation/CrossCorrelation.cpp

DescriptorDistance.o: correlation/DescriptorDistance.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Matrix.h \
		segmentation/Filters.h \
		correlation/DescriptorDistance.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o DescriptorDistance.o correlation/DescriptorDistance.cpp

Converter.o: utils/Converter.cpp utils/Converter.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Converter.o utils/Converter.cpp

Drawing.o: utils/Drawing.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Matrix.h \
		utils/Drawing.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Drawing.o utils/Drawing.cpp

ImageConvert.o: utils/ImageConvert.cpp imageModel/Point.h \
		imageModel/Matrix.h \
		utils/ImageConvert.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o ImageConvert.o utils/ImageConvert.cpp

ImageViewer.o: ui/ImageViewer.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Edge.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		io/Reader.h \
		segmentation/Thresholds.h \
		segmentation/Canny.h \
		segmentation/Suzuki.h \
		segmentation/Projection.h \
		segmentation/Filters.h \
		pht/GHTInPoint.h \
		pht/PHTEntry.h \
		pht/PHoughTransform.h \
		pht/PCA.h \
		pht/SVD.h \
		histograms/ShapeHistogram.h \
		histograms/LBP.h \
		pointInterest/Treatments.h \
		pointInterest/Segmentation.h \
		pointInterest/ProHoughTransform.h \
		pointInterest/LandmarkDetection.h \
		utils/ImageConvert.h \
		utils/Drawing.h \
		utils/Converter.h \
		correlation/CrossCorrelation.h \
		correlation/DescriptorDistance.h \
		MAELab.h \
		ui/ImageViewer.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o ImageViewer.o ui/ImageViewer.cpp

MAELab.o: MAELab.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Edge.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		histograms/ShapeHistogram.h \
		pht/PHTEntry.h \
		pht/PHoughTransform.h \
		correlation/CrossCorrelation.h \
		segmentation/Thresholds.h \
		segmentation/Canny.h \
		segmentation/Suzuki.h \
		segmentation/Filters.h \
		pointInterest/Treatments.h \
		pointInterest/Segmentation.h \
		pointInterest/GeometricHistgoram.h \
		pointInterest/ProHoughTransform.h \
		pointInterest/LandmarkDetection.h \
		MAELab.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o MAELab.o MAELab.cpp

MAELab_Procrustes4.o: MAELab_Procrustes4.cpp imageModel/Point.h \
		imageModel/Line.h \
		imageModel/Edge.h \
		imageModel/Matrix.h \
		imageModel/Image.h \
		utils/Drawing.h \
		io/Reader.h \
		histograms/ShapeHistogram.h \
		pht/PHTEntry.h \
		pht/PHoughTransform.h \
		correlation/CrossCorrelation.h \
		segmentation/Thresholds.h \
		segmentation/Canny.h \
		segmentation/Suzuki.h \
		segmentation/Filters.h \
		pointInterest/Treatments.h \
		pointInterest/Segmentation.h \
		pointInterest/GeometricHistgoram.h \
		pointInterest/ProHoughTransform.h \
		pointInterest/LandmarkDetection.h \
		MAELab.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o MAELab_Procrustes4.o MAELab_Procrustes4.cpp

moc_ImageViewer.o: moc_ImageViewer.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_ImageViewer.o moc_ImageViewer.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:

