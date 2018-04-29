all: shape_detect.cpp 
#	g++ opencv-kalman.cpp -o asl -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videoio -lopencv_videostab
	g++ --pedantic -pg -g shape_detect_profiling.cpp -o shape_detect_profiling -lopencv_imgcodecs -lopencv_shape -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videoio -lopencv_videostab -lraspicam -lraspicam_cv -lrt -lpthread -lFestival -I/usr/include/festival  -I/usr/lib/speech_tools/include -leststring -lestools -lestbase -lwiringPi	

shape_detect:shape_detect.cpp
		g++-7.2.0 --pedantic -pg -g shape_detect.cpp -o shape_detect -lopencv_imgcodecs -lopencv_shape -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videoio -lopencv_videostab -lraspicam -lraspicam_cv -lrt -lpthread -lFestival -I/usr/include/festival  -I/usr/lib/speech_tools/include -leststring -lestools -lestbase -lwiringPi	

shape_detect_profiling:shape_detect_profiling.cpp
		g++ --pedantic -pg -g shape_detect_profiling.cpp profiling_functions.cpp -o shape_detect_profiling -lopencv_imgcodecs -lopencv_shape -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videoio -lopencv_videostab -lraspicam -lraspicam_cv -lrt -lpthread -lFestival -I/usr/include/festival  -I/usr/lib/speech_tools/include -leststring -lestools -lestbase -lwiringPi	> /dev/null

clean:
	rm -rf shape_detect_profiling shape_detect
