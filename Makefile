all: shape_detect.cpp 
#	g++ opencv-kalman.cpp -o asl -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videoio -lopencv_videostab
	g++ --pedantic -pg shape_detect.cpp -o shape_detect -lopencv_imgcodecs -lopencv_shape -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videoio -lopencv_videostab -lraspicam -lraspicam_cv -lrt -lpthread -lFestival -I/usr/include/festival  -I/usr/lib/speech_tools/include -leststring -lestools -lestbase	
clean:
	rm -rf shape_detect
