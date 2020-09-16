all:
	#g++ main.cpp featureExtractor.cpp -o main -L/usr/lib -lopencv_core -lopencv_highgui -lopencv_videoio 
	#-lopencv_imgcodecs -lopencv_imgproc -lopencv_features2d -lopencv_types
	g++ main.cpp featureExtractor.cpp -o main `pkg-config --cflags --libs opencv` -O3

clean:
	rm -rf main