all: main.cpp
	g++ -g `pkg-config --cflags gtk+-3.0 gmodule-2.0 opencv` main.cpp -o main `pkg-config --libs gtk+-3.0 gmodule-export-2.0 opencv`
	g++ -g `pkg-config --cflags gtk+-3.0 gmodule-2.0 opencv` main_test_undistort.cpp -o main_test_undistort `pkg-config --libs gtk+-3.0 gmodule-export-2.0 opencv`
