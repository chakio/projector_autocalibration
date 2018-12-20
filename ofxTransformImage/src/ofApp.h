#pragma once

#include "ofMain.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <sstream>
#include <string>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

#include <std_msgs/Float32MultiArray.h>
#include "ofxOpenCv.h"
using namespace std;
class ofApp : public ofBaseApp{

	public:
		ofApp(int argc, char *argv[]);
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
	
		void getVertex(const std_msgs::Float32MultiArray::ConstPtr &msg);
		

	   	ros::NodeHandle n;
		ros::Subscriber _sub_vertex;
		ofPoint v[4],s[4];

		ofxCvColorImage cimColor;
		ofxCvColorImage cimColor2;
		ofImage imgSample;
		bool fullscreen;
		bool calibrate;

};
