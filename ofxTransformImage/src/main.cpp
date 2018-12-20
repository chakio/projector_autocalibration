#include "ofMain.h"
#include "ofApp.h"
#include <ros/ros.h>

//========================================================================

int main(int argc, char *argv[]){
    ros::init(argc, argv, "ros_tutorial_msg_publisher");  // ノード名の初期化

	ofSetupOpenGL(1024,768,OF_WINDOW);			// <-------- setup the GL context

    
	ofRunApp(new ofApp(argc, argv));

}
