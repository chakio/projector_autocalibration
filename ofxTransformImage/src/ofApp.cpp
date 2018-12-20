#include "ofApp.h"

//--------------------------------------------------------------
ofApp::ofApp(int argc, char *argv[])
{
	_sub_vertex  = n.subscribe("/vertex", 5, &ofApp::getVertex, this);
	
}
void ofApp::setup(){
    ofSetBackgroundColor(0);
    imgSample.loadImage("images.jpeg");
    cimColor.allocate(225, 224);
    cimColor.setFromPixels(imgSample.getPixelsRef());
    s[0]=ofPoint(0,0);
    s[1]=ofPoint(224,0);
    s[2]=ofPoint(224,224);
    s[3]=ofPoint(0,224);
    v[0]=ofPoint(0,0);
    v[1]=ofPoint(50,0);
    v[2]=ofPoint(1000,50);
    v[3]=ofPoint(0,50);

    cimColor2.allocate(ofGetWidth(), ofGetHeight());
    fullscreen=false;
    calibrate=false;
}

//--------------------------------------------------------------
void ofApp::update(){
    ros::Rate r(60);
    
	if(ros::ok())
	{
        cimColor2.warpIntoMe(cimColor,s,v);
	}
	else
	{
		abort();
	}
	r.sleep();
	ros::spinOnce();

}
//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255,255,255);
    if(calibrate)
    {
        cimColor2.draw(ofPoint(0,0));
    }
    else
    {
        //アフィン変換
        cimColor.draw(ofPoint(ofGetWidth()/2-112,ofGetHeight()-336));
    }
}

void ofApp::getVertex(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	v[0].x=msg->data[0]*1000+ofGetWidth()/2;
    v[0].y=msg->data[1]*1000+ofGetHeight()-112;
    v[1].x=msg->data[2]*1000+ofGetWidth()/2;
    v[1].y=msg->data[3]*1000+ofGetHeight()-112;
    v[2].x=msg->data[4]*1000+ofGetWidth()/2;
    v[2].y=msg->data[5]*1000+ofGetHeight()-112;
    v[3].x=msg->data[6]*1000+ofGetWidth()/2;
    v[3].y=msg->data[7]*1000+ofGetHeight()-112;

    if(v[0].x>v[1].x)
    {
        ofPoint temp;
        temp=v[0];
        v[0]=v[1];
        v[1]=temp;
    }
    if(v[2].x<v[3].x)
    {
        ofPoint temp;
        temp=v[2];
        v[2]=v[3];
        v[3]=temp;
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key='f')
    {
        if(fullscreen)
        {
            fullscreen=false;
            ofSetFullscreen(false);
        }
        else
        {
            fullscreen=true;
            ofSetFullscreen(true);
        }
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    if(calibrate)
    {
        calibrate=false;
    }
    else
    {
        calibrate=true;
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------

