#include "testApp.h"

#include "ofxBt.h"

ofxBt::SoftbodyWorld world;

btSoftBody *o;

//--------------------------------------------------------------
void testApp::setup()
{
	ofSetFrameRate(60);
	ofSetVerticalSync(true);

	ofBackground(30);

	world.setup(ofVec3f(0, -98, 0));

	world.addPlane(ofVec3f(0, 1, 0), ofVec3f(0, 0, 0));
	// world.addWorldBox(ofVec3f(-200, -200, -200), ofVec3f(200, 200, 200));
	
	o = world.addRope(ofVec3f(-100, 200, 0), ofVec3f(100, 200, 0), 10, 4);
}

//--------------------------------------------------------------
void testApp::update()
{
	world.update();

}

//--------------------------------------------------------------
void testApp::draw()
{
	cam.begin();

	world.draw();

	cam.end();
}

//--------------------------------------------------------------
void testApp::keyPressed(int key)
{
}

//--------------------------------------------------------------
void testApp::keyReleased(int key)
{

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y)
{

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg)
{

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo)
{

}