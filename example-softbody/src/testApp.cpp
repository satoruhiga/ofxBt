#include "testApp.h"

#include "ofxBt.h"

ofxBt::SoftbodyWorld world;

ofxBt::Soft rope;

ofxBt::Rigid box;

//--------------------------------------------------------------
void testApp::setup()
{
	ofSetFrameRate(60);
	ofSetVerticalSync(true);

	ofBackground(30);

	world.setup(ofVec3f(0, -980, 0));

	world.addPlane(ofVec3f(0, 1, 0), ofVec3f(0, 0, 0));
	
	box = world.addBox(ofVec3f(100, 100, 100), ofVec3f(0, 300, 0));
	box.setMass(5.5);
	
	rope = world.addRope(ofVec3f(0, 500, 0), ofVec3f(50, 300, 50), 10);
	rope.setMass(7);
	rope.setFixedAt(0);
	rope.attachRigidBodyAt(rope.getNumNode()-1, box);
	rope.setStiffness(1, 1, 1);
	
	ofxBt::Soft o = world.addEllipsoid(ofVec3f(0, 800, 0), ofVec3f(150, 150, 150), 300);
	o.setMass(5, true);
	o.setStiffness(1, 1, 0.1);
	o.setRigidContactsHrdness(1);
}

//--------------------------------------------------------------
void testApp::update()
{
	world.update();
	
	rope.setNodePositionAt(0, ofVec3f(ofGetMouseX(), ofGetMouseY(), 0));
}

//--------------------------------------------------------------
void testApp::draw()
{
	cam.begin();

	glTranslated(0, -200, 0);
	
	glRotatef(ofGetElapsedTimef() * 30, 0, 1, 0);
	
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