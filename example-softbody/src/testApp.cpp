#include "testApp.h"

#include "ofxBt.h"

ofxBt::SoftbodyWorld world;

ofxBt::Soft o;

ofxBt::Rigid sp;

//--------------------------------------------------------------
void testApp::setup()
{
	ofSetFrameRate(60);
	ofSetVerticalSync(true);

	ofBackground(30);

	world.setup(ofVec3f(0, -980, 0));

	ofxBt::Rigid pp = world.addPlane(ofVec3f(0, 1, 0), ofVec3f(0, 0, 0));
	// world.addWorldBox(ofVec3f(-2000, 0, -2000), ofVec3f(2000, 2000, 2000));
	
	// sp = world.addSphere(50, ofVec3f(0, 300, 0));
//	sp = world.addBox(ofVec3f(50, 50, 50), ofVec3f(0, 300, 0));
//	sp.setMass(0.5);
	
//	o = world.addRope(ofVec3f(0, 250, 0), ofVec3f(25, 300, 25), 10);
//	o.setMass(7);
//	o.setFixed(0);
//	o.attachRigidBody(o.getNumNode()-1, sp);
//	o.setStiffness(1, 1, 1);
	
//	o = world.addPatch(ofVec3f(-200, 100, -200), ofVec3f(-200, 100, 200),
//					   ofVec3f(200, 100, -200), ofVec3f(200, 100, 200), 10, 10);
	
	o = world.addEllipsoid(ofVec3f(0, 300, 0), ofVec3f(50, 50, 50), 100);
	o.setMass(50, true);
	o.setSolverIterations(4);
	o.setPoseMatching(0.1);
	o->generateBendingConstraints(2);
	o->randomizeConstraints();
}

//--------------------------------------------------------------
void testApp::update()
{
	world.update();
	
//	o.setNodePositionAt(0, ofVec3f(ofGetMouseX(), ofGetMouseY(), 0));
}

//--------------------------------------------------------------
void testApp::draw()
{
	cam.begin();

	world.draw();
	
	ofNoFill();
	ofBox(0, 50, 0, 100);

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