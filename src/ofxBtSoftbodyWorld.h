#pragma once

#include "ofxBtWorld.h"

#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

namespace ofxBt
{
	class SoftbodyWorld;
};

class ofxBt::SoftbodyWorld : public ofxBt::World
{
public:
	
	void setup(ofVec3f gravity = ofVec3f(0, -98, 0));
	void setGravity(ofVec3f gravity);
	
	btSoftBody* addRope(const ofVec3f& from, const ofVec3f& to, int res, int fixeds);
	
protected:
	
	btSoftBodyWorldInfo	m_softBodyWorldInfo;
	
	vector<btSoftBody*> softBodies;
	
	btCollisionConfiguration* createCollisionConfiguration();
	btDiscreteDynamicsWorld* createDynamicsWorld();
	
	btSoftRigidDynamicsWorld* getDynamicsWorld();
	
	btSoftBody* setupSoftBody(btSoftBody *body);
};
