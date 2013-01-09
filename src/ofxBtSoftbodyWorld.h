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
	
	void setup(ofVec3f gravity = ofVec3f(0, 9.8, 0));
	void setGravity(ofVec3f gravity);
	
protected:
	
	btSoftBodyWorldInfo	m_softBodyWorldInfo;
	
	btCollisionConfiguration* createCollisionConfiguration();
	btDiscreteDynamicsWorld* createDynamicsWorld();
};
