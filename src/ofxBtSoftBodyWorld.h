#pragma once

#include "ofxBtWorld.h"

#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "ofxBtSoftBody.h"

namespace ofxBt
{
	class SoftBodyWorld;
};

class ofxBt::SoftBodyWorld : public ofxBt::World
{
public:
	
	void setup(ofVec3f gravity = ofVec3f(0, -980, 0), float world_scale = 100);
	virtual void update();
	
	void clear();
	
	void setGravity(ofVec3f gravity);
	
	SoftBody addRope(const ofVec3f& from, const ofVec3f& to, int res = 10);
	SoftBody addPatch(const ofVec3f& v0, const ofVec3f& v1, const ofVec3f& v2, const ofVec3f& v3, int resx = 10, int resy = 10);
	SoftBody addEllipsoid(const ofVec3f& center, const ofVec3f& radius, int res = 10);

	btSoftBody* setupSoftBody(btSoftBody *body);
	void disposeSoftBody(btSoftBody *body);

protected:
	
	btSoftBodyWorldInfo	m_softBodyWorldInfo;
	
	btBroadphaseInterface* createBroadphase();
	btCollisionConfiguration* createCollisionConfiguration();
	btDiscreteDynamicsWorld* createDynamicsWorld();
	
	btSoftRigidDynamicsWorld* getDynamicsWorld();
	
	vector<btSoftBody*> softBodies;
	
};
