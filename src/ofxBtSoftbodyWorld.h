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
	
	void setup(ofVec3f gravity = ofVec3f(0, -980, 0), float world_scale = 100);
	virtual void update();
	
	void setGravity(ofVec3f gravity);
	
	btSoftBody* addRope(const ofVec3f& from, const ofVec3f& to, int res = 10);
	btSoftBody* addPatch(const ofVec3f& v0, const ofVec3f& v1, const ofVec3f& v2, const ofVec3f& v3, int resx = 10, int resy = 10);
	btSoftBody* addEllipsoid(const ofVec3f& center, const ofVec3f& radius, int res = 10);

	void removeSoftBody(btSoftBody *body);
	
protected:
	
	btSoftBodyWorldInfo	m_softBodyWorldInfo;
	
	vector<btSoftBody*> softBodies;
	
	btCollisionConfiguration* createCollisionConfiguration();
	btDiscreteDynamicsWorld* createDynamicsWorld();
	
	btSoftRigidDynamicsWorld* getDynamicsWorld();
	
	btSoftBody* setupSoftBody(btSoftBody *body);
	void disposeSoftBody(btSoftBody *body);
};
