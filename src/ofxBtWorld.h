#pragma once

#include "ofMain.h"

#include "btBulletDynamicsCommon.h"

namespace ofxBt
{
	class World;
}

class ofxBt::World
{
public:
	
	World();
	~World();
	
	void clear();
	
	void setup(ofVec3f gravity = ofVec3f(0, 9.8, 0));
	void update();
	void draw();
	
	btRigidBody* addBox(const ofVec3f& size, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0), float mass = 1);
	btRigidBody* addSphere(const float size, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0), float mass = 1);
	btRigidBody* addCylinder(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0), float mass = 1);
	btRigidBody* addCapsule(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0), float mass = 1);
	btRigidBody* addCone(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0), float mass = 1);
	btRigidBody* addPlane(const ofVec3f& up, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0), float mass = 0);
	vector<btRigidBody*> addWorldBox(const ofVec3f &leftTopFar, const ofVec3f& rightBottomNear);
	
	void removeRegidBody(btRigidBody *body);
	
	void setGravity(ofVec3f gravity);
	
protected:
	
	bool inited;
	
	btBroadphaseInterface *m_broadphase;
	btCollisionConfiguration *m_collisionConfiguration;
	btCollisionDispatcher *m_dispatcher;
	btSequentialImpulseConstraintSolver *m_solver;
	btDiscreteDynamicsWorld *m_dynamicsWorld;
	
	vector<btRigidBody*> rigidBodies;
	
	btRigidBody* createRigidBody(btCollisionShape* shape, float mass, const ofVec3f& pos, const ofVec3f& rot);
	void disposeRigidBody(btRigidBody* body);
	
	//	class btThreadSupportInterface *m_threadSupportCollision;
	//	class btThreadSupportInterface *m_threadSupportSolver;
	
	virtual btCollisionConfiguration* createCollisionConfiguration();
	virtual btDiscreteDynamicsWorld* createDynamicsWorld();
};