#pragma once

#include "ofMain.h"

#pragma managed(push, off)
#include "btBulletDynamicsCommon.h"
#pragma managed(pop)

#include "ofxBtRigidBody.h"
#include "ofxBtJoint.h"

namespace ofxBt
{
	class World;
}

class ofxBt::World
{
public:
	
	struct CollisionEventArg
	{
		btRigidBody *rigid0;
		btRigidBody *rigid1;
	};
	
	ofEvent<CollisionEventArg> collisionEvent;
	
	World();
	virtual ~World();
	
	void setup(ofVec3f gravity = ofVec3f(0, -980, 0), float world_scale = 100);
	virtual void update();
	void draw();
	
	void clear();
	
	void setGravity(ofVec3f gravity);
	
	inline float getWorldScale() { return world_scale; }
	
	static bool ContactProcessedCallback(btManifoldPoint& manifold, void* object0, void* object1);
	static bool ContactAddedCallback(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1);
	
	//
	
	RigidBody addBox(const ofVec3f& size, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	RigidBody addSphere(const float size, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	RigidBody addCylinder(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	RigidBody addCapsule(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	RigidBody addCone(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	RigidBody addPlane(const ofVec3f& up, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	RigidBody addMesh(const ofMesh &mesh, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	RigidBody addStaticMesh(const ofMesh &mesh, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	vector<RigidBody> addWorldBox(const ofVec3f &leftTopFar, const ofVec3f& rightBottomNear);
	
	btRigidBody* addRigidBody(btCollisionShape* shape, const ofVec3f& pos, const ofVec3f& rot, float mass = 1);
	void removeRigidBody(btRigidBody* body);
	
	//
	
	

protected:
	
	btBroadphaseInterface *m_broadphase;
	btCollisionConfiguration *m_collisionConfiguration;
	btCollisionDispatcher *m_dispatcher;
	btSequentialImpulseConstraintSolver *m_solver;
	btDynamicsWorld *m_dynamicsWorld;
	
	virtual btBroadphaseInterface* createBroadphase();
	virtual btCollisionConfiguration* createCollisionConfiguration();
	virtual btDiscreteDynamicsWorld* createDynamicsWorld();
	
	btDiscreteDynamicsWorld* getDynamicsWorld();
	
	vector<btRigidBody*> rigidBodies;
	
	inline float getMargin() { return 0.04 * world_scale; }
	
private:
	
	float world_scale;
};