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
	
	struct CollisionEventArg
	{
		btRigidBody *rigid0;
		btRigidBody *rigid1;
	};
	
	ofEvent<CollisionEventArg> collisionEvent;
	
	World();
	~World();
	
	void setup(ofVec3f gravity = ofVec3f(0, 98, 0), float world_scale = 100);
	virtual void update();
	void draw();
	
	void clear();
	
	void setGravity(ofVec3f gravity);
	
	btRigidBody* addBox(const ofVec3f& size, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	btRigidBody* addSphere(const float size, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	btRigidBody* addCylinder(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	btRigidBody* addCapsule(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	btRigidBody* addCone(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	btRigidBody* addPlane(const ofVec3f& up, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	btRigidBody* addMesh(const ofMesh &mesh, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	btRigidBody* addStaticMesh(const ofMesh &mesh, const ofVec3f& pos, const ofVec3f& rot = ofVec3f(0, 0, 0));
	vector<btRigidBody*> addWorldBox(const ofVec3f &leftTopFar, const ofVec3f& rightBottomNear);
	
	btRigidBody* setupRigidBody(btCollisionShape* shape, const ofVec3f& pos, const ofVec3f& rot, float mass = 1);
	void disposeRigidBody(btRigidBody* body);
	
	inline float getWorldScale() { return world_scale; }
	
	static bool ContactProcessedCallback(btManifoldPoint& manifold, void* object0, void* object1);
	static bool ContactAddedCallback(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1);
	
protected:
	
	btBroadphaseInterface *m_broadphase;
	btCollisionConfiguration *m_collisionConfiguration;
	btCollisionDispatcher *m_dispatcher;
	btSequentialImpulseConstraintSolver *m_solver;
	btDynamicsWorld *m_dynamicsWorld;
	
	vector<btRigidBody*> rigidBodies;
	
	virtual btBroadphaseInterface* createBroadphase();
	virtual btCollisionConfiguration* createCollisionConfiguration();
	virtual btDiscreteDynamicsWorld* createDynamicsWorld();
	
	btDiscreteDynamicsWorld* getDynamicsWorld();
	
	inline float getMargin() { return 0.04 * world_scale; }
	
private:
	
	float world_scale;
};