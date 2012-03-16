#pragma once

#include "ofMain.h"

#include <btBulletDynamicsCommon.h>
#include "ofxBtHelper.h"

namespace ofxBt {

#pragma mark -	

class Rigid
{
public:
	
	Rigid() : body(NULL) {}
	Rigid(btRigidBody *body) : body(body) {}
	~Rigid() { body = NULL; }
	
	inline Rigid(const Rigid& copy)
	{
		body = copy.body;
	}
	
	inline Rigid& operator=(const Rigid& copy)
	{
		body = copy.body;
		return *this;
	}
	
	inline void clear() { body = NULL; }
	
	inline operator bool() const { return body != NULL; }
	
	inline btRigidBody* operator *() const { return body; }
	inline btRigidBody* get() const { return body; }
	
	inline Rigid& setProperty(float rest, float frict, float lin_damping, float ang_damping)
	{
		setRestitution(rest);
		setFriction(frict);
		setDamping(lin_damping, ang_damping);
		return *this;
	}
	
	inline Rigid& setRestitution(float rest)
	{
		body->setRestitution(rest);
		return *this;
	}
	
	inline Rigid& setFriction(float frict)
	{
		body->setFriction(frict);
		return *this;
	}
	
	inline Rigid& setDamping(float lin_damping, float ang_damping)
	{
		body->setDamping(lin_damping, ang_damping);
		return *this;
	}
	
	inline ofMatrix4x4 getTransform() const
	{
		btTransform worldTrans;
		body->getMotionState()->getWorldTransform(worldTrans);
		return toOF(worldTrans);
	}
	
	inline ofVec3f getPosition() const
	{
		return getTransform().getTranslation();
	}
	
	inline ofQuaternion getRotate() const
	{
		return getTransform().getRotate();
	}
	
	inline ofVec3f getRotateEuler() const
	{
		return getRotate().getEuler();
	}
	
	ofVec3f getSize() const
	{
		btCollisionShape *shape = body->getCollisionShape();
		int type = shape->getShapeType();
		
		btConvexInternalShape* s = dynamic_cast<btConvexInternalShape*>(shape);
		if (s)
			return toOF(s->getImplicitShapeDimensions());
		else
			ofLogWarning("ofxBt::getSize", "unimplemented shape type");
		
		return ofVec3f(0, 0, 0);
	}
	
	void setSize(const ofVec3f& size)
	{
		btCollisionShape *shape = body->getCollisionShape();
		int type = shape->getShapeType();
		
		btConvexInternalShape* s = dynamic_cast<btConvexInternalShape*>(shape);
		if (s)
		{
			s->setImplicitShapeDimensions(toBt(size));
		}
		else
			ofLogWarning("ofxBt::setSize", "unimplemented shape type");
	}
	
	inline Rigid& activate()
	{
		body->activate(true);
		return *this;
	}
	
	inline Rigid& applyForce(const ofVec3f& force, const ofVec3f& pos = ofVec3f(0, 0, 0))
	{
		activate();
		
		body->applyForce(toBt(force), toBt(pos));
		return *this;
	}

	inline Rigid& applyCentralForce(const ofVec3f& force)
	{
		activate();
		
		body->applyCentralForce(toBt(force));
		return *this;
	}

	inline Rigid& applyTorque(const ofVec3f& force)
	{
		activate();
		
		body->applyTorque(toBt(force));
		return *this;
	}
	
	inline Rigid& setKinematic()
	{
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	}
	
	inline bool isKinematic() const
	{
		return (body->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT);
	}
	
protected:
	
	btRigidBody *body;
	
};
	
#pragma mark -

class World
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
	
	btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
	
	vector<btRigidBody*> rigidBodies;
	
	btRigidBody* createRigidBody(btCollisionShape* shape, float mass, const ofVec3f& pos, const ofVec3f& rot);
	void disposeRigidBody(btRigidBody* body);
	
	class	btThreadSupportInterface*		m_threadSupportCollision;
	class	btThreadSupportInterface*		m_threadSupportSolver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	
};
	
}
