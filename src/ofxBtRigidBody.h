#pragma once

#include "ofMain.h"

#pragma managed(push, off)
#include "btBulletDynamicsCommon.h"
#pragma managed(pop)

#include "ofxBtHelper.h"
#include "ofxBtCollisionObject.h"

namespace ofxBt
{
	class RigidBody;
}

class ofxBt::RigidBody : public ofxBt::CollisionObject
{
public:
	
	RigidBody() {}
	RigidBody(btRigidBody *body) : CollisionObject(body) {}
	
	inline operator btRigidBody*() const { return body(); }
	inline btRigidBody* operator->() const { return body(); }
	
	inline RigidBody& setMass(float mass)
	{
		btVector3 inertia(0, 0, 0);
		body()->getCollisionShape()->calculateLocalInertia(mass, inertia);
		body()->setMassProps(mass, inertia);
		body()->updateInertiaTensor();
		return *this;
	}
	
	inline float getMass()
	{
		return 1. / body()->getInvMass();
	}
	
	//
	
	inline void setProperty(float rest, float frict, float lin_damping, float ang_damping)
	{
		setRestitution(rest);
		setFriction(frict);
		setDamping(lin_damping, ang_damping);
	}

	inline void setDamping(float lin_damping, float ang_damping)
	{
		body()->setDamping(lin_damping, ang_damping);
	}
	
	//
	
	ofVec3f getSize() const
	{
		btConvexInternalShape* s = dynamic_cast<btConvexInternalShape*>(shape());
		
		if (s)
		{
			return toOF(s->getImplicitShapeDimensions());
		}
		else
			ofLogWarning("ofxBt::getSize") << "unimplemented shape type";
		
		return ofVec3f(0, 0, 0);
	}
	
	void setSize(const ofVec3f& size)
	{
		btConvexInternalShape* s = dynamic_cast<btConvexInternalShape*>(shape());
		
		if (s)
		{
			s->setImplicitShapeDimensions(toBt(size));
		}
		else
			ofLogWarning("ofxBt::setSize") << "unimplemented shape type";
	}
	
	//
	
	inline RigidBody& applyForce(const ofVec3f& force, const ofVec3f& pos = ofVec3f(0, 0, 0))
	{
		activate();
		
		body()->applyForce(toBt(force), toBt(pos));
		return *this;
	}
	
	inline RigidBody& applyCentralForce(const ofVec3f& force)
	{
		activate();
		
		body()->applyCentralForce(toBt(force));
		return *this;
	}
	
	inline RigidBody& applyTorque(const ofVec3f& force)
	{
		activate();
		
		body()->applyTorque(toBt(force));
		return *this;
	}
	
protected:
	
	inline btRigidBody* body() { return (btRigidBody*)object; }
	inline btRigidBody* body() const { return (btRigidBody*)object; }
	
};
