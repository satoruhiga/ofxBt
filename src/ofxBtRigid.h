#pragma once

#include "ofMain.h"

#include "btBulletDynamicsCommon.h"

#include "ofxBtHelper.h"

namespace ofxBt {
	class Rigid;
}

class ofxBt::Rigid
{
public:
	
	// TODO: setUserPointer / getUserPointer
	
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
	
	inline Rigid& setMass(float mass)
	{
		btVector3 inertia(0, 0, 0);
		body->getCollisionShape()->calculateLocalInertia(mass, inertia);
		body->setMassProps(mass, inertia);
		body->updateInertiaTensor();
		return *this;
	}
	
	inline float getMass()
	{
		return 1. / body->getInvMass();
	}
	
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
		{
			return toOF(s->getImplicitShapeDimensions());
		}
		else
			ofLogWarning("ofxBt::getSize") << "unimplemented shape type";
		
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
			ofLogWarning("ofxBt::setSize") << "unimplemented shape type";
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
	
	inline Rigid& setKinematic(bool v)
	{
		if (v)
		{
			body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			body->setActivationState(DISABLE_DEACTIVATION);
		}
		else
		{
			body->setCollisionFlags(body->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
			body->setActivationState(DISABLE_DEACTIVATION);
		}
		return *this;
	}
	
	inline bool isKinematic() const
	{
		return (body->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT);
	}
	
protected:
	
	btRigidBody *body;
	
};
