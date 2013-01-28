#pragma once

#include "btBulletDynamicsCommon.h"

namespace ofxBt
{
	class CollisionObject;
}

class ofxBt::CollisionObject
{
public:
	
	CollisionObject() : object(NULL) {}
	CollisionObject(btCollisionObject *object) : object(object) {}
	virtual ~CollisionObject() { clear(); }
	
	inline void clear() { object = NULL; }
	inline operator bool() const { return object != NULL; }
	
	//
	
	inline void activate()
	{
		object->activate(true);
	}
	
	//
	
	inline void setProperty(float rest, float frict)
	{
		setRestitution(rest);
		setFriction(frict);
	}
	
	inline void setRestitution(float rest)
	{
		object->setRestitution(rest);
	}
	
	inline void setFriction(float frict)
	{
		object->setFriction(frict);
	}
	
	inline void setRollingFriction(float frict)
	{
		object->setRollingFriction(frict);
	}
	
	//

	inline ofMatrix4x4 getTransform() const
	{
		return toOF(object->getWorldTransform());
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
	
	//
	
	inline void setKinematic(bool v)
	{
		if (v)
		{
			object->setCollisionFlags(object->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			object->setActivationState(DISABLE_DEACTIVATION);
		}
		else
		{
			object->setCollisionFlags(object->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
			object->setActivationState(DISABLE_DEACTIVATION);
		}
	}
	
	inline bool isKinematic() const
	{
		return (object->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT);
	}
	
	inline void setStatic(bool v)
	{
		if (v)
		{
			object->setCollisionFlags(object->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
		}
		else
		{
			object->setCollisionFlags(object->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT);
		}
	}

	//
	
	inline void setMargin(float v)
	{
		shape()->setMargin(v);
	}
	
	//
	
	inline void setUserData(void *ptr)
	{
		object->setUserPointer(ptr);
	}
	
	inline void* getUserData()
	{
		return object->getUserPointer();
	}
	
protected:
	
	btCollisionObject *object;
	
	inline btCollisionShape* shape() { return object->getCollisionShape(); }
	inline btCollisionShape* shape() const { return object->getCollisionShape(); }
	
};