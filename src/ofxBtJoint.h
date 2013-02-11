#pragma once

#include "btBulletDynamicsCommon.h"

namespace ofxBt
{
	class Joint;
}

class ofxBt::Joint
{
public:
	
	Joint() : joint(NULL) {}
	Joint(btGeneric6DofConstraint *joint) : joint(joint) {}
	
	Joint(btRigidBody *obj0, btRigidBody *obj1);

protected:
	
	btGeneric6DofConstraint *joint;
	
};
