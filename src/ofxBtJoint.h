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
	
protected:
	
	btGeneric6DofConstraint *joint;
	
};
