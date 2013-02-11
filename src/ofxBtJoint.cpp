#include "ofxBtJoint.h"

using namespace ofxBt;

Joint::Joint(btRigidBody *obj0, btRigidBody *obj1)
{
	btTransform frameInA;
	btTransform frameInB;
	
	joint = new btGeneric6DofConstraint(*obj0, *obj1, frameInA, frameInB, true);
}
