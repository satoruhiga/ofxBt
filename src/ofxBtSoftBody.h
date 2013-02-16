#pragma once

#include "ofMain.h"

#pragma managed(push, off)
#include "BulletSoftBody/btSoftBody.h"
#pragma managed(pop)

#include "ofxBtHelper.h"
#include "ofxBtCollisionObject.h"

namespace ofxBt
{
	class SoftBody;
}

class ofxBt::SoftBody : public ofxBt::CollisionObject
{
public:
	
	SoftBody() {}
	SoftBody(btSoftBody *body) : ofxBt::CollisionObject(body) {}
	
	inline operator btSoftBody*() const { return body(); }
	inline btSoftBody* operator->() const { return body(); }
	
	void setMass(float mass, bool fromfaces = false) { body()->setTotalMass(mass, fromfaces); }
	float getMass() const { return body()->getTotalMass(); }
	
	void setLinearStiffness(float v);
	void setAngularStiffness(float v);
	void setVolumeStiffness(float v);
	void setStiffness(float linear, float angular, float volume);
	
	void setDamping(float v);
	void setDrag(float v);
	void setLift(float v);
	void setPressure(float v);
	
	void setVolumeConversation(float v);
	void setDynamicFriction(float v);
	void setPoseMatching(float v);
	
	void setRigidContactsHrdness(float v);
	void setKineticContactsHrdness(float v);
	void setSoftContactsHrdness(float v);
	void setAnchorsContactsHrdness(float v);
	
	inline size_t getNumNode() const { return body()->m_nodes.size(); }
	
	void setNodePositionAt(size_t n, const ofVec3f& pos);
	void setFixedAt(size_t n);
	void attachRigidBodyAt(size_t n, btRigidBody *rigid);
	
	void setSolverIterations(int n);
	
private:
	
	inline btSoftBody* body() { return (btSoftBody*)object; }
	inline btSoftBody* body() const { return (btSoftBody*)object; }
	
};