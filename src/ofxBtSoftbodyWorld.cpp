#include "ofxBtSoftbodyWorld.h"

#include "ofxBtHelper.h"
#include "ofxBtSoft.h"

using namespace ofxBt;

void SoftbodyWorld::setup(ofVec3f gravity)
{
	World::setup(gravity);
	
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;
	m_softBodyWorldInfo.m_broadphase = m_broadphase;
	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	
	setGravity(gravity);
}

btCollisionConfiguration* SoftbodyWorld::createCollisionConfiguration()
{
	return new btSoftBodyRigidBodyCollisionConfiguration;
}

btDiscreteDynamicsWorld* SoftbodyWorld::createDynamicsWorld()
{
	btSoftBodySolver *softBodySolver = 0;
	
	return new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, softBodySolver);
}

void SoftbodyWorld::setGravity(ofVec3f gravity)
{
	World::setGravity(gravity);
	m_softBodyWorldInfo.m_gravity = m_dynamicsWorld->getGravity();
}

btSoftRigidDynamicsWorld* SoftbodyWorld::getDynamicsWorld()
{
	return (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
}

btSoftBody* SoftbodyWorld::addRope(const ofVec3f& from, const ofVec3f& to, int res, int fixeds)
{
	btSoftBody *o = btSoftBodyHelpers::CreateRope(m_softBodyWorldInfo, toBt(from), toBt(to), res, fixeds);
	return setupSoftBody(o);
}

btSoftBody* SoftbodyWorld::setupSoftBody(btSoftBody *body)
{
	getDynamicsWorld()->addSoftBody(body);
	softBodies.push_back(body);
	
	ofxBt::Soft o = body;
	o.setMass(1);
	
	return body;
}
