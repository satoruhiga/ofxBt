#include "ofxBtSoftbodyWorld.h"

using namespace ofxBt;

void SoftbodyWorld::setup(ofVec3f gravity)
{
	World::setup(gravity);
	
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;
	m_softBodyWorldInfo.m_broadphase = m_broadphase;
	m_softBodyWorldInfo.m_sparsesdf.Initialize();
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