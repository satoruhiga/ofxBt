#include "ofxBtSoftBodyWorld.h"

#include "ofxBtHelper.h"
#include "ofxBtSoftBody.h"

using namespace ofxBt;

void SoftBodyWorld::setup(ofVec3f gravity, float world_scale)
{
	World::setup(gravity, world_scale);

	m_softBodyWorldInfo.air_density = 1.2;
	m_softBodyWorldInfo.water_density = 0;
	m_softBodyWorldInfo.water_offset = 0;
	m_softBodyWorldInfo.water_normal = btVector3(0,0,0);

	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;
	m_softBodyWorldInfo.m_broadphase = m_broadphase;
	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	
	setGravity(gravity);
}

void SoftBodyWorld::update()
{
	m_dynamicsWorld->stepSimulation(1.f / 60.f, 10, 1. / 240.);
}

void SoftBodyWorld::clear()
{
	World::clear();
	
	while (softBodies.size())
	{
		removeSoftBody(softBodies[0]);
	}
	
	softBodies.clear();
}

btBroadphaseInterface* SoftBodyWorld::createBroadphase()
{
	return new btDbvtBroadphase();
}

btCollisionConfiguration* SoftBodyWorld::createCollisionConfiguration()
{
	return new btSoftBodyRigidBodyCollisionConfiguration;
}

btDiscreteDynamicsWorld* SoftBodyWorld::createDynamicsWorld()
{
	btSoftBodySolver *softBodySolver = 0;
	
	return new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, softBodySolver);
}

void SoftBodyWorld::setGravity(ofVec3f gravity)
{
	World::setGravity(gravity);
	m_softBodyWorldInfo.m_gravity = m_dynamicsWorld->getGravity();
}

btSoftRigidDynamicsWorld* SoftBodyWorld::getDynamicsWorld()
{
	return (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
}

SoftBody SoftBodyWorld::addRope(const ofVec3f& from, const ofVec3f& to, int res)
{
	btSoftBody *o = btSoftBodyHelpers::CreateRope(m_softBodyWorldInfo, toBt(from), toBt(to), res, 0);
	return addSoftBody(o);
}

SoftBody SoftBodyWorld::addPatch(const ofVec3f& v0, const ofVec3f& v1, const ofVec3f& v2, const ofVec3f& v3, int resx, int resy)
{
	btSoftBody *o = btSoftBodyHelpers::CreatePatch(m_softBodyWorldInfo,
												   toBt(v0),
												   toBt(v1),
												   toBt(v2),
												   toBt(v3),
												   resx,
												   resy,
												   0, true);
	return addSoftBody(o);
}

SoftBody SoftBodyWorld::addEllipsoid(const ofVec3f& center, const ofVec3f& radius, int res)
{
	btSoftBody *o = btSoftBodyHelpers::CreateEllipsoid(m_softBodyWorldInfo,
													   toBt(center), toBt(radius), res);
	return addSoftBody(o);
}

btSoftBody* SoftBodyWorld::addSoftBody(btSoftBody *body)
{
	getDynamicsWorld()->addSoftBody(body);
	softBodies.push_back(body);
	
	ofxBt::SoftBody o = body;
	
	o.setMass(1);
	o.setStiffness(0.9, 0.9, 0.9);

	o.setSolverIterations(4);

	body->getCollisionShape()->setMargin(getMargin() * 2);
	
	return body;
}

void SoftBodyWorld::removeSoftBody(btSoftBody *body)
{
	if (ICollisionCallbackDispatcher *user_data = (ICollisionCallbackDispatcher*)body->getUserPointer())
	{
		delete user_data;
		body->setUserPointer(NULL);
	}
	
	getDynamicsWorld()->removeSoftBody(body);
	softBodies.erase(remove(softBodies.begin(), softBodies.end(), body));
	delete body;
}
