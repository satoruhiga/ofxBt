#include "ofxBtSoftbodyWorld.h"

#include "ofxBtHelper.h"
#include "ofxBtSoft.h"

using namespace ofxBt;

void SoftbodyWorld::setup(ofVec3f gravity, float world_scale)
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

btSoftBody* SoftbodyWorld::addRope(const ofVec3f& from, const ofVec3f& to, int res)
{
	btSoftBody *o = btSoftBodyHelpers::CreateRope(m_softBodyWorldInfo, toBt(from), toBt(to), res, 0);
	return setupSoftBody(o);
}

btSoftBody* SoftbodyWorld::addPatch(const ofVec3f& v0, const ofVec3f& v1, const ofVec3f& v2, const ofVec3f& v3, int resx, int resy)
{
	btSoftBody *o = btSoftBodyHelpers::CreatePatch(m_softBodyWorldInfo,
												   toBt(v0),
												   toBt(v1),
												   toBt(v2),
												   toBt(v3),
												   resx,
												   resy,
												   0, true);
	return setupSoftBody(o);
}

btSoftBody* SoftbodyWorld::addEllipsoid(const ofVec3f& center, const ofVec3f& radius, int res)
{
	btSoftBody *o = btSoftBodyHelpers::CreateEllipsoid(m_softBodyWorldInfo,
													   toBt(center), toBt(radius), res);
	return setupSoftBody(o);
}

void SoftbodyWorld::removeSoftBody(btSoftBody *body)
{
	disposeSoftBody(body);
}

btSoftBody* SoftbodyWorld::setupSoftBody(btSoftBody *body)
{
	getDynamicsWorld()->addSoftBody(body);
	softBodies.push_back(body);
	
	ofxBt::Soft o = body;
	o.setMass(1);
	o.setStiffness(0.9, 0.9, 0.9);
	
	o.setSolverIterations(4);
	
	o->m_cfg.kCHR = 1;
	o->m_cfg.kKHR = 1;
	o->m_cfg.kSHR = 1;
	o->m_cfg.kAHR = 1;
	
	body->getCollisionShape()->setMargin(getMargin());
	
	return body;
}

void SoftbodyWorld::disposeSoftBody(btSoftBody *body)
{
	getDynamicsWorld()->removeSoftBody(body);
	softBodies.erase(remove(softBodies.begin(), softBodies.end(), body));
	delete body;
}
