#include "ofxBtWorld.h"

#include "ofxBtRender.h"
#include "ofxBtRigidBody.h"

using namespace ofxBt;

World::World()
{
}

World::~World()
{
	clear();
}

void World::setup(ofVec3f gravity, float world_scale)
{
	this->world_scale = world_scale;
	
	m_collisionConfiguration = createCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = createBroadphase();
	m_solver = new btSequentialImpulseConstraintSolver;
	
	m_dynamicsWorld = createDynamicsWorld();
	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	
	setGravity(gravity);
	
	m_dynamicsWorld->setDebugDrawer(new Render(world_scale));
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE);
}

void World::clear()
{
	while (rigidBodies.size())
	{
		removeRegidBody(rigidBodies[0]);
	}
	
	rigidBodies.clear();
}

void World::update()
{
	m_dynamicsWorld->stepSimulation(1.f / 60.f, 10);
}

void World::draw()
{
	m_dynamicsWorld->debugDrawWorld();
}

btRigidBody* World::addBox(const ofVec3f& size, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = new btBoxShape(toBt(size * 0.5));
	return setupRigidBody(shape, pos, rot);
}

btRigidBody* World::addSphere(const float size, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = new btSphereShape(size);
	return setupRigidBody(shape, pos, rot);
}

btRigidBody* World::addCylinder(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = new btCylinderShape(btVector3(radius, height, radius));
	return setupRigidBody(shape, pos, rot);
}

btRigidBody* World::addCapsule(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = new btCapsuleShape(radius, height);
	return setupRigidBody(shape, pos, rot);
}

btRigidBody* World::addCone(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = new btConeShape(radius, height);
	return setupRigidBody(shape, pos, rot);
}

btRigidBody* World::addPlane(const ofVec3f& up, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = new btStaticPlaneShape(toBt(up), 0);
	ofxBt::RigidBody rigid = setupRigidBody(shape, pos, rot, 0);
	return rigid;
}

btRigidBody* World::addMesh(const ofMesh &mesh, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = convertToCollisionShape(mesh, getWorldScale(), false);
	ofxBt::RigidBody rigid = setupRigidBody(shape, pos, rot);
	return rigid;
}

btRigidBody* World::addStaticMesh(const ofMesh &mesh, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = convertToCollisionShape(mesh, getWorldScale(), true);
	ofxBt::RigidBody rigid = setupRigidBody(shape, pos, rot, 0);
	return rigid;
}

vector<btRigidBody*> World::addWorldBox(const ofVec3f &leftTopFar, const ofVec3f& rightBottomNear)
{
	vector<btRigidBody*> result;
	
	result.push_back(addPlane(ofVec3f(-1, 0, 0), ofVec3f(rightBottomNear.x, 0, 0)));
	result.push_back(addPlane(ofVec3f(1, 0, 0), ofVec3f(leftTopFar.x, 0, 0)));
	
	result.push_back(addPlane(ofVec3f(0, -1, 0), ofVec3f(0, rightBottomNear.y, 0)));
	result.push_back(addPlane(ofVec3f(0, 1, 0), ofVec3f(0, leftTopFar.y, 0)));
	
	result.push_back(addPlane(ofVec3f(0, 0, -1), ofVec3f(0, 0, rightBottomNear.z)));
	result.push_back(addPlane(ofVec3f(0, 0, 1), ofVec3f(0, 0, leftTopFar.z)));
	
	return result;
}

void World::removeRegidBody(btRigidBody *body)
{
	assert(body);
	disposeRigidBody(body);
}

void World::setGravity(ofVec3f gravity)
{
	m_dynamicsWorld->setGravity(toBt(gravity));
}

btRigidBody* World::setupRigidBody(btCollisionShape* shape, const ofVec3f& pos, const ofVec3f& rot, float mass)
{
	btTransform t(btQuaternion(rot.x * DEG_TO_RAD, rot.y * DEG_TO_RAD, rot.z * DEG_TO_RAD), toBt(pos));
	btDefaultMotionState* ms = new btDefaultMotionState(t);
	
	btVector3 inertia(0, 0, 0);
	shape->calculateLocalInertia(mass, inertia);
	shape->setMargin(0);
	
	btRigidBody::btRigidBodyConstructionInfo info(mass, ms, shape, inertia);
	RigidBody rigid = new btRigidBody(info);
	
	assert(rigid);
	
	rigid.setProperty(0.4, 0.75, 0.25, 0.25);
	
	m_dynamicsWorld->addRigidBody(rigid);
	rigidBodies.push_back(rigid);
	
	return rigid;
}

void World::disposeRigidBody(btRigidBody* body)
{
	if (body->getCollisionShape())
	{
		delete body->getCollisionShape();
		body->setCollisionShape(NULL);
	}
	
	if (body->getMotionState())
	{
		delete body->getMotionState();
		body->setMotionState(NULL);
	}
	
	m_dynamicsWorld->removeRigidBody(body);
	
	rigidBodies.erase(remove(rigidBodies.begin(), rigidBodies.end(), body), rigidBodies.end());
	
	delete body;
}

btBroadphaseInterface* World::createBroadphase()
{
	// -100unit ~ +100unit
	btVector3 min(-100 * getWorldScale(), -100 * getWorldScale(), -100 * getWorldScale());
	btVector3 max(100 * getWorldScale(), 100 * getWorldScale(), 100 * getWorldScale());

	return new btAxisSweep3(min, max);
}

btCollisionConfiguration* World::createCollisionConfiguration()
{
	return new btDefaultCollisionConfiguration;
}

btDiscreteDynamicsWorld* World::createDynamicsWorld()
{
	return new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
}

btDiscreteDynamicsWorld* World::getDynamicsWorld()
{
	return (btDiscreteDynamicsWorld*)m_dynamicsWorld;
}
