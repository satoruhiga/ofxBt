#include "ofxBtWorld.h"

#include "ofxBtRender.h"
#include "ofxBtRigid.h"

using namespace ofxBt;

World::World() : inited(false)
{
}

World::~World()
{
	clear();
}

void World::setup(ofVec3f gravity)
{
	inited = true;
	
	btVector3 worldAabbMin(-1000, -1000, -1000);
	btVector3 worldAabbMax(1000, 1000, 1000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);
	
	m_collisionConfiguration = createCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_solver = new btSequentialImpulseConstraintSolver;
	
	m_dynamicsWorld = createDynamicsWorld();
	
	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	
	setGravity(gravity);
	
	m_dynamicsWorld->setDebugDrawer(new Render);
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
	assert(inited);
	m_dynamicsWorld->debugDrawWorld();
}

btRigidBody* World::addBox(const ofVec3f& size, const ofVec3f& pos, const ofVec3f& rot)
{
	assert(inited);
	
	btCollisionShape *shape = new btBoxShape(toBt(size * 0.5));
	return setupRigidBody(shape, pos, rot);
}

btRigidBody* World::addSphere(const float size, const ofVec3f& pos, const ofVec3f& rot)
{
	assert(inited);
	
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
	assert(inited);
	
	btCollisionShape *shape = new btStaticPlaneShape(toBt(up), 1);
	ofxBt::Rigid rigid = setupRigidBody(shape, pos, rot, 0);
	return rigid.get();
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
	assert(inited);
	assert(body);
	
	disposeRigidBody(body);
}

void World::setGravity(ofVec3f gravity)
{
	assert(inited);
	m_dynamicsWorld->setGravity(toBt(gravity));
}

btRigidBody* World::setupRigidBody(btCollisionShape* shape, const ofVec3f& pos, const ofVec3f& rot, float mass)
{
	btTransform t(btQuaternion(rot.x * DEG_TO_RAD, rot.y * DEG_TO_RAD, rot.z * DEG_TO_RAD), toBt(pos));
	btDefaultMotionState* ms = new btDefaultMotionState(t);
	
	btVector3 inertia(0, 0, 0);
	shape->calculateLocalInertia(mass, inertia);
	shape->setMargin(4);
	
	btRigidBody::btRigidBodyConstructionInfo info(mass, ms, shape, inertia);
	btRigidBody* rigid = new btRigidBody(info);
	
	assert(rigid);
	
	Rigid(rigid).setProperty(0.4, 0.75, 0.25, 0.25);
	
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
