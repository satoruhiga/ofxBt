#include "ofxBtWorld.h"

#include "ofxBtRender.h"
#include "ofxBtRigidBody.h"

#include <assert.h>

using namespace ofxBt;

extern ContactProcessedCallback gContactProcessedCallback;
static World *current_dynamics_world = NULL;

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
		removeRigidBody(rigidBodies[0]);
	}
	
	rigidBodies.clear();
}

void World::update()
{
	prepareUpdate();
	
	m_dynamicsWorld->stepSimulation(1.f / 60.f, 10);
	
	postUpdate();
}

void World::prepareUpdate()
{
	current_dynamics_world = this;
	gContactProcessedCallback = ContactProcessedCallback;
}

void World::postUpdate()
{
	gContactProcessedCallback = NULL;
}

void World::draw()
{
	m_dynamicsWorld->debugDrawWorld();
}

RigidBody World::addBox(const ofVec3f& size, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = new btBoxShape(toBt(size * 0.5));
	return addRigidBody(shape, pos, rot);
}

RigidBody World::addSphere(const float size, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = new btSphereShape(size);
	return addRigidBody(shape, pos, rot);
}

RigidBody World::addCylinder(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = new btCylinderShape(btVector3(radius, height, radius));
	return addRigidBody(shape, pos, rot);
}

RigidBody World::addCapsule(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = new btCapsuleShape(radius, height);
	return addRigidBody(shape, pos, rot);
}

RigidBody World::addCone(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = new btConeShape(radius, height);
	return addRigidBody(shape, pos, rot);
}

RigidBody World::addPlane(const ofVec3f& up, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = new btStaticPlaneShape(toBt(up), 0);
	ofxBt::RigidBody rigid = addRigidBody(shape, pos, rot, 0);
	return rigid;
}

RigidBody World::addMesh(const ofMesh &mesh, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = convertToCollisionShape(mesh, false, getMargin());
	ofxBt::RigidBody rigid = addRigidBody(shape, pos, rot);
	return rigid;
}

RigidBody World::addStaticMesh(const ofMesh &mesh, const ofVec3f& pos, const ofVec3f& rot)
{
	btCollisionShape *shape = convertToCollisionShape(mesh, true, getMargin());
	ofxBt::RigidBody rigid = addRigidBody(shape, pos, rot, 0);
	return rigid;
}

vector<RigidBody> World::addWorldBox(const ofVec3f &leftTopFar, const ofVec3f& rightBottomNear)
{
	vector<RigidBody> result;
	
	result.push_back(addPlane(ofVec3f(-1, 0, 0), ofVec3f(rightBottomNear.x, 0, 0)));
	result.push_back(addPlane(ofVec3f(1, 0, 0), ofVec3f(leftTopFar.x, 0, 0)));
	
	result.push_back(addPlane(ofVec3f(0, -1, 0), ofVec3f(0, rightBottomNear.y, 0)));
	result.push_back(addPlane(ofVec3f(0, 1, 0), ofVec3f(0, leftTopFar.y, 0)));
	
	result.push_back(addPlane(ofVec3f(0, 0, -1), ofVec3f(0, 0, rightBottomNear.z)));
	result.push_back(addPlane(ofVec3f(0, 0, 1), ofVec3f(0, 0, leftTopFar.z)));
	
	return result;
}

void World::setGravity(ofVec3f gravity)
{
	m_dynamicsWorld->setGravity(toBt(gravity));
}

btRigidBody* World::addRigidBody(btCollisionShape* shape, const ofVec3f& pos, const ofVec3f& rot, float mass)
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

void World::removeRigidBody(btRigidBody* body)
{
	if (ICollisionCallbackDispatcher *user_data = (ICollisionCallbackDispatcher*)body->getUserPointer())
	{
		delete user_data;
		body->setUserPointer(NULL);
	}
	
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

bool World::ContactProcessedCallback(btManifoldPoint& manifold, void* object0, void* object1)
{
	// NOTICE: maybe this operation is not threadsafe
	if (!current_dynamics_world) return NULL;
	
	const float collision_threshold = current_dynamics_world->getMargin() * 0.5;
	if (fabs(manifold.m_distance1) > collision_threshold)
	{
		btRigidBody *RB0 = btRigidBody::upcast((btCollisionObject*)object0);
		btRigidBody *RB1 = btRigidBody::upcast((btCollisionObject*)object1);

		if (RB0 && RB1)
		{
			CollisionEventArg e = {RB0, RB1};
			ofNotifyEvent(current_dynamics_world->collisionEvent, e);
		}
		
		ICollisionCallbackDispatcher *o0 = (ICollisionCallbackDispatcher*)RB0->getUserPointer();
		ICollisionCallbackDispatcher *o1 = (ICollisionCallbackDispatcher*)RB1->getUserPointer();
		
		if (o0) (*o0)((btCollisionObject*)object1);
		if (o1) (*o1)((btCollisionObject*)object0);
	}
	
	return true;
}
