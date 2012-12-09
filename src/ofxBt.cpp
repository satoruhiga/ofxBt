#include "ofxBt.h"

namespace ofxBt {
	
class DebugRender : public btIDebugDraw
{
public:
	
	DebugRender() : m_debugMode(false) {}
	
	void drawLine(const btVector3& from, const btVector3& to, 
				  const btVector3& fromColor, const btVector3& toColor)
	{
		glBegin(GL_LINES);
		glColor3f(fromColor.getX(), fromColor.getY(), fromColor.getZ());
		glVertex3d(from.getX(), from.getY(), from.getZ());
		glColor3f(toColor.getX(), toColor.getY(), toColor.getZ());
		glVertex3d(to.getX(), to.getY(), to.getZ());
		glEnd();
	}
	
	void drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
	{
		drawLine(from, to, color, color);
	}
	
	inline void billboard()
	{
		GLfloat m[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, m);
		
		float inv_len;
		
		m[8] = -m[12];
		m[9] = -m[13];
		m[10] = -m[14];
		inv_len = 1. / sqrt(m[8] * m[8] + m[9] * m[9] + m[10] * m[10]);
		m[8] *= inv_len;
		m[9] *= inv_len;
		m[10] *= inv_len;
		
		m[0] = -m[14];
		m[1] = 0.0;
		m[2] = m[12];
		inv_len = 1. / sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
		m[0] *= inv_len;
		m[1] *= inv_len;
		m[2] *= inv_len;
		
		m[4] = m[9] * m[2] - m[10] * m[1];
		m[5] = m[10] * m[0] - m[8] * m[2];
		m[6] = m[8] * m[1] - m[9] * m[0];
		
		glLoadMatrixf(m);
	}

	void drawSphere(btScalar radius, const btTransform& transform, const btVector3& color)
	{
		ofPushStyle();
		
		ofNoFill();
		
		glColor4f(color.getX(), color.getY(), color.getZ(), btScalar(1.0f));
		
		ofMatrix4x4 m;
		m.glTranslate(toOF(transform.getOrigin()));
		
		glPushMatrix();
		glMultMatrixf(m.getPtr());
		billboard();
		ofCircle(0, 0, radius);
		glPopMatrix();
		
		ofPopStyle();
	}
	
	void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) {}
	void draw3dText(const btVector3& location,const char* textString) {}	
	void reportErrorWarning(const char* warningString) {}
	
	void setDebugMode(int debugMode) { m_debugMode = debugMode; }
	int getDebugMode() const { return m_debugMode;}
	
protected:
	
	int m_debugMode;
};

#pragma mark -

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
	
	collisionConfiguration = new btDefaultCollisionConfiguration;
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver;
	
	// broadphase = new btDbvtBroadphase();
	
	btVector3 worldAabbMin(-1000, -1000, -1000);
	btVector3 worldAabbMax(1000, 1000, 1000);
	broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);
	
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	
	setGravity(gravity);
	
	dynamicsWorld->setDebugDrawer(new DebugRender);
	dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE);
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
	dynamicsWorld->stepSimulation(1.f / 60.f, 10);
}

void World::draw()
{
	assert(inited);
	dynamicsWorld->debugDrawWorld();
}

btRigidBody* World::addBox(const ofVec3f& size, const ofVec3f& pos, const ofVec3f& rot, float mass)
{
	assert(inited);
	
	btCollisionShape *shape = new btBoxShape(toBt(size * 0.5));
	return createRigidBody(shape, mass, pos, rot);
}

btRigidBody* World::addSphere(const float size, const ofVec3f& pos, const ofVec3f& rot, float mass)
{
	assert(inited);
	
	btCollisionShape *shape = new btSphereShape(size);
	return createRigidBody(shape, mass, pos, rot);
}

btRigidBody* World::addCylinder(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot, float mass)
{
	btCollisionShape *shape = new btCylinderShape(btVector3(radius, height, radius));
	return createRigidBody(shape, mass, pos, rot);
}

btRigidBody* World::addCapsule(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot, float mass)
{
	btCollisionShape *shape = new btCapsuleShape(radius, height);
	return createRigidBody(shape, mass, pos, rot);
}

btRigidBody* World::addCone(const float radius, const float height, const ofVec3f& pos, const ofVec3f& rot, float mass)
{
	btCollisionShape *shape = new btConeShape(radius, height);
	return createRigidBody(shape, mass, pos, rot);
}

btRigidBody* World::addPlane(const ofVec3f& up, const ofVec3f& pos, const ofVec3f& rot, float mass)
{
	assert(inited);
	
	btCollisionShape *shape = new btStaticPlaneShape(toBt(up), 1);
	return createRigidBody(shape, mass, pos, rot);
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
	dynamicsWorld->setGravity(toBt(gravity));
}

btRigidBody* World::createRigidBody(btCollisionShape* shape, float mass, const ofVec3f& pos, const ofVec3f& rot)
{
	btTransform t(btQuaternion(rot.x * DEG_TO_RAD, rot.y * DEG_TO_RAD, rot.z * DEG_TO_RAD), toBt(pos));
	btDefaultMotionState* ms = new btDefaultMotionState(t);
	
	btVector3 inertia(0, 0, 0);
	shape->calculateLocalInertia(mass, inertia);
	
	btRigidBody::btRigidBodyConstructionInfo info(mass, ms, shape, inertia);
	btRigidBody* rigid = new btRigidBody(info);
	
	assert(rigid);
	
	Rigid(rigid).setProperty(0.4, 0.75, 0.25, 0.25);
	
	dynamicsWorld->addRigidBody(rigid);
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
	
	dynamicsWorld->removeRigidBody(body);
	
	rigidBodies.erase(remove(rigidBodies.begin(), rigidBodies.end(), body), rigidBodies.end());

	delete body;
}

}