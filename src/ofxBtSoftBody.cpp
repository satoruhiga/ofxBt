#include "ofxBtSoftBody.h"

using namespace ofxBt;

void SoftBody::setLinearStiffness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_materials[0]->m_kLST = v;
}

void SoftBody::setAngularStiffness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_materials[0]->m_kAST = v;
}

void SoftBody::setVolumeStiffness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_materials[0]->m_kVST = v;
}

void SoftBody::setStiffness(float linear, float angular, float volume)
{
	setLinearStiffness(linear);
	setAngularStiffness(angular);
	setVolumeStiffness(volume);
}

void SoftBody::setDamping(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kDP = v;
}

void SoftBody::setDrag(float v)
{
	if (v < 0) v = 0;
	body()->m_cfg.kDG = v;
}

void SoftBody::setLift(float v)
{
	if (v < 0) v = 0;
	body()->m_cfg.kPR = v;
}

void SoftBody::setPressure(float v)
{
	body()->m_cfg.kLF = v;
}

void SoftBody::setVolumeConversation(float v)
{
	if (v < 0) v = 0;
	body()->m_cfg.kVC = v;
}

void SoftBody::setDynamicFriction(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kVC = v;
}

void SoftBody::setPoseMatching(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kMT = v;
}

void SoftBody::setRigidContactsHrdness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kCHR = v;
}

void SoftBody::setKineticContactsHrdness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kKHR = v;
}

void SoftBody::setSoftContactsHrdness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kSHR = v;
}

void SoftBody::setAnchorsContactsHrdness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kAHR = v;
}

void SoftBody::setSolverIterations(int n)
{
	body()->m_cfg.piterations = n;
}

void SoftBody::setFixedAt(size_t n)
{
	body()->setMass(n, 0);
}

void SoftBody::setNodePositionAt(size_t n, const ofVec3f& pos)
{
	body()->m_nodes.at(n).m_x = toBt(pos);
}

void SoftBody::attachRigidBodyAt(size_t n, btRigidBody *rigid)
{
	body()->appendAnchor(n, rigid);
}