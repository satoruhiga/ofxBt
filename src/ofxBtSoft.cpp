#include "ofxBtSoft.h"

using namespace ofxBt;

void Soft::setLinearStiffness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_materials[0]->m_kLST = v;
}

void Soft::setAngularStiffness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_materials[0]->m_kAST = v;
}

void Soft::setVolumeStiffness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_materials[0]->m_kVST = v;
}

void Soft::setStiffness(float linear, float angular, float volume)
{
	setLinearStiffness(linear);
	setAngularStiffness(angular);
	setVolumeStiffness(volume);
}

void Soft::setDamping(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kDP = v;
}

void Soft::setDrag(float v)
{
	if (v < 0) v = 0;
	body()->m_cfg.kDG = v;
}

void Soft::setLift(float v)
{
	if (v < 0) v = 0;
	body()->m_cfg.kPR = v;
}

void Soft::setPressure(float v)
{
	body()->m_cfg.kLF = v;
}

void Soft::setVolumeConversation(float v)
{
	if (v < 0) v = 0;
	body()->m_cfg.kVC = v;
}

void Soft::setDynamicFriction(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kVC = v;
}

void Soft::setPoseMatching(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kMT = v;
}

void Soft::setRigidContactsHrdness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kCHR = v;
}

void Soft::setKineticContactsHrdness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kKHR = v;
}

void Soft::setSoftContactsHrdness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kSHR = v;
}

void Soft::setAnchorsContactsHrdness(float v)
{
	v = ofClamp(v, 0, 1);
	body()->m_cfg.kAHR = v;
}

void Soft::setSolverIterations(int n)
{
	body()->m_cfg.piterations = n;
}

void Soft::setFixedAt(size_t n)
{
	body()->setMass(n, 0);
}

void Soft::setNodePositionAt(size_t n, const ofVec3f& pos)
{
	body()->m_nodes.at(n).m_x = toBt(pos);
}

void Soft::attachRigidBodyAt(size_t n, btRigidBody *rigid)
{
	body()->appendAnchor(n, rigid);
}