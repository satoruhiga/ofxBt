#pragma once

#include "ofMain.h"

#include "BulletSoftBody/btSoftBody.h"

namespace ofxBt
{
	class Soft;
}

class ofxBt::Soft
{
public:
	
	Soft() : body(NULL) {}
	Soft(btSoftBody *body) : body(body) {}
	~Soft() { body = NULL; }
	
	void setMass(float mass);
	
private:
	
	btSoftBody *body;
	
};