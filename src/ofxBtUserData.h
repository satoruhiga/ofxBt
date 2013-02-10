#pragma once

#include "ofMain.h"

#include "btBulletDynamicsCommon.h"

namespace ofxBt
{
	struct UserData;
}

struct ofxBt::UserData
{
	btCollisionObject *self;
	
	UserData(btCollisionObject *self) : self(self) {}
	~UserData();
};

// TODO: set custom userdata type
