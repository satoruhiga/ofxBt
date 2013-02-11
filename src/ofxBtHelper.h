#pragma once

#include "ofMain.h"

#include "btBulletDynamicsCommon.h"

template <typename T1, typename T2>
const T2& toOF(const T1& o1, T2& o2);

template <typename T1, typename T2>
const T2& toBt(const T1& o1, T2& o2);

// Vec3

template <>
inline const ofVec3f& toOF(const btVector3& o1, ofVec3f& o2)
{
	o2.set(o1.x(), o1.y(), o1.z());
	return o2;
}

inline ofVec3f toOF(const btVector3& o)
{
	ofVec3f result;
	return toOF(o, result);
}

template <>
inline const btVector3& toBt(const ofVec3f& o1, btVector3& o2)
{
	o2.setValue(o1.x, o1.y, o1.z);
	return o2;
}

inline btVector3 toBt(const ofVec3f& o)
{
	btVector3 result;
	return toBt(o, result);
}

// Quat

template <>
inline const ofQuaternion& toOF(const btQuaternion& o1, ofQuaternion& o2)
{
	o2.set(o1.x(), o1.y(), o1.z(), o1.w());
	return o2;
}

inline ofQuaternion toOF(const btQuaternion& o)
{
	ofQuaternion result;
	return toOF(o, result);
}

template <>
inline const btQuaternion& toBt(const ofQuaternion& o1, btQuaternion& o2)
{
	o2.setValue(o1.x(), o1.y(), o1.z(), o1.w());
	return o2;
}

inline btQuaternion toBt(const ofQuaternion& o)
{
	btQuaternion result;
	return toBt(o, result);
}

// Matrix
	
template <>
inline const ofMatrix4x4& toOF(const btTransform& o1, ofMatrix4x4& o2)
{
	o1.getOpenGLMatrix(o2.getPtr());
	return o2;
}

inline ofMatrix4x4 toOF(const btTransform& o)
{
	ofMatrix4x4 result;
	return toOF(o, result);
}

template <>
inline const btTransform& toBt(const ofMatrix4x4& o1, btTransform& o2)
{
	o2.setFromOpenGLMatrix(o1.getPtr());
	return o2;
}

inline btTransform toBt(const ofMatrix4x4& o)
{
	btTransform result;
	return toBt(o, result);
}


namespace ofxBt
{
	
	btCollisionShape* convertToCollisionShape(const ofMesh &mesh, bool is_static_shape = true, float margin = 0.04);
	
}