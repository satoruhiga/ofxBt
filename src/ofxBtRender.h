#pragma once

#include "ofxBtHelper.h"

namespace ofxBt {
	
class Render : public btIDebugDraw
{
public:
	
	Render(float world_scale) : m_debugMode(DBG_DrawWireframe), world_scale(world_scale) {}
	
	void drawTransform(const btTransform& transform, btScalar orthoLen)
	{
		orthoLen = 0.05 * world_scale;
		
		btVector3 start = transform.getOrigin();
		drawLine(start, start+transform.getBasis() * btVector3(orthoLen, 0, 0), btVector3(1, 0, 0));
		drawLine(start, start+transform.getBasis() * btVector3(0, orthoLen, 0), btVector3(0, 1, 0));
		drawLine(start, start+transform.getBasis() * btVector3(0, 0, orthoLen), btVector3(0, 0, 1));
	}
	
	inline void drawLine(const btVector3& from, const btVector3& to,
				  const btVector3& fromColor, const btVector3& toColor)
	{
		glBegin(GL_LINES);
		glColor3fv(fromColor.m_floats);
		glVertex3fv(from.m_floats);
		glColor3fv(toColor.m_floats);
		glVertex3fv(to.m_floats);
		glEnd();
	}
	
	inline void drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
	{
		drawLine(from, to, color, color);
	}
	
	inline void billboard()
	{
		ofMatrix4x4 m;
		glGetFloatv(GL_MODELVIEW_MATRIX, m.getPtr());
		
		const ofVec3f& s = m.getScale();
		
		m(0, 0) = s.x;
		m(0, 1) = 0;
		m(0, 2) = 0;

		m(1, 0) = 0;
		m(1, 1) = s.y;
		m(1, 2) = 0;

		m(2, 0) = 0;
		m(2, 1) = 0;
		m(2, 2) = s.z;
		
		glLoadMatrixf(m.getPtr());
	}
	
	void drawSphere(btScalar radius, const btTransform& transform, const btVector3& color)
	{
		ofPushStyle();
		
		ofNoFill();
		
		glColor3fv(color.m_floats);
		
		const btVector3 &v = transform.getOrigin();
		
		glPushMatrix();
		glTranslatef(v.x(), v.y(), v.z());
		
		billboard();
		ofCircle(0, 0, radius);

		glPopMatrix();
		
		ofPopStyle();
	}
	
	void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) {}
	
	void draw3dText(const btVector3& location,const char* textString)
	{
		ofDrawBitmapString(textString, toOF(location));
	}
	
	void reportErrorWarning(const char* warningString)
	{
		ofLogError("ofxBt") << warningString;
	}
	
	void setDebugMode(int debugMode) { m_debugMode = debugMode; }
	int getDebugMode() const { return m_debugMode;}
	
protected:
	
	int m_debugMode;
	float world_scale;
	
};
	
}
