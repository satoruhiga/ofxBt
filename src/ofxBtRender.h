#pragma once

#include "ofxBtHelper.h"

namespace ofxBt {
	
class Render : public btIDebugDraw
{
public:
	
	Render() : m_debugMode(false) {}
	
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
	void reportErrorWarning(const char* warningString)
	{
		ofLogError("ofxBt") << warningString;
	}
	
	void setDebugMode(int debugMode) { m_debugMode = debugMode; }
	int getDebugMode() const { return m_debugMode;}
	
protected:
	
	int m_debugMode;
};
	
}
