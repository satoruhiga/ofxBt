#include "ofxBtHelper.h"

#include "btShapeHull.h"

btCollisionShape* ofxBt::convertToCollisionShape(const ofMesh &mesh, float scale, bool is_static_shape)
{
	const vector <ofIndexType> &indicies = mesh.getIndices();
	const vector <ofVec3f> &verticies = mesh.getVertices();
	
	btCollisionShape* shape = NULL;
	
	int num_index = indicies.size();
	if (num_index > 0
		&& (num_index % 3) == 0)
	{
		btTriangleMesh* trimesh = new btTriangleMesh();
		
		for (int i = 0; i < num_index; i += 3)
		{
			ofIndexType i0 = i;
			ofIndexType i1 = i + 1;
			ofIndexType i2 = i + 2;
			
			trimesh->addTriangle(toBt(verticies[indicies[i0]] * scale), toBt(verticies[indicies[i1]] * scale), toBt(verticies[indicies[i2]] * scale));
		}
		
		if (!is_static_shape)
		{
			btConvexShape* convex = new btConvexTriangleMeshShape(trimesh);
			btShapeHull* hull = new btShapeHull(convex);
			hull->buildHull(convex->getMargin());
			
			btConvexHullShape* temp = new btConvexHullShape;
			for( int i = 0; i < hull->numVertices(); i++ ){
				temp->addPoint(hull->getVertexPointer()[i]);
			}
			
			shape = temp;
			
			delete convex;
			delete hull;
			delete trimesh;
		}
		else
		{
			bool useQuantization = true;
			shape = new btBvhTriangleMeshShape(trimesh, useQuantization);
			
			// FIXME: delete trimesh??
		}
	}
	
	return shape;
}