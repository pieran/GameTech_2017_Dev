/******************************************************************************
Class: ObjectMesh
Implements: Object
Author:Pieran Marris <p.marris@newcastle.ac.uk>
Description: A simple game object implementation that has a mesh and an optional texture component
that are rendered to the scene.


		(\_/)							
		( '_')							
	 /""""""""""""\=========     -----D
	/"""""""""""""""""""""""\			
....\_@____@____@____@____@_/

*//////////////////////////////////////////////////////////////////////////////
#pragma once
#include "Object.h"
#include <nclgl/Mesh.h>

class ObjectMesh : public Object
{
public:
	ObjectMesh(const std::string& name);
	virtual ~ObjectMesh();

	void	SetMesh(Mesh* mesh, bool deleteMeshOnCleanup);
	Mesh*	GetMesh()		{ return m_pMesh; }

	void	SetTexture(GLuint tex, bool deleteTexOnCleanup);
	GLuint  GetTexture()	{ return m_Texture; }
protected:
	void	OnRenderObject() override;				//Handles OpenGL calls to Render the object

protected:
	bool	m_DeleteMeshOnCleanup, m_DeleteTexOnCleanup;

	GLuint  m_Texture;
	Mesh*	m_pMesh;
};