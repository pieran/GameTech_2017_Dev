/******************************************************************************
Class:GameObject
Implements:
Author: Rich Davison	<richard.davison4@newcastle.ac.uk> and YOU!
Description: This is the base class for all of the objects in your game - the
player character, enemies, pickups etc would all be derived classes of the
GameObject.

This class 'wraps up' all of the communication between the subsystems of your
game - it has a renderer component (The SceneNode you are used to using), a
'physics' component (The PhysicsNode you've been introduced to in this code
download), and eventually you'll add a 'sound' component, in the short sound
workshop in semester 2.

The sub systems handle the updating of the 'sub components' of a GameEntity,
so in the Update function of this class will just be the 'gameplay' specific
type functionality (has the player picked up some health / has he shot an enemy
and so forth).

-_-_-_-_-_-_-_,------,
_-_-_-_-_-_-_-|   /\_/\   NYANYANYAN
-_-_-_-_-_-_-~|__( ^ .^) /
_-_-_-_-_-_-_-""  ""

*//////////////////////////////////////////////////////////////////////////////
#pragma once
#include <nclgl\Matrix4.h>
#include "PhysicsObject.h"
#include <vector>

class Scene;
class PhysicsEngine;
class RenderList;
class SceneRenderer;

class Object
{
	//This are the only class that can manually set the world transform
	friend class Scene;
	friend class RenderList;
	friend class SceneRenderer;
	friend class PhysicsEngine;
	friend class ScreenPicker;

public:
	Object(const std::string& name = "");
	virtual ~Object();



	void CreatePhysicsNode();
	bool HasPhysics() { return (m_PhysicsObject != NULL); }

	PhysicsObject*		Physics() { return m_PhysicsObject; }



	const std::string&	GetName()			{ return m_Name; }
	std::vector<Object*>& GetChildren()		{ return m_Children; }
	

	Object*				FindGameObject(const std::string& name);
	void				AddChildObject(Object* child);

	virtual bool IsOpaque() { return m_Colour.w >= 0.999f; }


	void			SetLocalTransform(const Matrix4& transform)			{ m_LocalTransform = transform; }
	const Matrix4&  GetLocalTransform()									{ return m_LocalTransform; }


	void			SetColour(const Vector4& colour)	{ m_Colour = colour; }
	const Vector4&	GetColour()							{ return m_Colour; }

	void			SetBoundingRadius(float radius)		{ m_BoundingRadius = radius; }
	float			GetBoundingRadius()					{ return m_BoundingRadius; }

	uint			GetFrustumCullFlags()				{ return m_FrustumCullFlags; }
	uint			GetScreenPickerIdx()				{ return m_ScreenPickerIdx; }
	
	const Matrix4&  GetWorldTransform()					{ return m_WorldTransform; }


protected:
	virtual void OnRenderObject()				{};				//Handles OpenGL calls to Render the object
	virtual void OnUpdateObject(float dt)		{};				//Override to handle things like AI etc on update loop

	// Mouse Interactivity - To Enable the gameobject must call "ScreenPicker::Instance()->RegisterObject(this)"
	virtual void OnMouseEnter(float dt)			{};
	virtual void OnMouseLeave(float dt)			{};
	virtual void OnMouseDown(float dt, const Vector3& worldPos)									{};
	virtual void OnMouseMove(float dt, const Vector3& worldPos, const Vector3& worldChange)		{};
	virtual void OnMouseUp(float dt, const Vector3& worldPos)									{};

protected:
	std::string					m_Name;
	Scene*						m_Scene;
	Object*						m_Parent;
	std::vector<Object*>		m_Children;

	PhysicsObject*				m_PhysicsObject;

	Vector4						m_Colour;
	float						m_BoundingRadius;	//For Frustum Culling
	Matrix4						m_LocalTransform;
	Matrix4						m_WorldTransform;
	uint						m_FrustumCullFlags; //Series of boolean checks to identify if the object is already inside the given renderlist's frustum
	uint						m_ScreenPickerIdx;  //Unique ID for screen picking
};