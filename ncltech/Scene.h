/******************************************************************************
Class: Scene
Implements: OGLRenderer
Author:Pieran Marris <p.marris@newcastle.ac.uk>, Rich Davison <richard.davison4@newcastle.ac.uk> and YOU!
Description: For this module, you are being provided with a basic working
Renderer - to give you more time to work on your physics and AI!

It is basically the Renderer from the Graphics For Games Module as it was
by Tutorial 7 - Scene Management. It will split nodes up into those that are
opaque and transparent, and render accordingly.

The only new bits are the ability to search for Game Object's by their name,
this is not a fast function but does allow some ability to talk between objects in a more
complicated scene.


		(\_/)								-_-_-_-_-_-_-_,------,
		( '_')								_-_-_-_-_-_-_-|   /\_/\   NYANYANYAN
	 /""""""""""""\=========     -----D		-_-_-_-_-_-_-~|__( ^ .^) /
	/"""""""""""""""""""""""\				_-_-_-_-_-_-_-""  ""
....\_@____@____@____@____@_/

*//////////////////////////////////////////////////////////////////////////////
#pragma once

#include <nclgl/OGLRenderer.h>
#include <nclgl/Camera.h>
#include <nclgl/Shader.h>
#include <nclgl/Frustum.h>

#include "TSingleton.h"
#include "Object.h"
#include "RenderList.h"


class Scene
{
public:
	Scene(const std::string& friendly_name); //Called once at program start - all scene initialization should be done in 'OnInitialize'
	~Scene();

	virtual void OnInitializeScene()	{}								//Called when scene is being activated, and will begin being rendered/updated. - initialize objects/physics here
	virtual void OnCleanupScene()		{ DeleteAllGameObjects(); };	//Called when scene is being swapped and will no longer be rendered/updated - remove objects/physics here
	virtual void OnUpdateScene(float dt);								//This is msec * 0.001f (e.g time relative to seconds not milliseconds)

	void DeleteAllGameObjects(); //Easiest way of cleaning up the scene - unless you need to save some game objects after scene becomes innactive for some reason.

	void AddGameObject(Object* game_object);
	Object* FindGameObject(const std::string& name);

	//Sets maximum bounds of the scene - for use in shadowing
	void  SetWorldRadius(float radius)	{ m_RootGameObject->SetBoundingRadius(radius); }
	float GetWorldRadius()				{ return m_RootGameObject->GetBoundingRadius(); }

	void BuildWorldMatrices();
	void InsertToRenderList(RenderList* list, const Frustum& frustum);

	const std::string& GetSceneName() { return m_SceneName; }

protected:

	void	UpdateWorldMatrices(Object* node, const Matrix4& parentWM);
	void	InsertToRenderList(Object* node, RenderList* list, const Frustum& frustum);
	void	UpdateNode(float dt, Object* cNode);

protected:
	std::string			m_SceneName;
	Object*				m_RootGameObject;
};