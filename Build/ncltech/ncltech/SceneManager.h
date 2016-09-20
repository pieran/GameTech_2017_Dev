/******************************************************************************
Class: SceneManager
Implements: SceneRenderer, TSingleton
Author: Pieran Marris <p.marris@newcastle.ac.uk>
Description:
Extends the SceneRenderer to provide means to have multiple enqueued Scenes that
can be switched between easily. Scenes can be enqueued at the start of the program
by calling SceneManager::Instance()->EnqueueScene(new <MyScene>(<SceneName?>));
If it is the first Scene to be enqueued, it will also instantly become the active scene
shown to the user.

Scenes can be switched between by using one of the JumpToScene methods. This will
call <scene>->OnInitializeScene() and <oldscene>->OnCleanupScene() respectively.

This class is a singleton, so is unique and can be accessed globally.

		(\_/)
		( '_')
	 /""""""""""""\=========     -----D
	/"""""""""""""""""""""""\
....\_@____@____@____@____@_/

*//////////////////////////////////////////////////////////////////////////////

#pragma once
#include "SceneRenderer.h"

class SceneManager : public SceneRenderer, public TSingleton<SceneManager>
{
	friend class TSingleton < SceneManager >;

public:
	void EnqueueScene(Scene* scene);

	void JumpToScene(); //Jumps to next scene (or reloads current scene if it's the only one)
	void JumpToScene(int idx);									//Scenes stored in array format, first-0, second-1 etc..
	void JumpToScene(const std::string& friendly_name);

	inline Scene* GetCurrentScene()			{ return m_Scene; }
	inline uint   GetCurrentSceneIndex()	{ return m_SceneIdx; }
	inline uint   SceneCount()				{ return m_AllScenes.size(); }


protected:
	SceneManager();
	virtual ~SceneManager();

protected:

	uint				m_SceneIdx;
	std::vector<Scene*> m_AllScenes;	//List of all scenes available to swap between
};