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