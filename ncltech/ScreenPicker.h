#pragma once

#include "RenderList.h"
#include "TSingleton.h"

//Our texture only stores 16bit unsigned shorts, so has a hard limit on the number of values it can store. 
//  Hopefully you will never be able to trigger this value though. :)
#define MAX_PICKABLE_OBJECTS 65534

//NSIGHT doesnt (at the time of writing) support reading GL_RED_INTEGER format. So this hack will treat all indicies as 32bit floats
#define USE_NSIGHT_HACK

class ScreenPicker : public TSingleton<ScreenPicker>
{
	friend class TSingleton<ScreenPicker>;
	friend class SceneRenderer;

public:

	void RegisterObject(Object* obj);
	void UnregisterObject(Object* obj);

protected:
	//Called by ScreenRenderer
	void ClearAllObjects();
	void UpdateFBO(int screen_width, int screen_height);
	void RenderPickingScene(RenderList* scene_renderlist, const Matrix4& proj_matrix, const Matrix4& view_matrix);

	//ScreenRenderer Update Phase
	bool HandleMouseClicks(float dt); //Returns true if an object has been clicked

	//Pseodo Protected
	ScreenPicker();
	virtual ~ScreenPicker();

	void HandleObjectMouseUp(float dt, bool mouse_in_window, Vector3& clip_space);
	void HandleObjectMouseMove(float dt, Vector3& clip_space);


protected:

	Object*			m_CurrentlyHoverObject;
	Object*			m_CurrentlyHeldObject;
	float			m_OldDepth;
	Vector3			m_OldWorldSpacePos;
	Matrix4			m_invViewProjMtx;

	std::vector<Object*> m_AllRegisteredObjects;

	int m_TexWidth, m_TexHeight;
	GLuint m_PickerFBO;
	GLuint m_PickerRB; //Must use renderbuffer outputing integer colour data to texture is not supported. This is the same as texture output framebuffer except they are not in themselves valid textures
	GLuint m_PickerDepthRB;

	Shader* m_ShaderPicker;
};