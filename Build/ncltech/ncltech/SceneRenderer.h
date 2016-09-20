
#pragma once

#include <nclgl\OGLRenderer.h>
#include <nclgl\common.h>
#include "Scene.h"
#include "RenderList.h"
#include "TSingleton.h"

enum ScreenTextures
{
	SCREENTEX_DEPTH = 0,
	SCREENTEX_STENCIL = 0,
	SCREENTEX_COLOUR0 = 1, //Main Render (gamma corrected) (Ping-Pong)
	SCREENTEX_COLOUR1 = 2, //Main Render (gamma corrected) (Ping-Pong)
	SCREENTEX_NORMALS = 3,
	SCREENTEX_LIGHTING = 4, //Diffuse/Spec Calculations
	SCREENTEX_MAX
};

#define SHADOWMAP_MAX 16 //Hard limit defined in shader
#define PROJ_FAR				80.0f			//See 80m
#define PROJ_NEAR				0.01f
#define PROJ_FOV				45.0f

class SceneRenderer : public OGLRenderer
{
public:
	virtual void	InitializeOGLContext(Window& parent) override;

	virtual void RenderScene() override;
	virtual void UpdateScene(float dt) override; //This is msec * 0.001f (e.g time relative to seconds not milliseconds)


	inline Camera* GetCamera()							{ return m_Camera; }

	inline const Vector3& GetBackgroundColor()			{ return m_BackgroundColour; }
	inline const Vector3& GetAmbientColor()				{ return m_AmbientColour; }
	inline const Vector3& GetInverseLightDirection()	{ return m_InvLightDirection; }
	inline const float GetSpecularIntensity()			{ return m_SpecularIntensity; }

	inline void SetBackgroundColor(const Vector3& col)			{ m_BackgroundColour = col; }
	inline void SetAmbientColor(const Vector3& col)				{ m_AmbientColour = col; }
	inline void SetInverseLightDirection(const Vector3& dir)	{ m_InvLightDirection = dir; m_InvLightDirection.Normalise(); }
	inline void SetSpecularIntensity(float intensity)			{ m_SpecularIntensity = intensity; }

	inline bool GetVsyncEnabled()						{ return m_VsyncEnabled; }
	inline void SetVsyncEnabled(bool enabled)			{ wglSwapIntervalEXT((m_VsyncEnabled = enabled) ? 1 : 0); }

	inline float GetGammaCorrection()								{ return m_GammaCorrection; }
	inline void  SetGammaCorrection(float gamma)					{ m_GammaCorrection = gamma; }

	inline uint GetShadowMapSize()						{ return m_ShadowMapSize; }
	inline uint GetShadowMapNum()						{ return m_ShadowMapNum; }

	inline float GetSuperSamplingScalar()				{ return m_NumSuperSamples; }
	inline void  SetSuperSamplingScalar(float scalar)   { m_NumSuperSamples = scalar; }

	void SetShadowMapNum(uint num);
	void SetShadowMapSize(uint size);

protected:
	SceneRenderer();
	virtual ~SceneRenderer();

	bool InitialiseGL();
	void InitializeDefaults();

	void BuildFBOs();
	bool ShadersLoad();
	void ShadersSetDefaults();

	void DeferredRenderOpaqueObjects();
	void ForwardRenderTransparentObjects();

	void RenderShadowMaps();

protected:
	Scene*				m_Scene;
	Camera*				m_Camera;

	Shader*				m_ShaderShadow;
	Shader*				m_ShaderForwardLighting;
	Shader*				m_ShaderColNorm; 
	Shader*				m_ShaderLightDir;
	Shader*				m_ShaderCombineLighting;
	Shader*				m_ShaderPresentToWindow;

	Frustum				m_FrameFrustum;
	RenderList*			m_FrameRenderList;

	GLuint	m_ScreenTexWidth, m_ScreenTexHeight;
	GLuint  m_ScreenFBO;
	GLuint  m_ScreenTex[SCREENTEX_MAX];

	
	uint	m_ShadowMapNum;
	uint	m_ShadowMapSize;
	bool	m_ShadowMapsInvalidated;
	GLuint	m_ShadowFBO;
	GLuint  m_ShadowTex[SHADOWMAP_MAX];
	Matrix4 m_ShadowProj[SHADOWMAP_MAX];
	Matrix4 m_ShadowProjView[SHADOWMAP_MAX];
	RenderList* m_ShadowRenderLists[SHADOWMAP_MAX];

	float	m_GammaCorrection; //Monitor Default: 1.0 / 2.2 (Where 2.2 here is the gamma of the monitor which we need to invert before showing)
	Vector3 m_BackgroundColour;
	Vector3 m_AmbientColour;
	Vector3 m_InvLightDirection;
	float   m_SpecularIntensity;
	bool	m_VsyncEnabled;
	float	m_NumSuperSamples;
};