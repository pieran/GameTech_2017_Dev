
#pragma once

#include <ncltech\Scene.h>
#include <ncltech\SceneManager.h>

class NetworkEvalScene : public Scene
{
public:
	NetworkEvalScene(const std::string& friendly_name);
	virtual ~NetworkEvalScene();

	virtual void OnInitializeScene() override;
	virtual void OnUpdateScene(float dt) override;

private:
	Object* BuildCuboidObject(
		const std::string& name,
		const Vector3& pos,
		const Vector3& halfdims,
		float invmass,
		const Vector4& colour);

	GLuint m_TexWhite;
};