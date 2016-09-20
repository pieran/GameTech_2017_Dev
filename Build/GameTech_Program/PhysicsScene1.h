
#pragma once

#include <ncltech\Scene.h>
#include <ncltech\SceneManager.h>
#include "Utils.h"

class PhysicsScene1 : public Scene
{
public:
	PhysicsScene1(const std::string& friendly_name)
		: Scene(friendly_name)
	{}

	virtual void OnInitializeScene() override
	{
		SceneManager::Instance()->GetCamera()->SetPosition(Vector3(-3.0f, 10.0f, 15.0f));
		SceneManager::Instance()->GetCamera()->SetYaw(-10.f);
		SceneManager::Instance()->GetCamera()->SetPitch(-30.f);

		//Create Ground
		this->AddGameObject(Utils::BuildCuboidObject("Ground", Vector3(0.0f, -1.0f, 0.0f), Vector3(20.0f, 1.0f, 20.0f), 0.0f, true, false, Vector4(0.2f, 0.5f, 1.0f, 1.0f)));

		//Create Cubes Triangle Stack
		const int stack_height = 6;
		for (int y = 0; y < stack_height; ++y)
		{
			for (int x = 0; x <= y; ++x)
			{
				Vector4 colour = Utils::GenColour(y * 0.2f, 1.0f);
				Object* cube = Utils::BuildCuboidObject("", Vector3(x - y * 0.5f, -0.5f - y + stack_height, 0.0f), Vector3(0.5f, 0.5f, 0.5f), 1.f, true, true, colour);
				cube->Physics()->SetFriction(1.0f);
				this->AddGameObject(cube);
			}
		}
	}
};