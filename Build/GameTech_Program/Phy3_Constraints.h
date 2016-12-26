
#pragma once

#include <ncltech\Scene.h>
#include <ncltech\SceneManager.h>
#include <ncltech\PhysicsEngine.h>
#include <ncltech\NCLDebug.h>
#include <ncltech\DistanceConstraint.h>
#include <ncltech\CommonUtils.h>

class Phy3_Constraints : public Scene
{
public:
	Phy3_Constraints(const std::string& friendly_name)
		: Scene(friendly_name)
	{
	}

	virtual void OnInitializeScene() override
	{
		SceneManager::Instance()->GetCamera()->SetPosition(Vector3(-3.0f, 10.0f, 10.0f));
		SceneManager::Instance()->GetCamera()->SetPitch(-20.f);
		PhysicsEngine::Instance()->SetDebugDrawFlags(DEBUHDRAW_FLAGS_CONSTRAINT);

		//Create Ground (..why not?)
		Object* ground = CommonUtils::BuildCuboidObject(
			"Ground",
			Vector3(0.0f, 0.0f, 0.0f),
			Vector3(20.0f, 1.0f, 20.0f),
			false,
			0.0f,
			false,
			false,
			Vector4(0.2f, 0.5f, 1.0f, 1.0f));

		this->AddGameObject(ground);


		//Create Hanging Ball
		{
			Object* handle = CommonUtils::BuildSphereObject("",
				Vector3(-7.f, 7.f, -5.0f),				//Position
				0.5f,									//Radius
				true,									//Has Physics Object
				0.0f,									//Infinite Mass
				false,									//No Collision Shape Yet
				true,									//Dragable by the user
				CommonUtils::GenColour(0.45f, 0.5f));	//Color

			Object* ball = CommonUtils::BuildSphereObject("",
				Vector3(-4.f, 7.f, -5.0f),				//Position
				0.5f,									//Radius
				true,									//Has Physics Object
				1.0f,									// Inverse Mass = 1 / 1kg mass
				false,									//No Collision Shape Yet
				true,									//Dragable by the user
				CommonUtils::GenColour(0.5f, 1.0f));	//Color

			this->AddGameObject(handle);
			this->AddGameObject(ball);

			//Add distance constraint between the two objects
			DistanceConstraint* constraint = new DistanceConstraint(
				handle->Physics(),					//Physics Object A
				ball->Physics(),					//Physics Object B
				handle->Physics()->GetPosition(),	//Attachment Position on Object A	-> Currently the centre
				ball->Physics()->GetPosition());	//Attachment Position on Object B	-> Currently the centre  
			PhysicsEngine::Instance()->AddConstraint(constraint);
		}




		//Create Hanging Cube (Attached by corner)
		{
			Object* handle = CommonUtils::BuildSphereObject("",
				Vector3(4.f, 7.f, -5.0f),				//Position
				0.5f,									//Radius
				true,									//Has Physics Object
				0.0f,									//Infinite Mass
				false,									//No Collision Shape Yet
				true,									//Dragable by the user
				CommonUtils::GenColour(0.55f, 0.5f));	//Color

			Object* cube = CommonUtils::BuildCuboidObject("",
				Vector3(7.f, 7.f, -5.0f),				//Position
				Vector3(0.5f, 0.5f, 0.5f),				//Half Dimensions
				true,									//Has Physics Object
				1.0f,									//Infinite Mass
				false,									//No Collision Shape Yet
				true,									//Dragable by the user
				CommonUtils::GenColour(0.6f, 1.0f));	//Color

			this->AddGameObject(handle);
			this->AddGameObject(cube);

			PhysicsEngine::Instance()->AddConstraint(new DistanceConstraint(
				handle->Physics(),													//Physics Object A
				cube->Physics(),													//Physics Object B
				handle->Physics()->GetPosition(),									//Attachment Position on Object A	-> Currently the far right edge
				cube->Physics()->GetPosition() + Vector3(-0.5f, -0.5f, -0.5f)));	//Attachment Position on Object B	-> Currently the far left edge 
		}


	}

	virtual void OnUpdateScene(float dt) override
	{
		Scene::OnUpdateScene(dt);

		bool drawConstraints = PhysicsEngine::Instance()->GetDebugDrawFlags() & DEBUHDRAW_FLAGS_CONSTRAINT;

		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "Physics:");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "     Draw Constraints : %s (Press C to toggle)", drawConstraints ? "Enabled" : "Disabled");

		if (Window::GetKeyboard()->KeyTriggered(KEYBOARD_C))
		{
			drawConstraints = !drawConstraints;
		}

		PhysicsEngine::Instance()->SetDebugDrawFlags(drawConstraints ? DEBUHDRAW_FLAGS_CONSTRAINT : NULL);

	}
};