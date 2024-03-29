
#pragma once

#include <ncltech\Scene.h>
#include <ncltech\SceneManager.h>
#include <ncltech\PhysicsEngine.h>
#include <ncltech\NCLDebug.h>
#include <ncltech\DistanceConstraint.h>
#include <ncltech\CommonUtils.h>

class Phy5_ColManifolds : public Scene
{
public:
	Phy5_ColManifolds(const std::string& friendly_name)
		: Scene(friendly_name)
	{
	}

	const Vector3 ss_pos = Vector3(-5.5f, 1.5f, -5.0f);
	const Vector3 sc_pos = Vector3(4.5f, 1.5f, -5.0f);
	const Vector3 cc_pos = Vector3(-0.5f, 1.5f, 5.0f);

	virtual void OnInitializeScene() override
	{
		SceneManager::Instance()->GetCamera()->SetPosition(Vector3(-3.0f, 10.0f, 10.0f));
		SceneManager::Instance()->GetCamera()->SetPitch(-20.f);
		PhysicsEngine::Instance()->SetDebugDrawFlags(DEBUHDRAW_FLAGS_MANIFOLD);

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


		//Create Sphere-Sphere Manifold Test
		{

			this->AddGameObject(CommonUtils::BuildSphereObject("orbiting_sphere1",
				ss_pos + Vector3(0.75f, 0.0f, 0.0f),	//Position leading to 0.25 meter overlap between spheres
				0.5f,									//Radius
				true,									//Has Physics Object
				0.0f,									//Infinite Mass
				true,									//Has Collision Shape
				true,									//Dragable by the user
				CommonUtils::GenColour(0.45f, 0.5f)));	//Color

			this->AddGameObject(CommonUtils::BuildSphereObject("",
				ss_pos,									//Position
				0.5f,									//Radius
				true,									//Has Physics Object
				0.0f,									//Infinite Mass
				true,									//Has Collision Shape
				true,									//Dragable by the user
				CommonUtils::GenColour(0.5f, 1.0f)));	//Color
		}

		//Create Sphere-Cuboid Manifold Test
		{

			this->AddGameObject(CommonUtils::BuildSphereObject("orbiting_sphere2",
				sc_pos + Vector3(0.9f, 0.0f, 0.0f),		//Position leading to 0.1 meter overlap on faces, and more on diagonals
				0.5f,									//Radius
				true,									//Has Physics Object
				0.0f,									//Infinite Mass
				true,									//Has Collision Shape
				true,									//Dragable by the user
				CommonUtils::GenColour(0.45f, 0.5f)));	//Color

			this->AddGameObject(CommonUtils::BuildCuboidObject("",
				sc_pos,									//Position
				Vector3(0.5f, 0.5f, 0.5f),				//Half dimensions
				true,									//Has Physics Object
				0.0f,									//Infinite Mass
				true,									//Has Collision Shape
				true,									//Dragable by the user
				CommonUtils::GenColour(0.5f, 1.0f)));	//Color
		}

		//Create Cuboid-Cuboid Manifold Test
		{
			
			this->AddGameObject(CommonUtils::BuildCuboidObject("rotating_cuboid1",
				cc_pos + Vector3(0.75f, 0.0f, 0.0f),	//Position leading to 0.25 meter overlap on faces, and more on diagonals
				Vector3(0.5f, 0.5f, 0.5f),				//Half dimensions
				true,									//Has Physics Object
				0.0f,									//Infinite Mass
				true,									//Has Collision Shape
				true,									//Dragable by the user
				CommonUtils::GenColour(0.45f, 0.5f)));	//Color

			this->AddGameObject(CommonUtils::BuildCuboidObject("",
				cc_pos,									//Position
				Vector3(0.5f, 0.5f, 0.5f),				//Half dimensions
				true,									//Has Physics Object
				0.0f,									//Infinite Mass
				true,									//Has Collision Shape
				true,									//Dragable by the user
				CommonUtils::GenColour(0.5f, 1.0f)));	//Color
		}

	}

	float m_AccumTime;
	virtual void OnUpdateScene(float dt) override
	{
		Scene::OnUpdateScene(dt);
		

		//Update Rotating Objects!
		if (Window::GetKeyboard()->KeyDown(KEYBOARD_B))
		{
			m_AccumTime += dt;

			Object* orbiting_sphere1 = this->FindGameObject("orbiting_sphere1");
			if (orbiting_sphere1 != NULL)
			{
				//Move orbiting sphere1 around centre object at 45 degrees per second with an orbiting radius of 75cm
				orbiting_sphere1->Physics()->SetPosition(Vector3(
					ss_pos.x + cos(DegToRad(m_AccumTime * 45.0f)) * 0.75f,
					ss_pos.y,
					ss_pos.z + sin(DegToRad(m_AccumTime * 45.0f)) * 0.75f));
			}

			Object* orbiting_sphere2 = this->FindGameObject("orbiting_sphere2");
			if (orbiting_sphere2 != NULL)
			{
				//Move orbiting sphere2 around centre object at 45 degrees per second with an orbiting radius of 90cm
				orbiting_sphere2->Physics()->SetPosition(Vector3(
					sc_pos.x + cos(DegToRad(m_AccumTime * 45.0f)) * 0.9f,
					sc_pos.y,
					sc_pos.z + sin(DegToRad(m_AccumTime * 45.0f)) * 0.9f));
			}

			Object* rotating_cuboid1 = this->FindGameObject("rotating_cuboid1");
			if (rotating_cuboid1 != NULL)
			{
				rotating_cuboid1->Physics()->SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(1.0f, 0.0f, 0.0f), m_AccumTime * 45.0f));
			}
		}





		uint drawFlags = PhysicsEngine::Instance()->GetDebugDrawFlags();


		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "Physics:");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "     Draw Collision Normals : %s (Press N to toggle)", (drawFlags & DEBUHDRAW_FLAGS_COLLISIONNORMALS) ? "Enabled" : "Disabled");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "     Draw Manifolds : %s (Press M to toggle)", (drawFlags & DEBUHDRAW_FLAGS_MANIFOLD) ? "Enabled" : "Disabled");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "");
		NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "HOLD 'B' TO ROTATE OBJECTS");

		if (Window::GetKeyboard()->KeyTriggered(KEYBOARD_N))
			drawFlags ^= DEBUHDRAW_FLAGS_COLLISIONNORMALS;

		if (Window::GetKeyboard()->KeyTriggered(KEYBOARD_M))
			drawFlags ^= DEBUHDRAW_FLAGS_MANIFOLD;

		PhysicsEngine::Instance()->SetDebugDrawFlags(drawFlags);

	}
};