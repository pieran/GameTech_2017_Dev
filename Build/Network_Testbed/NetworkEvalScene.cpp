#include "NetworkEvalScene.h"
#include <ncltech\ObjectMesh.h>
#include <ncltech\CommonMeshes.h>
#include <ncltech\CuboidCollisionShape.h>
#include <ncltech\PhysicsEngine.h>
#include <ncltech\NCLDebug.h>
#include "Player.h"

NetworkEvalScene::NetworkEvalScene(const std::string& friendly_name)
	: Scene(friendly_name)
	, m_TexWhite(NULL)
{
	glGenTextures(1, &m_TexWhite);
	glBindTexture(GL_TEXTURE_2D, m_TexWhite);
	int white_pixel = 0xFFFFFFFF;
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, &white_pixel);
}

NetworkEvalScene::~NetworkEvalScene()
{
	if (m_TexWhite)
	{
		glDeleteTextures(1, &m_TexWhite);
		m_TexWhite = NULL;
	}
}

void NetworkEvalScene::OnInitializeScene()
{
	SceneManager::Instance()->GetCamera()->SetPosition(Vector3(-7.0f, 25.0f, 30.0f));
	SceneManager::Instance()->GetCamera()->SetYaw(-10.f);
	SceneManager::Instance()->GetCamera()->SetPitch(-40.f);

	//Create Ground
	this->AddGameObject(BuildCuboidObject("Ground", Vector3(0.0f, 1.0f, 0.0f), Vector3(40.0f, 1.0f, 40.0f), 0.0f, Vector4(0.2f, 0.5f, 1.0f, 1.0f)));

	//Create Player
	Object* player = new Player("Player", m_TexWhite);
	player->Physics()->SetPosition(Vector3(0.0f, 3.0f, 17.0f));
	this->AddGameObject(player);

	//Create Little Cubes	
	for (int z = -15; z < 15; ++z)
	{
		for (int x = -15; x <= 15; ++x)
		{
			Object* cube = BuildCuboidObject("Cubicle", Vector3(x - 0.25f, 2.2f, z - 0.25f), Vector3(0.2f, 0.2f, 0.2f), 10.0f, Vector4(0.6f, 0.6f, 0.6f, 1.0f));
			cube->Physics()->SetFriction(1.0f);
			cube->Physics()->SetElasticity(0.9f);
			this->AddGameObject(cube);
		}
	}
}

Object* NetworkEvalScene::BuildCuboidObject(const std::string& name, const Vector3& pos, const Vector3& halfdims, float invmass, const Vector4& colour)
{
	ObjectMesh* cuboid = new ObjectMesh(name);

	cuboid->SetMesh(CommonMeshes::Cube(), false);
	cuboid->SetTexture(m_TexWhite, false);
	cuboid->SetLocalTransform(Matrix4::Scale(halfdims));
	cuboid->SetColour(colour);
	cuboid->SetBoundingRadius(halfdims.Length());

	cuboid->CreatePhysicsNode();
	cuboid->Physics()->SetPosition(pos);
	cuboid->Physics()->SetFriction(0.1f);
	cuboid->Physics()->SetElasticity(0.7f);
	cuboid->Physics()->SetCollisionShape(new CuboidCollisionShape(halfdims));

	cuboid->Physics()->SetInverseMass(invmass);
	cuboid->Physics()->SetInverseInertia(cuboid->Physics()->GetCollisionShape()->BuildInverseInertia(cuboid->Physics()->GetInverseMass()));

	return cuboid;
}

void NetworkEvalScene::OnUpdateScene(float dt)
{
	Scene::OnUpdateScene(dt);

	uint drawFlags = PhysicsEngine::Instance()->GetDebugDrawFlags();

	NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "Physics:");
	NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "     Draw Collision Volumes : %s (Press C to toggle)", (drawFlags & DEBUHDRAW_FLAGS_COLLISIONVOLUMES) ? "Enabled" : "Disabled");
	NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "     Draw Collision Normals : %s (Press N to toggle)", (drawFlags & DEBUHDRAW_FLAGS_COLLISIONNORMALS) ? "Enabled" : "Disabled");
	NCLDebug::AddStatusEntry(Vector4(1.0f, 0.9f, 0.8f, 1.0f), "     Draw Manifolds : %s (Press M to toggle)", (drawFlags & DEBUHDRAW_FLAGS_MANIFOLD) ? "Enabled" : "Disabled");


	if (Window::GetKeyboard()->KeyTriggered(KEYBOARD_C))
		drawFlags ^= DEBUHDRAW_FLAGS_COLLISIONVOLUMES;

	if (Window::GetKeyboard()->KeyTriggered(KEYBOARD_N))
		drawFlags ^= DEBUHDRAW_FLAGS_COLLISIONNORMALS;

	if (Window::GetKeyboard()->KeyTriggered(KEYBOARD_M))
		drawFlags ^= DEBUHDRAW_FLAGS_MANIFOLD;

	PhysicsEngine::Instance()->SetDebugDrawFlags(drawFlags);
}