
#pragma once
#include <ncltech\ObjectMesh.h>
#include <ncltech\NCLDebug.h>
#include <ncltech\CommonMeshes.h>
#include <ncltech\CuboidCollisionShape.h>
#include <ncltech\PhysicsObject.h>
#include <ncltech\PhysicsEngine.h>
#include <ncltech\SphereCollisionShape.h>
#include <ncltech\REMOVEME_Broadphase.h>

class Player : public ObjectMesh
{
public:
	Player(const std::string& name, GLuint whitetex)
		: ObjectMesh(name) 
	{
		const Vector3 halfdims = Vector3(0.5f, 0.5f, 0.5f);
		const float pull_radius = 2.0f;

		SetMesh(CommonMeshes::Cube(), false);
		SetTexture(whitetex, false);
		SetLocalTransform(Matrix4::Scale(halfdims));
		SetColour(Vector4(1.0f, 0.0f, 0.0f, 1.0f));
		SetBoundingRadius(halfdims.Length());

		CreatePhysicsNode();
		Physics()->SetPosition(Vector3(0.0f, halfdims.y, 0.0f));
		Physics()->SetFriction(1.0f);
		Physics()->SetElasticity(0.8f);
		Physics()->SetCollisionShape(new CuboidCollisionShape(halfdims));
		Physics()->SetInverseMass(1.0f);
		Physics()->SetInverseInertia(Physics()->GetCollisionShape()->BuildInverseInertia(Physics()->GetInverseMass()));


		m_ImpactRadii = new PhysicsObject();
		m_ImpactRadii->SetCollisionShape(new SphereCollisionShape(pull_radius));
		m_ImpactRadii->awake = false;
		m_ImpactRadii->SetOnCollisionCallback([&](PhysicsObject* self_obj, PhysicsObject* other_obj)
		{
			Object* gobj = other_obj->GetAssociatedObject();
			if (gobj != NULL && gobj->GetName() == "Cubicle")
			{
				Vector3 ab_norm = Physics()->GetPosition() - other_obj->GetPosition();
				float ab_len = ab_norm.Length();
				ab_norm = ab_norm * (1.0f / ab_len);

				const float strength = 10.0f;

				other_obj->SetLinearVelocity(- ab_norm * strength / (ab_len + 1.0f));

				other_obj->awake = true;
			}



			return false;
		});
		PhysicsEngine::Instance()->AddPhysicsObject(m_ImpactRadii);

	}

	virtual ~Player()
	{
		if (m_ImpactRadii != NULL)
		{
			//PhysicsEngine::Instance()->RemovePhysicsObject(m_ImpactRadii);
			//delete m_ImpactRadii;
			m_ImpactRadii = NULL;
		}
	}

protected:
	virtual void	OnUpdateObject(float dt) override
	{
		const float mv_speed = 5.f;			//Meters per second^2

		Vector3 force = Vector3(0.0f, 0.0f, 0.0f);
		if (Window::GetKeyboard()->KeyDown(KEYBOARD_UP))
			force.z -= mv_speed;

		if (Window::GetKeyboard()->KeyDown(KEYBOARD_DOWN))
			force.z += mv_speed;

		if (Window::GetKeyboard()->KeyDown(KEYBOARD_LEFT))
			force.x -= mv_speed;
			
		if (Window::GetKeyboard()->KeyDown(KEYBOARD_RIGHT))
			force.x += mv_speed;
			
		Physics()->SetForce(force); //zero'd force if no button down
		Physics()->SetTorque(Vector3::Cross(Vector3(0.0f, 0.5f, 0.0f), force));
		Physics()->awake = true;


		m_ImpactRadii->SetPosition(Physics()->GetPosition());
		m_ImpactRadii->awake = true;
		m_ImpactRadii->SetLinearVelocity(Vector3(0, 0, 0));
		REMOVEME_Broadphase::Instance()->UpdateObject(m_ImpactRadii);
	}


	PhysicsObject* m_ImpactRadii;
};