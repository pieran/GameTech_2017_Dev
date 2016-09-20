#include "Manifold.h"
#include <nclgl\Matrix3.h>
#include <ncltech\NCLDebug.h>
#include "PhysicsEngine.h"

#define persistentThresholdSq 0.025f

typedef std::list<Contact> ContactList;
typedef ContactList::iterator ContactListItr;

Manifold::Manifold(PhysicsObject* nodeA, PhysicsObject* nodeB) : m_NodeA(nodeA), m_NodeB(nodeB)
{
}

Manifold::~Manifold()
{

}

void Manifold::ApplyImpulse()
{
	float softness = (m_NodeA->GetInverseMass() + m_NodeB->GetInverseMass()) / m_Contacts.size();
	for (Contact& contact : m_Contacts)
	{
		SolveContactPoint(contact);
		/*contact.normal.softness = softness;
		contact.normal.ApplyImpulse();

		//float friction_limit = sqrtf(2.0f) * contact.normal.impulseSum;
		float friction_limit = contact.normal.impulseSum;

		contact.friction1.impulseSumMin = -friction_limit;
		contact.friction1.impulseSumMax = friction_limit;
		
		contact.friction2.impulseSumMin = -friction_limit;
		contact.friction2.impulseSumMax = friction_limit;

		contact.friction1.ApplyImpulse();
		contact.friction2.ApplyImpulse();*/
	}
}

void Manifold::SolveContactPoint(Contact& c)
{
	//virtual void ApplyImpulse(PhysicsObject* objA, PhysicsObject* objB,
	//	const Vector3& globalOnA, const Vector3& globalOnB,
	//	const Vector3& colNormal, float colPenetration)
	//{
		if (m_NodeA->GetInverseMass() + m_NodeB->GetInverseMass() == 0.0f)
			return;

		Vector3 r1 = c.relPosA;
		Vector3 r2 = c.relPosB;
		
		Vector3 v0 = m_NodeA->GetLinearVelocity() + Vector3::Cross(m_NodeA->GetAngularVelocity(), r1);
		Vector3 v1 = m_NodeB->GetLinearVelocity() + Vector3::Cross(m_NodeB->GetAngularVelocity(), r2);

		Vector3 normal = c.collisionNormal;
		Vector3 dv = v0 - v1;
		//if (Vector3::Dot(dv, normal) < 0.01f)
		//	return;

		{
			float constraintMass = (m_NodeA->GetInverseMass() + m_NodeB->GetInverseMass()) +
				Vector3::Dot(normal,
				Vector3::Cross(m_NodeA->GetInverseInertia()*Vector3::Cross(r1, normal), r1) +
				Vector3::Cross(m_NodeB->GetInverseInertia()*Vector3::Cross(r2, normal), r2));

			//Baumgarte Offset (Adds energy to the system to counter slight solving errors that accumulate over time - known as 'constraint drift')
			float b = 0.0f;
			{
				float distance_offset = c.collisionPenetration;
				float baumgarte_scalar = 0.1f;
				float baumgarte_slop = 0.01f;
				float penetration_slop = min(c.collisionPenetration + baumgarte_slop, 0.0f);
				b = -(baumgarte_scalar / PhysicsEngine::Instance()->GetDeltaTime()) * penetration_slop;
			}

			
			float jn = -(Vector3::Dot(dv, normal) + b + c.elatisity_term) / constraintMass;

			//As this is run multiple times per frame,
			// we need to clamp the total amount of movement to be positive
			// otherwise in some scenarios we may end up solving the constraint backwards 
			// to compensate for collisions with other objects
			float oldSumImpulseContact = c.sumImpulseContact;
			c.sumImpulseContact = min(c.sumImpulseContact + jn, 0.0f);
			float real_jn = c.sumImpulseContact - oldSumImpulseContact;


			m_NodeA->SetLinearVelocity(m_NodeA->GetLinearVelocity() + normal*(real_jn*m_NodeA->GetInverseMass()));
			m_NodeB->SetLinearVelocity(m_NodeB->GetLinearVelocity() - normal*(real_jn*m_NodeB->GetInverseMass()));

			m_NodeA->SetAngularVelocity(m_NodeA->GetAngularVelocity() + m_NodeA->GetInverseInertia()* Vector3::Cross(r1, normal * real_jn));
			m_NodeB->SetAngularVelocity(m_NodeB->GetAngularVelocity() - m_NodeB->GetInverseInertia()* Vector3::Cross(r2, normal * real_jn));
		}


		//Friction
		{

			Vector3 tangent = dv - normal * Vector3::Dot(dv, normal);
			float tangent_len = tangent.Length();

			if (tangent_len > 0.001f)
			{
				tangent = tangent * (1.0f / tangent_len);

				float tangDiv = (m_NodeA->GetInverseMass() + m_NodeB->GetInverseMass()) +
					Vector3::Dot(tangent,
					Vector3::Cross(m_NodeA->GetInverseInertia()* Vector3::Cross(r1, tangent), r1) +
					Vector3::Cross(m_NodeB->GetInverseInertia()* Vector3::Cross(r2, tangent), r2));

				float frictionCoef = (m_NodeA->GetFriction() * m_NodeB->GetFriction()) / m_Contacts.size();
				float jt = -1 * frictionCoef * Vector3::Dot(dv, tangent) / tangDiv;

				//Stop Friction from ever being more than frictionCoef * normal resolution impulse
				float oldImpulseFriction = c.sumImpulseFriction.Length();
				c.sumImpulseFriction = c.sumImpulseFriction + tangent * jt;
				float diff = abs(c.sumImpulseContact) / c.sumImpulseFriction.Length() *frictionCoef;
				float real_jt = jt * min(diff, 1.0f);


				m_NodeA->SetLinearVelocity(m_NodeA->GetLinearVelocity() + tangent*(real_jt*m_NodeA->GetInverseMass()));
				m_NodeB->SetLinearVelocity(m_NodeB->GetLinearVelocity() - tangent*(real_jt*m_NodeB->GetInverseMass()));

				m_NodeA->SetAngularVelocity(m_NodeA->GetAngularVelocity() + m_NodeA->GetInverseInertia()* Vector3::Cross(r1, tangent * real_jt));
				m_NodeB->SetAngularVelocity(m_NodeB->GetAngularVelocity() - m_NodeB->GetInverseInertia()* Vector3::Cross(r2, tangent * real_jt));
			}
		}
}

void Manifold::PreSolverStep(float dt)
{
	for (Contact& contact : m_Contacts)
	{


		UpdateConstraint(contact);
	}
}

void Manifold::UpdateConstraint(Contact& contact)
{

	Vector3 r1 = contact.relPosA ;
	Vector3 r2 = contact.relPosB;


	Vector3 v1 = m_NodeA->GetLinearVelocity() + Vector3::Cross(m_NodeA->GetAngularVelocity(), r1);
	Vector3 v2 = m_NodeB->GetLinearVelocity() + Vector3::Cross(m_NodeB->GetAngularVelocity(), r2);

	contact.sumImpulseContact = 0.0f;
	contact.sumImpulseFriction = Vector3(0.0f, 0.0f, 0.0f);
	//Elasticity
	{
		const float elasticity = m_NodeA->GetElasticity() * m_NodeB->GetElasticity();
		const float elasticity_slop = 0.5f;

		float elatisity_term = elasticity * Vector3::Dot(contact.collisionNormal,
			m_NodeA->GetLinearVelocity()
			+ Vector3::Cross(r1, m_NodeA->GetAngularVelocity())
			- m_NodeB->GetLinearVelocity()
			- Vector3::Cross(r2, m_NodeB->GetAngularVelocity())
			);

		contact.elatisity_term = max(elatisity_term - elasticity_slop, 0.0f);
	}



	Vector3 dv = v2 - v1;
	Vector3 tangent1 = dv - (contact.collisionNormal * Vector3::Dot(dv, contact.collisionNormal));
	if (Vector3::Dot(tangent1, tangent1) < 0.001f)
	{
		tangent1 = Vector3::Cross(contact.collisionNormal, Vector3(1, 0, 0));
		if (Vector3::Dot(tangent1, tangent1) < 0.001f)
		{
			tangent1 = Vector3::Cross(contact.collisionNormal, Vector3(0, 0, 1));
		}
	}
	tangent1.Normalise();

	Vector3 tangent2 = Vector3::Cross(contact.collisionNormal, tangent1);
	tangent2.Normalise();




	//Normal Collision Constraint
	/*float b = 0.0f;

	//Baumgarte Offset
	{
		const float baumgarte_scalar = 0.2f;
		const float baumgarte_slop = 0.01f;
		float penetration_slop = min(contact.collisionPenetration + baumgarte_slop, 0.0f);
		b += (baumgarte_scalar / PhysicsEngine::Instance()->GetDeltaTime()) * penetration_slop;
	}

	//Elasticity
	{
		const float elasticity = 0.8f;
		const float elasticity_slop = 0.5f;

		float elatisity_term = elasticity * Vector3::Dot(contact.collisionNormal,
			-m_NodeA->GetLinearVelocity()
			- Vector3::Cross(r1, m_NodeA->GetAngularVelocity())
			+ m_NodeB->GetLinearVelocity()
			+ Vector3::Cross(r2, m_NodeB->GetAngularVelocity())
			);

		b += min(elatisity_term + elasticity_slop, 0.0f);
	}


	//Friction Collision Constraints
	float friction = (m_NodeA->GetFriction() * m_NodeB->GetFriction());


	contact.normal = Constraint(m_NodeA, m_NodeB,
		-contact.collisionNormal,
		Vector3::Cross(-r1, contact.collisionNormal),
		contact.collisionNormal,
		Vector3::Cross(r2, contact.collisionNormal),
		b);

	contact.normal.impulseSumMin = 0.0f;

	contact.friction1 = Constraint(m_NodeA, m_NodeB,
		-tangent1 * friction,
		Vector3::Cross(-r1, tangent1) * friction,
		tangent1 * friction,
		Vector3::Cross(r2, tangent1) * friction,
		0.0f);

	contact.friction2 = Constraint(m_NodeA, m_NodeB,
		-tangent2 * friction,
		Vector3::Cross(-r1, tangent2) * friction,
		tangent2 * friction,
		Vector3::Cross(r2, tangent2) * friction,
		0.0f);*/

}

void Manifold::AddContact(const Vector3& globalOnA, const Vector3& globalOnB, const Vector3& normal, const float& penetration)
{
	Vector3 r1 = (globalOnA - m_NodeA->GetPosition());
	Vector3 r2 = (globalOnB - m_NodeB->GetPosition());


	Contact contact;
	contact.relPosA = r1;
	contact.relPosB = r2;
	contact.collisionNormal = normal;
	contact.collisionPenetration = penetration;

	m_Contacts.push_back(contact);
}

void Manifold::DebugDraw() const
{
	if (m_Contacts.size() > 0)
	{
		const Contact& c = m_Contacts.back();

		Vector3 globalOnA1 = m_NodeA->GetPosition() +  m_Contacts.back().relPosA;
		for (const Contact& contact : m_Contacts)
		{
			Vector3 globalOnA2 = m_NodeA->GetPosition() + contact.relPosA;
			Vector3 globalOnB = m_NodeB->GetPosition() + contact.relPosB;

			NCLDebug::DrawThickLine(globalOnA1, globalOnA2, 0.02f, Vector4(0.0f, 1.0f, 0.0f, 1.0f));
			NCLDebug::DrawPoint(globalOnA2, 0.05f, Vector4(0.0f, 0.5f, 0.0f, 1.0f));

			NCLDebug::DrawThickLine(globalOnB, globalOnA2, 0.01f, Vector4(1.0f, 0.0f, 1.0f, 1.0f));

			globalOnA1 = globalOnA2;
		}
	}
}