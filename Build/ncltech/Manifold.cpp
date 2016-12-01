#include "Manifold.h"
#include <nclgl\Matrix3.h>
#include "NCLDebug.h"
#include "PhysicsEngine.h"

#define persistentThresholdSq 0.025f

typedef std::list<ContactPoint> ContactList;
typedef ContactList::iterator ContactListItr;

Manifold::Manifold() 
	: m_pNodeA(NULL)
	, m_pNodeB(NULL)
{
}

Manifold::~Manifold()
{

}

void Manifold::Initiate(PhysicsObject* nodeA, PhysicsObject* nodeB)
{
	m_vContacts.clear();

	m_pNodeA = nodeA;
	m_pNodeB = nodeB;
}

void Manifold::ApplyImpulse()
{
	for (auto& contact : m_vContacts)
	{
		SolveContactPoint(contact);
	}
}


void Manifold::SolveContactPoint(ContactPoint& c)
{
	if (m_pNodeA->GetInverseMass() + m_pNodeB->GetInverseMass() == 0.0f)
		return;

	Vector3 r1 = c.relPosA;
	Vector3 r2 = c.relPosB;

	Vector3 v0 = m_pNodeA->GetLinearVelocity() + Vector3::Cross(m_pNodeA->GetAngularVelocity(), r1);
	Vector3 v1 = m_pNodeB->GetLinearVelocity() + Vector3::Cross(m_pNodeB->GetAngularVelocity(), r2);
	Vector3 dv = v0 - v1;
	Vector3 normal = c.collisionNormal;
	

	//Collision Resolution
	{
		float cMass = (m_pNodeA->GetInverseMass() + m_pNodeB->GetInverseMass())
			+ Vector3::Dot(normal,
				Vector3::Cross(m_pNodeA->GetInverseInertia() * Vector3::Cross(r1, normal), r1)
				+ Vector3::Cross(m_pNodeB->GetInverseInertia() * Vector3::Cross(r2, normal), r2)
				);

		float b = 0.0f;
		{
			float b_scalar = 0.3f;
			float b_slop = 0.001f;

			float p_slop = min(c.collisionPenetration + b_slop, 0.0f);
			b = -(b_scalar / PhysicsEngine::Instance()->GetDeltaTime() * p_slop);
		}

		float jn = -(Vector3::Dot(dv, normal) + b + c.elatisity_term) / cMass;
		jn = min(jn, 0.0f);

		float oldSumImpulseContact = c.sumImpulseContact;
		c.sumImpulseContact = min(c.sumImpulseContact + jn, 0.0f);
		jn = c.sumImpulseContact - oldSumImpulseContact;

		m_pNodeA->SetLinearVelocity(m_pNodeA->GetLinearVelocity() + normal *(jn * m_pNodeA->GetInverseMass()));
		m_pNodeB->SetLinearVelocity(m_pNodeB->GetLinearVelocity() - normal * (jn * m_pNodeB->GetInverseMass()));

		m_pNodeA->SetAngularVelocity(m_pNodeA->GetAngularVelocity() + m_pNodeA->GetInverseInertia() * Vector3::Cross(r1, normal * jn));
		m_pNodeB->SetAngularVelocity(m_pNodeB->GetAngularVelocity() - m_pNodeB->GetInverseInertia() * Vector3::Cross(r2, normal * jn));
	}

	//v0 = m_pNodeA->GetLinearVelocity() + Vector3::Cross(m_pNodeA->GetAngularVelocity(), r1);
	//v1 = m_pNodeB->GetLinearVelocity() + Vector3::Cross(m_pNodeB->GetAngularVelocity(), r2);
	//dv = v0 - v1;

	//Friction
	{
		Vector3 tangent = dv - normal * Vector3::Dot(dv, normal);
		float tangent_len = tangent.Length();
		if (tangent_len > 0.001f)
		{
			tangent = tangent * (1.0f / tangent_len);

			float cMass = (m_pNodeA->GetInverseMass() + m_pNodeB->GetInverseMass())
				+ Vector3::Dot(tangent,
					Vector3::Cross(m_pNodeA->GetInverseInertia() * Vector3::Cross(r1, tangent), r1)
					+ Vector3::Cross(m_pNodeB->GetInverseInertia() * Vector3::Cross(r2, tangent), r2)
					);

			float friction_coef = sqrtf(m_pNodeA->GetFriction() * m_pNodeB->GetFriction());
			float tn = -friction_coef * (Vector3::Dot(dv, tangent)) / cMass;

			float oldTangentImpulse = c.sumImpulseFriction;
			float maxTn = friction_coef * c.sumImpulseContact;
			c.sumImpulseFriction = max(min(oldTangentImpulse + tn, -maxTn), maxTn);
			tn = c.sumImpulseFriction - oldTangentImpulse;

			m_pNodeA->SetLinearVelocity(m_pNodeA->GetLinearVelocity() + tangent * (tn * m_pNodeA->GetInverseMass()));
			m_pNodeB->SetLinearVelocity(m_pNodeB->GetLinearVelocity() - tangent * (tn * m_pNodeB->GetInverseMass()));

			m_pNodeA->SetAngularVelocity(m_pNodeA->GetAngularVelocity() + m_pNodeA->GetInverseInertia() * Vector3::Cross(r1, tangent * tn));
			m_pNodeB->SetAngularVelocity(m_pNodeB->GetAngularVelocity() - m_pNodeB->GetInverseInertia() * Vector3::Cross(r2, tangent * tn));
			
		}
	}
}

void Manifold::PreSolverStep(float dt)
{
	for (ContactPoint& contact : m_vContacts)
	{
		UpdateConstraint(contact);
	}
}

void Manifold::UpdateConstraint(ContactPoint& contact)
{
	//Reset total impulse forces computed this physics timestep 
	contact.sumImpulseContact = 0.0f;
	contact.sumImpulseFriction = 0.0f;

	{
		const float elasticity = sqrtf(m_pNodeA->GetElasticity() * m_pNodeB->GetElasticity());

		float elasticity_term = elasticity * Vector3::Dot(contact.collisionNormal,
			m_pNodeA->GetLinearVelocity()
			+ Vector3::Cross(contact.relPosA, m_pNodeA->GetAngularVelocity())
			- m_pNodeB->GetLinearVelocity()
			- Vector3::Cross(contact.relPosB, m_pNodeB->GetAngularVelocity()));

		if (elasticity_term < 0.0f)
		{
			contact.elatisity_term = 0.0f;
		}
		else
		{
			const float elasticity_slop = 0.2f;
			if (elasticity_term < elasticity_slop)
				elasticity_term = 0.0f;

			contact.elatisity_term = elasticity_term;
		}
	}
}

void Manifold::AddContact(const Vector3& globalOnA, const Vector3& globalOnB, const Vector3& _normal, const float& _penetration)
{
	//Get relative offsets from each object centre of mass
	// Used to compute rotational velocity at the point of contact.
	Vector3 r1 = (globalOnA - m_pNodeA->GetPosition());
	Vector3 r2 = (globalOnB - m_pNodeB->GetPosition());

	//Create our new contact descriptor
	ContactPoint contact;
	contact.relPosA = r1;
	contact.relPosB = r2;
	contact.collisionNormal = _normal;
	contact.collisionPenetration = _penetration;


	//Check to see if we already contain a contact point almost in that location
	const float min_allowed_dist_sq = 0.2f * 0.2f;
	bool should_add = true;
	for (auto itr = m_vContacts.begin(); itr != m_vContacts.end(); )
	{
		Vector3 ab = itr->relPosA - contact.relPosA;
		float distsq = Vector3::Dot(ab, ab);


		//Choose the contact point with the largest penetration and therefore the largest collision response
		if (distsq < min_allowed_dist_sq)
		{
			if (itr->collisionPenetration > contact.collisionPenetration)
			{
				itr = m_vContacts.erase(itr);
				continue;
			}
			else
			{
				should_add = false;
			}
			
		}
		
		itr++;
	}


	
	if (should_add)
		m_vContacts.push_back(contact);
}

void Manifold::DebugDraw() const
{
	if (m_vContacts.size() > 0)
	{
		//Loop around all contact points and draw them all as a line-fan
		Vector3 globalOnA1 = m_pNodeA->GetPosition() + m_vContacts.back().relPosA;
		for (const ContactPoint& contact : m_vContacts)
		{
			Vector3 globalOnA2 = m_pNodeA->GetPosition() + contact.relPosA;
			Vector3 globalOnB = m_pNodeB->GetPosition() + contact.relPosB;

			//Draw line to form area given by all contact points
			NCLDebug::DrawThickLineNDT(globalOnA1, globalOnA2, 0.02f, Vector4(0.0f, 1.0f, 0.0f, 1.0f));

			//Draw descriptors for indivdual contact point
			NCLDebug::DrawPointNDT(globalOnA2, 0.05f, Vector4(0.0f, 0.5f, 0.0f, 1.0f));
			NCLDebug::DrawThickLineNDT(globalOnB, globalOnA2, 0.01f, Vector4(1.0f, 0.0f, 1.0f, 1.0f));

			globalOnA1 = globalOnA2;
		}
	}
}