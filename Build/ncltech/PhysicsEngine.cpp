#include "PhysicsEngine.h"
#include "Object.h"
#include "CollisionDetectionSAT.h"
#include "NCLDebug.h"
#include <nclgl\Window.h>


void PhysicsEngine::SetDefaults()
{
	m_DebugDrawFlags = NULL;
	m_IsPaused = false;
	m_UpdateTimestep = 1.0f / 60.f;
	m_UpdateAccum = 0.0f;
	m_Gravity = Vector3(0.0f, -9.81f, 0.0f);
	m_DampingFactor = 0.9999f;
}

PhysicsEngine::PhysicsEngine()
{
	SetDefaults();
}

PhysicsEngine::~PhysicsEngine()
{
	for (PhysicsObject* obj : m_PhysicsObjects)
	{
		delete obj;
	}
	m_PhysicsObjects.clear();

	for (Constraint* c : m_Constraints)
	{
		delete c;
	}
	m_Constraints.clear();

	for (Manifold* m : m_Manifolds)
	{
		delete m;
	}
	m_Manifolds.clear();
}

void PhysicsEngine::AddPhysicsObject(PhysicsObject* obj)
{
	m_PhysicsObjects.push_back(obj);
}

void PhysicsEngine::RemovePhysicsObject(PhysicsObject* obj)
{
	auto found_loc = std::find(m_PhysicsObjects.begin(), m_PhysicsObjects.end(), obj);

	if (found_loc != m_PhysicsObjects.end())
	{
		m_PhysicsObjects.erase(found_loc);
	}
}

void PhysicsEngine::RemoveAllPhysicsObjects()
{
	for (PhysicsObject* obj : m_PhysicsObjects)
	{
		if (obj != NULL)
		{
			if (obj->m_Parent != NULL) obj->m_Parent->m_PhysicsObject = NULL;
			delete obj;
		}
	}
	m_PhysicsObjects.clear();

	for (Constraint* c : m_Constraints)
	{
		delete c;
	}
	m_Constraints.clear();

	for (Manifold* m : m_Manifolds)
	{
		delete m;
	}
	m_Manifolds.clear();
}

void PhysicsEngine::Update(float deltaTime)
{
	const int max_updates_per_frame = 5;

	if (!m_IsPaused)
	{
		m_UpdateAccum += deltaTime;
		for (int i = 0; (m_UpdateAccum >= m_UpdateTimestep) && i < max_updates_per_frame; ++i)
		{
			m_UpdateAccum -= m_UpdateTimestep;
			if (!m_IsPaused) UpdatePhysics(); //Additional check here incase physics was paused mid-update and the contents of the physics need to be displayed
		}

		if (m_UpdateAccum >= m_UpdateTimestep)
		{
			NCLERROR("Physics too slow to run in real time!");
		}
	}
}


void PhysicsEngine::UpdatePhysics()
{
	for (Manifold* m : m_Manifolds)
	{
		delete m;
	}
	m_Manifolds.clear();

	//Check for collisions
	BroadPhaseCollisions();
	NarrowPhaseCollisions();

	//Solve collision constraints
	SolveConstraints();

	//Update movement
	UpdatePhysicsObjects();
}

void PhysicsEngine::DebugRender()
{
	if (m_DebugDrawFlags & DEBUHDRAW_FLAGS_MANIFOLD)
	{
		for (Manifold* m : m_Manifolds)
		{
			m->DebugDraw();
		}
	}

	if (m_DebugDrawFlags & DEBUHDRAW_FLAGS_CONSTRAINT)
	{
		for (Constraint* c : m_Constraints)
		{
			c->DebugDraw();
		}
	}

	if (m_DebugDrawFlags & DEBUHDRAW_FLAGS_COLLISIONVOLUMES)
	{
		for (PhysicsObject* obj : m_PhysicsObjects)
		{
			if (obj->GetCollisionShape() != NULL)
			{
				obj->GetCollisionShape()->DebugDraw(obj);
			}
		}
	}
}


void PhysicsEngine::UpdatePhysicsObjects()
{
	for (PhysicsObject* obj : m_PhysicsObjects)
	{
		UpdatePhysicsObject(obj);
	}
}

void PhysicsEngine::UpdatePhysicsObject(PhysicsObject* obj)
{
	//Semi-Implicit Euler Intergration
	obj->m_LinearVelocity += obj->m_Force * obj->m_InvMass * m_UpdateTimestep;

	//Apply Gravity
	//	Technically this is (m_Gravity / invMass) * invMass * dt
	//	hence the check for invMass being zero is required here even though it appears gravity is not affected by the objects mass
	if (obj->m_InvMass > 0.0f)
		obj->m_LinearVelocity += m_Gravity * m_UpdateTimestep; 

	obj->m_LinearVelocity = obj->m_LinearVelocity * m_DampingFactor;
	obj->m_Position += obj->m_LinearVelocity * m_UpdateTimestep;


	//Angular Rotation
	Vector3 angluarAccel = obj->m_InvInertia * obj->m_Torque;
	obj->m_AngularVelocity = obj->m_AngularVelocity + angluarAccel * m_UpdateTimestep;
	obj->m_AngularVelocity = obj->m_AngularVelocity * m_DampingFactor;

	obj->m_Orientation = obj->m_Orientation + obj->m_Orientation*(obj->m_AngularVelocity*m_UpdateTimestep*0.5f); //Quaternion equiv of the above position calculation
	obj->m_Orientation.Normalise();

	obj->m_wsTransformInvalidated = true; //inform the physics object that it's world space transform is invalid
}

void PhysicsEngine::BroadPhaseCollisions()
{
	m_BroadphaseCollisionPairs.clear();

	PhysicsObject *objA, *objB;

	//This is a brute force broadphase, basically compiling a list to check every object against every other object
	for (size_t i = 0; i < m_PhysicsObjects.size() - 1; ++i)
	{
		for (size_t j = i + 1; j < m_PhysicsObjects.size(); ++j)
		{
			objA = m_PhysicsObjects[i];
			objB = m_PhysicsObjects[j];

			//Check they both have collision shapes
			if (objA->GetCollisionShape() != NULL 
				&& objB->GetCollisionShape() != NULL)
			{
				CollisionPair cp;
				cp.objectA = objA;
				cp.objectB = objB;
				m_BroadphaseCollisionPairs.push_back(cp);
			}
				
		}
	}
}

void PhysicsEngine::NarrowPhaseCollisions()
{
	if (m_BroadphaseCollisionPairs.size() > 0)
	{
		CollisionData colData;
		CollisionDetectionSAT colDetect;

		
		CollisionShape *shapeA, *shapeB;

		for (CollisionPair& cp : m_BroadphaseCollisionPairs)
		{
			shapeA = cp.objectA->GetCollisionShape();
			shapeB = cp.objectB->GetCollisionShape();

			colDetect.BeginNewPair(
				cp.objectA,
				cp.objectB,
				cp.objectA->GetCollisionShape(),
				cp.objectB->GetCollisionShape());


			if (colDetect.AreColliding(&colData))
			{
				//Draw collision data to the window
				if (m_DebugDrawFlags & DEBUHDRAW_FLAGS_COLLISIONNORMALS)
				{
					NCLDebug::DrawPointNDT(colData.pointOnPlane, 0.1f, Vector4(0.5f, 0.5f, 1.0f, 1.0f));
					NCLDebug::DrawThickLineNDT(colData.pointOnPlane, colData.pointOnPlane - colData.normal * colData.penetration, 0.05f, Vector4(0.0f, 0.0f, 1.0f, 1.0f));
				}

				//Check to see if any of the objects have collision callbacks that dont want the objects to physically collide
				bool okA = cp.objectA->FireOnCollisionEvent(cp.objectA, cp.objectB);
				bool okB = cp.objectB->FireOnCollisionEvent(cp.objectA, cp.objectB);

				if (okA && okB)
				{
					//Build full collision manifold that will also handle the collision response between the two objects in the solver stage
					Manifold* manifold = new Manifold(cp.objectA, cp.objectB);
					m_Manifolds.push_back(manifold);
					colDetect.GenContactPoints(manifold);
				}
			}
		}
	}
}



void PhysicsEngine::SolveConstraints()
{
	for (Manifold* m : m_Manifolds)
	{
		m->PreSolverStep(m_UpdateTimestep);
	}

	for (Constraint* c : m_Constraints)
	{
		c->PreSolverStep(m_UpdateTimestep);
	}
	

	for (size_t i = 0; i < SOLVER_ITERATIONS; ++i)
	{
		for (Manifold* m : m_Manifolds)
		{
			m->ApplyImpulse();
		}

		for (Constraint* c : m_Constraints)
		{
			c->ApplyImpulse();
		}
	}
}