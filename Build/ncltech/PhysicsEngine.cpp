#include "PhysicsEngine.h"
#include "Object.h"
#include "CollisionDetectionSAT.h"
#include "NCLDebug.h"
#include <nclgl\Window.h>
#include <omp.h>


void PhysicsEngine::SetDefaults()
{
	m_DebugDrawFlags = NULL;
	m_IsPaused = false;
	m_UpdateTimestep = 1.0f / 60.f;
	m_UpdateAccum = 0.0f;
	m_Gravity = Vector3(0.0f, -9.81f, 0.0f);
	m_DampingFactor = 0.999f;
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
	//Apply Gravity
	//	Technically gravity here is calculated by formula: ( m_Gravity / invMass * invMass * dt )
	//	So even though the divide and multiply cancel out, we still need to handle the possibility of divide by zero.
	if (obj->m_InvMass > 0.0f)
		obj->m_LinearVelocity += m_Gravity * m_UpdateTimestep; 


	//Semi-Implicit Euler Intergration
	// - See "Update Position" below
	obj->m_LinearVelocity += obj->m_Force * obj->m_InvMass * m_UpdateTimestep;


	//Apply Velocity Damping
	//	- This removes a tiny bit of energy from the simulation each update to stop slight calculation errors accumulating and adding force from nowhere.
	//  - In it's present form this can be seen as a rough approximation of air resistance, albeit (wrongly?) making the assumption that all objects have the same surface area.
	obj->m_LinearVelocity = obj->m_LinearVelocity * m_DampingFactor;


	//Update Position
	//  - Euler integration, works on the assumption that linearvelocity does not change over time (or changes so slightly it doesnt make a difference).
	//	- In this scenario, gravity /will/ be increasing velocity over time. The in-accuracy of not taking into account of these changes over time can be
	//  - visibly seen in tutorial 1.. and thus how better integration schemes lead to better approximations by taking into account of curvature.
	obj->m_Position += obj->m_LinearVelocity * m_UpdateTimestep;


	//Angular Rotation
	//  - These are the exact same calculations as the three lines above, except for rotations rather than positions.
	//		- Mass		-> Torque
	//		- Velocity  -> Rotational Velocity
	//		- Position  -> Orientation  
	obj->m_AngularVelocity += obj->m_InvInertia * obj->m_Torque * m_UpdateTimestep;


	//Apply Velocity Damping
	obj->m_AngularVelocity = obj->m_AngularVelocity * m_DampingFactor;


	//Update Orientation
	// - This is slightly different calculation due to the wierdness of quaternions. This, along with the normalise function to enforce it as a rotation, is the best way
	// - to update the quaternion based on a angular velocity, and thats all you need to know. If you are interested in it's derivation, there is lots of stuff online about it.
	obj->m_Orientation = obj->m_Orientation + obj->m_Orientation * ( obj->m_AngularVelocity * m_UpdateTimestep * 0.5f);
	obj->m_Orientation.Normalise();


	//Finally invalidate the world-transform matrix. 
	// - The next time it is requested now, it will be rebuilt from scratch with the new position/orientation we set above.
	obj->m_wsTransformInvalidated = true; 
}

void PhysicsEngine::BroadPhaseCollisions()
{
	m_BroadphaseCollisionPairs.clear();

	PhysicsObject *objA, *objB;
	//	The broadphase needs to build a list of all potentially colliding objects in the world,
	//	which then get accurately assesed in narrowphase. If this is too coarse then the system slows down with
	//	the complexity of narrowphase collision checking, if this is too fine then collisions may be missed.


	//	Brute force approach.
	//  - Assumes every object could collide with every other object even if they are on other sides of the world.
	for (size_t i = 0; i < m_PhysicsObjects.size() - 1; ++i)
	{
		for (size_t j = i + 1; j < m_PhysicsObjects.size(); ++j)
		{
			objA = m_PhysicsObjects[i];
			objB = m_PhysicsObjects[j];

			//Check they both atleast have collision shapes
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

		CollisionData colData;				//Collision data to pass between detection and manifold generation stages.
		CollisionDetectionSAT colDetect;	//Collision Detection Algorithm

		for (size_t i = 0; i < m_BroadphaseCollisionPairs.size(); ++i)
		{
			CollisionPair& cp = m_BroadphaseCollisionPairs[i];

			CollisionShape *shapeA = cp.objectA->GetCollisionShape();
			CollisionShape *shapeB = cp.objectB->GetCollisionShape();

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
					Manifold* manifold = new Manifold();
					manifold->Initiate(cp.objectA, cp.objectB);
					colDetect.GenContactPoints(manifold);
					m_Manifolds.push_back(manifold);
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
	
	for (int i = 0; i < SOLVER_ITERATIONS; ++i)
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