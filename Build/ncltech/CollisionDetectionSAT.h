
#pragma once
#include "PhysicsObject.h"
#include "CollisionShape.h"
#include "Manifold.h"

struct CollisionData
{
	//The direction of collision from obj1 to obj2
	Vector3		normal;

	//The amount the objects penetrate eachother (negative overlap distance)
	float		penetration;

	//The point on obj1 where they overlap
	Vector3		pointOnPlane;
};

class CollisionDetectionSAT
{
public:
	CollisionDetectionSAT();

	void BeginNewPair(
		PhysicsObject* obj1,
		PhysicsObject* obj2,
		CollisionShape* shape1,
		CollisionShape* shape2);

	bool AreColliding(CollisionData* out_coldata = NULL);

	void GenContactPoints(Manifold* out_manifold);
	
protected:
	//<---- SAT ---->
	void FindAllPossibleCollisionAxes();
	bool CheckCollisionAxis(const Vector3& axis, CollisionData* coldata);


	
	//<---- UTILS ---->
	Vector3 GetClosestPoint(const Vector3& pos, std::vector<CollisionEdge>& edges);
	bool AddPossibleCollisionAxis(Vector3 axis);
	Vector3 PlaneEdgeIntersection(const Plane& plane, const Vector3& start, const Vector3& end) const;
	void SutherlandHodgesonClipping(
		const std::list<Vector3>& input_polygon,
		int num_clip_planes,
		const Plane* clip_planes,
		std::list<Vector3>* out_polygon,
		bool removeNotClipToPlane) const;
private:
	const PhysicsObject*	m_Obj1;
	const PhysicsObject*	m_Obj2;
	const CollisionShape*	m_Shape1;
	const CollisionShape*	m_Shape2;

	std::vector<Vector3>	m_PossibleCollisionAxes;

	bool					m_Colliding;
	CollisionData			m_BestColData;
};