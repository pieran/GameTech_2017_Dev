#include "CollisionDetectionSAT.h"
#include "NCLDebug.h"


CollisionDetectionSAT::CollisionDetectionSAT()
{
}

void CollisionDetectionSAT::BeginNewPair(
	PhysicsObject* obj1,
	PhysicsObject* obj2,
	CollisionShape* shape1,
	CollisionShape* shape2)
{
	m_PossibleCollisionAxes.clear();

	m_Obj1 = obj1;
	m_Obj2 = obj2;
	m_Shape1 = obj1->GetCollisionShape();
	m_Shape2 = obj2->GetCollisionShape();

	m_Colliding = false;
}

#pragma region COLLISION_DETECTION
bool CollisionDetectionSAT::AreColliding(CollisionData* out_coldata)
{
	if (!m_Shape1 || !m_Shape2)
		return false;


	m_Colliding = false;

	FindAllPossibleCollisionAxes();



	//Seperating axis theorem says that if a single axis can be found where the two objects are not colliding, then they cannot be colliding.
	//  - So we have to check each possible axis until we either return false, or return the best axis (one with the least penetration) found.

	CollisionData cur_colData;

	m_BestColData.penetration = -FLT_MAX;
	for (const Vector3& axis : m_PossibleCollisionAxes)
	{
		//If the collision axis does NOT intersect then return immediately as we know that atleast in one direction/axis the two objects do not intersect
		if (!CheckCollisionAxis(axis, &cur_colData))
			return false;

		if (cur_colData.penetration >= m_BestColData.penetration)
		{
			m_BestColData = cur_colData;
		}
	}

	if (out_coldata) *out_coldata = m_BestColData;

	m_Colliding = true;
	return true;
}

void CollisionDetectionSAT::FindAllPossibleCollisionAxes()
{

//<----- DEFAULT AXES ------->
	m_Shape1->GetCollisionAxes(m_Obj1, &m_PossibleCollisionAxes);
	m_Shape2->GetCollisionAxes(m_Obj2, &m_PossibleCollisionAxes);


//<----- EDGE-EDGE CASES ----->
//	Handles the case where two edges meet and the final collision direction is mid way between two default collision axes
//  provided above.
	std::vector<CollisionEdge> shape1_edges;
	std::vector<CollisionEdge> shape2_edges;

	m_Shape1->GetEdges(m_Obj1, &shape1_edges);
	m_Shape2->GetEdges(m_Obj2, &shape2_edges);

	for (const CollisionEdge& edge1 : shape1_edges)
	{
		for (const CollisionEdge& edge2 : shape2_edges)
		{
			Vector3 e1 = edge1.posB - edge1.posA;
			Vector3 e2 = edge2.posB - edge2.posA;

			e1.Normalise();
			e2.Normalise();

			AddPossibleCollisionAxis(Vector3::Cross(e1, e2));
		}
	}


//<------ CURVED-SURFACE CASES ----->
//	Curved surfaces technically have infinite possible axis to test. 
//	However can also be simplified to only one that needs to be checked
//	as they can be defined by a constant distance from the centre. 
//	This can be seen as the proof behind the sphere-sphere test performed
//	earlier.
	bool shape1_isSphere = shape1_edges.empty();
	bool shape2_isSphere = shape2_edges.empty();

	//If both are spheres
	//	- then the only axes we have to check is between the two centre points
	if (shape1_isSphere && shape2_isSphere) 
	{
		Vector3 axis = m_Obj2->GetPosition() - m_Obj1->GetPosition();
		axis.Normalise();
		AddPossibleCollisionAxis(axis);
	}

	//If only shape1 is a sphere 
	//	- then we have to get the closest point on the edge of shape2 to use as an axis
	else if (shape1_isSphere) 
	{
		Vector3 p = GetClosestPoint(m_Obj1->GetPosition(), shape2_edges);

		Vector3 p_t = m_Obj1->GetPosition() - p;
		p_t.Normalise();
		AddPossibleCollisionAxis(p_t);
	}

	//If only shape2 is a sphere
	//	- then we haev to get the closest point on the edge of shape1 to use as an axis
	else if (shape2_isSphere) 
	{
		Vector3 p = GetClosestPoint(m_Obj2->GetPosition(), shape1_edges);

		Vector3 p_t = m_Obj2->GetPosition() - p;
		p_t.Normalise();
		AddPossibleCollisionAxis(p_t);
	}

}

bool CollisionDetectionSAT::AddPossibleCollisionAxis(Vector3 axis)
{
	const float epsilon = 0.0001f;

	//is axis 0,0,0??
	if (Vector3::Dot(axis, axis) < epsilon)
		return false;

	axis.Normalise();

	for (const Vector3& p_axis : m_PossibleCollisionAxes)
	{
		//Is axis very close to the same as a previous axis already in the list of axes??
		if (Vector3::Dot(axis, p_axis) >= 1.0f - epsilon)
			return false;
	}

	m_PossibleCollisionAxes.push_back(axis);
	return true;
}

bool CollisionDetectionSAT::CheckCollisionAxis(const Vector3& axis, CollisionData* coldata)
{
	Vector3 min1, min2, max1, max2;

	//Get the min/max vertices along the axis from shape1 and shape2
	m_Shape1->GetMinMaxVertexOnAxis(m_Obj1, axis, &min1, &max1);
	m_Shape2->GetMinMaxVertexOnAxis(m_Obj2, axis, &min2, &max2);

	float minCorrelation1 = Vector3::Dot(axis, min1);
	float maxCorrelation1 = Vector3::Dot(axis, max1);
	float minCorrelation2 = Vector3::Dot(axis, min2);
	float maxCorrelation2 = Vector3::Dot(axis, max2);


	//Object 1 mostly overlapping Object 2
	if (minCorrelation1 <= minCorrelation2
		&& maxCorrelation1 >= minCorrelation2)
	{
		if (coldata != NULL)
		{
			coldata->normal = axis;
			coldata->penetration = minCorrelation2 - maxCorrelation1;
			coldata->pointOnPlane = max1 + coldata->normal * coldata->penetration;
		}

		return true;
	}

	//Object 2 mostly overlapping Object 1
	if (minCorrelation2 <= minCorrelation1
		&& maxCorrelation2 >= minCorrelation1)
	{
		if (coldata != NULL)
		{
			coldata->normal = -axis;
			coldata->penetration = minCorrelation1 - maxCorrelation2;
			coldata->pointOnPlane = min1 + coldata->normal * coldata->penetration;
		}

		return true;
	}


	return false;
}

Vector3 CollisionDetectionSAT::GetClosestPoint(const Vector3& pos, std::vector<CollisionEdge>& edges)
{
	Vector3 final_closest_point, edge_closest_point;
	float final_closest_distsq = FLT_MAX;

	for (const CollisionEdge& edge : edges)
	{
		//As we have a line not two points, the final value could be anywhere between the edge points A/B
		//	To solve this by projecting the point (pos) onto the line described by the edge, this is very similar
		//	to the means we use to test each axis in SAT except the axis in this case is an edge which must be clamped
		//	between points A/B.
		Vector3 a_p = pos - edge.posA;
		Vector3 a_b = edge.posB - edge.posA;

		float ABAPproduct = Vector3::Dot(a_p, a_b);   //Distance along the line of point 'pos' in world distance 


		float magnitudeAB = Vector3::Dot(a_b, a_b); 
		float distance = ABAPproduct / magnitudeAB;	  //Distance along the line of point 'pos' between 0-1 where 0 is edgeA and 1 is edgeB

		//Clamp distance so it cant go beyond edgeA or edgeB in either direction
		distance = max(min(distance, 1.0f), 0.0f);

		edge_closest_point = edge.posA + a_b * distance;

		//Only store the closest point if it's closer than the results returned from previous edges.
		Vector3 c_t = pos - edge_closest_point;
		float temp_distsq = Vector3::Dot(c_t, c_t);

		if (temp_distsq < final_closest_distsq)
		{
			final_closest_distsq = temp_distsq;
			final_closest_point = edge_closest_point;
		}
	}

	return final_closest_point;
}
#pragma endregion //!COLLISION DETECTION


#pragma region CONTACT_GENERATION
void CollisionDetectionSAT::GenContactPoints(Manifold* out_manifold)
{
	if (!out_manifold || !m_Colliding)
		return;


	//Get the required face information for the two shapes around the collision normal
	std::list<Vector3>	 polygon1, polygon2;
	Vector3				 normal1, normal2;
	std::vector<Plane>	 adjPlanes1, adjPlanes2;

	m_Shape1->GetIncidentReferencePolygon(m_Obj1, m_BestColData.normal, &polygon1, &normal1, &adjPlanes1);
	m_Shape2->GetIncidentReferencePolygon(m_Obj2, -m_BestColData.normal, &polygon2, &normal2, &adjPlanes2);


	//If either shape1 or shape2 returned a single point, then it must be on a curve and thus the only contact point to generate is already availble
	if (polygon1.size() == 0 || polygon2.size() == 0)
	{
		return; //No points returned, resulting in no possible contact points
	}
	else if (polygon1.size() == 1)
	{
		out_manifold->AddContact(
			polygon1.front(),
			polygon1.front() + m_BestColData.normal * m_BestColData.penetration,
			m_BestColData.normal,
			m_BestColData.penetration);
	}
	else if (polygon2.size() == 1)
	{
		out_manifold->AddContact(
			polygon2.front() + m_BestColData.normal * m_BestColData.penetration,
			polygon2.front(),
			m_BestColData.normal,
			m_BestColData.penetration);
	}
	else
	{
		//Otherwise use clipping to cut down the incident face to fit inside the reference planes using the surrounding face planes

		bool				 flipped;
		std::list<Vector3>	 *incPolygon;
		Vector3				 *incNormal;
		std::vector<Plane>	 *refAdjPlanes;
		Plane				 refPlane;

		//Get the incident and reference polygons
		if (fabs(Vector3::Dot(m_BestColData.normal, normal1)) > fabs(Vector3::Dot(m_BestColData.normal, normal2)))
		{
			float planeDist = -Vector3::Dot(-normal1, polygon1.front());
			refPlane = Plane(-normal1, planeDist);
			refAdjPlanes = &adjPlanes1;

			incPolygon = &polygon2;
			incNormal = &normal2;

			flipped = false;
		}
		else
		{
			float planeDist = -Vector3::Dot(-normal2, polygon2.front());
			refPlane = Plane(-normal2, planeDist);
			refAdjPlanes = &adjPlanes2;

			incPolygon = &polygon1;
			incNormal = &normal1;

			flipped = true;
		}


		//Clip the incident face to the adjacent edges of the reference face
		SutherlandHodgesonClipping(*incPolygon, refAdjPlanes->size(), &(*refAdjPlanes)[0], incPolygon, false);

		//Finally clip (and remove) any contact points that are above the reference face
		SutherlandHodgesonClipping(*incPolygon, 1, &refPlane, incPolygon, true);

		//Now we are left with a selection of valid contact points to be used for the manifold
		Vector3 startPoint = incPolygon->back();
		for (const Vector3& endPoint : *incPolygon)
		{
			float contact_penetration;
			Vector3 globalOnA, globalOnB;

			if (flipped)
			{
				//Calculate distance to ref plane/face
				contact_penetration = -(Vector3::Dot(endPoint, m_BestColData.normal) - Vector3::Dot(m_BestColData.normal, polygon2.front()));
	
				globalOnA = endPoint + m_BestColData.normal * contact_penetration;
				globalOnB = endPoint;
			}
			else
			{
				//Calculate distance to ref plane/face
				contact_penetration = Vector3::Dot(endPoint, m_BestColData.normal) - Vector3::Dot(m_BestColData.normal, polygon1.front());

				globalOnA = endPoint;
				globalOnB = endPoint - m_BestColData.normal * contact_penetration;
			}

			//Just make a final sanity check that the contact point is actual a point of contact
			// not just a clipping bug
			if (contact_penetration < 0.0f)
			{
				out_manifold->AddContact(
					globalOnA,
					globalOnB,
					m_BestColData.normal,
					contact_penetration);
			}

			startPoint = endPoint;
		}

	}
}


Vector3 CollisionDetectionSAT::PlaneEdgeIntersection(const Plane& plane, const Vector3& start, const Vector3& end) const
{
	float start_dist = Vector3::Dot(start, plane.GetNormal()) + plane.GetDistance();
	float end_dist = Vector3::Dot(end, plane.GetNormal()) + plane.GetDistance();

	Vector3 ab = end - start;

	float ab_p = Vector3::Dot(plane.GetNormal(), ab);

	if (fabs(ab_p) > 0.0001f)
	{
		Vector3 p_co = plane.GetNormal() * (-plane.GetDistance());

		Vector3 w = start - p_co;
		float fac = -Vector3::Dot(plane.GetNormal(), w) / ab_p;
		ab = ab * fac;

		return start + ab;
	}

	return start;
}

void CollisionDetectionSAT::SutherlandHodgesonClipping(
	const std::list<Vector3>& input_polygon,
	int num_clip_planes,
	const Plane* clip_planes,
	std::list<Vector3>* out_polygon,
	bool removePoints) const
{
	if (!out_polygon)
		return;

	std::list<Vector3> ppPolygon1, ppPolygon2;
	std::list<Vector3> *input = &ppPolygon1, *output = &ppPolygon2;

	*output = input_polygon;
	for (int i = 0; i < num_clip_planes; ++i)
	{
		if (output->empty())
			break;

		const Plane& plane = clip_planes[i];

		std::swap(input, output);
		output->clear();

		Vector3 startPoint = input->back();
		for (const Vector3& endPoint : *input)
		{
			bool startInPlane = plane.PointInPlane(startPoint);
			bool endInPlane = plane.PointInPlane(endPoint);

			//If it's the final pass, just remove all points outside the reference plane
			if (removePoints)
			{
				if (endInPlane)
					output->push_back(endPoint);
			}
			else
			{
				//if entire edge is within the clipping plane, keep it as it is
				if (startInPlane && endInPlane)
					output->push_back(endPoint);

				//if edge interesects the clipping plane, cut the edge along clip plane
				else if (startInPlane && !endInPlane)
				{
					output->push_back(PlaneEdgeIntersection(plane, startPoint, endPoint));
				}
				else if (!startInPlane && endInPlane)
				{
					output->push_back(PlaneEdgeIntersection(plane, endPoint, startPoint));
					output->push_back(endPoint);
				}
			}

			//..otherwise the edge is entirely outside the clipping plane and should be removed


			startPoint = endPoint;
		}
	}

	*out_polygon = *output;
}

#pragma endregion //CONTACT_GENERATION