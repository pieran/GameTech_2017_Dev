#include "CommonUtils.h"

#include "ObjectMesh.h"
#include "ObjectMeshDragable.h"
#include "SphereCollisionShape.h"
#include "CuboidCollisionShape.h"
#include "CommonMeshes.h"

Vector4 CommonUtils::GenColour(float scalar, float alpha)
{
	Vector4 c;
	c.w = alpha;

	float t;
	c.x = abs(modf(scalar + 1.0f, &t) * 6.0f - 3.0f) - 1.0f;
	c.y = abs(modf(scalar + 2.0f / 3.0f, &t) * 6.0f - 3.0f) - 1.0f;
	c.z = abs(modf(scalar + 1.0f / 3.0f, &t) * 6.0f - 3.0f) - 1.0f;

	c.x = min(max(c.x, 0.0f), 1.0f);
	c.y = min(max(c.y, 0.0f), 1.0f);
	c.z = min(max(c.z, 0.0f), 1.0f);

	return c;
}

Object* CommonUtils::BuildSphereObject(
	const std::string& name,
	const Vector3& pos,
	float radius,
	bool physics_enabled,
	float inverse_mass,
	bool collidable,
	bool dragable,
	const Vector4& color)
{
	ObjectMesh* sphere = dragable
		? new ObjectMeshDragable(name)
		: new ObjectMesh(name);

	sphere->SetMesh(CommonMeshes::Sphere(), false);
	sphere->SetTexture(CommonMeshes::CheckerboardTex(), false);
	sphere->SetLocalTransform(Matrix4::Scale(Vector3(radius, radius, radius)));
	sphere->SetColour(color);
	sphere->SetBoundingRadius(radius);

	if (!physics_enabled)
	{
		//If no physics object is present, just set the local transform (modelMatrix) directly
		sphere->SetLocalTransform(Matrix4::Translation(pos) * sphere->GetLocalTransform());
	}
	else
	{
		//Otherwise create a physics object, and set it's position etc
		sphere->CreatePhysicsNode();

		sphere->Physics()->SetPosition(pos);
		sphere->Physics()->SetInverseMass(inverse_mass);
		
		if (!collidable)
		{
			//Even without a collision shape, the inertia matrix for rotation has to be derived from the objects shape
			sphere->Physics()->SetInverseInertia(SphereCollisionShape(radius).BuildInverseInertia(inverse_mass));
		}
		else
		{
			CollisionShape* colshape = new SphereCollisionShape(radius);
			sphere->Physics()->SetCollisionShape(colshape);
			sphere->Physics()->SetInverseInertia(colshape->BuildInverseInertia(inverse_mass));
		}			
	}

	return sphere;
}

Object* CommonUtils::BuildCuboidObject(
	const std::string& name,
	const Vector3& pos,
	const Vector3& halfdims,
	bool physics_enabled,
	float inverse_mass,
	bool collidable,
	bool dragable,
	const Vector4& color)
{
	ObjectMesh* cuboid = dragable
		? new ObjectMeshDragable(name)
		: new ObjectMesh(name);

	cuboid->SetMesh(CommonMeshes::Cube(), false);
	cuboid->SetTexture(CommonMeshes::CheckerboardTex(), false);
	cuboid->SetLocalTransform(Matrix4::Scale(halfdims));
	cuboid->SetColour(color);
	cuboid->SetBoundingRadius(halfdims.Length());

	if (!physics_enabled)
	{
		//If no physics object is present, just set the local transform (modelMatrix) directly
		cuboid->SetLocalTransform(Matrix4::Translation(pos) * cuboid->GetLocalTransform());
	}
	else
	{
		//Otherwise create a physics object, and set it's position etc
		cuboid->CreatePhysicsNode();
		cuboid->Physics()->SetPosition(pos);
		cuboid->Physics()->SetInverseMass(inverse_mass);

		if (!collidable)
		{
			//Even without a collision shape, the inertia matrix for rotation has to be derived from the objects shape
			cuboid->Physics()->SetInverseInertia(CuboidCollisionShape(halfdims).BuildInverseInertia(inverse_mass));
		}
		else
		{
			CollisionShape* colshape = new CuboidCollisionShape(halfdims);
			cuboid->Physics()->SetCollisionShape(colshape);
			cuboid->Physics()->SetInverseInertia(colshape->BuildInverseInertia(inverse_mass));
		}
	}

	return cuboid;
}
