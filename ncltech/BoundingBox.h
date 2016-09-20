
#pragma once
#include <nclgl\Matrix4.h>
#include <nclgl\Vector3.h>
#include <nclgl\common.h>

class BoundingBox
{
public:
	BoundingBox() : m_HasValue(false) {}

	Vector3 minPoints, maxPoints;
	

	void ExpandToFit(const Vector3& point)
	{
		if (!m_HasValue)
		{
			minPoints = point;
			maxPoints = point;
			m_HasValue = true;
		}
		else
		{
			minPoints.x = min(minPoints.x, point.x);
			minPoints.y = min(minPoints.y, point.y);
			minPoints.z = min(minPoints.z, point.z);
			maxPoints.x = max(maxPoints.x, point.x);
			maxPoints.y = max(maxPoints.y, point.y);
			maxPoints.z = max(maxPoints.z, point.z);
		}
	}

	BoundingBox Transform(const Matrix4& mtx)
	{
		BoundingBox bb;
		bb.ExpandToFit(mtx * Vector3(minPoints.x, minPoints.y, minPoints.z));
		bb.ExpandToFit(mtx * Vector3(maxPoints.x, minPoints.y, minPoints.z));
		bb.ExpandToFit(mtx * Vector3(minPoints.x, maxPoints.y, minPoints.z));
		bb.ExpandToFit(mtx * Vector3(maxPoints.x, maxPoints.y, minPoints.z));

		bb.ExpandToFit(mtx * Vector3(minPoints.x, minPoints.y, maxPoints.z));
		bb.ExpandToFit(mtx * Vector3(maxPoints.x, minPoints.y, maxPoints.z));
		bb.ExpandToFit(mtx * Vector3(minPoints.x, maxPoints.y, maxPoints.z));
		bb.ExpandToFit(mtx * Vector3(maxPoints.x, maxPoints.y, maxPoints.z));
		return bb;
	}

private:
	bool m_HasValue;
};