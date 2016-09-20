#pragma once

#include "ObjectMesh.h"

class ObjectMeshDragable : public ObjectMesh
{
public:
	ObjectMeshDragable(const std::string& name);
	virtual ~ObjectMeshDragable();

	void SetMouseOverColourOffset(const Vector4& col_offset);	//Change in colour when mouse is hovering over the object
	void SetMouseDownColourOffset(const Vector4& col_offset);	//Change in colour when clicked

protected:
	virtual void OnMouseEnter(float dt) override;
	virtual void OnMouseLeave(float dt) override;
	virtual void OnMouseDown(float dt, const Vector3& worldPos) override;
	virtual void OnMouseMove(float dt, const Vector3& worldPos, const Vector3& worldChange) override;
	virtual void OnMouseUp(float dt, const Vector3& worldPos) override;

protected:
	Vector4 m_MouseOverColOffset;
	Vector4 m_MouseDownColOffset;

	Vector3 m_LocalClickOffset;
};