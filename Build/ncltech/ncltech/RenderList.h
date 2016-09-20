
#pragma once
#include "Object.h"
#include <nclgl\Frustum.h>
#include <nclgl\common.h>
#include <nclgl\Vector3.h>
#include <vector>

//Maximum number of elements added to the renderlist per frame (will be added on future frames instead to share workload and prevent lock-ups)
#define MAX_LIST_CHANGE_PER_FRAME 300
#define SORT_OPAQUE_LIST FALSE //Sort opaque objects front to back to reduce over drawing - tie up between slow sorting or slow rendering, in the current usage the sorting is almost always the bottlekneck. (Transparent objects always need to be sorted in order to correctly draw)

struct RenderList_Object
{
	float cam_dist_sq;
	Object* target_obj;
};

class RenderList
{
public:
	virtual ~RenderList();

	static bool AllocateNewRenderList(RenderList** renderlist, bool supportsTransparency); //Attempts to create new renderlist - returns false if max-renderlists (32) have already previously been allocated.

	void UpdateCameraWorldPos(const Vector3& cameraPos); //Updates all current objects 'distance' to camera

	void SortLists(); //Sort lists based on camera position using insertion sort. With frame cohency, the list should be 'almost' sorted each frame and only a few elements need to be swapped.

	void RemoveExcessObjects(const Frustum& frustum); //Removes all objects no longer inside the frustum
	void InsertObject(Object* obj); //Called when object moves inside the frustum (Inserts via insertion sort)
	void RemoveObject(Object* obj); //Misc. Removes a single object from the list, in general just call 'RemoveExcessObjects' and let it remove the object automatically
	void RemoveAllObjects(); //Clears the entire list

	void RenderOpaqueObjects(const std::function<void(Object*)>& per_object_func);
	void RenderTransparentObjects(const std::function<void(Object*)>& per_object_func);

	uint BitMask() { return m_BitMask; }

	const std::vector<RenderList_Object>& GetRenderListOpaque()			{ return m_RenderListOpaque; }
	const std::vector<RenderList_Object>& GetRenderListTransparent()	{ return m_RenderListTransparent; }

protected:
	static uint g_NumRenderLists; //Keeps a list of the number of unique render lists as to enforce and maintain the hard limit on 32 renderlists.
	uint m_NumElementsChanged;

	uint m_BitMask;				 //Bit mask for renderlist, uint leads to 32 booleans (1's or 0's) thus there is a hardlimit of 31 shadow maps along with the main camera render list. 
	bool m_SupportsTransparancy; //If false - all transparent objects will be ignored (e.g. shadow render passes)

	Vector3 m_CameraPos;
	std::vector<RenderList_Object> m_RenderListOpaque;
	std::vector<RenderList_Object> m_RenderListTransparent;

private:
	//Private Constructor - Allocate through 'AllocateNewRenderList' function
	RenderList();

	RenderList(const RenderList& rl) {}
};
