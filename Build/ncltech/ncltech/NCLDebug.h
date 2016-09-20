/******************************************************************************
Class: NCLDebug
Author: Pieran Marris <p.marris@newcastle.ac.uk>
Description:
Provides a range of globally accessable debug utilities to allow visualising world space problems/algorithms easily.

Note: Both 'point_radius' (DrawPoint) and 'line_width' (DrawThickLine) are in world-space coordinates
 while 'font_size' (All Text Functions) is in terms of screen pixels - the same as microsoft word etc.


		(\_/)
		( '_')
	 /""""""""""""\=========     -----D
	/"""""""""""""""""""""""\
....\_@____@____@____@____@_/

*//////////////////////////////////////////////////////////////////////////////

#pragma once
#include <nclgl\Matrix4.h>
#include <nclgl\Matrix3.h>
#include <nclgl\Vector4.h>
#include <nclgl\Vector3.h>
#include <nclgl\Shader.h>
#include <vector>
#include <mutex>

#define MAX_LOG_SIZE		25
#define LOG_TEXT_SIZE  		14.0f
#define STATUS_TEXT_SIZE	16.0f

enum TextAlignment
{
	TEXTALIGN_LEFT,
	TEXTALIGN_RIGHT,
	TEXTALIGN_CENTRE
};

struct LogEntry
{
	Vector4		colour;
	std::string text;
};

#define NCLERROR(str, ...) NCLDebug::LogE(__FILE__, __LINE__, str, __VA_ARGS__)



class NCLDebug
{
	friend class Scene;
	friend class SceneRenderer;

public:
	//Note: Functions appended with 'NDT' (no depth testing) will always be rendered in the foreground. This can be useful for debugging things inside objects.


	//Draw Point (circle)
	static void DrawPoint(const Vector3& pos, float point_radius, const Vector3& colour = Vector3(1.0f, 1.0f, 1.0f));
	static void DrawPoint(const Vector3& pos, float point_radius, const Vector4& colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f));
	static void DrawPointNDT(const Vector3& pos, float point_radius, const Vector3& colour = Vector3(1.0f, 1.0f, 1.0f));
	static void DrawPointNDT(const Vector3& pos, float point_radius, const Vector4& colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f));

	//Draw Line with a given thickness 
	static void DrawThickLine(const Vector3& start, const Vector3& end, float line_width, const Vector3& colour = Vector3(1.0f, 1.0f, 1.0f));
	static void DrawThickLine(const Vector3& start, const Vector3& end, float line_width, const Vector4& colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f));
	static void DrawThickLineNDT(const Vector3& start, const Vector3& end, float line_width, const Vector3& colour = Vector3(1.0f, 1.0f, 1.0f));
	static void DrawThickLineNDT(const Vector3& start, const Vector3& end, float line_width, const Vector4& colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f));

	//Draw line with thickness of 1 screen pixel regardless of distance from camera
	static void DrawHairLine(const Vector3& start, const Vector3& end, const Vector3& colour = Vector3(1.0f, 1.0f, 1.0f));
	static void DrawHairLine(const Vector3& start, const Vector3& end, const Vector4& colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f));
	static void DrawHairLineNDT(const Vector3& start, const Vector3& end, const Vector3& colour = Vector3(1.0f, 1.0f, 1.0f));
	static void DrawHairLineNDT(const Vector3& start, const Vector3& end, const Vector4& colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f));

	//Draw Matrix (x,y,z axis at pos)
	static void DrawMatrix(const Matrix4& transform_mtx);
	static void DrawMatrix(const Matrix3& rotation_mtx, const Vector3& position);
	static void DrawMatrixNDT(const Matrix4& transform_mtx);
	static void DrawMatrixNDT(const Matrix3& rotation_mtx, const Vector3& position);

	//Draw Triangle 
	static void DrawTriangle(const Vector3& v0, const Vector3& v1, const Vector3& v2, const Vector4& colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f));
	static void DrawTriangleNDT(const Vector3& v0, const Vector3& v1, const Vector3& v2, const Vector4& colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f));

	//Draw Polygon (Renders as a triangle fan, so verts must be arranged in order)
	static void DrawPolygon(int n_verts, const Vector3* verts, const Vector4& colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f));
	static void DrawPolygonNDT(int n_verts, const Vector3* verts, const Vector4& colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f));


	//Draw Text WorldSpace (pos given here in worldspace)
	static void DrawTextWs(const Vector3& pos, const float font_size, const TextAlignment alignment, const Vector4 colour, const string text, ...); ///See "printf" for usuage manual
	static void DrawTextWsNDT(const Vector3& pos, const float font_size, const TextAlignment alignment, const Vector4 colour, const string text, ...); ///See "printf" for usuage manual

	//Draw Text (pos is assumed to be pre-multiplied by projMtx * viewMtx at this point)
	static void DrawTextClipSpace(const Vector4& pos, const float font_size, const string& text, const TextAlignment alignment = TEXTALIGN_LEFT, const Vector4 colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f));

	//Add a status entry at the top left of the screen (Cleared each frame)
	static void AddStatusEntry(const Vector4& colour, const std::string text, ...); ///See "printf" for usuage manual

	//Add a log entry at the bottom left - persistent until scene reset
	static void Log(const Vector3& colour, const std::string text, ...); ///See "printf" for usuage manual

	//Add an error using default error formatting - use "NCLERROR("error description", <printf params>) to automatically call this function and fill in the required params
	static void LogE(const char* filename, int linenumber, const std::string text, ...);



protected:
	//Actual functions managing data parsing to save code bloat - called by public functions
	static void GenDrawPoint(bool ndt, const Vector3& pos, float point_radius, const Vector4& colour);
	static void GenDrawThickLine(bool ndt, const Vector3& start, const Vector3& end, float line_width, const Vector4& colour);
	static void GenDrawHairLine(bool ndt, const Vector3& start, const Vector3& end, const Vector4& colour);
	static void GenDrawTriangle(bool ndt, const Vector3& v0, const Vector3& v1, const Vector3& v2, const Vector4& colour);
	static void GenDrawPolygon(bool ndt, int n_verts, const Vector3* verts, const Vector4& colour);




	static void AddLogEntry(const Vector3& colour, const std::string& text);

	//Called by Scene Renderer class
	static void ClearDebugLists();
	static void SortDebugLists();
	static void DrawDebugLists();
	static void DrawDebubHUD();

	static void LoadShaders();
	static void ReleaseShaders();

	static void ClearLog();

	static void SetDebugDrawData(const Matrix4& projViewMatrix, const Vector3& camera_pos)
	{ 
		m_ProjView = projViewMatrix;
		m_CameraPosition = camera_pos;
	}

protected:
	static Vector3	m_CameraPosition;
	static Matrix4	m_ProjView;

	static int m_NumStatusEntries;
	static std::vector<LogEntry> m_LogEntries;
	static int m_LogEntriesOffset;

	static std::vector<Vector4> m_Characters;
	struct DebugDrawList
	{
		std::vector<Vector4> points;	
		std::vector<Vector4> thickLines;
		std::vector<Vector4> hairLines;
		std::vector<Vector4> tris;
	};
	static DebugDrawList m_DrawList;			//Depth-Tested
	static DebugDrawList m_DrawListNDT;			//Not Depth-Tested

	static Shader*	m_pShaderPoints;
	static Shader*	m_pShaderLines;
	static Shader*	m_pShaderHairLines;
	static Shader*	m_pShaderText;

	static GLuint	m_glArray, m_glBuffer;
	static GLuint	m_glFontTex;
	static size_t	m_OffsetChars;

	static std::mutex m_DebugMutex;
};