#ifndef PHYSX_RENDER_H
#define PHYSX_RENDER_H

#include "PxPhysicsAPI.h"
#include "foundation/PxPreprocessor.h"

#if PX_WINDOWS
#include <windows.h>
#pragma warning(disable: 4505)
#include <GL/glut.h>
#elif PX_LINUX_FAMILY
#include <GL/glut.h>
#elif PX_OSX
#include <GLUT/glut.h>
#else
#error platform not supported.
#endif

namespace RenderHandler
{
	void setupDefaultWindow(const char* name);
	void setupDefaultRenderState();

	void startRender(const physx::PxVec3& cameraEye, const physx::PxVec3& cameraDir, physx::PxReal nearClip = 1.f, physx::PxReal farClip = 10000.f);
	void finishRender();

	class TriggerRender
	{
	public:
		virtual	bool	isTrigger(physx::PxShape*)	const = 0;
	};

	void renderActors(physx::PxRigidActor** actors, const physx::PxU32 numActors, bool shadows = false, const physx::PxVec3& color = physx::PxVec3(0.0f, 0.75f, 0.0f), TriggerRender* cb = nullptr);
	//	void renderGeoms(const physx::PxU32 nbGeoms, const physx::PxGeometry* geoms, const physx::PxTransform* poses, bool shadows, const physx::PxVec3& color);
	void renderGeoms(const physx::PxU32 nbGeoms, const physx::PxGeometryHolder* geoms, const physx::PxTransform* poses, bool shadows, const physx::PxVec3& color);
}

#endif //PHYSX_RENDER_H