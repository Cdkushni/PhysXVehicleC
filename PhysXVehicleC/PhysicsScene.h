#ifndef _PHYSICS_H
#define _PHYSICS_H


#include <PxPhysicsAPI.h>
#include <PxShape.h>
#include <vector>

using namespace physx;

class PhysicsScene
{
public:
	PhysicsScene();
	~PhysicsScene();
	void cleanupPhysics();
	unsigned int simulate(unsigned int* ids, float* positions, float* quaternions, float* scales, unsigned int* bitfields, unsigned int numIds, float elapsedTime, float* velocities);
	void raycast(float* origin, float* direction, float maxDist, unsigned int& hit, float* position, float* normal, float& distance, unsigned int& objectId, unsigned int& faceIndex);

	PxDefaultAllocator* allocator = nullptr;
	PxDefaultErrorCallback* errorCallback = nullptr;
	PxFoundation* foundation = nullptr;
	PxDefaultCpuDispatcher* dispatcher = nullptr;
	PxPhysics* physics = nullptr;
	PxCooking* cooking = nullptr;
	PxScene* scene = nullptr;
	PxPvd* pvd = nullptr;
	PxControllerManager* controllerManager = nullptr;
	std::vector<PxRigidActor*> actors;
	PxRigidStatic* gGroundPlane = nullptr;
	//SimulationEventCallback2* simulationEventCallback = nullptr;

};

#endif
