#include "PhysicsScene.h"
#include <string>
#include <iostream>
#include "PhysxVehicleFilterShader.h"
#include "PhysXWheelContactModifyCallback.h"
#include "PhysXWheelCCDContactModifyCallback.h"
#include "VehicleSceneQuery.h"

#define PX_RELEASE(x)	if(x)	{ x->release(); x = NULL;	}

PxRigidStatic* createDrivablePlane(const PxFilterData & simFilterData, PxMaterial * material, PxPhysics * physics)
{
	//Add a plane to the scene.
	PxRigidStatic* groundPlane = PxCreatePlane(*physics, PxPlane(0, 1, 0, 0), *material);

	//Get the plane shape so we can set query and simulation filter data.
	PxShape* shapes[1];
	groundPlane->getShapes(shapes, 1);

	//Set the query filter data of the ground plane so that the vehicle raycasts can hit the ground.
	PxFilterData qryFilterData;
	pvehicle::setupDrivableSurface(qryFilterData);
	shapes[0]->setQueryFilterData(qryFilterData);

	//Set the simulation filter data of the ground plane so that it collides with the chassis of a vehicle but not the wheels.
	shapes[0]->setSimulationFilterData(simFilterData);

	return groundPlane;
}


PhysicsScene::PhysicsScene()
{
	allocator = new PxDefaultAllocator();
	errorCallback = new PxDefaultErrorCallback();
	foundation = PxCreateFoundation(PX_PHYSICS_VERSION, *allocator, *errorCallback);
	if (!foundation) throw("PxCreateFoundation failed!");
	PxTolerancesScale toleranceScale;

	pvd = PxCreatePvd(*foundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	pvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	toleranceScale.length = 100;		// typical length of an object
	toleranceScale.speed = 981;		// typical speed of an object, gravity*1s is a reasonable choice
	physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, toleranceScale, true, pvd);

	cooking = PxCreateCooking(PX_PHYSICS_VERSION, *foundation, PxCookingParams(PxTolerancesScale()));

	PxSceneDesc sceneDesc(physics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	sceneDesc.flags |= PxSceneFlag::eENABLE_ACTIVE_ACTORS;
	sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
	sceneDesc.broadPhaseType = PxBroadPhaseType::eABP;

	if (!sceneDesc.cpuDispatcher) {
		dispatcher = PxDefaultCpuDispatcherCreate(0);
		if (!dispatcher) {
			std::cerr << "PxDefaultCpuDispatcherCreate failed!" << std::endl;
		}
		sceneDesc.cpuDispatcher = dispatcher;
	}

	sceneDesc.filterShader = vehicle::VehicleFilterShader;
	sceneDesc.contactModifyCallback = &gWheelContactModifyCallback;
	sceneDesc.ccdContactModifyCallback = &gWheelCCDContactModifyCallback;
	scene = physics->createScene(sceneDesc);
	controllerManager = PxCreateControllerManager(*scene);
	controllerManager->setOverlapRecoveryModule(true);

	PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	//Create a plane to drive on.
	PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.6f);
	PxFilterData groundPlaneSimFilterData(vehicle::COLLISION_FLAG_GROUND, vehicle::COLLISION_FLAG_GROUND_AGAINST, 0, 0);
	gGroundPlane = createDrivablePlane(groundPlaneSimFilterData, material, physics);
	scene->addActor(*gGroundPlane);

}

PhysicsScene::~PhysicsScene()
{
	std::cout << "Scene Destructor" << std::endl;
	abort();
}

unsigned int PhysicsScene::simulate(unsigned int* ids, float* positions, float* quaternions, float* scales, unsigned int* bitfields, unsigned int numIds, float elapsedTime, float* velocities)
{
	return 0;
}

void PhysicsScene::cleanupPhysics()
{
	PX_RELEASE(cooking);
	PX_RELEASE(scene);
	PX_RELEASE(dispatcher);
	PX_RELEASE(physics);
	PX_RELEASE(gGroundPlane);
	if (pvd)
	{
		PxPvdTransport* transport = pvd->getTransport();
		pvd->release();
		pvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(foundation);
}
