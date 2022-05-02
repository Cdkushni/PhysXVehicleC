#ifndef _PVEHICLE_H
#define _PVEHICLE_H

#include <PxPhysicsAPI.h>

#include "PhysicsScene.h"
#include "VehicleSceneQuery.h"
#include "PhysXVehicle.h"

using namespace physx;
using namespace pvehicle;

enum class DriveMode
{
	eDRIVE_MODE_ACCEL_FORWARDS = 0,
	eDRIVE_MODE_ACCEL_REVERSE,
	eDRIVE_MODE_HARD_TURN_LEFT,
	eDRIVE_MODE_HANDBRAKE_TURN_LEFT,
	eDRIVE_MODE_HARD_TURN_RIGHT,
	eDRIVE_MODE_HANDBRAKE_TURN_RIGHT,
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_NONE
};


class PhysicsVehicleController
{
public:
	PhysicsVehicleController(PhysicsScene* Scene);
	~PhysicsVehicleController();

	// Controller
	bool handleKey(unsigned char key, int x, int y, float speed = 1.0f);
	bool handleKeyUp(unsigned char key, int x, int y, float speed = 1.0f);
	void updateInputs(const PxF32 timestep);
	void startAccelerateForwardsMode();
	void startAccelerateReverseMode();
	void startBrakeMode();
	void startHandBrakeMode();
	void startTurnHardLeftMode();
	void startTurnHardRightMode();
	void startHandbrakeTurnLeftMode();
	void startHandbrakeTurnRightMode();
	void releaseAllControls();

	// Physics Handlers
	void setupPhysXVehicleActor();
	void runUpdate();
	void releasePhysicsVehicle();

	// Vehicle Physics Utilities
	PxVec3 GetVehicleForwardBasis() const;
	PxVec3 GetVehiclePosition() const;
	PxVehicleDrive4WRawInputData* GetVehicleInputData() const;
	PxVehicleDrive4W* GetVehicle() const;

private:
	PxPhysics* physics = nullptr;
	PxMaterial* material = nullptr;
	PhysicsScene* gPhysicsScene = nullptr;

	PxVehicleDrive4W* gVehicle = nullptr;
	PhysXVehicle* vehicleActor = nullptr;

	VehicleSceneQueryData* gVehicleSceneQueryData = nullptr;
	PxBatchQuery* gBatchQuery = nullptr;
	PxVehicleDrivableSurfaceToTireFrictionPairs* gFrictionPairs = nullptr;

	PxVehicleDrive4WRawInputData* gVehicleInputData;
	DriveMode gCurrentDriveMode = DriveMode::eDRIVE_MODE_NONE;

	bool					gIsVehicleInAir = true;

	PxF32					gVehicleModeTimer = 0.0f;
	bool					gMimicKeyInputs = false;

	double					timeAtLastFrameBegin = 0;
	double					timeAtThisFrameBegin = 0;

	// Key Inputs
	PxF32					forwardsInput = 0;
	int						steerInput = 0;
	int						brakeInput = 0;

};

#endif

