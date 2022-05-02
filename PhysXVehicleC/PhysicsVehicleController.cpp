#include "PhysicsVehicleController.h"
#include <ctype.h>
#include "CustomUtils.h"
#include <iostream>

#define PX_RELEASE(x)	if(x)	{ x->release(); x = NULL;	}

PxF32 gSteerVsForwardSpeedData[2 * 8] =
{
	0.0f,		0.75f,
	5.0f,		0.75f,
	30.0f,		0.125f,
	120.0f,		0.1f,
	PX_MAX_F32, PX_MAX_F32,
	PX_MAX_F32, PX_MAX_F32,
	PX_MAX_F32, PX_MAX_F32,
	PX_MAX_F32, PX_MAX_F32
};
PxFixedSizeLookupTable<8> gSteerVsForwardSpeedTable(gSteerVsForwardSpeedData, 4);

PxVehicleKeySmoothingData gKeySmoothingData =
{
	{
		6.0f,	//rise rate eANALOG_INPUT_ACCEL
		6.0f,	//rise rate eANALOG_INPUT_BRAKE		
		6.0f,	//rise rate eANALOG_INPUT_HANDBRAKE	
		2.5f,	//rise rate eANALOG_INPUT_STEER_LEFT
		2.5f,	//rise rate eANALOG_INPUT_STEER_RIGHT
	},
	{
		10.0f,	//fall rate eANALOG_INPUT_ACCEL
		10.0f,	//fall rate eANALOG_INPUT_BRAKE		
		10.0f,	//fall rate eANALOG_INPUT_HANDBRAKE	
		5.0f,	//fall rate eANALOG_INPUT_STEER_LEFT
		5.0f	//fall rate eANALOG_INPUT_STEER_RIGHT
	}
};

PxVehiclePadSmoothingData gPadSmoothingData =
{
	{
		6.0f,	//rise rate eANALOG_INPUT_ACCEL
		6.0f,	//rise rate eANALOG_INPUT_BRAKE		
		6.0f,	//rise rate eANALOG_INPUT_HANDBRAKE	
		2.5f,	//rise rate eANALOG_INPUT_STEER_LEFT
		2.5f,	//rise rate eANALOG_INPUT_STEER_RIGHT
	},
	{
		10.0f,	//fall rate eANALOG_INPUT_ACCEL
		10.0f,	//fall rate eANALOG_INPUT_BRAKE		
		10.0f,	//fall rate eANALOG_INPUT_HANDBRAKE	
		5.0f,	//fall rate eANALOG_INPUT_STEER_LEFT
		5.0f	//fall rate eANALOG_INPUT_STEER_RIGHT
	}
};

PxVehicleDrive4WRawInputData gVehicleInputData;

bool PhysicsVehicleController::handleKeyUp(unsigned char key, int x, int y, float speed)
{
	PX_UNUSED(x);
	PX_UNUSED(y);

	if (toupper(key) == 'W') {
		forwardsInput = 0;
	}
	else if (toupper(key) == 'S') {
		forwardsInput = 0;
	}
	if (toupper(key) == 'A') {
		steerInput = 0;
	}
	else if (toupper(key) == 'D') {
		steerInput = 0;
	}
	if (toupper(key) == 'B') {
		brakeInput = 0;
	}
	else if (toupper(key) == 'H') {
		brakeInput = 0;
	}
	return true;
}

bool PhysicsVehicleController::handleKey(unsigned char key, int x, int y, float speed)
{
	PX_UNUSED(x);
	PX_UNUSED(y);

	if (toupper(key) == 'W') {
		forwardsInput = 1;
	}
	else if (toupper(key) == 'S') {
		forwardsInput = -1;
	}
	if (toupper(key) == 'A') {
		steerInput = 1;
	}
	else if (toupper(key) == 'D') {
		steerInput = -1;
	}
	if (toupper(key) == 'B') {
		brakeInput = 1;
	}
	else if (toupper(key) == 'H') {
		brakeInput = -1;
	}
	return true;
}

void PhysicsVehicleController::updateInputs(const PxF32 timestep)
{
	gVehicleModeTimer += timestep;
	releaseAllControls();

	// Only allow new inputs after a slight delay when there has been no continuous input
	if (gVehicleModeTimer > 0.03) {
		if (forwardsInput == 0 && steerInput == 0 && brakeInput == 0) {
			gVehicleModeTimer = 0;
		}

		if (steerInput > 0) {
			startTurnHardRightMode();
		}
		else if (steerInput < 0) {
			startTurnHardLeftMode();
		}

		if (forwardsInput > 0) {
			startAccelerateForwardsMode();
		}
		else if (forwardsInput < 0) {
			startAccelerateReverseMode();
		}

		if (brakeInput > 0) {
			startBrakeMode();
		}
		else if (brakeInput < 0) {
			startHandBrakeMode();
		}
	}
	
}

PhysicsVehicleController::PhysicsVehicleController(PhysicsScene* Scene)
{
	gPhysicsScene = Scene;
	physics = gPhysicsScene->physics;
	gVehicleInputData = new PxVehicleDrive4WRawInputData();
}

PhysicsVehicleController::~PhysicsVehicleController()
{
	releasePhysicsVehicle();
}

void PhysicsVehicleController::startAccelerateForwardsMode()
{
	// If we are already in reverse then change to first gear
	if (DriveMode::eDRIVE_MODE_ACCEL_REVERSE == gCurrentDriveMode)
	{
		gVehicle->mDriveDynData.forceGearChange(PxVehicleGearsData::eFIRST);
		gCurrentDriveMode = DriveMode::eDRIVE_MODE_ACCEL_FORWARDS;
	}

	if (gMimicKeyInputs)
	{
		gVehicleInputData->setDigitalAccel(true);
	}
	else
	{
		gVehicleInputData->setAnalogAccel(1.0f);
	}
}

void PhysicsVehicleController::startAccelerateReverseMode()
{
	// Move gear and drive mode to reverse mode
	gVehicle->mDriveDynData.forceGearChange(PxVehicleGearsData::eREVERSE);
	gCurrentDriveMode = DriveMode::eDRIVE_MODE_ACCEL_REVERSE;

	if (gMimicKeyInputs)
	{
		gVehicleInputData->setDigitalAccel(true);
	}
	else
	{
		gVehicleInputData->setAnalogAccel(1.0f);
	}
}

void PhysicsVehicleController::startBrakeMode()
{
	if (gMimicKeyInputs)
	{
		gVehicleInputData->setDigitalBrake(true);
	}
	else
	{
		gVehicleInputData->setAnalogBrake(1.0f);
	}
}

void PhysicsVehicleController::startHandBrakeMode()
{
	if (gMimicKeyInputs)
	{
		gVehicleInputData->setDigitalHandbrake(true);
	}
	else
	{
		gVehicleInputData->setAnalogHandbrake(1.0f);
	}
}

void PhysicsVehicleController::startTurnHardLeftMode()
{
	if (gMimicKeyInputs)
	{
		gVehicleInputData->setDigitalSteerLeft(true);
	}
	else
	{
		gVehicleInputData->setAnalogSteer(-1.0f);
	}
}

void PhysicsVehicleController::startTurnHardRightMode()
{
	if (gMimicKeyInputs)
	{
		gVehicleInputData->setDigitalSteerRight(true);
	}
	else
	{
		gVehicleInputData->setAnalogSteer(1.0f);
	}
}

void PhysicsVehicleController::startHandbrakeTurnLeftMode()
{
	if (gMimicKeyInputs)
	{
		gVehicleInputData->setDigitalSteerLeft(true);
	}
	else
	{
		gVehicleInputData->setAnalogSteer(-1.0f);
	}
}

void PhysicsVehicleController::startHandbrakeTurnRightMode()
{
	if (gMimicKeyInputs)
	{
		gVehicleInputData->setDigitalSteerRight(true);
	}
	else
	{
		gVehicleInputData->setAnalogSteer(1.0f);
	}
}


void PhysicsVehicleController::releaseAllControls()
{
	if (gMimicKeyInputs)
	{
		gVehicleInputData->setDigitalAccel(false);
		gVehicleInputData->setDigitalSteerLeft(false);
		gVehicleInputData->setDigitalSteerRight(false);
		gVehicleInputData->setDigitalBrake(false);
		gVehicleInputData->setDigitalHandbrake(false);
	}
	else
	{
		gVehicleInputData->setAnalogAccel(0.0f);
		gVehicleInputData->setAnalogSteer(0.0f);
		gVehicleInputData->setAnalogBrake(0.0f);
		gVehicleInputData->setAnalogHandbrake(0.0f);
	}
}

void PhysicsVehicleController::setupPhysXVehicleActor()
{

	material = physics->createMaterial(0.5f, 0.5f, 0.6f);

	vehicleActor = new PhysXVehicle(gPhysicsScene, material);
	vehicleActor->StartPhysXVehicleSDK();
	gVehicle = vehicleActor->gVehicle;

	//Set the vehicle to rest in first gear.
	//Set the vehicle to use auto-gears.
	gVehicle->setToRestState();
	gVehicle->mDriveDynData.forceGearChange(PxVehicleGearsData::eFIRST);
	gVehicle->mDriveDynData.setUseAutoGears(true);

	gVehicleModeTimer = 0.0f;
	startBrakeMode();
}

void PhysicsVehicleController::runUpdate()
{
	timeAtLastFrameBegin = timeAtThisFrameBegin;
	timeAtThisFrameBegin = GetCurrentTimeSeconds();
	float timestep = static_cast<float>(timeAtThisFrameBegin - timeAtLastFrameBegin);
	timestep = Clamp(timestep, 0.0f, 0.1f);

	//Update the control inputs for the vehicle.
	updateInputs(timestep);
	if (gMimicKeyInputs) {
		PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs(gKeySmoothingData, gSteerVsForwardSpeedTable, *gVehicleInputData, timestep, gIsVehicleInAir, *gVehicle);
	}
	else {
		PxVehicleDrive4WSmoothAnalogRawInputsAndSetAnalogInputs(gPadSmoothingData, gSteerVsForwardSpeedTable, *gVehicleInputData, timestep, gIsVehicleInAir, *gVehicle);
	}

	vehicleActor->VehiclePhysicsUpdate(timestep);
	//Work out if the vehicle is in the air.
	gIsVehicleInAir = gVehicle->getRigidDynamicActor()->isSleeping() ? false : PxVehicleIsInAir(vehicleActor->vehicleQueryResults[0]);

}

void PhysicsVehicleController::releasePhysicsVehicle()
{
	vehicleActor->ReleaseVehicleActor();
	PX_RELEASE(material);
}

PxVec3 PhysicsVehicleController::GetVehicleForwardBasis() const
{
	PxMat44 pose = gVehicle->getRigidDynamicActor()->getGlobalPose();
	PxVec3 pxForward = pose.getBasis(2);

	return pxForward;
}

PxVec3 PhysicsVehicleController::GetVehiclePosition() const
{
	PxMat44 pose = gVehicle->getRigidDynamicActor()->getGlobalPose();
	PxVec3 pxPosition = pose.getPosition();

	return pxPosition;
}

PxVehicleDrive4WRawInputData* PhysicsVehicleController::GetVehicleInputData() const
{
	return gVehicleInputData;
}

PxVehicleDrive4W* PhysicsVehicleController::GetVehicle() const
{
	return gVehicle;
}