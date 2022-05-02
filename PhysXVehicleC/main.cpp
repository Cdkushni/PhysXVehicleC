#include <GL/glut.h>


#include<PxPhysicsAPI.h>
#include<PxShape.h>
#include "PhysicsScene.h"
#include "PhysicsVehicleController.h"
#include "RenderHandler.h"
#include "CameraObject.h"
#include <iostream>

PhysicsScene* pScene = new PhysicsScene();
PhysicsVehicleController* pVehicleController = new PhysicsVehicleController(pScene);

void setupPhysicsScene()
{
	// create simulation
	physx::PxMaterial* mMaterial = NULL;
	mMaterial = pScene->physics->createMaterial(0.5f, 0.5f, 0.6f);
	physx::PxRigidStatic* groundPlane = PxCreatePlane(*(pScene->physics), physx::PxPlane(0, 1, 0, 1), *mMaterial);
	pScene->scene->addActor(*groundPlane);

	pVehicleController->setupPhysXVehicleActor();
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	PX_UNUSED(camera);
	PX_UNUSED(key);
}

namespace
{
	RenderHandler::Camera* sCamera;


/*
void motionCallback(int x, int y)
{
	int i = x;
	//sCamera->handleMotion(x, y);
}
*/

void keyboardReleasedCallback(unsigned char key, int x, int y)
{
	pVehicleController->handleKeyUp(key, x, y);
}

void keyboardCallback(unsigned char key, int x, int y)
{
	if (key == 27)
		exit(0);

	pVehicleController->handleKey(key, x, y);

}

void mouseCallback(int button, int state, int x, int y)
{
	sCamera->handleMouse(button, state, x, y);
}

void idleCallback()
{
	glutPostRedisplay();
}

void renderCallback()
{

	pVehicleController->runUpdate();

	RenderHandler::startRender(sCamera->getEye(), sCamera->getDir());

	PxU32 nbActors = pScene->scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if (nbActors)
	{
		std::vector<PxRigidActor*> actors(nbActors);
		pScene->scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		RenderHandler::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
	}

	RenderHandler::finishRender();


}

void exitCallback(void)
{
	delete sCamera;
	pVehicleController->releasePhysicsVehicle();
	delete pVehicleController;
	pVehicleController = nullptr;

	pScene->cleanupPhysics();
	delete pScene;
	pScene = nullptr;
}
}

void renderLoop()
{
	sCamera = new RenderHandler::Camera(PxVec3(10.0f, 10.0f, 10.0f), PxVec3(-0.6f, -0.2f, -0.7f));

	RenderHandler::setupDefaultWindow("PhysX Physics Vehicle Test");
	RenderHandler::setupDefaultRenderState();

	glutIdleFunc(idleCallback);
	glutDisplayFunc(renderCallback);
	glutKeyboardFunc(keyboardCallback);
	glutKeyboardUpFunc(keyboardReleasedCallback);
	glutMouseFunc(mouseCallback);
	//glutMotionFunc(motionCallback);
	//motionCallback(0, 0);

	atexit(exitCallback);

	setupPhysicsScene();
	glutMainLoop();

}

void main()
{
	renderLoop();
}
