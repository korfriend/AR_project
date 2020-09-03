#pragma once

#include <stdio.h>
#include <stdlib.h>

#include "btVector3.h"
#include "rigidBody.h"
#include "softbody.h"
#include "softBodyHelper.h"

#define PI 3.1415926536f
#define EPSILON  0.0000001f
#define GRID_SIZE 10

class Simulation {
private:
	// solver iteration //
	float fTimeStep;
	float fCurrentTime;
	double fAccumulator;

	btVector3 v3Gravity;

public:
	btAlignedObjectArray<CiRigidBody*> rigidBodies;
	btAlignedObjectArray<CiSoftBody*> softBodies;

public:
	Simulation();
	~Simulation();
	void destroySimulation();


	void stepPhysics(void);
	void computeForces(void);
	void integrate(float fDeltaTime);
	void updateConstraints(float fDeltaTime);
	void setTimeStep(float fDeltaTime);
	float getTimeStep(void);
	void accumulateTime(float fTime);

	// init object, tool //
	void initSSUDeform(const char* pcDataRoot);
	void initTool(const char* pcDataRoot);
	CiSoftBody* initSoftBody(const char* pcDataRoot);
};