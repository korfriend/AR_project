#ifndef _RIGID_BODY_H
#define _RIGID_BODY_H

#include "btVector3.h"
#include "btMatrix3x3.h"
#include "btTransform.h"

#include "btAlignedAllocator.h"
#include "btAlignedObjectArray.h"
#include "btPolarDecomposition.h"

#include "softbody.h"

class Simulation;	// forward declartions

class CiRigidBody
{
public:
	CiRigidBody(void);
	~CiRigidBody(void);
	
	void translate(const btVector3& trs);
	void rotate(const btQuaternion& rot, bool center=true);
	void transform(const btTransform& trs);

	void initVisVertices(void);
	btVector3 calcCenterPoint(void);
	void addFiducialPoint(btVector3 fp);

	void setType(int nType);
	int getType(void);
	struct	bodyType {
		enum _ {
			RIGID,
			GHOST,
			TOOL
		};
	};

	void setSimulationSpace(Simulation* s);
	Simulation* getSimulationSpace();

public:
	Simulation* m_simulationSpace;
	int m_bodyType;

	btAlignedObjectArray<btVector3> m_vertices;
	btAlignedObjectArray<int> m_indices;
	btAlignedObjectArray<btVector3> m_fiducialPoint;

	// 가시화용
	btAlignedObjectArray<btVector3> m_visVertices;
	btAlignedObjectArray<btVector3> m_visFiducialPoint;
};

#endif