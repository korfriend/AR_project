#include "rigidBody.h"

CiRigidBody::CiRigidBody(void)
{
	m_bodyType = 0;
	m_simulationSpace = NULL;
}

CiRigidBody::~CiRigidBody(void)
{
	m_simulationSpace = NULL;

	m_vertices.clear();
	m_indices.clear();
	m_fiducialPoint.clear();

	m_visVertices.clear();
	m_visFiducialPoint.clear();
}

void CiRigidBody::setSimulationSpace(Simulation* s)
{
	m_simulationSpace = s;
}
Simulation* CiRigidBody::getSimulationSpace()
{
	return m_simulationSpace;
}

void CiRigidBody::setType(int nType)
{
	m_bodyType = nType;
}
int CiRigidBody::getType()
{
	return m_bodyType;
}

void CiRigidBody::initVisVertices(void)
{
	m_visVertices.clear();
	m_visVertices.copyFromArray(m_vertices);

	m_visFiducialPoint.clear();
	m_visFiducialPoint.copyFromArray(m_fiducialPoint);
}

void CiRigidBody::addFiducialPoint(btVector3 fp)
{
	m_fiducialPoint.push_back(fp);
	m_visFiducialPoint.push_back(fp);
}

void CiRigidBody::translate(const btVector3& trs)
{
	btTransform t;
	t.setIdentity();
	t.setOrigin(trs);
	transform(t);
}
void CiRigidBody::rotate(const btQuaternion& rot, bool center)
{
	if (center) {
		btVector3 center = calcCenterPoint();
		translate(-center);

		btTransform t;
		t.setIdentity();
		t.setRotation(rot);
		transform(t);

		translate(center);
	}
	else {
		btTransform t;
		t.setIdentity();
		t.setRotation(rot);
		transform(t);
	}
}

void CiRigidBody::transform(const btTransform& trs)
{
	for (int i = 0, ni = m_visVertices.size(); i < ni; i++) {
		btVector3 a = trs * m_visVertices[i];
		m_visVertices[i] = a;
	}

	for (int i = 0, ni = m_visFiducialPoint.size(); i < ni; i++) {
		btVector3 a = trs * m_visFiducialPoint[i];
		m_visFiducialPoint[i] = a;
	}
}

btVector3 CiRigidBody::calcCenterPoint(void)
{
	int nSize = m_visVertices.size();
	btScalar x = 0;
	btScalar y = 0;
	btScalar z = 0;

	for (int i = 0; i < nSize; i++) {
		btVector3 p = m_visVertices[i];
		x += p.x();
		y += p.y();
		z += p.z();
	}

	x /= nSize;
	y /= nSize;
	z /= nSize;

	return btVector3(x, y, z);
}