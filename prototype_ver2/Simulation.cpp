#include "Simulation.h"

Simulation::Simulation()
{
	fTimeStep = 1.0 / 60.0;
	fCurrentTime = fTimeStep;
	fAccumulator = 0;

	v3Gravity = btVector3(0.0f, -0.00981f, 0.0f);
}

Simulation::~Simulation()
{
	destroySimulation();
}

void Simulation::destroySimulation()
{
	printf("destroySimulation\n");
	if(rigidBodies.size()) {
		for(int i=0, ni=rigidBodies.size(); i<ni; i++) {
			delete rigidBodies[i];
		}
	}
	if(softBodies.size()) {
		for(int i=0, ni=softBodies.size(); i<ni; i++) {
			printf("%d\n", i);
			delete softBodies[i];
		}
	}

	rigidBodies.clear();
	softBodies.clear();
}

void Simulation::initSSUDeform(const char* pcDataRoot)
{
	// skin
	char skinPath[MAX_PATH];
	sprintf_s(skinPath, sizeof(char)*MAX_PATH, "%s\\skin.obj", pcDataRoot);

	CiRigidBody* pgb_skin = CiSoftBodyHelpers::CreateFromTriMeshFile(skinPath, false);
	pgb_skin->setSimulationSpace(this);
	pgb_skin->setType(CiRigidBody::bodyType::GHOST);
	rigidBodies.push_back(pgb_skin);

	// obj
	CiSoftBody* psb = initSoftBody(pcDataRoot);
	psb->setSimulationSpace(this);

	softBodies.push_back(psb);
}

CiSoftBody* Simulation::initSoftBody(const char* pcDataRoot)
{
	/*
	char node[] = "Data\\brain.node";			// brain_c.node
	char ele[] = "Data\\brain.ele";				// brain.ele
	char face[] = "Data\\brain.face";				// brain surface
	char link[] = "Data\\brain.link";				// brain link
	char obj[] = "Data\\brain.obj";				// brainT_c

	char node2[] = "Data\\ventricle.node";		//ventricle_c
	char ele2[] = "Data\\ventricle.ele";			//ventricle
	char face2[] = "Data\\ventricle.face";			// ventricle surface
	char link2[] = "Data\\ventricle.link";			// ventricle link
	char obj2[] = "Data\\ventricle.obj";			//ventricleT_c(half)

	char hetero[] = "Data\\brain-ventricle.het";
	*/

	char node[MAX_PATH], ele[MAX_PATH], face[MAX_PATH], link[MAX_PATH], obj[MAX_PATH];
	char node2[MAX_PATH], ele2[MAX_PATH], face2[MAX_PATH], link2[MAX_PATH], obj2[MAX_PATH];
	char hetero[MAX_PATH];

	sprintf_s(node, sizeof(char)*MAX_PATH, "%s\\brain.node", pcDataRoot);
	sprintf_s(ele, sizeof(char)*MAX_PATH, "%s\\brain.ele", pcDataRoot);
	sprintf_s(face, sizeof(char)*MAX_PATH, "%s\\brain.face", pcDataRoot);
	sprintf_s(link, sizeof(char)*MAX_PATH, "%s\\brain.link", pcDataRoot);
	sprintf_s(obj, sizeof(char)*MAX_PATH, "%s\\brain.obj", pcDataRoot);

	sprintf_s(node2, sizeof(char)*MAX_PATH, "%s\\ventricle.node", pcDataRoot);
	sprintf_s(ele2, sizeof(char)*MAX_PATH, "%s\\ventricle.ele", pcDataRoot);
	sprintf_s(face2, sizeof(char)*MAX_PATH, "%s\\ventricle.face", pcDataRoot);
	sprintf_s(link2, sizeof(char)*MAX_PATH, "%s\\ventricle.link", pcDataRoot);
	sprintf_s(obj2, sizeof(char)*MAX_PATH, "%s\\ventricle.obj", pcDataRoot);

	sprintf_s(hetero, sizeof(char)*MAX_PATH, "%s\\brain-ventricle.het", pcDataRoot);

	//
	float fMassg[] = { 1.5, 0.1 };			// mass
	float kLSTg[] = { 0.01, 0.01 };			// 0.7, 0.01
	float kASTg[] = { 0.01, 0.01 };			// 0.8, 0.01
	float kVSTg[] = { 0.01, 0.01 };			// 1.0, 0.01
	float kESTg[] = { 0, 0 };
	float kYoungsModulus[] = { 30, 1.1 };	// young's modulus (CSf: 0.00016, Pia meter: 1.1)
	float kPoissonRatio[] = { 0, 0 };		// poisson ratio (0.45~0.49)

	int nIterationCnt = 10;					// 10
	float kDamping = 0.3;


	// material setting //
	CiSoftBody::Material* pm[2];
	for (int i = 0; i < 2; i++) {
		pm[i] = new(btAlignedAlloc(sizeof(CiSoftBody::Material), 16)) CiSoftBody::Material();
		pm[i]->m_kLST = kLSTg[i];
		pm[i]->m_kAST = kASTg[i];
		pm[i]->m_kVST = kVSTg[i];
	}

	
	bool bFlipYZ = false;
	int nStartIdx = 1;
	
	btVector3 center = CiSoftBodyHelpers::getCenter(node, nStartIdx, bFlipYZ);
	btVector3 trans(0, 0, 0);
	CiSoftBody*	psbTetra1 = CiSoftBodyHelpers::CreateFromTetGenFile(ele, face, node, link, false, true, true, fMassg[0], pm[0], nStartIdx, bFlipYZ, 1, true, center, trans);
	CiSoftBody* psb1 = CiSoftBodyHelpers::generateHybridModel(psbTetra1, obj, bFlipYZ, 1, false, center, trans);

	CiSoftBody*	psbTetra2 = CiSoftBodyHelpers::CreateFromTetGenFile(ele2, face2, node2, link2, false, true, true, fMassg[1], pm[1], nStartIdx, bFlipYZ, 1, false, center, trans);
	CiSoftBody* psb2 = CiSoftBodyHelpers::generateHybridModel(psbTetra2, obj2, bFlipYZ, 1, false, center, trans);

	CiSoftBody* psb = CiSoftBodyHelpers::mergeTetra(psbTetra1, psbTetra2, hetero);
	
	
	/*
	bool bFlipYZ = false;
	int nStartIdx = 1;
	btVector3 center = CiSoftBodyHelpers::getCenter(node, nStartIdx, bFlipYZ);
	btVector3 trans(0, 0, 0);
	CiSoftBody*	psb = CiSoftBodyHelpers::CreateFromTetGenFile(ele, face, node, link, false, true, true, fMassg[0], pm[0], nStartIdx, bFlipYZ, 1, true, center, trans);
	*/

	/// Properties //
	psb->m_cfg.kDP = kDamping;			// [0-1] Damping Coefficient					  (댐핑과 동시에 속도가 느려짐)
	psb->m_cfg.m_contactMargin = 1;

	/// Pose Matching //
	//psb->m_cfg.kMT = 0.02;				// Pose Matching Coefficient (0.02) [0,1]
	//psb->setPose(false, true);			// When on, the soft body tries to maintain its original volume / shape.

	/// iteration //
	psb->m_cfg.timescale = 1;				// 시간 조절 (객체마다 독립적으로 계산할 수 있음) (0이면 정지)
	psb->m_cfg.piterations = nIterationCnt;	// 1일때보다, stiffness 변수들 영향을 잘받음
	psb->m_cfg.viterations = 0;


	printf("== Constraint 초기값 계산 ==\n");
	psb->initConstraints();

	return psb;
}

void Simulation::initTool(const char* pcDataRoot)
{
	// tool
	char toolPath[MAX_PATH];
	sprintf_s(toolPath, sizeof(char)*MAX_PATH, "%s\\operationTool.obj", pcDataRoot);

	CiRigidBody* pgb_tool = CiSoftBodyHelpers::CreateFromTriMeshFile(toolPath, false);
	pgb_tool->setSimulationSpace(this);
	pgb_tool->setType(CiRigidBody::bodyType::TOOL);

	pgb_tool->addFiducialPoint(btVector3(-0.9, -68.0838, 1.49));
	pgb_tool->addFiducialPoint(btVector3(-0.9, 35.0838, 1.49));

	pgb_tool->translate(btVector3(0, 1000, 0));

	rigidBodies.push_back(pgb_tool);
}

void Simulation::stepPhysics()
{
	if (fAccumulator >= fTimeStep)
	{
		//printf("stepPhysics (%f %f)\n", fAccumulator, fTimeStep);
		computeForces();
		integrate(fTimeStep);
		updateConstraints(fTimeStep);
		fAccumulator -= fTimeStep;
	}
}
void Simulation::computeForces()
{
	// gravity //
	for (int i = 0; i < softBodies.size(); i++) {
		softBodies[i]->setForce(btVector3(0, 0, 0));
		//softBodies[i]->addForce(v3Gravity * 3);
	}
}
void Simulation::updateConstraints(float fDeltaTime)
{
	for (int i = 0; i < softBodies.size(); i++) {
		softBodies[i]->solveConstraints();
	}
}
void Simulation::integrate(float fDeltaTime)
{
	float dt = fDeltaTime;

	for (int i = 0; i < softBodies.size(); i++) {
		CiSoftBody* psb = softBodies[i];
		psb->m_sst.sdt = dt * psb->m_cfg.timescale;
		psb->m_sst.isdt = 1 / psb->m_sst.sdt;
	}

	// integrate //
	for (int i = 0; i < softBodies.size(); i++) {
		CiSoftBody* psb = softBodies[i];

		for (int j = 0, nj = softBodies[i]->m_nodes.size(); j < nj; j++) {
			CiSoftBody::Node& n = psb->m_nodes[j];
			n.m_q = n.m_x;
			n.m_v += n.m_f * n.m_im * psb->m_sst.sdt;
			n.m_x += n.m_v * psb->m_sst.sdt;
			n.m_f = btVector3(0, 0, 0);
		}
	}
}

void Simulation::accumulateTime(float fTime)
{
	fAccumulator += fTime;
}
void Simulation::setTimeStep(float fDeltaTime)
{
	fTimeStep = fDeltaTime;
}
float Simulation::getTimeStep(void)
{
	return fTimeStep;
}