#include "softbody.h"
#include "Simulation.h"


CiSoftBody::CiSoftBody()
{
	initDefaults();
}
CiSoftBody::CiSoftBody(int iNodeCnt, const btVector3* x, const btScalar* m)
{	
	/* Init		*/ 
	initDefaults();

	/* Default material	*/ 
	Material*	pm=appendMaterial();
	pm->m_kLST	=	1;
	pm->m_kAST	=	1;
	pm->m_kVST	=	1;

	/* Nodes			*/ 
	m_nodes.resize(iNodeCnt);
	for(int i=0,ni=iNodeCnt;i<ni;++i)
	{	
		Node&	n=m_nodes[i];
		ZeroInitialize(n);

		n.m_idx = i;
		n.m_x		=	x?*x++:btVector3(0,0,0);
		n.m_q		=	n.m_x;
		n.m_x0      =   n.m_x;
		n.m_im		=	m?*m++:1;
		n.m_im		=	n.m_im>0?1/n.m_im:0;

		n.m_surface = false;
		n.m_nAdjLinkCnt = 0;
	}
}
void CiSoftBody::initDefaults()
{
	m_simulationSpace = NULL;
	m_child = NULL;
	m_childCnt = 0;

	m_cfg.timescale		=	1;
	m_cfg.viterations	=	0;
	m_cfg.piterations	=	10;
	m_cfg.kMT			=	0.1;
	m_cfg.kDP			=	0.05;
	m_cfg.m_draw		=	DRAW_INIT_PROCESS;
	

	m_pose.m_bframe		=	false;
	m_pose.m_volume		=	0;
	m_pose.m_com		=	btVector3(0,0,0);
	m_pose.m_rot.setIdentity();
	m_pose.m_scl.setIdentity();

	setSolver(eSolverPresets::Positions);

	computeCenter();
}
CiSoftBody::~CiSoftBody()
{
	for(int i=0;i<m_materials.size();++i) 
		btAlignedFree(m_materials[i]);
}


void CiSoftBody::setSimulationSpace(Simulation* s)
{
	m_simulationSpace = s;
}
Simulation* CiSoftBody::getSimulationSpace()
{
	return m_simulationSpace;
}


void CiSoftBody::setNodeColor(btScalar r, btScalar g, btScalar b, btScalar a)
{
	for(int i=0,ni=m_nodes.size(); i<ni; i++) {
		Node& n = m_nodes[i];
		n.m_color = btVector3(r, g, b);
		n.m_colorAlpha = a;
	}
}
void CiSoftBody::setFaceColor(btScalar r, btScalar g, btScalar b, btScalar a)
{
	for(int i=0,ni=m_faces.size(); i<ni; i++) {
		Face& f = m_faces[i];
		f.m_color = btVector3(r, g, b);
		f.m_colorAlpha = a;
	}
}
void CiSoftBody::setTetraColor(btScalar r, btScalar g, btScalar b, btScalar a)
{
	for(int i=0,ni=m_tetras.size(); i<ni; i++) {
		Tetra& t = m_tetras[i];
		t.m_color = btVector3(r, g, b);
		t.m_colorAlpha = a;
	}
}


bool CiSoftBody::checkLink(int node0,int node1)
{
	return(checkLink(&m_nodes[node0],&m_nodes[node1]));
}
bool CiSoftBody::checkLink(const Node* node0,const Node* node1)
{
	const Node*	n[]={node0,node1};
	for(int i=0,ni=m_links.size();i<ni;++i)
	{
		const Link&	l=m_links[i];
		if(	(l.m_n[0]==n[0]&&l.m_n[1]==n[1])||
			(l.m_n[0]==n[1]&&l.m_n[1]==n[0]))
		{
			return(true);
		}
	}
	return(false);
}
bool CiSoftBody::checkFace(int node0,int node1,int node2)
{
	const Node*	n[]={	&m_nodes[node0],
		&m_nodes[node1],
		&m_nodes[node2]};
	for(int i=0,ni=m_faces.size();i<ni;++i)
	{
		const Face&	f=m_faces[i];
		int			c=0;
		for(int j=0;j<3;++j)
		{
			if(	(f.m_n[j]==n[0])||
				(f.m_n[j]==n[1])||
				(f.m_n[j]==n[2])) c|=1<<j; else break;
		}
		if(c==7) return(true);
	}
	return(false);
}

CiSoftBody::Material* CiSoftBody::appendMaterial()
{
	Material* pm=new(btAlignedAlloc(sizeof(Material),16)) Material();
	if(m_materials.size()>0)
		*pm=*m_materials[0];
	else
		ZeroInitialize(*pm);
	m_materials.push_back(pm);

	return(pm);
}
CiSoftBody::Material* CiSoftBody::appendMaterial(Material* m)
{
	Material* pm = appendMaterial();
	memcpy(pm, m, sizeof(Material));

	return(pm);
}
void CiSoftBody::appendNode(const btVector3& x,btScalar m)
{
	if(m_nodes.capacity()==m_nodes.size())
	{
		m_nodes.reserve(m_nodes.size()*2+1);
	}
	m_nodes.push_back(Node());
	Node&			n=m_nodes[m_nodes.size()-1];
	ZeroInitialize(n);
	n.m_idx			=	m_nodes.size()-1;
	n.m_x			=	x;
	n.m_q			=	n.m_x;
	n.m_x0          =   n.m_x;
	n.m_im			=	m>0?1/m:0;
}
void CiSoftBody::appendLink(int model,Material* mat)
{
	Link l;
	if(model>=0) {l=m_links[model];}
	else { ZeroInitialize(l);}

	m_links.push_back(l);
}
void CiSoftBody::appendLink(int node0, int node1, Material* mat, bool bcheckexist)
{
	appendLink(&m_nodes[node0],&m_nodes[node1],mat,bcheckexist);
}
void CiSoftBody::appendLink(Node* node0, Node* node1, Material* mat, bool bcheckexist)
{
	if((!bcheckexist)||(!checkLink(node0,node1)))
	{
		appendLink(-1,mat);
		Link&	l=m_links[m_links.size()-1];
		l.m_n[0]		=	node0;
		l.m_n[1]		=	node1;
		l.m_rl			=	(l.m_n[0]->m_x-l.m_n[1]->m_x).length();
		l.m_idx			=	m_links.size()-1;
	}
}
void CiSoftBody::appendFace(int model,Material* mat)
{
	Face f;
	if(model>=0){ f=m_faces[model]; }
	else{ ZeroInitialize(f);}

	m_faces.push_back(f);
}
void CiSoftBody::appendFace(int node0,int node1,int node2,Material* mat)
{
	if (node0==node1)
		return;
	if (node1==node2)
		return;
	if (node2==node0)
		return;

	appendFace(-1,mat);
	Face&	f=m_faces[m_faces.size()-1];
	btAssert(node0!=node1);
	btAssert(node1!=node2);
	btAssert(node2!=node0);
	f.m_n[0]	=	&m_nodes[node0];
	f.m_n[1]	=	&m_nodes[node1];
	f.m_n[2]	=	&m_nodes[node2];
	f.m_ra		=	AreaOf(	f.m_n[0]->m_x, f.m_n[1]->m_x, f.m_n[2]->m_x);	
}
void CiSoftBody::appendFaceTet(int node0,int node1,int node2, int node3)
{
	if (node0==node1)
		return;
	if (node1==node2)
		return;
	if (node2==node0)
		return;

	btAssert(node0!=node1);
	btAssert(node1!=node2);
	btAssert(node2!=node0);

	Tetra*	t = &m_tetras[m_tetras.size()-1];
	Face f[4];

	ZeroInitialize(f[0]);
	f[0].m_n[0]	=	&m_nodes[node0];
	f[0].m_n[1]	=	&m_nodes[node2];
	f[0].m_n[2]	=	&m_nodes[node1];
	f[0].m_ra		=	AreaOf(	f[0].m_n[0]->m_x, f[0].m_n[1]->m_x, f[0].m_n[2]->m_x);
	f[0].m_idx = m_tetrasFaces.size()-1;
	m_tetrasFaces.push_back(f[0]);

	// 1
	ZeroInitialize(f[1]);
	f[1].m_n[0]	=	&m_nodes[node1];
	f[1].m_n[1]	=	&m_nodes[node3];
	f[1].m_n[2]	=	&m_nodes[node0];
	f[1].m_ra		=	AreaOf(	f[1].m_n[0]->m_x, f[1].m_n[1]->m_x, f[1].m_n[2]->m_x);
	f[1].m_idx = m_tetrasFaces.size()-1;
	m_tetrasFaces.push_back(f[1]);
	
	// 2
	ZeroInitialize(f[2]);
	f[2].m_n[0]	=	&m_nodes[node2];
	f[2].m_n[1]	=	&m_nodes[node0];
	f[2].m_n[2]	=	&m_nodes[node3];
	f[2].m_ra		=	AreaOf(	f[2].m_n[0]->m_x, f[2].m_n[1]->m_x, f[2].m_n[2]->m_x);
	f[2].m_idx = m_tetrasFaces.size()-1;
	m_tetrasFaces.push_back(f[2]);

	// 3
	ZeroInitialize(f[3]);
	f[3].m_n[0]	=	&m_nodes[node3];
	f[3].m_n[1]	=	&m_nodes[node1];
	f[3].m_n[2]	=	&m_nodes[node2];
	f[3].m_ra		=	AreaOf(	f[3].m_n[0]->m_x, f[3].m_n[1]->m_x, f[3].m_n[2]->m_x);
	f[3].m_idx = m_tetrasFaces.size()-1;
	m_tetrasFaces.push_back(f[3]);
}
void CiSoftBody::appendTetra(int model, Material* mat)
{
	Tetra	t;
	if(model>=0) {
		t=m_tetras[model];
	}
	else
	{ 
		ZeroInitialize(t);
	}

	m_tetras.push_back(t);
}
void CiSoftBody::appendTetra(int node0, int node1, int node2, int node3, Material* mat)
{
	appendTetra(-1,mat);
	Tetra&	t=m_tetras[m_tetras.size()-1];
	t.m_n[0]	=	&m_nodes[node0];
	t.m_n[1]	=	&m_nodes[node1];
	t.m_n[2]	=	&m_nodes[node2];
	t.m_n[3]	=	&m_nodes[node3];
	t.m_v0		=	VolumeOf(t.m_n[0]->m_x,t.m_n[1]->m_x,t.m_n[2]->m_x,t.m_n[3]->m_x);
	t.m_rv		=	t.m_v0;
	t.m_idx = m_tetras.size() - 1;
	
	// angle //
	Node x[4];
	x[0] = *t.m_n[0];
	x[1] = *t.m_n[1];
	x[2] = *t.m_n[2];
	x[3] = *t.m_n[3];

	btVector3 n1 = btCross( (x[2].m_x-x[0].m_x), (x[3].m_x-x[0].m_x) );	n1 /= n1.length2();
	btVector3 n2 = btCross( (x[3].m_x-x[1].m_x), (x[2].m_x-x[1].m_x) );	n2 /= n2.length2();
				
	n1.normalize();
	n2.normalize();
	btScalar dot = btDot(n1, n2);
	t.m_a0		= acos(dot);
}

void CiSoftBody::addForce(const btVector3& force)
{
	for(int i=0, ni=m_nodes.size(); i < ni; i++) {
		Node& n = m_nodes[i];

		if(m_nodes[i].m_im > 0) {
			n.m_f += force;
		}
	}
}
void CiSoftBody::addForce(const btVector3& force, int node)
{
	Node& n = m_nodes[node];
	if(m_nodes[node].m_im > 0) {
		n.m_f += force;
	}
}
void CiSoftBody::setForce(const btVector3& force)
{
	for(int i=0, ni=m_nodes.size(); i < ni; i++) {
		Node& n = m_nodes[i];

		if(m_nodes[i].m_im > 0) {
			n.m_f = force;
		}
	}
}
void CiSoftBody::addVelocity(const btVector3& velocity)
{

}
void CiSoftBody::addVelocity(const btVector3& velocity, int node)
{

}
void CiSoftBody::setVelocity(const btVector3& velocity)
{

}
void CiSoftBody::setMass(int node, btScalar mass)
{
	m_nodes[node].m_im=mass>0?1/mass:0;
}
void CiSoftBody::setTotalMass(btScalar mass, bool fromfaces)
{
	int i;

	if(fromfaces)
	{
		for(i=0;i<m_nodes.size();++i)
		{
			m_nodes[i].m_im=0;
		}
		for(i=0;i<m_faces.size();++i)
		{
			const Face&		f=m_faces[i];
			const btScalar	twicearea=AreaOf(	f.m_n[0]->m_x,
				f.m_n[1]->m_x,
				f.m_n[2]->m_x);
			for(int j=0;j<3;++j)
			{
				f.m_n[j]->m_im+=twicearea;
			}
		}
		for( i=0;i<m_nodes.size();++i)
		{
			m_nodes[i].m_im=1/m_nodes[i].m_im;
		}
	}
	const btScalar	tm=getTotalMass();
	const btScalar	itm=1/tm;
	for( i=0;i<m_nodes.size();++i)
	{
		m_nodes[i].m_im/=itm*mass;
	}
}
btScalar CiSoftBody::getMass(int node)
{
	return(m_nodes[node].m_im>0?1/m_nodes[node].m_im:0);
}
btScalar CiSoftBody::getTotalMass()
{
	btScalar mass=0;
	for(int i=0;i<m_nodes.size();++i)
	{
		mass+=getMass(i);
	}
	return(mass);
}
void CiSoftBody::setMeshType(int type)
{
	m_cfg.m_meshType = type;
}
btScalar CiSoftBody::getVolume()
{
	CiSoftBody* psb = this;

	btScalar	vol=0;
	if(m_nodes.size()>0)
	{
		int i,ni;

		//const btVector3	org=m_nodes[0].m_x;
		btVector3 org(0, 0, 0);

		for(int i=0; i< m_nodes.size(); i++) {
			org += m_nodes[i].m_x;
		}
		org /= m_nodes.size();

		switch(psb->m_cfg.m_meshType) {
			case cfgMeshType::Mesh:
			{
				for(i=0,ni=m_faces.size();i<ni;++i)
				{
					const Face&	f=m_faces[i];
					vol += VolumeOf(org, f.m_n[0]->m_x, f.m_n[1]->m_x, f.m_n[2]->m_x);
				}
				break;
			}
			case cfgMeshType::Tetra: 
			{
				btScalar tetVol = 0;
				for(i=0; i< m_tetras.size(); ++i) {
					Node* n1 = m_tetras[i].m_n[0];
					Node* n2 = m_tetras[i].m_n[1];
					Node* n3 = m_tetras[i].m_n[2];
					Node* n4 = m_tetras[i].m_n[3];

					vol += VolumeOf(n1->m_x, n2->m_x, n3->m_x, n4->m_x);
				}
				break;
			}
		}
	}
	return(vol);
}
void CiSoftBody::updateNormals()
{
	if(m_cfg.m_meshType == cfgMeshType::Mesh) {
		const btVector3	zv(0,0,0);
		int i,ni;

		for(i=0,ni=m_nodes.size();i<ni;++i)
		{
			m_nodes[i].m_n=zv;
		}
		for(i=0,ni=m_faces.size();i<ni;++i)
		{
			CiSoftBody::Face&	f=m_faces[i];
			const btVector3		n=btCross(f.m_n[1]->m_x-f.m_n[0]->m_x,
				f.m_n[2]->m_x-f.m_n[0]->m_x);
			f.m_normal=n.normalized();
			f.m_n[0]->m_n+=n;
			f.m_n[1]->m_n+=n;
			f.m_n[2]->m_n+=n;
		}
		for(i=0,ni=m_nodes.size();i<ni;++i)
		{
			btScalar len = m_nodes[i].m_n.length();
			if (len>SIMD_EPSILON)
				m_nodes[i].m_n /= len;
		}
	}
}
void CiSoftBody::updateArea(bool averageArea)
{
	if(m_cfg.m_meshType == cfgMeshType::Mesh) {
		int i,ni;

		/* Face area		*/ 
		for(i=0,ni=m_faces.size();i<ni;++i)
		{
			Face&		f=m_faces[i];
			f.m_ra	=	AreaOf(f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x);
		}
	
		/* Node area		*/ 
		btAlignedObjectArray<int>	counts;
		counts.resize(m_nodes.size(),0);
		int index = 0;

		if (averageArea)
		{
			for(i=0,ni=m_nodes.size();i<ni;++i)
			{
				m_nodes[i].m_area	=	0;
			}
			for(i=0,ni=m_faces.size();i<ni;++i)
			{
				CiSoftBody::Face&	f=m_faces[i];
				for(int j=0;j<3;++j)
				{
					index=(int)(f.m_n[j]-&m_nodes[0]);
					counts[index]++;
					f.m_n[j]->m_area+=btFabs(f.m_ra);
				}
			}
			for(i=0,ni=m_nodes.size();i<ni;++i)
			{
				if(counts[i]>0)
					m_nodes[i].m_area /= (btScalar)counts[i];
				else
					m_nodes[i].m_area = 0;
			}
		}
		else
		{
			// initialize node area as zero
			for(i=0,ni=m_nodes.size();i<ni;++i)
			{
				m_nodes[i].m_area=0;	
			}

			for(i=0,ni=m_faces.size();i<ni;++i)
			{
				CiSoftBody::Face&	f=m_faces[i];

				for(int j=0;j<3;++j)
				{
					f.m_n[j]->m_area += f.m_ra;
				}
			}

			for(i=0,ni=m_nodes.size();i<ni;++i)
			{
				m_nodes[i].m_area *= 0.3333333f;
			}
		}
	}
}
void CiSoftBody::updateConstants()
{
	updateNormals();
	updateArea();
}
void CiSoftBody::updateSurfaceVertices()
{
	//return;
	float mrgParam = 0.7;	// 0.7
	float fixParam = 1.2;	// 1.2
	for (int i = 0, ni = m_surfaceMeshNode.size(); i < ni; i++) {
		CiSoftBody::Node* n = &m_surfaceMeshNode[i];
		int tetIdx = m_attachedTetraIdx[i];

		CiSoftBody::Tetra* t = &m_tetras[tetIdx];
		btVector3 newPos;
		newPos.setZero();

		for (int j = 0; j < 4; j++) {
			newPos += t->m_n[j]->m_x * (m_bary[i].m_floats[j]);
		}

		btVector3 centroid = (t->m_n[0]->m_x + t->m_n[1]->m_x + t->m_n[2]->m_x + t->m_n[3]->m_x) / 4.0;
		btScalar length = (centroid - newPos).length();


		float dLen[4] = { 0 };
		for (int j = 0; j < 4; j++) {
			dLen[j] = (t->m_n[j]->m_x - n->m_x).length();
		}
		float minD = dLen[0];
		int minIdx = 0;
		for (int j = 1; j < 4; j++) {
			if (minD > dLen[j]) {
				minD = dLen[j];
				minIdx = j;
			}
		}
		n->m_x = t->m_n[minIdx]->m_x;

		/*
		if (length*mrgParam > m_avgSurfaceMargin) {
			// 대안...? (가장 가까운 Tetra점과 선형보간....)

			float dLen[4] = { 0 };
			for (int j = 0; j < 4; j++) {
				dLen[j] = (t->m_n[j]->m_x - n->m_x).length();
			}
			float minD = dLen[0];
			int minIdx = 0;
			for (int j = 1; j < 4; j++) {
				if (minD > dLen[j]) {
					minD = dLen[j];
					minIdx = j;
				}
			}

			float rate = m_avgSurfaceMargin * fixParam / length;
			n->m_x = t->m_n[minIdx]->m_x*(1 - rate) + newPos * (rate);
		}
		else {
			n->m_x = newPos;
		}
		*/
		
	}

	if (m_child != NULL) {
		// 이종형질인 경우
		mrgParam = 0.7;
		fixParam = 1.2;

		for (int s = 0; s < m_childCnt; s++) {
			for (int i = 0, ni = m_child[s].m_surfaceMeshNode.size(); i < ni; i++) {
				CiSoftBody::Node* n = &m_child[s].m_surfaceMeshNode[i];
				int tetIdx = m_child[s].m_attachedTetraIdx[i];

				CiSoftBody::Tetra* t = &m_tetras[tetIdx];
				btVector3 newPos;
				newPos.setZero();

				for (int j = 0; j < 4; j++) {
					newPos += t->m_n[j]->m_x * (m_child[s].m_bary[i].m_floats[j]);
				}

				btVector3 centroid = (t->m_n[0]->m_x + t->m_n[1]->m_x + t->m_n[2]->m_x + t->m_n[3]->m_x) / 4.0;
				btScalar length = (centroid - newPos).length();

				if (length*mrgParam > m_child[s].m_avgSurfaceMargin) {
					// 대안...? (가장 가까운 Tetra점과 선형보간....)
					float dLen[4] = { 0 };
					for (int j = 0; j < 4; j++) {
						dLen[j] = (t->m_n[j]->m_x - n->m_x).length();
					}
					float minD = dLen[0];
					int minIdx = 0;
					for (int j = 1; j < 4; j++) {
						if (minD > dLen[j]) {
							minD = dLen[j];
							minIdx = j;
						}
					}

					float rate = m_child[s].m_avgSurfaceMargin*fixParam / length;
					n->m_x = t->m_n[minIdx]->m_x*(1 - rate) + newPos * (rate);

				}
				else {
					n->m_x = newPos;
				}
			}
		}
	}
}

void CiSoftBody::transform(const btTransform& trs)
{
	for(int i=0,ni=m_nodes.size();i<ni;++i)
	{
		Node&	n=m_nodes[i];
		n.m_x=trs*n.m_x;
		n.m_x0 = n.m_x;
		n.m_q=trs*n.m_q;
		n.m_n=trs.getBasis()*n.m_n;
	}
	updateNormals();
	updateConstants();
}
void CiSoftBody::translate(const btVector3& trs)
{
	btTransform	t;
	t.setIdentity();
	t.setOrigin(trs);
	transform(t);
}
void CiSoftBody::rotate(const btQuaternion& rot)
{
	btTransform	t;
	t.setIdentity();
	t.setRotation(rot);
	transform(t);
}
void CiSoftBody::scale(const btVector3& scl)
{
	for(int i=0,ni=m_nodes.size();i<ni;++i)
	{
		Node&	n=m_nodes[i];
		n.m_x*=scl;
		n.m_x0 = n.m_x;
		n.m_q*=scl;
	}
	updateNormals();
	updateConstants();
}
void CiSoftBody::moveToOrigin(bool yAxisOnly)
{
	if(yAxisOnly == false) {
		btVector3	center(0, 0, 0);
		for(int i=0, ni=m_nodes.size(); i<ni; i++) {
			center += m_nodes[i].m_x;
		}
		center /= m_nodes.size();
		translate(-center);
	}

	btScalar minY = FLT_MAX;
	for(int i=0, ni=m_nodes.size(); i<ni; i++) {
		if(m_nodes[i].m_x.y() < minY) {
			minY = m_nodes[i].m_x.y();
		}
	}

	translate(btVector3(0, -minY, 0));
}
void CiSoftBody::computeCenter()
{
	int nNodes = m_nodes.size();

	if (nNodes > 0) {
		btVector3 center(0, 0, 0);

		for (int i = 0; i < nNodes; i++) {
			center += m_nodes[i].m_x0;
		}

		center /= nNodes;
		m_oriCenter = center;
	}
}

// solver, Constraint
void CiSoftBody::setSolver(eSolverPresets::_ preset)
{
	m_cfg.m_psequence.clear();
	m_cfg.m_vsequence.clear();

	switch(preset)
	{
		case eSolverPresets::Positions:
			m_cfg.m_psequence.push_back(ePSolver::Stretch);
			m_cfg.m_psequence.push_back(ePSolver::Volume);
			m_cfg.m_psequence.push_back(ePSolver::Bending);
			//m_cfg.m_psequence.push_back(ePSolver::SelfCollision);
			//m_cfg.m_psequence.push_back(ePSolver::GroundCollision);
			m_cfg.m_psequence.push_back(ePSolver::ToolCollision);
			break;	
		case eSolverPresets::Velocities:
			m_cfg.m_vsequence.push_back(eVSolver::Stretch);
			break;
	}
}
CiSoftBody::psolver_t CiSoftBody::getSolver(ePSolver::_ solver)
{
	switch(solver)
	{
		case ePSolver::Stretch:		
			return(&CiSoftBody::PSolveStretch);
		case ePSolver::Volume:
			return(&CiSoftBody::PSolveVolume);
		case ePSolver::Bending:
			return(&CiSoftBody::PSolveBending);
		case ePSolver::SelfCollision:
			return(&CiSoftBody::PSolveSelfCollision);
		case ePSolver::GroundCollision:
			return(&CiSoftBody::PSolveGroundCollision);
		case ePSolver::ToolCollision:
			return(&CiSoftBody::PSolveToolCollision);
		default:
		{
		}
	}
	return(0);
}
CiSoftBody::vsolver_t	CiSoftBody::getSolver(eVSolver::_ solver)
{
	return 0;
}
void CiSoftBody::setConstraint(Constraint& c, Node** n, int nodeCnt, btScalar imSum, btScalar rest, btScalar prime)
{
	c.m_isUse = true;

	if(n != NULL && nodeCnt > 0) {
		// Constraint 계산 식에 n개의 노드만 사용될때
		c.m_nodeCnt = nodeCnt;
		c.m_n = new Node* [c.m_nodeCnt];
		for(int i=0; i< nodeCnt; i++) {
			c.m_n[i] = n[i];
		}
	}
	else {
		// Constraint 계산 식에 전체 노드가 사용될때
		c.m_nodeCnt = m_nodes.size();
		c.m_n = new Node* [c.m_nodeCnt];
		for(int i=0; i< c.m_nodeCnt; i++) {
			c.m_n[i] = &m_nodes[i];
		}
	}
	c.m_rest = rest;
	c.m_prime = prime;
	c.m_im = imSum;
}

void CiSoftBody::initPose()
{
	int i, ni;

	/* Weights		*/
	const btScalar	omass = getTotalMass();
	const btScalar	kmass = omass * m_nodes.size() * 1000;
	btScalar		tmass = omass;
	m_pose.m_wgh.resize(m_nodes.size());
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		if (m_nodes[i].m_im <= 0) tmass += kmass;
	}
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node&	n = m_nodes[i];
		m_pose.m_wgh[i] = n.m_im > 0 ? 1 / (m_nodes[i].m_im*tmass) : kmass / tmass;
	}
	/* Pos		*/
	const btVector3	com = evaluateCom();
	m_pose.m_pos.resize(m_nodes.size());
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		m_pose.m_pos[i] = m_nodes[i].m_x - com;
	}
	m_pose.m_volume = getVolume();
	m_pose.m_com = com;
	m_pose.m_rot.setIdentity();
	m_pose.m_scl.setIdentity();

	/* Aqq		*/
	m_pose.m_aqq[0] =
		m_pose.m_aqq[1] =
		m_pose.m_aqq[2] = btVector3(0, 0, 0);
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		const btVector3&	q = m_pose.m_pos[i];
		const btVector3		mq = m_pose.m_wgh[i] * q;
		m_pose.m_aqq[0] += mq.x()*q;
		m_pose.m_aqq[1] += mq.y()*q;
		m_pose.m_aqq[2] += mq.z()*q;
	}
	m_pose.m_aqq = m_pose.m_aqq.inverse();

	updateConstants();
}
void CiSoftBody::updatePose()
{
	if (m_pose.m_bframe)
	{
		CiSoftBody::Pose&	pose = m_pose;
		const btVector3		com = evaluateCom();

		pose.m_com = com;

		btMatrix3x3		Apq;
		const btScalar	eps = SIMD_EPSILON;
		Apq[0] = Apq[1] = Apq[2] = btVector3(0, 0, 0);
		Apq[0].setX(eps); Apq[1].setY(eps * 2); Apq[2].setZ(eps * 3);
		for (int i = 0, ni = m_nodes.size(); i < ni; ++i)
		{
			const btVector3		a = pose.m_wgh[i] * (m_nodes[i].m_x - com);
			const btVector3&	b = pose.m_pos[i];
			Apq[0] += a.x()*b;
			Apq[1] += a.y()*b;
			Apq[2] += a.z()*b;
		}
		btMatrix3x3		r, s;
		PolarDecompose(Apq, r, s);
		pose.m_rot = r;
		pose.m_scl = pose.m_aqq*r.transpose()*Apq;

		if (m_pose.m_bframe && (m_cfg.kMT > 0))
		{
			const btMatrix3x3	posetrs = m_pose.m_rot;
			for (int i = 0, ni = m_nodes.size(); i < ni; ++i)
			{
				Node&	n = m_nodes[i];
				if (n.m_im > 0)
				{
					const btVector3	x = posetrs * m_pose.m_pos[i] + m_pose.m_com;
					n.m_x = Lerp(n.m_x, x, m_cfg.kMT);
				}
			}
		}
	}
}
btVector3 CiSoftBody::evaluateCom()
{
	btVector3	com(0, 0, 0);
	if (m_pose.m_bframe)
	{
		for (int i = 0, ni = m_nodes.size(); i < ni; ++i)
		{
			com += m_nodes[i].m_x*m_pose.m_wgh[i];
		}
	}
	return(com);
}

void CiSoftBody::initConstraints()
{
	initStretch();
	initBending();
	initVolume();
	initPose();
}
void CiSoftBody::initStretch()
{
	CiSoftBody* psb = this;

	switch (m_cfg.m_meshType) {
	case cfgMeshType::Mesh:
	{
		int nConstraintCnt = 0;
		int nConstraintIdx_stretch = 0;

		// memory allocation //
		for (int i = 0, ni = psb->m_links.size(); i < ni; i++) {
			Link&	l = psb->m_links[i];
			Node* x[2];
			x[0] = l.m_n[0];
			x[1] = l.m_n[1];

			if (x[0]->m_im < 0) { continue; }
			if (x[1]->m_im < 0) { continue; }

			nConstraintCnt++;
		}

		psb->m_stretchConstraints.resize(nConstraintCnt);

		for (int i = 0, ni = psb->m_links.size(); i < ni; i++) {
			Link&	l = psb->m_links[i];
			Node* x[2];
			x[0] = l.m_n[0];
			x[1] = l.m_n[1];

			btVector3 dir = x[0]->m_x - x[1]->m_x;
			btScalar dirLength = dir.length();

			btScalar imSum = x[0]->m_im + x[1]->m_im;

			if (imSum > 0) {
				setConstraint(psb->m_stretchConstraints[nConstraintIdx_stretch++], x, 2, imSum, dirLength, psb->m_materials[0]->m_kLST);
			}
		}
		break;
	}
	case cfgMeshType::Tetra:
	{
		int nConstraintCnt = 0;
		int nConstraintIdx_stretch = 0;

		// memory allocation //
		for (int i = 0, ni = psb->m_links.size(); i < ni; i++) {
			Link&	l = psb->m_links[i];
			Node* p[2];
			p[0] = l.m_n[0];
			p[1] = l.m_n[1];

			if (p[0]->m_im < 0) { continue; }
			if (p[1]->m_im < 0) { continue; }

			nConstraintCnt++;
		}

		psb->m_stretchConstraints.resize(nConstraintCnt);

		for (int i = 0, ni = psb->m_links.size(); i < ni; i++) {
			Link&	l = psb->m_links[i];
			Node* p[2];
			p[0] = l.m_n[0];
			p[1] = l.m_n[1];

			btVector3 dir = p[0]->m_x - p[1]->m_x;
			btScalar dirLength = dir.length();

			btScalar imSum = p[0]->m_im + p[1]->m_im;

			if (imSum > SIMD_EPSILON) {
				setConstraint(psb->m_stretchConstraints[nConstraintIdx_stretch], p, 2, imSum, dirLength, p[0]->m_mat->m_kLST);
				nConstraintIdx_stretch++;
			}
		}

		break;
	}
	default:
	{
	}
	}
}
void CiSoftBody::initBending()
{
	CiSoftBody* psb = this;
	switch (psb->m_cfg.m_meshType) {
	case cfgMeshType::Mesh:
	{
		int nNodes = psb->m_nodes.size();
		int nLinks = psb->m_links.size();
		int nFaces = psb->m_faces.size();

		// link와 관련된 변수 (link수 * 2)
		int** m_f = NULL;		// 1개의 line을 공유하는 2개 face 번호
		int** m_fn = NULL;		// 2개의 면을 이루는 노드 중, 공유되지 않는 2개 Node 번호

		m_f = new int*[nLinks];
		m_fn = new int*[nLinks];
		for (int i = 0; i < nLinks; i++) {
			m_f[i] = new int[2];
			m_f[i][0] = -1;
			m_f[i][1] = -1;

			m_fn[i] = new int[2];
			m_fn[i][0] = -1;
			m_fn[i][1] = -1;
		}

		// face와 관련된 변수 (face수 * 3)
		int** m_linkIdx = NULL;	// 1개의 면을 이루는 3개의 line

		m_linkIdx = new int*[nFaces];
		for (int i = 0; i < nFaces; i++) {
			m_linkIdx[i] = new int[3];
			m_linkIdx[i][0] = -1;
			m_linkIdx[i][1] = -1;
			m_linkIdx[i][2] = -1;
		}

		// dihedral angle 측정을 위한 인접한 두 면 확인 //////////////////////////////////////////////
#define IDX(_x_,_y_) ((_y_)*nNodes + (_x_))
		btAlignedObjectArray<int> chks;
		chks.resize(nNodes*nNodes, -1);

		int nLinkIdx = 0;
		for (int i = 0, ni = nFaces; i < ni; i++)
		{
			const int idx[] = {
				psb->m_faces[i].m_n[0]->m_idx,
				psb->m_faces[i].m_n[1]->m_idx,
				psb->m_faces[i].m_n[2]->m_idx
			};

			for (int j = 2, k = 0; k < 3; j = k++)
			{
				if (chks[IDX(idx[j], idx[k])] == -1)
				{
					chks[IDX(idx[j], idx[k])] = nLinkIdx;
					chks[IDX(idx[k], idx[j])] = nLinkIdx;
					nLinkIdx++;
				}
			}
		}
		for (int i = 0; i < nFaces; i++) {
			int I[3] = { 0 };
			I[0] = chks[IDX(psb->m_faces.at(i).m_n[2]->m_idx, psb->m_faces.at(i).m_n[0]->m_idx)];
			I[1] = chks[IDX(psb->m_faces.at(i).m_n[0]->m_idx, psb->m_faces.at(i).m_n[1]->m_idx)];
			I[2] = chks[IDX(psb->m_faces.at(i).m_n[1]->m_idx, psb->m_faces.at(i).m_n[2]->m_idx)];

			m_linkIdx[i][0] = I[0];
			m_linkIdx[i][1] = I[1];
			m_linkIdx[i][2] = I[2];
		}

		btAlignedObjectArray<int> pnFaceCount;
		pnFaceCount.resize(nLinks, 0);
		for (int i = 0; i < nFaces; i++) {
			int fn[3] = { 0 };
			for (int j = 0; j < 3; j++) {
				fn[j] = psb->m_faces[i].m_n[j]->m_idx;
			}
			for (int j = 0; j < 3; j++) {
				int lineIdx = m_linkIdx[i][j];
				if (pnFaceCount[lineIdx] < 2) {
					int ln[2] = { 0 };
					ln[0] = psb->m_links[lineIdx].m_n[0]->m_idx;
					ln[1] = psb->m_links[lineIdx].m_n[1]->m_idx;

					int targetNodeIdx = -1;
					for (int k = 0; k < 3; k++) {
						if (ln[0] != fn[k] && ln[1] != fn[k]) {
							targetNodeIdx = fn[k];
							break;
						}
					}
					int idx = pnFaceCount[lineIdx];
					m_f[lineIdx][idx] = i;
					m_fn[lineIdx][idx] = targetNodeIdx;
					pnFaceCount[lineIdx]++;
				}
			}
		}
#undef IDX

		// init diherdral, triangle //////////////////////////////////////////////
		int nConstraintCnt = 0;
		int nConstraintIdx_dihedral = 0;
		int nConstraintIdx_triangle = 0;

		for (int i = 0; i < psb->m_links.size(); i++) {
			CiSoftBody::Link* l = &psb->m_links[i];
			if (m_fn[i][0] == -1 || m_fn[i][1] == -1) {
				continue;
			}
			nConstraintCnt++;
		}
		m_bendingConstraints_dihedral.resize(nConstraintCnt);
		m_bendingConstraints_triangle.resize(nConstraintCnt * 4);

		for (int i = 0; i < psb->m_links.size(); i++) {
			CiSoftBody::Link* l = &psb->m_links[i];
			if (m_fn[i][0] == -1 || m_fn[i][1] == -1) {
				continue;
			}

			CiSoftBody::Node* p[4];
			p[0] = l->m_n[0];
			p[1] = l->m_n[1];
			p[2] = &psb->m_nodes[m_fn[i][0]];
			p[3] = &psb->m_nodes[m_fn[i][1]];

			// init triangle /////////////////////////////////////////////////////////////
			for (int j = 0; j < 2; j++) {
				Node* pCopy[3];
				pCopy[0] = p[2];
				pCopy[1] = p[3];
				pCopy[2] = p[j];

				btScalar w = p[2]->m_im + p[3]->m_im + 2 * p[j]->m_im;
				btVector3 center = (p[2]->m_x0 + p[3]->m_x0 + p[j]->m_x0) * 0.3333f;
				btVector3 dirCenter = p[j]->m_x0 - center;
				btScalar prevRestLength = dirCenter.length();

				setConstraint(m_bendingConstraints_triangle[nConstraintIdx_triangle++], pCopy, 3, w, prevRestLength, m_materials[0]->m_kAST);
			}
			for (int j = 2; j < 4; j++) {
				btScalar w = p[0]->m_im + p[1]->m_im + p[j]->m_im * 2;
				btVector3 center = (p[j]->m_x0 + p[0]->m_x0 + p[1]->m_x0) * 0.3333f;
				btVector3 dirCenter = p[j]->m_x0 - center;
				btScalar prevRestLength = dirCenter.length();

				Node* pCopy[3];
				pCopy[0] = p[0];
				pCopy[1] = p[1];
				pCopy[2] = p[j];
				setConstraint(m_bendingConstraints_triangle[nConstraintIdx_triangle++], pCopy, 3, w, prevRestLength, m_materials[0]->m_kAST);
			}

			// init dihedral //////////////////////////////////////////////////////////////
			btVector3 v = (p[0]->m_x0 + p[1]->m_x0) * 0.5f;
			btVector3 n1 = (p[2]->m_x0 - v);
			btScalar n1Len = n1.length();
			if (n1Len <= SIMD_EPSILON) { continue; }
			n1 /= n1Len;

			btVector3 n2 = (p[3]->m_x0 - v);
			btScalar n2Len = n2.length();
			if (n2Len <= SIMD_EPSILON) { continue; }
			n2 /= n2Len;

			btScalar d = btDot(n1, n2);
			if (d < -1.0)
				d = -1.0;
			else if (d > 1.0)
				d = 1.0;

			btScalar angle = acos(d);
			btScalar w = p[0]->m_im + p[1]->m_im + p[2]->m_im + p[3]->m_im;
			setConstraint(m_bendingConstraints_dihedral[nConstraintIdx_dihedral++], p, 4, w, angle, m_materials[0]->m_kAST);
		}

		SAFE_DELETE_VOLUME(m_f, 2);
		SAFE_DELETE_VOLUME(m_fn, 2);
		SAFE_DELETE_VOLUME(m_linkIdx, 3);
		break;
	}
	case cfgMeshType::Tetra:
	{
		// memory allocation //
		int nConstraintCnt = 0;
		int nConstraintIdx_bending = 0;

		for (int i = 0; i < psb->m_tetras.size(); i++) {
			Tetra* t = &psb->m_tetras[i];
			for (int j = 0; j < 4; j++) {
				Node* p[3];
				p[0] = psb->m_tetrasFaces[i * 4 + j].m_n[0];
				p[1] = psb->m_tetrasFaces[i * 4 + j].m_n[1];
				p[2] = psb->m_tetrasFaces[i * 4 + j].m_n[2];

				if (p[0]->m_im < 0) { continue; }
				if (p[1]->m_im < 0) { continue; }
				if (p[2]->m_im < 0) { continue; }

				nConstraintCnt++;
			}
		}

		psb->m_bendingConstraints_triangle.resize(nConstraintCnt);

		for (int i = 0; i < psb->m_tetras.size(); i++) {
			Tetra* t = &psb->m_tetras[i];
			for (int j = 0; j < 4; j++) {
				Node* p[3];
				p[0] = psb->m_tetrasFaces[i * 4 + j].m_n[0];
				p[1] = psb->m_tetrasFaces[i * 4 + j].m_n[1];
				p[2] = psb->m_tetrasFaces[i * 4 + j].m_n[2];

				btScalar imSum = p[0]->m_im + p[1]->m_im + 2 * p[2]->m_im;
				btVector3 center = (p[0]->m_x0 + p[1]->m_x0 + p[2]->m_x0) / 3.0;
				btVector3 dirCenter = p[2]->m_x0 - center;
				btScalar dirLength = dirCenter.length();

				if (imSum > SIMD_EPSILON) {
					setConstraint(psb->m_bendingConstraints_triangle[nConstraintIdx_bending], p, 3, imSum, dirLength, p[0]->m_mat->m_kAST);
					nConstraintIdx_bending++;
				}
			}
		}
		break;
	}
	}
}
void CiSoftBody::initVolume()
{
	CiSoftBody* psb = this;
	switch (psb->m_cfg.m_meshType) {
	case cfgMeshType::Mesh:
	{
		int nNodes = m_nodes.size();
		btScalar restVolume = getVolume();
		btScalar imSum = 0;

		for (int i = 0; i < nNodes; i++) {
			imSum += m_nodes[i].m_im;
		}

		m_volumeConstraints_surface.resize(1);
		setConstraint(m_volumeConstraints_surface[0], NULL, nNodes, imSum, restVolume, m_materials[0]->m_kVST);
		break;
	}
	case cfgMeshType::Tetra:
	{
		// memory allocation //
		int nConstraintCnt = 0;
		int nConstraintIdx_volume = 0;

		for (int i = 0; i < psb->m_tetras.size(); i++) {
			Tetra* t = &psb->m_tetras[i];

			bool isUse = true;
			for (int j = 0; j < 4; j++) {
				if (t->m_n[j]->m_im < 0) {
					isUse = false;
				}
			}
			if (isUse) {
				nConstraintCnt++;
			}
		}

		psb->m_volumeConstraints.resize(nConstraintCnt);

		for (int i = 0; i < psb->m_tetras.size(); i++) {
			Tetra* t = &psb->m_tetras[i];
			Node* p[4];

			for (int j = 0; j < 4; j++) {
				p[j] = t->m_n[j];
			}

			btScalar imSum = p[0]->m_im + p[1]->m_im + 2 * p[2]->m_im;
			btScalar volume = (VolumeOf(p[0]->m_x, p[1]->m_x, p[2]->m_x, p[3]->m_x));

			if (imSum > SIMD_EPSILON) {
				setConstraint(psb->m_volumeConstraints[nConstraintIdx_volume], p, 4, imSum, volume, p[0]->m_mat->m_kVST);
				nConstraintIdx_volume++;
			}
		}
		break;
	}
	}
}
void CiSoftBody::initEnergyPreserving()
{
	CiSoftBody* psb = this;
	if(psb->m_cfg.m_meshType == cfgMeshType::Tetra)
	{
		for(int i=0,ni=psb->m_tetras.size(); i<ni; i++) {

		}
	}
}
void CiSoftBody::PSolveStretch(CiSoftBody* psb)
{
	switch (psb->m_cfg.m_meshType) {
	case cfgMeshType::Mesh:
	{
		for (int i = 0, ni = psb->m_stretchConstraints.size(); i < ni; i++) {
			Constraint& c = psb->m_stretchConstraints[i];
			if (c.m_isUse == false) { continue; }

			btScalar prime = c.m_prime;
			Node* x[2];
			x[0] = c.m_n[0];
			x[1] = c.m_n[1];

			btVector3 dir = x[0]->m_x - x[1]->m_x;
			btScalar dirLength = dir.length();	if (dirLength <= SIMD_EPSILON) { continue; }
			btVector3 dirNorm = dir / dirLength;

			btVector3 g[2];
			g[0] = dirNorm;
			g[1] = -dirNorm;

			btScalar dLen = dirLength - c.m_rest;
			btScalar gSum = 2, imSum = c.m_im, s = 0, n = 2;		// gSum = g[0]^2 + g[1]^2 = 2

			if (imSum <= SIMD_EPSILON) { continue; }
			s = -1.0 / imSum * dLen * prime;			// (n/gSum/imSum*dLen*prime :: n/gSum = 2/2 = 1)
			for (int j = 0; j < 2; j++) {
				if (x[j]->m_im > 0.0) { x[j]->m_x += s * g[j] * x[j]->m_im; }
			}
		}
		break;
	}
	case cfgMeshType::Tetra:
	{
		for (int i = 0, ni = psb->m_stretchConstraints.size(); i < ni; i++) {
			Constraint& c = psb->m_stretchConstraints[i];
			if (c.m_isUse == false) { continue; }

			btScalar prime = c.m_prime;
			Node* p[2];
			p[0] = c.m_n[0];
			p[1] = c.m_n[1];

			btVector3 dir = p[0]->m_x - p[1]->m_x;
			btScalar dirLength = dir.length();	if (dirLength <= SIMD_EPSILON) { continue; }
			btVector3 dirNorm = dir / dirLength;

			btVector3 g[2];
			g[0] = dirNorm;
			g[1] = -dirNorm;

			btScalar dLen = dirLength - c.m_rest;
			btScalar gSum = 2, imSum = c.m_im, s = 0, n = 2;		// gSum = g[0]^2 + g[1]^2 = 2

			if (imSum <= SIMD_EPSILON) { continue; }
			s = -1.0 / imSum * dLen * prime;			// (n/gSum/imSum*dLen*prime :: n/gSum = 2/2 = 1)
			for (int j = 0; j < 2; j++) {
				if (p[j]->m_im > 0.0) { p[j]->m_x += s * g[j] * p[j]->m_im; }
			}
		}
		break;
	}

	}
}
void CiSoftBody::PSolveBending(CiSoftBody* psb)
{
	switch (psb->m_cfg.m_meshType) {
	case cfgMeshType::Mesh:
	{
		// dihedral bending constraint //
		for (int i = 0, ni = psb->m_bendingConstraints_dihedral.size(); i < ni; i++) {
			Constraint& c = psb->m_bendingConstraints_dihedral[i];
			if (c.m_isUse == false) { continue; }

			btScalar prime = c.m_prime;
			Node* p[4];
			for (int j = 0; j < 4; j++) {
				p[j] = c.m_n[j];
			}

			btVector3 p2p1 = p[1]->m_x - p[0]->m_x;
			btVector3 p3p1 = p[2]->m_x - p[0]->m_x;
			btVector3 p4p1 = p[3]->m_x - p[0]->m_x;
			btVector3 p2p3c = btCross(p2p1, p3p1);
			btVector3 p2p4c = btCross(p2p1, p4p1);

			btScalar p2p3cLen = p2p3c.length();		if (p2p3cLen <= SIMD_EPSILON) { continue; }
			btScalar p2p4cLen = p2p4c.length();		if (p2p4cLen <= SIMD_EPSILON) { continue; }

			btVector3 n1 = p2p3c / p2p3cLen;
			btVector3 n2 = p2p4c / p2p4cLen;

			// dAngle 계산 가능 각도(0-pi) 범위로 이동 // (범위 넘어가면 불안정)
			btScalar d, phi, dAngle, dAngleAbs;

			d = btDot(n1, n2);
			if (d > 1.0) { d = 1.0; }
			if (d < -1.0) { d = -1.0; }
			phi = btAcos(d);
			dAngle = phi - c.m_rest;
			dAngleAbs = fabs(dAngle);
			if (dAngleAbs <= SIMD_EPSILON) { continue; }
			if (dAngleAbs > PI) {
				if (dAngle > PI) {
					dAngle = -dAngle + PI;
				}
				else {
					dAngle = dAngle + PI;
				}
			}

			// gradient term
			btVector3 p2n1c = btCross(p2p1, n1);
			btVector3 p2n2c = btCross(p2p1, n2);
			btVector3 p3n1c = btCross(p3p1, n1);
			btVector3 p3n2c = btCross(p3p1, n2);
			btVector3 p4n1c = btCross(p4p1, n1);
			btVector3 p4n2c = btCross(p4p1, n2);

			btVector3 g[4];
			g[2] = (p2n2c - (p2n1c*d)) / p2p3cLen;
			g[3] = (p2n1c - (p2n2c*d)) / p2p4cLen;
			g[1] = -((p3n2c - (p3n1c*d)) / p2p3cLen) - ((p4n1c - (p4n2c*d)) / p2p4cLen);
			g[0] = -(g[1] + g[2] + g[3]);

			btScalar gSum = 0, imSum = c.m_im, s = 0, n = 4, f1sqrtD2 = 0;
			for (int j = 0; j < n; j++) {
				gSum += g[j].length2();
			}

			f1sqrtD2 = sqrt(1 - (d*d));

			if (gSum <= SIMD_EPSILON) { continue; }
			if (imSum <= SIMD_EPSILON) { continue; }
			if (f1sqrtD2 <= SIMD_EPSILON) { continue; }
			s = -n / imSum / gSum * f1sqrtD2 * dAngle * c.m_prime;

			// gradient 값 계산 (결과는 진동현상 개선 후 반영)
			btVector3 newP[4];
			btVector3 lambda[4];
			for (int j = 0; j < n; j++) {
				lambda[j] = p[j]->m_im * s * g[j];
				newP[j] = p[j]->m_x + lambda[j];
			}

			// 진동현상 개선 (dAngle 부호가 바뀔정도로 과한힘이 들어가는 경우)
			btScalar prime2 = 1;
			p2p1 = newP[1] - newP[0];
			p3p1 = newP[2] - newP[0];
			p4p1 = newP[3] - newP[0];
			p2p3c = btCross(p2p1, p3p1);
			p2p4c = btCross(p2p1, p4p1);
			p2p3cLen = p2p3c.length();
			p2p4cLen = p2p4c.length();

			n1 = p2p3c / p2p3cLen;
			n2 = p2p4c / p2p4cLen;

			btScalar d2, phi2, dAngle2;
			d2 = btDot(n1, n2);
			if (d2 > 1.0) { d2 = 1.0; }
			if (d2 < -1.0) { d2 = -1.0; }
			phi2 = btAcos(d2);
			dAngle2 = phi2 - c.m_rest;

			if (fabs(dAngle2) > SIMD_EPSILON) {
				if (dAngle > 0 && dAngle2 < 0) {
					prime2 *= 0.5;
				}
				else if (dAngle < 0 && dAngle2 > 0) {
					prime2 *= 0.5;
				}
			}

			// dihedral 결과 최종적으로 반영
			for (int j = 0; j < n; j++) {
				p[j]->m_x += lambda[j] * prime2;
			}
		}

		// triangle bending constraint //
		int nCount = psb->m_bendingConstraints_triangle.size();
		for (int i = 0; i < nCount; i++) {
			Constraint& c = psb->m_bendingConstraints_triangle[i];
			if (c.m_isUse == false) { continue; }

			btScalar prime = c.m_prime;
			Node* p[3];
			for (int j = 0; j < 3; j++) {
				p[j] = c.m_n[j];
			}

			btVector3 center = (p[0]->m_x + p[1]->m_x + p[2]->m_x) / 3.0f;
			btVector3 dirCenter = p[2]->m_x - center;
			btScalar dirLength = dirCenter.length();

			if (dirLength <= SIMD_EPSILON) { continue; }
			btScalar diff = 1.0f - (c.m_rest / dirLength);
			btVector3 dirForce = dirCenter * diff;

			if (c.m_im <= SIMD_EPSILON) { continue; }
			btVector3 dir = 2 * prime * dirForce / c.m_im;

			if (p[0]->m_im > 0.0) { p[0]->m_x += dir * (p[0]->m_im); }
			if (p[1]->m_im > 0.0) { p[1]->m_x += dir * (p[1]->m_im); }
			if (p[2]->m_im > 0.0) { p[2]->m_x += -dir * (2.0f * p[2]->m_im); }
		}
		break;
	}
	case cfgMeshType::Tetra:
	{
		for (int i = 0, ni = psb->m_bendingConstraints_triangle.size(); i < ni; i++) {
			Constraint& c = psb->m_bendingConstraints_triangle[i];
			if (c.m_isUse == false) { continue; }

			btScalar prime = c.m_prime;
			Node* p[3];
			for (int j = 0; j < 3; j++) {
				p[j] = c.m_n[j];
			}

			btScalar w = c.m_im;
			btScalar restLength = c.m_rest;

			btVector3 center = (p[0]->m_x + p[1]->m_x + p[2]->m_x) / 3.0;
			btVector3 dirCenter = p[2]->m_x - center;
			btScalar length = dirCenter.length();

			btScalar diff = 1.0f - (restLength / length);
			btVector3 dirForce = dirCenter * diff;
			btVector3 dir = 2.0 * prime * dirForce / w;

			p[0]->m_x += dir * (p[0]->m_im);
			p[1]->m_x += dir * (p[1]->m_im);
			p[2]->m_x += -dir * (2.0f * p[2]->m_im);
		}
		break;
	}
	}
}
void CiSoftBody::PSolveVolume(CiSoftBody* psb)
{
	switch (psb->m_cfg.m_meshType) {
	case cfgMeshType::Mesh:
	{
		int nNodes = psb->m_volumeConstraints_surface[0].m_nodeCnt;
		btScalar imSum = psb->m_volumeConstraints_surface[0].m_im;
		btScalar prime = psb->m_volumeConstraints_surface[0].m_prime;
		btScalar restVolume = psb->m_volumeConstraints_surface[0].m_rest;

		psb->updateNormals();
		psb->updateArea();
		btScalar dVolume = (psb->getVolume() - restVolume);

		for (int i = 0; i < nNodes; i++) {
			Node* x = &psb->m_nodes[i];
			if (x->m_im > 0) {
				x->m_f += x->m_n * x->m_area;
			}
		}

		btScalar gSum = 0, s = 0, n = nNodes;
		for (int i = 0; i < nNodes; i++) {
			Node* x = &psb->m_nodes[i];
			gSum += x->m_f.length2();
			imSum += x->m_im;
		}

		s = -n / gSum / imSum * dVolume * prime * 1.5;
		if (s > SIMD_EPSILON) {
			for (int i = 0; i < nNodes; i++) {
				Node* x = &psb->m_nodes[i];
				if (x->m_im > 0.0) { x->m_x += s * x->m_f * x->m_im; }
				x->m_f = btVector3(0, 0, 0);
			}
		}
		break;
	}
	case cfgMeshType::Tetra:
	{
		bool bGlobal = false;
		bool bLocal = true;

		// local //
		if (bLocal) {
			for (int i = 0, ni = psb->m_volumeConstraints.size(); i < ni; i++) {
				Constraint& c = psb->m_volumeConstraints[i];
				if (c.m_isUse == false) { continue; }

				btScalar prime = c.m_prime;
				Node* p[4];
				for (int j = 0; j < 4; j++) {
					p[j] = c.m_n[j];
				}

				btVector3 p2p1 = (p[1]->m_x - p[0]->m_x);
				btVector3 p3p1 = (p[2]->m_x - p[0]->m_x);
				btVector3 p4p1 = (p[3]->m_x - p[0]->m_x);

				btVector3 g[4];
				g[1] = btCross(p3p1, p4p1) / 6.0;
				g[2] = btCross(p4p1, p2p1) / 6.0;
				g[3] = btCross(p2p1, p3p1) / 6.0;
				g[0] = -(g[1] + g[2] + g[3]);

				btScalar dVolume = (VolumeOf(p[0]->m_x, p[1]->m_x, p[2]->m_x, p[3]->m_x) - c.m_rest);
				btScalar gSum = 0, imSum = 0, s = 0, n = 4;

				for (int j = 0; j < 4; j++) {
					gSum += g[j].length2();
					imSum += p[j]->m_im;
				}

				s = -n / gSum / imSum * dVolume * prime / 2.5;

				if (s > SIMD_EPSILON) {
					for (int j = 0; j < 4; j++) {
						p[j]->m_x += s * g[j] * p[j]->m_im;
					}
				}
			}
		}

		// global
		if (bGlobal) {
			btScalar prime = psb->m_materials[0]->m_kVST;

			const btVector3	zv(0, 0, 0);
			int i, ni;

			// update normal //
			for (i = 0, ni = psb->m_nodes.size(); i < ni; ++i)
			{
				psb->m_nodes[i].m_n = zv;
			}
			for (i = 0, ni = psb->m_tetrasSurface.size(); i < ni; ++i)
			{
				CiSoftBody::Face&	f = psb->m_tetrasSurface[i];
				const btVector3		n = btCross(f.m_n[1]->m_x - f.m_n[0]->m_x,
					f.m_n[2]->m_x - f.m_n[0]->m_x);
				f.m_normal = n.normalized();
				f.m_n[0]->m_n += n;
				f.m_n[1]->m_n += n;
				f.m_n[2]->m_n += n;
			}
			for (i = 0, ni = psb->m_nodes.size(); i < ni; ++i)
			{
				if (psb->m_nodes[i].m_surface == false) { continue; }
				btScalar len = psb->m_nodes[i].m_n.length();
				if (len > SIMD_EPSILON) {
					psb->m_nodes[i].m_n /= len;
				}
			}

			// update area //
			for (i = 0, ni = psb->m_tetrasSurface.size(); i < ni; ++i)
			{
				Face&		f = psb->m_tetrasSurface[i];
				f.m_ra = AreaOf(f.m_n[0]->m_x, f.m_n[1]->m_x, f.m_n[2]->m_x);
			}
			btAlignedObjectArray<int>	counts;
			counts.resize(psb->m_nodes.size(), 0);
			int index = 0;

			for (i = 0, ni = psb->m_nodes.size(); i < ni; ++i)
			{
				psb->m_nodes[i].m_area = 0;
			}
			for (i = 0, ni = psb->m_tetrasSurface.size(); i < ni; ++i)
			{
				CiSoftBody::Face&	f = psb->m_tetrasSurface[i];
				for (int j = 0; j < 3; ++j)
				{
					index = (int)(f.m_n[j] - &psb->m_nodes[0]);
					counts[index]++;
					f.m_n[j]->m_area += btFabs(f.m_ra);
				}
			}
			for (i = 0, ni = psb->m_nodes.size(); i < ni; ++i)
			{
				if (psb->m_nodes[i].m_surface == false) { continue; }
				if (counts[i] > 0)
					psb->m_nodes[i].m_area /= (btScalar)counts[i];
				else
					psb->m_nodes[i].m_area = 0;
			}

			btScalar dVolume = (psb->getVolume() - psb->m_pose.m_volume);
			for (int i = 0, ni = psb->m_nodes.size(); i < ni; i++) {
				if (psb->m_nodes[i].m_surface == false) { continue; }
				Node* x = &psb->m_nodes[i];

				if (x->m_im > 0) {
					x->m_f += x->m_n * x->m_area;
				}
			}

			btScalar gSum = 0, imSum = 0, s = 0, n = 0;
			for (int i = 0, ni = psb->m_nodes.size(); i < ni; i++) {
				if (psb->m_nodes[i].m_surface == false) { continue; }

				Node* x = &psb->m_nodes[i];
				gSum += x->m_f.length2();
				imSum += x->m_im;
				n++;
			}

			s = -n / gSum / imSum * dVolume * prime;
			if (s > SIMD_EPSILON) {
				for (int i = 0, ni = psb->m_nodes.size(); i < ni; i++) {
					if (psb->m_nodes[i].m_surface == false) { continue; }

					Node* x = &psb->m_nodes[i];
					if (x->m_im > 0.0) { x->m_x += s * x->m_f * x->m_im; }
					x->m_f = zv;
				}
			}
		}
		break;
	}
	}
}
void CiSoftBody::PSolveGroundCollision(CiSoftBody* psb)
{
	return;

	for(int i=0, ni=psb->m_nodes.size(); i< ni; i++) {
		Node& n = psb->m_nodes[i];
		if(n.m_x.y() < 0) {
			btScalar dy = btFabs(n.m_x.y() - n.m_q.y());
			btScalar newX=0, newY=0, newZ=0;

			btScalar xy = btFabs(n.m_x.y());
			btScalar qy = btFabs(n.m_q.y());
			btScalar xy_dy = xy/dy;
			btScalar qy_dy = qy/dy;

			newX = n.m_q.x()*(xy_dy) + n.m_x.x()*(qy_dy);
			newY = n.m_q.y()*(xy_dy) + n.m_x.y()*(qy_dy);
			newZ = n.m_q.z()*(xy_dy) + n.m_x.z()*(qy_dy);
			n.m_x = btVector3(newX, 0, newZ);
			
		}
	}
}
void CiSoftBody::PSolveSelfCollision(CiSoftBody* psb)
{

}
void CiSoftBody::PSolveToolCollision(CiSoftBody* psb)
{
	btScalar toolContactRange = 2;			// tool 충돌처리 판정범위	2.0
	btScalar toolRepulsiveStrength = 1;	// tool이 밀어내는 힘 (셀수록 변형이 크게 일어나지만, 불안정해짐) 0.5
	btScalar toolCorrectionRange = 0.1;		// tool이 밀어냈을 때 이전 frame과 비교해서 일정 거리(toolCorrectionRange)를 벗어나면 오변형으로 간주하고 변형 x
	float marginOrigin = 1.7;	// 1.5
	float shapeMargin = marginOrigin * toolContactRange * psb->m_cfg.m_contactMargin;

	int nToolIdx = -1;
	for (int i = 0, ni = psb->m_simulationSpace->rigidBodies.size(); i < ni; i++) {
		if (psb->m_simulationSpace->rigidBodies[i]->getType() == CiRigidBody::bodyType::TOOL) {
			nToolIdx = i;
			break;
		}
	}
	if (nToolIdx == -1) {
		return;
	}

	btVector3 toolCollision1 = psb->m_simulationSpace->rigidBodies[nToolIdx]->m_visFiducialPoint[0];
	btVector3 toolCollision2 = psb->m_simulationSpace->rigidBodies[nToolIdx]->m_visFiducialPoint[1];

	btVector3 toolDir = (toolCollision2 - toolCollision1);
	btScalar toolLen = toolDir.length();

	btScalar tc1_x = toolCollision1.x();
	btScalar tc1_y = toolCollision1.y();
	btScalar tc1_z = toolCollision1.z();
	btScalar tc2_x = toolCollision2.x();
	btScalar tc2_y = toolCollision2.y();
	btScalar tc2_z = toolCollision2.z();

	//printf("toolCollision1 : %f %f %f\n", tc1_x, tc1_y, tc1_z);
	//printf("toolCollision2 : %f %f %f\n", tc2_x, tc2_y, tc2_z);

	for (int i = 0, ni = psb->m_nodes.size(); i < ni; i++) {
		Node& node = psb->m_nodes[i];
		btVector3 nodeDir = (toolCollision1 - node.m_x);
		btVector3 toolNodeCross = btCross(toolDir, nodeDir);
		btScalar distance = toolNodeCross.length() / toolDir.length();
		btScalar margin = shapeMargin;

		if (distance < margin) {
			// tempP (node와 가장 인접한 선위의 점= 수선의 발)
			btScalar toolToTempPLen = sqrt(nodeDir.length2() - distance * distance);
			toolDir.normalize();

			// 원기둥의 양 밑면의 중심 (P, Q), 반지름 r, 임의의점 A라고 가정할 때,
			// (1) 삼각형 APQ의 넓이가 PQ * r * 0.5보다 작아야함
			// (2) APQ와 AQP의 각도가 90도보다 작아야함
			btScalar PA_length = distance;
			btScalar PQ_length = toolLen;
			btScalar deg_length = distance * toolLen;
			btVector3 PA = -nodeDir; // node.m_x - toolCollision1
			btVector3 PQ = toolDir;  // toolCollision2 - toolCollision1
			btScalar APQ_deg = acos(btDot(PA, PQ) / deg_length);
			btVector3 QA = node.m_x - toolCollision2;
			btVector3 QP = -toolDir; // toolCollsion1 - toolCollision2
			btScalar AQP_deg = acos(btDot(QA, QP) / deg_length);

			btScalar x, y, z;

			x = toolDir.x()*toolToTempPLen + toolCollision1.x();
			y = toolDir.y()*toolToTempPLen + toolCollision1.y();
			z = toolDir.z()*toolToTempPLen + toolCollision1.z();
			btVector3 tempP(x, y, z);

			if (deg_length < SIMD_EPSILON) {
				continue;
			}

			if (fabs(APQ_deg) <= 1.570796) {
				if (fabs(AQP_deg) <= 1.570796) {
					if (((tc1_x <= x && x <= tc2_x) || (tc2_x <= x && x <= tc1_x)) &&
						((tc1_y <= y && y <= tc2_y) || (tc2_y <= y && y <= tc1_y)) &&
						((tc1_z <= z && z <= tc2_z) || (tc2_z <= z && z <= tc1_z))) {
						// 위의 2조건을 거치고 나서도 가끔 이상하게 판정을 못할때가 있어서 아래 조건식 추가
						btVector3 tempPToNode = node.m_x - tempP;	// 기울기
						btScalar tempPToNodeLen = tempPToNode.length();

						if (tempPToNodeLen >= SIMD_EPSILON) {
							tempPToNode /= tempPToNodeLen;
							btVector3 newPos = tempPToNode * margin*0.5 + tempP;

							if ((node.m_x - node.m_q).length() > margin*toolCorrectionRange) {

							}
							else {
								btScalar lengthBefore = (node.m_x - tempP).length();

								if (lengthBefore > margin*0.5) {
									node.m_x = tempPToNode * lengthBefore + tempP;
									node.m_collision = true;
								}
								else {
									if (tempPToNodeLen < margin*0.25) {
										node.m_x = tempPToNode * margin*0.25 + tempP;
										node.m_collision = true;
									}
									else {
										node.m_x = newPos;
										node.m_collision = true;
									}
								}

							}
						}

					}
				}
			}
			else {
				btVector3 tempPCol1 = tempP - toolCollision1;
				btVector3 tempPCol2 = toolCollision2 - toolCollision1;
				btScalar tempPCol1Len = (tempPCol1).length();
				if (tempPCol1Len < margin * 0.5) {
					// 반지름
					btVector3 tempPToNode = node.m_x - tempP;	// 기울기
					btScalar tempPToNodeLen = tempPToNode.length();

					if (tempPToNodeLen >= SIMD_EPSILON) {
						tempPToNode /= tempPToNodeLen;
						btVector3 newPos = tempPToNode * margin*0.5 + toolCollision1;

						btScalar x = newPos.x();
						btScalar y = newPos.y();
						btScalar z = newPos.z();

						//if ((toolCollision1 - newPos).length() > margin*0.5) { continue; }

						btVector3 nodeDir = (toolCollision1 - newPos);
						btVector3 toolNodeCross = btCross(toolDir, nodeDir);
						btScalar distance = toolNodeCross.length() / toolDir.length();
						btScalar PA_length = distance;
						btScalar PQ_length = toolLen;
						btScalar deg_length = distance * toolLen;
						btVector3 PA = -nodeDir; // node.m_x - toolCollision1
						btVector3 PQ = toolDir;  // toolCollision2 - toolCollision1
						btScalar APQ_deg = acos(btDot(PA, PQ) / deg_length);
						btVector3 QA = node.m_x - toolCollision2;
						btVector3 QP = -toolDir; // toolCollsion1 - toolCollision2
						btScalar AQP_deg = acos(btDot(QA, QP) / deg_length);

						if (fabs(APQ_deg) <= 1.570796 && fabs(AQP_deg) <= 1.570796) {
							btVector3 tempPToNode = node.m_x - tempP;	// 기울기
							btScalar tempPToNodeLen = tempPToNode.length();

							if (tempPToNodeLen >= SIMD_EPSILON) {
								tempPToNode /= tempPToNodeLen;
								btVector3 newPos = tempPToNode * margin*0.5 + tempP;

								if ((node.m_x - node.m_q).length() > margin*toolCorrectionRange) {
									//node.m_x = tempPToNode * margin*toolCorrectionRange + tempP;
									node.m_collision = true;
								}
								else {
									btScalar lengthBefore = (node.m_x - tempP).length();
									if (lengthBefore > margin*0.5) {
										node.m_x = tempPToNode * lengthBefore + tempP;
										node.m_collision = true;
									}
									else {
										if (tempPToNodeLen < margin*0.25) {
											node.m_x = tempPToNode * margin*0.25 + tempP;
											node.m_collision = true;
										}
										else {
											node.m_x = newPos;
											node.m_collision = true;
										}
									}
								}
							}
						}
						else {
							if (tempPToNodeLen < margin*0.25) {
								node.m_x = tempPToNode * margin*0.25 + tempP;
								node.m_collision = true;
							}
							else {
								btScalar lengthBefore = (node.m_x - toolCollision1).length();
								if (lengthBefore > margin*0.5) {
									node.m_x = tempPToNode * lengthBefore + toolCollision1;
									node.m_collision = true;
								}
								else {
									node.m_x = newPos;
									node.m_collision = true;
								}
							}
						}
					}
				}
			}
		}
	}
}

void CiSoftBody::solveConstraints()
{
	/* Prepare links		*/
	int i, ni;

	/* Solve velocities		*/
	if (m_cfg.viterations > 0)
	{
		/* Solve			*/
		for (int isolve = 0; isolve < m_cfg.viterations; ++isolve)
		{
			for (int iseq = 0; iseq < m_cfg.m_vsequence.size(); ++iseq)
			{
				getSolver(m_cfg.m_vsequence[iseq])(this);
			}
		}
		/* Update			*/
		for (i = 0, ni = m_nodes.size(); i < ni; ++i)
		{
			Node&	n = m_nodes[i];
			n.m_x = n.m_q + n.m_v*m_sst.sdt;
		}
	}

	// position solver //
	if (m_cfg.piterations > 0) {
		// iteration //
		for (int iSolve = 0; iSolve < m_cfg.piterations; iSolve++) {
			for (int iSeq = 0; iSeq < m_cfg.m_psequence.size(); iSeq++) {
				getSolver(m_cfg.m_psequence[iSeq])(this);
			}
		}

		const btScalar	vc = m_sst.isdt*(1 - m_cfg.kDP);	// damping constraints
		for (int i = 0, ni = m_nodes.size(); i < ni; ++i)
		{
			Node&	n = m_nodes[i];
			n.m_v = (n.m_x - n.m_q)*vc;
			n.m_f = btVector3(0, 0, 0);
		}
	}

	PSolveToolCollision(this);

	updateNormals();
	updateSurfaceVertices();
}