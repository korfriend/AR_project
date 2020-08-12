#ifndef _SOFT_BODY_H
#define _SOFT_BODY_H

#include "btVector3.h"
#include "btMatrix3x3.h"
#include "btTransform.h"

#include "btAlignedAllocator.h"
#include "btAlignedObjectArray.h"
#include "btPolarDecomposition.h"

#include "rigidBody.h"

#include <string.h>

#define SAFE_DELETE_ARRAY(p)	{ if(p) delete[](p); p = NULL;}
#define SAFE_DELETE_VOLUME(p, depth)	{ if(p) {for (int i=0;i<depth;i++)	if(p[i]) delete[](p[i]); } delete[] p; p=NULL;};

#define PI 3.1415926536f
#define M_PI PI
#define EPSILON  0.0000001f
#define DRAW_INIT_PROCESS true
#define CONSTRAINT_ACCURACY false

class Simulation;	// forward declartions

class CiSoftBody
{
public:
	/// struct /////////////////////////////////////////////////////////////////////////////////////////
	struct Material
	{
		int						m_Idx;
		btScalar				m_kLST;			// Stretch stiffness coefficient [0,1]
		btScalar				m_kAST;			// Area/Angular stiffness coefficient [0,1]
		btScalar				m_kVST;			// Volume stiffness coefficient [0,1]
	};
	struct Node
	{
		int						m_idx;			// node idx
		btVector3				m_x;			// Position
		btVector3				m_q;			// Previous step position
		btVector3				m_x0;			// init Position
		btVector3				m_v;			// Velocity
		btVector3				m_f;			// Force accumulator
		btVector3				m_n;			// Normal
		btVector3				m_nCopy;		// Normal Copy
		btScalar				m_im;			// 1/mass
		btScalar				m_area;			// Area
		Material*				m_mat;			// homogeneous

		// for tetra
		bool					m_surface;		
		int						m_nAdjLinkCnt;
		bool					m_collision;

		btVector3				m_color;
		btScalar				m_colorAlpha;
	};
	struct Link
	{
		int						m_idx;			// link idx
		Node*					m_n[2];			// Node pointers
		btScalar				m_rl;			// Rest length
		Material*				m_mat;			// homogeneous
	};
	struct	Face
	{
		int						m_idx;			// face idx
		Node*					m_n[3];			// Node pointers
		btVector3				m_normal;		// Normal
		btScalar				m_ra;			// Rest area

		Material*				m_mat;			// homogeneous

		// bending constraint (mesh) (CiSoftBodyHelpers)
		btScalar				m_ln;			// Tetrahedron Bending Constraint
		btScalar				m_im;			// Tetrahedron Bending Constraint (p1,p2,p3)

		btVector3				m_color;
		btScalar				m_colorAlpha;
	};
	struct Tetra
	{
		int						m_idx;			// tetra idx
		Node*					m_n[4];			// Node pointers
		btScalar				m_v0;			// init volume
		btScalar				m_rv;			// Rest volume
		btScalar				m_a0;			// init angle
		Material*				m_mat;			// heterogeneous
		bool					m_hetero;		// heterogeneous

		btVector3				m_color;
		btScalar				m_colorAlpha;
	};


	///eSolverPresets
	struct	eSolverPresets { enum _ {
		Positions,
		Velocities,
		Default	=	Positions,
		END
	};};
	struct	ePSolver { enum _ {
		Stretch,			///Stretch solver
		Volume,
		Bending,
		SelfCollision,
		GroundCollision,
		ToolCollision,
		END
	};};
	///eVSolver : velocities solvers
	struct	eVSolver { enum _ {
		Stretch,		///Stretch solver
		Tetra,
		END
	};};
	typedef btAlignedObjectArray<ePSolver::_>	tPSolverArray;
	typedef btAlignedObjectArray<eVSolver::_>	tVSolverArray;



	///solverState
	struct SolverState
	{
		btScalar				sdt;			// dt*timescale
		btScalar				isdt;			// 1/sdt
	};
	struct	cfgMeshType { enum _ {
		Mesh,		
		Tetra
	};};
	struct Config
	{
		int						m_meshType;
		float					m_contactMargin;	
		btScalar				timescale;		// Time scale
		int						piterations;	// Positions solver iterations
		int						viterations;	// Velocities solver iterations
		tPSolverArray			m_psequence;	// Position solvers sequence
		tVSolverArray			m_vsequence;	// Velocity solvers sequence
		bool					m_draw;

		btScalar				kDP;
		btScalar				kMT;
	};
	struct Constraint
	{
		bool					m_isUse;
		Node**					m_n;			// node pointer
		int						m_nodeCnt;
		btScalar				m_rest;
		btScalar				m_im;
		btScalar				m_prime;
	};
	typedef btAlignedObjectArray<btScalar>	tScalarArray;
	typedef btAlignedObjectArray<btVector3>	tVector3Array;
	struct	Pose
	{
		bool					m_bvolume;		// Is valid
		bool					m_bframe;		// Is frame
		btScalar				m_volume;		// Rest volume
		tVector3Array			m_pos;			// Reference positions
		tScalarArray			m_wgh;			// Weights
		btVector3				m_com;			// COM
		btMatrix3x3				m_rot;			// Rotation
		btMatrix3x3				m_scl;			// Scale
		btMatrix3x3				m_aqq;			// Base scaling
	};

	typedef btAlignedObjectArray<Constraint>	tConstraintArray;
	typedef btAlignedObjectArray<Node>			tNodeArray;
	typedef btAlignedObjectArray<Link>			tLinkArray;
	typedef btAlignedObjectArray<Face>			tFaceArray;
	typedef btAlignedObjectArray<Tetra>			tTetraArray;
	typedef btAlignedObjectArray<Material*>		tMaterialArray;


	/// variable /////////////////////////////////////////////////////////////////////////////////////////
public:
	Simulation*				m_simulationSpace;
	Config					m_cfg;				// Configuration
	SolverState				m_sst;				// Solver state

	tNodeArray				m_nodes;			// Nodes
	tLinkArray				m_links;			// Links
	tFaceArray				m_faces;			// Faces

	tMaterialArray			m_materials;		// Materials
	btVector3				m_oriCenter;		// Origin Center (ÀÌµ¿ Àü)

	// child //
	CiSoftBody*				m_child;
	int						m_childCnt;

	// tetra surface //
	tTetraArray				m_tetras;			// Tetras
	tFaceArray				m_tetrasFaces;		// TetrasFaces
	tFaceArray				m_tetrasSurface;	// TetrasSurface

	// for coordinate mapping //
	tNodeArray									m_surfaceMeshNode;	//
	tFaceArray									m_surfaceMeshFace;
	btAlignedObjectArray<int>					m_attachedTetraIdx;
	btAlignedObjectArray<btVector4>				m_bary;
	btAlignedObjectArray<CiSoftBody::Tetra*>	m_boundaryTetras;
	btScalar									m_avgSurfaceMargin;

	// constraint //
	Pose					m_pose;							// Pose
	tConstraintArray		m_stretchConstraints;			// Stretch
	tConstraintArray		m_bendingConstraints_dihedral;	// Bending(Dihedral)
	tConstraintArray		m_bendingConstraints_triangle;	// Bending(Trianlge)
	tConstraintArray		m_volumeConstraints_surface;
	tConstraintArray		m_volumeConstraints;


	/// constructor /////////////////////////////////////////////////////////////////////////////////////////
	CiSoftBody();
	CiSoftBody(int iNodeCnt, const btVector3* x, const btScalar* m);
	~CiSoftBody();
	void initDefaults();
	void setSimulationSpace(Simulation* s);
	Simulation* getSimulationSpace();

	/// function ////////////////////////////////////////////////////////////////////////////////////////////
	void setNodeColor(btScalar r, btScalar g, btScalar b, btScalar a=1.0);
	void setFaceColor(btScalar r, btScalar g, btScalar b, btScalar a=1.0);
	void setTetraColor(btScalar r, btScalar g, btScalar b, btScalar a=1.0);
	
	bool checkLink(int node0, int node1);
	bool checkLink(const Node* node0,const Node* node1);
	bool checkFace(int node0,int node1, int node2);

	Material* appendMaterial();
	Material* appendMaterial(Material* m);
	void appendNode(const btVector3& x, btScalar m);
	void appendLink(int model=-1, Material* mat=0);
	void appendLink(int node0, int node1, Material* mat=0, bool bcheckexist=false);
	void appendLink(Node* node0, Node* node1, Material* mat=0, bool bcheckexist=false); 
	void appendFace(int model=-1,Material* mat=0);
	void appendFace(int node0, int node1, int node2, Material* mat=0);
	void appendFaceTet(int node0,int node1,int node2, int node3);
	void appendTetra(int model,Material* mat);
	void appendTetra(int node0,int node1,int node2,int node3,Material* mat=0);
	
	void addForce(const btVector3& force);
	void addForce(const btVector3& force, int node);
	void setForce(const btVector3& force);
	void addVelocity(const btVector3& velocity);
	void addVelocity(const btVector3& velocity, int node);
	void setVelocity(const btVector3& velocity);

	void setMass(int node, btScalar mass);
	void setTotalMass(btScalar mass, bool fromfaces=false);
	btScalar getMass(int node);
	btScalar getTotalMass();
	void setMeshType(int type);
	btScalar getVolume();
	void updateNormals();
	void updateArea(bool bAverageArea=true);
	void updateConstants();
	void updateSurfaceVertices();

	void initPose();
	void updatePose();
	btVector3 evaluateCom();

	void transform(const btTransform& trs);
	void translate(const btVector3& trs);
	void rotate(const btQuaternion& rot);
	void scale(const btVector3& scl);
	void moveToOrigin(bool yAxisOnly);
	void computeCenter();

	// solver ///////////////////////////////////////////////////////////////////////////////////
	void setSolver(eSolverPresets::_ preset);
	typedef void (*psolver_t)(CiSoftBody*);
	typedef void (*vsolver_t)(CiSoftBody*);
	static psolver_t getSolver(ePSolver::_ solver);
	static vsolver_t getSolver(eVSolver::_ solver);
	void initConstraints();
	void solveConstraints();
	void setConstraint(Constraint& c, Node** n, int nodeCnt, btScalar imSum, btScalar rest, btScalar prime);

	// constraint //////////////////////////////////////////////////////////////////////////////
	void initStretch();
	static void PSolveStretch(CiSoftBody* psb);
	void initBending();
	static void PSolveBending(CiSoftBody* psb);
	void initVolume();
	static void PSolveVolume(CiSoftBody* psb);
	void initEnergyPreserving();

	static void PSolveSelfCollision(CiSoftBody* psb);
	static void PSolveGroundCollision(CiSoftBody* psb);
	static void PSolveToolCollision(CiSoftBody* psb);
};



// math //////////////////////////////////////////////////////////////////////////////

template <typename T>
static inline void			ZeroInitialize(T& value)
{
	memset(&value, 0, sizeof(T));
}
//
template <typename T>
static inline bool			CompLess(const T& a, const T& b)
{
	return(a < b);
}
//
template <typename T>
static inline bool			CompGreater(const T& a, const T& b)
{
	return(a > b);
}
//
template <typename T>
static inline T				Lerp(const T& a, const T& b, btScalar t)
{
	return(a + (b - a)*t);
}
//
template <typename T>
static inline T				InvLerp(const T& a, const T& b, btScalar t)
{
	return((b + a * t - b * t) / (a*b));
}
//
static inline btMatrix3x3	Lerp(const btMatrix3x3& a,
	const btMatrix3x3& b,
	btScalar t)
{
	btMatrix3x3	r;
	r[0] = Lerp(a[0], b[0], t);
	r[1] = Lerp(a[1], b[1], t);
	r[2] = Lerp(a[2], b[2], t);
	return(r);
}
//
static inline btVector3		Clamp(const btVector3& v, btScalar maxlength)
{
	const btScalar sql = v.length2();
	if (sql > (maxlength*maxlength))
		return((v*maxlength) / btSqrt(sql));
	else
		return(v);
}
//
template <typename T>
static inline T				Clamp(const T& x, const T& l, const T& h)
{
	return(x<l ? l : x>h ? h : x);
}
//
template <typename T>
static inline T				Sq(const T& x)
{
	return(x*x);
}
//
template <typename T>
static inline T				Cube(const T& x)
{
	return(x*x*x);
}
//
template <typename T>
static inline T				Sign(const T& x)
{
	return((T)(x < 0 ? -1 : +1));
}
//
template <typename T>
static inline bool			SameSign(const T& x, const T& y)
{
	return((x*y) > 0);
}
//
static inline btScalar		ClusterMetric(const btVector3& x, const btVector3& y)
{
	const btVector3	d = x - y;
	return(btFabs(d[0]) + btFabs(d[1]) + btFabs(d[2]));
}
//
static inline btMatrix3x3	ScaleAlongAxis(const btVector3& a, btScalar s)
{
	const btScalar	xx = a.x()*a.x();
	const btScalar	yy = a.y()*a.y();
	const btScalar	zz = a.z()*a.z();
	const btScalar	xy = a.x()*a.y();
	const btScalar	yz = a.y()*a.z();
	const btScalar	zx = a.z()*a.x();
	btMatrix3x3		m;
	m[0] = btVector3(1 - xx + xx * s, xy*s - xy, zx*s - zx);
	m[1] = btVector3(xy*s - xy, 1 - yy + yy * s, yz*s - yz);
	m[2] = btVector3(zx*s - zx, yz*s - yz, 1 - zz + zz * s);
	return(m);
}
//
static inline btMatrix3x3	Cross(const btVector3& v)
{
	btMatrix3x3	m;
	m[0] = btVector3(0, -v.z(), +v.y());
	m[1] = btVector3(+v.z(), 0, -v.x());
	m[2] = btVector3(-v.y(), +v.x(), 0);
	return(m);
}
//
static inline btMatrix3x3	Diagonal(btScalar x)
{
	btMatrix3x3	m;
	m[0] = btVector3(x, 0, 0);
	m[1] = btVector3(0, x, 0);
	m[2] = btVector3(0, 0, x);
	return(m);
}
//
static inline btMatrix3x3	Add(const btMatrix3x3& a,
	const btMatrix3x3& b)
{
	btMatrix3x3	r;
	for (int i = 0; i < 3; ++i) r[i] = a[i] + b[i];
	return(r);
}
//
static inline btMatrix3x3	Sub(const btMatrix3x3& a,
	const btMatrix3x3& b)
{
	btMatrix3x3	r;
	for (int i = 0; i < 3; ++i) r[i] = a[i] - b[i];
	return(r);
}
//
static inline btMatrix3x3	Mul(const btMatrix3x3& a,
	btScalar b)
{
	btMatrix3x3	r;
	for (int i = 0; i < 3; ++i) r[i] = a[i] * b;
	return(r);
}
//
static inline void			Orthogonalize(btMatrix3x3& m)
{
	m[2] = btCross(m[0], m[1]).normalized();
	m[1] = btCross(m[2], m[0]).normalized();
	m[0] = btCross(m[1], m[2]).normalized();
}
//
static inline btMatrix3x3	MassMatrix(btScalar im, const btMatrix3x3& iwi, const btVector3& r)
{
	const btMatrix3x3	cr = Cross(r);
	return(Sub(Diagonal(im), cr*iwi*cr));
}

//
static inline btMatrix3x3	ImpulseMatrix(btScalar dt,
	btScalar ima,
	btScalar imb,
	const btMatrix3x3& iwi,
	const btVector3& r)
{
	return(Diagonal(1 / dt)*Add(Diagonal(ima), MassMatrix(imb, iwi, r)).inverse());
}

//
static inline btMatrix3x3	ImpulseMatrix(btScalar ima, const btMatrix3x3& iia, const btVector3& ra,
	btScalar imb, const btMatrix3x3& iib, const btVector3& rb)
{
	return(Add(MassMatrix(ima, iia, ra), MassMatrix(imb, iib, rb)).inverse());
}

//
static inline btMatrix3x3	AngularImpulseMatrix(const btMatrix3x3& iia,
	const btMatrix3x3& iib)
{
	return(Add(iia, iib).inverse());
}

//
static inline btVector3		ProjectOnAxis(const btVector3& v,
	const btVector3& a)
{
	return(a*btDot(v, a));
}
//
static inline btVector3		ProjectOnPlane(const btVector3& v,
	const btVector3& a)
{
	return(v - ProjectOnAxis(v, a));
}

//
static inline void			ProjectOrigin(const btVector3& a,
	const btVector3& b,
	btVector3& prj,
	btScalar& sqd)
{
	const btVector3	d = b - a;
	const btScalar	m2 = d.length2();
	if (m2 > SIMD_EPSILON)
	{
		const btScalar	t = Clamp<btScalar>(-btDot(a, d) / m2, 0, 1);
		const btVector3	p = a + d * t;
		const btScalar	l2 = p.length2();
		if (l2 < sqd)
		{
			prj = p;
			sqd = l2;
		}
	}
}
//
static inline void			ProjectOrigin(const btVector3& a,
	const btVector3& b,
	const btVector3& c,
	btVector3& prj,
	btScalar& sqd)
{
	const btVector3&	q = btCross(b - a, c - a);
	const btScalar		m2 = q.length2();
	if (m2 > SIMD_EPSILON)
	{
		const btVector3	n = q / btSqrt(m2);
		const btScalar	k = btDot(a, n);
		const btScalar	k2 = k * k;
		if (k2 < sqd)
		{
			const btVector3	p = n * k;
			if ((btDot(btCross(a - p, b - p), q) > 0) &&
				(btDot(btCross(b - p, c - p), q) > 0) &&
				(btDot(btCross(c - p, a - p), q) > 0))
			{
				prj = p;
				sqd = k2;
			}
			else
			{
				ProjectOrigin(a, b, prj, sqd);
				ProjectOrigin(b, c, prj, sqd);
				ProjectOrigin(c, a, prj, sqd);
			}
		}
	}
}

//
template <typename T>
static inline T				BaryEval(const T& a,
	const T& b,
	const T& c,
	const btVector3& coord)
{
	return(a*coord.x() + b * coord.y() + c * coord.z());
}
//
static inline btVector3		BaryCoord(const btVector3& a,
	const btVector3& b,
	const btVector3& c,
	const btVector3& p)
{
	const btScalar	w[] = { btCross(a - p,b - p).length(),
		btCross(b - p,c - p).length(),
		btCross(c - p,a - p).length() };
	const btScalar	isum = 1 / (w[0] + w[1] + w[2]);
	return(btVector3(w[1] * isum, w[2] * isum, w[0] * isum));
}


//
static inline btVector3		NormalizeAny(const btVector3& v)
{
	const btScalar l = v.length();
	if (l > SIMD_EPSILON)
		return(v / l);
	else
		return(btVector3(0, 0, 0));
}

//
static inline btVector3			CenterOf(const CiSoftBody::Face& f)
{
	return((f.m_n[0]->m_x + f.m_n[1]->m_x + f.m_n[2]->m_x) / 3);
}

//
static inline btScalar			AreaOf(const btVector3& x0,
	const btVector3& x1,
	const btVector3& x2)
{
	const btVector3	a = x1 - x0;
	const btVector3	b = x2 - x0;
	const btVector3	cr = btCross(a, b);
	const btScalar	area = cr.length();
	return(area / 2.0f);
}

//
static inline btScalar		VolumeOf(const btVector3& x0,
	const btVector3& x1,
	const btVector3& x2,
	const btVector3& x3)
{

	const btVector3	a = x1 - x0;
	const btVector3	b = x2 - x0;
	const btVector3	c = x3 - x0;

	return btDot(btCross(a, b), c) / 6.0f;
}

static inline btScalar VolumeOfDet(const btVector3& x0,
	const btVector3& x1,
	const btVector3& x2,
	const btVector3& x3)
{
	double m00 = x0.m_floats[0], m01 = x0.m_floats[1], m02 = x0.m_floats[2], m03 = 1;
	double m10 = x1.m_floats[0], m11 = x1.m_floats[1], m12 = x1.m_floats[2], m13 = 1;
	double m20 = x2.m_floats[0], m21 = x2.m_floats[1], m22 = x2.m_floats[2], m23 = 1;
	double m30 = x3.m_floats[0], m31 = x3.m_floats[1], m32 = x3.m_floats[2], m33 = 1;

	double value;
	value =
		m03 * m12 * m21 * m30 - m02 * m13 * m21 * m30 - m03 * m11 * m22 * m30 + m01 * m13 * m22 * m30 +
		m02 * m11 * m23 * m30 - m01 * m12 * m23 * m30 - m03 * m12 * m20 * m31 + m02 * m13 * m20 * m31 +
		m03 * m10 * m22 * m31 - m00 * m13 * m22 * m31 - m02 * m10 * m23 * m31 + m00 * m12 * m23 * m31 +
		m03 * m11 * m20 * m32 - m01 * m13 * m20 * m32 - m03 * m10 * m21 * m32 + m00 * m13 * m21 * m32 +
		m01 * m10 * m23 * m32 - m00 * m11 * m23 * m32 - m02 * m11 * m20 * m33 + m01 * m12 * m20 * m33 +
		m02 * m10 * m21 * m33 - m00 * m12 * m21 * m33 - m01 * m10 * m22 * m33 + m00 * m11 * m22 * m33;

	return value;
}

//
static inline void			ApplyClampedForce(CiSoftBody::Node& n,
	const btVector3& f,
	btScalar dt)
{
	const btScalar	dtim = dt * n.m_im;
	if ((f*dtim).length2() > n.m_v.length2())
	{/* Clamp	*/
		n.m_f -= ProjectOnAxis(n.m_v, f.normalized()) / dtim;
	}
	else
	{/* Apply	*/
		n.m_f += f;
	}
}

//
static inline int		MatchEdge(const CiSoftBody::Node* a,
	const CiSoftBody::Node* b,
	const CiSoftBody::Node* ma,
	const CiSoftBody::Node* mb)
{
	if ((a == ma) && (b == mb)) return(0);
	if ((a == mb) && (b == ma)) return(1);
	return(-1);
}

static inline int			PolarDecompose(const btMatrix3x3& m, btMatrix3x3& q, btMatrix3x3& s)
{
	static const btPolarDecomposition polar;
	return polar.decompose(m, q, s);
}

static inline const double Determinant4x4(const btVector4 & v0, const btVector4 & v1, const btVector4 & v2, const btVector4 & v3)
{
	/*
	double m00 = v0.m_floats[0], m01 = v0.m_floats[1], m02 = v0.m_floats[2], m03 = v0.m_floats[3];
	double m10 = v1.m_floats[0], m11 = v1.m_floats[1], m12 = v1.m_floats[2], m13 = v1.m_floats[3];
	double m20 = v2.m_floats[0], m21 = v2.m_floats[1], m22 = v2.m_floats[2], m23 = v2.m_floats[3];
	double m30 = v3.m_floats[0], m31 = v3.m_floats[1], m32 = v3.m_floats[2], m33 = v3.m_floats[3];

	double value;
	value =
		m03 * m12 * m21 * m30 - m02 * m13 * m21 * m30 - m03 * m11 * m22 * m30 + m01 * m13 * m22 * m30 +
		m02 * m11 * m23 * m30 - m01 * m12 * m23 * m30 - m03 * m12 * m20 * m31 + m02 * m13 * m20 * m31 +
		m03 * m10 * m22 * m31 - m00 * m13 * m22 * m31 - m02 * m10 * m23 * m31 + m00 * m12 * m23 * m31 +
		m03 * m11 * m20 * m32 - m01 * m13 * m20 * m32 - m03 * m10 * m21 * m32 + m00 * m13 * m21 * m32 +
		m01 * m10 * m23 * m32 - m00 * m11 * m23 * m32 - m02 * m11 * m20 * m33 + m01 * m12 * m20 * m33 +
		m02 * m10 * m21 * m33 - m00 * m12 * m21 * m33 - m01 * m10 * m22 * m33 + m00 * m11 * m22 * m33;

	return -value;
	*/

	double m00 = v0.m_floats[0], m01 = v0.m_floats[1], m02 = v0.m_floats[2], m03 = 1;
	double m10 = v1.m_floats[0], m11 = v1.m_floats[1], m12 = v1.m_floats[2], m13 = 1;
	double m20 = v2.m_floats[0], m21 = v2.m_floats[1], m22 = v2.m_floats[2], m23 = 1;
	double m30 = v3.m_floats[0], m31 = v3.m_floats[1], m32 = v3.m_floats[2], m33 = 1;

	double value;
	value =
		m03 * m12 * m21 * m30 - m02 * m13 * m21 * m30 - m03 * m11 * m22 * m30 + m01 * m13 * m22 * m30 +
		m02 * m11 * m23 * m30 - m01 * m12 * m23 * m30 - m03 * m12 * m20 * m31 + m02 * m13 * m20 * m31 +
		m03 * m10 * m22 * m31 - m00 * m13 * m22 * m31 - m02 * m10 * m23 * m31 + m00 * m12 * m23 * m31 +
		m03 * m11 * m20 * m32 - m01 * m13 * m20 * m32 - m03 * m10 * m21 * m32 + m00 * m13 * m21 * m32 +
		m01 * m10 * m23 * m32 - m00 * m11 * m23 * m32 - m02 * m11 * m20 * m33 + m01 * m12 * m20 * m33 +
		m02 * m10 * m21 * m33 - m00 * m12 * m21 * m33 - m01 * m10 * m22 * m33 + m00 * m11 * m22 * m33;

	return value;
}

static inline const btVector4 GetBarycentricCoordinate(const btVector3 & v0_, const btVector3 & v1_, const btVector3 & v2_, const btVector3 & v3_, const btVector3 & p0_)
{
	btVector4 v0(v0_, 1);
	btVector4 v1(v1_, 1);
	btVector4 v2(v2_, 1);
	btVector4 v3(v3_, 1);
	btVector4 p0(p0_, 1);
	btVector4 barycentricCoord = btVector4();

	const double det0 = Determinant4x4(v0, v1, v2, v3);
	const double det1 = Determinant4x4(p0, v1, v2, v3);
	const double det2 = Determinant4x4(v0, p0, v2, v3);
	const double det3 = Determinant4x4(v0, v1, p0, v3);
	const double det4 = Determinant4x4(v0, v1, v2, p0);

	barycentricCoord.setX((det1 / det0));
	barycentricCoord.setY((det2 / det0));
	barycentricCoord.setZ((det3 / det0));
	barycentricCoord.setW((det4 / det0));

	return barycentricCoord;
}

static inline bool pointInTetrahedron(const btVector3 & v1, const btVector3 & v2, const btVector3 & v3, const btVector3 & v4, const btVector3 & p)
{
	btVector3 normal = btCross(v2 - v1, v3 - v1);
	btScalar dot4 = btDot(normal, v4 - v1);
	btScalar dotP = btDot(normal, p - v1);
	return Sign(dot4) == Sign(dotP);
}

#endif