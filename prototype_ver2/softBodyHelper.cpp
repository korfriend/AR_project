#include "softBodyHelper.h"

CiRigidBody* CiSoftBodyHelpers::CreateFromTriMeshFile(
	const char* objPath,
	bool bFlipYZAxis)
{
	char relativeobjPath[1024];
	if (b3ResourcePath::findResourcePath(objPath, relativeobjPath, 1024)) {
		char pathPrefix[1024];
		b3FileUtils::extractPath(relativeobjPath, pathPrefix, 1024);
	}
	std::vector<tinyobj::shape_t> shapes;
	std::string err = tinyobj::LoadObj(shapes, relativeobjPath, "");

	btAlignedObjectArray<btScalar> vertices;
	btAlignedObjectArray<int> indices;

	//loop through all the shapes and add vertices and indices
	int offset = 0;
	for (int i = 0; i < (int)shapes.size(); ++i) {
		const tinyobj::shape_t& shape = shapes[i];

		//add vertices
		for (int j = 0; j < shape.mesh.positions.size(); ++j) {
			vertices.push_back(shape.mesh.positions[j]);
		}

		//add indices
		for (int j = 0; j < shape.mesh.indices.size(); ++j) {
			indices.push_back(offset + shape.mesh.indices[j]);
		}
		offset += shape.mesh.positions.size();
	}

	// copy
	CiRigidBody* rigidBody = new CiRigidBody();

	for (int i = 0, ni = vertices.size() / 3; i < ni; i++) {
		if (bFlipYZAxis) {
			rigidBody->m_vertices.push_back(btVector3(vertices[3 * i + 0], vertices[3 * i + 2], vertices[3 * i + 1]));
		}
		else {
			rigidBody->m_vertices.push_back(btVector3(vertices[3 * i + 0], vertices[3 * i + 1], vertices[3 * i + 2]));
		}
	}
	for (int i = 0, ni = indices.size(); i < ni; i++) {
		rigidBody->m_indices.push_back(indices[i]);
	}

	rigidBody->initVisVertices();

	return rigidBody;
}


CiSoftBody * CiSoftBodyHelpers::CreateFromTetGenFile(
	const char* ele,
	const char* face,
	const char* node,
	const char* link,
	bool bfacelinks,
	bool btetralinks,
	bool bfacesfromtetras,
	float fMass,
	CiSoftBody::Material* m,
	int nStartIndexNum,
	bool bFlipYZAxis,
	float fScale,
	bool alignObjectCenter,
	btVector3& center,
	btVector3& trans)
{
	printf("== Tetrahedron Load ==\n");
	btAlignedObjectArray<btVector3>	pos;
	int								nnode = 0;
	int								ndims = 0;
	int								nattrb = 0;
	int								hasbounds = 0;

	FILE* nodefp = NULL;
	fopen_s(&nodefp, node, "r");
	if (nodefp == NULL)
	{
		printf("%s", ele);
		printf("%s", node);
		system("pause");
		exit(1);
	}

	char pcBuf[1024] = { 0 };

	fgets(pcBuf, sizeof(pcBuf), nodefp);
	while (pcBuf[0] == '#') {
		fgets(pcBuf, sizeof(pcBuf), nodefp);
	}

	int result = sscanf_s(pcBuf, "%d %d %d %d", &nnode, &ndims, &nattrb, &hasbounds);

	pos.resize(nnode);

	fgets(pcBuf, sizeof(pcBuf), nodefp);
	while (pcBuf[0] == '#') {
		fgets(pcBuf, sizeof(pcBuf), nodefp);
	}
	for (int i = 0; i < nnode; ++i) {
		int index = 0;
		float	x, y, z;

		sscanf_s(pcBuf, "%d %f %f %f", &index, &x, &y, &z);

		if (bFlipYZAxis) {
			pos[index - nStartIndexNum].setX(btScalar((x + trans.x()) *-1));
			pos[index - nStartIndexNum].setY(btScalar(z + trans.z()));
			pos[index - nStartIndexNum].setZ(btScalar(y + trans.y()));
		}
		else {
			pos[index - nStartIndexNum].setX(btScalar(x + trans.x()));
			pos[index - nStartIndexNum].setY(btScalar(y + trans.y()));
			pos[index - nStartIndexNum].setZ(btScalar(z + trans.z()));
		}
		pos[index - nStartIndexNum].setW(0);

		//pos.push_back(btVector3(x, y, z));

		fgets(pcBuf, sizeof(pcBuf), nodefp);
	}
	fclose(nodefp);


	// scaling //
	if (alignObjectCenter) {
		btVector3 center(0, 0, 0);
		for (int i = 0; i < nnode; i++) {
			center += pos[i];
		}
		center /= nnode;
		for (int i = 0; i < nnode; i++) {
			pos[i] += -center;
		}
		for (int i = 0; i < nnode; i++) {
			pos[i] *= fScale;
		}
		for (int i = 0; i < nnode; i++) {
			pos[i] += center;
		}
	}
	else {
		for (int i = 0; i < nnode; i++) {
			pos[i] += -center;
		}
		for (int i = 0; i < nnode; i++) {
			pos[i] *= fScale;
		}
		for (int i = 0; i < nnode; i++) {
			pos[i] += center;
		}
	}


	// 정점좌표 중복검사 //
	btAlignedObjectArray<int>	posIdx;
	btAlignedObjectArray<btVector3> newPos;

	posIdx.resize(nnode);
	int nDupCnt = 0;
	for (int i = 0; i < nnode; i++) {
		bool isDup = false;

		for (int j = 0, nj = newPos.size(); j < nj; j++) {
			if (pos[i] == newPos[j]) {
				isDup = true;
				posIdx[i] = j;
				break;
			}
		}

		if (!isDup) {
			newPos.push_back(pos[i]);
			posIdx[i] = newPos.size() - 1;
		}
	}

	nnode = newPos.size();

	//CiSoftBody* psb = new CiSoftBody(&worldInfo, nnode, &newPos[0], 0);
	CiSoftBody* psb = new CiSoftBody(nnode, &newPos[0], 0);

	// properties
	psb->setTotalMass(fMass, false);
	CiSoftBody::Material* pm = psb->m_materials[0];
	memcpy(pm, m, sizeof(CiSoftBody::Material));
	btAlignedFree(m);

	//////////////////////////////////////////////////////////////////////////////////////////
	// file load //
	FILE* elefp = NULL;
	fopen_s(&elefp, ele, "r");
	if (elefp == NULL)
	{
		system("pause");
		exit(1);
	}

	int	ntetra = 0;
	int	ncorner = 0;
	int neattrb = 0;

	fgets(pcBuf, sizeof(pcBuf), elefp);
	while (pcBuf[0] == '#') {
		fgets(pcBuf, sizeof(pcBuf), elefp);
	}
	sscanf_s(pcBuf, "%d %d %d", &ntetra, &ncorner, &neattrb);
	fgets(pcBuf, sizeof(pcBuf), elefp);
	while (pcBuf[0] == '#') {
		fgets(pcBuf, sizeof(pcBuf), elefp);
	}

	printf("== 중복 Link 검사 ==\n");
	//////////// 중복 link 검사 ////////////////////////////////
	int nodeCnt = psb->m_nodes.size();

	FILE* linkfp = NULL;
	fopen_s(&linkfp, link, "r");
	if (linkfp != NULL)
	{
		printf("----link file load\n");
		// tetra load //
		for (int i = 0; i < ntetra; ++i) {
			int	index = 0;
			int	ni[4];

			sscanf_s(pcBuf, "%d %d %d %d %d", &index, &ni[0], &ni[1], &ni[2], &ni[3]);
			fgets(pcBuf, sizeof(pcBuf), elefp);

			// 중복 배제
			//printf("[%d] 0:%d 1:%d 2:%d 3:%d -> 0:%d 1:%d 2:%d 3:%d\n", i, ni[0], ni[1], ni[2], ni[3], posIdx[ni[0]-nStartIndexNum], posIdx[ni[1] - nStartIndexNum], posIdx[ni[2] - nStartIndexNum], posIdx[ni[3] - nStartIndexNum]);
			ni[0] = posIdx[ni[0] - nStartIndexNum];
			ni[1] = posIdx[ni[1] - nStartIndexNum];
			ni[2] = posIdx[ni[2] - nStartIndexNum];
			ni[3] = posIdx[ni[3] - nStartIndexNum];

			psb->appendTetra(ni[0], ni[1], ni[2], ni[3]);
			psb->appendFaceTet(ni[0], ni[1], ni[2], ni[3]);
		}

		int nLinkSize = 0, nLinkIdx = 0, nLinkNode1 = 0, nLinkNode2 = 0;
		fgets(pcBuf, sizeof(pcBuf), linkfp);
		sscanf_s(pcBuf, "%d", &nLinkSize);


		psb->m_links.resize(nLinkSize);

		for (int i = 0; i < nLinkSize; i++) {
			fgets(pcBuf, sizeof(pcBuf), linkfp);
			sscanf_s(pcBuf, "%d %d %d", &nLinkIdx, &nLinkNode1, &nLinkNode2);

			//psb->appendLink(nLinkNode1, nLinkNode2, pm, true);
			ZeroInitialize(psb->m_links[i]);
			psb->m_links[i].m_n[0] = &psb->m_nodes[nLinkNode1];
			psb->m_links[i].m_n[1] = &psb->m_nodes[nLinkNode2];
			psb->m_links[i].m_rl = -1;
			psb->m_links[i].m_idx = i;
		}
		fclose(linkfp);

	}
	else {
#define IDX(_x_,_y_) ((_y_)*nodeCnt + (_x_))	
		bool* chks = new bool[nodeCnt * nodeCnt];
		for (int i = 0; i < nodeCnt*nodeCnt; i++) {
			chks[i] = false;
		}

		for (int i = 0; i < ntetra; ++i) {
			int	index = 0;
			int	ni[4];

			sscanf_s(pcBuf, "%d %d %d %d %d", &index, &ni[0], &ni[1], &ni[2], &ni[3]);
			fgets(pcBuf, sizeof(pcBuf), elefp);

			// 중복 배제
			//printf("[%d] 0:%d 1:%d 2:%d 3:%d -> 0:%d 1:%d 2:%d 3:%d\n", i, ni[0], ni[1], ni[2], ni[3], posIdx[ni[0]-nStartIndexNum], posIdx[ni[1] - nStartIndexNum], posIdx[ni[2] - nStartIndexNum], posIdx[ni[3] - nStartIndexNum]);
			ni[0] = posIdx[ni[0] - nStartIndexNum];
			ni[1] = posIdx[ni[1] - nStartIndexNum];
			ni[2] = posIdx[ni[2] - nStartIndexNum];
			ni[3] = posIdx[ni[3] - nStartIndexNum];

			psb->appendTetra(ni[0], ni[1], ni[2], ni[3]);
			if (btetralinks) {
				int idx1[6] = { 0, 1, 2, 0, 1, 2 };
				int idx2[6] = { 1, 2, 0, 3, 3, 3 };

				for (int j = 0; j < 6; j++) {
					int n[2] = { 0 };

					n[0] = ni[idx1[j]];
					n[1] = ni[idx2[j]];
					//printf("n[0]:%d n[1]:%d\n", n[0], n[1]);

					if (chks[IDX(n[0], n[1])] == false) {
						chks[IDX(n[0], n[1])] = true;
						chks[IDX(n[1], n[0])] = true;
						psb->appendLink(n[0], n[1], pm, true);
					}
				}
			}

			// Tetra 면 생성 //
			psb->appendFaceTet(ni[0], ni[1], ni[2], ni[3]);
		}
		delete[] chks;
#undef IDX

		fopen_s(&linkfp, link, "w");
		fprintf(linkfp, "%d\n", psb->m_links.size());

		for (int i = 0, ni = psb->m_links.size(); i < ni; i++) {
			fprintf(linkfp, "%d %d %d\n", i + 1, psb->m_links[i].m_n[0]->m_idx, psb->m_links[i].m_n[1]->m_idx);
		}
		fclose(linkfp);
	}

	fclose(elefp);


	printf("== Tetrahedron 모델 외곽 Face 추출 ==\n");
	printf("----memory load\n");

	FILE* facefp = NULL;
	fopen_s(&facefp, face, "r");
	if (facefp != NULL)
	{
		printf("----surface file load\n");

		int nTetraSurfaceSize = 0;
		fgets(pcBuf, sizeof(pcBuf), facefp);
		sscanf_s(pcBuf, "%d", &nTetraSurfaceSize);

		int nTetraSurfaceIdx = 0, nTetraSurfaceNode1 = 0, nTetraSurfaceNode2 = 0, nTetraSurfaceNode3 = 0;
		for (int i = 0; i < nTetraSurfaceSize; i++) {
			fgets(pcBuf, sizeof(pcBuf), facefp);
			sscanf_s(pcBuf, "%d %d %d %d", &nTetraSurfaceIdx, &nTetraSurfaceNode1, &nTetraSurfaceNode2, &nTetraSurfaceNode3);

			psb->m_nodes[nTetraSurfaceNode1].m_surface = true;
			psb->m_nodes[nTetraSurfaceNode2].m_surface = true;
			psb->m_nodes[nTetraSurfaceNode3].m_surface = true;

			CiSoftBody::Face f;
			ZeroInitialize(f);
			f.m_n[0] = &psb->m_nodes[nTetraSurfaceNode1];
			f.m_n[1] = &psb->m_nodes[nTetraSurfaceNode2];
			f.m_n[2] = &psb->m_nodes[nTetraSurfaceNode3];
			psb->m_tetrasSurface.push_back(f);
		}

		fclose(facefp);
	}
	else {
		int nTetraSize = psb->m_tetras.size();
		int nTetraFaceSize = psb->m_tetrasFaces.size();
		int* tetraFaceIdx = new int[nTetraFaceSize];
		int* tetraFaceNodeList0 = new int[nTetraFaceSize];
		int* tetraFaceNodeList1 = new int[nTetraFaceSize];
		int* tetraFaceNodeList2 = new int[nTetraFaceSize];
		int* tetraFaceNodeNotSort0 = new int[nTetraFaceSize];
		int* tetraFaceNodeNotSort1 = new int[nTetraFaceSize];
		int* tetraFaceNodeNotSort2 = new int[nTetraFaceSize];
		int nFaceNodeListIdx = 0;

		for (int i = 0; i < nTetraFaceSize; i++) {
			//tetraFaceIdx[i] = i;
			tetraFaceIdx[i] = -1;
		}

		int nodeIdx[3];

		printf("----중복 Face 통합\n");
		for (int i = 0; i < nTetraFaceSize; i++) {
			nodeIdx[0] = psb->m_tetrasFaces[i].m_n[0]->m_idx;
			nodeIdx[1] = psb->m_tetrasFaces[i].m_n[1]->m_idx;
			nodeIdx[2] = psb->m_tetrasFaces[i].m_n[2]->m_idx;
			std::sort(nodeIdx, nodeIdx + 3);

			bool dup = false;
			for (int j = 0; j < nFaceNodeListIdx; j++) {			// !!!!check
				if ((tetraFaceNodeList0[j] == nodeIdx[0]) && (tetraFaceNodeList1[j] == nodeIdx[1]) && (tetraFaceNodeList2[j] == nodeIdx[2]))
				{
					dup = true;
					tetraFaceIdx[i] = j;
					break;
				}
			}

			if (dup == false) {
				tetraFaceNodeList0[nFaceNodeListIdx] = nodeIdx[0];
				tetraFaceNodeList1[nFaceNodeListIdx] = nodeIdx[1];
				tetraFaceNodeList2[nFaceNodeListIdx] = nodeIdx[2];
				tetraFaceNodeNotSort0[nFaceNodeListIdx] = psb->m_tetrasFaces[i].m_n[0]->m_idx;
				tetraFaceNodeNotSort1[nFaceNodeListIdx] = psb->m_tetrasFaces[i].m_n[1]->m_idx;
				tetraFaceNodeNotSort2[nFaceNodeListIdx] = psb->m_tetrasFaces[i].m_n[2]->m_idx;
				tetraFaceIdx[i] = nFaceNodeListIdx;

				nFaceNodeListIdx++;
			}
		}
		printf("----Face 중복횟수 Count\n");
		int nSize = nFaceNodeListIdx;
		int* piFaceCnt = new int[nSize];
		memset(piFaceCnt, 0, sizeof(int)*nSize);
		for (int i = 0, ni = psb->m_tetrasFaces.size(); i < ni; i++) {
			piFaceCnt[tetraFaceIdx[i]]++;
		}
		printf("----surface node\n");
		for (int i = 0; i < nSize; i++) {
			if (piFaceCnt[i] == 1) {
				psb->m_nodes[tetraFaceNodeNotSort0[i]].m_surface = true;
				psb->m_nodes[tetraFaceNodeNotSort1[i]].m_surface = true;
				psb->m_nodes[tetraFaceNodeNotSort2[i]].m_surface = true;

				CiSoftBody::Face f;
				ZeroInitialize(f);
				f.m_n[0] = &psb->m_nodes[tetraFaceNodeNotSort0[i]];
				f.m_n[1] = &psb->m_nodes[tetraFaceNodeNotSort1[i]];
				f.m_n[2] = &psb->m_nodes[tetraFaceNodeNotSort2[i]];
				psb->m_tetrasSurface.push_back(f);
			}
		}

		// surface file 생성
		fopen_s(&facefp, face, "w");
		fprintf(facefp, "%d\n", psb->m_tetrasSurface.size());
		int nFaceIdx = 1;
		for (int i = 0; i < nSize; i++) {
			if (piFaceCnt[i] == 1) {
				fprintf(facefp, "%d %d %d %d\n", nFaceIdx, tetraFaceNodeNotSort0[i], tetraFaceNodeNotSort1[i], tetraFaceNodeNotSort2[i]);
				nFaceIdx++;
			}
		}
		fclose(facefp);


		printf("delete memory\n");
		delete[] piFaceCnt;
		piFaceCnt = NULL;

		delete[] tetraFaceIdx;
		delete[] tetraFaceNodeList0;
		delete[] tetraFaceNodeList1;
		delete[] tetraFaceNodeList2;
		delete[] tetraFaceNodeNotSort0;
		delete[] tetraFaceNodeNotSort1;
		delete[] tetraFaceNodeNotSort2;
	}


	int nSurfaceNodeCnt = 0;
	for (int i = 0; i < psb->m_nodes.size(); i++) {
		if (psb->m_nodes[i].m_surface) {
			nSurfaceNodeCnt++;
		}
	}
	printf("(surface node)%d / (surface face)%d \n", nSurfaceNodeCnt, psb->m_tetrasSurface.size());

	int nNodes = psb->m_nodes.size();
	int nLinks = psb->m_links.size();
	for (int i = 0; i < nNodes; i++) {
		psb->m_nodes[i].m_nAdjLinkCnt = 0;
	}

	for (int i = 0; i < nLinks; i++) {
		psb->m_links[i].m_n[0]->m_nAdjLinkCnt++;
		psb->m_links[i].m_n[1]->m_nAdjLinkCnt++;
	}

	int nCount = 0;
	for (int i = 0; i < nNodes; i++) {
		if (psb->m_nodes[i].m_surface == true)
		{
			nCount++;
		}
	}

	std::printf("Nodes:  %u\r\n", psb->m_nodes.size());
	std::printf("Links:  %u\r\n", psb->m_links.size());
	std::printf("Faces:  %u\r\n", psb->m_faces.size());
	std::printf("Tetras: %u\r\n", psb->m_tetras.size());

	//////////////////////////////////////////////////////////////////////////////////////////
		// material
	for (int i = 0, ni = psb->m_nodes.size(); i < ni; i++) {
		CiSoftBody::Node* n = &psb->m_nodes[i];
		if (n->m_mat == NULL) {
			n->m_mat = pm;
		}
	}
	for (int i = 0, ni = psb->m_links.size(); i < ni; i++) {
		CiSoftBody::Link* l = &psb->m_links[i];
		if (l->m_mat == NULL) {
			l->m_mat = pm;
		}
	}
	for (int i = 0, ni = psb->m_tetras.size(); i < ni; i++) {
		CiSoftBody::Tetra* t = &psb->m_tetras[i];
		if (t->m_mat == NULL) {
			t->m_mat = pm;
		}
	}

	psb->setTotalMass(fMass, false);
	psb->setMeshType(CiSoftBody::cfgMeshType::Tetra);

	posIdx.clear();
	newPos.clear();

	return(psb);
}


CiSoftBody* CiSoftBodyHelpers::generateHybridModel(CiSoftBody* psbTetra, const char* meshFile, bool bFlipYZAxis, float fScale, bool bAlignObjectCenter, btVector3& center, btVector3& trans)
{
	printf("== generateHybridModel ==\n");
	// loadMesh ////////////////////////////////////////////////////////////////////////////////
	char relativeobjPath[1024];
	if (b3ResourcePath::findResourcePath(meshFile, relativeobjPath, 1024)) {
		char pathPrefix[1024];
		b3FileUtils::extractPath(relativeobjPath, pathPrefix, 1024);
	}
	std::vector<tinyobj::shape_t> shapes;
	std::string err = tinyobj::LoadObj(shapes, relativeobjPath, "");

	btAlignedObjectArray<btScalar> vertices;
	btAlignedObjectArray<int> indices;

	//loop through all the shapes and add vertices and indices
	int offset = 0;
	for (int i = 0; i < (int)shapes.size(); ++i) {
		const tinyobj::shape_t& shape = shapes[i];

		//add vertices
		for (int j = 0; j < shape.mesh.positions.size(); ++j) {
			vertices.push_back(shape.mesh.positions[j]);
		}

		//add indices
		for (int j = 0; j < shape.mesh.indices.size(); ++j) {
			indices.push_back(offset + shape.mesh.indices[j]);
		}
		offset += shape.mesh.positions.size();
	}

	if (vertices.size() == 0) {
		printf("file load 실패\n");
		system("pause");
		exit(0);
	}

	// input mesh points and faces //////////////////////////////////////////////////////////////
	int nMeshNodeCnt = vertices.size() / 3.0;
	int nMeshFaceCnt = indices.size() / 3.0;
	psbTetra->m_surfaceMeshNode.resize(nMeshNodeCnt);
	psbTetra->m_surfaceMeshFace.resize(nMeshFaceCnt);
	psbTetra->m_attachedTetraIdx.resize(nMeshNodeCnt, -1);
	psbTetra->m_bary.resize(nMeshNodeCnt);


	for (int i = 0; i < nMeshNodeCnt; i++) {
		CiSoftBody::Node n;
		memset(&n, 0, sizeof(CiSoftBody::Node));

		btScalar x = vertices[3 * i + 0];
		btScalar y = vertices[3 * i + 1];
		btScalar z = vertices[3 * i + 2];


		if (bFlipYZAxis) {
			n.m_x.setValue(-x, z, y);
			n.m_x0.setValue(-x, z, y);
		}
		else {
			n.m_x.setValue(x, y, z);
			n.m_x0.setValue(x, y, z);
		}

		n.m_idx = i;
		psbTetra->m_surfaceMeshNode[i] = n;
	}

	for (int i = 0; i < nMeshFaceCnt; i++) {
		CiSoftBody::Face f;
		memset(&f, 0, sizeof(CiSoftBody::Face));

		for (int j = 0; j < 3; j++) {
			int idx = indices[3 * i + j];
			f.m_n[j] = &psbTetra->m_surfaceMeshNode[idx];
		}
		psbTetra->m_surfaceMeshFace[i] = f;
	}

	// scaling //
	if (bAlignObjectCenter) {
		for (int i = 0; i < nMeshNodeCnt; i++) {
			psbTetra->m_surfaceMeshNode[i].m_x += -center;
		}
		for (int i = 0; i < nMeshNodeCnt; i++) {
			psbTetra->m_surfaceMeshNode[i].m_x *= fScale;
		}
		for (int i = 0; i < nMeshNodeCnt; i++) {
			psbTetra->m_surfaceMeshNode[i].m_x += center;
		}

		btVector3 meshCenter(0, 0, 0);

		for (int i = 0; i < nMeshNodeCnt; i++) {
			meshCenter += psbTetra->m_surfaceMeshNode[i].m_x;
		}
		meshCenter /= nMeshNodeCnt;
		btVector3 diff = center - meshCenter;

		for (int i = 0; i < nMeshNodeCnt; i++) {
			psbTetra->m_surfaceMeshNode[i].m_x += diff;
		}
	}
	else {
		for (int i = 0; i < nMeshNodeCnt; i++) {
			psbTetra->m_surfaceMeshNode[i].m_x += -center;
		}
		for (int i = 0; i < nMeshNodeCnt; i++) {
			psbTetra->m_surfaceMeshNode[i].m_x *= fScale;
		}
		for (int i = 0; i < nMeshNodeCnt; i++) {
			psbTetra->m_surfaceMeshNode[i].m_x += center;
		}
	}

	for (int i = 0; i < nMeshNodeCnt; i++) {
		psbTetra->m_surfaceMeshNode[i].m_x += trans;
	}

	for (int i = 0; i < nMeshNodeCnt; i++) {
		psbTetra->m_surfaceMeshNode[i].m_x0 = psbTetra->m_surfaceMeshNode[i].m_x;
	}


	// generate Hybrid Model ////////////////////////////////////////////////////////////////////
	btAlignedObjectArray<CiSoftBody::Tetra*> boundaryTetras;
	btAlignedObjectArray<btVector3> centroidTetras;

	for (int i = 0, ni = psbTetra->m_tetras.size(); i < ni; i++) {
		int nBoundary = 0;
		CiSoftBody::Tetra* t = &psbTetra->m_tetras[i];
		CiSoftBody::Node* p[4];

		for (int j = 0; j < 4; j++) {
			p[j] = t->m_n[j];
			if (p[j]->m_surface) {
				nBoundary++;
			}
		}

		if (nBoundary >= 3) {
			boundaryTetras.push_back(t);
			btVector3 centroid;
			centroid.setZero();

			for (int j = 0; j < 4; j++) {
				centroid += p[j]->m_x;
			}
			centroid /= 4.0;
			centroidTetras.push_back(centroid);
		}

		btVector4 v0(p[0]->m_x.x(), p[0]->m_x.y(), p[0]->m_x.z(), 1);
		btVector4 v1(p[1]->m_x.x(), p[1]->m_x.y(), p[1]->m_x.z(), 1);
		btVector4 v2(p[2]->m_x.x(), p[2]->m_x.y(), p[2]->m_x.z(), 1);
		btVector4 v3(p[3]->m_x.x(), p[3]->m_x.y(), p[3]->m_x.z(), 1);

		t->m_rv = Determinant4x4(v0, v1, v2, v3);
	}

	for (int i = 0, ni = boundaryTetras.size(); i < ni; i++) {
		psbTetra->m_boundaryTetras.push_back(boundaryTetras[i]);
	}

	///// 방법 1  /////
	btScalar avgLen = 0.0;
	int nTetraSize = psbTetra->m_tetras.size();

	for (int i = 0, ni = nMeshNodeCnt; i < ni; i++) {
		btVector3& p = psbTetra->m_surfaceMeshNode[i].m_x;
		btScalar minD = FLT_MAX;
		int minTetIdx = -1;

		// find closest mesh point tet centroid
		for (int j = 0, nj = boundaryTetras.size(); j < nj; j++) {
			btVector3& centroid = centroidTetras[j];
			btScalar d = (centroid - p).length();
			if (minD >= d) {
				minD = d;
				minTetIdx = boundaryTetras[j]->m_idx;
			}
		}

		// 가장 가까운 tet이 선택되었을 때, 해당 tetra barycentric coordinate로 표현
		if (minTetIdx != -1 && minTetIdx < nTetraSize) {
			CiSoftBody::Tetra* t = &psbTetra->m_tetras[minTetIdx];
			CiSoftBody::Node* n[4];
			for (int j = 0; j < 4; j++) {
				n[j] = t->m_n[j];
			}

			btVector4 b = GetBarycentricCoordinate(n[0]->m_x, n[1]->m_x, n[2]->m_x, n[3]->m_x, p);
			psbTetra->m_bary[i] = b;
			psbTetra->m_attachedTetraIdx[i] = minTetIdx;
			avgLen += minD;
		}
		else {
			printf("generateHybridModel: minTetIdx: -1\n");
			printf("stop");
			system("pause");
		}
	}
	avgLen /= nMeshNodeCnt;
	psbTetra->m_avgSurfaceMargin = avgLen;

	centroidTetras.clear();
	boundaryTetras.clear();

	return psbTetra;
}
CiSoftBody* CiSoftBodyHelpers::mergeTetra(CiSoftBody* psb, CiSoftBody* psb2, const char* hetero)
{
	printf("== mergeTetra ==\n");
	if (psb->m_faces.size() == 0) {
		// 기존 tetra에 이종 tetra material 번호 재할당
		for (int midx = 0, nmidx = psb2->m_materials.size(); midx < nmidx; midx++) {

			CiSoftBody::Material* m = psb->appendMaterial(psb2->m_materials[midx]);
			for (int i = 0, ni = psb->m_materials.size(); i < ni; i++) {
				psb->m_materials[i]->m_Idx = i;
			}

			FILE* heterofp = NULL;
			fopen_s(&heterofp, hetero, "r");

			if (heterofp != NULL) {
				char pcBuf[1024] = { 0 };

				int nTetraSize = 0;
				fgets(pcBuf, sizeof(pcBuf), heterofp);
				sscanf_s(pcBuf, "%d", &nTetraSize);

				int nIdx = 0, nBool = 0;
				for (int i = 0; i < nTetraSize; i++) {
					CiSoftBody::Tetra& t = psb->m_tetras[i];

					fgets(pcBuf, sizeof(pcBuf), heterofp);
					sscanf_s(pcBuf, "%d %d", &nIdx, &nBool);

					t.m_hetero = nBool;
				}

				fclose(heterofp);
			}
			else {
				// input
				for (int i = 0, ni = psb->m_tetras.size(); i < ni; i++) {
					//printf("L1 %d<%d\n", i, psb->m_tetras.size());

					CiSoftBody::Tetra& t = psb->m_tetras[i];
					btVector3 n[4];
					for (int j = 0; j < 4; j++) {
						n[j] = t.m_n[j]->m_x0;
					}

					int nInterpolationCnt = 2;
					for (int j = 0, nj = psb2->m_links.size(); j < nj; j++) {
						btVector3 node[2];
						node[0] = psb2->m_links[j].m_n[0]->m_x0;
						node[1] = psb2->m_links[j].m_n[1]->m_x0;

						for (int k = 0; k <= nInterpolationCnt; k++) {
							btVector3 p = node[0].lerp(node[1], k / (float)nInterpolationCnt);

							bool isInTetrahedron =
								pointInTetrahedron(n[0], n[1], n[2], n[3], p) &&
								pointInTetrahedron(n[1], n[2], n[3], n[0], p) &&
								pointInTetrahedron(n[2], n[3], n[0], n[1], p) &&
								pointInTetrahedron(n[3], n[0], n[1], n[2], p);

							if (isInTetrahedron) {
								t.m_hetero = true;
							}
						}
					}
				}

				fopen_s(&heterofp, hetero, "w");
				fprintf(heterofp, "%d\n", psb->m_tetras.size());

				for (int i = 0, ni = psb->m_tetras.size(); i < ni; i++) {
					CiSoftBody::Tetra& t = psb->m_tetras[i];
					fprintf(heterofp, "%d %d\n", i + 1, t.m_hetero);
				}
				fclose(heterofp);
			}



			// Node만 바꾸기 (우선)
			for (int i = 0, ni = psb->m_tetras.size(); i < ni; i++) {
				CiSoftBody::Tetra& t = psb->m_tetras[i];
				if (t.m_hetero == false) { continue; }

				CiSoftBody::Node* n[4];
				for (int j = 0; j < 4; j++) {
					n[j] = t.m_n[j];
				}

				// node
				for (int j = 0; j < 4; j++) {
					n[j]->m_mat = m;
				}
			}

			// tetra 중간중간 빈거 채우기 (노드 material idx보고)
			for (int i = 0, ni = psb->m_tetras.size(); i < ni; i++) {
				CiSoftBody::Tetra* t = &psb->m_tetras[i];

				if (t->m_hetero == false) {
					int nCnt = 0;
					CiSoftBody::Node* n[4];
					for (int j = 0; j < 4; j++) {
						n[j] = t->m_n[j];

						if (n[j]->m_mat->m_Idx == 1) {
							nCnt++;
						}
					}
					if (nCnt >= 4 && t->m_mat->m_Idx == 0) {
						t->m_hetero = true;
					}
				}
			}

			// 재질 바꾸기
			for (int i = 0, ni = psb->m_tetras.size(); i < ni; i++) {
				CiSoftBody::Tetra& t = psb->m_tetras[i];
				if (t.m_hetero == false) { continue; }

				CiSoftBody::Node* n[4];
				for (int j = 0; j < 4; j++) {
					n[j] = t.m_n[j];
				}

				// node
				for (int j = 0; j < 4; j++) {
					n[j]->m_mat = m;
				}

				// link
				for (int j = 0; j < 4; j++) {
					for (int k = 0; k < 4; k++) {
						if (j == k) { continue; }

						CiSoftBody::Link* l = NULL;
						for (int s = 0, ns = psb->m_links.size(); s < ns; ++s)
						{
							l = &psb->m_links[s];
							if ((l->m_n[0] == n[0] && l->m_n[1] == n[1]) ||
								(l->m_n[0] == n[1] && l->m_n[1] == n[0]))
							{
								break;
							}
						}
						if (l != NULL) {
							l->m_mat = m;
						}
					}
				}

				// tetra
				t.m_mat = m;
			}

			int nHeteroSize = 0;
			for (int i = 0; i < psb->m_tetras.size(); i++) {
				if (psb->m_tetras[i].m_hetero) {
					nHeteroSize++;
				}
			}
			printf("surfaceMeshNode =%d hetero = %d\n", psb->m_surfaceMeshNode.size(), nHeteroSize);
		}

		// tertaFace //
		/*
		for(int i=0,ni=psb->m_tetrasSurface.size(); i<ni; i++) {
			CiSoftBody::Face& f = psb->m_tetrasSurface[i];

			int heteroPointCnt = 0;
			for(int j=0; j<3; j++) {
				CiSoftBody::Node* n = f.m_n[j];
				if(n->m_surface && n->m_mat->m_Idx == 1) {
					heteroPointCnt++;
				}
			}
			if(heteroPointCnt == 3) {
				f.m_isDrawSkip = true;
			}
		}
		*/
		//printf("%d\n", psb->m_tetrasSurface.size());

		/////////////////////////////////////////////////////////////////////
		// child copy 
		psb->m_child.push_back(psb2);

		// coordinate mapping 변경
		if (psb2->m_surfaceMeshNode.size()) {
			// node, face는 그대로, 이외의 값은 현재 병합된 Tetra의 값을 따라야함
			btAlignedObjectArray<CiSoftBody::Tetra*> heteroTetras;
			btAlignedObjectArray<btVector3> centroidTetras;

			for (int i = 0, ni = psb->m_tetras.size(); i < ni; i++) {
				CiSoftBody::Tetra* t = &psb->m_tetras[i];
				if (t->m_hetero) {
					heteroTetras.push_back(t);
					btVector3 centroid;
					centroid.setZero();

					for (int j = 0; j < 4; j++) {
						centroid += t->m_n[j]->m_x;
					}
					centroid /= 4.0;
					centroidTetras.push_back(centroid);

					btVector4 v0(t->m_n[0]->m_x, 1);
					btVector4 v1(t->m_n[1]->m_x, 1);
					btVector4 v2(t->m_n[2]->m_x, 1);
					btVector4 v3(t->m_n[3]->m_x, 1);

					t->m_rv = Determinant4x4(v0, v1, v2, v3);
				}
			}

			int nMeshNodeCount = psb2->m_surfaceMeshNode.size();
			int nMeshFaceCount = psb2->m_surfaceMeshFace.size();
			int nheteroTetrasCount = heteroTetras.size();
			psb2->m_attachedTetraIdx.clear();
			psb2->m_attachedTetraIdx.resize(nMeshNodeCount);
			for (int i = 0; i < nMeshNodeCount; i++) {
				psb2->m_attachedTetraIdx[i] = -1;
			}

			btScalar avgLen = 0.0;
			for (int i = 0; i < nMeshNodeCount; i++) {
				btVector3& p = psb2->m_surfaceMeshNode[i].m_x;
				btScalar minD = FLT_MAX;
				int minTetIdx = -1;

				// find closest mesh point tet centroid
				for (int j = 0, nj = nheteroTetrasCount; j < nj; j++) {
					btVector3& centroid = centroidTetras[j];
					btScalar d = (centroid - p).length();
					if (minD >= d) {
						minD = d;
						minTetIdx = heteroTetras[j]->m_idx;
					}
				}

				if (minTetIdx != -1) {
					CiSoftBody::Tetra* t = &psb->m_tetras[minTetIdx];
					CiSoftBody::Node* n[4];
					for (int j = 0; j < 4; j++) {
						n[j] = t->m_n[j];
					}

					btVector4 b = GetBarycentricCoordinate(n[0]->m_x, n[1]->m_x, n[2]->m_x, n[3]->m_x, p);

					psb2->m_bary[i] = b;
					psb2->m_attachedTetraIdx[i] = minTetIdx;
					avgLen += minD;
				}
				else {
					printf("generateHybridModel: minTetIdx: -1\n");
					printf("stop");
					system("pause");
				}
			}

			avgLen /= nMeshNodeCount;
			psb2->m_avgSurfaceMargin = avgLen;
		}
	}

	return psb;
}

btVector3 CiSoftBodyHelpers::getCenter(const char * node, int nStartIndexNum, bool bFlipYZAxis)
{
	btAlignedObjectArray<btVector3>	pos;
	int								nnode = 0;
	int								ndims = 0;
	int								nattrb = 0;
	int								hasbounds = 0;

	FILE* nodefp = NULL;
	fopen_s(&nodefp, node, "r");
	if (nodefp == NULL)

	{
		printf("%s", node);
		system("pause");
		exit(1);
	}

	char pcBuf[1024] = { 0 };

	fgets(pcBuf, sizeof(pcBuf), nodefp);
	while (pcBuf[0] == '#') {
		fgets(pcBuf, sizeof(pcBuf), nodefp);
	}
	int result = sscanf_s(pcBuf, "%d %d %d %d", &nnode, &ndims, &nattrb, &hasbounds);

	pos.resize(nnode);

	fgets(pcBuf, sizeof(pcBuf), nodefp);
	while (pcBuf[0] == '#') {
		fgets(pcBuf, sizeof(pcBuf), nodefp);
	}
	for (int i = 0; i < nnode; ++i) {
		int index = 0;
		float	x, y, z;

		sscanf_s(pcBuf, "%d %f %f %f", &index, &x, &y, &z);


		if (bFlipYZAxis) {
			pos[index - nStartIndexNum].setX(btScalar(-x));
			pos[index - nStartIndexNum].setY(btScalar(z));
			pos[index - nStartIndexNum].setZ(btScalar(y));
		}
		else {
			pos[index - nStartIndexNum].setX(btScalar(x));
			pos[index - nStartIndexNum].setY(btScalar(y));
			pos[index - nStartIndexNum].setZ(btScalar(z));
		}
		pos[index - nStartIndexNum].setW(0);

		fgets(pcBuf, sizeof(pcBuf), nodefp);
	}
	fclose(nodefp);


	// get Center

	btVector3 center(0, 0, 0);
	for (int i = 0; i < nnode; i++) {
		center += pos[i];
	}
	center /= nnode;

	return center;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int b3ResourcePath::getExePath(char* path, int maxPathLenInBytes)
{
	int numBytes = 0;

#if __APPLE__
 uint32_t  bufsize = uint32_t(maxPathLenInBytes);

	if (_NSGetExecutablePath(path, &bufsize)!=0)
	{
		b3Warning("Cannot find executable path\n");
		return false;
	} else
	{
		numBytes = strlen(path);
	}
#else
#ifdef _WIN32
	//https://msdn.microsoft.com/en-us/library/windows/desktop/ms683197(v=vs.85).aspx

	HMODULE hModule = GetModuleHandle(NULL);
	numBytes = GetModuleFileNameA(hModule, path, maxPathLenInBytes);

#else
	///http://stackoverflow.com/questions/933850/how-to-find-the-location-of-the-executable-in-c
	numBytes = (int)readlink("/proc/self/exe", path, maxPathLenInBytes-1);
	if (numBytes > 0) 
	{
		path[numBytes] = 0;
	} else
	{
		b3Warning("Cannot find executable path\n");
	}
#endif //_WIN32
#endif //__APPLE__

	return numBytes;
}
void b3ResourcePath::setAdditionalSearchPath(const char* path)
{
	if (path)
	{
		int len = strlen(path);
		if (len<(B3_MAX_EXE_PATH_LEN-1))
		{
			strcpy_s(sAdditionalSearchPath, B3_MAX_EXE_PATH_LEN-1, path);
			sAdditionalSearchPath[len] = 0;
		}
	} else
	{
		sAdditionalSearchPath[0] = 0;
	}
}
int b3ResourcePath::findResourcePath(const char* resourceName, char* resourcePathOut, int resourcePathMaxNumBytes)
{
	//first find in a resource/<exeName> location, then in various folders within 'data' using b3FileUtils
	char exePath[B3_MAX_EXE_PATH_LEN];

	bool res = b3FileUtils::findFile(resourceName, resourcePathOut, resourcePathMaxNumBytes);
	if (res)
    {
            return strlen(resourcePathOut);
    }

	if (sAdditionalSearchPath[0])
	{
		TempResourcePath tmpPath(resourcePathMaxNumBytes+1024);
		char* resourcePathIn = tmpPath.m_path;
		sprintf_s(resourcePathIn,sizeof(char)*(resourcePathMaxNumBytes + 1024),"%s/%s",sAdditionalSearchPath,resourceName);
		//printf("try resource at %s\n", resourcePath);	
		if (b3FileUtils::findFile(resourcePathIn, resourcePathOut, resourcePathMaxNumBytes))
		{
			return strlen(resourcePathOut);
		}
	}

	int l = b3ResourcePath::getExePath(exePath, B3_MAX_EXE_PATH_LEN);
	if (l)
	{
 		char pathToExe[B3_MAX_EXE_PATH_LEN];

        	int exeNamePos = b3FileUtils::extractPath(exePath,pathToExe,B3_MAX_EXE_PATH_LEN);
        	if (exeNamePos)
        	{
				TempResourcePath tmpPath(resourcePathMaxNumBytes+1024);
				char* resourcePathIn = tmpPath.m_path;
				sprintf_s(resourcePathIn, sizeof(char)*(resourcePathMaxNumBytes + 1024), "%s../data/%s",pathToExe,resourceName);
				//printf("try resource at %s\n", resourcePath);	
				if (b3FileUtils::findFile(resourcePathIn, resourcePathOut, resourcePathMaxNumBytes))
				{
					return strlen(resourcePathOut);
				}

				sprintf_s(resourcePathIn,sizeof(char)*B3_MAX_EXE_PATH_LEN,"%s../resources/%s/%s",pathToExe,&exePath[exeNamePos],resourceName);
				//printf("try resource at %s\n", resourcePath);	
				if (b3FileUtils::findFile(resourcePathIn, resourcePathOut, resourcePathMaxNumBytes))
				{
					return strlen(resourcePathOut);
				}
				sprintf_s(resourcePathIn, sizeof(char)*B3_MAX_EXE_PATH_LEN, "%s.runfiles/google3/third_party/bullet/data/%s",exePath,resourceName);
				//printf("try resource at %s\n", resourcePath);	
				if (b3FileUtils::findFile(resourcePathIn, resourcePathOut, resourcePathMaxNumBytes))
				{
					return strlen(resourcePathOut);
				}  
        	}
	}
	return 0;
}

namespace tinyobj {

std::istream& safeGetline(std::istream& is, std::string& t)
{
    t.clear();

    std::istream::sentry se(is, true);
    std::streambuf* sb = is.rdbuf();

    for(;;) {
        int c = sb->sbumpc();
        switch (c) {
        case '\n':
            return is;
        case '\r':
            if(sb->sgetc() == '\n')
                sb->sbumpc();
            return is;
        case EOF:
            // Also handle the case when the last line has no line ending
            if(t.empty())
                is.setstate(std::ios::eofbit);
            return is;
        default:
            t += (char)c;
        }
    }
}

struct vertex_index {
  int v_idx, vt_idx, vn_idx, dummy;
};
struct MyIndices
{
	int m_offset;
	int m_numIndices;
};


// for std::map
static inline bool operator<(const vertex_index& a, const vertex_index& b)
{
  if (a.v_idx != b.v_idx) return (a.v_idx < b.v_idx);
  if (a.vn_idx != b.vn_idx) return (a.vn_idx < b.vn_idx);
  if (a.vt_idx != b.vt_idx) return (a.vt_idx < b.vt_idx);

  return false;
}


static inline bool isSpace(const char c) {
  return (c == ' ') || (c == '\t');
}

static inline bool isNewLine(const char c) {
  return (c == '\r') || (c == '\n') || (c == '\0');
}

// Make index zero-base, and also support relative index. 
static inline int fixIndex(int idx, int n)
{
  int i;

  if (idx > 0) {
    i = idx - 1;
  } else if (idx == 0) {
    i = 0;
  } else { // negative value = relative
    i = n + idx;
  }
  return i;
}

static inline std::string parseString(const char*& token)
{
  std::string s;
  int b = strspn(token, " \t");
  int e = strcspn(token, " \t\r");
  s = std::string(&token[b], &token[e]);

  token += (e - b);
  return s;
}

static inline float parseFloat(const char*& token)
{
  token += strspn(token, " \t");
  float f = (float)atof(token);
  token += strcspn(token, " \t\r");
  return f;
}

static inline void parseFloat2(
  float& x, float& y,
  const char*& token)
{
  x = parseFloat(token);
  y = parseFloat(token);
}

static inline void parseFloat3(
  float& x, float& y, float& z,
  const char*& token)
{
  x = parseFloat(token);
  y = parseFloat(token);
  z = parseFloat(token);
}


// Parse triples: i, i/j/k, i//k, i/j
static vertex_index parseTriple(
  const char* &token,
  int vsize,
  int vnsize,
  int vtsize)
{
    vertex_index vi;
	vi.vn_idx = -1;
	vi.vt_idx = -1;
	vi.v_idx= -1;

    vi.v_idx = fixIndex(atoi(token), vsize);
    token += strcspn(token, "/ \t\r");
    if (token[0] != '/') {
      return vi;
    }
    token++;

    // i//k
    if (token[0] == '/') {
      token++;
      vi.vn_idx = fixIndex(atoi(token), vnsize);
      token += strcspn(token, "/ \t\r");
      return vi;
    }
    
    // i/j/k or i/j
    vi.vt_idx = fixIndex(atoi(token), vtsize);
    token += strcspn(token, "/ \t\r");
    if (token[0] != '/') {
      return vi;
    }

    // i/j/k
    token++;  // skip '/'
    vi.vn_idx = fixIndex(atoi(token), vnsize);
    token += strcspn(token, "/ \t\r");
    return vi; 
}


static unsigned int
updateVertex(
  std::map<vertex_index, unsigned int>& vertexCache,
  std::vector<float>& positions,
  std::vector<float>& normals,
  std::vector<float>& texcoords,
  const std::vector<float>& in_positions,
  const std::vector<float>& in_normals,
  const std::vector<float>& in_texcoords,
  const vertex_index& i)
{
  const std::map<vertex_index, unsigned int>::iterator it = vertexCache.find(i);

  if (it != vertexCache.end()) {
    // found cache
    return it->second;
  }

  assert(static_cast<int>(in_positions.size()) > (3*i.v_idx+2));

  positions.push_back(in_positions[3*i.v_idx+0]);
  positions.push_back(in_positions[3*i.v_idx+1]);
  positions.push_back(in_positions[3*i.v_idx+2]);

  if (i.vn_idx >= 0 && ((3*i.vn_idx+2)<in_normals.size())) {
    normals.push_back(in_normals[3*i.vn_idx+0]);
    normals.push_back(in_normals[3*i.vn_idx+1]);
    normals.push_back(in_normals[3*i.vn_idx+2]);
  }

  if (i.vt_idx >= 0) {
	int numTexCoords = in_texcoords.size();
	int index0 = 2*i.vt_idx+0;
	int index1 = 2*i.vt_idx+1;

	if (index0>=0 && (index0)<numTexCoords)
	{
		texcoords.push_back(in_texcoords[index0]);
	}
	if (index1>=0 && (index1)<numTexCoords)
	{
		texcoords.push_back(in_texcoords[index1]);
	}
  }

  unsigned int idx = positions.size() / 3 - 1;
  vertexCache[i] = idx;

  return idx;
}


static bool
exportFaceGroupToShape(
  shape_t& shape,
  const std::vector<float>& in_positions,
  const std::vector<float>& in_normals,
  const std::vector<float>& in_texcoords,
  const std::vector<MyIndices >& faceGroup,
  const material_t material,
  const std::string name,
  std::vector<vertex_index>& allIndices
  )
{
  if (faceGroup.empty()) {
    return false;
  }

  // Flattened version of vertex data
  std::vector<float> positions;
  std::vector<float> normals;
  std::vector<float> texcoords;
  std::map<vertex_index, unsigned int> vertexCache;
  std::vector<unsigned int> indices;

  // Flatten vertices and indices
  /*
  for (size_t i = 0; i < faceGroup.size(); i++) 
  {
    const MyIndices& face = faceGroup[i];
	
	//Node번호가 바뀜 //
	
	vertex_index i0 = allIndices[face.m_offset];
    vertex_index i1;
	i1.vn_idx = -1;
	i1.vt_idx = -1;
	i1.v_idx= -1;
	vertex_index i2 = allIndices[face.m_offset+1];

	size_t npolys = face.m_numIndices;//.size();
	{
		// Polygon -> triangle fan conversion
		for (size_t k = 2; k < npolys; k++) 
		{
		  i1 = i2;
		  i2 = allIndices[face.m_offset+k];

		  unsigned int v0 = updateVertex(vertexCache, positions, normals, texcoords, in_positions, in_normals, in_texcoords, i0);
		  unsigned int v1 = updateVertex(vertexCache, positions, normals, texcoords, in_positions, in_normals, in_texcoords, i1);
		  unsigned int v2 = updateVertex(vertexCache, positions, normals, texcoords, in_positions, in_normals, in_texcoords, i2);

		  indices.push_back(v0);
		  indices.push_back(v1);
		  indices.push_back(v2);
		}
	}
  }
  */

  for(size_t i=0; i< in_positions.size(); i++) {
	  positions.push_back(in_positions.at(i));
  }

  for(size_t i=0; i< faceGroup.size()*3; i++) {
	  indices.push_back(allIndices.at(i).v_idx);
  }

  //
  // Construct shape.
  //
  shape.name = name;
  shape.mesh.positions.swap(positions);
  shape.mesh.normals.swap(normals);
  shape.mesh.texcoords.swap(texcoords);
  shape.mesh.indices.swap(indices);

  shape.material = material;

  return true;

}


  
void InitMaterial(material_t& material) {
  material.name = "";
  material.ambient_texname = "";
  material.diffuse_texname = "";
  material.specular_texname = "";
  material.normal_texname = "";
  for (int i = 0; i < 3; i ++) {
    material.ambient[i] = 0.f;
    material.diffuse[i] = 0.f;
    material.specular[i] = 0.f;
    material.transmittance[i] = 0.f;
    material.emission[i] = 0.f;
  }
  material.shininess = 1.f;
}

std::string LoadMtl (
  std::map<std::string, material_t>& material_map,
  const char* filename,
  const char* mtl_basepath)
{
  material_map.clear();
  std::stringstream err;

  std::string filepath;

  if (mtl_basepath) {
    filepath = std::string(mtl_basepath) + std::string(filename);
  } else {
    filepath = std::string(filename);
  }

  std::ifstream ifs(filepath.c_str());
  if (!ifs) {
    err << "Cannot open file [" << filepath << "]" << std::endl;
    return err.str();
  }

  material_t material;
  
  int maxchars = 8192;  // Alloc enough size.
  std::vector<char> buf(maxchars);  // Alloc enough size.
  while (ifs.peek() != -1) {

    std::string linebuf;
    safeGetline(ifs,linebuf);



    // Trim newline '\r\n' or '\r'
    if (linebuf.size() > 0) {
      if (linebuf[linebuf.size()-1] == '\n') linebuf.erase(linebuf.size()-1);
    }
    if (linebuf.size() > 0) {
      if (linebuf[linebuf.size()-1] == '\n') linebuf.erase(linebuf.size()-1);
    }

    // Skip if empty line.
    if (linebuf.empty()) {
      continue;
    }

    linebuf = linebuf.substr(0, linebuf.find_last_not_of(" \t") + 1);

    // Skip leading space.
    const char* token = linebuf.c_str();
    token += strspn(token, " \t");

    assert(token);
    if (token[0] == '\0') continue; // empty line
    
    if (token[0] == '#') continue;  // comment line
    
    // new mtl
    if ((0 == strncmp(token, "newmtl", 6)) && isSpace((token[6]))) {
      // flush previous material.
      material_map.insert(std::pair<std::string, material_t>(material.name, material));

      // initial temporary material
      InitMaterial(material);

      // set new mtl name
      char namebuf[4096];
      token += 7;
      sscanf_s(token, "%s", namebuf, sizeof(char)*4096);
      material.name = namebuf;
      continue;
    }
    
    // ambient
    if (token[0] == 'K' && token[1] == 'a' && isSpace((token[2]))) {
      token += 2;
      float r, g, b;
      parseFloat3(r, g, b, token);
      material.ambient[0] = r;
      material.ambient[1] = g;
      material.ambient[2] = b;
      continue;
    }
    
    // diffuse
    if (token[0] == 'K' && token[1] == 'd' && isSpace((token[2]))) {
      token += 2;
      float r, g, b;
      parseFloat3(r, g, b, token);
      material.diffuse[0] = r;
      material.diffuse[1] = g;
      material.diffuse[2] = b;
      continue;
    }
    
    // specular
    if (token[0] == 'K' && token[1] == 's' && isSpace((token[2]))) {
      token += 2;
      float r, g, b;
      parseFloat3(r, g, b, token);
      material.specular[0] = r;
      material.specular[1] = g;
      material.specular[2] = b;
      continue;
    }
    
    // specular
    if (token[0] == 'K' && token[1] == 't' && isSpace((token[2]))) {
      token += 2;
      float r, g, b;
      parseFloat3(r, g, b, token);
      material.specular[0] = r;
      material.specular[1] = g;
      material.specular[2] = b;
      continue;
    }

    // emission
    if(token[0] == 'K' && token[1] == 'e' && isSpace(token[2])) {
      token += 2;
      float r, g, b;
      parseFloat3(r, g, b, token);
      material.emission[0] = r;
      material.emission[1] = g;
      material.emission[2] = b;
      continue;
    }

    // shininess
    if(token[0] == 'N' && token[1] == 's' && isSpace(token[2])) {
      token += 2;
      material.shininess = parseFloat(token);
      continue;
    }

    // ambient texture
    if ((0 == strncmp(token, "map_Ka", 6)) && isSpace(token[6])) {
      token += 7;
      material.ambient_texname = token;
      continue;
    }

    // diffuse texture
    if ((0 == strncmp(token, "map_Kd", 6)) && isSpace(token[6])) {
      token += 7;
      material.diffuse_texname = token;
      continue;
    }

    // specular texture
    if ((0 == strncmp(token, "map_Ks", 6)) && isSpace(token[6])) {
      token += 7;
      material.specular_texname = token;
      continue;
    }

    // normal texture
    if ((0 == strncmp(token, "map_Ns", 6)) && isSpace(token[6])) {
      token += 7;
      material.normal_texname = token;
      continue;
    }

    // unknown parameter
    const char* _space = strchr(token, ' ');
    if(!_space) {
      _space = strchr(token, '\t');
    }
    if(_space) {
      int len = _space - token;
      std::string key(token, len);
      std::string value = _space + 1;
      material.unknown_parameter.insert(std::pair<std::string, std::string>(key, value));
    }
  }
  // flush last material.
  material_map.insert(std::pair<std::string, material_t>(material.name, material));

  return err.str();
}

std::string
LoadObj(
  std::vector<shape_t>& shapes,
  const char* filename,
  const char* mtl_basepath)
{
  std::string tmp = filename;
  if (!mtl_basepath) {
    int last_slash = 0;
    for (int c=0; c<(int)tmp.size(); ++c)
      if (tmp[c]=='/' || tmp[c]=='\\')
        last_slash = c;
    tmp = tmp.substr(0, last_slash);
    mtl_basepath = tmp.c_str();
    //fprintf(stderr, "MTL PATH '%s' orig '%s'\n", mtl_basepath, filename);
  }

  shapes.resize(0);
  std::vector<vertex_index> allIndices;
  allIndices.reserve(1024*1024);

  MyIndices face;

  std::stringstream err;

  std::ifstream ifs(filename);
  if (!ifs) {
    err << "Cannot open file [" << filename << "]" << std::endl;
    return err.str();
  }

  std::vector<float> v;
  v.reserve(1024*1024);
  std::vector<float> vn;
  vn.reserve(1024*1024);
  std::vector<float> vt;
  vt.reserve(1024*1024);
  //std::vector<std::vector<vertex_index> > faceGroup;
  std::vector<MyIndices> faceGroup;
  faceGroup.reserve(1024*1024);
  std::string name;

  // material
  std::map<std::string, material_t> material_map;
  material_t material;
  InitMaterial(material);

  int maxchars = 8192;  // Alloc enough size.
  std::vector<char> buf(maxchars);  // Alloc enough size.
  std::string linebuf;
  linebuf.reserve(maxchars);

  while (ifs.peek() != -1) {

	linebuf.resize(0);
    safeGetline(ifs,linebuf);

    // Trim newline '\r\n' or '\r'
    if (linebuf.size() > 0) {
      if (linebuf[linebuf.size()-1] == '\n') linebuf.erase(linebuf.size()-1);
    }
    if (linebuf.size() > 0) {
      if (linebuf[linebuf.size()-1] == '\n') linebuf.erase(linebuf.size()-1);
    }

    // Skip if empty line.
    if (linebuf.empty()) {
      continue;
    }

    // Skip leading space.
    const char* token = linebuf.c_str();
    token += strspn(token, " \t");

    assert(token);
    if (token[0] == '\0') continue; // empty line
    
    if (token[0] == '#') continue;  // comment line

    // vertex
    if (token[0] == 'v' && isSpace((token[1]))) {
      token += 2;
      float x, y, z;
      parseFloat3(x, y, z, token);
      v.push_back(x);
      v.push_back(y);
      v.push_back(z);
      continue;
    }

    // normal
    if (token[0] == 'v' && token[1] == 'n' && isSpace((token[2]))) {
      token += 3;
      float x, y, z;
      parseFloat3(x, y, z, token);
      vn.push_back(x);
      vn.push_back(y);
      vn.push_back(z);
      continue;
    }

    // texcoord
    if (token[0] == 'v' && token[1] == 't' && isSpace((token[2]))) {
      token += 3;
      float x, y;
      parseFloat2(x, y, token);
      vt.push_back(x);
      vt.push_back(y);
      continue;
    }

    // face
    if (token[0] == 'f' && isSpace((token[1]))) {
      token += 2;
      token += strspn(token, " \t");

	  face.m_offset = allIndices.size();
	  face.m_numIndices = 0;
      
      while (!isNewLine(token[0])) {
        vertex_index vi = parseTriple(token, v.size() / 3, vn.size() / 3, vt.size() / 2);
        allIndices.push_back(vi);
		face.m_numIndices++;
        int n = strspn(token, " \t\r");
        token += n;
      }

      faceGroup.push_back(face);
	        
      continue;
    }

    // use mtl
    if ((0 == strncmp(token, "usemtl", 6)) && isSpace((token[6]))) {

      char namebuf[4096];
      token += 7;
      sscanf_s(token, "%s", namebuf, sizeof(char)*4096);

      if (material_map.find(namebuf) != material_map.end()) {
        material = material_map[namebuf];
      } else {
        // { error!! material not found }
        InitMaterial(material);
      }
      continue;

    }

    // load mtl
    if ((0 == strncmp(token, "mtllib", 6)) && isSpace((token[6]))) {
      char namebuf[4096];
      token += 7;
      sscanf_s(token, "%s", namebuf, sizeof(char)*4096);

      std::string err_mtl = LoadMtl(material_map, namebuf, mtl_basepath);
      if (!err_mtl.empty()) {
        //faceGroup.resize(0);  // for safety
        //return err_mtl;
      }
      continue;
    }

    // group name
    if (token[0] == 'g' && isSpace((token[1]))) {

      // flush previous face group.
      shape_t shape;
      bool ret = exportFaceGroupToShape(shape, v, vn, vt, faceGroup, material, name,allIndices);
      if (ret) {
        shapes.push_back(shape);
      }

      faceGroup.resize(0);

      std::vector<std::string> names;
      while (!isNewLine(token[0])) {
        std::string str = parseString(token);
        names.push_back(str);
        token += strspn(token, " \t\r"); // skip tag
      }

      assert(names.size() > 0);

      // names[0] must be 'g', so skipt 0th element.
      if (names.size() > 1) {
        name = names[1];
      } else {
        name = "";
      }

      continue;
    }

    // object name
    if (token[0] == 'o' && isSpace((token[1]))) {

      // flush previous face group.
      shape_t shape;
      bool ret = exportFaceGroupToShape(shape, v, vn, vt, faceGroup, material, name,allIndices);
      if (ret) {
        shapes.push_back(shape);
      }

      faceGroup.resize(0);

      // @todo { multiple object name? }
      char namebuf[4096];
      token += 2;
      sscanf_s(token, "%s", namebuf, sizeof(char)*4096);
      name = std::string(namebuf);


      continue;
    }

    // Ignore unknown command.
  }

  shape_t shape;
  bool ret = exportFaceGroupToShape(shape, v, vn, vt, faceGroup, material, name, allIndices);
  if (ret) {
    shapes.push_back(shape);
  }
  faceGroup.resize(0);  // for safety
  
  return err.str();
}

};