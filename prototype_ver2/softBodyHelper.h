#pragma once
#include "softbody.h"
#include "rigidBody.h"

#include <algorithm>

#define SAFE_DELETE_ARRAY(p)	{ if(p) delete[](p); p = NULL;}
#define SAFE_DELETE_VOLUME(p, depth)	{ if(p) {for (int i=0;i<depth;i++)	if(p[i]) delete[](p[i]); } delete[] p; p=NULL;};

struct CiSoftBodyHelpers
{
	/// object /////////////////////////////////////////////////////////////////////////////////
	static CiRigidBody* CreateFromTriMeshFile(
		const char* objPath,
		bool bFlipYZAxis);

	static CiSoftBody * CreateFromTetGenFile(
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
		btVector3& trans);

	static CiSoftBody * CreateFromTetGenFile_test(
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
		btVector3& trans);

	static CiSoftBody* generateHybridModel(CiSoftBody* psbTetra, const char* meshFile, bool bFlipYZAxis, float fScale, bool bAlignObjectCenter, btVector3& center, btVector3& trans);
	static CiSoftBody* mergeTetra(CiSoftBody* psb, CiSoftBody* psb2, const char* hetero);

	static btVector3 getCenter(const char * node, int nStartIndexNum, bool bFlipYZAxis);
};

/// fileLoader //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef B3_LOGGING_H
#define B3_LOGGING_H

#ifdef __cplusplus
extern "C" {
#endif
    
///We add the do/while so that the statement "if (condition) b3Printf("test"); else {...}" would fail
///You can also customize the message by uncommenting out a different line below
#define b3Printf(...) b3OutputPrintfVarArgsInternal(__VA_ARGS__)
//#define b3Printf(...) do {b3OutputPrintfVarArgsInternal("b3Printf[%s,%d]:",__FILE__,__LINE__);b3OutputPrintfVarArgsInternal(__VA_ARGS__); } while(0)
//#define b3Printf b3OutputPrintfVarArgsInternal
//#define b3Printf(...) printf(__VA_ARGS__)
//#define b3Printf(...)

#define b3Warning(...) do {b3OutputWarningMessageVarArgsInternal("b3Warning[%s,%d]:\n",__FILE__,__LINE__);b3OutputWarningMessageVarArgsInternal(__VA_ARGS__); }while(0)
#define b3Error(...) do {b3OutputErrorMessageVarArgsInternal("b3Error[%s,%d]:\n",__FILE__,__LINE__);b3OutputErrorMessageVarArgsInternal(__VA_ARGS__); } while(0)


#ifndef B3_NO_PROFILE

void b3EnterProfileZone(const char* name);
void b3LeaveProfileZone();
#ifdef __cplusplus

class	b3ProfileZone
{
public:
	b3ProfileZone(const char* name)
	{ 
		b3EnterProfileZone( name ); 
	}

	~b3ProfileZone()
	{ 
		b3LeaveProfileZone(); 
	}
};

#define	B3_PROFILE( name )			b3ProfileZone __profile( name )
#endif

#else //B3_NO_PROFILE

#define	B3_PROFILE( name )
#define b3StartProfile(a)
#define b3StopProfile

#endif //#ifndef B3_NO_PROFILE


typedef void (b3PrintfFunc)(const char* msg);
typedef void (b3WarningMessageFunc)(const char* msg);
typedef void (b3ErrorMessageFunc)(const char* msg);
typedef void (b3EnterProfileZoneFunc)(const char* msg);
typedef void (b3LeaveProfileZoneFunc)();

///The developer can route b3Printf output using their own implementation
void b3SetCustomPrintfFunc(b3PrintfFunc* printfFunc);
void b3SetCustomWarningMessageFunc(b3WarningMessageFunc* warningMsgFunc);
void b3SetCustomErrorMessageFunc(b3ErrorMessageFunc* errorMsgFunc);

///Set custom profile zone functions (zones can be nested)
void b3SetCustomEnterProfileZoneFunc(b3EnterProfileZoneFunc* enterFunc);
void b3SetCustomLeaveProfileZoneFunc(b3LeaveProfileZoneFunc* leaveFunc);

///Don't use those internal functions directly, use the b3Printf or b3SetCustomPrintfFunc instead (or warning/error version)
void b3OutputPrintfVarArgsInternal(const char *str, ...);
void b3OutputWarningMessageVarArgsInternal(const char *str, ...);
void b3OutputErrorMessageVarArgsInternal(const char *str, ...);

#ifdef __cplusplus
    }
#endif

#endif//B3_LOGGING_H

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef B3_FILE_UTILS_H
#define B3_FILE_UTILS_H

#include <stdio.h>
#include <stddef.h>//ptrdiff_h
#include <string.h>

struct b3FileUtils
{
	b3FileUtils()
	{
	}
	virtual ~b3FileUtils()
	{
	}

	static bool findFile(const char* orgFileName, char* relativeFileName, int maxRelativeFileNameMaxLen)
	{
		FILE* f=0;
		fopen_s(&f, orgFileName,"rb");
                if (f)
                {
			//printf("original file found: [%s]\n", orgFileName);
			sprintf_s(relativeFileName, maxRelativeFileNameMaxLen * sizeof(char), "%s", orgFileName);
			//sprintf(relativeFileName, "%s", orgFileName);
			fclose(f);
			return true;
		}

		//printf("Trying various directories, relative to current working directory\n");	
			const char* prefix[]={"./","./data/","../data/","../../data/","../../../data/","../../../../data/"};
			int numPrefixes = sizeof(prefix)/sizeof(const char*);
	
			f=0;
			bool fileFound = false;

			for (int i=0;!f && i<numPrefixes;i++)
			{
#ifdef _WIN32
				sprintf_s(relativeFileName,maxRelativeFileNameMaxLen,"%s%s",prefix[i],orgFileName);
#else
				sprintf(relativeFileName,"%s%s",prefix[i],orgFileName);
#endif
				fopen_s(&f,relativeFileName,"rb");
				if (f)
				{
					fileFound = true;
					break;
				}
			}
			if (f)
			{
				fclose(f);
			}
	
		return fileFound;
	}

	static const char* strip2(const char* name, const char* pattern)
	{
		size_t const patlen = strlen(pattern);
		size_t patcnt = 0;
		const char * oriptr;
		const char * patloc;
		// find how many times the pattern occurs in the original string
		for (oriptr = name; (patloc = strstr(oriptr, pattern)); oriptr = patloc + patlen)
		{
			patcnt++;
		}
		return oriptr;
	}

	

	static int extractPath(const char* fileName, char* path, int maxPathLength)
	{
		const char* stripped = strip2(fileName, "/");
		stripped = strip2(stripped, "\\");

		ptrdiff_t len = stripped-fileName;

		if (len && ((len+1)<maxPathLength))
		{

			for (int i=0;i<len;i++)
			{
				path[i] = fileName[i];
			}
			path[len]=0;
		} else
		{
			len = 0;
			if (maxPathLength>0)
			{
				path[len] = 0;
			}
		}
		return len;
	}

	static char toLowerChar(const char t)
	{
		if (t>=(char)'A' && t<=(char)'Z')
			return t + ((char)'a' - (char)'A');
		else
			return t;
	}


	static void toLower(char* str)
	{
		int len=strlen(str);
		for (int i=0;i<len;i++)
		{
			str[i] = toLowerChar(str[i]);
		}
	}
};
#endif //B3_FILE_UTILS_H

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _B3_RESOURCE_PATH_H
#define _B3_RESOURCE_PATH_H 

#define B3_MAX_EXE_PATH_LEN 4096
#include <string>

struct TempResourcePath
{
	char* m_path;
	TempResourcePath(int len)
	{
		m_path = (char*)malloc(len);
		memset(m_path,0,len);
	}
	virtual ~TempResourcePath()
	{
		free(m_path);
	}
};
static char sAdditionalSearchPath[B3_MAX_EXE_PATH_LEN] = {0};

class b3ResourcePath
{
public:
	static int getExePath(char* path, int maxPathLenInBytes);
	static int findResourcePath(const char* sourceName, char* resourcePath, int maxResourcePathLenInBytes);
	static void setAdditionalSearchPath(const char* path);
};
#endif

#ifdef __APPLE__
#include <mach-o/dyld.h>	/* _NSGetExecutablePath */
#else
#ifdef _WIN32
#include <windows.h>
#else
//not Mac, not Windows, let's cross the fingers it is Linux :-)
#include <unistd.h>
#endif
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _TINY_OBJ_LOADER_H
#define _TINY_OBJ_LOADER_H

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <cassert>

namespace tinyobj {

	typedef struct
	{
		std::string name;

		float ambient[3];
		float diffuse[3];
		float specular[3];
		float transmittance[3];
		float emission[3];
		float shininess;

		std::string ambient_texname;
		std::string diffuse_texname;
		std::string specular_texname;
		std::string normal_texname;
		std::map<std::string, std::string> unknown_parameter;
	} material_t;

	typedef struct
	{
		std::vector<float>          positions;
		std::vector<float>          normals;
		std::vector<float>          texcoords;
		std::vector<unsigned int>   indices;
	} mesh_t;

	typedef struct
	{
		std::string  name;
		material_t   material;
		mesh_t       mesh;
	} shape_t;

	/// Loads .obj from a file.
	/// 'shapes' will be filled with parsed shape data
	/// The function returns error string.
	/// Returns empty string when loading .obj success.
	/// 'mtl_basepath' is optional, and used for base path for .mtl file.
	std::string LoadObj(
		std::vector<shape_t>& shapes,   // [output]
		const char* filename,
		const char* mtl_basepath = NULL);

};

#endif  // _TINY_OBJ_LOADER_H

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////