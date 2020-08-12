#include "btAlignedAllocator.h"

int gNumAlignedAllocs = 0;
int gNumAlignedFree = 0;
int gTotalBytesAlignedAllocs = 0;//detect memory leaks

static void *btAllocDefault(size_t size)
{
	return malloc(size);
}

static void btFreeDefault(void *ptr)
{
	free(ptr);
}

static btAllocFunc *sAllocFunc = btAllocDefault;
static btFreeFunc *sFreeFunc = btFreeDefault;



#if defined (BT_HAS_ALIGNED_ALLOCATOR)
#include <malloc.h>
static void *btAlignedAllocDefault(size_t size, int alignment)
{
	return _aligned_malloc(size, (size_t)alignment);
}

static void btAlignedFreeDefault(void *ptr)
{
	_aligned_free(ptr);
}
#elif defined(__CELLOS_LV2__)
#include <stdlib.h>

static inline void *btAlignedAllocDefault(size_t size, int alignment)
{
	return memalign(alignment, size);
}

static inline void btAlignedFreeDefault(void *ptr)
{
	free(ptr);
}
#else





static inline void *btAlignedAllocDefault(size_t size, int alignment)
{
  void *ret;
  char *real;
  real = (char *)sAllocFunc(size + sizeof(void *) + (alignment-1));
  if (real) {
	ret = btAlignPointer(real + sizeof(void *),alignment);
    *((void **)(ret)-1) = (void *)(real);
  } else {
    ret = (void *)(real);
  }
  return (ret);
}

static inline void btAlignedFreeDefault(void *ptr)
{
  void* real;

  if (ptr) {
    real = *((void **)(ptr)-1);
    sFreeFunc(real);
  }
}
#endif


static btAlignedAllocFunc *sAlignedAllocFunc = btAlignedAllocDefault;
static btAlignedFreeFunc *sAlignedFreeFunc = btAlignedFreeDefault;

void btAlignedAllocSetCustomAligned(btAlignedAllocFunc *allocFunc, btAlignedFreeFunc *freeFunc)
{
  sAlignedAllocFunc = allocFunc ? allocFunc : btAlignedAllocDefault;
  sAlignedFreeFunc = freeFunc ? freeFunc : btAlignedFreeDefault;
}

void btAlignedAllocSetCustom(btAllocFunc *allocFunc, btFreeFunc *freeFunc)
{
  sAllocFunc = allocFunc ? allocFunc : btAllocDefault;
  sFreeFunc = freeFunc ? freeFunc : btFreeDefault;
}

#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
//this generic allocator provides the total allocated number of bytes
#include <stdio.h>

void*   btAlignedAllocInternal  (size_t size, int alignment,int line,char* filename)
{
 void *ret;
 char *real;

 gTotalBytesAlignedAllocs += size;
 gNumAlignedAllocs++;

 
 real = (char *)sAllocFunc(size + 2*sizeof(void *) + (alignment-1));
 if (real) {
   ret = (void*) btAlignPointer(real + 2*sizeof(void *), alignment);
   *((void **)(ret)-1) = (void *)(real);
       *((int*)(ret)-2) = size;

 } else {
   ret = (void *)(real);//??
 }

 printf("allocation#%d at address %x, from %s,line %d, size %d\n",gNumAlignedAllocs,real, filename,line,size);

 int* ptr = (int*)ret;
 *ptr = 12;
 return (ret);
}

void    btAlignedFreeInternal   (void* ptr,int line,char* filename)
{

 void* real;
 gNumAlignedFree++;

 if (ptr) {
   real = *((void **)(ptr)-1);
       int size = *((int*)(ptr)-2);
       gTotalBytesAlignedAllocs -= size;

	   printf("free #%d at address %x, from %s,line %d, size %d\n",gNumAlignedFree,real, filename,line,size);

   sFreeFunc(real);
 } else
 {
	 printf("NULL ptr\n");
 }
}

#else //BT_DEBUG_MEMORY_ALLOCATIONS

void*	btAlignedAllocInternal	(size_t size, int alignment)
{
	gNumAlignedAllocs++;
	void* ptr;
	ptr = sAlignedAllocFunc(size, alignment);
//	printf("btAlignedAllocInternal %d, %x\n",size,ptr);
	return ptr;
}

void	btAlignedFreeInternal	(void* ptr)
{
	if (!ptr)
	{
		return;
	}

	gNumAlignedFree++;
//	printf("btAlignedFreeInternal %x\n",ptr);
	sAlignedFreeFunc(ptr);
}

#endif //BT_DEBUG_MEMORY_ALLOCATIONS

