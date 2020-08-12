#ifndef BT_ALIGNED_ALLOCATOR
#define BT_ALIGNED_ALLOCATOR

///we probably replace this with our own aligned memory allocator
///so we replace _aligned_malloc and _aligned_free with our own
///that is better portable and more predictable

#include "btScalar.h"
//#define BT_DEBUG_MEMORY_ALLOCATIONS 1
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS

#define btAlignedAlloc(a,b) \
		btAlignedAllocInternal(a,b,__LINE__,__FILE__)

#define btAlignedFree(ptr) \
		btAlignedFreeInternal(ptr,__LINE__,__FILE__)

void*	btAlignedAllocInternal	(size_t size, int alignment,int line,char* filename);

void	btAlignedFreeInternal	(void* ptr,int line,char* filename);

#else
	void*	btAlignedAllocInternal	(size_t size, int alignment);
	void	btAlignedFreeInternal	(void* ptr);

	#define btAlignedAlloc(size,alignment) btAlignedAllocInternal(size,alignment)
	#define btAlignedFree(ptr) btAlignedFreeInternal(ptr)

#endif
typedef int	size_type;

typedef void *(btAlignedAllocFunc)(size_t size, int alignment);
typedef void (btAlignedFreeFunc)(void *memblock);
typedef void *(btAllocFunc)(size_t size);
typedef void (btFreeFunc)(void *memblock);

///The developer can let all Bullet memory allocations go through a custom memory allocator, using btAlignedAllocSetCustom
void btAlignedAllocSetCustom(btAllocFunc *allocFunc, btFreeFunc *freeFunc);
///If the developer has already an custom aligned allocator, then btAlignedAllocSetCustomAligned can be used. The default aligned allocator pre-allocates extra memory using the non-aligned allocator, and instruments it.
void btAlignedAllocSetCustomAligned(btAlignedAllocFunc *allocFunc, btAlignedFreeFunc *freeFunc);


///The btAlignedAllocator is a portable class for aligned memory allocations.
///Default implementations for unaligned and aligned allocations can be overridden by a custom allocator using btAlignedAllocSetCustom and btAlignedAllocSetCustomAligned.
template < typename T , unsigned Alignment >
class btAlignedAllocator {
	
	typedef btAlignedAllocator< T , Alignment > self_type;
	
public:

	//just going down a list:
	btAlignedAllocator() {}
	/*
	btAlignedAllocator( const self_type & ) {}
	*/

	template < typename Other >
	btAlignedAllocator( const btAlignedAllocator< Other , Alignment > & ) {}

	typedef const T*         const_pointer;
	typedef const T&         const_reference;
	typedef T*               pointer;
	typedef T&               reference;
	typedef T                value_type;

	pointer       address   ( reference        ref ) const                           { return &ref; }
	const_pointer address   ( const_reference  ref ) const                           { return &ref; }
	pointer       allocate  ( size_type        n   , const_pointer *      hint = 0 ) {
		(void)hint;
		return reinterpret_cast< pointer >(btAlignedAlloc( sizeof(value_type) * n , Alignment ));
	}
	void          construct ( pointer          ptr , const value_type &   value    ) { new (ptr) value_type( value ); }
	void          deallocate( pointer          ptr ) {
		btAlignedFree( reinterpret_cast< void * >( ptr ) );
	}
	void          destroy   ( pointer          ptr )                                 { ptr->~value_type(); }
	

	template < typename O > struct rebind {
		typedef btAlignedAllocator< O , Alignment > other;
	};
	template < typename O >
	self_type & operator=( const btAlignedAllocator< O , Alignment > & ) { return *this; }

	friend bool operator==( const self_type & , const self_type & ) { return true; }
};



#endif //BT_ALIGNED_ALLOCATOR

