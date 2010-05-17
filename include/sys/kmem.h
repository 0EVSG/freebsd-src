/*****************************************************************************\
 *  Copyright (C) 2007-2010 Lawrence Livermore National Security, LLC.
 *  Copyright (C) 2007 The Regents of the University of California.
 *  Produced at Lawrence Livermore National Laboratory (cf, DISCLAIMER).
 *  Written by Brian Behlendorf <behlendorf1@llnl.gov>.
 *  UCRL-CODE-235197
 *
 *  This file is part of the SPL, Solaris Porting Layer.
 *  For details, see <http://github.com/behlendorf/spl/>.
 *
 *  The SPL is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  The SPL is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the SPL.  If not, see <http://www.gnu.org/licenses/>.
\*****************************************************************************/

#ifndef _SPL_KMEM_H
#define	_SPL_KMEM_H

#undef DEBUG_KMEM_UNIMPLEMENTED

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm_compat.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/hash.h>
#include <linux/ctype.h>
#include <asm/atomic.h>
#include <sys/types.h>
#include <sys/debug.h>
#include <sys/vmsystm.h>

/*
 * Memory allocation interfaces
 */
#define KM_SLEEP                        GFP_KERNEL
#define KM_NOSLEEP                      GFP_ATOMIC
#undef  KM_PANIC                        /* No linux analog */
#define KM_PUSHPAGE                     (KM_SLEEP | __GFP_HIGH)
#define KM_VMFLAGS                      GFP_LEVEL_MASK
#define KM_FLAGS                        __GFP_BITS_MASK

/*
 * Used internally, the kernel does not need to support this flag
 */
#ifndef __GFP_ZERO
# define __GFP_ZERO                     0x8000
#endif

/*
 * __GFP_NOFAIL looks like it will be removed from the kernel perhaps as
 * early as 2.6.32.  To avoid this issue when it occurs in upstream kernels
 * we retry the allocation here as long as it is not __GFP_WAIT (GFP_ATOMIC).
 * I would prefer the caller handle the failure case cleanly but we are
 * trying to emulate Solaris and those are not the Solaris semantics.
 */
static inline void *
kmalloc_nofail(size_t size, gfp_t flags)
{
	void *ptr;

	do {
		ptr = kmalloc(size, flags);
	} while (ptr == NULL && (flags & __GFP_WAIT));

	return ptr;
}

static inline void *
kzalloc_nofail(size_t size, gfp_t flags)
{
	void *ptr;

	do {
		ptr = kzalloc(size, flags);
	} while (ptr == NULL && (flags & __GFP_WAIT));

	return ptr;
}

#ifdef HAVE_KMALLOC_NODE
static inline void *
kmalloc_node_nofail(size_t size, gfp_t flags, int node)
{
	void *ptr;

	do {
		ptr = kmalloc_node(size, flags, node);
	} while (ptr == NULL && (flags & __GFP_WAIT));

	return ptr;
}
#endif /* HAVE_KMALLOC_NODE */

#ifdef DEBUG_KMEM
# ifdef HAVE_ATOMIC64_T

extern atomic64_t kmem_alloc_used;
extern unsigned long long kmem_alloc_max;
extern atomic64_t vmem_alloc_used;
extern unsigned long long vmem_alloc_max;

# define kmem_alloc_used_add(size)      atomic64_add(size, &kmem_alloc_used)
# define kmem_alloc_used_sub(size)      atomic64_sub(size, &kmem_alloc_used)
# define kmem_alloc_used_read()         atomic64_read(&kmem_alloc_used)
# define kmem_alloc_used_set(size)      atomic64_set(&kmem_alloc_used, size)
# define vmem_alloc_used_add(size)      atomic64_add(size, &vmem_alloc_used)
# define vmem_alloc_used_sub(size)      atomic64_sub(size, &vmem_alloc_used)
# define vmem_alloc_used_read()         atomic64_read(&vmem_alloc_used)
# define vmem_alloc_used_set(size)      atomic64_set(&vmem_alloc_used, size)

# else

extern atomic_t kmem_alloc_used;
extern unsigned long long kmem_alloc_max;
extern atomic_t vmem_alloc_used;
extern unsigned long long vmem_alloc_max;

# define kmem_alloc_used_add(size)      atomic_add(size, &kmem_alloc_used)
# define kmem_alloc_used_sub(size)      atomic_sub(size, &kmem_alloc_used)
# define kmem_alloc_used_read()         atomic_read(&kmem_alloc_used)
# define kmem_alloc_used_set(size)      atomic_set(&kmem_alloc_used, size)
# define vmem_alloc_used_add(size)      atomic_add(size, &vmem_alloc_used)
# define vmem_alloc_used_sub(size)      atomic_sub(size, &vmem_alloc_used)
# define vmem_alloc_used_read()         atomic_read(&vmem_alloc_used)
# define vmem_alloc_used_set(size)      atomic_set(&vmem_alloc_used, size)

# endif /* _LP64 */

# define kmem_alloc(size, flags)             __kmem_alloc((size), (flags), 0, 0)
# define kmem_zalloc(size, flags)            __kmem_alloc((size), ((flags) |  \
                                                 __GFP_ZERO), 0, 0)

/* The node alloc functions are only used by the SPL code itself */
# ifdef HAVE_KMALLOC_NODE
#  define kmem_alloc_node(size, flags, node) __kmem_alloc((size), (flags), 1, \
                                                 node)
# else
#  define kmem_alloc_node(size, flags, node) __kmem_alloc((size), (flags), 0, 0)
# endif

# define vmem_zalloc(size, flags)            vmem_alloc((size), ((flags) |    \
                                                 __GFP_ZERO))

# ifdef DEBUG_KMEM_TRACKING

extern void *kmem_alloc_track(size_t size, int flags, const char *func,
    int line, int node_alloc, int node);
extern void kmem_free_track(void *ptr, size_t size);
extern void *vmem_alloc_track(size_t size, int flags, const char *func,
    int line);
extern void vmem_free_track(void *ptr, size_t size);

#  define __kmem_alloc(size, flags, na, node) kmem_alloc_track((size),        \
                                                  (flags), __FUNCTION__,      \
                                                  __LINE__, (na), (node))
#  define kmem_free(ptr, size)                kmem_free_track((ptr), (size))
#  define vmem_alloc(size, flags)             vmem_alloc_track((size),        \
                                                  (flags),__FUNCTION__,       \
                                                  __LINE__)
#  define vmem_free(ptr, size)                vmem_free_track((ptr), (size))

# else /* DEBUG_KMEM_TRACKING */

extern void *kmem_alloc_debug(size_t size, int flags, const char *func,
    int line, int node_alloc, int node);
extern void kmem_free_debug(void *ptr, size_t size);
extern void *vmem_alloc_debug(size_t size, int flags, const char *func,
    int line);
extern void vmem_free_debug(void *ptr, size_t size);

#  define __kmem_alloc(size, flags, na, node) kmem_alloc_debug((size),        \
                                                  (flags), __FUNCTION__,      \
                                                  __LINE__, (na), (node))
#  define kmem_free(ptr, size)                kmem_free_debug((ptr), (size))
#  define vmem_alloc(size, flags)             vmem_alloc_debug((size),        \
                                                  (flags), __FUNCTION__,      \
                                                  __LINE__)
#  define vmem_free(ptr, size)                vmem_free_debug((ptr), (size))

# endif /* DEBUG_KMEM_TRACKING */

#else /* DEBUG_KMEM */

# define kmem_alloc(size, flags)              kmalloc_nofail((size), (flags))
# define kmem_zalloc(size, flags)             kzalloc_nofail((size), (flags))
# define kmem_free(ptr, size)                 ((void)(size), kfree(ptr))

# ifdef HAVE_KMALLOC_NODE
#  define kmem_alloc_node(size, flags, node)                                  \
          kmalloc_node_nofail((size), (flags), (node))
# else
#  define kmem_alloc_node(size, flags, node)                                  \
          kmalloc_nofail((size), (flags))
# endif

# define vmem_alloc(size, flags)              __vmalloc((size), ((flags) |    \
                                                  __GFP_HIGHMEM), PAGE_KERNEL)
# define vmem_zalloc(size, flags)                                             \
({                                                                            \
        void *_ptr_ = __vmalloc((size),((flags)|__GFP_HIGHMEM),PAGE_KERNEL);  \
        if (_ptr_)                                                            \
                memset(_ptr_, 0, (size));                                     \
        _ptr_;                                                                \
})
# define vmem_free(ptr, size)           ((void)(size), vfree(ptr))

#endif /* DEBUG_KMEM */

#ifdef DEBUG_KMEM_UNIMPLEMENTED
static __inline__ void *
kmem_alloc_tryhard(size_t size, size_t *alloc_size, int kmflags)
{
#error "kmem_alloc_tryhard() not implemented"
}
#endif /* DEBUG_KMEM_UNIMPLEMENTED */

/*
 * Slab allocation interfaces
 */
enum {
	KMC_BIT_NOTOUCH		= 0,	/* Don't update ages */
	KMC_BIT_NODEBUG		= 1,	/* Default behavior */
	KMC_BIT_NOMAGAZINE	= 2,	/* XXX: Unsupported */
	KMC_BIT_NOHASH		= 3,	/* XXX: Unsupported */
	KMC_BIT_QCACHE		= 4,	/* XXX: Unsupported */
	KMC_BIT_KMEM		= 5,	/* Use kmem cache */
	KMC_BIT_VMEM		= 6,	/* Use vmem cache */
	KMC_BIT_OFFSLAB		= 7,	/* Objects not on slab */
	KMC_BIT_REAPING		= 16,	/* Reaping in progress */
	KMC_BIT_DESTROY		= 17,	/* Destroy in progress */
};

#define KMC_NOTOUCH		(1 << KMC_BIT_NOTOUCH)
#define KMC_NODEBUG		(1 << KMC_BIT_NODEBUG)
#define KMC_NOMAGAZINE		(1 << KMC_BIT_NOMAGAZINE)
#define KMC_NOHASH		(1 << KMC_BIT_NOHASH)
#define KMC_QCACHE		(1 << KMC_BIT_QCACHE)
#define KMC_KMEM		(1 << KMC_BIT_KMEM)
#define KMC_VMEM		(1 << KMC_BIT_VMEM)
#define KMC_OFFSLAB		(1 << KMC_BIT_OFFSLAB)
#define KMC_REAPING		(1 << KMC_BIT_REAPING)
#define KMC_DESTROY		(1 << KMC_BIT_DESTROY)

#define KMC_REAP_CHUNK			INT_MAX
#define KMC_DEFAULT_SEEKS		1

#ifdef DEBUG_KMEM_UNIMPLEMENTED
static __inline__ void kmem_init(void) {
#error "kmem_init() not implemented"
}

static __inline__ void kmem_thread_init(void) {
#error "kmem_thread_init() not implemented"
}

static __inline__ void kmem_mp_init(void) {
#error "kmem_mp_init() not implemented"
}

static __inline__ void kmem_reap_idspace(void) {
#error "kmem_reap_idspace() not implemented"
}

static __inline__ size_t kmem_avail(void) {
#error "kmem_avail() not implemented"
}

static __inline__ size_t kmem_maxavail(void) {
#error "kmem_maxavail() not implemented"
}

static __inline__ uint64_t kmem_cache_stat(spl_kmem_cache_t *cache) {
#error "kmem_cache_stat() not implemented"
}
#endif /* DEBUG_KMEM_UNIMPLEMENTED */

/* XXX - Used by arc.c to adjust its memory footprint. We may want
 *       to use this hook in the future to adjust behavior based on
 *       debug levels.  For now it's safe to always return 0.
 */
static __inline__ int
kmem_debugging(void)
{
        return 0;
}

extern int kmem_set_warning(int flag);

extern struct list_head spl_kmem_cache_list;
extern struct rw_semaphore spl_kmem_cache_sem;

#define SKM_MAGIC			0x2e2e2e2e
#define SKO_MAGIC			0x20202020
#define SKS_MAGIC			0x22222222
#define SKC_MAGIC			0x2c2c2c2c

#define SPL_KMEM_CACHE_DELAY		15	/* Minimum slab release age */
#define SPL_KMEM_CACHE_REAP		0	/* Default reap everything */
#define SPL_KMEM_CACHE_OBJ_PER_SLAB	32	/* Target objects per slab */
#define SPL_KMEM_CACHE_OBJ_PER_SLAB_MIN	8	/* Minimum objects per slab */
#define SPL_KMEM_CACHE_ALIGN		8	/* Default object alignment */

typedef int (*spl_kmem_ctor_t)(void *, void *, int);
typedef void (*spl_kmem_dtor_t)(void *, void *);
typedef void (*spl_kmem_reclaim_t)(void *);

typedef struct spl_kmem_magazine {
	uint32_t		skm_magic;	/* Sanity magic */
	uint32_t		skm_avail;	/* Available objects */
	uint32_t		skm_size;	/* Magazine size */
	uint32_t		skm_refill;	/* Batch refill size */
	struct spl_kmem_cache	*skm_cache;	/* Owned by cache */
	struct delayed_work	skm_work;	/* Magazine reclaim work */
	unsigned long		skm_age;	/* Last cache access */
	void			*skm_objs[0];	/* Object pointers */
} spl_kmem_magazine_t;

typedef struct spl_kmem_obj {
        uint32_t		sko_magic;	/* Sanity magic */
	void			*sko_addr;	/* Buffer address */
	struct spl_kmem_slab	*sko_slab;	/* Owned by slab */
	struct list_head	sko_list;	/* Free object list linkage */
} spl_kmem_obj_t;

typedef struct spl_kmem_slab {
        uint32_t		sks_magic;	/* Sanity magic */
	uint32_t		sks_objs;	/* Objects per slab */
	struct spl_kmem_cache	*sks_cache;	/* Owned by cache */
	struct list_head	sks_list;	/* Slab list linkage */
	struct list_head	sks_free_list;	/* Free object list */
	unsigned long		sks_age;	/* Last modify jiffie */
	uint32_t		sks_ref;	/* Ref count used objects */
} spl_kmem_slab_t;

typedef struct spl_kmem_cache {
	uint32_t		skc_magic;	/* Sanity magic */
	uint32_t		skc_name_size;	/* Name length */
	char			*skc_name;	/* Name string */
	spl_kmem_magazine_t	*skc_mag[NR_CPUS]; /* Per-CPU warm cache */
	uint32_t		skc_mag_size;	/* Magazine size */
	uint32_t		skc_mag_refill;	/* Magazine refill count */
	spl_kmem_ctor_t		skc_ctor;	/* Constructor */
	spl_kmem_dtor_t		skc_dtor;	/* Destructor */
	spl_kmem_reclaim_t	skc_reclaim;	/* Reclaimator */
	void			*skc_private;	/* Private data */
	void			*skc_vmp;	/* Unused */
	unsigned long		skc_flags;	/* Flags */
	uint32_t		skc_obj_size;	/* Object size */
	uint32_t		skc_obj_align;	/* Object alignment */
	uint32_t		skc_slab_objs;	/* Objects per slab */
	uint32_t		skc_slab_size;	/* Slab size */
	uint32_t		skc_delay;	/* Slab reclaim interval */
	uint32_t		skc_reap;	/* Slab reclaim count */
	atomic_t		skc_ref;	/* Ref count callers */
	struct delayed_work	skc_work;	/* Slab reclaim work */
	struct list_head	skc_list;	/* List of caches linkage */
	struct list_head	skc_complete_list;/* Completely alloc'ed */
	struct list_head	skc_partial_list; /* Partially alloc'ed */
	spinlock_t		skc_lock;	/* Cache lock */
	uint64_t		skc_slab_fail;	/* Slab alloc failures */
	uint64_t		skc_slab_create;/* Slab creates */
	uint64_t		skc_slab_destroy;/* Slab destroys */
	uint64_t		skc_slab_total;	/* Slab total current */
	uint64_t		skc_slab_alloc;	/* Slab alloc current */
	uint64_t		skc_slab_max;	/* Slab max historic  */
	uint64_t		skc_obj_total;	/* Obj total current */
	uint64_t		skc_obj_alloc;	/* Obj alloc current */
	uint64_t		skc_obj_max;	/* Obj max historic */
} spl_kmem_cache_t;
#define kmem_cache_t		spl_kmem_cache_t

extern spl_kmem_cache_t *
spl_kmem_cache_create(char *name, size_t size, size_t align,
        spl_kmem_ctor_t ctor, spl_kmem_dtor_t dtor, spl_kmem_reclaim_t reclaim,
        void *priv, void *vmp, int flags);

extern void spl_kmem_cache_destroy(spl_kmem_cache_t *skc);
extern void *spl_kmem_cache_alloc(spl_kmem_cache_t *skc, int flags);
extern void spl_kmem_cache_free(spl_kmem_cache_t *skc, void *obj);
extern void spl_kmem_cache_reap_now(spl_kmem_cache_t *skc);
extern void spl_kmem_reap(void);

int spl_kmem_init_kallsyms_lookup(void);
int spl_kmem_init(void);
void spl_kmem_fini(void);

#define kmem_cache_create(name,size,align,ctor,dtor,rclm,priv,vmp,flags) \
        spl_kmem_cache_create(name,size,align,ctor,dtor,rclm,priv,vmp,flags)
#define kmem_cache_destroy(skc)		spl_kmem_cache_destroy(skc)
#define kmem_cache_alloc(skc, flags)	spl_kmem_cache_alloc(skc, flags)
#define kmem_cache_free(skc, obj)	spl_kmem_cache_free(skc, obj)
#define kmem_cache_reap_now(skc)	spl_kmem_cache_reap_now(skc)
#define kmem_reap()			spl_kmem_reap()
#define kmem_virt(ptr)			(((ptr) >= (void *)VMALLOC_START) && \
					 ((ptr) <  (void *)VMALLOC_END))

#endif	/* _SPL_KMEM_H */
