/*
 *   sljit emulator
 */

#include <stdint.h>
#include <byteswap.h>

#define MEMORY_SIZE (1*1024*1024)  // 1MB

// instruction formats (so far)
typedef enum {
    FMT_OP0 =         1,
    FMT_OP1 =         2,
    FMT_OP2 =         3,
    FMT_OP2U =        4,
    FMT_OP2R =        5,
    FMT_SI =          6,
    FMT_OP_SRC =      7,
    FMT_OP_DST =      8,
    FMT_FOP1 =        9,
    FMT_FOP2 =        10,
    FMT_FOP2R =       11,
    FMT_FSET32 =      12,
    FMT_FSET64 =      13,
    FMT_FCOPY =       14,
    FMT_LABEL =       15,
    FMT_JUMP =        16,
    FMT_CALL =        17,
    FMT_CMP =         18,
    FMT_FCMP =        19,
    FMT_IJUMP =       20,
    FMT_ICALL =       21,
    FMT_ENTER =       22,
    FMT_SET_CONTEXT = 23,
    FMT_RETURN =      24,
    FMT_RETURN_VOID = 25,
    FMT_SIMD_OP2 =    26,
    FMT_MCALL =       27,  // module:function call
    FMT_MJUMP =       28,  // module:name jump
    FMT_CONST =       29,
    FMT_OP_ADDR =     30,
    FMT_OP2CMPZ =     31,
    FMT_OP_FLAGS =    32,
    FMT_SELECT   =    33,
    FMT_FSELECT   =   34,
    FMT_MEM      =    35,
    FMT_MEM_UPDATE =  36,
    FMT_FMEM      =   37,
    FMT_FMEM_UPDATE = 38,
    FMT_SIMD_MOV    = 39,
    FMT_SIMD_REPLICATE = 40,
    FMT_SIMD_LANE_MOV  = 41,
    FMT_SIMD_LANE_REPLICATE = 42,
    FMT_SIMD_EXTEND  = 43,
    FMT_SIMD_SIGN = 44,
    FMT_ATOMIC_LOAD = 46,
    FMT_ATOMIC_STORE = 47,
    FMT_RETURN_TO = 48,
} sljitter_fmt;

// 64 bytes?
typedef struct {
    sljitter_fmt fmt;
    sljit_s32 op;
    sljit_s32 type;    
    sljit_s32 dst; sljit_sw dstw;
    sljit_s32 src1; sljit_sw src1w;
    sljit_s32 src2; sljit_sw src2w;
    sljit_s32 src3; sljit_sw src3w;
    union {
	sljit_f32 f32_val;
	sljit_f64 f64_val;
	sljit_sw  sw_val;
	sljit_uw  target;
	sljit_s32 arg_types;
	sljit_s32 lane_index;
    };
} sljitter_inst_t;


#define SWSIZE  (8*sizeof(sljit_sw))
#define UWSIZE  (8*sizeof(sljit_uw))
#define S32SIZE (8*sizeof(sljit_s32))
#define U32SIZE (8*sizeof(sljit_u32))

#define S32WMASK  0xffffffff
#define U32WMASK  0xffffffff

#define SHIFTM_32 (S32SIZE-1)
#define SHIFTM    (SWSIZE-1)

#define CAT_HELPER3(p,x,y) p ## x ## y
#define CAT3(p,x,y) CAT_HELPER3(p,x,y)

#define CAT_HELPER2(x,y) x ## y
#define CAT2(x,y) CAT_HELPER2(x,y)


#if __SIZEOF_LONG__ == 4
typedef int64_t sljit_sww;
typedef uint64_t sljit_uww;
#define SWMASK    0xffffffff
#define UWMASK    0xffffffff
#define bswap_uw(x) bswap_32((x))
#elif __SIZEOF_LONG__ == 8
typedef __int128_t sljit_sww;
typedef __uint128_t sljit_uww;
#define SWMASK    0xffffffffffffffff
#define UWMASK    0xffffffffffffffff
#define bswap_uw(x) bswap_64((x))
#endif

#define extend_16(x) (((x) & 0x8000) ? (((x) & 0xffff) | 0xffff0000) : ((x) & 0xffff))
#define extend_32(x) (((x) & 0x80000000) ? (((x) & 0xffffffff) | 0xffffffff00000000) : ((x) & 0xffffffff))


#define VSIZE 16

typedef struct {
    union {
	sljit_u8  vi8[VSIZE/(sizeof(sljit_u8))];
	sljit_u16 vi16lem[VSIZE/(sizeof(sljit_u16))];
	sljit_u32 vi32[VSIZE/(sizeof(sljit_u32))];
	sljit_f32 vf32[VSIZE/(sizeof(sljit_f32))];
	sljit_f64 vf64[VSIZE/(sizeof(sljit_f64))];
    };
} vec_t;

typedef union {
    sljit_sw  sw;
    sljit_uw  uw;
    int32_t   s32;
    int16_t   s16;    
    int8_t    s8;
    uint32_t  u32;
    uint16_t  u16;    
    uint8_t   u8;
    uintptr_t ptr;
} reg_t;

#define R(st,i)   ((st)->r[(i)].sw)
#define R32(st,i) ((st)->r[(i)].s32)
#define FR(st,i)  ((st)->fr[(i)])
#define VR(st,i)  ((st)->vr[(i)])

// flags field
#define FLAG_C    0x1   // carry
#define FLAG_V    0x2   // overflow
#define FLAG_Z    0x4   // zero
#define FLAG_N    0x8   // negative

#define FLAGS_ALL(st,fs) (((st)->flags & (fs)) == (fs))
#define FLAGS_ANY(st,fs) (((st)->flags & (fs)) != 0)
#define FLAGS_NONE(st,fs) (((st)->flags & (fs)) == 0)

typedef sljit_uw cpu_flags_t;

typedef struct {
    cpu_flags_t flags;
    reg_t      r[SLJIT_NUMBER_OF_REGISTERS];          // SLJIT_R(i)
    sljit_f64 fr[SLJIT_NUMBER_OF_FLOAT_REGISTERS];    // SLJIT_FR(i)
    vec_t     vr[SLJIT_NUMBER_OF_VECTOR_REGISTERS];   // SLKIT_VR(i)
    sljit_u8  mem[MEMORY_SIZE];
} cpu_state_t;

#define TP   u8
#define TYPE sljit_u8
#include "sljitEmulator.i"

#define TP   s8
#define TYPE sljit_s8
#include "sljitEmulator.i"

#define TP   u16
#define TYPE sljit_u16
#include "sljitEmulator.i"

#define TP   s16
#define TYPE sljit_s16
#include "sljitEmulator.i"

#define TP   u32
#define TYPE sljit_u32
#include "sljitEmulator.i"

#define TP   s32
#define TYPE sljit_s32
#include "sljitEmulator.i"

#define TP   sw
#define TYPE sljit_sw
#include "sljitEmulator.i"

#define TP   uw
#define TYPE sljit_uw
#include "sljitEmulator.i"

#define TP   ptr
#define TYPE uintptr_t
#include "sljitEmulator.i"

#define INC_SIZE(s) compiler->size += (s)
#define UNUSED(v) (void) v

static inline void set_flags_addc(cpu_flags_t* fp, sljit_uw a, sljit_uw b, sljit_uw c)
{
    (void) b;
    int f = ((c < a)) ? FLAG_C : 0;
    *fp = (*fp & ~(FLAG_C)) | f;
}

static inline void set_flags_add(cpu_flags_t* fp, sljit_uw a, sljit_uw b, sljit_uw c)
{
    int f = 0;
    if (c == 0) f |= FLAG_Z;
    if ((c >> (UWSIZE-1)) & 1) f |= FLAG_N;
    if ((c < a)) f |= FLAG_C;
    if ((((a ^ c) & (b ^ c)) >> (UWSIZE-1)) & 1) f |= FLAG_V;
    *fp = (*fp & ~(FLAG_C|FLAG_V|FLAG_Z|FLAG_N)) | f;
}

static inline void set_flags_subc(cpu_flags_t* fp, sljit_uw a, sljit_uw b, sljit_uw c)
{    
    (void) c;
    cpu_flags_t f = ((a < b)) ? FLAG_C : 0;
    *fp = (*fp & ~(FLAG_C)) | f;
}

static inline void set_flags_sub(cpu_flags_t* fp, sljit_uw a, sljit_uw b, sljit_uw c)
{
    cpu_flags_t f = 0;
    if (c == 0) f |= FLAG_Z;
    if ((c >> (UWSIZE-1)) & 1) f |= FLAG_N;
    if ((a < b)) f |= FLAG_C;
    if ((((a ^ b) & (a ^ c)) >> (UWSIZE-1)) & 1) f |= FLAG_V;
    *fp = (*fp & ~(FLAG_C|FLAG_V|FLAG_Z|FLAG_N)) | f;
}

// overflow?
static inline void set_flags_mul(cpu_flags_t* fp, sljit_sw a, sljit_sw b, sljit_sw c)
{
    sljit_sww cc =(sljit_sww) a * (sljit_sww) b;
    int f = (((sljit_sw) cc) != c) ? FLAG_V : 0;
    *fp = (*fp & ~(FLAG_V)) | f;
}

static inline void set_flags_logic(cpu_flags_t *fp, sljit_uw c)
{
    int f = (c==0) ? FLAG_C : 0;
    *fp = (*fp & ~(FLAG_C)) | f;
}

static inline void set_flags_zero(cpu_flags_t *fp, sljit_uw c)
{
    int f = (c==0) ? FLAG_Z : 0;
    *fp = (*fp & ~(FLAG_Z)) | f;
}


static inline void set_flags_addc32(cpu_flags_t* fp, sljit_u32 a, sljit_u32 b, sljit_u32 c)
{
    (void) b;
    int f = (c < a) ? FLAG_C : 0;
    *fp = (*fp & ~(FLAG_C)) | f;    
}

static inline void set_flags_add32(cpu_flags_t* fp, sljit_u32 a, sljit_u32 b, sljit_u32 c)
{
    int f = 0;
    if (c == 0) f |= FLAG_Z;
    if ((c >> (U32SIZE-1)) & 1) f |= FLAG_N;
    if ((c < a)) f |= FLAG_C;
    if ((((a ^ c) & (b ^ c)) >> (U32SIZE-1)) & 1) f |= FLAG_V;    
    *fp = (*fp & ~(FLAG_C|FLAG_V|FLAG_Z|FLAG_N)) | f;
}

static inline void set_flags_subc32(cpu_flags_t* fp, sljit_u32 a, sljit_u32 b, sljit_u32 c)
{
    (void) c;
    int f = (a < b) ? FLAG_C : 0;
    *fp = (*fp & ~(FLAG_C)) | f;    
}

static inline void set_flags_sub32(cpu_flags_t* fp, sljit_u32 a, sljit_u32 b, sljit_u32 c)
{
    int f = 0;
    if (c == 0) f |= FLAG_Z;
    if ((c >> (U32SIZE-1)) & 1) f |= FLAG_N;
    if ((a < b)) f |= FLAG_C;
    if ((((a ^ b) & (a ^ c)) >> (U32SIZE-1)) & 1) f |= FLAG_V;
    *fp = (*fp & ~(FLAG_C|FLAG_V|FLAG_Z|FLAG_N)) | f;    
}

// overflow?
static inline void set_flags_mul32(cpu_flags_t* fp, sljit_s32 a, sljit_s32 b, sljit_s32 c)
{
    int64_t cc = (int64_t)a * (int64_t)b;
    int f = (((sljit_s32)cc) != c) ? FLAG_V : 0;
    *fp = (*fp & ~(FLAG_V)) | f;
}


static inline void set_flags_logic32(cpu_flags_t *fp, sljit_u32 c)
{
    int f = (c==0) ? FLAG_C : 0;
    *fp = (*fp & ~(FLAG_C)) | f;
}

static inline void set_flags_zero32(cpu_flags_t *fp, sljit_u32 c)
{
    int f = (c==0) ? FLAG_Z : 0;
    *fp = (*fp & ~(FLAG_Z)) | f;
}


void slijit_emulate(sljitter_inst_t* prog, size_t size, cpu_state_t* st)
{
    int pc = 0;
    int r = 0;
next:
    if (pc >= (int)size)
	return; // error?
    switch(prog[pc].fmt) {
    case FMT_CMP: {
	// fixme: mask type and check SLJIT_32, rewriteable jump
	if (prog[pc].type & SLJIT_32) {
	    sljit_s32 a, b;
	    load_s32(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_s32(&b, prog[pc].src2, prog[pc].src2w, st);
	    set_flags_sub32(&st->flags, (sljit_u32) a, (sljit_u32) b,
			    ((sljit_u32)a-(sljit_u32)b));
	}
	else {
	    sljit_sw a, b;
	    load_sw(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_sw(&b, prog[pc].src2, prog[pc].src2w, st);
	    set_flags_sub(&st->flags, (sljit_uw) a, (sljit_uw) b,
			    ((sljit_uw)a-(sljit_uw)b));
	}
	switch(prog[pc].type & 0xff) {
	case SLJIT_EQUAL:
	    r = FLAGS_ALL(st, FLAG_Z); break;
	case SLJIT_NOT_EQUAL:
	    r = FLAGS_NONE(st, FLAG_Z); break;
	case SLJIT_LESS:
	    r = FLAGS_ALL(st, FLAG_C); break;
	case SLJIT_SIG_LESS:
	    r = FLAGS_ALL(st,FLAG_N) != FLAGS_ALL(st,FLAG_V); break;
	case SLJIT_GREATER_EQUAL:
	    r = FLAGS_NONE(st, FLAG_C); break;
	case SLJIT_SIG_GREATER_EQUAL:
	    r = FLAGS_ALL(st,FLAG_N) == FLAGS_ALL(st,FLAG_V); break;
	case SLJIT_GREATER:
	    r = FLAGS_NONE(st,FLAG_C|FLAG_Z); break;
	case SLJIT_SIG_GREATER:
	    r = FLAGS_NONE(st,FLAG_Z) &&
		(FLAGS_ALL(st,FLAG_N) == FLAGS_ALL(st,FLAG_V)); break;
	case SLJIT_LESS_EQUAL:
	    r = FLAGS_ANY(st, FLAG_C|FLAG_Z); break;	    
	case SLJIT_SIG_LESS_EQUAL:
	    r = (FLAGS_ALL(st,FLAG_N) != FLAGS_ALL(st,FLAG_V)) ||
		FLAGS_ALL(st,FLAG_Z); break;
	default: goto ignore;
	}	
	if (r) {
	    pc = prog[pc].target;
	    goto next;
	}
	break;	    
    }

    case FMT_JUMP:
	switch(prog[pc].type) {
	case SLJIT_EQUAL:
	    r = FLAGS_ALL(st,FLAG_Z); break;
	case SLJIT_NOT_EQUAL:
	    r = FLAGS_NONE(st,FLAG_Z); break;
	case SLJIT_LESS:
	    r = FLAGS_ALL(st,FLAG_C); break;
	case SLJIT_SIG_LESS:
	    r = (FLAGS_ALL(st,FLAG_N) != FLAGS_ALL(st,FLAG_V)); break;
	case SLJIT_GREATER_EQUAL:
	    r = FLAGS_ALL(st,FLAG_C); break;
	case SLJIT_SIG_GREATER_EQUAL:
	    r = FLAGS_ALL(st,FLAG_N) == FLAGS_ALL(st,FLAG_V); break;
	case SLJIT_GREATER:
	    r = FLAGS_NONE(st,FLAG_C|FLAG_Z); break;
	case SLJIT_SIG_GREATER:
	    r = FLAGS_NONE(st,FLAG_Z) &&
		(FLAGS_ALL(st,FLAG_N) == FLAGS_ALL(st,FLAG_V)); break;	    
	case SLJIT_LESS_EQUAL:
	    r = FLAGS_ANY(st, FLAG_C|FLAG_Z); break;	    
	case SLJIT_SIG_LESS_EQUAL:
	    r = (FLAGS_ALL(st,FLAG_N) != FLAGS_ALL(st,FLAG_V)) ||
		FLAGS_ALL(st,FLAG_Z); break;
	    
	case SLJIT_OVERFLOW:     r = FLAGS_ALL(st,FLAG_V); break;
	case SLJIT_NOT_OVERFLOW: r = FLAGS_NONE(st,FLAG_V); break;
	case SLJIT_CARRY:        r = FLAGS_ALL(st,FLAG_C); break;
	case SLJIT_NOT_CARRY:    r = FLAGS_NONE(st,FLAG_C); break;
	    // used?
	case SLJIT_ATOMIC_STORED:  break;
	case SLJIT_ATOMIC_NOT_STORED:  break;
	    // floatint point
	case SLJIT_F_EQUAL: break;
	case SLJIT_F_NOT_EQUAL: break;
	case SLJIT_F_LESS: break;
	case SLJIT_F_GREATER_EQUAL: break;
	case SLJIT_F_GREATER: break;
	case SLJIT_F_LESS_EQUAL: break;
	case SLJIT_UNORDERED: break;
	case SLJIT_ORDERED: break;
        /* Ordered / unordered floating point comparison types.
	   Note: each comparison type has an ordered and unordered form. Some
	   architectures supports only either of them (see: sljit_cmp_info). */
	case SLJIT_ORDERED_EQUAL: break;
	case SLJIT_UNORDERED_OR_NOT_EQUAL: break;
	case SLJIT_ORDERED_LESS: break;
	case SLJIT_UNORDERED_OR_GREATER_EQUAL: break;
	case SLJIT_ORDERED_GREATER: break;
	case SLJIT_UNORDERED_OR_LESS_EQUAL: break;
	case SLJIT_UNORDERED_OR_EQUAL: break;
	case SLJIT_ORDERED_NOT_EQUAL: break;
	case SLJIT_UNORDERED_OR_LESS: break;
	case SLJIT_ORDERED_GREATER_EQUAL: break;
	case SLJIT_UNORDERED_OR_GREATER: break;
	case SLJIT_ORDERED_LESS_EQUAL: break;
	case SLJIT_JUMP: r = 1; break;
	default: goto ignore;
	}
	if (r) {
	    pc = prog[pc].target;
	    goto next;
	}
	break;
	
    case FMT_OP0:
	switch(prog[pc].op) {
	case SLJIT_BREAKPOINT:
	    break;
	case SLJIT_NOP:
	    break;
	case SLJIT_LMUL_UW: {
	    sljit_uw  a = st->r[R(st,0)].uw;
	    sljit_uw  b = st->r[R(st,1)].uw;
	    sljit_uww c = (sljit_uww)a*(sljit_uww)b;
	    st->r[R(st,1)].uw = (c >> (UWSIZE-1));
	    st->r[R(st,0)].uw = (c & UWMASK);
	    break;
	}
	case SLJIT_LMUL_SW: {
	    sljit_sw  a = st->r[R(st,0)].sw;
	    sljit_sw  b = st->r[R(st,1)].sw;
	    sljit_sww c = (sljit_sww)a*(sljit_sww)b;
	    st->r[R(st,1)].sw = (c >> (SWSIZE-1));
	    st->r[R(st,0)].sw = (c & SWMASK);
	    break;
	}
	case SLJIT_DIVMOD_UW: {
	    sljit_uw  a = st->r[R(st,0)].uw;
	    sljit_uw  b = st->r[R(st,1)].uw;
	    st->r[R(st,0)].uw = a / b;
	    st->r[R(st,1)].uw = a % b;	    
	    break;
	}
	case SLJIT_DIVMOD_U32: {
	    sljit_u32  a = st->r[R(st,0)].u32;
	    sljit_u32  b = st->r[R(st,1)].u32;
	    st->r[R(st,0)].u32 = a / b;
	    st->r[R(st,1)].u32 = a % b;	    	    
	    break;
	}
	case SLJIT_DIVMOD_SW: {
	    sljit_sw  a = st->r[R(st,0)].sw;
	    sljit_sw  b = st->r[R(st,1)].sw;
	    st->r[R(st,0)].sw = a / b;
	    st->r[R(st,1)].sw = a % b;	    	    
	    break;
	}
	case SLJIT_DIVMOD_S32: {
	    sljit_s32  a = st->r[R(st,0)].s32;
	    sljit_s32  b = st->r[R(st,1)].s32;
	    st->r[R(st,0)].s32 = a / b;
	    st->r[R(st,1)].s32 = a % b;	    	    
	    break;	    
	}
	case SLJIT_DIV_UW: {
	    sljit_uw  a = st->r[R(st,0)].uw;
	    sljit_uw  b = st->r[R(st,1)].uw;	    
	    st->r[R(st,0)].uw = a / b;
	    break;
	}
	case SLJIT_DIV_U32: {
	    sljit_u32  a = st->r[R(st,0)].u32;
	    sljit_u32  b = st->r[R(st,1)].u32;
	    st->r[R(st,0)].u32 = a / b;	    
	    break;
	}
	case SLJIT_DIV_SW: {
	    sljit_sw  a = st->r[R(st,0)].sw;
	    sljit_sw  b = st->r[R(st,1)].sw;
	    st->r[R(st,0)].sw = a / b;
	    break;
	}
	case SLJIT_DIV_S32: {
	    sljit_s32  a = st->r[R(st,0)].s32;
	    sljit_s32  b = st->r[R(st,1)].s32;
	    st->r[R(st,0)].s32 = a / b;	    
	    break;
	}
	case SLJIT_MEMORY_BARRIER:
	    break;
	case SLJIT_ENDBR:
	    break;
	case SLJIT_SKIP_FRAMES_BEFORE_RETURN:
	    break;
	default:
	    goto ignore;
	}
	break;
	
    case FMT_OP1:
	switch(prog[pc].op) {
	case SLJIT_MOV: {
	    sljit_sw a;
	    load_sw(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_sw(a, prog[pc].dst, prog[pc].dstw, st);
	    break;
	}
	case SLJIT_MOV32: {
	    sljit_s32 a;
	    load_s32(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_s32(a, prog[pc].dst, prog[pc].dstw, st);
	    break;
	}	    
	case SLJIT_MOV_U8:
	case SLJIT_MOV32_U8: {
	    uint8_t a;
	    load_u8(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_u8(a, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}
	case SLJIT_MOV_S8:
	case SLJIT_MOV32_S8: {
	    int8_t a;
	    load_s8(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_s8(a, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}	    
	case SLJIT_MOV_U16:
	case SLJIT_MOV32_U16: {
	    uint16_t a;
	    load_u16(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_u16(a, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}	    
	case SLJIT_MOV_S16:
	case SLJIT_MOV32_S16: {
	    int16_t a;
	    load_s16(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_s16(a, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}
	    
	case SLJIT_MOV_U32: {
	    uint32_t a;
	    load_u32(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_u32(a, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}

	case SLJIT_MOV_S32: {
	    int32_t a;
	    load_s32(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_s32(a, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}
	case SLJIT_MOV_P: {
	    uintptr_t p;
	    load_ptr(&p, prog[pc].src1, prog[pc].src1w, st);
	    store_ptr(p, prog[pc].dst, prog[pc].dstw, st);
	    break;	    	    
	}

	case SLJIT_CLZ: {  // count leading zero's
	    sljit_uw a;
	    load_uw(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_uw(__builtin_clz(a), prog[pc].dst, prog[pc].dstw, st);
	    break;
	}	    	    
	    
	case SLJIT_CLZ32: {  // count leading zero's
	    sljit_u32 a;
	    load_u32(&a, prog[pc].src1,prog[pc].src1w,st);
	    store_u32(__builtin_clz(a), prog[pc].dst,prog[pc].dstw,st);
	    break;
	}
	    
	case SLJIT_CTZ: {  // count trailing zero's
	    sljit_uw a;
	    load_uw(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_uw(__builtin_ctz(a), prog[pc].dst, prog[pc].dstw, st);
	    break;
	}
	    
	case SLJIT_CTZ32: {  // count leading zero's
	    sljit_u32 a;
	    load_u32(&a, prog[pc].src1,prog[pc].src1w,st);
	    store_u32(__builtin_ctz(a), prog[pc].dst,prog[pc].dstw,st);
	    break;
	}
	    
	case SLJIT_REV: { // reverse bytes
	    sljit_uw a;
	    load_uw(&a, prog[pc].src1,prog[pc].src1w,st);
	    store_uw(bswap_uw(a), prog[pc].dst,prog[pc].dstw,st);
	    break;
	}
	    
	case SLJIT_REV32: {
	    sljit_u32 a;
	    load_u32(&a, prog[pc].src1,prog[pc].src1w,st);
	    store_u32(bswap_32(a), prog[pc].dst,prog[pc].dstw,st);
	    break;
	}
	    
	case SLJIT_REV_U16:
	case SLJIT_REV32_U16: {
	    sljit_u16 a;
	    load_u16(&a, prog[pc].src1,prog[pc].src1w,st);
	    store_u16(bswap_16(a), prog[pc].dst,prog[pc].dstw,st);
	    break;	    
	}
	    
	case SLJIT_REV_S16:
	case SLJIT_REV32_S16: {
	    sljit_s16 a;
	    sljit_sw b;
	    load_s16(&a, prog[pc].src1,prog[pc].src1w,st);
	    b = extend_16(bswap_16(a));
	    store_sw(b, prog[pc].dst,prog[pc].dstw,st);
	    break;    
	}
	    
	case SLJIT_REV_U32:  {
	    sljit_u32 a;
	    sljit_uw b;
	    load_u32(&a, prog[pc].src1,prog[pc].src1w,st);
	    b = bswap_32(a);
	    store_uw(b, prog[pc].dst,prog[pc].dstw,st);
	    break;	    
	}
	case SLJIT_REV_S32:  {
	    sljit_s32 a;
	    sljit_sw b;
	    load_s32(&a, prog[pc].src1,prog[pc].src1w,st);
	    b = extend_32(bswap_32(a));
	    store_sw(b, prog[pc].dst,prog[pc].dstw,st);
	    break;	    
	}
	    
	default: goto ignore;
	}
	break;
	
    case FMT_OP2U:  // use op2
	prog[pc].dst = 0; // signal no storage dst=0
	// fall-through 
    case FMT_OP2:
	if (prog[pc].op & SLJIT_32) {
	    sljit_s32 a, b, c;
	    load_s32(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_s32(&b, prog[pc].src2, prog[pc].src2w, st);	
	    switch(prog[pc].op & ~SLJIT_32) {
	    case SLJIT_ADD:
		c = a+b;
		set_flags_add32(&st->flags,
				(sljit_u32) a, (sljit_u32) b, (sljit_u32) c);
		break;
	    case SLJIT_ADDC:
		c = a+b;
		set_flags_addc32(&st->flags,
				 (sljit_u32)a, (sljit_u32)b, (sljit_u32) c);
		break;
	    case SLJIT_SUB:
		c = a-b;
		set_flags_sub32(&st->flags,
				(sljit_u32) a, (sljit_u32) b, (sljit_u32) c);
		break;
	    case SLJIT_SUBC:
		c = a-b;
		set_flags_subc32(&st->flags,
				 (sljit_u32) a, (sljit_u32) b, (sljit_u32) c);
		break;
	    case SLJIT_MUL:
		c = a*b;
		set_flags_mul32(&st->flags, a, b, c);
		break;
	    case SLJIT_AND:
		c = a&b;
		set_flags_logic32(&st->flags, (sljit_u32) c);
		break;
	    case SLJIT_OR:
		c = a|b;
		set_flags_logic32(&st->flags, (sljit_u32) c);
		break;
	    case SLJIT_XOR:
		c = a^b;		
		set_flags_logic32(&st->flags, (sljit_u32) c);
		break;		
	    case SLJIT_SHL:
		c = a<<b;
		set_flags_zero32(&st->flags, (sljit_u32) c);
		break;
	    case SLJIT_MSHL:
		c = a<<(b&SHIFTM);
		set_flags_zero32(&st->flags, (sljit_u32) c);		
		break;
	    case SLJIT_LSHR:
		c = ((sljit_u32)a)>>b;
		set_flags_zero32(&st->flags, (sljit_u32) c);		
		break;
	    case SLJIT_MLSHR:
		c = ((sljit_u32)a)>>(b&SHIFTM);
		set_flags_zero32(&st->flags, (sljit_u32) c);		
		break;
	    case SLJIT_ASHR:
		c = a>>b;
		set_flags_zero32(&st->flags, (sljit_u32) c);
		break;
	    case SLJIT_MASHR:
		c = a>>(b&SHIFTM);
		set_flags_zero32(&st->flags, (sljit_u32) c);
		break;	    
	    case SLJIT_ROTL:
		c = (a<<(b&SHIFTM)) | (a>>(SWSIZE-(b&SHIFTM)));
		break;
	    case SLJIT_ROTR:
		c = (a>>(b&SHIFTM)) | (a<<(SWSIZE-(b&SHIFTM)));
		break;
	    default:
		goto ignore;
	    }
	    store_s32(c, prog[pc].dst, prog[pc].dstw, st);
	}
	else {
	    sljit_sw a, b, c;
	    load_sw(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_sw(&b, prog[pc].src2, prog[pc].src2w, st);
	    switch(prog[pc].op) {
	    case SLJIT_ADD:
		c = a+b;
		set_flags_add(&st->flags, (sljit_uw) a, (sljit_uw) b, (sljit_uw) c);
		break;
	    case SLJIT_ADDC:
		c = a+b;
		set_flags_addc(&st->flags, (sljit_uw) a, (sljit_uw) b, (sljit_uw) c);		
		break;
	    case SLJIT_SUB:
		c = a-b;
		set_flags_sub(&st->flags, (sljit_uw) a, (sljit_uw) b, (sljit_uw) c);
		break;
	    case SLJIT_SUBC:
		c = a-b;
		set_flags_subc(&st->flags, (sljit_uw) a, (sljit_uw) b, (sljit_uw) c);
		break;
	    case SLJIT_MUL:
		c = a*b;
		set_flags_mul(&st->flags, a, b, c);
		break;
	    case SLJIT_AND:
		c = a&b;
		set_flags_logic(&st->flags, (sljit_u32) c);
		break;
	    case SLJIT_OR:
		c = a|b;
		set_flags_logic(&st->flags, (sljit_u32) c);		
		break;
	    case SLJIT_XOR:
		c = a^b;		
		set_flags_logic(&st->flags, (sljit_u32) c);
		break;
	    case SLJIT_SHL:
		c = a<<b;
		set_flags_zero(&st->flags, (sljit_u32) c);
		break;
	    case SLJIT_MSHL:
		c = a<<(b&SHIFTM);
		set_flags_zero(&st->flags, (sljit_u32) c);
		break;
	    case SLJIT_LSHR:
		c = ((sljit_u32)a)>>b;
		set_flags_zero(&st->flags, (sljit_u32) c);		
		break;
	    case SLJIT_MLSHR:
		c = ((sljit_u32)a)>>(b&SHIFTM);
		set_flags_zero(&st->flags, (sljit_u32) c);
		break;
	    case SLJIT_ASHR:
		c = a>>b;
		set_flags_zero(&st->flags, (sljit_u32) c);
		break;
	    case SLJIT_MASHR:
		c = a>>(b&SHIFTM);
		set_flags_zero(&st->flags, (sljit_u32) c);
		break;
	    case SLJIT_ROTL:
		c = (a<<(b&SHIFTM)) | (a>>(SWSIZE-(b&SHIFTM)));
		// randomize flags option?
		break;
	    case SLJIT_ROTR:
		c = (a>>(b&SHIFTM)) | (a<<(SWSIZE-(b&SHIFTM)));
		// randomize flags option?
		break;
	    default: goto ignore;		
	    }
	    if (prog[pc].dst != 0) // check for OP2U
		store_sw(c, prog[pc].dst, prog[pc].dstw, st);
	}
	break;
	
    case FMT_OP2R: {
	if (prog[pc].op & SLJIT_32) {
	    sljit_s32 a, b, c;
	    load_s32(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_s32(&b, prog[pc].src2, prog[pc].src2w, st);
	    load_s32(&c, prog[pc].dst, 0, st);
	    switch(prog[pc].op &  ~SLJIT_32) {
	    case SLJIT_MULADD:
		c = a*b + c;
		break;
	    default: goto ignore;
	    }
	    store_s32(c, prog[pc].dst & 0x7f, 0, st);  
	}
	else {
	    sljit_sw a, b, c;
	    load_sw(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_sw(&b, prog[pc].src2, prog[pc].src2w, st);
	    load_sw(&c, prog[pc].dst, 0, st);
	    switch(prog[pc].op) {
	    case SLJIT_MULADD:
		c = a*b + c;
		break;
	    default: goto ignore;
	    }
	    store_sw(c, prog[pc].dst & 0x7f, 0, st);	    
	}

	break;
    }
    default:
	break;
    }
ignore:
    pc++;
    goto next;
}

// FIXME: !!!!
extern void* ensure_buf(struct sljit_compiler *compiler, sljit_uw size);
extern void* ensure_abuf(struct sljit_compiler *compiler, sljit_uw size);
extern void set_label(struct sljit_label *label,
		      struct sljit_compiler *compiler);
extern void set_const(struct sljit_const *const_,
		      struct sljit_compiler *compiler);
extern void set_mov_addr(struct sljit_jump *jump, struct sljit_compiler *compiler, sljit_uw offset);

extern void sljit_set_label(struct sljit_jump *jump, struct sljit_label* label);
extern void sljit_set_target(struct sljit_jump *jump, sljit_uw target);

SLJIT_API_FUNC_ATTRIBUTE const char* sljit_get_platform_name(void)
{
    return "emu" SLJIT_CPUINFO;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_has_cpu_feature(sljit_s32 feature_type)
{
    switch (feature_type) {
    case SLJIT_HAS_FPU: return 1;
    case SLJIT_HAS_VIRTUAL_REGISTERS: return 1;
    case SLJIT_HAS_CLZ: return 1;
    case SLJIT_HAS_CTZ: return 1;
    case SLJIT_HAS_CMOV: return 1;
    case SLJIT_HAS_REV:
    case SLJIT_HAS_ROT:
    case SLJIT_HAS_PREFETCH:
    case SLJIT_HAS_COPY_F32:
    case SLJIT_HAS_COPY_F64:
    case SLJIT_HAS_ATOMIC:
    case SLJIT_HAS_MEMORY_BARRIER:
    case SLJIT_HAS_SIMD:	
	return 1;
    default:
	return 0;
    }
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_enter(struct sljit_compiler *compiler,
						    sljit_s32 options,
						    sljit_s32 arg_types,
						    sljit_s32 scratches,
						    sljit_s32 saveds,
						    sljit_s32 local_size)
{
    // FIXME!
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_set_context(struct sljit_compiler *compiler,
						     sljit_s32 options,
						     sljit_s32 arg_types,
						     sljit_s32 scratches,
						     sljit_s32 saveds,
						     sljit_s32 local_size)
{
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_op0(struct sljit_compiler *compiler, sljit_s32 op)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_OP0;  
    ip->op = op;
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_op1(struct sljit_compiler *compiler, sljit_s32 op,
			 sljit_s32 dst, sljit_sw dstw,
			 sljit_s32 src, sljit_sw srcw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));    
    ip->fmt = FMT_OP1;
    ip->op = op;
    ip->dst = dst;
    ip->dstw = dstw;
    ip->src1 = src;
    ip->src1w = srcw;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_op2(struct sljit_compiler *compiler, sljit_s32 op,
			 sljit_s32 dst, sljit_sw dstw,
			 sljit_s32 src1, sljit_sw src1w,
			 sljit_s32 src2, sljit_sw src2w)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_OP2;
    ip->op = op;
    ip->dst = dst;
    ip->dstw = dstw;
    ip->src1 = src1;
    ip->src1w = src1w;
    ip->src2 = src2;
    ip->src2w = src2w;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_op2u(struct sljit_compiler *compiler, sljit_s32 op,
			  sljit_s32 src1, sljit_sw src1w,
			  sljit_s32 src2, sljit_sw src2w)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_OP2;
    ip->op = op;
    ip->dst = 0;
    ip->dstw = 0;
    ip->src1 = src1;
    ip->src1w = src1w;
    ip->src2 = src2;
    ip->src2w = src2w;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;    
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_op2r(struct sljit_compiler *compiler, sljit_s32 op,
			  sljit_s32 dst_reg,
			  sljit_s32 src1, sljit_sw src1w,
			  sljit_s32 src2, sljit_sw src2w)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_OP2R;
    ip->op = op;
    ip->dst = dst_reg;
    ip->src1 = src1;
    ip->src1w = src1w;
    ip->src2 = src2;
    ip->src2w = src2w;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_shift_into(struct sljit_compiler *compiler, sljit_s32 op,
				sljit_s32 dst_reg,
				sljit_s32 src1_reg,
				sljit_s32 src2_reg,
				sljit_s32 src3, sljit_sw src3w)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_SI;
    ip->op = op;
    ip->dst = dst_reg;
    ip->src1 = src1_reg;
    ip->src2 = src2_reg;
    ip->src3 = src3;
    ip->src3w = src3w;
    INC_SIZE(1);
    return SLJIT_SUCCESS;    
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_op_src(struct sljit_compiler *compiler, sljit_s32 op,
			    sljit_s32 src, sljit_sw srcw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_OP_SRC;
    ip->op = op;
    ip->src1 = src;
    ip->src1w = srcw;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;    
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_op_dst(struct sljit_compiler *compiler,
			    sljit_s32 op,
			    sljit_s32 dst, sljit_sw dstw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_OP_DST;
    ip->op = op;
    ip->dst = dst;
    ip->dstw = dstw;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;        
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_fop1(struct sljit_compiler *compiler,
			  sljit_s32 op,
			  sljit_s32 dst, sljit_sw dstw,
			  sljit_s32 src, sljit_sw srcw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));    
    ip->fmt = FMT_FOP1;
    ip->op = op;
    ip->dst = dst;
    ip->dstw = dstw;
    ip->src1 = src;
    ip->src1w = srcw;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;    
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_fop2(struct sljit_compiler *compiler,
			  sljit_s32 op,
			  sljit_s32 dst, sljit_sw dstw,
			  sljit_s32 src1, sljit_sw src1w,
			  sljit_s32 src2, sljit_sw src2w)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_FOP2;
    ip->op = op;
    ip->dst = dst;
    ip->dstw = dstw;
    ip->src1 = src1;
    ip->src1w = src1w;
    ip->src2 = src2;
    ip->src2w = src2w;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;    
}

/*
sljit_s32 sljit_emit_fop2r(struct sljit_compiler *compiler, sljit_s32 op,
			   sljit_s32 dst_freg,
			   sljit_s32 src1, sljit_sw src1w,
			   sljit_s32 src2, sljit_sw src2w)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_FOP2R;
    ip->op = op;
    ip->dst = dst_freg;
    ip->src1 = src1;
    ip->src1w = src1w;
    ip->src2 = src2;
    ip->src2w = src2w;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;        
}
*/

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_fset32(struct sljit_compiler *compiler,
			    sljit_s32 freg, sljit_f32 value)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_FSET32;
    ip->dst = freg;
    ip->f32_val = value;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;            
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_fset64(struct sljit_compiler *compiler,
			    sljit_s32 freg, sljit_f64 value)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_FSET64;
    ip->dst = freg;
    ip->f64_val = value;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;                
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_fcopy(struct sljit_compiler *compiler,
			   sljit_s32 op,
			   sljit_s32 freg, sljit_s32 reg)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_FCOPY;
    ip->op  = op;
    ip->dst = freg;
    ip->src1 = reg;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;    
}


SLJIT_API_FUNC_ATTRIBUTE struct sljit_label* sljit_emit_label(struct sljit_compiler *compiler)
{
    struct sljit_label* label;
    
    if (compiler->last_label && (compiler->last_label->size == compiler->size))
	return compiler->last_label;
    label = (struct sljit_label*)ensure_abuf(compiler,sizeof(struct sljit_label));
    set_label(label, compiler);
    return label;
}

SLJIT_API_FUNC_ATTRIBUTE struct sljit_label* sljit_emit_aligned_label(struct sljit_compiler *compiler,
					     sljit_s32 alignment,
					     struct sljit_read_only_buffer *buffers)
{
    UNUSED(alignment);
    UNUSED(buffers);
    struct sljit_label* label;
    
    if (compiler->last_label && (compiler->last_label->size == compiler->size))
	return compiler->last_label;
    label = (struct sljit_label*)ensure_abuf(compiler,sizeof(struct sljit_label));
    set_label(label, compiler);
    return label;    
}


struct sljit_jump* sljit_emit_jump(struct sljit_compiler *compiler, sljit_s32 type)
{
    sljitter_inst_t* ip;
    struct sljit_jump* jump;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_JUMP;
    ip->type = type;
    jump = (struct sljit_jump*)ensure_abuf(compiler, sizeof(struct sljit_jump));
    jump->addr = compiler->size;
    INC_SIZE(1);
    return jump;    
}


SLJIT_API_FUNC_ATTRIBUTE struct sljit_jump* sljit_emit_call(struct sljit_compiler *compiler,
				   sljit_s32 type, sljit_s32 arg_types)
{
    sljitter_inst_t* ip;
    struct sljit_jump* jump;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_CALL;
    ip->type = type;
    ip->arg_types = arg_types;
    jump = (struct sljit_jump*)ensure_abuf(compiler, sizeof(struct sljit_jump));
    jump->addr = compiler->size;
    INC_SIZE(1);
    return jump;    
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_return_void(struct sljit_compiler *compiler)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_RETURN_VOID;
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_return_to(struct sljit_compiler *compiler,
	sljit_s32 src, sljit_sw srcw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_RETURN_TO;
    ip->src1 = src;
    ip->src1w = srcw;
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}

/*
struct sljit_jump* sljit_emit_cmp(struct sljit_compiler *compiler,
				  sljit_s32 type,
				  sljit_s32 src1, sljit_sw src1w,
				  sljit_s32 src2, sljit_sw src2w)
{
    sljitter_inst_t* ip;
    struct sljit_jump* jump;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_CMP;
    ip->type = type;
    ip->src1 = src1;
    ip->src1w = src1w;
    ip->src2 = src2;
    ip->src2w = src2w;
    jump = (struct sljit_jump*)ensure_abuf(compiler, sizeof(struct sljit_jump));
    jump->addr = compiler->size;    
    INC_SIZE(1);
    return jump;
}
*/

/*
struct sljit_jump* sljit_emit_fcmp(struct sljit_compiler *compiler,
				   sljit_s32 type,
				   sljit_s32 src1, sljit_sw src1w,
				   sljit_s32 src2, sljit_sw src2w)
{
    sljitter_inst_t* ip;
    struct sljit_jump* jump;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_FCMP;
    ip->type = type;
    ip->src1 = src1;
    ip->src1w = src1w;
    ip->src2 = src2;
    ip->src2w = src2w;
    jump = (struct sljit_jump*)ensure_abuf(compiler, sizeof(struct sljit_jump));
    jump->addr = compiler->size;
    INC_SIZE(1);
    return jump;    
}
*/

/*
struct sljit_jump* sljit_emit_op2cmpz(struct sljit_compiler *compiler,
				      sljit_s32 op,
				      sljit_s32 dst, sljit_sw dstw,
				      sljit_s32 src1, sljit_sw src1w,
				      sljit_s32 src2, sljit_sw src2w)
{
    sljitter_inst_t* ip;
    struct sljit_jump* jump;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_OP2CMPZ;
    ip->op = op;
    ip->dst = dst;
    ip->dstw = dstw;    
    ip->src1 = src1;
    ip->src1w = src1w;
    ip->src2 = src2;
    ip->src2w = src2w;
    jump = (struct sljit_jump*)ensure_abuf(compiler, sizeof(struct sljit_jump));
    jump->addr = compiler->size;
    INC_SIZE(1);
    return jump;    
}
*/

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_ijump(struct sljit_compiler *compiler,
			   sljit_s32 type, sljit_s32 src, sljit_sw srcw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_IJUMP;
    ip->type = type;
    ip->src1 = src;
    ip->src1w = srcw;
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_icall(struct sljit_compiler *compiler,
			   sljit_s32 type, sljit_s32 arg_types,
			   sljit_s32 src, sljit_sw srcw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_ICALL;
    ip->type = type;
    ip->src1 = src;
    ip->src1w = srcw;
    ip->arg_types = arg_types;
    INC_SIZE(1);
    return SLJIT_SUCCESS;    
}

sljit_s32 sljit_emit_op_flags(struct sljit_compiler *compiler, sljit_s32 op,
			      sljit_s32 dst, sljit_sw dstw,
			      sljit_s32 type)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_OP_FLAGS;
    ip->op = op;
    ip->type = type;    
    ip->dst = dst;
    ip->dstw = dstw;
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_select(struct sljit_compiler *compiler,
			    sljit_s32 type,
			    sljit_s32 dst_reg,
			    sljit_s32 src1, sljit_sw src1w,
			    sljit_s32 src2_reg)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_SELECT;
    ip->type = type;
    ip->dst = dst_reg;
    ip->src1 = src1;
    ip->src1w = src1w;
    ip->src2 = src2_reg;
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_fselect(struct sljit_compiler *compiler,
			     sljit_s32 type,
			     sljit_s32 dst_freg,
			     sljit_s32 src1, sljit_sw src1w,
			     sljit_s32 src2_freg)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_FSELECT;
    ip->type = type;
    ip->dst = dst_freg;
    ip->src1 = src1;
    ip->src1w = src1w;
    ip->src2 = src2_freg;
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_mem(struct sljit_compiler *compiler,
			 sljit_s32 type,
			 sljit_s32 reg,
			 sljit_s32 mem, sljit_sw memw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_MEM;
    ip->type = type;
    ip->dst = reg;
    ip->src1 = mem;
    ip->src1w = memw;
    INC_SIZE(1);
    return SLJIT_SUCCESS;    
}

/*
SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_mem_update(struct sljit_compiler *compiler,
				sljit_s32 type,
				sljit_s32 reg,
				sljit_s32 mem, sljit_sw memw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_MEM_UPDATE;
    ip->type = type;
    ip->dst = reg;
    ip->src1 = mem;
    ip->src1w = memw;
    INC_SIZE(1);
    return SLJIT_SUCCESS;        
}
*/

/*
SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_fmem(struct sljit_compiler *compiler,
			  sljit_s32 type,
			  sljit_s32 freg,
			  sljit_s32 mem, sljit_sw memw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_FMEM;
    ip->type = type;
    ip->dst = freg;
    ip->src1 = mem;
    ip->src1w = memw;
    INC_SIZE(1);
    return SLJIT_SUCCESS;        
}
*/

/*
SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_fmem_update(struct sljit_compiler *compiler,
				 sljit_s32 type,
				 sljit_s32 freg,
				 sljit_s32 mem, sljit_sw memw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_FMEM_UPDATE;
    ip->type = type;
    ip->dst = freg;
    ip->src1 = mem;
    ip->src1w = memw;
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}
*/

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_simd_mov(struct sljit_compiler *compiler,
			      sljit_s32 type,
			      sljit_s32 vreg,
			      sljit_s32 srcdst, sljit_sw srcdstw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_SIMD_MOV;
    ip->type = type;
    ip->dst = vreg;
    ip->src1 = srcdst;
    ip->src1w = srcdstw;
    INC_SIZE(1);
    return SLJIT_SUCCESS;                
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_simd_replicate(struct sljit_compiler *compiler,
				    sljit_s32 type,
				    sljit_s32 vreg,
				    sljit_s32 src, sljit_sw srcw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_SIMD_REPLICATE;
    ip->type = type;
    ip->dst = vreg;
    ip->src1 = src;
    ip->src1w = srcw;
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_simd_lane_mov(struct sljit_compiler *compiler,
				   sljit_s32 type,
				   sljit_s32 vreg, sljit_s32 lane_index,
				   sljit_s32 srcdst, sljit_sw srcdstw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_SIMD_LANE_MOV;
    ip->type = type;
    ip->dst = vreg;
    ip->src1 = srcdst;
    ip->src1w = srcdstw;
    ip->lane_index = lane_index;
    INC_SIZE(1);
    return SLJIT_SUCCESS;    
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_simd_lane_replicate(struct sljit_compiler *compiler,
					 sljit_s32 type,
					 sljit_s32 vreg,
					 sljit_s32 src,
					 sljit_s32 src_lane_index)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_SIMD_LANE_REPLICATE;
    ip->type = type;
    ip->dst = vreg;
    ip->src1 = src;
    ip->lane_index = src_lane_index;
    INC_SIZE(1);
    return SLJIT_SUCCESS;        
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_simd_extend(struct sljit_compiler *compiler,
				 sljit_s32 type,
				 sljit_s32 vreg,
				 sljit_s32 src, sljit_sw srcw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_SIMD_EXTEND;
    ip->type = type;
    ip->dst = vreg;
    ip->src1 = src;
    ip->src1w = srcw;    
    INC_SIZE(1);
    return SLJIT_SUCCESS;            
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_simd_sign(struct sljit_compiler *compiler,
			       sljit_s32 type,
			       sljit_s32 vreg,
			       sljit_s32 dst, sljit_sw dstw)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_SIMD_SIGN;
    ip->type = type;
    ip->src1 = vreg;
    ip->dst = dst;
    ip->dstw = dstw;
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_simd_op2(struct sljit_compiler *compiler,
			      sljit_s32 type,
			      sljit_s32 dst_vreg,
			      sljit_s32 src1_vreg,
			      sljit_s32 src2, sljit_sw src2w)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_SIMD_OP2;
    ip->type = type;
    ip->dst = dst_vreg;
    ip->src1 = src1_vreg;
    ip->src2 = src2;
    ip->src2w = src2w;
    INC_SIZE(1);
    return SLJIT_SUCCESS;    
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_atomic_load(struct sljit_compiler *compiler,
				 sljit_s32 op,
				 sljit_s32 dst_reg,
				 sljit_s32 mem_reg)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_ATOMIC_LOAD;
    ip->op = op;
    ip->dst = dst_reg;
    ip->src1 = mem_reg;
    INC_SIZE(1);
    return SLJIT_SUCCESS;        
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_atomic_store(struct sljit_compiler *compiler,
				  sljit_s32 op,
				  sljit_s32 src_reg,
				  sljit_s32 mem_reg,
				  sljit_s32 temp_reg)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_ATOMIC_STORE;
    ip->op = op;
    ip->dst = mem_reg;    
    ip->src1 = src_reg;
    ip->src2 = temp_reg;
    INC_SIZE(1);
    return SLJIT_SUCCESS;            

}

SLJIT_API_FUNC_ATTRIBUTE struct sljit_const* sljit_emit_const(struct sljit_compiler *compiler,
				     sljit_s32 op,
				     sljit_s32 dst, sljit_sw dstw,
				     sljit_sw init_value)
{
    sljitter_inst_t* ip;
    struct sljit_const *const_;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    const_ = (struct sljit_const*)ensure_abuf(compiler, sizeof(struct sljit_const));
    set_const(const_, compiler);    
    ip->fmt = FMT_CONST;
    ip->op = op;
    ip->dst = dst;
    ip->dstw = dstw;
    ip->sw_val = init_value;
    INC_SIZE(1);
    return const_;
}


SLJIT_API_FUNC_ATTRIBUTE struct sljit_jump* sljit_emit_op_addr(struct sljit_compiler *compiler,
				      sljit_s32 op,
				      sljit_s32 dst, sljit_sw dstw)
{
    sljitter_inst_t* ip;
    struct sljit_jump* jump;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    ip->fmt = FMT_OP_ADDR;
    ip->op = op;
    ip->dst = dst;
    ip->dstw = dstw;
    jump = (struct sljit_jump*)ensure_abuf(compiler, sizeof(struct sljit_jump));
    set_mov_addr(jump, compiler, 0);    
    jump->addr = compiler->size;
    INC_SIZE(1);
    return jump;
}

SLJIT_API_FUNC_ATTRIBUTE void sljit_set_jump_addr(sljit_uw addr, sljit_uw new_target, sljit_sw executable_offset)
{
    // tricky, but possible
}

SLJIT_API_FUNC_ATTRIBUTE void sljit_set_const(sljit_uw addr, sljit_s32 op, sljit_sw new_constant, sljit_sw executable_offset)
{
    // tricky, but possible    
}


SLJIT_API_FUNC_ATTRIBUTE void* sljit_generate_code(struct sljit_compiler *compiler, sljit_s32 options, void *exec_allocator_data)
{
    struct sljit_memory_fragment *buf;
    sljitter_inst_t* code;
    sljitter_inst_t* dst;
    struct sljit_jump* jump;
    
    if (!compiler->size)
	return NULL;
    code = (sljitter_inst_t*) malloc(sizeof(sljitter_inst_t)*compiler->size);
    
    reverse_buf(compiler);
    buf = compiler->buf;
    // label = compiler->labels;
    jump = compiler->jumps;
    // const_ = compiler->consts;
    dst = code;
    // copy all instructions into "code" area
    while(buf) {
	sljit_uw len = buf->used_size;
	sljitter_inst_t* src = (sljitter_inst_t*) buf->memory;
	while(len >= sizeof(sljitter_inst_t)) {
	    *dst++ = *src++;
	    len -= sizeof(sljitter_inst_t);
	}
	buf = buf->next;
    }

    compiler->error = SLJIT_ERR_COMPILED;
    compiler->executable_offset = 0;
    compiler->executable_size = compiler->size*sizeof(sljitter_inst_t);

    jump = compiler->jumps;
    while (jump) {
	sljit_uw addr = jump->addr;
	switch(code[addr].fmt) {
	case FMT_CMP:
	case FMT_JUMP: {
	    struct sljit_label* target_label = jump->u.label;
	    code[addr].target = target_label->u.addr;  // absolute jump!
	    break;
	}
	default:
	    break;
	}
	jump = jump->next;	
    }
    
    return (void*)code;    
}
