/*
 *   sljit emulator
 */

#include <stdint.h>
#include <byteswap.h>
#include <math.h>

#ifdef DEBUG_ACCESS
#define DBG_ACCESS(fmt,args...) \
    fprintf(stderr, fmt "\r\n", args)
#else
#define DBG_ACCESS(fmt,args...)
#endif

#ifdef DEBUG_FRAME
#define DBG_FRAME(fmt,args...) \
    fprintf(stderr, fmt "\r\n", args)
#else
#define DBG_FRAME(fmt,args...)
#endif

#ifdef DEBUG_TRACE
#define DBG_TRACE(fmt,args...) \
    fprintf(stderr, fmt "\r\n", args)
#else
#define DBG_TRACE(fmt,args...)
#endif

#ifdef DEBUG_STATE
#define DUMP_STATE(st, pc) dump_state((st),(pc))
#else
#define DUMP_STATE(st, pc)
#endif


// instruction formats (so far)
typedef enum {
    FMT_OP0 =         1,
    FMT_OP1 =         2,
    FMT_OP2 =         3,
    FMT_OP2U =        4,
    FMT_OP2R =        5,
    FMT_SHIFT_INTO =  6,
    FMT_OP_SRC =      7,
    FMT_OP_DST =      8,
    FMT_FOP1 =        9,
    FMT_FOP2 =        10,
    FMT_FOP2R =       11,  // NOT GENERATED
    FMT_FSET32 =      12,
    FMT_FSET64 =      13,
    FMT_FCOPY =       14,
    FMT_LABEL =       15,
    FMT_JUMP =        16,
    FMT_CALL =        17,
    FMT_CMP =         18,  // NOT GENERATED
    FMT_FCMP =        19,  // NOT GENERATED
    FMT_IJUMP =       20,
    FMT_ICALL =       21,
    FMT_ENTER =       22,
    FMT_SET_CONTEXT = 23,
    FMT_RETURN =      24,  // NOT GENERATED
    FMT_RETURN_VOID = 25,
    FMT_SIMD_OP2 =    26,
    FMT_MCALL =       27,  // module:function call
    FMT_MJUMP =       28,  // module:name jump
    FMT_CONST =       29,
    FMT_OP_ADDR =     30,
    FMT_OP2CMPZ =     31,  // NOT GENERATED
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
} sljitter_fmt_t;

// 64 bytes?
typedef struct {
    sljitter_fmt_t fmt;
    sljit_s32 arg_types; // multiple uses (fixme)
    union {
	struct {
	    sljit_s32 op;
	    sljit_s32 type;
	    sljit_s32 dst; sljit_sw dstw;
	    sljit_s32 src1; sljit_sw src1w;
	    sljit_s32 src2; sljit_sw src2w;
	    sljit_s32 src3; sljit_sw src3w;
	};
	struct {
	    sljit_s32 options;
	    sljit_s32 scratches;
	    sljit_s32 saveds;
	    sljit_s32 local_size;
	};
    };
    union {
	sljit_f32 f32_val;
	sljit_f64 f64_val;
	sljit_sw  sw_val;
	sljit_uw  target;
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

#define U32_MAX    0xffffffff
#define S32_MAX    0x7fffffff
#define S32_MIN   -0x80000000

#if __SIZEOF_LONG__ == 4
typedef int64_t sljit_sww;
typedef uint64_t sljit_uww;
#define SWMASK    0xffffffff
#define UWMASK    0xffffffff
#define UW_MAX    0xffffffff
#define SW_MAX    0x7fffffff
#define SW_MIN   -0x80000000
#define bswap_uw(x) bswap_32((x))
#elif __SIZEOF_LONG__ == 8
typedef __int128_t sljit_sww;
typedef __uint128_t sljit_uww;
#define SWMASK    0xffffffffffffffff
#define UWMASK    0xffffffffffffffff
#define UW_MAX    0xffffffffffffffff
#define SW_MAX    0x7fffffffffffffff
#define SW_MIN   -0x8000000000000000
#define bswap_uw(x) bswap_64((x))
#endif

#define extend_16(x) (((x) & 0x8000) ? (((x) & 0xffff) | 0xffff0000) : ((x) & 0xffff))
#define extend_32(x) (((x) & 0x80000000) ? (((x) & 0xffffffff) | 0xffffffff00000000) : ((x) & 0xffffffff))

#define R(st,i)   ((st)->r[(i)].sw)
#define R32(st,i) ((st)->r[(i)].s32)
#define FR(st,i)  ((st)->fr[(i)])
#define VR(st,i)  ((st)->vr[(i)])

// flags field
#define FLAG_C    0x01   // carry
#define FLAG_V    0x02   // overflow
#define FLAG_Z    0x04   // zero
#define FLAG_N    0x08   // negative
#define FLAG_L    0x10   // less (float)
#define FLAG_G    0x20   // greater (float)
#define FLAG_E    0x40   // equal (float)
#define FLAGS_LGE (FLAG_L|FLAG_G|FLAG_E)
#define FLAGS_CVZN (FLAG_C|FLAG_V|FLAG_Z|FLAG_N)

#define FLAGS_ALL(st,fs) (((st)->flags & (fs)) == (fs))
#define FLAGS_ANY(st,fs) (((st)->flags & (fs)) != 0)
#define FLAGS_NONE(st,fs) (((st)->flags & (fs)) == 0)

// get the effective address assume SLJIT_MEM is set!
static sljit_sw effective_addr(sljit_s32 dst, sljit_sw dstw, emulator_state_t* st)
{
    sljit_sw addr;
    if (dst == SLJIT_MEM) {
	addr = dstw;
	DBG_ACCESS("imm:%ld", addr);
	return addr;
    }
    else {
	int r1 = dst & 0x7f;
	int r2;
	if ((r1 == 0) || (r1 > (SLJIT_NUMBER_OF_REGISTERS+1)))
	    return -1;
	if ((r2 = ((dst >> 8) & 0x7f)) > 0) {
	    if (r2 > (SLJIT_NUMBER_OF_REGISTERS+1))
		return -1;
	    else if (dstw == 0) {
		addr = R(st,r1-1) + R(st,r2-1);
		DBG_ACCESS("R%d+R%d:%ld", r1-1, r2-1, addr);
		return addr;
	    }
	    else { // error if srcw > 3?
		addr = R(st,r1-1) + (R(st,r2-1) << (dstw & 3));
		DBG_ACCESS("R%d+R%d*%d:%ld", r1-1, r2-1, (1<<(dstw & 3)), addr);
		return addr;
	    }
	}
	addr = R(st,r1-1) + dstw;
	DBG_ACCESS("R%d+%ld:%ld", r1-1, dstw, addr); 
	return addr;
    }
}

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

#define TP   f32
#define TYPE sljit_f32
#include "sljitEmulatorfxx.i"

#define TP   f64
#define TYPE sljit_f64
#include "sljitEmulatorfxx.i"

#define INC_SIZE(s) compiler->size += (s)
#define UNUSED(v) (void) v

#define ARGTYPE_RET(type)  ((type) & 0xf)
#define ARGTYPE(type,i)    (((type) >> (((i)+1)*SLJIT_ARG_SHIFT)) & 0xf)

// FIXME: this must match sljit_nif right now....
typedef union arg_val {
    sljit_sw  sw;
    sljit_f64 f64;
} arg_val_t;

#define SLJIT_ARG_TYPE_TERM    6
#define SLJIT_ARG_TYPE_TERM_R  (6|SLJIT_ARG_TYPE_SCRATCH_REG)

#include <avcall.h>

typedef union code_val {
    sljit_sw  sw;
    sljit_s32 s32;
    void*     ptr;
    sljit_f64 f64;
    sljit_f32 f32;
} code_val_t;


// 0xttttt = 4
// 0x0tttt = 3
// 0x00ttt = 2
// 0x000tt = 1
// 0x0000t = 0
static int arg_types_argc(sljit_s32 arg_types)
{
    if (arg_types & 0x70000) return 4;
    if (arg_types & 0x07000) return 3;
    if (arg_types & 0x00700) return 2;
    if (arg_types & 0x00070) return 1;
    return 0;
}

typedef struct {
    sljit_sw sp;
    sljit_sw pc;
} emulator_frame_t;

// move stack pointer size bytes, but do not push data
static void* alloc(emulator_state_t* st, size_t size)
{
    sljit_sw sp0 = st->r[SLJIT_SP-1].sw;
    sljit_sw sp1 = sp0 - (sljit_sw) size;
    if (sp1 < (sljit_sw) (st->ram_size - st->stack_size))
	return NULL;
    st->r[SLJIT_SP-1].sw = sp1;
    return st->mem_base+sp1;
}

static void* dealloc(emulator_state_t* st, size_t size)
{
    sljit_sw sp0 = st->r[SLJIT_SP-1].sw;
    sljit_sw sp1 = sp0 + (sljit_sw)size;
    if (sp1 > (sljit_sw)st->ram_size)
	return NULL;
    st->r[SLJIT_SP-1].sw = sp1;
    return st->mem_base+sp0;
}

static int push(emulator_state_t* st, void* data, size_t size)
{
    void* ptr;
    if ((ptr = alloc(st, size)) == NULL) return -1;
    memcpy(ptr, data, size);
    return 0;
}

static int pop(emulator_state_t* st, void* data, size_t size)
{
    void* ptr;
    if ((ptr = dealloc(st, size)) == NULL) return -1;
    memcpy(data, ptr, size);
    return 0;
}

// Create a stack frame with local variables
static int push_frame(emulator_state_t* st, size_t local_size, sljit_sw pc)
{
    sljit_sw sp0 = st->r[SLJIT_SP-1].sw;
    emulator_frame_t* fp;
    if ((fp = alloc(st, local_size+sizeof(emulator_frame_t))) == NULL)
	return -1;
    fp->sp = sp0;   // save old sp
    fp->pc = pc;    // and current pc (current call instruction)
    st->r[SLJIT_SP-1].sw += sizeof(emulator_frame_t); // move to locals area
    return 0;
}

// Remove locals
static int pop_frame(emulator_state_t* st, sljit_sw* pc_ptr)
{
    sljit_sw sp0 = st->r[SLJIT_SP-1].sw;
    emulator_frame_t* fp =
	(emulator_frame_t*)(st->mem_base + sp0 - sizeof(emulator_frame_t));
    *pc_ptr = fp->pc;
    sp0 = fp->sp;
    st->r[SLJIT_SP-1].sw = sp0;
    return 0;
}

static int push_uw(emulator_state_t* st, sljit_uw data)
{
    return push(st, &data, sizeof(data));
}

static int push_f64(emulator_state_t* st, sljit_f64 data)
{
    return push(st, &data, sizeof(data));
}

#if 0

static int push_sw(emulator_state_t* st, sljit_sw data)
{
    return push(st, &data, sizeof(data));
}

static int push_f32(emulator_state_t* st, sljit_f32 data)
{
    return push(st, &data, sizeof(data));
}

#endif


static int pop_uw(emulator_state_t* st, sljit_uw* data)
{
    return pop(st, data, sizeof(*data));
}

static int pop_f64(emulator_state_t* st, sljit_f64* data)
{
    return pop(st, data, sizeof(*data));
}

#if 0
static int pop_sw(emulator_state_t* st, sljit_sw* data)
{
    return pop(st, data, sizeof(*data));
}

static int pop_f32(emulator_state_t* st, sljit_f32* data)
{
    return pop(st, data, sizeof(*data));
}
#endif



// extract flags from operation
static inline sljit_s32 get_flags(sljit_s32 op)
{
    cpu_flags_t f = 0;    
    f = (op & SLJIT_SET_Z) ? FLAG_Z : 0;
    f |= (GET_FLAG_TYPE(op) == GET_FLAG_TYPE(SLJIT_SET_CARRY)) ? FLAG_C : 0;
    f |= (GET_FLAG_TYPE(op) == SLJIT_OVERFLOW) ? FLAG_V : 0;
    // more flags to generate...
    return f;
}

static inline void set_flags_addc(cpu_flags_t* fp, cpu_flags_t set, sljit_uw a, sljit_uw b, sljit_uw c)
{
    UNUSED(b);
    cpu_flags_t f = 0;    
    if ((set & FLAG_C) && (c < a)) f |= FLAG_C;
    *fp = (*fp & ~(FLAG_C)) | f;
}

static inline void set_flags_add(cpu_flags_t* fp, cpu_flags_t set, sljit_uw a, sljit_uw b, sljit_uw c)
{
    cpu_flags_t f = 0;
    if ((set & FLAG_Z) && (c == 0)) f |= FLAG_Z;
    if ((set & FLAG_N) && ((c >> (UWSIZE-1)) & 1)) f |= FLAG_N;
    if ((set & FLAG_C) && (c < a)) f |= FLAG_C;
    if ((set & FLAG_V) && ((((a ^ c) & (b ^ c)) >> (UWSIZE-1)) & 1))
	f |= FLAG_V;
    *fp = (*fp & ~FLAGS_CVZN) | f;
}

static inline void set_flags_subc(cpu_flags_t* fp, cpu_flags_t set, sljit_uw a, sljit_uw b, sljit_uw c)
{    
    (void) c;
    cpu_flags_t f = 0;
    if ((set & FLAG_C) && (a < b)) f |= FLAG_C;
    *fp = (*fp & ~(FLAG_C)) | f;
}

static inline void set_flags_sub(cpu_flags_t* fp, cpu_flags_t set, sljit_uw a, sljit_uw b, sljit_uw c)
{
    cpu_flags_t f = 0;
    if ((set & FLAG_Z) && (c == 0)) f |= FLAG_Z;
    if ((set & FLAG_N) && ((c >> (UWSIZE-1)) & 1)) f |= FLAG_N;
    if ((set & FLAG_C) && (a < b)) f |= FLAG_C;
    if ((set & FLAG_V) && ((((a ^ b) & (a ^ c)) >> (UWSIZE-1)) & 1))
	f |= FLAG_V;
    *fp = (*fp & ~FLAGS_CVZN) | f;
}


// ???? FIXME ZERO?
static inline void set_flags_logic(cpu_flags_t* fp, cpu_flags_t set, sljit_uw c)
{
    cpu_flags_t f = 0;
    if ((set & FLAG_C) && (c == 0)) f |= FLAG_C;
    *fp = (*fp & ~(FLAG_C)) | f;
}

static inline void set_flags_zero(cpu_flags_t* fp, cpu_flags_t set, sljit_uw c)
{
    cpu_flags_t f = 0;
    if ((set & FLAG_Z) && (c == 0)) f |= FLAG_C;
    *fp = (*fp & ~(FLAG_Z)) | f;
}

static inline void set_flags_addc32(cpu_flags_t* fp, cpu_flags_t set, sljit_u32 a, sljit_u32 b, sljit_u32 c)
{
    UNUSED(b);
    cpu_flags_t f = 0;    
    if ((set & FLAG_C) && (c < a)) f |= FLAG_C;
    *fp = (*fp & ~(FLAG_C)) | f;    
}

static inline void set_flags_add32(cpu_flags_t* fp, cpu_flags_t set, sljit_u32 a, sljit_u32 b, sljit_u32 c)
{
    cpu_flags_t f = 0;
    if ((set & FLAG_Z) && (c == 0)) f |= FLAG_Z;
    if ((set & FLAG_N) && ((c >> (U32SIZE-1)) & 1)) f |= FLAG_N;
    if ((set & FLAG_C) && (c < a)) f |= FLAG_C;
    if ((set & FLAG_V) && ((((a ^ c) & (b ^ c)) >> (U32SIZE-1)) & 1))
	f |= FLAG_V;
    *fp = (*fp & ~FLAGS_CVZN) | f;
}

static inline void set_flags_subc32(cpu_flags_t* fp, cpu_flags_t set, sljit_u32 a, sljit_u32 b, sljit_u32 c)
{
    (void) c;
    cpu_flags_t f = 0;
    if ((set & FLAG_C) && (a < b)) f |= FLAG_C;
    *fp = (*fp & ~(FLAG_C)) | f;
}

static inline void set_flags_sub32(cpu_flags_t* fp, cpu_flags_t set, sljit_u32 a, sljit_u32 b, sljit_u32 c)
{
    cpu_flags_t f = 0;
    if ((set & FLAG_Z) && (c == 0)) f |= FLAG_Z;
    if ((set & FLAG_N) && ((c >> (U32SIZE-1)) & 1)) f |= FLAG_N;
    if ((set & FLAG_C) && (a < b)) f |= FLAG_C;
    if ((set & FLAG_V) && ((((a ^ b) & (a ^ c)) >> (U32SIZE-1)) & 1))
	f |= FLAG_V;
    *fp = (*fp & ~FLAGS_CVZN) | f;
}

static inline void set_flags_logic32(cpu_flags_t* fp, cpu_flags_t set, sljit_u32 c)
{
    cpu_flags_t f = 0;
    if ((set & FLAG_C) && (c == 0)) f |= FLAG_C;
    *fp = (*fp & ~(FLAG_C)) | f;    
}

static inline void set_flags_zero32(cpu_flags_t*fp, cpu_flags_t set, sljit_u32 c)
{
    cpu_flags_t f = 0;
    if ((set & FLAG_Z) && (c == 0)) f |= FLAG_C;
    *fp = (*fp & ~(FLAG_Z)) | f;    
}

#if 0
static inline void set_flags_mul(cpu_flags_t* fp, cpu_flags_t set, sljit_sw a, sljit_sw b, sljit_sw c)
{
    cpu_flags_t f = 0;
    if (set & FLAG_V) {
	sljit_sww cc =(sljit_sww) a * (sljit_sww) b;
	if (((sljit_sw) cc) != c) f |= FLAG_V;
    }
    *fp = (*fp & ~(FLAG_V)) | f;
}
#endif

// EQUAL_F | LESS_F | GREATER_EQUAL_F | GREATER_F | LESS_EQUAL_F
static void cmp_f32(cpu_flags_t* fp, sljit_f32 a, sljit_f32 b)
{
    cpu_flags_t f = 0;

    if (a < b)   f |= FLAG_L;
    if (a > b)   f |= FLAG_G;
    if (a == b)  f |= FLAG_E;
    *fp = (*fp & ~(FLAGS_LGE)) | f;
}

static void cmp_f64(cpu_flags_t* fp, sljit_f64 a, sljit_f64 b)
{
    cpu_flags_t f = 0;

    if (a < b)   f |= FLAG_L;
    if (a > b)   f |= FLAG_G;
    if (a == b)  f |= FLAG_E;
    *fp = (*fp & ~(FLAGS_LGE)) | f;
}


static void mul_s32(cpu_flags_t* fp, cpu_flags_t set, sljit_s32 a, sljit_s32 b, sljit_s32* c)
{
    cpu_flags_t f = 0;
    if (set & FLAG_V) {
	if ((a == 0) || (b == 0))
	    *c = 0;
	else if (((a == -1) && (b == (sljit_s32)S32_MIN)) ||
		 ((b == -1) && (a ==  (sljit_s32)S32_MIN))) {
	    f = FLAG_V;
	    *c = S32_MIN;
	}
	else {
	    sljit_s32 r = a * b;
	    if (((a > 0) && (b > 0) && (r < 0)) ||
		((a < 0) && (b < 0) && (r < 0)) ||
		((a > 0) && (b < 0) && (r > 0)) ||
		((a < 0) && (b > 0) && (r > 0))) {
		f = FLAG_V;
	    }
	    *c = r;
	}
    }
    else {
	*c = a * b;
    }
    *fp = (*fp & ~(FLAG_V)) | f;
}

#if 0
static void mul_u32(cpu_flags_t* fp, cpu_flags_t set, sljit_u32 a, sljit_u32 b, sljit_u32* c)
{
    cpu_flags_t f = 0;
    if (set & FLAG_V) {
	if ((a == 0) || (b == 0))
	    *c = 0;
	else {
	    *c = a * b;
	    if (b > U32_MAX / a)
		f = FLAG_V;
	}
    }
    else {
	*c = a * b;
    }
    *fp = (*fp & ~(FLAG_V)) | f;
}
#endif


static void mul_sw(cpu_flags_t* fp, cpu_flags_t set, sljit_sw a, sljit_sw b, sljit_sw* c)
{
    cpu_flags_t f = 0;
    if (set & FLAG_V) {
	if ((a == 0) || (b == 0))
	    *c = 0;
	else if (((a == -1) && (b == (sljit_sw)SW_MIN)) ||
		 ((b == -1) && (a == (sljit_sw)SW_MIN))) {
	    f = FLAG_V;
	    *c = (sljit_sw)S32_MIN;
	}
	else {
	    sljit_sw r = a * b;
	    if (((a > 0) && (b > 0) && (r < 0)) ||
		((a < 0) && (b < 0) && (r < 0)) ||
		((a > 0) && (b < 0) && (r > 0)) ||
		((a < 0) && (b > 0) && (r > 0))) {
		f = FLAG_V;
	    }
	    *c = r;
	}
    }
    else {
	*c = a * b;
    }
    *fp = (*fp & ~(FLAG_V)) | f;
}

#if 0
static void mul_uw(cpu_flags_t* fp, cpu_flags_t set, sljit_uw a, sljit_uw b, sljit_uw* c)
{
    cpu_flags_t f = 0;
    if (set & FLAG_V) {
	if ((a == 0) || (b == 0))
	    *c = 0;
	else {
	    *c = a * b;
	    if (b > U32_MAX / a)
		f = FLAG_V;
	}
    }
    else {
	*c = a * b;
    }
    *fp = (*fp & ~(FLAG_V)) | f;
}
#endif

static sljitter_inst_t* new_inst(struct sljit_compiler *compiler,
				 sljitter_fmt_t fmt,
				 sljit_s32 op, sljit_s32 type)
{
    sljitter_inst_t* ip;
    ip = ensure_buf(compiler, sizeof(sljitter_inst_t));
    SLJIT_ZEROMEM(ip, sizeof(sljitter_inst_t));
    ip->fmt = fmt;
    ip->op  = op;
    ip->type = type;
    return ip;
}

#ifdef DEBUG_STATE
static void dump_state(emulator_state_t* st, sljit_sw pc)
{
    int i, j;
    
    fprintf(stderr, "emulator state\r\n");
    fprintf(stderr, "  pc = %ld\r\n", pc);
    fprintf(stderr, "  ram_size = %lu\r\n", st->ram_size);
    fprintf(stderr, "  rom_size = %lu\r\n", st->rom_size);    
    fprintf(stderr, "  stack_size = %lu\r\n", st->stack_size); 
    fprintf(stderr, "  flags = %lx\r\n",    st->flags);
    for (i = 0; i < SLJIT_NUMBER_OF_REGISTERS; i++)
	fprintf(stderr, "  R%d = %lu\r\n", i, st->r[i].uw);
    fprintf(stderr, "  SP = %lu\r\n", st->r[SLJIT_SP-1].uw);    
    for (i = 0; i < SLJIT_NUMBER_OF_FLOAT_REGISTERS; i++)
	fprintf(stderr, "  FR%d = %f\r\n", i, st->fr[i].f64);
    for (i = 0; i < 16; i++) {
	fprintf(stderr, "  mem[%04d]: ", i*16);
	for (j = 0; j < 16; j++)
	    fprintf(stderr, "%02x ", st->mem_base[i*16+j]);
	fprintf(stderr, "\r\n");
    }
}
#endif

static int enter(emulator_state_t* st, sljitter_inst_t* ip, sljit_sw pc)
{
    sljit_s32 options    = ip->options;
    sljit_s32 arg_types  = ip->arg_types;
    sljit_s32 scratches  = ip->scratches;
    sljit_s32 saveds     = ip->saveds;
    sljit_s32 local_size = ip->local_size;
    sljit_s32 fscratches = ENTER_GET_FLOAT_REGS(scratches);
    sljit_s32 fsaveds    = ENTER_GET_FLOAT_REGS(saveds);
    sljit_s32 vscratches = ENTER_GET_VECTOR_REGS(scratches);
    sljit_s32 vsaveds    = ENTER_GET_VECTOR_REGS(saveds);    
    sljit_s32 i, tmp; // offset;
    sljit_s32 saved_arg_count = SLJIT_KEPT_SAVEDS_COUNT(options);

    scratches = ENTER_GET_REGS(scratches);
    saveds = ENTER_GET_REGS(saveds);
    local_size += GET_SAVED_REGISTERS_SIZE(scratches,saveds-saved_arg_count,1);
    local_size += GET_SAVED_FLOAT_REGISTERS_SIZE(fscratches, fsaveds, f64);
    // align local size to 16 byte (128 bit) boundry
    local_size = (local_size + SLJIT_LOCALS_OFFSET + 15) & ~0xf;

    DBG_FRAME("arg_types = %x", arg_types);
    DBG_FRAME("local_size = %d", local_size);

    // skip past old frames return frame
    alloc(st, sizeof(emulator_frame_t));
    
    tmp = SLJIT_S0 - saveds;
    for (i = SLJIT_S0 - saved_arg_count; i > tmp; i--) {
	DBG_FRAME("PUSH S%d (R%d) = %lu",
		  SLJIT_NUMBER_OF_REGISTERS-i, i-1, st->r[i-1].uw);
	push_uw(st, st->r[i-1].uw);
    }
    for (i = scratches; i >= SLJIT_FIRST_SAVED_REG; i--) {
	DBG_FRAME("PUSH R%d = %lu", i-1, st->r[i-1].uw);
	push_uw(st, st->r[i-1].uw);
    }

    tmp = SLJIT_FS0 - fsaveds;
    for (i = SLJIT_FS0; i > tmp; i--) {
	DBG_FRAME("PUSH FS%d (FR%d) = %f",
		  SLJIT_NUMBER_OF_FLOAT_REGISTERS-i, i-1,
		  st->fr[i-1].f64);
	push_f64(st, st->fr[i-1].f64);
    }
    for (i = fscratches; i >= SLJIT_FIRST_SAVED_FLOAT_REG; i--) {
	DBG_FRAME("PUSH FR%d = %f", i-1, st->fr[i-1].f64);
	push_f64(st, st->fr[i-1].f64);
    }

    tmp = SLJIT_VS0 - vsaveds;
    for (i = SLJIT_VS0; i > tmp; i--) {
	DBG_FRAME("PUSH VS%d (VR%d) = fixme",
		  SLJIT_NUMBER_OF_VECTOR_REGISTERS-i, i-1);
	push(st, st->vr[i-1].vi8, VSIZE);
    }
    for (i = vscratches; i >= SLJIT_FIRST_SAVED_VECTOR_REG; i--) {
	DBG_FRAME("PUSH VR%d = fixme", i-1);
	push(st, st->vr[i-1].vi8, VSIZE);	
    }
    
    push_frame(st, local_size, pc);
    
    // move arguments to saved register (unless SCRATCH marked)
    arg_types >>= SLJIT_ARG_SHIFT;
    saved_arg_count = 0;
    tmp = SLJIT_R0;
    while (arg_types > 0) {
	if ((arg_types & SLJIT_ARG_MASK) < SLJIT_ARG_TYPE_F64) {
	    if (!(arg_types & SLJIT_ARG_TYPE_SCRATCH_REG)) {
		int sj = SLJIT_S0-saved_arg_count;
		DBG_FRAME("MOVE S%d = R%d", SLJIT_NUMBER_OF_REGISTERS-sj,
			  tmp-1);
		st->r[sj-1] = st->r[tmp-1];
		DBG_FRAME("S%d = %lu", SLJIT_NUMBER_OF_REGISTERS-sj,
			  st->r[tmp-1].uw);
		saved_arg_count++;
	    }
	    tmp++;
	}
	arg_types >>= SLJIT_ARG_SHIFT;
    }

    return SLJIT_SUCCESS;
}

// undo the effect of enter
static int leave(emulator_state_t* st, sljitter_inst_t* ip, sljit_sw* pc_ptr)
{
    sljit_s32 options    = ip->options;
    sljit_s32 scratches  = ip->scratches;
    sljit_s32 saveds     = ip->saveds;
    sljit_s32 fscratches = ENTER_GET_FLOAT_REGS(scratches);
    sljit_s32 fsaveds    = ENTER_GET_FLOAT_REGS(saveds);
    sljit_s32 vscratches = ENTER_GET_VECTOR_REGS(scratches);
    sljit_s32 vsaveds    = ENTER_GET_VECTOR_REGS(saveds);
    sljit_s32 i, tmp; // offset;
    sljit_s32 saved_arg_count = SLJIT_KEPT_SAVEDS_COUNT(options);
    // sljit_s32 local_size;

    DBG_TRACE("LEAVE scratches=%x, saveds=%x, saved_arg_count=%d",
	    scratches, saveds, saved_arg_count);    
    
    scratches = ENTER_GET_REGS(scratches);
    saveds = ENTER_GET_REGS(saveds);

    pop_frame(st, pc_ptr);

    // reverse pop all registers
    for (i = SLJIT_FIRST_SAVED_VECTOR_REG; i <= vscratches; i++) {
	pop(st, st->vr[i-1].vi8, VSIZE);
	DBG_FRAME("POP VR%d = fixme", i-1);
    }
    
    tmp = SLJIT_VS0 - vsaveds;
    for (i = tmp+1; i <= SLJIT_VS0; i++) {
	pop(st, st->vr[i-1].vi8, VSIZE);	
	DBG_FRAME("POP VS%d (VR%d) = fixme",
		  SLJIT_NUMBER_OF_VECTOR_REGISTERS-i, i-1);
    }

    for (i = SLJIT_FIRST_SAVED_FLOAT_REG; i <= fscratches; i++) {
	pop_f64(st, &st->fr[i-1].f64);
	DBG_FRAME("POP FR%d = %f", i-1, st->fr[i-1].f64);
    }
    
    tmp = SLJIT_FS0 - fsaveds;
    for (i = tmp+1; i <= SLJIT_FS0; i++) {
	pop_f64(st, &st->fr[i-1].f64);
	DBG_FRAME("POP FS%d (FR%d) = %f",
		  SLJIT_NUMBER_OF_FLOAT_REGISTERS-i, i-1, st->fr[i-1].f64);
    }

    for (i = SLJIT_FIRST_SAVED_REG; i <= scratches; i++) {
	pop_uw(st, &st->r[i-1].uw);
	DBG_FRAME("POP R%d = %lu", i-1, st->r[i-1].uw);	
    }
    
    tmp = SLJIT_S0 - saveds;
    for (i = tmp+1; i <= SLJIT_S0 - saved_arg_count; i++) {
	pop_uw(st, &st->r[i-1].uw);
	DBG_FRAME("POP S%d (R%d) = %lu",
		  SLJIT_NUMBER_OF_REGISTERS-i, i-1, st->r[i-1].uw);
    }

    // move past the frame 
    dealloc(st, sizeof(emulator_frame_t)); 
    
    return SLJIT_SUCCESS;    
}

//
// sljit opcodes:
//         0-63     0-255 opcode
//         fffffnz3 xxxxxxxx
//              |||  
//              ||+-- SLJIT_32 32-bit operation
//              |+--- SLJIT_SET_Z  zero flag
//              +----- inverted flags value
//
static void emu(emulator_state_t* st, sljitter_inst_t* prog, size_t size,
		sljit_sw pc)
{
    int r = 0;
    DUMP_STATE(st, pc);     
next:
    if (pc >= (sljit_sw)size) {
	fprintf(stderr, "emu: error prog out of range\r\n");
	DUMP_STATE(st, pc);	
	return;
    }
    DBG_TRACE("emu: pc=%ld, fmt=%d, op=%d",
	      pc, prog[pc].fmt, prog[pc].op);
    
    switch(prog[pc].fmt) {
    case FMT_ENTER: {
	DBG_TRACE("ENTER (call=%ld)", st->call);
	enter(st, &prog[pc], st->call);
	break;
    }
    case FMT_SET_CONTEXT: // like a nop
	break;
    case FMT_OP_SRC:
    case FMT_OP_DST:
	break;
    case FMT_FSET32:
	store_f32(prog[pc].f32_val,prog[pc].dst, 0, st);
	break;
    case FMT_FSET64:
	store_f64(prog[pc].f64_val,prog[pc].dst, 0, st);
	break;
    case FMT_FCOPY:   // dst=freg, src1=reg
	switch(GET_OPCODE(prog[pc].op)) {
	case SLJIT_COPY_TO_F64: {
	    union {
		sljit_uw  u;
		sljit_f64 f;
	    } val;
	    load_uw(&val.u, prog[pc].src1, 0, st);
	    store_f64(val.f, prog[pc].dst, 0, st);
	    break;
	}
	case SLJIT_COPY32_TO_F32: {
	    union {
		sljit_u32 u;
		sljit_f32 f;
	    } val; 
	    load_u32(&val.u, prog[pc].src1, 0, st);
	    store_f32(val.f, prog[pc].dst, 0, st);
	    break;
	}	   
	case SLJIT_COPY_FROM_F64: {
	    union {
		sljit_uw  u;
		sljit_f64 f;
	    } val;
	    load_f64(&val.f, prog[pc].dst, 0, st);
	    store_uw(val.u, prog[pc].src1, 0, st);
	    break;	    
	}
	case SLJIT_COPY32_FROM_F32: {
	    union {
		sljit_u32 u;
		sljit_f32 f;
	    } val;
	    load_f32(&val.f, prog[pc].dst, 0, st);
	    store_u32(val.u, prog[pc].src1, 0, st);
	    break;
	}
	default:
	    break;
	}
	break;	

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
	    r = FLAGS_NONE(st,FLAG_C); break;
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
	case SLJIT_F_EQUAL:
	    r = FLAGS_ALL(st,FLAG_E) && FLAGS_NONE(st,FLAG_L|FLAG_G);
	    break;
	case SLJIT_F_NOT_EQUAL:
	    r = FLAGS_NONE(st,FLAG_E) && FLAGS_ANY(st,FLAG_L|FLAG_G);
	    break;
	case SLJIT_F_LESS:
	    r = FLAGS_NONE(st,FLAG_E|FLAG_G) && FLAGS_ALL(st,FLAG_L);
	    break;
	case SLJIT_F_GREATER_EQUAL:
	    r = FLAGS_NONE(st,FLAG_L) && FLAGS_ANY(st,FLAG_G|FLAG_E);
	    break;	    
	case SLJIT_F_GREATER:
	    r = FLAGS_NONE(st,FLAG_L|FLAG_E) && FLAGS_ALL(st,FLAG_G);
	    break;
	case SLJIT_F_LESS_EQUAL:
	    r = FLAGS_ANY(st,FLAG_L|FLAG_E) && FLAGS_NONE(st,FLAG_G);
	    break;
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

    case FMT_CALL: {
	DBG_TRACE("CALL %ld", prog[pc].target);
	st->call = pc;  // set call address, used by enter
	pc = prog[pc].target;
	goto next;
    }

    case FMT_ICALL: {
	int i;
	int argc;
	int ri = 0;
	int fri = 0;
	code_val_t ret;	
	uintptr_t addr;
	sljit_s32 at = prog[pc].arg_types;
	av_alist alist;

	load_ptr(&addr, prog[pc].src1, prog[pc].src1w, st);

	DBG_TRACE("ICALL addr=%p, arg_types=%x",
		  (void*)addr, at);

	// setup av return
	switch (ARGTYPE_RET(at)) {
	case SLJIT_ARG_TYPE_RET_VOID:	    
	    av_start_void(alist, addr);
	case SLJIT_ARG_TYPE_TERM:
	case SLJIT_ARG_TYPE_TERM_R:
	case SLJIT_ARG_TYPE_W:
	case SLJIT_ARG_TYPE_W_R:	    
	    av_start_long(alist, addr, (sljit_sw*) &ret.sw);
	    break;
	case SLJIT_ARG_TYPE_32:
	case SLJIT_ARG_TYPE_32_R:	    
	    av_start_int(alist, addr, (sljit_s32*) &ret.s32);
	    break;
	case SLJIT_ARG_TYPE_F32:	    
	    av_start_float(alist, addr, (float*) &ret.f32);
	    break;
	case SLJIT_ARG_TYPE_F64:	    
	    av_start_double(alist, addr, (double*) &ret.f64);
	    break;
	case SLJIT_ARG_TYPE_P:
	case SLJIT_ARG_TYPE_P_R:	    
	    av_start_ptr(alist, addr, void*,  &ret.ptr);
	    break;
	}
	
	argc = arg_types_argc(at);

	// load arguments into registers
	for (i = 0; i < argc; i++) {
	    switch(ARGTYPE(at, i)) {
	    case SLJIT_ARG_TYPE_W:
	    case SLJIT_ARG_TYPE_W_R:
	    case SLJIT_ARG_TYPE_TERM:
	    case SLJIT_ARG_TYPE_TERM_R:
		av_long(alist, (long) st->r[ri++].sw);
		break;
	    case SLJIT_ARG_TYPE_32:
	    case SLJIT_ARG_TYPE_32_R:
		av_int(alist, (int) st->r[ri++].s32);		
		break;
	    case SLJIT_ARG_TYPE_P:
	    case SLJIT_ARG_TYPE_P_R:
		av_ptr(alist, void*, st->r[ri++].uw);
		break;
	    case SLJIT_ARG_TYPE_F64:
		av_double(alist, st->fr[fri++].f64);		
		break;
	    case SLJIT_ARG_TYPE_F32:
		av_float(alist, st->fr[fri++].f32);
		break;
	    default: // bad argument
		goto ignore;
	    }
	}
	
	av_call(alist);
	
	switch(ARGTYPE_RET(at)) {
	case SLJIT_ARG_TYPE_RET_VOID:
	    break;
	case SLJIT_ARG_TYPE_TERM:
	case SLJIT_ARG_TYPE_TERM_R:
	case SLJIT_ARG_TYPE_W:
	case SLJIT_ARG_TYPE_W_R:
	    st->r[0].sw = ret.sw; break;
	case SLJIT_ARG_TYPE_32:
	case SLJIT_ARG_TYPE_32_R:
	    st->r[0].s32 = ret.sw; break;
	case SLJIT_ARG_TYPE_F32:
	    st->fr[0].f32 = ret.f32; break;
	case SLJIT_ARG_TYPE_F64:
	    st->fr[0].f64 = ret.f64; break;
	case SLJIT_ARG_TYPE_P:
	case SLJIT_ARG_TYPE_P_R:
	    st->r[0].uw = (sljit_uw)ret.sw;
	    break;
	default:
	    break;
	}
	break;
    }
	
    case FMT_OP0:
	switch(GET_OPCODE(prog[pc].op)) {
	case SLJIT_BREAKPOINT:
	    // fixme: add break points
	    break;
	case SLJIT_NOP:
	    break;
	case SLJIT_LMUL_UW: {
	    sljit_uw  a = st->r[0].uw;
	    sljit_uw  b = st->r[0].uw;
	    sljit_uww c = (sljit_uww)a*(sljit_uww)b;
	    st->r[1].uw = (c >> (UWSIZE-1));
	    st->r[0].uw = (c & UWMASK);
	    break;
	}
	case SLJIT_LMUL_SW: {
	    sljit_sw  a = st->r[0].sw;
	    sljit_sw  b = st->r[1].sw;
	    sljit_sww c = (sljit_sww)a*(sljit_sww)b;
	    st->r[1].sw = (c >> (SWSIZE-1));
	    st->r[0].sw = (c & SWMASK);
	    break;
	}
	case SLJIT_DIVMOD_UW: {
	    sljit_uw  a = st->r[0].uw;
	    sljit_uw  b = st->r[1].uw;
	    st->r[0].uw = a / b;
	    st->r[1].uw = a % b;	    
	    break;
	}
	case SLJIT_DIVMOD_U32: {
	    sljit_u32  a = st->r[0].u32;
	    sljit_u32  b = st->r[1].u32;
	    st->r[0].u32 = a / b;
	    st->r[1].u32 = a % b;	    	    
	    break;
	}
	case SLJIT_DIVMOD_SW: {
	    sljit_sw  a = st->r[0].sw;
	    sljit_sw  b = st->r[1].sw;
	    st->r[0].sw = a / b;
	    st->r[1].sw = a % b;	    	    
	    break;
	}
	case SLJIT_DIVMOD_S32: {
	    sljit_s32  a = st->r[0].s32;
	    sljit_s32  b = st->r[1].s32;
	    st->r[0].s32 = a / b;
	    st->r[1].s32 = a % b;	    	    
	    break;	    
	}
	case SLJIT_DIV_UW: {
	    sljit_uw  a = st->r[0].uw;
	    sljit_uw  b = st->r[1].uw;	    
	    st->r[0].uw = a / b;
	    break;
	}
	case SLJIT_DIV_U32: {
	    sljit_u32  a = st->r[0].u32;
	    sljit_u32  b = st->r[1].u32;
	    st->r[0].u32 = a / b;	    
	    break;
	}
	case SLJIT_DIV_SW: {
	    sljit_sw  a = st->r[0].sw;
	    sljit_sw  b = st->r[1].sw;
	    st->r[0].sw = a / b;
	    break;
	}
	case SLJIT_DIV_S32: {
	    sljit_s32  a = st->r[0].s32;
	    sljit_s32  b = st->r[1].s32;
	    st->r[0].s32 = a / b;	    
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
	switch(GET_OPCODE(prog[pc].op)) {
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
	if (GET_OPCODE(prog[pc].op) & SLJIT_32) {
	    sljit_s32 a, b, c;
	    sljit_s32 setf = get_flags(prog[pc].op);
	    load_s32(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_s32(&b, prog[pc].src2, prog[pc].src2w, st);
	    switch(GET_OPCODE(prog[pc].op) & ~SLJIT_32) {
	    case SLJIT_ADD:
		c = a+b;
		set_flags_add32(&st->flags, setf,
				(sljit_u32) a, (sljit_u32) b, (sljit_u32) c);
		break;
	    case SLJIT_ADDC:
		c = a+b;
		set_flags_addc32(&st->flags, setf,
				 (sljit_u32)a, (sljit_u32)b, (sljit_u32) c);
		break;
	    case SLJIT_SUB:
		c = a-b;
		set_flags_sub32(&st->flags, setf,
				(sljit_u32) a, (sljit_u32) b, (sljit_u32) c);
		break;
	    case SLJIT_SUBC:
		c = a-b;
		set_flags_subc32(&st->flags, setf,
				 (sljit_u32) a, (sljit_u32) b, (sljit_u32) c);
		break;
	    case SLJIT_MUL:
		mul_s32(&st->flags, setf, a, b, &c);
		break;
	    case SLJIT_AND:
		c = a&b;
		set_flags_logic32(&st->flags, setf, (sljit_u32) c);
		break;
	    case SLJIT_OR:
		c = a|b;
		set_flags_logic32(&st->flags, setf, (sljit_u32) c);
		break;
	    case SLJIT_XOR:
		c = a^b;		
		set_flags_logic32(&st->flags, setf, (sljit_u32) c);
		break;		
	    case SLJIT_SHL:
		c = a<<b;
		set_flags_zero32(&st->flags, setf, (sljit_u32) c);
		break;
	    case SLJIT_MSHL:
		c = a<<(b&SHIFTM_32);
		set_flags_zero32(&st->flags, setf, (sljit_u32) c);		
		break;
	    case SLJIT_LSHR:
		c = ((sljit_u32)a)>>b;
		set_flags_zero32(&st->flags, setf, (sljit_u32) c);		
		break;
	    case SLJIT_MLSHR:
		c = ((sljit_u32)a)>>(b&SHIFTM_32);
		set_flags_zero32(&st->flags, setf, (sljit_u32) c);		
		break;
	    case SLJIT_ASHR:
		c = a>>b;
		set_flags_zero32(&st->flags, setf, (sljit_u32) c);
		break;
	    case SLJIT_MASHR:
		c = a>>(b&SHIFTM_32);
		set_flags_zero32(&st->flags, setf, (sljit_u32) c);
		break;	    
	    case SLJIT_ROTL:
		c = (a<<(b&SHIFTM_32)) | (a>>(SWSIZE-(b&SHIFTM_32)));
		break;
	    case SLJIT_ROTR:
		c = (a>>(b&SHIFTM_32)) | (a<<(SWSIZE-(b&SHIFTM_32)));
		break;
	    default:
		goto ignore;
	    }
	    store_s32(c, prog[pc].dst, prog[pc].dstw, st);
	}
	else {
	    sljit_sw a, b, c;
	    sljit_s32 setf = get_flags(prog[pc].op);	    
	    load_sw(&a, prog[pc].src1, prog[pc].src1w, st);	    
	    load_sw(&b, prog[pc].src2, prog[pc].src2w, st);
	    switch(GET_OPCODE(prog[pc].op)) {
	    case SLJIT_ADD:
		c = a+b;
		set_flags_add(&st->flags, setf, (sljit_uw) a, (sljit_uw) b,
			      (sljit_uw) c);
		break;
	    case SLJIT_ADDC:
		c = a+b;
		set_flags_addc(&st->flags, setf, (sljit_uw) a, (sljit_uw) b, (sljit_uw) c);		
		break;
	    case SLJIT_SUB:
		c = a-b;
		set_flags_sub(&st->flags, setf, (sljit_uw) a, (sljit_uw) b,
			      (sljit_uw) c);
		break;
	    case SLJIT_SUBC:
		c = a-b;
		set_flags_subc(&st->flags, setf, (sljit_uw) a, (sljit_uw) b,
			       (sljit_uw) c);
		break;
	    case SLJIT_MUL:
		mul_sw(&st->flags, setf, a, b, &c);
		break;
	    case SLJIT_AND:
		c = a&b;
		set_flags_logic(&st->flags, setf, (sljit_u32) c);
		break;
	    case SLJIT_OR:
		c = a|b;
		set_flags_logic(&st->flags, setf, (sljit_u32) c);		
		break;
	    case SLJIT_XOR:
		c = a^b;		
		set_flags_logic(&st->flags, setf, (sljit_u32) c);
		break;
	    case SLJIT_SHL:
		c = a<<b;
		set_flags_zero(&st->flags, setf, (sljit_u32) c);
		break;
	    case SLJIT_MSHL:
		c = a<<(b&SHIFTM);
		set_flags_zero(&st->flags, setf, (sljit_u32) c);
		break;
	    case SLJIT_LSHR:
		c = ((sljit_u32)a)>>b;
		set_flags_zero(&st->flags, setf, (sljit_u32) c);		
		break;
	    case SLJIT_MLSHR:
		c = ((sljit_u32)a)>>(b&SHIFTM);
		set_flags_zero(&st->flags, setf, (sljit_u32) c);
		break;
	    case SLJIT_ASHR:
		c = a>>b;
		set_flags_zero(&st->flags, setf, (sljit_u32) c);
		break;
	    case SLJIT_MASHR:
		c = a>>(b&SHIFTM);
		set_flags_zero(&st->flags, setf, (sljit_u32) c);
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
	if (GET_OPCODE(prog[pc].op) & SLJIT_32) {
	    sljit_s32 a, b, c;
	    load_s32(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_s32(&b, prog[pc].src2, prog[pc].src2w, st);
	    load_s32(&c, prog[pc].dst, 0, st);
	    switch(GET_OPCODE(prog[pc].op) & ~SLJIT_32) {
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
	    switch(GET_OPCODE(prog[pc].op)) {
	    case SLJIT_MULADD:
		c = a*b + c;
		break;
	    default: goto ignore;
	    }
	    store_sw(c, prog[pc].dst & 0x7f, 0, st);	    
	}
	break;
    }

    case FMT_SHIFT_INTO: {
	if (prog[pc].op & SLJIT_32) {
	    sljit_u32 a, b, s, c;
	    
	    load_u32(&a, prog[pc].src1, 0, st);
	    load_u32(&b, prog[pc].src2, 0, st);
	    load_u32(&s, prog[pc].src3, prog[pc].src3w, st);
	    
	    switch(GET_OPCODE(prog[pc].op) & ~SLJIT_32) {
	    case SLJIT_MSHL:
		s &= SHIFTM_32;
		// fall through
	    case SLJIT_SHL:
		c = a << s;
		if ((prog[pc].op & SLJIT_SHIFT_INTO_NON_ZERO) || (s != 0))
		    c |= ((b >> 1) >> (s ^ SHIFTM_32));
		break;
	    case SLJIT_MLSHR:
		s &= SHIFTM_32;
                // fall through
	    case SLJIT_LSHR:
		c = a >> s;
		if ((prog[pc].op & SLJIT_SHIFT_INTO_NON_ZERO) || (s != 0))	
		    c |= ((b << 1) << (s ^ SHIFTM_32));
		break;
	    }
	    store_u32(c, prog[pc].dst, prog[pc].dstw, st);
	}
	else {
	    sljit_uw a, b, s, c;
	    
	    load_uw(&a, prog[pc].src1, 0, st);
	    load_uw(&b, prog[pc].src2, 0, st);
	    load_uw(&s, prog[pc].src3, prog[pc].src3w, st);
	    
	    switch(GET_OPCODE(prog[pc].op)) {
	    case SLJIT_MSHL:
		s &= SHIFTM_32;
                // fall through		
	    case SLJIT_SHL:
		c = a << s;
		if ((prog[pc].op & SLJIT_SHIFT_INTO_NON_ZERO) || (s != 0))
		    c |= ((b >> 1) >> (s ^ SHIFTM_32));
		break;
	    case SLJIT_MLSHR:
		s &= SHIFTM_32;
                // fall through		
	    case SLJIT_LSHR:
		c = a >> s;
		if ((prog[pc].op & SLJIT_SHIFT_INTO_NON_ZERO) || (s != 0))
		    c |= ((b << 1) << (s ^ SHIFTM_32));
		break;
	    }
	    store_uw(c, prog[pc].dst, prog[pc].dstw, st);
	}
	break;
    }
	
    case FMT_FOP1: {
	switch(GET_OPCODE(prog[pc].op)) {
	case SLJIT_MOV_F64: {
	    sljit_f64 a;
	    load_f64(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_f64(a, prog[pc].dst, prog[pc].dstw, st);
	    break;
	}
	case SLJIT_MOV_F32: {
	    sljit_f32 a;
	    load_f32(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_f32(a, prog[pc].dst, prog[pc].dstw, st);
	    break;
	}
	case SLJIT_CONV_F64_FROM_F32: {
	    sljit_f32 a;
	    sljit_f64 b;
	    load_f32(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = a;
	    store_f64(b, prog[pc].dst, prog[pc].dstw, st);
	    break;
	}
	case SLJIT_CONV_F32_FROM_F64: {
	    sljit_f64 a;
	    sljit_f32 b;
	    load_f64(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = a;
	    store_f32(b, prog[pc].dst, prog[pc].dstw, st);
	    break;	
	}
	case SLJIT_CONV_SW_FROM_F64: {
	    sljit_f64 a;
	    sljit_sw  b;
	    load_f64(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = (sljit_sw) trunc(a);
	    store_sw(b, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}
	case SLJIT_CONV_SW_FROM_F32: {
	    sljit_f32 a;
	    sljit_sw  b;
	    load_f32(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = (sljit_sw) truncf(a);
	    store_sw(b, prog[pc].dst, prog[pc].dstw, st);
	    break;
	}
	case SLJIT_CONV_S32_FROM_F64: {
	    sljit_f64 a;
	    sljit_s32  b;
	    load_f64(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = (sljit_s32) trunc(a);
	    store_s32(b, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}
	case SLJIT_CONV_S32_FROM_F32: {
	    sljit_f32 a;
	    sljit_s32  b;
	    load_f32(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = (sljit_s32) truncf(a);
	    store_s32(b, prog[pc].dst, prog[pc].dstw, st);
	    break;
	}
	case SLJIT_CONV_F64_FROM_SW: {
	    sljit_sw a;
	    sljit_f64 b;
	    load_sw(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = (sljit_f64) a;
	    store_f64(b, prog[pc].dst, prog[pc].dstw, st);
	    break;
	}
	case SLJIT_CONV_F32_FROM_SW: {
	    sljit_sw a;
	    sljit_f32 b;
	    load_sw(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = (sljit_f32) a;
	    store_f32(b, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}
	case SLJIT_CONV_F64_FROM_S32: {
	    sljit_s32 a;
	    sljit_f64 b;
	    load_s32(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = (sljit_f64) a;
	    store_f64(b, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}
	case SLJIT_CONV_F32_FROM_S32: {
	    sljit_s32 a;
	    sljit_f32 b;
	    load_s32(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = (sljit_f32) a;
	    store_f32(b, prog[pc].dst, prog[pc].dstw, st);
	    break;	    	    
	}
	case SLJIT_CONV_F64_FROM_UW: {
	    sljit_uw a;
	    sljit_f64 b;
	    load_uw(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = (sljit_f64) a;
	    store_f64(b, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}
	case SLJIT_CONV_F32_FROM_UW: {
	    sljit_uw a;
	    sljit_f32 b;
	    load_uw(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = (sljit_f32) a;
	    store_f32(b, prog[pc].dst, prog[pc].dstw, st);
	    break;	    	    
	}
	case SLJIT_CONV_F64_FROM_U32: {
	    sljit_u32 a;
	    sljit_f64 b;
	    load_u32(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = (sljit_f64) a;
	    store_f64(b, prog[pc].dst, prog[pc].dstw, st);
	    break;	    	    
	}
	case SLJIT_CONV_F32_FROM_U32: {
	    sljit_u32 a;
	    sljit_f32 b;
	    load_u32(&a, prog[pc].src1, prog[pc].src1w, st);
	    b = (sljit_f32) a;
	    store_f32(b, prog[pc].dst, prog[pc].dstw, st);
	    break;	    	    	    
	}
	case SLJIT_CMP_F64: {
	    sljit_f64 a, b;
	    load_f64(&a, prog[pc].dst, prog[pc].dstw, st);
	    load_f64(&b, prog[pc].src1, prog[pc].src1w, st);
	    cmp_f64(&st->flags, a, b);
	    break;
	}
	case SLJIT_CMP_F32: {
	    sljit_f32 a, b;
	    load_f32(&a, prog[pc].dst, prog[pc].dstw, st);
	    load_f32(&b, prog[pc].src1, prog[pc].src1w, st);
	    cmp_f32(&st->flags, a, b);
	    break;
	}
	case SLJIT_NEG_F64: {
	    sljit_f64 a;
	    load_f64(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_f64(-a, prog[pc].dst, prog[pc].dstw, st);
	    break;
	}
	case SLJIT_NEG_F32: {
	    sljit_f32 a;
	    load_f32(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_f32(-a, prog[pc].dst, prog[pc].dstw, st);
	    break;
	}
	case SLJIT_ABS_F64: {
	    sljit_f64 a;
	    load_f64(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_f64(fabs(a), prog[pc].dst, prog[pc].dstw, st);
	    break;
	}
	case SLJIT_ABS_F32: {
	    sljit_f32 a;
	    load_f32(&a, prog[pc].src1, prog[pc].src1w, st);
	    store_f32(fabsf(a), prog[pc].dst, prog[pc].dstw, st);
	    break;
	}
	default: break;
	}
	break;
    }
    case FMT_FOP2: {
	switch(GET_OPCODE(prog[pc].op)) {
	case SLJIT_ADD_F64: {
	    sljit_f64 a, b, c;
	    load_f64(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_f64(&b, prog[pc].src2, prog[pc].src2w, st);	    
	    c = a+b;
	    store_f64(c, prog[pc].dst, prog[pc].dstw, st);
	    break;
	}
	case SLJIT_ADD_F32: {
	    sljit_f32 a, b, c;
	    load_f32(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_f32(&b, prog[pc].src2, prog[pc].src2w, st);	    
	    c = a+b;
	    store_f32(c, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}
	case SLJIT_SUB_F64: {
	    sljit_f64 a, b, c;
	    load_f64(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_f64(&b, prog[pc].src2, prog[pc].src2w, st);	    
	    c = a-b;
	    store_f64(c, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}
	case SLJIT_SUB_F32: {
	    sljit_f32 a, b, c;
	    load_f32(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_f32(&b, prog[pc].src2, prog[pc].src2w, st);	    
	    c = a-b;
	    store_f32(c, prog[pc].dst, prog[pc].dstw, st);
	    break;	    	    
	}
	case SLJIT_MUL_F64: {
	    sljit_f64 a, b, c;
	    load_f64(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_f64(&b, prog[pc].src2, prog[pc].src2w, st);	    
	    c = a*b;
	    store_f64(c, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}
	case SLJIT_MUL_F32: {
	    sljit_f32 a, b, c;
	    load_f32(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_f32(&b, prog[pc].src2, prog[pc].src2w, st);	    
	    c = a*b;
	    store_f32(c, prog[pc].dst, prog[pc].dstw, st);
	    break;	    	    
	}
	case SLJIT_DIV_F64: {
	    sljit_f64 a, b, c;
	    load_f64(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_f64(&b, prog[pc].src2, prog[pc].src2w, st);	    
	    c = a/b;
	    store_f64(c, prog[pc].dst, prog[pc].dstw, st);
	    break;	    
	}
	case SLJIT_DIV_F32: {
	    sljit_f32 a, b, c;
	    load_f32(&a, prog[pc].src1, prog[pc].src1w, st);
	    load_f32(&b, prog[pc].src2, prog[pc].src2w, st);	    
	    c = a/b;
	    store_f32(c, prog[pc].dst, prog[pc].dstw, st);
	    break;	    	    
	}
	default: break;
	}
	break;
    }
    case FMT_CONST: {
	switch(GET_OPCODE(prog[pc].op)) {
	case SLJIT_MOV:
	    store_sw(prog[pc].sw_val, prog[pc].dst, prog[pc].dstw, st);
	    break;
	case SLJIT_MOV32:
	case SLJIT_MOV_S32:
	    store_s32(prog[pc].sw_val, prog[pc].dst, prog[pc].dstw, st); 
	    break;
	case SLJIT_MOV_U8:
	case SLJIT_MOV32_U8:
	    store_u8(prog[pc].sw_val, prog[pc].dst, prog[pc].dstw, st);
	    break;	    	    
	}	    
	break;
    }
    case FMT_RETURN_VOID:
	DUMP_STATE(st, pc);
	leave(st, &prog[pc], &pc);
	DUMP_STATE(st, pc);
	if (pc == -1)
	    return;
	break;
    case FMT_RETURN_TO:
	DUMP_STATE(st, pc);	
	return;

    default:
	break;
    }
ignore:
    pc++;
    goto next;
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_cmp_info(sljit_s32 type)
{
    UNUSED(type);
    return 0;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_get_register_index(sljit_s32 type, sljit_s32 reg)
{
    switch(type) {
    case SLJIT_GP_REGISTER:
	return (reg-1) & 0xf;
    case SLJIT_FLOAT_REGISTER:
	return (reg-1) & 0xf;
    case SLJIT_SIMD_REG_64:
    case SLJIT_SIMD_REG_128:
    case SLJIT_SIMD_REG_256:
    case SLJIT_SIMD_REG_512:
	return (reg-1) & 0xf;
    default:
	return -1;
    }
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_op_custom(struct sljit_compiler *compiler,
							void *instruction, sljit_u32 size)
{
    UNUSED(compiler);
    UNUSED(instruction);
    UNUSED(size);    
    // ignore
    return SLJIT_SUCCESS;    
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_run(emulator_state_t* st,
					     void* code, size_t code_size,
					     void* addr,
					     void* args0,
					     sljit_s32 at, void* ret0)
{
    int i;
    int ri = 0;
    int fri = 0;
    arg_val_t* arg = (arg_val_t*) args0;
    arg_val_t* ret = (arg_val_t*) ret0;
    int n = arg_types_argc(at);

    // load arguments into registers
    for (i = 0; i < n; i++) {
	switch(ARGTYPE(at, i)) {
	case SLJIT_ARG_TYPE_W:
	case SLJIT_ARG_TYPE_W_R:
	case SLJIT_ARG_TYPE_TERM:
	case SLJIT_ARG_TYPE_TERM_R:
	    st->r[ri++].sw = arg[i].sw;
	    break;
	case SLJIT_ARG_TYPE_32:
	case SLJIT_ARG_TYPE_32_R:
	    st->r[ri++].s32 = arg[i].sw; break;
	case SLJIT_ARG_TYPE_P:
	case SLJIT_ARG_TYPE_P_R:
	    st->r[ri++].uw = arg[i].sw; break;
	case SLJIT_ARG_TYPE_F64:
	    st->fr[fri++].f64 = arg[i].f64; break;
	case SLJIT_ARG_TYPE_F32:
	    st->fr[fri++].f32 = arg[i].f64; break;
	default: // bad argument
	    return -1;
	}
    }

    st->call = -1; // no direct emulated caller
    emu(st, (sljitter_inst_t*) code, code_size, (sljit_sw) addr);

    // load return value from r[0]/fr[0]
    switch(ARGTYPE_RET(at)) {
    case SLJIT_ARG_TYPE_RET_VOID:
	ret->sw = 0; break;
    case SLJIT_ARG_TYPE_TERM:
    case SLJIT_ARG_TYPE_TERM_R:
    case SLJIT_ARG_TYPE_W:
    case SLJIT_ARG_TYPE_W_R:
	ret->sw = st->r[0].sw; break;
    case SLJIT_ARG_TYPE_32:
    case SLJIT_ARG_TYPE_32_R:
	ret->sw = st->r[0].s32; break;
    case SLJIT_ARG_TYPE_F32:
	ret->f64 = st->fr[0].f32; break;
    case SLJIT_ARG_TYPE_F64:
	ret->f64 = st->fr[0].f64; break;
    case SLJIT_ARG_TYPE_P:
    case SLJIT_ARG_TYPE_P_R:
	ret->sw = (sljit_sw) st->r[0].uw; break;
    default:
	break;
    }
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE const char* sljit_get_platform_name(void)
{
    return "emulator";
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

static void set_context(sljitter_inst_t* ip,
			sljit_s32 options,
			sljit_s32 arg_types,
			sljit_s32 scratches,
			sljit_s32 saveds,
			sljit_s32 local_size)
{
    ip->options = options;
    ip->arg_types = arg_types;
    ip->scratches = scratches;
    ip->saveds = saveds;
    ip->local_size = local_size;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_enter(struct sljit_compiler *compiler,
						    sljit_s32 options,
						    sljit_s32 arg_types,
						    sljit_s32 scratches,
						    sljit_s32 saveds,
						    sljit_s32 local_size)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_ENTER, 0, 0);
    set_context(ip, options, arg_types, scratches, saveds, local_size);
    set_emit_enter(compiler, options, arg_types, scratches, saveds, local_size);
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_set_context(struct sljit_compiler *compiler,
						     sljit_s32 options,
						     sljit_s32 arg_types,
						     sljit_s32 scratches,
						     sljit_s32 saveds,
						     sljit_s32 local_size)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_SET_CONTEXT, 0, 0);
    set_context(ip, options, arg_types, scratches, saveds, local_size);
    set_emit_enter(compiler, options, arg_types, scratches, saveds, local_size);
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_op0(struct sljit_compiler *compiler, sljit_s32 op)
{
    new_inst(compiler, FMT_OP0, op, 0);
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_op1(struct sljit_compiler *compiler, sljit_s32 op,
			 sljit_s32 dst, sljit_sw dstw,
			 sljit_s32 src, sljit_sw srcw)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_OP1, op, 0);
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_OP2, op, 0);    
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_OP2U, op, 0);    
    ip->src1   = src1;
    ip->src1w  = src1w;
    ip->src2   = src2;
    ip->src2w  = src2w;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;    
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_op2r(struct sljit_compiler *compiler, sljit_s32 op,
			  sljit_s32 dst_reg,
			  sljit_s32 src1, sljit_sw src1w,
			  sljit_s32 src2, sljit_sw src2w)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_OP2R, op, 0);        
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_SHIFT_INTO, op, 0);
    ip->dst = dst_reg;
    ip->dstw = 0;
    ip->src1 = src1_reg;
    ip->src1w = 0;
    ip->src2 = src2_reg;
    ip->src2w = 0;
    ip->src3 = src3;
    ip->src3w = src3w;
    INC_SIZE(1);
    return SLJIT_SUCCESS;    
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_op_src(struct sljit_compiler *compiler, sljit_s32 op,
			    sljit_s32 src, sljit_sw srcw)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_OP_SRC, op, 0);    
    ip->src1 = src;
    ip->src1w = srcw;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;    
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_op_dst(struct sljit_compiler *compiler,
			    sljit_s32 op,
			    sljit_s32 dst, sljit_sw dstw)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_OP_DST, op, 0);        
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_FOP1, op, 0);    
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_FOP2, op, 0);
    ip->dst = dst;
    ip->dstw = dstw;
    ip->src1 = src1;
    ip->src1w = src1w;
    ip->src2 = src2;
    ip->src2w = src2w;
    INC_SIZE(1);    
    return SLJIT_SUCCESS;    
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_fset32(struct sljit_compiler *compiler,
			    sljit_s32 freg, sljit_f32 value)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_FSET32, 0, 0);
    ip->dst = freg;
    ip->f32_val = value;
    INC_SIZE(1);
    return SLJIT_SUCCESS;            
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_fset64(struct sljit_compiler *compiler,
			    sljit_s32 freg, sljit_f64 value)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_FSET64, 0, 0);
    ip->dst = freg;
    ip->f64_val = value;
    INC_SIZE(1);
    return SLJIT_SUCCESS;                
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_fcopy(struct sljit_compiler *compiler,
			   sljit_s32 op,
			   sljit_s32 freg, sljit_s32 reg)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_FCOPY, op, 0);
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

    sljit_reset_read_only_buffers(buffers);    
    
    if (compiler->last_label && (compiler->last_label->size == compiler->size))
	return compiler->last_label;
    label = (struct sljit_label*)ensure_abuf(compiler,sizeof(struct sljit_label));
    set_label(label, compiler);
    return label;    
}


SLJIT_API_FUNC_ATTRIBUTE struct sljit_jump* sljit_emit_jump(struct sljit_compiler *compiler, sljit_s32 type)
{
    struct sljit_jump* jump;
    new_inst(compiler, FMT_JUMP, 0, type);
    jump = (struct sljit_jump*)ensure_abuf(compiler, sizeof(struct sljit_jump));
    set_jump(jump, compiler,  type & SLJIT_REWRITABLE_JUMP);
    jump->addr = compiler->size; // (source address)
    INC_SIZE(1);
    return jump;
}

SLJIT_API_FUNC_ATTRIBUTE struct sljit_jump* sljit_emit_call(struct sljit_compiler *compiler, sljit_s32 type, sljit_s32 arg_types)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_CALL, 0, type);
    struct sljit_jump* jump;
    ip->arg_types = arg_types;
    jump = (struct sljit_jump*)ensure_abuf(compiler, sizeof(struct sljit_jump));
    jump->addr = compiler->size;
    INC_SIZE(1);
    return jump;    
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_return_void(struct sljit_compiler *compiler)
{
    sljitter_inst_t* ip;    
    sljit_s32 scratches;
    sljit_s32 saveds;
    // recreate from enter!
    scratches =
	(compiler->scratches & 0xff) |
	((compiler->fscratches & 0xff) << 8) |
	((compiler->vscratches & 0xff) << 16);
    saveds =
	(compiler->saveds & 0xff) |
	((compiler->fsaveds & 0xff) << 8) |
	((compiler->vsaveds & 0xff) << 16);
    ip = new_inst(compiler, FMT_RETURN_VOID, 0, 0);
    set_context(ip, compiler->options, 0,
		scratches, saveds, 0);
    DBG_TRACE("emit_return_void scratches%x, saveds=%x",
	      scratches, saveds);
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_return_to(struct sljit_compiler *compiler,
	sljit_s32 src, sljit_sw srcw)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_RETURN_TO, 0, 0);
    ip->src1 = src;
    ip->src1w = srcw;
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_ijump(struct sljit_compiler *compiler,
			   sljit_s32 type, sljit_s32 src, sljit_sw srcw)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_IJUMP, 0, type);
    ip->src1 = src;
    ip->src1w = srcw;
    INC_SIZE(1);
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_icall(struct sljit_compiler *compiler,
						    sljit_s32 type, sljit_s32 arg_types,
						    sljit_s32 src, sljit_sw srcw)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_ICALL, 0, type);
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_OP_FLAGS, op, type);
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_SELECT, 0, type);    
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_FSELECT, 0, type); 
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
    if (!(reg & REG_PAIR_MASK))
	return sljit_emit_mem_unaligned(compiler, type, reg, mem, memw);
    else {
	sljitter_inst_t* ip = new_inst(compiler, FMT_MEM, 0, type);     
	ip->dst = reg;
	ip->src1 = mem;
	ip->src1w = memw;
	INC_SIZE(1);
	return SLJIT_SUCCESS;
    }
}

/*
SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_mem_update(struct sljit_compiler *compiler,
				sljit_s32 type,
				sljit_s32 reg,
				sljit_s32 mem, sljit_sw memw)
{
    sljitter_inst_t* ip = new_inst(compiler, FMT_MEM_UPDATE, 0, type);     
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_FMEM, 0, type);     
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_FMEM_UPDATE, 0, type);     
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_SIMD_MOV, 0, type); 
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_SIMD_REPLICATE, 0, type);
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_SIMD_LANE_MOV, 0, type);
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_SIMD_LANE_REPLICATE, 0, type);
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_SIMD_EXTEND, 0, type);    
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_SIMD_SIGN, 0, type);    
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_SIMD_OP2, 0, type);        
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_ATOMIC_LOAD, op, 0);
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_ATOMIC_STORE, op, 0);
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_CONST, op, 0);
    struct sljit_const *const_;
    
    const_ = (struct sljit_const*)ensure_abuf(compiler, sizeof(struct sljit_const));
    set_const(const_, compiler);
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
    sljitter_inst_t* ip = new_inst(compiler, FMT_OP_ADDR, op, 0);
    struct sljit_jump* jump;
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
    if (executable_offset == 0)
	*((sljit_uw*) addr) = new_target;
}

SLJIT_API_FUNC_ATTRIBUTE void sljit_set_const(sljit_uw addr, sljit_s32 op, sljit_sw new_constant, sljit_sw executable_offset)
{
    if (executable_offset == 0) {
	switch(op) {
	case SLJIT_MOV:
	    *((sljit_sw*) addr) = (sljit_sw) new_constant;
	    break;
	case SLJIT_MOV32:
	case SLJIT_MOV_S32:
	    *((sljit_s32*) addr) = (sljit_s32)new_constant;
	    break;	    	    
	case SLJIT_MOV_U8:
	case SLJIT_MOV32_U8:	    
	    *((sljit_u8*) addr) = (sljit_u8)new_constant;
	    break;	    	    
	}
    }
}

SLJIT_API_FUNC_ATTRIBUTE void* sljit_generate_code(struct sljit_compiler *compiler, sljit_s32 options, void *exec_allocator_data)
{
    UNUSED(options);
    UNUSED(exec_allocator_data);
    struct sljit_memory_fragment *buf;
    sljitter_inst_t* code;
    sljitter_inst_t* dst;
    struct sljit_label* label;
    struct sljit_jump* jump;
    struct sljit_const* const_;
    
    if (!compiler->size)
	return NULL;
    code = (sljitter_inst_t*) malloc(sizeof(sljitter_inst_t)*compiler->size);
    
    reverse_buf(compiler);
    buf = compiler->buf;
    // label = compiler->labels;

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

    label = compiler->labels;
    while (label) {
	label->u.addr = label->size; // addr=instruction pos (same size!)
	label = label->next;
    }
    
    jump = compiler->jumps;
    while (jump) {
	sljit_uw addr = jump->addr;
	switch(code[addr].fmt) {
	case FMT_CMP:
	case FMT_CALL:
	case FMT_JUMP: {
	    struct sljit_label* target_label = jump->u.label;
	    sljit_uw new_target = target_label->u.addr;
	    DBG_TRACE("generate_code: patch addr=%lu target=%lu",
		      addr, new_target);
	    code[addr].target = new_target;  // absolute jump!
	    // allow patch with set_const! (not a jumping target!!!)
	    // target_label->u.addr = (sljit_uw) &code[addr].target;
	    break;
	}
	default:
	    break;
	}
	jump = jump->next;	
    }

    // second pass make patchable jumps
    jump = compiler->jumps;
    while (jump) {
	sljit_uw addr = jump->addr;
	switch(code[addr].fmt) {
	case FMT_CMP:
	case FMT_CALL:	    
	case FMT_JUMP: {
	    struct sljit_label* target_label = jump->u.label;
	    target_label->u.addr = (sljit_uw) &code[addr].target;
	    break;
	}
	default:
	    break;
	}
	jump = jump->next;	
    }    

    const_ = compiler->consts;
    while (const_) {
	sljit_uw addr = const_->addr;
	switch(code[addr].fmt) {
	case FMT_CONST: {
	    const_->addr = (sljit_uw) &code[addr].sw_val;
	    break;
	}
	default:
	    break;
	}
	const_ = const_->next;	
    }    
    return (void*)code;    
}
