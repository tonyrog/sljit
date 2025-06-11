/*
 * Extra SIMD operations like ADD, SUB, MUL, ...
 */

#include "sljitSimd.h"

// SIMD
#define PADDB  (0xfc|EX86_PREF_66)
#define PADDW  (0xfd|EX86_PREF_66)
#define PADDD  (0xfe|EX86_PREF_66)
#define PADDQ  (0xd4|EX86_PREF_66)
#define ADDPD  (0x58|EX86_PREF_66)
#define ADDPS  (0x58)

#define PSUBB  (0xf8|EX86_PREF_66)
#define PSUBW  (0xf9|EX86_PREF_66)
#define PSUBD  (0xfa|EX86_PREF_66)
#define PSUBQ  (0xfb|EX86_PREF_66)
#define SUBPD  (0x5c|EX86_PREF_66)
#define SUBPS  (0x5c)

#define PMULB  0
#define PMULW  (0xd5|EX86_PREF_66)
#define PMULD  (0x40|EX86_PREF_66|VEX_OP_0F38)   // (SSE4_1)
#define PMULQ  0
#define MULPD  (0x59|EX86_PREF_66)
#define MULPS  (0x59)

#define PSLLB_i8  0
#define PSLLW_i8  (0x71|EX86_PREF_66)
#define PSLLD_i8  (0x72|EX86_PREF_66)
#define PSLLQ_i8  (0x73|EX86_PREF_66)

#define PSLLB  0
#define PSLLW  (0xf1|EX86_PREF_66)
#define PSLLD  (0xf2|EX86_PREF_66)
#define PSLLQ  (0xf3|EX86_PREF_66)

#define PSRLB_i8  0
#define PSRLW_i8  (0x71|EX86_PREF_66)
#define PSRLD_i8  (0x72|EX86_PREF_66)
#define PSRLQ_i8  (0x73|EX86_PREF_66)

#define PSRLB  0
#define PSRLW  (0xd1|EX86_PREF_66)
#define PSRLD  (0xd2|EX86_PREF_66)
#define PSRLQ  (0xd3|EX86_PREF_66)

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_simd_arith_op2(
    struct sljit_compiler *compiler,
    sljit_s32 type,
    sljit_s32 dst_vreg, sljit_s32 src1_vreg, sljit_s32 src2, sljit_sw src2w)
{
    sljit_s32 reg_size = SLJIT_SIMD_GET_REG_SIZE(type);
    sljit_s32 elem_size = SLJIT_SIMD_GET_ELEM_SIZE(type);
    sljit_s32 use_vex = (cpu_feature_list & CPU_FEATURE_AVX) && (compiler->options & SLJIT_ENTER_USE_VEX);
    sljit_uw op = 0;
    sljit_uw mov_op = 0;
    sljit_s32 simd_op;

    simd_op = SLJIT_SIMD_GET_OPCODE(type);
    fprintf(stderr, "simd_arith_op2: type:%x, simd_op:%d, reg_size:%d, elem_size:%d, use_vex:%d\r\n",
	    type, simd_op, reg_size, elem_size, use_vex);
    
    CHECK_ERROR();
    CHECK(check_sljit_emit_simd_op2(compiler, type, dst_vreg, src1_vreg, src2, src2w));
    ADJUST_LOCAL_OFFSET(src2, src2w);
    
#if (defined SLJIT_CONFIG_X86_64 && SLJIT_CONFIG_X86_64)
    compiler->mode32 = 1;
#endif /* SLJIT_CONFIG_X86_64 */
    
    if (reg_size == 5) {
	if (!(cpu_feature_list & CPU_FEATURE_AVX2))
	    return SLJIT_ERR_UNSUPPORTED;
    } else if (reg_size != 4)
	return SLJIT_ERR_UNSUPPORTED;
    
    if ((type & SLJIT_SIMD_FLOAT) && (elem_size < 2 || elem_size > 3))
	return SLJIT_ERR_UNSUPPORTED;
    switch (simd_op) {
    case SLJIT_SIMD_ARITH_OP2_ADD:
	if (type & SLJIT_SIMD_FLOAT) {
	    switch(elem_size) {
	    case ELEM_32:  op = ADDPS; break;
	    case ELEM_64:  op = ADDPD; break;
	    default: break;
	    }
	}
	else {
	    switch(elem_size) {
	    case ELEM_8:   op = PADDB; break;
	    case ELEM_16:  op = PADDW; break;
	    case ELEM_32:  op = PADDD; break;
	    case ELEM_64:  op = PADDQ; break;
	    case ELEM_128:
	    case ELEM_256:
	    default: break;
	    }
	}
	break;
	
    case SLJIT_SIMD_ARITH_OP2_SUB:
	if (type & SLJIT_SIMD_FLOAT) {
	    switch(elem_size) {
	    case ELEM_32:  op = SUBPS; break;
	    case ELEM_64:  op = SUBPD; break;
	    default: break;
	    }
	}
	else {
	    switch(elem_size) {
	    case ELEM_8:   op = PSUBB; break;
	    case ELEM_16:  op = PSUBW; break;
	    case ELEM_32:  op = PSUBD; break;
	    case ELEM_64:  op = PSUBQ; break;
	    case ELEM_128:
	    case ELEM_256:
	    default: break;
	    }
	}
	break;
	
    case SLJIT_SIMD_ARITH_OP2_MUL:
	if (type & SLJIT_SIMD_FLOAT) {
	    switch(elem_size) {
	    case ELEM_32:  op = MULPS; break;
	    case ELEM_64:  op = MULPD; break;
	    default: break;
	    }
	}
	else {
	    switch(elem_size) {
	    case ELEM_8:   op = PMULB; break;
	    case ELEM_16:  op = PMULW; break;
	    case ELEM_32:  op = PMULD; break;
	    case ELEM_64:  op = PMULQ; break;
	    case ELEM_128:
	    case ELEM_256:
	    default: break;
	    }
	}
	break;	
	
    default:
	return SLJIT_ERR_UNSUPPORTED;	    
    }
    
    if (type & SLJIT_SIMD_TEST)
	return SLJIT_SUCCESS;

    if ((src2 & SLJIT_MEM) && SLJIT_SIMD_GET_ELEM2_SIZE(type) < reg_size) {
	mov_op = ((type & SLJIT_SIMD_FLOAT) ? (MOVUPS_x_xm | (elem_size == 3 ? EX86_PREF_66 : 0)) : (MOVDQU_x_xm | EX86_PREF_F3)) | EX86_SSE2;
	if (use_vex)
	    FAIL_IF(emit_vex_instruction(compiler, mov_op, TMP_FREG, 0, src2, src2w));
	else
	    FAIL_IF(emit_groupf(compiler, mov_op, TMP_FREG, src2, src2w));
	
	src2 = TMP_FREG;
	src2w = 0;
    }

    if ((reg_size == 5) || use_vex) {
	if (reg_size == 5)
	    op |= VEX_256;
	
	return emit_vex_instruction(compiler, op | EX86_SSE2 | VEX_SSE2_OPV, dst_vreg, src1_vreg, src2, src2w);
    }
    
    if (dst_vreg != src1_vreg) {
	if (dst_vreg == src2) {
	    if (SLJIT_SIMD_GET_OPCODE(type) == SLJIT_SIMD_OP2_SHUFFLE) {
		FAIL_IF(emit_simd_mov(compiler, type, TMP_FREG, src2));
		FAIL_IF(emit_simd_mov(compiler, type, dst_vreg, src1_vreg));
		src2 = TMP_FREG;
		src2w = 0;
	    } else
		src2 = src1_vreg;
	} else
	    FAIL_IF(emit_simd_mov(compiler, type, dst_vreg, src1_vreg));
    }
    
    if (op & (VEX_OP_0F38 | VEX_OP_0F3A))
	return emit_groupf_ext(compiler, op | EX86_SSE2, dst_vreg, src2, src2w);
    return emit_groupf(compiler, op | EX86_SSE2, dst_vreg, src2, src2w);
}



/*
 * implement PMULB
 */

/*
 T1 = temporary vector reg
 MOVDQA(T1, DST);
 PMULLW(T1, SRC);
 PSLLW(T1, 8);
 PSRLW(T1, 8);
    
 PSRLW(DST, 8);
 PSRLW(SRC, 8);  -- FIXME: do not modify SRC!
 PMULLW(DST, SRC);
 PSLLW(DST, 8);
 POR(DST, T1);
*/
