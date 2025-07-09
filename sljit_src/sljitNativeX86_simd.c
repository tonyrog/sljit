/*
 * Extra SIMD operations like ADD, SUB, MUL, ...
 */

#include "sljitSimd.h"

// SIMD
#define MOVAPS_ld (0x28)
#define MOVAPS_st (0x29)

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

#define PMULB  0  // emit_sse2_mul_int8 
#define PMULW  (0xd5|EX86_PREF_66)
#define PMULD  (0x40|EX86_PREF_66|VEX_OP_0F38)   // (SSE4_1)
#define PMULQ  0  // emit_sse2_mul_int64
#define MULPD  (0x59|EX86_PREF_66)
#define MULPS  (0x59)

#define DIVPD  (0x5e|EX86_PREF_66)   // 66 0F 5e
#define DIVPS  (0x5e)                // 0F 5e

#define PMULLW (0xd5|EX86_PREF_66)
#define PMULLD (0xe5|EX86_PREF_66)

#define PMULUDQ (0xf4|EX86_PREF_66)

#define PSLLBi  0  // emit_sse2_vssl_imm8
#define PSLLWi  (0x71|EX86_PREF_66)   // 66 0F 71 /6 ib
#define PSLLDi  (0x72|EX86_PREF_66)
#define PSLLQi  (0x73|EX86_PREF_66)

#define PSLLB  0
#define PSLLW  (0xf1|EX86_PREF_66)
#define PSLLD  (0xf2|EX86_PREF_66)
#define PSLLQ  (0xf3|EX86_PREF_66)

#define PSRLBi  0
#define PSRLWi  (0x71|EX86_PREF_66)   // 66 0F 71 /2 ib
#define PSRLDi  (0x72|EX86_PREF_66)
#define PSRLQi  (0x73|EX86_PREF_66)

#define PSRLB  0
#define PSRLW  (0xd1|EX86_PREF_66)
#define PSRLD  (0xd2|EX86_PREF_66)
#define PSRLQ  (0xd3|EX86_PREF_66)

#define PSRAB  0
#define PSRAW  (0xe1|EX86_PREF_66)
#define PSRAD  (0xe2|EX86_PREF_66)
#define PSRAQ  (0xe3|EX86_PREF_66)

#define PSRABi  0
#define PSRAWi  (0x71|EX86_PREF_66)  // 66 0F 71 /4 ib
#define PSRADi  (0x72|EX86_PREF_66)
#define PSRAQi  (0x73|EX86_PREF_66) // exist?

#define PUNPCKLBW (0x60|EX86_PREF_66)
#define PUNPCKLWD (0x61|EX86_PREF_66)
#define PUNPCKLDQ (0x62|EX86_PREF_66)

#define PACKUSWB (0x67|EX86_PREF_66)

#define PUNPCKHBW (0x68|EX86_PREF_66)
#define PUNPCKHWD (0x69|EX86_PREF_66)
#define PUNPCKHDQ (0x6a|EX86_PREF_66)

#define PCMPEQB   (0x74|EX86_PREF_66)
#define PCMPEQW   (0x75|EX86_PREF_66)
#define PCMPEQD   (0x76|EX86_PREF_66)
#define PCMPEQQ   0

#define PCMPGTB   (0x64|EX86_PREF_66)
#define PCMPGTW   (0x65|EX86_PREF_66)
#define PCMPGTD   (0x66|EX86_PREF_66)
#define PCMPGTQ   0
#define CMPPD     (0xc2|EX86_PREF_66)
#define CMPPS     (0xc2)
#define   CMP_EQ    0
#define   CMP_LT    1
#define   CMP_LTE   2
#define   CMP_UNORD 3
#define   CMP_NEQ   4
#define   CMP_GTE   5
#define   CMP_GT    6
#define   CMP_ORD   7

#define PAND (0xdb|EX86_PREF_66)
#define POR  (0xeb|EX86_PREF_66)
#define PXOR (0xef|EX86_PREF_66)

// SSSE3 (triple-S)
#define PABSB      (0x1c|EX86_PREF_66)  // 66 0F 38 1C
#define PABSW      (0x1d|EX86_PREF_66)  // 66 0F 38 1D
#define PABSD      (0x1e|EX86_PREF_66)  // 66 0F 38 1E



#define SRL  0x2 // 010
#define SRA  0x4 // 100
#define SLL  0x6 // 110

// 1100-0000
// 0001-0000
//
SLJIT_API_FUNC_ATTRIBUTE sljit_s32 emit_sse2_imm8(
    struct sljit_compiler *compiler, int op, int shop,
    sljit_s32 vreg, int8_t imm8)
{
    sljit_s32 ri;
    FAIL_IF(emit_byte(compiler, 0x66));       // Operand-size prefix för SSE2
    ri = sljit_get_register_index(SLJIT_SIMD_REG_128, vreg);
    if (ri > 7)
	FAIL_IF(emit_byte(compiler, REX_R));
    FAIL_IF(emit_byte(compiler, 0x0F));       // 2-byte opcode prefix
    FAIL_IF(emit_byte(compiler, op));
    FAIL_IF(emit_byte(compiler, 0xC0 | (shop<<3) | (ri & 0x7)));
    FAIL_IF(emit_byte(compiler, imm8));
    return SLJIT_SUCCESS;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 emit_sse2_vssl_imm8(
    struct sljit_compiler *compiler, sljit_s32 vreg, int8_t imm8)
{
    return SLJIT_SUCCESS;    
}

// elem_size = 8 bits, reg_size = 128
SLJIT_API_FUNC_ATTRIBUTE sljit_s32 emit_sse2_mul_int8(
    struct sljit_compiler *compiler,
    sljit_s32 type,
    sljit_s32 dst_vreg, sljit_s32 src1_vreg, sljit_s32 src2, sljit_sw src2w)
{
    sljit_s32 vr0, vr1, tmp1, tmp2;
    sljit_s32 r;
    
    if (type & SLJIT_SIMD_TEST)
	return SLJIT_SUCCESS;

    FAIL_IF(sljit_emit_simd_mov(compiler, type & ~SLJIT_SIMD_STORE,
				dst_vreg, src2, src2w));
    vr0 = dst_vreg;
    vr1 = src1_vreg;
    tmp1 = TMP_VREG1;
    tmp2 = TMP_VREG2;

    // vr0 *= vr1
    emit_simd_mov(compiler, type, tmp1, vr1);
    emit_groupf(compiler, PUNPCKLBW | EX86_SSE2, tmp1, vr1, 0);
    emit_groupf(compiler, PUNPCKHBW | EX86_SSE2, vr1,  vr1, 0);
    emit_simd_mov(compiler, type, tmp2, vr0);
    emit_groupf(compiler, PUNPCKLBW | EX86_SSE2, tmp2, vr0, 0);
    emit_groupf(compiler, PUNPCKHBW | EX86_SSE2, vr0, vr0, 0);

    emit_groupf(compiler, PMULLW | EX86_SSE2, tmp1, tmp2, 0);
    emit_groupf(compiler, PMULLW | EX86_SSE2, vr1, vr0, 0);

    emit_groupf(compiler, PCMPEQD | EX86_SSE2, tmp2, tmp2, 0);
    emit_sse2_imm8(compiler, PSRLWi, SRL, tmp2, 8);

    emit_groupf(compiler, PAND | EX86_SSE2, tmp1, tmp2, 0);
    emit_simd_mov(compiler, type, vr0, tmp1);
    emit_groupf(compiler, PAND | EX86_SSE2, tmp2, vr1, 0);    
    r = emit_groupf(compiler, PACKUSWB | EX86_SSE2, vr0, tmp2, 0);
    return r;
}

#define TMP_VREG3 SLJIT_VR(13)

// elem_size = 8 bits, reg_size = 128
SLJIT_API_FUNC_ATTRIBUTE sljit_s32 emit_sse2_mul_int64(
    struct sljit_compiler *compiler,
    sljit_s32 type,
    sljit_s32 dst_vreg, sljit_s32 src1_vreg, sljit_s32 src2, sljit_sw src2w)
{
    sljit_s32 vr0, vr1, tmp1, tmp2, tmp3;
    sljit_s32 r;
    
    if (type & SLJIT_SIMD_TEST)
	return SLJIT_SUCCESS;

    FAIL_IF(sljit_emit_simd_mov(compiler, type & ~SLJIT_SIMD_STORE,
				dst_vreg, src2, src2w));
    vr0 = dst_vreg;
    vr1 = src1_vreg;
    tmp1 = TMP_VREG1;
    tmp2 = TMP_VREG2;
    tmp3 = TMP_VREG3;

    emit_simd_mov(compiler, type, tmp1, vr0);
    emit_simd_mov(compiler, type, tmp2, vr1);

    emit_groupf(compiler, PMULUDQ | EX86_SSE2, vr0, vr1, 0);
    emit_simd_mov(compiler, type, vr1, tmp1);

    emit_sse2_imm8(compiler, PSRLQi, SRL, vr1, 0x20);
    emit_simd_mov(compiler, type, tmp3, tmp2);
    emit_sse2_imm8(compiler, PSRLQi, SRL, tmp3, 0x20);

    emit_groupf(compiler, PMULUDQ | EX86_SSE2, vr1, tmp2, 0);
    emit_groupf(compiler, PMULUDQ | EX86_SSE2, tmp1, tmp3, 0);    
    emit_groupf(compiler, PADDQ | EX86_SSE2, vr1, tmp1, 0);

    emit_sse2_imm8(compiler, PSRLQi, SRL, vr1, 0x20);
    r = emit_groupf(compiler, PADDQ | EX86_SSE2, vr0, vr1, 0);
    return r;
}

SLJIT_API_FUNC_ATTRIBUTE sljit_s32 emit_sse2_psraq_imm8(
    struct sljit_compiler *compiler, sljit_s32 type, sljit_s32 vreg, uint8_t imm8)
{
    sljit_s32 r;
    sljit_s32 tmp1;
    
    tmp1 = TMP_VREG1;
    emit_simd_mov(compiler, type, tmp1, vreg);           // tmp1 = xmm0
    emit_sse2_imm8(compiler, PSRLQi, SRL, vreg, imm8);   // psrlq       xmm0, imm8
    emit_groupf(compiler, PSHUFD_x_xm | EX86_PREF_66 | EX86_SSE2, tmp1, vreg,
		0b11110101);  //     pshufd      tmp1, xmm1, 0b11110101
    emit_sse2_imm8(compiler, PSRADi, SRA, tmp1, 31);  // psrad       tmp1, 31
    emit_sse2_imm8(compiler, PSLLQi, SLL, tmp1, 32);  // psllq       tmp1, 32
    emit_sse2_imm8(compiler, PSRLQi, SRL, tmp1, imm8); // psrlq       tmp1, imm8
    r = emit_groupf(compiler, POR | EX86_SSE2, vreg, tmp1, 0); // por         vreg, tmp1
    return r;
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_simd_arith_op1(
    struct sljit_compiler *compiler,
    sljit_s32 type,
    sljit_s32 dst_vreg, sljit_s32 src2, sljit_sw src2w)
{
    sljit_s32 reg_size = SLJIT_SIMD_GET_REG_SIZE(type);
    sljit_s32 elem_size = SLJIT_SIMD_GET_ELEM_SIZE(type);
    sljit_s32 use_vex =
	(cpu_feature_list & CPU_FEATURE_AVX) &&
	(compiler->options & SLJIT_ENTER_USE_VEX);
    sljit_uw op = 0;
    sljit_uw xop = 0;    

    CHECK_ERROR();
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

    switch (SLJIT_SIMD_GET_OPCODE(type)) {
    case SLJIT_SIMD_ARITH_OP1_NEG: 	// dst_vreg = 0 - src2[src2w]
	if (type & SLJIT_SIMD_FLOAT) {
	    switch(elem_size) {
	    case ELEM_32:  op = SUBPS; break;
	    case ELEM_64:  op = SUBPD; break;
	    default: return SLJIT_ERR_UNSUPPORTED;
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
	    default: return SLJIT_ERR_UNSUPPORTED;
	    }
	}
	xop = (type & SLJIT_SIMD_FLOAT) ? (XORPD_x_xm | EX86_PREF_66) :
	    (PXOR_x_xm| EX86_PREF_66);
	emit_groupf(compiler, xop | EX86_SSE2, dst_vreg, dst_vreg, 0);
	break;
    case SLJIT_SIMD_ARITH_OP1_NOT: // dst_vreg = 1 ^ src2[src2w]
	emit_groupf(compiler, PCMPEQD | EX86_SSE2, dst_vreg, dst_vreg, 0);
	op = (type & SLJIT_SIMD_FLOAT) ?
	    (XORPD_x_xm | EX86_PREF_66):
	    (PXOR_x_xm | EX86_PREF_66);
	break;
    case SLJIT_SIMD_ARITH_OP1_ABS:
	// tmp1 = src2 < 0
	// tmp2 = -src2
	// tmp1 = tmp2 & mask;
	// tmp2 = src2 & ~mask;
	// res  = tmp1 | tmp2;
	return SLJIT_ERR_UNSUPPORTED;	
    default:
	return SLJIT_ERR_UNSUPPORTED;
    }
    
    if (type & SLJIT_SIMD_TEST)
	return SLJIT_SUCCESS; 

    if ((src2 & SLJIT_MEM) && SLJIT_SIMD_GET_ELEM2_SIZE(type) < reg_size) {
	sljit_uw mov_op = ((type & SLJIT_SIMD_FLOAT) ?
			   (MOVUPS_x_xm | (elem_size == 3 ? EX86_PREF_66:0)) :
			   (MOVDQU_x_xm | EX86_PREF_F3)) | EX86_SSE2;
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
	return emit_vex_instruction(compiler, op | EX86_SSE2 | VEX_SSE2_OPV, dst_vreg, dst_vreg, src2, src2w);
    }
    
//    if (dst_vreg != src1_vreg) {
//	if (dst_vreg == src2)
//	    src2 = src1_vreg;
//	else
//	    FAIL_IF(emit_simd_mov(compiler, type, dst_vreg, src1_vreg));
//    }
    if (op & (VEX_OP_0F38 | VEX_OP_0F3A))
	return emit_groupf_ext(compiler, op | EX86_SSE2, dst_vreg, src2, src2w);
    return emit_groupf(compiler, op | EX86_SSE2, dst_vreg, src2, src2w);    
}


SLJIT_API_FUNC_ATTRIBUTE sljit_s32 sljit_emit_simd_arith_op2(
    struct sljit_compiler *compiler,
    sljit_s32 type,
    sljit_s32 dst_vreg, sljit_s32 src1_vreg, sljit_s32 src2, sljit_sw src2w)
{
    sljit_s32 reg_size = SLJIT_SIMD_GET_REG_SIZE(type);
    sljit_s32 elem_size = SLJIT_SIMD_GET_ELEM_SIZE(type);
    sljit_s32 elem_type = elem_size | ((type & SLJIT_SIMD_FLOAT) >> 6);
    sljit_s32 use_vex =
	(cpu_feature_list & CPU_FEATURE_AVX) &&
	(compiler->options & SLJIT_ENTER_USE_VEX);
    sljit_uw op = 0;
    sljit_sw inv = 0;
    sljit_s32 r;

    CHECK_ERROR();
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
    
    switch (SLJIT_SIMD_GET_OPCODE(type)) {
    case SLJIT_SIMD_ARITH_OP2_ADD:
	switch(elem_type) {
	case ELEM_8:   op = PADDB; break;
	case ELEM_16:  op = PADDW; break;
	case ELEM_32:  op = PADDD; break;
	case ELEM_64:  op = PADDQ; break;
	case ELEM_F32:  op = ADDPS; break;
	case ELEM_F64:  op = ADDPD; break;
	default: return SLJIT_ERR_UNSUPPORTED;
	}
	break;
	
    case SLJIT_SIMD_ARITH_OP2_SUB:
	switch(elem_type) {	
	case ELEM_8:   op = PSUBB; break;
	case ELEM_16:  op = PSUBW; break;
	case ELEM_32:  op = PSUBD; break;
	case ELEM_64:  op = PSUBQ; break;
	case ELEM_F32:  op = SUBPS; break;
	case ELEM_F64:  op = SUBPD; break;
	default: return SLJIT_ERR_UNSUPPORTED;
	}
	break;
	
    case SLJIT_SIMD_ARITH_OP2_MUL:
	switch(elem_type) {
	case ELEM_8:
	    return emit_sse2_mul_int8(compiler, type,
				      dst_vreg, src1_vreg,
				      src2, src2w);
	case ELEM_16:  op = PMULW; break;
	case ELEM_32:  op = PMULD; break;
	case ELEM_64:
	    return emit_sse2_mul_int64(compiler, type,
				       dst_vreg, src1_vreg,
				       src2, src2w);
	case ELEM_F32:  op = MULPS; break;
	case ELEM_F64:  op = MULPD; break;
	default: return SLJIT_ERR_UNSUPPORTED;
	}
	break;

    case SLJIT_SIMD_ARITH_OP2_SLL:
	if (src2 == SLJIT_IMM) {
	    if (dst_vreg != src1_vreg)
		emit_simd_mov(compiler, type, dst_vreg, src1_vreg);
	    switch(elem_size) {
	    case ELEM_8:
		return emit_sse2_vssl_imm8(compiler,dst_vreg,src2w);
	    case ELEM_16:
		return emit_sse2_imm8(compiler,PSLLWi,SLL,dst_vreg,src2w);
	    case ELEM_32:
		return emit_sse2_imm8(compiler,PSLLDi,SLL,dst_vreg,src2w);
	    case ELEM_64:
		return emit_sse2_imm8(compiler,PSLLQi,SLL,dst_vreg,src2w);
	    default:
		return SLJIT_ERR_UNSUPPORTED;
	    }	    
	}
	else {
	    switch(elem_size) {
	    case ELEM_8:   op = PSLLB; break;
	    case ELEM_16:  op = PSLLW; break;
	    case ELEM_32:  op = PSLLD; break;
	    case ELEM_64:  op = PSLLQ; break;
	    default: return SLJIT_ERR_UNSUPPORTED;
	    }
	}
	break;
    case SLJIT_SIMD_ARITH_OP2_SRL:
	if (src2 == SLJIT_IMM) {
	    if (dst_vreg != src1_vreg)
		emit_simd_mov(compiler, type, dst_vreg, src1_vreg);
	    switch(elem_size) {
	    case ELEM_8:
		return emit_sse2_imm8(compiler,PSRLBi,SRL,dst_vreg,src2w);
	    case ELEM_16:
		return emit_sse2_imm8(compiler,PSRLWi,SRL,dst_vreg,src2w);
	    case ELEM_32:
		return emit_sse2_imm8(compiler,PSRLDi,SRL,dst_vreg,src2w);
	    case ELEM_64:
		return emit_sse2_imm8(compiler,PSRLQi,SRL,dst_vreg,src2w);
	    default:
		return SLJIT_ERR_UNSUPPORTED;
	    }	    
	}
	else {
	    switch(elem_size) {
	    case ELEM_8:   op = PSRLB; break;
	    case ELEM_16:  op = PSRLW; break;
	    case ELEM_32:  op = PSRLD; break;
	    case ELEM_64:  op = PSRLQ; break;
	    default: return SLJIT_ERR_UNSUPPORTED;
	    }
	}
	break;	

    case SLJIT_SIMD_ARITH_OP2_SRA:
	if (src2 == SLJIT_IMM) {
	    if (dst_vreg != src1_vreg)
		emit_simd_mov(compiler, type, dst_vreg, src1_vreg);
	    switch(elem_size) {
	    case ELEM_8:
		return emit_sse2_imm8(compiler,PSRABi,SRA,dst_vreg,src2w);
	    case ELEM_16:
		return emit_sse2_imm8(compiler,PSRAWi,SRA,dst_vreg,src2w);
	    case ELEM_32:
		return emit_sse2_imm8(compiler,PSRADi,SRA,dst_vreg,src2w);
	    case ELEM_64:
		return emit_sse2_psraq_imm8(compiler,type,dst_vreg,src2w);
	    default:
		return SLJIT_ERR_UNSUPPORTED;
	    }
	}
	else {
	    switch(elem_size) {
	    case ELEM_8:   op = PSRAB; break;
	    case ELEM_16:  op = PSRAW; break;
	    case ELEM_32:  op = PSRAD; break;
	    case ELEM_64:  op = PSRAQ; break;
	    default: return SLJIT_ERR_UNSUPPORTED;		
	    }
	}
	break;

    case SLJIT_SIMD_ARITH_OP2_CMP_NOT_EQUAL:
	inv = 1;
	// fall through
    case SLJIT_SIMD_ARITH_OP2_CMP_EQUAL:
	switch(elem_type) {
	case ELEM_8:   op = PCMPEQB; break;
	case ELEM_16:  op = PCMPEQW; break;
	case ELEM_32:  op = PCMPEQD; break;
	case ELEM_64:  op = PCMPEQQ; break;
	case ELEM_F32: op = CMPPS;  src2w=CMP_EQ; break;
	case ELEM_F64:  op = CMPPD; src2w=CMP_EQ; break;
	default: return SLJIT_ERR_UNSUPPORTED;
	}
	break;

    case SLJIT_SIMD_ARITH_OP2_CMP_LESS_EQUAL:
	inv = 1;
	// fall through	
    case SLJIT_SIMD_ARITH_OP2_CMP_GREATER:
	switch(elem_type) {
	case ELEM_8:   op = PCMPGTB; break;
	case ELEM_16:  op = PCMPGTW; break;
	case ELEM_32:  op = PCMPGTD; break;
	case ELEM_64:  op = PCMPGTQ; break;
	case ELEM_F32: op = CMPPS; src2w=CMP_GT; break;
	case ELEM_F64:  op = CMPPD; src2w=CMP_GT; break;
	default: return SLJIT_ERR_UNSUPPORTED;
	}
	break;	
    case SLJIT_SIMD_ARITH_OP2_CMP_GREATER_EQUAL:
	inv = 1;
	// fall through		
    case SLJIT_SIMD_ARITH_OP2_CMP_LESS:
	switch(elem_type) {
	case ELEM_8:   op = PCMPGTB; break;
	case ELEM_16:  op = PCMPGTW; break;
	case ELEM_32:  op = PCMPGTD; break;
	case ELEM_64:  op = PCMPGTQ; break;
	case ELEM_F32: op = CMPPS;  src2w=CMP_LT; goto emit;
	case ELEM_F64:  op = CMPPD; src2w=CMP_LT; goto emit;
	default: return SLJIT_ERR_UNSUPPORTED;
	}
	// swap dst and src
	emit_simd_mov(compiler, type, TMP_VREG1, dst_vreg);
	emit_simd_mov(compiler, type, dst_vreg, src2);
	emit_simd_mov(compiler, type, dst_vreg, TMP_VREG1);
	break;
    default:
	return SLJIT_ERR_UNSUPPORTED;	    
    }

emit:
    if (type & SLJIT_SIMD_TEST)
	return SLJIT_SUCCESS;

    if ((src2 & SLJIT_MEM) && SLJIT_SIMD_GET_ELEM2_SIZE(type) < reg_size) {
	sljit_uw mov_op = ((type & SLJIT_SIMD_FLOAT) ?
			   (MOVUPS_x_xm | (elem_size == 3 ? EX86_PREF_66:0)) :
			   (MOVDQU_x_xm | EX86_PREF_F3)) | EX86_SSE2;
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
	if (dst_vreg == src2)
	    src2 = src1_vreg;
	else
	    FAIL_IF(emit_simd_mov(compiler, type, dst_vreg, src1_vreg));
    }
    if (op & (VEX_OP_0F38 | VEX_OP_0F3A))
	r = emit_groupf_ext(compiler, op | EX86_SSE2, dst_vreg, src2, src2w);
    r = emit_groupf(compiler, op | EX86_SSE2, dst_vreg, src2, src2w);
    if (inv) {
	emit_groupf(compiler, PCMPEQD | EX86_SSE2, TMP_VREG1, TMP_VREG1, 0);
	r = emit_groupf(compiler, PXOR | EX86_SSE2, dst_vreg, TMP_VREG1, 0);
    }
    return r;
}
