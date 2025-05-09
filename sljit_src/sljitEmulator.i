// load and store functions

int CAT2(load_mem_,TP)(TYPE* ptr, sljit_sw addr, cpu_state_t* st)
{
    if ((addr < 0) || (addr >= MEMORY_SIZE))
	return -1;
    // FIXME: check alignment!
    *ptr = *((TYPE*) (((uint8_t*)st->mem)+addr));
    return 0;
}

int CAT2(load_,TP)(TYPE* ptr, sljit_s32 src, sljit_sw srcw, cpu_state_t* st)
{
    if (src == SLJIT_IMM)
	*ptr = srcw;
    else if (src == (SLJIT_MEM|SLJIT_IMM))
	return CAT2(load_mem_,TP)(ptr, srcw, st);
    else {
	int r1 = src & 0x7f;
	if ((r1 == 0) || (r1 > SLJIT_NUMBER_OF_REGISTERS))
	    return -1;
	if (src & SLJIT_MEM) {
	    int r2 = (src >> 8) & 0x7f;
	    if (r2 > 0) {
		if (r2 > SLJIT_NUMBER_OF_REGISTERS)
		    return -1;
		else if (srcw == 0)
		    return CAT2(load_mem_,TP)(ptr, R(st,r1-1) + R(st,r2-1), st);
		else // error if srcw > 3?
		    return CAT2(load_mem_,TP)(ptr, R(st,r1-1) + (R(st,r2-1) << (srcw & 3)), st);
	    }
	    else
		return CAT2(load_mem_,TP)(ptr, R(st,r1-1) + srcw, st);
	}
	else {
	    *ptr = st->r[r1-1].TP;
	    return 0;
	}
    }
    return -1;
}

int CAT2(store_mem_,TP)(TYPE val, sljit_sw addr, cpu_state_t* st)
{
    if ((addr < 0) || (addr >= MEMORY_SIZE))
	return -1;
    // check alignment!
    *((TYPE*) (((uint8_t*) st->mem)+addr)) = val;
    return 0;
}

int CAT2(store_,TP)(TYPE val, sljit_s32 dst, sljit_sw dstw, cpu_state_t* st)
{
    if (dst == SLJIT_IMM)
	return -1;
    else if (dst == (SLJIT_MEM|SLJIT_IMM))
	return CAT2(store_mem_,TP)(val, dstw, st);
    else {
	int r1 = dst & 0x7f;
	if ((r1 == 0) || (r1 > SLJIT_NUMBER_OF_REGISTERS))
	    return -1;
	if (dst & SLJIT_MEM) {	
	    int r2 = (dst >> 8) & 0x7f;
	    if (r2 > 0) {
		if (r2 > SLJIT_NUMBER_OF_REGISTERS)
		    return -1;
		else if (dstw == 0)
		    return CAT2(store_mem_,TP)(val, R(st,r1-1) + R(st,r2-1), st);
		else // error if dstw > 3?
		    return CAT2(store_mem_,TP)(val, R(st,r1-1) + (R(st,r2-1) << (dstw & 3)), st);
	    }
	    else
		return CAT2(store_mem_,TP)(val, R(st,r1-1) + dstw, st);
	}
	else {
	    st->r[r1-1].TP = val;
	    return 0;
	}
    }
    return -1;
}

#undef TP
#undef TYPE

