// load and store functions

int CAT2(load_mem_,TP)(TYPE* ptr, sljit_sw addr, emulator_state_t* st)
{
    if ((addr < 0) || (addr > (sljit_sw)(st->ram_size-sizeof(TYPE))))
	return -1;
    // *ptr = *((TYPE*) (st->mem_base+addr));
    memcpy(ptr, st->mem_base+addr, VSIZE);
    return 0;
}

int CAT2(load_,TP)(TYPE* ptr, sljit_s32 src, sljit_sw srcw, emulator_state_t* st)
{
    if ((src & SLJIT_MEM) == 0) {
	int r1;
	if (((r1 = src & 0x7f) == 0) || (r1 > SLJIT_NUMBER_OF_VECTOR_REGISTERS))
	    return -1;
	*ptr = st->vr[r1-1];
	return 0;
    }
    else {
	sljit_sw addr = effective_addr(src, srcw, st);
	if (addr < 0)
	    return -1;
	return CAT2(load_mem_,TP)(ptr, addr, st);
    }
}

int CAT2(store_mem_,TP)(TYPE* ptr, sljit_sw addr, emulator_state_t* st)
{
    if ((addr < 0) || (addr > (sljit_sw)(st->ram_size-sizeof(TYPE))))    
	return -1;
    // *((TYPE*) (st->mem_base+addr)) = val;
    memcpy(st->mem_base+addr, ptr, VSIZE);
    return 0;
}

int CAT2(store_,TP)(TYPE* ptr, sljit_s32 dst, sljit_sw dstw, emulator_state_t* st)
{
    if (dst == SLJIT_IMM) {
	return -1;
    }
    else if ((dst & SLJIT_MEM) == 0) {
	int r1;
	if (((r1 = dst & 0x7f) == 0) || (r1 > SLJIT_NUMBER_OF_VECTOR_REGISTERS))
	    return -1;
	st->vr[r1-1] = *ptr;
	return 0;
    }
    else {
	sljit_sw addr = effective_addr(dst, dstw, st);
	if (addr < 0)
	    return -1;
	return CAT2(store_mem_,TP)(ptr, addr, st);
    }
}

#undef TP
#undef TYPE

