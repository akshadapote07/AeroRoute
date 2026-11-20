# COA CO3: x86-64 GNU assembler routines using the Windows x64 ABI.
# The C++ layer validates inputs, manages std::string storage, and logs results.

    .text

    # ADD algorithm starts here.

    .globl aeroroute_add64_asm
    .def aeroroute_add64_asm; .scl 2; .type 32; .endef
aeroroute_add64_asm:
    # RCX holds the left operand and RDX the right operand.
    movq    %rcx, %rax
    # Return the 64-bit sum in RAX.
    addq    %rdx, %rax
    ret
    # ADD algorithm ends here.

    # SUB algorithm starts here.
    .globl aeroroute_sub64_asm
    .def aeroroute_sub64_asm; .scl 2; .type 32; .endef
aeroroute_sub64_asm:
    # Copy the left operand to the return register.
    movq    %rcx, %rax
    # Subtract the right operand; the result is returned in RAX.
    subq    %rdx, %rax
    ret
    # SUB algorithm ends here.

    # Signed compare algorithm starts here.
    .globl aeroroute_compare64_asm
    .def aeroroute_compare64_asm; .scl 2; .type 32; .endef
aeroroute_compare64_asm:
    # Compare signed 64-bit operands without calculating left - right.
    cmpq    %rdx, %rcx
    # Record whether left is less than or greater than right.
    setl    %al
    setg    %dl
    # Convert the Boolean results to integers.
    movzbl  %al, %eax
    movzbl  %dl, %edx
    # greater - less produces 1, 0, or -1 in EAX.
    subl    %eax, %edx
    movl    %edx, %eax
    ret
    # Signed compare algorithm ends here.

    # ADD-and-shift multiplication algorithm starts here.
    .globl aeroroute_mul_add_shift_asm
    .def aeroroute_mul_add_shift_asm; .scl 2; .type 32; .endef
aeroroute_mul_add_shift_asm:
    # C++ bounds each operand to +/-1,000,000; save the signed multiplicand.
    movq    %rcx, %r8
    # Save the signed multiplier and clear the product-sign bit.
    movq    %rdx, %r10
    xorl    %r11d, %r11d

    # Convert the multiplicand to its magnitude and toggle result sign.
    testq   %r8, %r8
    jns     .Ladd_shift_left_positive
    negq    %r8
    xorl    $1, %r11d
.Ladd_shift_left_positive:
    # Convert the multiplier to its magnitude and toggle result sign.
    testq   %r10, %r10
    jns     .Ladd_shift_right_positive
    negq    %r10
    xorl    $1, %r11d
.Ladd_shift_right_positive:
    # Initialize product to zero and process the 32 multiplier bits.
    xorl    %eax, %eax
    movl    $32, %ecx
.Ladd_shift_loop:
    # If the current multiplier bit is 1, add the current multiplicand.
    testb   $1, %r10b
    jz      .Ladd_shift_next
    addq    %r8, %rax
.Ladd_shift_next:
    # Advance to the next bit: multiplicand *= 2, multiplier /= 2.
    shlq    $1, %r8
    shrq    $1, %r10
    # Repeat until all 32 multiplier bits have been examined.
    decl    %ecx
    jnz     .Ladd_shift_loop

    # Negate the product only when exactly one input was negative.
    testl   %r11d, %r11d
    jz      .Ladd_shift_done
    negq    %rax
.Ladd_shift_done:
    ret
    # ADD-and-shift multiplication algorithm ends here.

    # Booth multiplication algorithm starts here.
    .globl aeroroute_mul_booth_asm
    .def aeroroute_mul_booth_asm; .scl 2; .type 32; .endef
aeroroute_mul_booth_asm:
    # R11 is signed M; R10D is the 32-bit two's-complement multiplier Q.
    movq    %rcx, %r11
    movl    %edx, %r10d
    # Q-1 starts at zero; RAX is the initially zero Booth accumulator A.
    xorl    %r9d, %r9d
    xorl    %eax, %eax
    # The supported operand range fits in a signed 32-bit multiplier.
    movl    $32, %r8d
.Lbooth_loop:
    # Inspect the pair (Q0, Q-1): 10 means A = A - M.
    testb   $1, %r10b
    jz      .Lbooth_current_zero
    testl   %r9d, %r9d
    jnz     .Lbooth_next
    subq    %r11, %rax
    jmp     .Lbooth_next
.Lbooth_current_zero:
    # Pair 01 means A = A + M; pairs 00 and 11 need no arithmetic.
    testl   %r9d, %r9d
    jz      .Lbooth_next
    addq    %r11, %rax
.Lbooth_next:
    # Remember Q0 as the next Q-1, then shift Q and the weighted M.
    movl    %r10d, %r9d
    andl    $1, %r9d
    shrl    $1, %r10d
    shlq    $1, %r11
    # After 32 recoding steps, A is the signed product in RAX.
    decl    %r8d
    jnz     .Lbooth_loop
    ret
    # Booth multiplication algorithm ends here.

    # Signed shift-and-subtract division algorithm starts here.
    .globl aeroroute_div_shift_sub_asm
    .def aeroroute_div_shift_sub_asm; .scl 2; .type 32; .endef
aeroroute_div_shift_sub_asm:
    # Preserve Windows x64 nonvolatile registers used by this routine.
    pushq   %r12
    pushq   %r13
    pushq   %r14
    pushq   %r15

    # Keep output pointers; RAX/R10 become dividend/divisor magnitudes.
    movq    %r8, %r12
    movq    %r9, %r13
    movq    %rcx, %rax
    movq    %rdx, %r10
    # R14 and R15 record the original operand signs.
    xorl    %r14d, %r14d
    xorl    %r15d, %r15d

    # Make the dividend magnitude (two's-complement negation also handles LLONG_MIN).
    testq   %rax, %rax
    jns     .Ldividend_magnitude_ready
    movl    $1, %r14d
    negq    %rax
.Ldividend_magnitude_ready:
    # Make the divisor magnitude and remember whether it was negative.
    testq   %r10, %r10
    jns     .Ldivisor_magnitude_ready
    movl    $1, %r15d
    negq    %r10
.Ldivisor_magnitude_ready:
    # R11 builds the quotient; R8 holds the running remainder.
    xorl    %r11d, %r11d
    xorl    %r8d, %r8d
    # Process all 64 dividend bits, from most significant to least significant.
    movl    $64, %ecx
.Ldivision_loop:
    # Shift the next dividend bit into the partial remainder.
    shlq    $1, %rax
    rclq    $1, %r8
    # If remainder >= divisor, subtract it and append quotient bit 1.
    cmpq    %r10, %r8
    jb      .Ldivision_no_subtract
    subq    %r10, %r8
    shlq    $1, %r11
    orq     $1, %r11
    jmp     .Ldivision_next
.Ldivision_no_subtract:
    # Otherwise append quotient bit 0.
    shlq    $1, %r11
.Ldivision_next:
    # Continue with the next dividend bit.
    decl    %ecx
    jnz     .Ldivision_loop

    # Quotient sign is negative only when operand signs differ.
    movl    %r14d, %edx
    xorl    %r15d, %edx
    testl   %edx, %edx
    jz      .Lquotient_sign_ready
    negq    %r11
.Lquotient_sign_ready:
    # C/C++ signed division keeps the remainder's sign with the dividend.
    testl   %r14d, %r14d
    jz      .Lremainder_sign_ready
    negq    %r8
.Lremainder_sign_ready:
    # Write both results through the pointers supplied by the C++ wrapper.
    movq    %r11, (%r12)
    movq    %r8, (%r13)

    popq    %r15
    popq    %r14
    popq    %r13
    popq    %r12
    ret
    # Signed shift-and-subtract division algorithm ends here.

    # SCASB string-length algorithm starts here.
    .globl aeroroute_strlen_asm
    .def aeroroute_strlen_asm; .scl 2; .type 32; .endef
aeroroute_strlen_asm:
    # Save RDI because it is nonvolatile in the Windows x64 ABI.
    pushq   %rdi
    # RCX is the string pointer; RDX is its size plus the terminator.
    movq    %rcx, %rdi
    movq    %rdx, %r8
    movq    %rdx, %rcx
    # AL=0 makes REPNE SCASB stop at the null terminator or bound.
    xorl    %eax, %eax
    repne   scasb
    # Bytes scanned include the terminator, so subtract one for length.
    subq    %rcx, %r8
    leaq    -1(%r8), %rax
    popq    %rdi
    ret
    # SCASB string-length algorithm ends here.

    # MOVSB string-append algorithm starts here.
    .globl aeroroute_memcpy_asm
    .def aeroroute_memcpy_asm; .scl 2; .type 32; .endef
aeroroute_memcpy_asm:
    # Save RSI/RDI because the Windows x64 ABI requires their values preserved.
    pushq   %rsi
    pushq   %rdi
    # Map arguments to MOVSB's destination, source, and count registers.
    movq    %rcx, %rdi
    movq    %rdx, %rsi
    movq    %r8, %rcx
    # Copy exactly the appended byte count.
    rep     movsb
    popq    %rdi
    popq    %rsi
    ret
    # MOVSB string-append algorithm ends here.

# Five-stage pipeline schedule-generation algorithm starts here.
    .globl aeroroute_pipeline_schedule_asm
    .def aeroroute_pipeline_schedule_asm; .scl 2; .type 32; .endef
aeroroute_pipeline_schedule_asm:
    # RCX points to a 4-row by 8-cycle byte table, initially zeroed by C++.
    movq    %rcx, %r8
    xorl    %r9d, %r9d
.Lpipeline_row:
    # R9D selects one of four independent instructions.
    xorl    %r10d, %r10d
.Lpipeline_cycle:
    # Empty cells contain 0; active cells contain stage codes 1 through 5.
    xorl    %eax, %eax
    cmpl    %r9d, %r10d
    jb      .Lpipeline_store
    # Stage offset is cycle index minus instruction index.
    movl    %r10d, %eax
    subl    %r9d, %eax
    cmpl    $5, %eax
    jae     .Lpipeline_empty
    # Encode IF=1, ID=2, EX=3, MEM=4, and WB=5.
    incl    %eax
    jmp     .Lpipeline_store
.Lpipeline_empty:
    # Cycles outside this instruction's five stages stay blank.
    xorl    %eax, %eax
.Lpipeline_store:
    # Save the stage code and advance to the next cycle cell.
    movb    %al, (%r8)
    incq    %r8
    incl    %r10d
    cmpl    $8, %r10d
    jb      .Lpipeline_cycle
    # After eight cycles, start the next instruction row.
    incl    %r9d
    cmpl    $4, %r9d
    jb      .Lpipeline_row
    ret
# Five-stage pipeline schedule-generation algorithm ends here.

# ============================================================================
# Five-stage pipeline example starts here (comment-only CO4 demonstration).
# x86-64 user assembly cannot directly control the processor's internal
# pipeline stages. The assembly routine above generates the numeric schedule;
# displayFiveStagePipelineDemo() in Co4.cpp prints the stage names. This listing
# shows the independent MOV sequence represented by that schedule.
#
# Example instruction stream:
# I1: movq fromCol, %rax
# I2: movq toCol,   %rbx
# I3: movq fromRow, %rcx
# I4: movq toRow,   %rdx
#
# IF  = Instruction Fetch, ID  = Instruction Decode,
# EX  = Execute,            MEM = Memory Access,
# WB  = Write Back.
#
# Cycle | I1  | I2  | I3  | I4
#   C1  | IF  |     |     |
#   C2  | ID  | IF  |     |
#   C3  | EX  | ID  | IF  |
#   C4  | MEM | EX  | ID  | IF
#   C5  | WB  | MEM | EX  | ID
#   C6  |     | WB  | MEM | EX
#   C7  |     |     | WB  | MEM
#   C8  |     |     |     | WB
#
# Each row is a cycle; instructions overlap because these MOVs are independent.
# This instruction listing is intentionally commented out; it is not executable.
# Five-stage pipeline example ends here.
# ============================================================================
