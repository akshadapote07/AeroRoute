# COA CO2: x86-64 GNU assembler routine for grid displacement.
# Windows x64 calling convention:
#   RCX=fromRow, RDX=fromCol, R8=toRow, R9=toCol
#   [RSP+40]=deltaX*, [RSP+48]=deltaY*, [RSP+56]=distance*

    .text
    .globl aeroroute_displacement_asm
    .def aeroroute_displacement_asm; .scl 2; .type 32; .endef
aeroroute_displacement_asm:
    # deltaX = toCol - fromCol
    # Sign-extend the 32-bit destination/source columns to 64 bits.
    movslq  %r9d, %rax
    movslq  %edx, %r10
    # Subtract source column, then store deltaX through its output pointer.
    subq    %r10, %rax
    movq    40(%rsp), %r11
    movq    %rax, (%r11)
    # Keep deltaX in R10 and square it.
    movq    %rax, %r10
    imulq   %r10, %r10

    # deltaY = toRow - fromRow
    # Sign-extend the 32-bit destination/source rows to 64 bits.
    movslq  %r8d, %rax
    movslq  %ecx, %r11
    # Subtract source row, store deltaY, and square it.
    subq    %r11, %rax
    movq    48(%rsp), %r11
    movq    %rax, (%r11)
    imulq   %rax, %rax

    # distance = sqrt(deltaX^2 + deltaY^2)
    # Add the squared deltas and convert the integer sum to double precision.
    addq    %r10, %rax
    cvtsi2sdq %rax, %xmm0
    # Compute the Euclidean distance using the SSE square-root instruction.
    sqrtsd  %xmm0, %xmm0
    # Store the distance and return to the C++ caller.
    movq    56(%rsp), %rax
    movsd   %xmm0, (%rax)
    ret
