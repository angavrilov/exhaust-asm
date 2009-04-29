BITS 64

; ***** ARGUMENT ACCESS MODES *****

%define MODE_DIRECT    0
%define MODE_IMMEDIATE 1
%define MODE_INDIRECT  2
%define MODE_PREDEC    3
%define MODE_POSTINC   4
%define MODE_AINDIRECT 10
%define MODE_APREDEC   11
%define MODE_APOSTINC  12

%define mode_base(x)     ((x)&7)
%define mode_is_a(x)     (((x)&8) != 0)
%define mode_ext_code(x) (mode_base(x) + (((x)&8)>>3)*3)

%macro gen_all_amodes 1
    %assign %$AMODE MODE_DIRECT
    %1
    %assign %$AMODE MODE_IMMEDIATE
    %1
    %assign %$AMODE MODE_INDIRECT
    %1
    %assign %$AMODE MODE_PREDEC
    %1
    %assign %$AMODE MODE_POSTINC
    %1
    %assign %$AMODE MODE_AINDIRECT
    %1
    %assign %$AMODE MODE_APREDEC
    %1
    %assign %$AMODE MODE_APOSTINC
    %1
%endmacro

%macro gen_all_bmodes 1
    %assign %$BMODE MODE_DIRECT
    gen_all_amodes %1
    %assign %$BMODE MODE_IMMEDIATE
    gen_all_amodes %1
    %assign %$BMODE MODE_INDIRECT
    gen_all_amodes %1
    %assign %$BMODE MODE_PREDEC
    gen_all_amodes %1
    %assign %$BMODE MODE_POSTINC
    gen_all_amodes %1
    %assign %$BMODE MODE_AINDIRECT
    gen_all_amodes %1
    %assign %$BMODE MODE_APREDEC
    gen_all_amodes %1
    %assign %$BMODE MODE_APOSTINC
    gen_all_amodes %1
%endmacro

; ***** COMMAND MODIFIERS *****

%define MOD_F   0
%define MOD_A   1
%define MOD_B   2
%define MOD_AB  3
%define MOD_BA  4
%define MOD_X   5
%define MOD_I   6

%macro gen_all_modes 1
    %push modes
    %assign %$MOD MOD_F
    gen_all_bmodes %1
    %assign %$MOD MOD_A
    gen_all_bmodes %1
    %assign %$MOD MOD_B
    gen_all_bmodes %1
    %assign %$MOD MOD_AB
    gen_all_bmodes %1
    %assign %$MOD MOD_BA
    gen_all_bmodes %1
    %assign %$MOD MOD_X
    gen_all_bmodes %1
    %assign %$MOD MOD_I
    gen_all_bmodes %1
    %pop
%endmacro

%macro gen_one_mod 3
    %push modes
    %assign %$MOD %2
    %rep %3
    gen_all_amodes %1
    %endrep
    %pop
%endmacro

; ***** COMMAND OPCODES *****

%define OP_DAT 0
%define OP_SPL 1
%define OP_MOV 2
%define OP_DJN 3
%define OP_ADD 4
%define OP_JMZ 5
%define OP_SUB 6
%define OP_SEQ 7
%define OP_SNE 8
%define OP_SLT 9
%define OP_JMN 10
%define OP_JMP 11
%define OP_NOP 12
%define OP_MUL 13
%define OP_MODM 14
%define OP_DIV 15
%define OP_LTP 16
%define OP_STP 17

%define full_opcode(o,m,ma,mb) (mode_ext_code(ma) | (mode_ext_code(mb)<<3) | ((m)<<6) | ((o)<<9))
%define opcode_index(o,m,ma,mb) (mode_ext_code(ma) + 8*mode_ext_code(mb) + 64*(m) + 448*(o))

; ***** REGISTER ALLOCATION *****
;
;  rax  tmp                  r8   CUR_WARRIOR
;  rbx  CUR_COFS             r9   NEXT_WARRIOR
;  rcx  CMD_COUNTER          r10  NEXT_COFS
;  rdx                       r11  NEXT_CMD
;  rsi  ARG_A_OFS            r12  CORE_BASE
;  rdi  ARG_B_OFS            r13  CORE_SIZE
;  rbp                       r14  QUEUE_BASE
;  rsp                       r15  QUEUE_MASK
;
;  xmm0  tmp                 xmm8   MASK_1_XMM
;  xmm1  work                xmm9   MASK_A_XMM
;  xmm2  work                xmm10  MASK_B_XMM
;  xmm3  work                xmm11  ARG_A_XMM
;  xmm4                      xmm12  ARG_B_XMM
;  xmm5                      xmm13  CUR_COFS_XMM
;  xmm6  MASK_AB_XMM         xmm14  CUR_CMD_XMM
;  xmm7  CORE_SIZE_1_XMM     xmm15  CORE_SIZE_XMM
;  

%define ARG_A_XMM    xmm11
%define ARG_B_XMM    xmm12
%define ARG_A_OFS    rsi
%define ARG_A_OFS_32 esi
%define ARG_B_OFS    rdi
%define ARG_B_OFS_32 edi

%define MASK_1_XMM xmm8
%define MASK_A_XMM xmm9
%define MASK_B_XMM xmm10
%define MASK_AB_XMM xmm6

%define CMD_COUNTER ecx

%define CUR_WARRIOR r8
%define CUR_COFS rbx
%define CUR_COFS_32 ebx
%define CUR_COFS_XMM xmm13
%define CUR_CMD_XMM xmm14

%define NEXT_WARRIOR r9
%define NEXT_COFS r10
%define NEXT_COFS_32 r10d
%define NEXT_CMD r11

%define CORE_BASE r12
%define CORE_SIZE r13
%define CORE_SIZE_32 r13d
%define CORE_SIZE_1_XMM xmm7
%define CORE_SIZE_XMM xmm15

%define QUEUE_BASE r14
%define QUEUE_MASK r15

; ***** UTILS *****

%define queue(idx) [QUEUE_BASE+(idx)*4]
%define instr(ofs) [CORE_BASE+(ofs)]
%define fld_a(ofs) [CORE_BASE+(ofs)+8]
%define fld_b(ofs) [CORE_BASE+(ofs)+12]

%define warrior_head(ptr) [ptr]
%define warrior_tail(ptr) [(ptr)+8]
%define warrior_next(ptr) [(ptr)+8*2]
%define warrior_live(ptr) [(ptr)+8*3]
%define warrior_prev(ptr) [(ptr)+8*4]
%define warrior_id(ptr)   [(ptr)+8*5]

%macro step_queue 1
    add %1, 1
    and %1, QUEUE_MASK
%endmacro

%macro queue_cmd 1
    mov rax, warrior_tail(CUR_WARRIOR)
    mov queue(rax), %1
    step_queue rax
    mov warrior_tail(CUR_WARRIOR), rax
%endmacro

%macro add_wrap 3
    add %1, %2
    mov rax, %1
    sub rax, %3
    cmp %1, %3
    cmovae %1, rax
%endmacro

; ***** SSE UTILS *****

%define shuffle(x0, x1, x2, x3) ((x0) | ((x1)<<2) | ((x2)<<4) | ((x3)<<6))

%macro set_xmm_to 2
    movd %1, %2
    pshufd %1, %1, shuffle(0,0,0,0)
%endmacro

%macro set_xmm_up_to 2
    movd %1, %2
    pshufd %1, %1, shuffle(1,1,0,0)
%endmacro    

%macro get_dword 3
  %ifdef SSE4
    pextrd %1, %2, %3
  %else
    pshufd xmm0, %2, shuffle(%3,%3,%3,%3)
    movd %1, xmm0
  %endif
%endmacro

%macro use_mask 3
  %if %3 < 0
    %1 %2, MASK_AB_XMM
  %elif mode_is_a(%3)
    %1 %2, MASK_A_XMM
  %else
    %1 %2, MASK_B_XMM
  %endif
%endmacro

%macro xmm_add_wrap 3
    movdqa  xmm0, %3
    paddd   %1, %2
    pcmpgtd xmm0, %1
    pandn   xmm0, %3
    psubd   %1, xmm0
%endmacro

%macro xmm_sub_wrap 3
    pxor    xmm0, xmm0
    psubd   %1, %2
    pcmpgtd xmm0, %1
    pand    xmm0, %3
    paddd   %1, xmm0
%endmacro

%macro xmm_dec_mask_wrap 3
    pxor     xmm0, xmm0
    pcmpeqd  xmm0, %1
    use_mask pand, xmm0, %2
    pand     xmm0, %3
    por      %1, xmm0
    use_mask paddd, %1, %2
%endmacro

%macro xmm_inc_mask_wrap 3
    movdqa   xmm0, MASK_1_XMM
  %if %2 >= 0
    use_mask pand, xmm0, %2
  %endif
    paddd    xmm0, %1
    movdqa   %1, %3
    pcmpeqd  %1, xmm0
    use_mask pand, %1, %2
    pandn    %1, xmm0
%endmacro

    align 16
bool_get_mask:
    db 0, 4, 8, 12
    dd 0xFFFFFFFF
    dd 0xFFFFFFFF
    dd 0xFFFFFFFF

%macro test_xmm_lb 1
  %ifdef SSE4
    ptest %1, %1
  %else
    pshufb %1, [bool_get_mask]
    movd eax, %1
    test eax, eax
  %endif
%endmacro

; ***** GENERIC COMMANDS *****

%define NEED_OFS 1
%define NEED_VAL 2

%define flag_set(v,f) (((v)&(f)) != 0)
%define has_side_effects(v) ((mode_base(v) == MODE_PREDEC) || (mode_base(v) == MODE_POSTINC))

%macro load_one_arg_addr 4
  %if (mode_base(%3) != MODE_IMMEDIATE) && (has_side_effects(%3) || flag_set(%4, NEED_OFS|NEED_VAL))
    get_dword       ARG_%1_OFS_32, xmm1, %2
  %endif
%endmacro

%macro load_one_arg 4
  %if mode_base(%3) == MODE_IMMEDIATE
    %if flag_set(%4, NEED_OFS)
      mov    ARG_%1_OFS, CUR_COFS
    %endif
    %if flag_set(%4, NEED_VAL)
      movdqa ARG_%1_XMM, CUR_CMD_XMM
    %endif
  %elif has_side_effects(%3) || flag_set(%4, NEED_OFS|NEED_VAL)
    %if mode_base(%3) != MODE_DIRECT
        lea             rdx, instr(ARG_%1_OFS)
      %if mode_base(%3) != MODE_INDIRECT
        movdqa          xmm2, [rdx]
      %endif
      %if mode_base(%3) == MODE_PREDEC
          xmm_dec_mask_wrap  xmm2, %3, CORE_SIZE_1_XMM
          movdqa             [rdx], xmm2
      %endif
      %if flag_set(%4, NEED_OFS|NEED_VAL)
        %if mode_base(%3) == MODE_PREDEC
          %if mode_is_a(%3)
            get_dword     eax, xmm2, 2
          %else
            get_dword     eax, xmm2, 3
          %endif
        %else
          %if mode_is_a(%3)
            mov           eax, [rdx+8]
          %else
            mov           eax, [rdx+12]
          %endif
        %endif
          shl             rax, 4
          add_wrap        ARG_%1_OFS, rax, CORE_SIZE
        %if flag_set(%4, NEED_VAL)
          movdqa          ARG_%1_XMM, instr(ARG_%1_OFS)
        %endif
      %endif
      %if mode_base(%3) == MODE_POSTINC
          xmm_inc_mask_wrap  xmm2, %3, CORE_SIZE_1_XMM
          movdqa             [rdx], xmm2
      %endif
    %elif flag_set(%4, NEED_VAL)
        lea             rdx, instr(ARG_%1_OFS)
        movdqa          ARG_%1_XMM, [rdx]
    %endif
  %endif
%endmacro

%macro load_args 4
  %if has_side_effects(%1) || has_side_effects(%2) || %3 != 0 || %4 != 0
    movdqa CUR_CMD_XMM, instr(CUR_COFS)
  %endif

  %if mode_base(%1) == MODE_IMMEDIATE && mode_base(%2) == MODE_IMMEDIATE
    prefetcht0    instr(rsi)

    %if flag_set(%3, NEED_OFS)
      mov    ARG_A_OFS, CUR_COFS
    %endif
    %if flag_set(%3, NEED_VAL)
      movdqa ARG_A_XMM, CUR_CMD_XMM
    %endif
    %if flag_set(%4, NEED_OFS)
      mov    ARG_B_OFS, CUR_COFS
    %endif
    %if flag_set(%4, NEED_VAL)
      movdqa ARG_B_XMM, CUR_CMD_XMM
    %endif
  %elif flag_set(%3, NEED_OFS|NEED_VAL) || has_side_effects(%1) || \
        flag_set(%4, NEED_OFS|NEED_VAL) || has_side_effects(%2)
      set_xmm_up_to CUR_COFS_XMM, CUR_COFS_32

      prefetcht0    instr(rsi)

      movdqa       xmm1, CUR_CMD_XMM
      pslld        xmm1, 4
      xmm_add_wrap xmm1, CUR_COFS_XMM, CORE_SIZE_XMM

      load_one_arg_addr A,2,%1,%3
      load_one_arg_addr B,3,%2,%4

      load_one_arg A,2,%1,%3
      load_one_arg B,3,%2,%4
  %else
    prefetcht0    instr(rsi)
  %endif
%endmacro

%macro cmd_preamble 0
    mov rax, warrior_head(NEXT_WARRIOR)
    mov NEXT_COFS_32, queue(rax)
    mov esi,          queue(rax+1)    ; For prefetch
    step_queue rax
    mov NEXT_CMD, instr(NEXT_COFS)
    mov warrior_head(NEXT_WARRIOR), rax
%endmacro

%macro cmd_end 0
    sub CMD_COUNTER, 1
    jz  timeout_reached
    mov CUR_WARRIOR,NEXT_WARRIOR
    mov CUR_COFS,NEXT_COFS
    mov NEXT_WARRIOR,warrior_next(NEXT_WARRIOR)
    jmp NEXT_CMD
%endmacro

%macro cmd_end_next 0-1 1
    add CUR_COFS, (16*%1)
    cmp CUR_COFS, CORE_SIZE
    jae %%fix_cwrap
  %%fixed_cwrap:
    queue_cmd CUR_COFS_32
    cmd_end
  %%fix_cwrap:
    sub CUR_COFS, CORE_SIZE
    jmp %%fixed_cwrap
%endmacro

%macro begin_cmd 3
    align 16
    dd full_opcode(%1,%$MOD,%$AMODE,%$BMODE)
    dd opcode_index(%1,%$MOD,%$AMODE,%$BMODE)
    dd 0
    dd 0
_cmd_%1_%{$MOD}_%{$AMODE}_%{$BMODE}:
    cmd_preamble
    load_args %$AMODE, %$BMODE, %2, %3
%endmacro

; CORE

;
;  rax  tmp                  r8   CUR_WARRIOR
;  rbx  CUR_COFS             r9   NEXT_WARRIOR
;  rcx  CMD_COUNTER          r10  NEXT_COFS
;  rdx                       r11  NEXT_CMD
;  rsi  ARG_A_OFS            r12  CORE_BASE
;  rdi  ARG_B_OFS            r13  CORE_SIZE
;  rbp                       r14  QUEUE_BASE
;  rsp                       r15  QUEUE_MASK
;
;  xmm0  tmp                 xmm8   MASK_1_XMM
;  xmm1  work                xmm9   MASK_A_XMM
;  xmm2  work                xmm10  MASK_B_XMM
;  xmm3  work                xmm11  ARG_A_XMM
;  xmm4                      xmm12  ARG_B_XMM
;  xmm5                      xmm13  CUR_COFS_XMM
;  xmm6                      xmm14  CUR_CMD_XMM
;  xmm7  CORE_SIZE_1_XMM     xmm15  CORE_SIZE_XMM

%define ALIVE_CNT  [rbp-8*5-8]
%define PROC_LIMIT [rbp+16+8*0]
%define DEATH_TAB  [rbp+16+8*1]

    global _do_simulate
_do_simulate:
    push rbp
    mov  rbp, rsp
    push r12
    push r13
    push r14
    push r15
    push rbx
    sub  rsp, 8
    
    ; Shuffle parameters
    mov CORE_BASE,    rdi
    mov CORE_SIZE,    rsi
    mov QUEUE_BASE,   rdx
    mov QUEUE_MASK,   rcx
    mov CUR_WARRIOR,  r8
    xor rcx, rcx
    mov CMD_COUNTER,  r9d
    
    ; Initialize xmm registers
    mov eax, -1
    movd xmm0, eax
    pshufd MASK_A_XMM, xmm0, shuffle(1,1,0,1)
    pshufd MASK_B_XMM, xmm0, shuffle(1,1,1,0)
    pshufd MASK_AB_XMM, xmm0, shuffle(1,1,0,0)
    mov eax, 1
    set_xmm_up_to MASK_1_XMM, eax
    mov eax, CORE_SIZE_32
    set_xmm_up_to CORE_SIZE_XMM, eax
    shr eax, 4
    set_xmm_up_to CORE_SIZE_1_XMM, eax

    ; Count live warriors
    mov rdx, CUR_WARRIOR
    xor rax, rax
 .loop:
    mov rdx, warrior_next(rdx)
    add rax, 1
    cmp rdx, CUR_WARRIOR
    jne .loop
    mov ALIVE_CNT, rax
    
    ; Initialize the scheduler registers
    mov NEXT_WARRIOR, warrior_next(CUR_WARRIOR)
    
    mov rax, warrior_head(CUR_WARRIOR)
    mov CUR_COFS_32, queue(rax)
    step_queue rax
    mov warrior_head(CUR_WARRIOR), rax
    
    ; Start the interpreter
    mov NEXT_CMD, instr(CUR_COFS)
    jmp NEXT_CMD

    align 16
timeout_reached:
one_warrior_left:
    mov rax, ALIVE_CNT
    add rsp, 8
    pop rbx
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbp
    ret

;  ---- NOP ----

%macro gen_nop_cmd 0
    begin_cmd OP_NOP, 0, 0
    cmd_end_next
%endmacro

gen_all_modes gen_nop_cmd

;  ---- JMP ----

%macro gen_jmp_cmd 0
    begin_cmd OP_JMP, NEED_OFS, 0
    queue_cmd ARG_A_OFS_32
    cmd_end
%endmacro

gen_all_modes gen_jmp_cmd

;  ---- DAT ----

%macro gen_dat_cmd 0
    begin_cmd OP_DAT, 0, 0
    jmp dat_cmd_common
%endmacro

dat_cmd_common:
    ; Decrement the process counter
    sub warrior_live(CUR_WARRIOR), dword 1
    jz .continue
    cmd_end
 .continue:
    ; Destroy the dead warrior
    mov rdx, warrior_prev(CUR_WARRIOR)
    mov warrior_next(rdx), NEXT_WARRIOR
    mov warrior_prev(NEXT_WARRIOR), rdx
    ; Decrease the step counter
    mov ebx, ALIVE_CNT ; Kills CUR_COFS
    mov eax, CMD_COUNTER
    xor edx, edx
    div ebx
    sub CMD_COUNTER, eax
    ; Remember the death
    mov rdx, DEATH_TAB
    mov eax, warrior_id(CUR_WARRIOR)
    mov [rdx], eax
    add rdx, 4
    mov DEATH_TAB, rdx
    ; Decrement the warrior counter
    sub ebx, 1
    mov ALIVE_CNT, ebx
    cmp ebx, 1
    jbe one_warrior_left
    cmd_end

gen_all_modes gen_dat_cmd

;  ---- SPL ----

%macro gen_spl_cmd 0
    begin_cmd OP_SPL, NEED_OFS, 0
    jmp spl_cmd_common
%endmacro

spl_cmd_common:
    add_wrap  CUR_COFS, 16, CORE_SIZE
    queue_cmd CUR_COFS_32
    mov eax, warrior_live(CUR_WARRIOR)
    cmp eax, PROC_LIMIT
    jae .continue
    add eax, 1
    mov warrior_live(CUR_WARRIOR), eax
    queue_cmd ARG_A_OFS_32
 .continue:
    cmd_end

gen_all_modes gen_spl_cmd

;  ---- MOV ----

%macro make_masks 1-2 movdqa
  %if (%$MOD == MOD_F) || (%$MOD == MOD_X) || (%$MOD == MOD_I)
    %2 %1, MASK_AB_XMM
  %elif (%$MOD == MOD_A) || (%$MOD == MOD_BA)
    %2 %1, MASK_A_XMM
  %elif (%$MOD == MOD_B) || (%$MOD == MOD_AB)
    %2 %1, MASK_B_XMM
  %endif
%endmacro

%macro make_shuffle 1
  %if (%$MOD == MOD_X) || (%$MOD == MOD_AB) || (%$MOD == MOD_BA)
    pshufhw %1, %1, shuffle(2,3,0,1)
  %endif
%endmacro

%macro make_save_mask 1
  %ifndef SSE4
    make_masks %1
  %endif
%endmacro

%macro save_b_fields 3
  %ifdef SSE4
    %if (%$MOD == MOD_F) || (%$MOD == MOD_X) || (%$MOD == MOD_I)
      pblendw %2, %3, 0x0F
    %elif (%$MOD == MOD_A) || (%$MOD == MOD_BA)
      pblendw %2, %3, 0xCF
    %elif (%$MOD == MOD_B) || (%$MOD == MOD_AB)
      pblendw %2, %3, 0x3F
    %endif
    movdqa instr(ARG_B_OFS), %2
  %else
    pand %2, %1
    pandn %1, %3
    por %2, %1
    movdqa instr(ARG_B_OFS), %2
  %endif
%endmacro

%macro gen_mov_cmd 0
    begin_cmd OP_MOV, NEED_VAL, NEED_OFS
  
  %if %$MOD == MOD_I
    movdqa instr(ARG_B_OFS), ARG_A_XMM
    cmp  ARG_B_OFS, NEXT_COFS
    je .fix
 .done:
    cmd_end_next
 .fix:
    movq NEXT_CMD, ARG_A_XMM
    jmp .done
  %else
    movdqa ARG_B_XMM, instr(ARG_B_OFS)
    make_save_mask xmm1
    make_shuffle ARG_A_XMM
    save_b_fields xmm1, ARG_A_XMM, ARG_B_XMM
    cmd_end_next
  %endif
%endmacro

gen_all_modes gen_mov_cmd

;  ---- DJN ----

%macro get_real_b_val 1
  %if mode_base(%$BMODE) == MODE_POSTINC
    movdqa %1, instr(ARG_B_OFS)
  %else
    movdqa %1, ARG_B_XMM
  %endif
%endmacro

%macro gen_djn_cmd 0
    begin_cmd OP_DJN, NEED_OFS, (NEED_OFS|NEED_VAL)
    
    get_real_b_val xmm1
    pcmpeqd ARG_B_XMM, MASK_1_XMM ; true => now 0
    
  %if %$MOD == MOD_BA || %$MOD == MOD_A
    xmm_dec_mask_wrap xmm1, MODE_AINDIRECT, CORE_SIZE_1_XMM
  %elif %$MOD == MOD_AB || %$MOD == MOD_B
    xmm_dec_mask_wrap xmm1, MODE_INDIRECT, CORE_SIZE_1_XMM
  %else
    xmm_dec_mask_wrap xmm1, -1, CORE_SIZE_1_XMM
  %endif
    
    make_masks ARG_B_XMM, pandn ; true => now nz in position
    movdqa instr(ARG_B_OFS), xmm1
    
    test_xmm_lb ARG_B_XMM
    jz %%nojump
    queue_cmd ARG_A_OFS_32      ; jump if nonzero
    cmd_end
  %%nojump:
    cmd_end_next
%endmacro

gen_all_modes gen_djn_cmd

;  ---- ADD ----

%macro gen_add_cmd 0
    begin_cmd OP_ADD, NEED_VAL, (NEED_OFS|NEED_VAL)
    make_save_mask xmm1
    make_shuffle ARG_A_XMM
    get_real_b_val xmm2
    xmm_add_wrap ARG_B_XMM, ARG_A_XMM, CORE_SIZE_1_XMM
    save_b_fields xmm1, ARG_B_XMM, xmm2
    cmd_end_next
%endmacro

gen_all_modes gen_add_cmd

;  ---- JMZ ----

%macro gen_jmz_cmd 0
    begin_cmd OP_JMZ, NEED_OFS, (NEED_OFS|NEED_VAL)

    pxor xmm0, xmm0
    pcmpeqd xmm0, ARG_B_XMM ; true => zero
    make_masks xmm0, pandn  ; true => nonzero in position

    test_xmm_lb xmm0
    jz %%jump
    cmd_end_next
  %%jump:
    queue_cmd ARG_A_OFS_32  ; jump if zero
    cmd_end
%endmacro

gen_all_modes gen_jmz_cmd

;  ---- SUB ----

%macro gen_sub_cmd 0
    begin_cmd OP_SUB, NEED_VAL, (NEED_OFS|NEED_VAL)
    make_save_mask xmm1
    make_shuffle ARG_A_XMM
    get_real_b_val xmm2
    xmm_sub_wrap ARG_B_XMM, ARG_A_XMM, CORE_SIZE_1_XMM
    save_b_fields xmm1, ARG_B_XMM, xmm2
    cmd_end_next
%endmacro

gen_all_modes gen_sub_cmd

;  ---- SNE ----

    align 16
full_mask:
    dd 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF

%macro gen_sne_cmd 0
    begin_cmd OP_SNE, NEED_VAL, NEED_VAL

    make_shuffle ARG_A_XMM
    pcmpeqd ARG_B_XMM, ARG_A_XMM  ; true => eq
  %if %$MOD != MOD_I    
    make_masks ARG_B_XMM, pandn   ; true => noneq in position
  %else
    pandn ARG_B_XMM, [full_mask]
  %endif

    test_xmm_lb ARG_B_XMM
    jz %%noskip
    cmd_end_next 2
  %%noskip:
    cmd_end_next
%endmacro

gen_all_modes gen_sne_cmd

;  ---- SEQ ----

%macro gen_seq_cmd 0
    begin_cmd OP_SEQ, NEED_VAL, NEED_VAL

    make_shuffle ARG_A_XMM
    pcmpeqd ARG_B_XMM, ARG_A_XMM  ; true => eq
  %if %$MOD != MOD_I    
    make_masks ARG_B_XMM, pandn   ; true => noneq in position
  %else
    pandn ARG_B_XMM, [full_mask]
  %endif

    test_xmm_lb ARG_B_XMM
    jz %%skip
    cmd_end_next
  %%skip:
    cmd_end_next 2
%endmacro

gen_all_modes gen_seq_cmd

;  ---- JMN ----

%macro gen_jmn_cmd 0
    begin_cmd OP_JMN, NEED_OFS, (NEED_OFS|NEED_VAL)

    pxor xmm0, xmm0
    pcmpeqd xmm0, ARG_B_XMM ; true => zero
    make_masks xmm0, pandn  ; true => nonzero in position

    test_xmm_lb xmm0
    jz %%nojump
    queue_cmd ARG_A_OFS_32  ; jump if nonzero
    cmd_end
  %%nojump:
    cmd_end_next
%endmacro

gen_all_modes gen_jmn_cmd

;  ---- MUL ----

%macro gen_mul_cmd 0
    begin_cmd OP_MUL, NEED_VAL, (NEED_OFS|NEED_VAL)
    make_save_mask xmm1
    make_shuffle ARG_A_XMM
    get_real_b_val xmm2

    mov rsi, CORE_SIZE  ; ARG_A_OFS unused
    shr rsi, 4
   
  %if (%$MOD != MOD_B) && (%$MOD != MOD_AB)
    get_dword eax, ARG_A_XMM, 2
    get_dword edx, ARG_B_XMM, 2
    mul edx
    div rsi
    movd xmm3, edx
    pshufd xmm3, xmm3, shuffle(1,1,0,1)
  %endif

  %if (%$MOD != MOD_A) && (%$MOD != MOD_BA)
    get_dword eax, ARG_A_XMM, 3
    get_dword edx, ARG_B_XMM, 3
    mul edx
    div rsi
    movd xmm4, edx
    pshufd xmm4, xmm4, shuffle(1,1,1,0)
  %endif

  %if (%$MOD == MOD_A) || (%$MOD == MOD_BA)
    save_b_fields xmm1, xmm3, xmm2
  %else
    %if (%$MOD != MOD_B) && (%$MOD != MOD_AB)
      por xmm4, xmm3
    %endif
    save_b_fields xmm1, xmm4, xmm2
  %endif

    cmd_end_next
%endmacro

gen_all_modes gen_mul_cmd

; ***** CORE CLEAR *****

    global _do_clear_core
_do_clear_core:
    mov  rax, _cmd_0_0_0_0
    movq xmm0, rax
    mov  ecx, esi
    shr  ecx, 2
    jz   .noloop1
 .loop1:
    movdqa [rdi], xmm0
    movdqa [rdi+16], xmm0
    movdqa [rdi+16*2], xmm0   
    movdqa [rdi+16*3], xmm0
    add rdi, 16*4
    loop .loop1
 .noloop1:
    mov ecx, esi
    and ecx, 3
    jz  .noloop2
 .loop2:
    movdqa [rdi], xmm0
    add rdi, 16
    loop .loop2
 .noloop2:
    ret

; ***** OPCODE TABLE *****

%macro gen_opcode_ref 0
    dd (_cmd_ %+ OPCODE %+ _%{$MOD}_%{$AMODE}_%{$BMODE} - _opcode_handler_table)
%endmacro

%macro gen_opcode_stub 0
    dd 0
%endmacro

    global _opcode_handler_table
_opcode_handler_table:
    %assign OPCODE OP_DAT
    gen_all_modes gen_opcode_ref
    %assign OPCODE OP_SPL
    gen_all_modes gen_opcode_ref
    %assign OPCODE OP_MOV
    gen_all_modes gen_opcode_ref
    %assign OPCODE OP_DJN
    gen_all_modes gen_opcode_ref
    %assign OPCODE OP_ADD
    gen_all_modes gen_opcode_ref
    %assign OPCODE OP_JMZ
    gen_all_modes gen_opcode_ref
    %assign OPCODE OP_SUB
    gen_all_modes gen_opcode_ref
    %assign OPCODE OP_SEQ
    gen_all_modes gen_opcode_ref
    %assign OPCODE OP_SNE
    gen_all_modes gen_opcode_ref
    %assign OPCODE OP_SLT
    gen_all_modes gen_opcode_stub
    %assign OPCODE OP_JMN
    gen_all_modes gen_opcode_ref
    %assign OPCODE OP_JMP
    gen_all_modes gen_opcode_ref
    %assign OPCODE OP_NOP
    gen_all_modes gen_opcode_ref
    %assign OPCODE OP_MUL
    gen_all_modes gen_opcode_ref
    %assign OPCODE OP_MODM
    gen_all_modes gen_opcode_stub
    %assign OPCODE OP_DIV
    gen_all_modes gen_opcode_stub
    %assign OPCODE OP_LTP
    gen_all_modes gen_opcode_stub
    %assign OPCODE OP_STP
    gen_all_modes gen_opcode_stub

