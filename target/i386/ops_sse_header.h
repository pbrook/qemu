/*
 *  MMX/3DNow!/SSE/SSE2/SSE3/SSSE3/SSE4/PNI support
 *
 *  Copyright (c) 2005 Fabrice Bellard
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#if SHIFT == 0
#define Reg MMXReg
#define SUFFIX _mmx
#else
#define Reg ZMMReg
#if SHIFT == 1
#define SUFFIX _xmm
#else
#define SUFFIX _ymm
#endif
#endif

#define dh_alias_Reg ptr
#define dh_alias_ZMMReg ptr
#define dh_alias_MMXReg ptr
#define dh_ctype_Reg Reg *
#define dh_ctype_ZMMReg ZMMReg *
#define dh_ctype_MMXReg MMXReg *
#define dh_typecode_Reg dh_typecode_ptr
#define dh_typecode_ZMMReg dh_typecode_ptr
#define dh_typecode_MMXReg dh_typecode_ptr

DEF_HELPER_4(glue(psrlw, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(psraw, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(psllw, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(psrld, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(psrad, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pslld, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(psrlq, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(psllq, SUFFIX), void, env, Reg, Reg, Reg)

#if SHIFT >= 1
DEF_HELPER_4(glue(psrldq, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pslldq, SUFFIX), void, env, Reg, Reg, Reg)
#endif

#define SSE_HELPER_B(name, F)\
    DEF_HELPER_4(glue(name, SUFFIX), void, env, Reg, Reg, Reg)

#define SSE_HELPER_W(name, F)\
    DEF_HELPER_4(glue(name, SUFFIX), void, env, Reg, Reg, Reg)

#define SSE_HELPER_L(name, F)\
    DEF_HELPER_4(glue(name, SUFFIX), void, env, Reg, Reg, Reg)

#define SSE_HELPER_Q(name, F)\
    DEF_HELPER_4(glue(name, SUFFIX), void, env, Reg, Reg, Reg)

SSE_HELPER_B(paddb, FADD)
SSE_HELPER_W(paddw, FADD)
SSE_HELPER_L(paddl, FADD)
SSE_HELPER_Q(paddq, FADD)

SSE_HELPER_B(psubb, FSUB)
SSE_HELPER_W(psubw, FSUB)
SSE_HELPER_L(psubl, FSUB)
SSE_HELPER_Q(psubq, FSUB)

SSE_HELPER_B(paddusb, FADDUB)
SSE_HELPER_B(paddsb, FADDSB)
SSE_HELPER_B(psubusb, FSUBUB)
SSE_HELPER_B(psubsb, FSUBSB)

SSE_HELPER_W(paddusw, FADDUW)
SSE_HELPER_W(paddsw, FADDSW)
SSE_HELPER_W(psubusw, FSUBUW)
SSE_HELPER_W(psubsw, FSUBSW)

SSE_HELPER_B(pminub, FMINUB)
SSE_HELPER_B(pmaxub, FMAXUB)

SSE_HELPER_W(pminsw, FMINSW)
SSE_HELPER_W(pmaxsw, FMAXSW)

SSE_HELPER_Q(pand, FAND)
SSE_HELPER_Q(pandn, FANDN)
SSE_HELPER_Q(por, FOR)
SSE_HELPER_Q(pxor, FXOR)

SSE_HELPER_B(pcmpgtb, FCMPGTB)
SSE_HELPER_W(pcmpgtw, FCMPGTW)
SSE_HELPER_L(pcmpgtl, FCMPGTL)

SSE_HELPER_B(pcmpeqb, FCMPEQ)
SSE_HELPER_W(pcmpeqw, FCMPEQ)
SSE_HELPER_L(pcmpeql, FCMPEQ)

SSE_HELPER_W(pmullw, FMULLW)
#if SHIFT == 0
DEF_HELPER_3(glue(pmulhrw, SUFFIX), void, env, Reg, Reg)
#endif
SSE_HELPER_W(pmulhuw, FMULHUW)
SSE_HELPER_W(pmulhw, FMULHW)

SSE_HELPER_B(pavgb, FAVG)
SSE_HELPER_W(pavgw, FAVG)

DEF_HELPER_4(glue(pmuludq, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pmaddwd, SUFFIX), void, env, Reg, Reg, Reg)

DEF_HELPER_4(glue(psadbw, SUFFIX), void, env, Reg, Reg, Reg)
#if SHIFT < 2
DEF_HELPER_4(glue(maskmov, SUFFIX), void, env, Reg, Reg, tl)
#endif
DEF_HELPER_2(glue(movl_mm_T0, SUFFIX), void, Reg, i32)
#ifdef TARGET_X86_64
DEF_HELPER_2(glue(movq_mm_T0, SUFFIX), void, Reg, i64)
#endif

#if SHIFT == 0
DEF_HELPER_3(glue(pshufw, SUFFIX), void, Reg, Reg, int)
#else
DEF_HELPER_3(glue(pshufd, SUFFIX), void, Reg, Reg, int)
DEF_HELPER_3(glue(pshuflw, SUFFIX), void, Reg, Reg, int)
DEF_HELPER_3(glue(pshufhw, SUFFIX), void, Reg, Reg, int)
#endif

#if SHIFT >= 1
/* FPU ops */
/* XXX: not accurate */

#define SSE_HELPER_P4(name, ...)                         \
    DEF_HELPER_4(glue(name ## ps, SUFFIX), __VA_ARGS__) \
    DEF_HELPER_4(glue(name ## pd, SUFFIX), __VA_ARGS__)

#define SSE_HELPER_P3(name, ...)                         \
    DEF_HELPER_3(glue(name ## ps, SUFFIX), __VA_ARGS__) \
    DEF_HELPER_3(glue(name ## pd, SUFFIX), __VA_ARGS__)

#if SHIFT == 1
#define SSE_HELPER_S4(name, ...)             \
    SSE_HELPER_P4(name, __VA_ARGS__)         \
    DEF_HELPER_4(name ## ss, __VA_ARGS__)   \
    DEF_HELPER_4(name ## sd, __VA_ARGS__)
#define SSE_HELPER_S3(name, ...)             \
    SSE_HELPER_P3(name, __VA_ARGS__)         \
    DEF_HELPER_3(name ## ss, __VA_ARGS__)   \
    DEF_HELPER_3(name ## sd, __VA_ARGS__)
#else
#define SSE_HELPER_S4(name, ...) SSE_HELPER_P4(name, __VA_ARGS__)
#define SSE_HELPER_S3(name, ...) SSE_HELPER_P3(name, __VA_ARGS__)
#endif

DEF_HELPER_4(glue(shufps, SUFFIX), void, Reg, Reg, Reg, int)
DEF_HELPER_4(glue(shufpd, SUFFIX), void, Reg, Reg, Reg, int)

SSE_HELPER_S4(add, void, env, Reg, Reg, Reg)
SSE_HELPER_S4(sub, void, env, Reg, Reg, Reg)
SSE_HELPER_S4(mul, void, env, Reg, Reg, Reg)
SSE_HELPER_S4(div, void, env, Reg, Reg, Reg)
SSE_HELPER_S4(min, void, env, Reg, Reg, Reg)
SSE_HELPER_S4(max, void, env, Reg, Reg, Reg)

SSE_HELPER_S3(sqrt, void, env, Reg, Reg)

DEF_HELPER_3(glue(cvtps2pd, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(cvtpd2ps, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(cvtdq2ps, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(cvtdq2pd, SUFFIX), void, env, Reg, Reg)

DEF_HELPER_3(glue(cvtps2dq, SUFFIX), void, env, ZMMReg, ZMMReg)
DEF_HELPER_3(glue(cvtpd2dq, SUFFIX), void, env, ZMMReg, ZMMReg)

DEF_HELPER_3(glue(cvttps2dq, SUFFIX), void, env, ZMMReg, ZMMReg)
DEF_HELPER_3(glue(cvttpd2dq, SUFFIX), void, env, ZMMReg, ZMMReg)

#if SHIFT == 1
DEF_HELPER_3(cvtss2sd, void, env, Reg, Reg)
DEF_HELPER_3(cvtsd2ss, void, env, Reg, Reg)
DEF_HELPER_3(cvtpi2ps, void, env, ZMMReg, MMXReg)
DEF_HELPER_3(cvtpi2pd, void, env, ZMMReg, MMXReg)
DEF_HELPER_3(cvtsi2ss, void, env, ZMMReg, i32)
DEF_HELPER_3(cvtsi2sd, void, env, ZMMReg, i32)

#ifdef TARGET_X86_64
DEF_HELPER_3(cvtsq2ss, void, env, ZMMReg, i64)
DEF_HELPER_3(cvtsq2sd, void, env, ZMMReg, i64)
#endif

DEF_HELPER_3(cvtps2pi, void, env, MMXReg, ZMMReg)
DEF_HELPER_3(cvtpd2pi, void, env, MMXReg, ZMMReg)
DEF_HELPER_2(cvtss2si, s32, env, ZMMReg)
DEF_HELPER_2(cvtsd2si, s32, env, ZMMReg)
#ifdef TARGET_X86_64
DEF_HELPER_2(cvtss2sq, s64, env, ZMMReg)
DEF_HELPER_2(cvtsd2sq, s64, env, ZMMReg)
#endif

DEF_HELPER_3(cvttps2pi, void, env, MMXReg, ZMMReg)
DEF_HELPER_3(cvttpd2pi, void, env, MMXReg, ZMMReg)
DEF_HELPER_2(cvttss2si, s32, env, ZMMReg)
DEF_HELPER_2(cvttsd2si, s32, env, ZMMReg)
#ifdef TARGET_X86_64
DEF_HELPER_2(cvttss2sq, s64, env, ZMMReg)
DEF_HELPER_2(cvttsd2sq, s64, env, ZMMReg)
#endif
#endif

DEF_HELPER_3(glue(rsqrtps, SUFFIX), void, env, ZMMReg, ZMMReg)
DEF_HELPER_3(glue(rcpps, SUFFIX), void, env, ZMMReg, ZMMReg)

#if SHIFT == 1
DEF_HELPER_3(rsqrtss, void, env, ZMMReg, ZMMReg)
DEF_HELPER_3(rcpss, void, env, ZMMReg, ZMMReg)
DEF_HELPER_3(extrq_r, void, env, ZMMReg, ZMMReg)
DEF_HELPER_4(extrq_i, void, env, ZMMReg, int, int)
DEF_HELPER_3(insertq_r, void, env, ZMMReg, ZMMReg)
DEF_HELPER_4(insertq_i, void, env, ZMMReg, int, int)
#endif

SSE_HELPER_P4(hadd, void, env, Reg, Reg, Reg)
SSE_HELPER_P4(hsub, void, env, Reg, Reg, Reg)
SSE_HELPER_P4(addsub, void, env, Reg, Reg, Reg)

#define SSE_HELPER_CMP(name, F, C) SSE_HELPER_S4(name, void, env, Reg, Reg, Reg)

SSE_HELPER_CMP(cmpeq, FPU_CMPQ, FPU_EQ)
SSE_HELPER_CMP(cmplt, FPU_CMPS, FPU_LT)
SSE_HELPER_CMP(cmple, FPU_CMPS, FPU_LE)
SSE_HELPER_CMP(cmpunord, FPU_CMPQ,  FPU_UNORD)
SSE_HELPER_CMP(cmpneq, FPU_CMPQ, !FPU_EQ)
SSE_HELPER_CMP(cmpnlt, FPU_CMPS, !FPU_LT)
SSE_HELPER_CMP(cmpnle, FPU_CMPS, !FPU_LE)
SSE_HELPER_CMP(cmpord, FPU_CMPQ, !FPU_UNORD)

SSE_HELPER_CMP(cmpequ, FPU_CMPQ, FPU_EQU)
SSE_HELPER_CMP(cmpnge, FPU_CMPS, !FPU_GE)
SSE_HELPER_CMP(cmpngt, FPU_CMPS, !FPU_GT)
SSE_HELPER_CMP(cmpfalse, FPU_CMPQ,  FPU_FALSE)
SSE_HELPER_CMP(cmpnequ, FPU_CMPQ, FPU_EQU)
SSE_HELPER_CMP(cmpge, FPU_CMPS, FPU_GE)
SSE_HELPER_CMP(cmpgt, FPU_CMPS, FPU_GT)
SSE_HELPER_CMP(cmptrue, FPU_CMPQ,  !FPU_FALSE)

SSE_HELPER_CMP(cmpeqs, FPU_CMPS, FPU_EQ)
SSE_HELPER_CMP(cmpltq, FPU_CMPQ, FPU_LT)
SSE_HELPER_CMP(cmpleq, FPU_CMPQ, FPU_LE)
SSE_HELPER_CMP(cmpunords, FPU_CMPS,  FPU_UNORD)
SSE_HELPER_CMP(cmpneqq, FPU_CMPS, !FPU_EQ)
SSE_HELPER_CMP(cmpnltq, FPU_CMPQ, !FPU_LT)
SSE_HELPER_CMP(cmpnleq, FPU_CMPQ, !FPU_LE)
SSE_HELPER_CMP(cmpords, FPU_CMPS, !FPU_UNORD)

SSE_HELPER_CMP(cmpequs, FPU_CMPS, FPU_EQU)
SSE_HELPER_CMP(cmpngeq, FPU_CMPQ, !FPU_GE)
SSE_HELPER_CMP(cmpngtq, FPU_CMPQ, !FPU_GT)
SSE_HELPER_CMP(cmpfalses, FPU_CMPS,  FPU_FALSE)
SSE_HELPER_CMP(cmpnequs, FPU_CMPS, FPU_EQU)
SSE_HELPER_CMP(cmpgeq, FPU_CMPQ, FPU_GE)
SSE_HELPER_CMP(cmpgtq, FPU_CMPQ, FPU_GT)
SSE_HELPER_CMP(cmptrues, FPU_CMPS,  !FPU_FALSE)

#if SHIFT == 1
DEF_HELPER_3(ucomiss, void, env, Reg, Reg)
DEF_HELPER_3(comiss, void, env, Reg, Reg)
DEF_HELPER_3(ucomisd, void, env, Reg, Reg)
DEF_HELPER_3(comisd, void, env, Reg, Reg)
#endif

DEF_HELPER_2(glue(movmskps, SUFFIX), i32, env, Reg)
DEF_HELPER_2(glue(movmskpd, SUFFIX), i32, env, Reg)
#endif

DEF_HELPER_2(glue(pmovmskb, SUFFIX), i32, env, Reg)
DEF_HELPER_4(glue(packsswb, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(packuswb, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(packssdw, SUFFIX), void, env, Reg, Reg, Reg)
#define UNPCK_OP(name, base)                                       \
    DEF_HELPER_4(glue(punpck ## name ## bw, SUFFIX), void, env, Reg, Reg, Reg) \
    DEF_HELPER_4(glue(punpck ## name ## wd, SUFFIX), void, env, Reg, Reg, Reg) \
    DEF_HELPER_4(glue(punpck ## name ## dq, SUFFIX), void, env, Reg, Reg, Reg)

UNPCK_OP(l, 0)
UNPCK_OP(h, 1)

#if SHIFT >= 1
DEF_HELPER_4(glue(punpcklqdq, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(punpckhqdq, SUFFIX), void, env, Reg, Reg, Reg)
#endif

/* 3DNow! float ops */
#if SHIFT == 0
DEF_HELPER_3(pi2fd, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pi2fw, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pf2id, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pf2iw, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfacc, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfadd, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfcmpeq, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfcmpge, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfcmpgt, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfmax, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfmin, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfmul, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfnacc, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfpnacc, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfrcp, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfrsqrt, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfsub, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pfsubr, void, env, MMXReg, MMXReg)
DEF_HELPER_3(pswapd, void, env, MMXReg, MMXReg)
#endif

/* SSSE3 op helpers */
DEF_HELPER_4(glue(phaddw, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(phaddd, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(phaddsw, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(phsubw, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(phsubd, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(phsubsw, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_3(glue(pabsb, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pabsw, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pabsd, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_4(glue(pmaddubsw, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pmulhrsw, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pshufb, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(psignb, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(psignw, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(psignd, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_5(glue(palignr, SUFFIX), void, env, Reg, Reg, Reg, s32)

/* SSE4.1 op helpers */
#if SHIFT >= 1
DEF_HELPER_5(glue(pblendvb, SUFFIX), void, env, Reg, Reg, Reg, Reg)
DEF_HELPER_5(glue(blendvps, SUFFIX), void, env, Reg, Reg, Reg, Reg)
DEF_HELPER_5(glue(blendvpd, SUFFIX), void, env, Reg, Reg, Reg, Reg)
DEF_HELPER_3(glue(ptest, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pmovsxbw, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pmovsxbd, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pmovsxbq, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pmovsxwd, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pmovsxwq, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pmovsxdq, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pmovzxbw, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pmovzxbd, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pmovzxbq, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pmovzxwd, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pmovzxwq, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(pmovzxdq, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_4(glue(pmuldq, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pcmpeqq, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(packusdw, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pminsb, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pminsd, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pminuw, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pminud, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pmaxsb, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pmaxsd, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pmaxuw, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pmaxud, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(pmulld, SUFFIX), void, env, Reg, Reg, Reg)
#if SHIFT == 1
DEF_HELPER_3(glue(phminposuw, SUFFIX), void, env, Reg, Reg)
#endif
DEF_HELPER_4(glue(roundps, SUFFIX), void, env, Reg, Reg, i32)
DEF_HELPER_4(glue(roundpd, SUFFIX), void, env, Reg, Reg, i32)
#if SHIFT == 1
DEF_HELPER_4(roundss_xmm, void, env, Reg, Reg, i32)
DEF_HELPER_4(roundsd_xmm, void, env, Reg, Reg, i32)
#endif
DEF_HELPER_5(glue(blendps, SUFFIX), void, env, Reg, Reg, Reg, i32)
DEF_HELPER_5(glue(blendpd, SUFFIX), void, env, Reg, Reg, Reg, i32)
DEF_HELPER_5(glue(pblendw, SUFFIX), void, env, Reg, Reg, Reg, i32)
DEF_HELPER_5(glue(dpps, SUFFIX), void, env, Reg, Reg, Reg, i32)
#if SHIFT == 1
DEF_HELPER_5(glue(dppd, SUFFIX), void, env, Reg, Reg, Reg, i32)
#endif
DEF_HELPER_5(glue(mpsadbw, SUFFIX), void, env, Reg, Reg, Reg, i32)
#endif

/* SSE4.2 op helpers */
#if SHIFT >= 1
DEF_HELPER_4(glue(pcmpgtq, SUFFIX), void, env, Reg, Reg, Reg)
#endif
#if SHIFT == 1
DEF_HELPER_4(glue(pcmpestri, SUFFIX), void, env, Reg, Reg, i32)
DEF_HELPER_4(glue(pcmpestrm, SUFFIX), void, env, Reg, Reg, i32)
DEF_HELPER_4(glue(pcmpistri, SUFFIX), void, env, Reg, Reg, i32)
DEF_HELPER_4(glue(pcmpistrm, SUFFIX), void, env, Reg, Reg, i32)
DEF_HELPER_3(crc32, tl, i32, tl, i32)
#endif

/* AES-NI op helpers */
#if SHIFT >= 1
DEF_HELPER_4(glue(aesdec, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(aesdeclast, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(aesenc, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(aesenclast, SUFFIX), void, env, Reg, Reg, Reg)
#if SHIFT == 1
DEF_HELPER_3(glue(aesimc, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_4(glue(aeskeygenassist, SUFFIX), void, env, Reg, Reg, i32)
#endif
DEF_HELPER_5(glue(pclmulqdq, SUFFIX), void, env, Reg, Reg, Reg, i32)
#endif

/* AVX helpers */
#if SHIFT >= 1
DEF_HELPER_3(glue(vbroadcastb, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(vbroadcastw, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(vbroadcastl, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(vbroadcastq, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_4(glue(vpermilpd, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(vpermilps, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(vpermilpd_imm, SUFFIX), void, env, Reg, Reg, i32)
DEF_HELPER_4(glue(vpermilps_imm, SUFFIX), void, env, Reg, Reg, i32)
DEF_HELPER_4(glue(vpsrlvd, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(vpsravd, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(vpsllvd, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(vpsrlvq, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(vpsravq, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_4(glue(vpsllvq, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_3(glue(vtestps, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_3(glue(vtestpd, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_4(glue(vpmaskmovd_st, SUFFIX), void, env, Reg, Reg, tl)
DEF_HELPER_4(glue(vpmaskmovq_st, SUFFIX), void, env, Reg, Reg, tl)
DEF_HELPER_4(glue(vpmaskmovd, SUFFIX), void, env, Reg, Reg, Reg)
DEF_HELPER_5(glue(vpgatherdd0, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherdq0, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherqd0, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherqq0, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherdd1, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherdq1, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherqd1, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherqq1, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherdd2, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherdq2, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherqd2, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherqq2, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherdd3, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherdq3, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherqd3, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_5(glue(vpgatherqq3, SUFFIX), void, env, Reg, Reg, Reg, tl)
DEF_HELPER_4(glue(vpmaskmovq, SUFFIX), void, env, Reg, Reg, Reg)
#if SHIFT == 2
DEF_HELPER_3(glue(vbroadcastdq, SUFFIX), void, env, Reg, Reg)
DEF_HELPER_1(vzeroall, void, env)
DEF_HELPER_1(vzeroupper, void, env)
#ifdef TARGET_X86_64
DEF_HELPER_1(vzeroall_hi8, void, env)
DEF_HELPER_1(vzeroupper_hi8, void, env)
#endif
DEF_HELPER_5(vpermdq_ymm, void, env, Reg, Reg, Reg, i32)
DEF_HELPER_4(vpermq_ymm, void, env, Reg, Reg, i32)
DEF_HELPER_4(vpermd_ymm, void, env, Reg, Reg, Reg)
#endif
#endif

#undef SHIFT
#undef Reg
#undef SUFFIX

#undef SSE_HELPER_B
#undef SSE_HELPER_W
#undef SSE_HELPER_L
#undef SSE_HELPER_Q
#undef SSE_HELPER_S3
#undef SSE_HELPER_S4
#undef SSE_HELPER_P3
#undef SSE_HELPER_P4
#undef SSE_HELPER_CMP
#undef UNPCK_OP
