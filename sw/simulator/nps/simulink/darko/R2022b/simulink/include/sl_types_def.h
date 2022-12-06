/* Copyright 2009-2022 The MathWorks, Inc. */

/*
 * File: sl_types_def.h
 *
 * Abstract:
 *   The embedded RTW code formats do not include simstruc.h, but
 *   needs these common types.
 */

#ifndef __SL_TYPES_DEF_H__
#define __SL_TYPES_DEF_H__

#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
/* SLSize is first defined in tmwtypes.h, however,
   some codegen targets' rtwtypes.h do not include
   tmwtypes.h, so SLSize is defined here. */
#ifndef SLSIZE_SLINDEX
#define SLSIZE_SLINDEX
#ifdef INT_TYPE_64_IS_SUPPORTED
typedef int64_T SLIndex;
typedef int64_T SLSize;
#else
typedef int SLIndex;
typedef int SLSize;
#endif
#endif
#endif

/* The following section is inlined from builtin_typeid_types.h */
#ifndef BUILTIN_TYPEID_TYPES
#define BUILTIN_TYPEID_TYPES

/* Enumeration of built-in data types */
typedef enum {
    SS_DOUBLE = 0, /* real_T    */
    SS_SINGLE = 1, /* real32_T  */
    SS_INT8 = 2,   /* int8_T    */
    SS_UINT8 = 3,  /* uint8_T   */
    SS_INT16 = 4,  /* int16_T   */
    SS_UINT16 = 5, /* uint16_T  */
    SS_INT32 = 6,  /* int32_T   */
    SS_UINT32 = 7, /* uint32_T  */
    SS_BOOLEAN = 8 /* boolean_T */
} BuiltInDTypeId;

#define SS_NUM_BUILT_IN_DTYPE ((int_T)SS_BOOLEAN + 1)

#ifndef _DTYPEID
#define _DTYPEID
/* Enumeration for MAT-file logging code */
typedef int_T DTypeId;
#endif

typedef enum {
    SS_FCN_CALL = 9,
    SS_INTEGER = 10,
    SS_POINTER = 11,
    SS_INTERNAL_DTYPE2 = 12,
    SS_TIMER_UINT32_PAIR = 13,
    SS_CONNECTION_TYPE = 14

    /* if last in list changes also update define below */

} PreDefinedDTypeId;

#endif /* BUILTIN_TYPEID_TYPES */

#define SS_DOUBLE_UINT32 SS_TIMER_UINT32_PAIR

#define SS_NUM_PREDEFINED_DTYPES 6

#ifndef SYMBOLIC_DIMS_ID
#define SYMBOLIC_DIMS_ID
typedef int_T SymbDimsId;
#endif /* SYMBOLIC_DIMS_ID */

#ifndef PRE_DEFINED_SYMBOLIC_DIMS_IDS
#define PRE_DEFINED_SYMBOLIC_DIMS_IDS
/*
 * Definition of Simulink builtin symbolic dimensions IDs
 */
enum {
    SL_INVALID_SYMBDIMS_ID = -2,
    SL_INHERIT = -1, /* must be the same as DYNAMICALLY_TYPED = -1 */
    SL_NOSYMBDIMS = 0
};
#endif /* PRE_DEFINED_SYMBOLIC_DIMS_IDS */

/*
 * DYNAMICALLY_TYPED - Specify for input/output port data types that can
 * accept a variety of data types.
 */
#define DYNAMICALLY_TYPED (-1)

#endif /* __SL_TYPES_DEF_H__ */
