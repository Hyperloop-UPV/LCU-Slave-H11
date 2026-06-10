/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-06-10T14:40:12+0200
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */


#include "network.h"
#include "network_data.h"

#include "ai_platform.h"
#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"



#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_network
 
#undef AI_NETWORK_MODEL_SIGNATURE
#define AI_NETWORK_MODEL_SIGNATURE     "0xc32a9fab1c0780c11466a217fb05e4e8"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "2026-06-10T14:40:12+0200"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_N_BATCHES
#define AI_NETWORK_N_BATCHES         (1)

static ai_ptr g_network_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_network_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  hidden_in_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 32, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Gemm_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 96, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Split_1_output_0_output0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Split_1_output_0_output1_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Split_1_output_0_output2_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  features_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 6, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 96, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Split_output_0_output0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Split_output_0_output1_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Split_output_0_output2_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Add_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Sigmoid_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Add_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Sigmoid_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Mul_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Add_2_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Tanh_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Sub_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Mul_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  hidden_out_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 32, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  _head_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  _Mul_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  _Tanh_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#25 */
AI_ARRAY_OBJ_DECLARE(
  voltage_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 1, AI_STATIC)

/* Array#26 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Gemm_1_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3072, AI_STATIC)

/* Array#27 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Gemm_1_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 96, AI_STATIC)

/* Array#28 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Split_1_output_0_num_or_size_splits_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 1, AI_STATIC)

/* Array#29 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 576, AI_STATIC)

/* Array#30 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 96, AI_STATIC)

/* Array#31 */
AI_ARRAY_OBJ_DECLARE(
  _gru_Split_output_0_num_or_size_splits_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 1, AI_STATIC)

/* Array#32 */
AI_ARRAY_OBJ_DECLARE(
  _head_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#33 */
AI_ARRAY_OBJ_DECLARE(
  _head_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#34 */
AI_ARRAY_OBJ_DECLARE(
  _Mul_output_0_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#35 */
AI_ARRAY_OBJ_DECLARE(
  _Mul_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#36 */
AI_ARRAY_OBJ_DECLARE(
  voltage_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  _Mul_output_0_bias, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_Mul_output_0_bias_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  _Mul_output_0_output, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_Mul_output_0_output_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  _Mul_output_0_scale, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_Mul_output_0_scale_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_output_0_output, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_Slice_output_0_output_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_output_0_output, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_Sub_output_0_output_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  _Tanh_output_0_output, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_Tanh_output_0_output_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Add_1_output_0_output, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Add_1_output_0_output_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Add_2_output_0_output, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Add_2_output_0_output_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Add_output_0_output, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Add_output_0_output_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Gemm_1_output_0_bias, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 1), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &_gru_Gemm_1_output_0_bias_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Gemm_1_output_0_output, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 1), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &_gru_Gemm_1_output_0_output_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Gemm_1_output_0_weights, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 32, 96, 1, 1), AI_STRIDE_INIT(4, 4, 128, 12288, 12288),
  1, &_gru_Gemm_1_output_0_weights_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Gemm_output_0_bias, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 1), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &_gru_Gemm_output_0_bias_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Gemm_output_0_output, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 1), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &_gru_Gemm_output_0_output_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Gemm_output_0_weights, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 6, 96, 1, 1), AI_STRIDE_INIT(4, 4, 24, 2304, 2304),
  1, &_gru_Gemm_output_0_weights_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Mul_1_output_0_output, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Mul_1_output_0_output_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Mul_output_0_output, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Mul_output_0_output_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Sigmoid_1_output_0_output, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Sigmoid_1_output_0_output_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Sigmoid_output_0_output, AI_STATIC,
  18, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Sigmoid_output_0_output_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Split_1_output_0_num_or_size_splits, AI_STATIC,
  19, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_gru_Split_1_output_0_num_or_size_splits_array, NULL)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Split_1_output_0_output0, AI_STATIC,
  20, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Split_1_output_0_output0_array, NULL)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Split_1_output_0_output1, AI_STATIC,
  21, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Split_1_output_0_output1_array, NULL)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Split_1_output_0_output2, AI_STATIC,
  22, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Split_1_output_0_output2_array, NULL)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Split_output_0_num_or_size_splits, AI_STATIC,
  23, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_gru_Split_output_0_num_or_size_splits_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Split_output_0_output0, AI_STATIC,
  24, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Split_output_0_output0_array, NULL)

/* Tensor #25 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Split_output_0_output1, AI_STATIC,
  25, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Split_output_0_output1_array, NULL)

/* Tensor #26 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Split_output_0_output2, AI_STATIC,
  26, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Split_output_0_output2_array, NULL)

/* Tensor #27 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Sub_output_0_output, AI_STATIC,
  27, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Sub_output_0_output_array, NULL)

/* Tensor #28 */
AI_TENSOR_OBJ_DECLARE(
  _gru_Tanh_output_0_output, AI_STATIC,
  28, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_gru_Tanh_output_0_output_array, NULL)

/* Tensor #29 */
AI_TENSOR_OBJ_DECLARE(
  _head_Gemm_output_0_bias, AI_STATIC,
  29, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_head_Gemm_output_0_bias_array, NULL)

/* Tensor #30 */
AI_TENSOR_OBJ_DECLARE(
  _head_Gemm_output_0_output, AI_STATIC,
  30, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_head_Gemm_output_0_output_array, NULL)

/* Tensor #31 */
AI_TENSOR_OBJ_DECLARE(
  _head_Gemm_output_0_weights, AI_STATIC,
  31, 0x0,
  AI_SHAPE_INIT(4, 32, 1, 1, 1), AI_STRIDE_INIT(4, 4, 128, 128, 128),
  1, &_head_Gemm_output_0_weights_array, NULL)

/* Tensor #32 */
AI_TENSOR_OBJ_DECLARE(
  features_output, AI_STATIC,
  32, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 1), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &features_output_array, NULL)

/* Tensor #33 */
AI_TENSOR_OBJ_DECLARE(
  hidden_in_output, AI_STATIC,
  33, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &hidden_in_output_array, NULL)

/* Tensor #34 */
AI_TENSOR_OBJ_DECLARE(
  hidden_out_output, AI_STATIC,
  34, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &hidden_out_output_array, NULL)

/* Tensor #35 */
AI_TENSOR_OBJ_DECLARE(
  voltage_output, AI_STATIC,
  35, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &voltage_output_array, NULL)

/* Tensor #36 */
AI_TENSOR_OBJ_DECLARE(
  voltage_scale, AI_STATIC,
  36, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &voltage_scale_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  voltage_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Tanh_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &voltage_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &voltage_scale, &_Mul_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  voltage_layer, 26,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &voltage_chain,
  NULL, &voltage_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Tanh_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Tanh_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Tanh_output_0_layer, 24,
  NL_TYPE, 0x0, NULL,
  nl, forward_tanh,
  &_Tanh_output_0_chain,
  NULL, &voltage_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_head_Gemm_output_0_output, &_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_output_0_layer, 23,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_output_0_chain,
  NULL, &_Tanh_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Mul_output_0_scale, &_Mul_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Mul_output_0_layer, 22,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_Mul_output_0_chain,
  NULL, &_Sub_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_u8 _Slice_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_output_0_axes_data, _Slice_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_output_0_starts_data[] = { 0 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_output_0_starts_data, _Slice_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_output_0_ends_data[] = { 1 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_output_0_ends_data, _Slice_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &features_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_output_0_layer, 20,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_output_0_chain,
  NULL, &_Mul_output_0_layer, AI_STATIC, 
  .axes = &_Slice_output_0_axes, 
  .starts = &_Slice_output_0_starts, 
  .ends = &_Slice_output_0_ends, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _head_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &hidden_out_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_head_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_head_Gemm_output_0_weights, &_head_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _head_Gemm_output_0_layer, 15,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_head_Gemm_output_0_chain,
  NULL, &_Slice_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  hidden_out_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_gru_Mul_1_output_0_output, &_gru_Tanh_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &hidden_out_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  hidden_out_layer, 14,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &hidden_out_chain,
  NULL, &_head_Gemm_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _gru_Mul_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_gru_Sub_output_0_output, &_gru_Sigmoid_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Mul_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _gru_Mul_1_output_0_layer, 13,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_gru_Mul_1_output_0_chain,
  NULL, &hidden_out_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _gru_Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &hidden_in_output, &_gru_Tanh_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _gru_Sub_output_0_layer, 12,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_gru_Sub_output_0_chain,
  NULL, &_gru_Mul_1_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _gru_Tanh_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Add_2_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Tanh_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _gru_Tanh_output_0_layer, 11,
  NL_TYPE, 0x0, NULL,
  nl, forward_tanh,
  &_gru_Tanh_output_0_chain,
  NULL, &_gru_Sub_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _gru_Add_2_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_gru_Split_output_0_output2, &_gru_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Add_2_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _gru_Add_2_output_0_layer, 10,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_gru_Add_2_output_0_chain,
  NULL, &_gru_Tanh_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _gru_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_gru_Split_1_output_0_output2, &_gru_Sigmoid_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _gru_Mul_output_0_layer, 9,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_gru_Mul_output_0_chain,
  NULL, &_gru_Add_2_output_0_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _gru_Sigmoid_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Sigmoid_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _gru_Sigmoid_output_0_layer, 6,
  NL_TYPE, 0x0, NULL,
  nl, forward_sigmoid,
  &_gru_Sigmoid_output_0_chain,
  NULL, &_gru_Mul_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _gru_Add_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_gru_Split_1_output_0_output0, &_gru_Split_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _gru_Add_output_0_layer, 5,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_gru_Add_output_0_chain,
  NULL, &_gru_Sigmoid_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _gru_Sigmoid_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Add_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Sigmoid_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _gru_Sigmoid_1_output_0_layer, 8,
  NL_TYPE, 0x0, NULL,
  nl, forward_sigmoid,
  &_gru_Sigmoid_1_output_0_chain,
  NULL, &_gru_Add_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _gru_Add_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_gru_Split_1_output_0_output1, &_gru_Split_output_0_output1),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Add_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _gru_Add_1_output_0_layer, 7,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_gru_Add_1_output_0_chain,
  NULL, &_gru_Sigmoid_1_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _gru_Split_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_gru_Split_output_0_output0, &_gru_Split_output_0_output1, &_gru_Split_output_0_output2),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Split_output_0_num_or_size_splits),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _gru_Split_output_0_layer, 2,
  SPLIT_TYPE, 0x0, NULL,
  split, forward_split,
  &_gru_Split_output_0_chain,
  NULL, &_gru_Add_1_output_0_layer, AI_STATIC, 
  .outer_elems = 1, 
  .outer_elems_stride = 384, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _gru_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &features_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_gru_Gemm_output_0_weights, &_gru_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _gru_Gemm_output_0_layer, 1,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_gru_Gemm_output_0_chain,
  NULL, &_gru_Split_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _gru_Split_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Gemm_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_gru_Split_1_output_0_output0, &_gru_Split_1_output_0_output1, &_gru_Split_1_output_0_output2),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Split_1_output_0_num_or_size_splits),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _gru_Split_1_output_0_layer, 4,
  SPLIT_TYPE, 0x0, NULL,
  split, forward_split,
  &_gru_Split_1_output_0_chain,
  NULL, &_gru_Gemm_output_0_layer, AI_STATIC, 
  .outer_elems = 1, 
  .outer_elems_stride = 384, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _gru_Gemm_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &hidden_in_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_gru_Gemm_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_gru_Gemm_1_output_0_weights, &_gru_Gemm_1_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _gru_Gemm_1_output_0_layer, 3,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_gru_Gemm_1_output_0_chain,
  NULL, &_gru_Split_1_output_0_layer, AI_STATIC, 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 15512, 1, 1),
    15512, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 1304, 1, 1),
    1304, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &features_output, &hidden_in_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &voltage_output, &hidden_out_output),
  &_gru_Gemm_1_output_0_layer, 0xc77b02df, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 15512, 1, 1),
      15512, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 1304, 1, 1),
      1304, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &features_output, &hidden_in_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &voltage_output, &hidden_out_output),
  &_gru_Gemm_1_output_0_layer, 0xc77b02df, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_network_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    features_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    features_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    hidden_in_output_array.data = AI_PTR(g_network_activations_map[0] + 24);
    hidden_in_output_array.data_start = AI_PTR(g_network_activations_map[0] + 24);
    _gru_Gemm_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 152);
    _gru_Gemm_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 152);
    _gru_Split_1_output_0_output0_array.data = AI_PTR(g_network_activations_map[0] + 536);
    _gru_Split_1_output_0_output0_array.data_start = AI_PTR(g_network_activations_map[0] + 536);
    _gru_Split_1_output_0_output1_array.data = AI_PTR(g_network_activations_map[0] + 664);
    _gru_Split_1_output_0_output1_array.data_start = AI_PTR(g_network_activations_map[0] + 664);
    _gru_Split_1_output_0_output2_array.data = AI_PTR(g_network_activations_map[0] + 792);
    _gru_Split_1_output_0_output2_array.data_start = AI_PTR(g_network_activations_map[0] + 792);
    _gru_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 152);
    _gru_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 152);
    _gru_Split_output_0_output0_array.data = AI_PTR(g_network_activations_map[0] + 920);
    _gru_Split_output_0_output0_array.data_start = AI_PTR(g_network_activations_map[0] + 920);
    _gru_Split_output_0_output1_array.data = AI_PTR(g_network_activations_map[0] + 1048);
    _gru_Split_output_0_output1_array.data_start = AI_PTR(g_network_activations_map[0] + 1048);
    _gru_Split_output_0_output2_array.data = AI_PTR(g_network_activations_map[0] + 1176);
    _gru_Split_output_0_output2_array.data_start = AI_PTR(g_network_activations_map[0] + 1176);
    _gru_Add_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 152);
    _gru_Add_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 152);
    _gru_Sigmoid_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 280);
    _gru_Sigmoid_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 280);
    _gru_Add_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 152);
    _gru_Add_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 152);
    _gru_Sigmoid_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 408);
    _gru_Sigmoid_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 408);
    _gru_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 152);
    _gru_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 152);
    _gru_Add_2_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 408);
    _gru_Add_2_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 408);
    _gru_Tanh_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 152);
    _gru_Tanh_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 152);
    _gru_Sub_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 408);
    _gru_Sub_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 408);
    _gru_Mul_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 24);
    _gru_Mul_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 24);
    hidden_out_output_array.data = AI_PTR(g_network_activations_map[0] + 280);
    hidden_out_output_array.data_start = AI_PTR(g_network_activations_map[0] + 280);
    _head_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 24);
    _head_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 24);
    _Slice_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 28);
    _Slice_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 28);
    _Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _Sub_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 4);
    _Sub_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 4);
    _Tanh_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _Tanh_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    voltage_output_array.data = AI_PTR(g_network_activations_map[0] + 4);
    voltage_output_array.data_start = AI_PTR(g_network_activations_map[0] + 4);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_ACTIVATIONS);
  return false;
}




/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_weights(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_weights_map(g_network_weights_map, 1, params)) {
    /* Updating weights (byte) offsets */
    
    _gru_Gemm_1_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _gru_Gemm_1_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 0);
    _gru_Gemm_1_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 0);
    _gru_Gemm_1_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _gru_Gemm_1_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 12288);
    _gru_Gemm_1_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 12288);
    _gru_Split_1_output_0_num_or_size_splits_array.format |= AI_FMT_FLAG_CONST;
    _gru_Split_1_output_0_num_or_size_splits_array.data = AI_PTR(g_network_weights_map[0] + 12672);
    _gru_Split_1_output_0_num_or_size_splits_array.data_start = AI_PTR(g_network_weights_map[0] + 12672);
    _gru_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _gru_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 12676);
    _gru_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 12676);
    _gru_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _gru_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 14980);
    _gru_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 14980);
    _gru_Split_output_0_num_or_size_splits_array.format |= AI_FMT_FLAG_CONST;
    _gru_Split_output_0_num_or_size_splits_array.data = AI_PTR(g_network_weights_map[0] + 15364);
    _gru_Split_output_0_num_or_size_splits_array.data_start = AI_PTR(g_network_weights_map[0] + 15364);
    _head_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _head_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 15368);
    _head_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 15368);
    _head_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _head_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 15496);
    _head_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 15496);
    _Mul_output_0_scale_array.format |= AI_FMT_FLAG_CONST;
    _Mul_output_0_scale_array.data = AI_PTR(g_network_weights_map[0] + 15500);
    _Mul_output_0_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 15500);
    _Mul_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _Mul_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 15504);
    _Mul_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 15504);
    voltage_scale_array.format |= AI_FMT_FLAG_CONST;
    voltage_scale_array.data = AI_PTR(g_network_weights_map[0] + 15508);
    voltage_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 15508);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_WEIGHTS);
  return false;
}


/**  PUBLIC APIs SECTION  *****************************************************/



AI_DEPRECATED
AI_API_ENTRY
ai_bool ai_network_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 5072,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0xc77b02df,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}



AI_API_ENTRY
ai_bool ai_network_get_report(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 5072,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0xc77b02df,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}


AI_API_ENTRY
ai_error ai_network_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}


AI_API_ENTRY
ai_error ai_network_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    AI_CONTEXT_OBJ(&AI_NET_OBJ_INSTANCE),
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}


AI_API_ENTRY
ai_error ai_network_create_and_init(
  ai_handle* network, const ai_handle activations[], const ai_handle weights[])
{
  ai_error err;
  ai_network_params params;

  err = ai_network_create(network, AI_NETWORK_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
    return err;
  }
  
  if (ai_network_data_params_get(&params) != true) {
    err = ai_network_get_error(*network);
    return err;
  }
#if defined(AI_NETWORK_DATA_ACTIVATIONS_COUNT)
  /* set the addresses of the activations buffers */
  for (ai_u16 idx=0; activations && idx<params.map_activations.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, idx, activations[idx]);
  }
#endif
#if defined(AI_NETWORK_DATA_WEIGHTS_COUNT)
  /* set the addresses of the weight buffers */
  for (ai_u16 idx=0; weights && idx<params.map_weights.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_weights, idx, weights[idx]);
  }
#endif
  if (ai_network_init(*network, &params) != true) {
    err = ai_network_get_error(*network);
  }
  return err;
}


AI_API_ENTRY
ai_buffer* ai_network_inputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_inputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_buffer* ai_network_outputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_outputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_handle ai_network_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}


AI_API_ENTRY
ai_bool ai_network_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = AI_NETWORK_OBJ(ai_platform_network_init(network, params));
  ai_bool ok = true;

  if (!net_ctx) return false;
  ok &= network_configure_weights(net_ctx, params);
  ok &= network_configure_activations(net_ctx, params);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_network_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}


AI_API_ENTRY
ai_i32 ai_network_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_NETWORK_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

