/**
  ******************************************************************************
  * @file    dpc_ai_controller.c
  * @brief   Firmware-facing wrapper for the recurrent 1DOF DPC X-CUBE-AI model.
  ******************************************************************************
  */

#include "Control/dpc_ai_controller.h"

#if defined(USE_DPC_AI)

#include <math.h>
#include <string.h>

#include "ai_platform.h"
#include "network.h"
#include "network_data.h"
#include "network_data_params.h"
#include "Timing/AppTiming.h"

#if (AI_NETWORK_IN_NUM != 2) || (AI_NETWORK_OUT_NUM != 2)
#error "DPC AI controller expects two network inputs and two network outputs"
#endif

#if (AI_NETWORK_IN_1_SIZE != 6) || (AI_NETWORK_OUT_1_SIZE != 1)
#error "DPC AI controller expects six features and one voltage output"
#endif

#if (AI_NETWORK_IN_2_SIZE != AI_NETWORK_OUT_2_SIZE)
#error "DPC AI controller expects matching recurrent hidden input/output sizes"
#endif

#define DPC_AI_INPUT_FEATURES      (AI_NETWORK_IN_1_SIZE)
#define DPC_AI_HIDDEN_SIZE         (AI_NETWORK_IN_2_SIZE)
#define DPC_AI_OUTPUT_VOLTAGE_SIZE (AI_NETWORK_OUT_1_SIZE)

static const float DPC_YSTD = 5.472512f;
static const float DPC_YMEAN = 15.032558f;
static const float DPC_CUR_MEAN = 14.099108f;
static const float DPC_CUR_STD = 20.243727f;
static const float DPC_INTEG_GAIN = 0.01f;
static const float DPC_INTEG_CLAMP = 30.0f;

static ai_handle s_network = AI_HANDLE_NULL;
AI_ALIGNED(32)
static ai_u64 s_network_weights_ram[AI_NETWORK_DATA_WEIGHTS_SIZE / sizeof(ai_u64)];
AI_ALIGNED(4)
static ai_u8 s_activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

static float s_features[DPC_AI_INPUT_FEATURES];
static float s_hidden_in[DPC_AI_HIDDEN_SIZE];
static float s_voltage_out[DPC_AI_OUTPUT_VOLTAGE_SIZE];
static float s_hidden_out[DPC_AI_HIDDEN_SIZE];

static float s_integrator;
static float s_airgap_prev_mm;
static float s_cn_prev;
static uint32_t s_last_inference_us;
static uint32_t s_max_inference_us;
static int s_initialized;

static float dpc_ai_clampf(float value, float low, float high)
{
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

static float dpc_ai_current_norm(float current_a)
{
  return (current_a - DPC_CUR_MEAN) / DPC_CUR_STD;
}

static int dpc_ai_hidden_is_finite(void)
{
  for (unsigned int i = 0; i < DPC_AI_HIDDEN_SIZE; ++i) {
    if (!isfinite(s_hidden_out[i])) {
      return 0;
    }
  }
  return 1;
}

static dpc_ai_output_t dpc_ai_safe_output(void)
{
  dpc_ai_output_t out;
  out.voltage_v = 0.0f;
  out.duty_percent = 0.0f;
  out.ok = 0;
  return out;
}

int dpc_ai_init(void)
{
  ai_error err;
  const ai_handle activations[] = {
    AI_HANDLE_PTR(s_activations),
  };
  const ai_handle weights[] = {
    AI_HANDLE_PTR(s_network_weights_ram),
  };

  if (s_initialized) {
    return 1;
  }

  memcpy(s_network_weights_ram, s_network_weights_array_u64, sizeof(s_network_weights_ram));

  err = ai_network_create_and_init(&s_network, activations, weights);
  if (err.type != AI_ERROR_NONE) {
    s_network = AI_HANDLE_NULL;
    s_initialized = 0;
    return 0;
  }

  dpc_ai_reset(0.0f, 0.0f);
  s_initialized = 1;
  return 1;
}

void dpc_ai_reset(float airgap_mm, float current_a)
{
  memset(s_hidden_in, 0, sizeof(s_hidden_in));
  memset(s_hidden_out, 0, sizeof(s_hidden_out));
  memset(s_features, 0, sizeof(s_features));
  s_voltage_out[0] = 0.0f;
  s_integrator = 0.0f;
  s_airgap_prev_mm = airgap_mm;
  s_cn_prev = dpc_ai_current_norm(current_a);
  s_last_inference_us = 0U;
  s_max_inference_us = 0U;
}

dpc_ai_output_t dpc_ai_step(float airgap_mm, float target_mm, float current_a, float vbat_v)
{
  ai_i32 batch_count;
  ai_u16 input_count = 0;
  ai_u16 output_count = 0;
  ai_buffer *inputs;
  ai_buffer *outputs;
  dpc_ai_output_t out = dpc_ai_safe_output();
  const float err = target_mm - airgap_mm;
  const float err_prev = target_mm - s_airgap_prev_mm;
  const float cn = dpc_ai_current_norm(current_a);
  uint32_t inference_start;

  s_last_inference_us = 0U;

  if (!s_initialized || s_network == AI_HANDLE_NULL) {
    return out;
  }
  if (!isfinite(airgap_mm) || !isfinite(target_mm) || !isfinite(current_a) || !isfinite(vbat_v)) {
    return out;
  }
  if (vbat_v <= 0.0f) {
    return out;
  }

  s_integrator = dpc_ai_clampf(
    s_integrator + err * DPC_INTEG_GAIN,
    -DPC_INTEG_CLAMP,
    DPC_INTEG_CLAMP
  );

  s_features[0] = err / DPC_YSTD;
  s_features[1] = (err - err_prev) / DPC_YSTD;
  s_features[2] = s_integrator / DPC_INTEG_CLAMP;
  s_features[3] = cn;
  s_features[4] = cn - s_cn_prev;
  s_features[5] = (target_mm - DPC_YMEAN) / DPC_YSTD;

  inputs = ai_network_inputs_get(s_network, &input_count);
  outputs = ai_network_outputs_get(s_network, &output_count);
  if (inputs == NULL || outputs == NULL || input_count != AI_NETWORK_IN_NUM || output_count != AI_NETWORK_OUT_NUM) {
    return out;
  }

  inputs[0].data = AI_HANDLE_PTR(s_features);
  inputs[1].data = AI_HANDLE_PTR(s_hidden_in);
  outputs[0].data = AI_HANDLE_PTR(s_voltage_out);
  outputs[1].data = AI_HANDLE_PTR(s_hidden_out);

  inference_start = app_timing_cycles();
  batch_count = ai_network_run(s_network, inputs, outputs);
  s_last_inference_us = app_timing_elapsed_cycles(inference_start);
  if (s_last_inference_us > s_max_inference_us) {
    s_max_inference_us = s_last_inference_us;
  }
  if (batch_count != 1) {
    return out;
  }
  if (!isfinite(s_voltage_out[0]) || !dpc_ai_hidden_is_finite()) {
    return out;
  }

  memcpy(s_hidden_in, s_hidden_out, sizeof(s_hidden_in));
  s_airgap_prev_mm = airgap_mm;
  s_cn_prev = cn;

  out.voltage_v = s_voltage_out[0];
  out.duty_percent = dpc_ai_clampf((out.voltage_v / vbat_v) * 100.0f, -100.0f, 100.0f);
  out.ok = 1;
  return out;
}

uint32_t dpc_ai_last_inference_us(void)
{
  return s_last_inference_us;
}

uint32_t dpc_ai_max_inference_us(void)
{
  return s_max_inference_us;
}

#endif /* USE_DPC_AI */
