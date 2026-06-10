/**
  ******************************************************************************
  * @file    dpc_ai_controller.h
  * @brief   Thin recurrent DPC controller wrapper around the generated
  *          X-CUBE-AI network.
  ******************************************************************************
  */

#ifndef DPC_AI_CONTROLLER_H
#define DPC_AI_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float voltage_v;
    float duty_percent;
    int ok;
} dpc_ai_output_t;

int dpc_ai_init(void);
void dpc_ai_reset(float airgap_mm, float current_a);
dpc_ai_output_t dpc_ai_step(float airgap_mm, float target_mm, float current_a, float vbat_v);
uint32_t dpc_ai_last_inference_us(void);
uint32_t dpc_ai_max_inference_us(void);

#ifdef __cplusplus
}
#endif

#endif /* DPC_AI_CONTROLLER_H */
