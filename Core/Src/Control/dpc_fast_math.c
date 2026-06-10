#ifdef USE_DPC_AI

#include <float.h>
#include <stdint.h>

static float dpc_fast_pow2i(int n) {
    if (n < -126) {
        return 0.0f;
    }
    if (n > 127) {
        return FLT_MAX;
    }

    union {
        uint32_t u;
        float f;
    } v = {.u = (uint32_t)(n + 127) << 23};
    return v.f;
}

float expf(float x) {
    if (x != x) {
        return x;
    }
    if (x > 88.0f) {
        return FLT_MAX;
    }
    if (x < -87.0f) {
        return 0.0f;
    }

    const float inv_ln2 = 1.4426950408889634f;
    const float ln2 = 0.6931471805599453f;

    const float scaled = x * inv_ln2;
    int n = (int)scaled;
    if (scaled < (float)n) {
        --n;
    }

    const float r = x - ((float)n * ln2);
    const float r2 = r * r;
    const float r3 = r2 * r;
    const float r4 = r2 * r2;
    const float r5 = r4 * r;

    const float poly = 1.0f + r + (0.5f * r2) + (0.1666666716f * r3) +
                       (0.0416666679f * r4) + (0.0083333310f * r5);

    return poly * dpc_fast_pow2i(n);
}

float tanhf(float x) {
    if (x != x) {
        return x;
    }
    if (x >= 3.0f) {
        return 1.0f;
    }
    if (x <= -3.0f) {
        return -1.0f;
    }

    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + (9.0f * x2));
}

#endif
