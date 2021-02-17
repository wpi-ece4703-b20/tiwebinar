#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "msp432_boostxl_init.h"
#include "msp432_arm_dsp.h"
#include <math.h>
#include <stdlib.h>

int float2q(float x, int f) {
    return (int) floor(x * (1 << f));
}

float q2float(int q, int f) {
    return (float) (q * 1.0f / (1 << f));
}

// This structure stores floating point coefficients
typedef struct sosflp {
    float32_t s[2];  // state
    float32_t b[3];  // nominator coeff  b0 b1 b2
    float32_t a[2];  // denominator coeff   a1 a2
} sosflp_t;

// This structure stores fix<16,15> coefficients
typedef struct sosint {
    int s[2];  // state
    int b[3];  // nominator coeff  b0 b1 b2
    int a[2];  // denominator coeff   a1 a2
} sosint_t;

#define PRECISION 12

void createsos(float32_t b0,
               float32_t b1,
               float32_t b2,
               float32_t a1,
               float32_t a2,
               sosflp_t *pflp,
               sosint_t *pfxp) {
    pflp->b[0] = b0;
    pflp->b[1] = b1;
    pflp->b[2] = b2;
    pflp->a[0] = a1;
    pflp->a[1] = a2;
    pflp->s[0] = pflp->s[1] = 0.0f;

    pfxp->b[0] = float2q(b0, PRECISION);
    pfxp->b[1] = float2q(b1, PRECISION);
    pfxp->b[2] = float2q(b2, PRECISION);
    pfxp->a[0] = float2q(a1, PRECISION);
    pfxp->a[1] = float2q(a2, PRECISION);
    pfxp->s[0] = pfxp->s[1] = float2q(0.0f, PRECISION);
}

float32_t filter_sosflp(float32_t x, sosflp_t *p) {

    float32_t y = (x * p->b[0]) + p->s[0];
    p->s[0]     = (x * p->b[1]) - (y * p->a[0]) + p->s[1];
    p->s[1]     = (x * p->b[2]) - (y * p->a[1]);

    return y;
}

q15_t filter_sosint(int x, sosint_t *p) {

    // input is Q15, convert to QPRECISION
    x = (x >> (15 - PRECISION));

    int y    = ((x * p->b[0]) >> PRECISION) + p->s[0];
    p->s[0]  = ((x * p->b[1]) >> PRECISION) - ((y * p->a[0]) >> PRECISION) + p->s[1];
    p->s[1]  = ((x * p->b[2]) >> PRECISION) - ((y * p->a[1]) >> PRECISION);

    // scale up output from QPRECISION to Q15
    y = (y << (15 - PRECISION));

    return y;
}

sosflp_t stageflp;
sosint_t stageint;

void initsos() {
    createsos(  /* b0 */  0.5f / 3.4730,
                /* b1 */  0.0f / 3.4730,
                /* b2 */  0.5f / 3.4730,
                /* a1 */  -1.663f,
                /* a2 */  0.81f,
                &stageflp,
                &stageint);
}

uint16_t processSample(uint16_t x) {
    float32_t inputflp;
    float32_t yflp;
    int       inputint;
    int       yint;
    int       adc;

    adc = 0x1800 + rand() % 0x1000;

    if (pushButtonLeftDown()) {

        inputflp = adc14_to_f32(adc);
        yflp = filter_sosflp(inputflp, &stageflp);

        inputint = adc14_to_q15(adc);
        yint = filter_sosint(inputint, &stageint);

        return f32_to_dac14(yflp - q2float(yint, 15));;

    } else if (pushButtonRightDown()) {

        inputflp = adc14_to_f32(adc);
        yflp = filter_sosflp(inputflp, &stageflp);

        return f32_to_dac14(yflp);

    } else {

        inputint = adc14_to_q15(adc);
        yint = filter_sosint(inputint, &stageint);

        return q15_to_dac14(yint);

    }

}

#include <stdio.h>

int main(void) {
    WDT_A_hold(WDT_A_BASE);

    initsos();

    msp432_boostxl_init_intr(FS_16000_HZ, BOOSTXL_J1_2_IN, processSample);
    msp432_boostxl_run();

    return 1;
}
