#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "msp432_boostxl_init.h"
#include "msp432_arm_dsp.h"

#define NUMTAPS 32
int taps[NUMTAPS];
int C[NUMTAPS] = { (int) (1 * (1 << 15)) };

// #define NOISE

uint16_t processSample(uint16_t x) {

#ifdef NOISE
    x = 0x1800 + rand() % 0x1000;
#endif

    int input = adc14_to_q15(x);

    taps[0] = input;

    int q = 0;
    uint16_t i;
    for (i = 0; i<NUMTAPS; i++)
        q += (taps[i] * C[i]) >> 15;

    for (i = NUMTAPS-1; i>0; i--)
        taps[i] = taps[i-1];

    return q15_to_dac14(q);
}


#include <stdio.h>

   int main(void) {
       WDT_A_hold(WDT_A_BASE);

       uint32_t c = measurePerfSample(processSample);

       printf("Cycles: %d\n", c);

       msp432_boostxl_init_intr(FS_32000_HZ, BOOSTXL_J1_2_IN, processSample);
       msp432_boostxl_run();

       return 1;
   }

