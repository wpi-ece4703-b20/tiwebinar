#include "msp432_boostxl_init.h"
#include "msp432_arm_dsp.h"

uint16_t processSample(uint16_t x) {
    float32_t input;
    static float32_t y;
    input = adc14_to_f32(x);
    y     = 0.9 * y + input;
    return f32_to_dac14(y);
}

int main(void) {
    WDT_A_hold(WDT_A_BASE);

    msp432_boostxl_init_intr(FS_16000_HZ,
                             BOOSTXL_J1_2_IN,
                             processSample);
    msp432_boostxl_run();

    return 1;
}

//    input = 0.1 * adc14_to_f32(0x1800 + rand() % 0x1000); // adc14_to_f32(x);
