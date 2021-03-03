#define LOC_8_DELAY 154.33+(0.100)*2
#define LOC_15_DELAY 154.33+(0.002)*2+(0.046)*2
#define LOC_16_DELAY 154.33+(0.107)*2+(0.007)*2
#define LOC_20_DELAY 154.33+(-0.004)*2
#define LOC_21_DELAY 154.33+(0.046)*2
#define LOC_22_DELAY 154.33+(0.104)*2
#define LOC_23_DELAY 154.33+(0.024)*2+(-0.003)*2
#define LOC_24_DELAY 154.33+(0.017)*2+(0.07)*2
#define LOC_26_DELAY 154.33+(0.055)*2
#define LOC_27_DELAY 154.33+(0.146)*2+(-0.009)*2
#define LOC_28_DELAY 154.33+(0.182)*2+(-0.011)*2
#define LOC_29_DELAY 154.33+(0.10)*2+(-0.04)*2
#define LOC_31_DELAY 154.33+(0.098)*2+(0.001)*2
#define LOC_34_DELAY 154.33+(-0.022)*2
#define LOC_35_DELAY 154.33+(0.026)*2+(-0.003)*2


#define UAV_7_DELAY 154.33+(0.098)*2+(0.001)*2
#define UAV_12_DELAY 154.33+(0.002)*2+(0.046)*2
#define UAV_13_DELAY 154.33+(0.182)*2+(-0.011)*2
#define UAV_14_DELAY 154.33+(0.046)*2
#define UAV_17_DELAY 154.33+(-0.004)*2
#define UAV_18_DELAY 154.33+(0.146)*2+(-0.009)*2
#define UAV_20_DELAY 154.33+(0.055)*2
#define UAV_24_DELAY 154.33+(0.100)*2
#define UAV_27_DELAY 154.33+(0.024)*2+(-0.003)*2
#define UAV_28_DELAY 154.33+(0.017)*2+(0.07)*2
#define UAV_29_DELAY 154.33+(0.10)*2+(-0.04)*2
#define UAV_33_DELAY 154.33+(0.026)*2+(-0.003)*2
#define UAV_34_DELAY 154.33+(0.104)*2
#define UAV_36_DELAY 154.33+(-0.022)*2
#define UAV_37_DELAY 154.33+(0.107)*2+(0.007)*2

#define UAV_7_LOC_DELAY LOC_31_DELAY
#define UAV_12_LOC_DELAY LOC_15_DELAY
#define UAV_13_LOC_DELAY LOC_28_DELAY
#define UAV_14_LOC_DELAY LOC_21_DELAY
#define UAV_17_LOC_DELAY LOC_20_DELAY
#define UAV_18_LOC_DELAY LOC_27_DELAY
#define UAV_20_LOC_DELAY LOC_26_DELAY
#define UAV_24_LOC_DELAY LOC_8_DELAY
#define UAV_27_LOC_DELAY LOC_23_DELAY
#define UAV_28_LOC_DELAY LOC_24_DELAY
#define UAV_29_LOC_DELAY LOC_29_DELAY
#define UAV_33_LOC_DELAY LOC_35_DELAY
#define UAV_34_LOC_DELAY LOC_22_DELAY
#define UAV_36_LOC_DELAY LOC_34_DELAY
#define UAV_37_LOC_DELAY LOC_16_DELAY

static const double UAV_LOC_DELAY[40]={
    #ifdef UAV_0_LOC_DELAY
        UAV_0_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_1_LOC_DELAY
        UAV_1_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_2_LOC_DELAY
        UAV_2_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_3_LOC_DELAY
        UAV_3_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_4_LOC_DELAY
        UAV_4_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_5_LOC_DELAY
        UAV_5_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_6_LOC_DELAY
        UAV_6_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_7_LOC_DELAY
        UAV_7_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_8_LOC_DELAY
        UAV_8_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_9_LOC_DELAY
        UAV_9_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_10_LOC_DELAY
        UAV_10_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_11_LOC_DELAY
        UAV_11_LOC_DELAY,
    #else
            0,
    #endif
        #ifdef UAV_12_LOC_DELAY
        UAV_12_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_13_LOC_DELAY
        UAV_13_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_14_LOC_DELAY
        UAV_14_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_15_LOC_DELAY
        UAV_15_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_16_LOC_DELAY
        UAV_16_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_17_LOC_DELAY
        UAV_17_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_18_LOC_DELAY
        UAV_18_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_19_LOC_DELAY
        UAV_19_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_20_LOC_DELAY
        UAV_20_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_21_LOC_DELAY
        UAV_21_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_22_LOC_DELAY
        UAV_22_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_23_LOC_DELAY
        UAV_23_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_24_LOC_DELAY
        UAV_24_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_25_LOC_DELAY
        UAV_25_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_26_LOC_DELAY
        UAV_26_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_27_LOC_DELAY
        UAV_27_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_28_LOC_DELAY
        UAV_28_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_29_LOC_DELAY
        UAV_29_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_30_LOC_DELAY
        UAV_30_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_31_LOC_DELAY
        UAV_31_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_32_LOC_DELAY
        UAV_32_LOC_DELAY,
    #else
            0,
    #endif
    #ifdef UAV_33_LOC_DELAY
        UAV_33_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_34_LOC_DELAY
        UAV_34_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_35_LOC_DELAY
        UAV_35_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_36_LOC_DELAY
        UAV_36_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_37_LOC_DELAY
        UAV_37_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_38_LOC_DELAY
        UAV_38_LOC_DELAY,
    #else
            0,
    #endif

    #ifdef UAV_39_LOC_DELAY
        UAV_39_LOC_DELAY,
    #else
            0,
    #endif
};
