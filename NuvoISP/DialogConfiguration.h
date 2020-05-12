/*

// TN020: ID Naming and Numbering Conventions
// https://msdn.microsoft.com/en-us/library/t2zechd4.aspx

The ID-Numbering Convention
The following table lists the valid ranges for the IDs of the specific types. Some of the limits are technical implementation limits, and others are conventions that are designed to prevent your IDs from colliding with Windows predefined IDs or MFC default implementations.
We strongly recommend that you define all IDs inside the recommended ranges. The lower limit of these ranges is 1 because 0 is not used. We recommend that you use the common convention and use 100 or 101 as the first ID.
Prefix              Resource type               Valid range
IDR_                multiple                    1 through 0x6FFF
IDD_                dialog templates            1 through 0x6FFF
IDC_,IDI_,IDB_      cursors, icons, bitmaps     1 through 0x6FFF
IDS_, IDP_          general strings             1 through 0x7FFF
ID_                 commands                    0x8000 through 0xDFFF
IDC_                controls                    8 through 0xDFFF

Reasons for these range limits:
    By convention, the ID value of 0 is not used.

    Windows implementation limitations restrict true resource IDs to be less than or equal to 0x7FFF.

    MFC's internal framework reserves these ranges:
        0x7000 through 0x7FFF (see afxres.h)
        0xE000 through 0xEFFF (see afxres.h)
        16000 through 18000 (see afxribbonres.h)
    These ranges may change in future MFC implementations.

    Several Windows system commands use the range of 0xF000 through 0xFFFF.

    Control IDs of 1 through 7 are reserved for standard controls such as IDOK and IDCANCEL.

    The range of 0x8000 through 0xFFFF for strings is reserved for menu prompts for commands.
*/


// Group Box
#define IDC_GROUP_CLOCK_SOURCE              3000
#define IDC_GROUP_BOOT_SELECT               3001
#define IDC_GROUP_BROWN_OUT_VOLTAGE         3002
#define IDC_GROUP_BROWN_OUT_RESET_VOLTAGE   IDC_GROUP_BROWN_OUT_VOLTAGE
#define IDC_GROUP_GPF                       3003
#define IDC_GROUP_GPF2                      3004
#define IDC_GROUP_PA8_STATE                 3005
#define IDC_GROUP_IO_STATE                  3006
#define IDC_GROUP_CONFIG_VALUE              3007
#define IDC_GROUP_CONFIG                    IDC_GROUP_CONFIG_VALUE
#define IDC_GROUP_DATA_FLASH                3008
#define IDC_GROUP_HXT_GAIN                  3009
#define IDC_GROUP_RC_TRIM4                  3010
#define IDC_GROUP_MII_MODE                  3011
#define IDC_GROUP_SPIM_SELECT               3012
#define IDC_GROUP_UART1_SELECT              3013
#define IDC_GROUP_LVR_LEVEL                 3014
#define IDC_GROUP_HXT_MODE                  3015
#define IDC_GROUP_CHIPRESET_TIMEEXT         3016
#define IDC_GROUP_RST_PIN_WIDTH             3017
#define IDC_GROUP_NSCBA                     3018
#define IDC_GROUP_TWO_LEVEL_LOCK            3019
#define IDC_GROUP_DPMS                      3020
#define IDC_GROUP_DPMNS                     3021
#define IDC_GROUP_DPMS_PASSWORD             3022
#define IDC_GROUP_DPMNS_PASSWORD            3023
#define IDC_GROUP_PLM_STAGE                 3024
#define IDC_GROUP_ADVANCE_LOCK              3025

// Clock Options
#define IDC_RADIO_CLK_HXT                   3026
#define IDC_RADIO_CLK_E12M                  IDC_RADIO_CLK_HXT
#define IDC_RADIO_CLK_E12M6M                IDC_RADIO_CLK_HXT
#define IDC_RADIO_CLK_HIRC                  3027
#define IDC_RADIO_CLK_I22M                  IDC_RADIO_CLK_HIRC
#define IDC_RADIO_CLK_I24M                  IDC_RADIO_CLK_HIRC
#define IDC_CHECK_CLOCK_FILTER_ENABLE       3028

// Boot Options
#define IDC_CHECK_BS_MKROM                  3030
#define IDC_CHECK_BOOT_LOADER               IDC_CHECK_BS_MKROM
#define IDC_RADIO_BS_LDROM                  3031
#define IDC_RADIO_BS_APROM                  3032
#define IDC_RADIO_BS_APROM_LDROM            3033
#define IDC_RADIO_BS_LDROM_APROM            3034

// Brown-out Voltage Options
#define IDC_RADIO_BOV_DISABLE               3039
#define IDC_RADIO_BOV_0                     3040
#define IDC_RADIO_BOV_1                     3041
#define IDC_RADIO_BOV_2                     3042
#define IDC_RADIO_BOV_3                     3043
#define IDC_RADIO_BOV_4                     3044
#define IDC_RADIO_BOV_5                     3045
#define IDC_RADIO_BOV_6                     3046
#define IDC_RADIO_BOV_7                     3047
#define IDC_RADIO_BOV_8                     3048
#define IDC_RADIO_BOV_9                     3049
#define IDC_RADIO_BOV_A                     3050
#define IDC_RADIO_BOV_B                     3051
#define IDC_RADIO_BOV_C                     3052
#define IDC_RADIO_BOV_D                     3053
#define IDC_RADIO_BOV_E                     3054
#define IDC_RADIO_BOV_F                     3055
#define IDC_CHECK_BROWN_OUT_DETECT          3056
#define IDC_CHECK_BROWN_OUT_RESET           3057
#define IDC_CHECK_BROWN_OUT_ENABLE          3058
#define IDC_CHECK_BROWN_OUT_IAP             3059

// GPF, GPG Multi-function Options
#define IDC_RADIO_GPF_GPIO                  3060
#define IDC_RADIO_GPF_CRYSTAL               3061
#define IDC_RADIO_GPG2_GPIO                 3062
#define IDC_RADIO_GPG2_32K                  3063

// Initial State of GPA8 Options
#define IDC_RADIO_PA8_IOINI                 3064
#define IDC_RADIO_PA8_LOW                   3065

// I/O Initial State Options
#define IDC_RADIO_IO_TRI                    3066
#define IDC_RADIO_IO_BI                     3067
#define IDC_RADIO_IO_PO                     3068

// Config Value
#define IDC_STATIC_CONFIG_0                 3070
#define IDC_STATIC_CONFIG_VALUE_0           3071
#define IDC_STATIC_CONFIG_1                 3072
#define IDC_STATIC_CONFIG_VALUE_1           3073
#define IDC_STATIC_CONFIG_2                 3074
#define IDC_STATIC_CONFIG_VALUE_2           3075
#define IDC_STATIC_CONFIG_3                 3076
#define IDC_STATIC_CONFIG_VALUE_3           3077
#define IDC_STATIC_CONFIG_4                 3078
#define IDC_STATIC_CONFIG_VALUE_4           3079

// Data Flash Options
#define IDC_CHECK_DATA_FLASH_ENABLE         3080
#define IDC_STATIC_FLASH_BASE_ADDRESS       3081
#define IDC_EDIT_FLASH_BASE_ADDRESS         3082
#define IDC_EDIT_DATA_FLASH_SIZE            3083
#define IDC_SPIN_DATA_FLASH_SIZE            3084
#define IDC_STATIC_DATA_FLASH_SIZE          3085
#define IDC_CHECK_DATA_FLASH_VAR_SIZE_ENABLE    3086

#define IDC_STATIC_FLASH_ADVANCE_LOCK       3088
#define IDC_EDIT_FLASH_ADVANCE_LOCK         3089

// others
#define IDC_CHECK_SECURITY_LOCK             3090
#define IDC_CHECK_ICE_LOCK                  3091
#define IDC_CHECK_WDT_ENABLE                3092
#define IDC_CHECK_WDT_POWER_DOWN            3093
#define IDC_SPROM_LOCK_CACHEABLE            3094
#define IDC_CHECK_SECURITYBOOT_LOCK         3095
#define IDC_CHECK_CLOCK_STOP_DETECT         3096
#define IDC_CHECK_MASS_ERASE                3097
#define IDC_CHECK_LDROM_EN                  3098 // IDD_DIALOG_CONFIGURATION_AU9100
#define IDC_PWM_DEBUG_ENABLE                3099 // IDD_DIALOG_CONFIGURATION_NM1120

// HXT Gain Options
#define IDC_RADIO_HXT_H16                   3100
#define IDC_RADIO_HXT_12_16                 3101
#define IDC_RADIO_HXT_8_12                  3102
#define IDC_RADIO_HXT_L8                    3103

// RC Oscillator Trim Option
#define IDC_RADIO_RC_44M                    3110
#define IDC_RADIO_RC_48M                    3111
#define IDC_RADIO_RC_22M                    3112
#define IDC_RADIO_RC_24M                    3113

// EMAC Interface Options
#define IDC_RADIO_MII_MODE                  3114
#define IDC_RADIO_RMII_MODE                 3115

// SPIM CLK/SS/MISO/MOSI Multi-function Pin Select
#define IDC_RADIO_SPIM_SEL0                 3116
#define IDC_RADIO_SPIM_SEL1                 3117
#define IDC_RADIO_SPIM_SEL2                 3118
#define IDC_RADIO_SPIM_SEL3                 3119

// 8051 1T
#define IDC_GROUP_FSYS                      3120
#define IDC_RADIO_FSYS_HIRC                 3121
#define IDC_RADIO_FSYS_LIRC                 3122
#define IDC_GROUP_RPD                       3123
#define IDC_RADIO_RPD_RESET                 3124
#define IDC_RADIO_RPD_INPUT                 3125
#define IDC_GROUP_OCDPWM                    3126
#define IDC_RADIO_OCDPWM_TRI                3127
#define IDC_RADIO_OCDPWM_CONTI              3128
#define IDC_GROUP_LDSIZE                    3129
#define IDC_RADIO_LDSIZE_0K                 3130
#define IDC_RADIO_LDSIZE_1K                 3131
#define IDC_RADIO_LDSIZE_2K                 3132
#define IDC_RADIO_LDSIZE_3K                 3133
#define IDC_RADIO_LDSIZE_4K                 3134
#define IDC_GROUP_WDT                       3135
#define IDC_RADIO_WDT_DISABLE               3136
#define IDC_RADIO_WDT_ENABLE_STOP           3137
#define IDC_RADIO_WDT_ENABLE_KEEP           3138
#define IDC_CHECK_OCD_ENABLE                3139

// NM1120
#define IDC_COMBO_GPA0_RINI                 3140
#define IDC_COMBO_GPA1_RINI                 3141
#define IDC_COMBO_GPA2_RINI                 3142
#define IDC_COMBO_GPA3_RINI                 3143
#define IDC_COMBO_GPA4_RINI                 3144
#define IDC_COMBO_GPA5_RINI                 3145
#define IDC_STATIC_GPA0                     3150
#define IDC_STATIC_GPA1                     3151
#define IDC_STATIC_GPA2                     3152
#define IDC_STATIC_GPA3                     3153
#define IDC_STATIC_GPA4                     3154
#define IDC_STATIC_GPA5                     3155

// UART1 TXD/RXD Multi-function Pin Select
#define IDC_RADIO_UART1_SEL0                3160
#define IDC_RADIO_UART1_SEL1                3161
#define IDC_RADIO_UART1_SEL2                3162
#define IDC_RADIO_UART1_SEL3                3163
#define IDC_RADIO_UART1_SEL4                3164

// MT500
#define IDC_CHECK_CHZ_BPWM_Ctrl             3165
#define IDC_CHECK_CHZ_Even0_Ctrl            3166
#define IDC_CHECK_CHZ_Odd0_Ctrl             3167
#define IDC_CHECK_CHZ_Even1_Ctrl            3168
#define IDC_CHECK_CHZ_Odd1_Ctrl             3169

// M031
#define IDC_RADIO_LVR_LEVEL_0               3170
#define IDC_RADIO_LVR_LEVEL_1               3171
#define IDC_RADIO_HXT_MODE_0                3172
#define IDC_RADIO_HXT_MODE_1                3173
#define IDC_RADIO_CHIPRESET_TIMEEXT_0       3174
#define IDC_RADIO_CHIPRESET_TIMEEXT_1       3175
#define IDC_RADIO_RST_PIN_WIDTH_0           3176
#define IDC_RADIO_RST_PIN_WIDTH_1           3177

// M2354
#define IDC_CHECK_TAMPER_POWER_DOWN         3180

//M258
#define IDC_RADIO_BOOT_CLOCK_SELECT_0       3190
#define IDC_RADIO_BOOT_CLOCK_SELECT_1       3191
#define IDC_GROUP_BOOT_CLOCK_SELECT         3192

// I96000
#define IDC_GROUP_LOAD_FAIL_OPTION          4000
#define IDC_GROUP_STRAP_SELECT              4001
#define IDC_RADIO_LOAD_FAIL_DISABLE         4002
#define IDC_RADIO_LOAD_FAIL_ENABLE          4003
#define IDC_RADIO_STRAP_PIN                 4004
#define IDC_RADIO_STRAP_FUSE                4005
#define IDC_CHECK_JTAG_BOOTFAIL             4006
#define IDC_CHECK_CLEAR_RSTSTS              4007
#define IDC_CHECK_JTAG_BOOTSUCCESS          4008
#define IDC_CHECK_PIN_WAKEUP_0              4009
#define IDC_CHECK_PIN_WAKEUP_1              4010
#define IDC_CHECK_PIN_WAKEUP_2              4011
#define IDC_CHECK_PIN_WAKEUP_3              4012
#define IDC_COMBO_PIN_WAKEUP_0              4013
#define IDC_COMBO_PIN_WAKEUP_1              4014
#define IDC_COMBO_PIN_WAKEUP_2              4015
#define IDC_COMBO_PIN_WAKEUP_3              4016
#define IDC_COMBO_FUSE_RETRY                4017
