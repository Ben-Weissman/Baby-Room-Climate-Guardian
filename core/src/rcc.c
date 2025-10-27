#include "rcc.h"
#include "board.h"
#include "status.h"

static const uint16_t ahb_prescale_table[16] = {
    1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 8, 16, 64, 128, 256, 512};
static const uint8_t pllp_div_table[4]     = {2, 4, 6, 8};
static const uint8_t apb_prescale_table[8] = {1, 1, 1, 1, 2, 4, 8, 16};

Status_t get_sys_clk(uint32_t* sys_clk) {
    uint32_t sws = (RCC->CFGR & RCC_CFGR_SWS_Msk) >> RCC_CFGR_SWS_Pos;
    if (sws == RCC_CFGR_SWS_HSI) {
        *sys_clk = HSI_VALUE;
    } else if (sws == RCC_CFGR_SWS_HSE) {
        *sys_clk = HSE_VALUE;
    } else if (sws == RCC_CFGR_SWS_PLL) {
        uint32_t pll_src_sel = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_Msk) >> RCC_PLLCFGR_PLLSRC_Pos;
        uint32_t pll_src_freq;
        if (pll_src_sel == RCC_PLLCFGR_PLLSRC_HSI) {
            pll_src_freq = HSI_VALUE;
        } else {
            pll_src_freq = HSE_VALUE;
        }

        // get M,N,Q,P factors, in this case we dont really need the Q factor
        // because we just want the system clock and not the 48Mhz/USB domain.
        uint32_t pll_m = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos;
        uint32_t pll_n = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos;
        uint32_t pll_p =
            pllp_div_table[(RCC->PLLCFGR & RCC_PLLCFGR_PLLP_Msk) >> RCC_PLLCFGR_PLLP_Pos];

        // calculate the VCO_output to get system clock
        uint32_t vco_input  = pll_src_freq / pll_m;
        uint32_t vco_output = vco_input * pll_n;
        *sys_clk            = vco_output / pll_p;
    } else {
        // Invalid clock input
        return STATUS_INVALID;
    }
    return STATUS_OK;
}

Status_t RCC_get_periph_clk(void* periph, uint32_t* periph_clk) {
    uint32_t sys_clk;

    if (get_sys_clk(&sys_clk) == STATUS_INVALID) {
        return STATUS_INVALID;
    }

    uint32_t ahb_prescaler =
        ahb_prescale_table[(RCC->CFGR & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos];

    // SYSCLK goes through the AHB prescaler → becomes HCLK (AHB bus, CPU)
    uint32_t hclk = sys_clk / ahb_prescaler;

    uint32_t apb1_prescaler =
        apb_prescale_table[(RCC->CFGR & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos];
    uint32_t apb2_prescaler =
        apb_prescale_table[(RCC->CFGR & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos];

    if (periph == USART2) {
        *periph_clk = hclk / apb1_prescaler;
    } else if (periph == USART6 || periph == USART1) {
        *periph_clk = hclk / apb2_prescaler;
    } else if (periph == I2C1 || periph == I2C2 || periph == I2C3) {
        *periph_clk = hclk / apb1_prescaler;
    } else {
        return STATUS_INVALID;
    }
    return STATUS_OK;
}
