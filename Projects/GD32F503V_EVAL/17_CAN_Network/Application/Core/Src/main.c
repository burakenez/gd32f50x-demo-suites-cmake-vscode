/*!
    \file    main.c
    \brief   CAN network communication demo

    \version 2025-11-10, V1.0.1, demo for GD32F50x
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f50x.h"
#include <stdio.h>
#include "gd32f503v_eval.h"

#define DEV_CAN0_ID          0xaabbU
#define DEV_CAN0_MASK        0x0000U

can_transmit_message_struct g_transmit_message;
can_receive_message_struct g_receive_message;
FlagStatus receive_flag;

void nvic_config(void);
void led_config(void);
void gpio_config(void);
void can_networking_init(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    uint8_t i = 0;
    receive_flag = RESET;
    /* configure Tamper key */
    gd_eval_key_init(KEY_TAMPER, KEY_MODE_GPIO);
    /* configure GPIO */
    gpio_config();
    /* configure USART */
    gd_eval_com_init(EVAL_COM0);
    /* configure NVIC */
    nvic_config();
    /* configure leds */
    led_config();
    /* set led off */
    gd_eval_led_off(LED1);

    /* initialize CAN and CAN filter*/
    can_networking_init();

    /* enable CAN receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INT_RFNE0);

    /* initialize transmit message */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &g_transmit_message);
    /* initialize transmit message */
    g_transmit_message.tx_sfid = 0x00U;
    g_transmit_message.tx_efid = 0x00U;
    g_transmit_message.tx_ft = CAN_FT_DATA;
    g_transmit_message.tx_ff = CAN_FF_EXTENDED;
    g_transmit_message.tx_dlen = 8U;
    /* initialize receive message */
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &g_receive_message);
    printf("\r\nplease press the Tamper key to transmit data!\r\n");

    while(1) {
        /* waiting for the Tamper key pressed */
        while(0 == gd_eval_key_state_get(KEY_TAMPER)) {
            g_transmit_message.tx_efid = DEV_CAN0_ID;
            g_transmit_message.tx_data[0] = 0xA0;
            g_transmit_message.tx_data[1] = 0xA1;
            g_transmit_message.tx_data[2] = 0xA2;
            g_transmit_message.tx_data[3] = 0xA3;
            g_transmit_message.tx_data[4] = 0xA4;
            g_transmit_message.tx_data[5] = 0xA5;
            g_transmit_message.tx_data[6] = 0xA6;
            g_transmit_message.tx_data[7] = 0xA7;
            printf("\r\n can0 transmit data:");
            for(i = 0; i < g_transmit_message.tx_dlen; i++) {
                printf(" %02x", g_transmit_message.tx_data[i]);
            }
            /* transmit message */
            can_message_transmit(CAN0, &g_transmit_message);
            /* waiting for Tamper key up */
            while(0 == gd_eval_key_state_get(KEY_TAMPER));
        }
        if(SET == receive_flag) {
            gd_eval_led_toggle(LED1);
            receive_flag = RESET;
            printf("\r\n can0 receive data:");
            for(i = 0; i < g_receive_message.rx_dlen; i++) {
                printf(" %02x", g_receive_message.rx_data[i]);
            }

        }
    }
}

/*!
    \brief      initialize CAN and filter
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_networking_init(void)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    /* initialize CAN register */
    can_deinit(CAN0);

    /* initialize CAN */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_10TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_3TQ;
    /* baudrate 1Mbps */
    can_parameter.prescaler = 18U;
    can_init(CAN0, &can_parameter);

    /* initialize filter */
    /* CAN0 filter number */
    can_filter.filter_number = 0U;

    /* initialize filter */
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    /* configure filter ID, only the 0xaabb expand ID can be received */
    can_filter.filter_list_high = (uint16_t)(DEV_CAN0_ID >> 13U);
    can_filter.filter_list_low = ((uint16_t)(DEV_CAN0_ID << 3U)) | (1U << 2U);
    /* configure filter mask */
    can_filter.filter_mask_high = 0x0000U;
    can_filter.filter_mask_low = 0x0000U;
    /* select receiver fifo */
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);
}

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{
    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);
}

/*!
    \brief      configure the led
    \param[in]  none
    \param[out] none
    \retval     none
*/
void led_config(void)
{
    gd_eval_led_init(LED1);
}

/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOB);

    /* configure CAN0 GPIO */
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, GPIO_PIN_8);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_8);

    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, GPIO_PIN_9);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_9);
}
