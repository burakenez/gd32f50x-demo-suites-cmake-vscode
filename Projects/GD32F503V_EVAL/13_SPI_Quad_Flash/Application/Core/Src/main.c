/*!
    \file    main.c
    \brief   SPI flash demo

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
#include "systick.h"
#include <stdio.h>
#include "gd32f503v_eval.h"
#include "gd25qxx.h"

#define BUFFER_SIZE              256
#define TX_BUFFER_SIZE           (countof(tx_buffer) - 1)
#define RX_BUFFER_SIZE           0xFF

#define countof(a)               (sizeof(a) / sizeof(*(a)))

#define SFLASH_ID                0xC84015
#define FLASH_WRITE_ADDRESS      0x000000
#define FLASH_READ_ADDRESS       FLASH_WRITE_ADDRESS

uint8_t led_count;
uint8_t tx_buffer[BUFFER_SIZE];
uint8_t rx_buffer[BUFFER_SIZE];
uint32_t flash_id = 0;
uint16_t i = 0;
uint8_t  is_successful = 0;

void turn_on_led(uint8_t led_num);
ErrStatus memory_compare(uint8_t *src, uint8_t *dst, uint16_t length);
void test_status_led_init(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* systick configuration*/
    systick_config();

    /* configure the led GPIO */
    test_status_led_init();

    /* USART parameter configuration */
    gd_eval_com_init(EVAL_COM0);

    /* configure SPI GPIO and parameter */
    spi_flash_init();

    /* printf CPU unique device id */
    printf("\n\rSPI Flash:GD25Q16 configured...\n\r");

    /* get flash id */
    flash_id = spi_flash_read_id();
    printf("\n\rThe Flash_ID:0x%X\n\r\n\r", flash_id);

    /* flash id is correct */
    if(SFLASH_ID == flash_id) {
        printf("\n\rWrite to tx_buffer:\n\r\n\r");

        /* printf tx_buffer value */
        for(i = 0; i < BUFFER_SIZE; i++) {
            tx_buffer[i] = i;
            printf("0x%02X ", tx_buffer[i]);

            if(15 == i % 16) {
                printf("\n\r");
            }
        }

        /* erase the specified flash sector */
        spi_flash_sector_erase(FLASH_WRITE_ADDRESS);

        /* write tx_buffer data to the flash */
        spi_quad_flash_buffer_write(tx_buffer, FLASH_WRITE_ADDRESS, BUFFER_SIZE);
        delay_ms(10);
        printf("\n\r\n\rRead from rx_buffer:\n\r\n\r");

        /* read a block of data from the flash to rx_buffer */
        spi_quad_flash_buffer_read(rx_buffer, FLASH_READ_ADDRESS, BUFFER_SIZE);

        /* printf rx_buffer value */
        for(i = 0; i < BUFFER_SIZE; i ++) {
            printf("0x%02X ", rx_buffer[i]);
            if(15 == i % 16) {
                printf("\n\r");
            }
        }

        if(ERROR == memory_compare(tx_buffer, rx_buffer, BUFFER_SIZE)) {
            printf("\n\rErr:Data Read and Write aren't Matching.\n\r");
            is_successful = 1;
        }

        /* spi qspi flash test passed */
        if(0 == is_successful) {
            printf("\n\rSPI-GD25Q16 Test Passed!\n\r");
        }
    } else {
        /* spi flash read id fail */
        printf("\n\rSPI Flash: Read ID Fail!\n\r");
    }

    while(1) {
        /* turn off all leds */
        gd_eval_led_off(LED1);
        gd_eval_led_off(LED2);
        gd_eval_led_off(LED3);

        /* turn on a led */
        turn_on_led(led_count % 3);
        led_count ++;
        if(4 <= led_count) {
            led_count = 0;
        }

        delay_ms(500);
    }
}

/*!
    \brief      test status led initialize
    \param[in]  none
    \param[out] none
    \retval     none
*/
void test_status_led_init(void)
{
    /* initialize the leds */
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);
    gd_eval_led_init(LED3);

    /* close all of leds */
    gd_eval_led_off(LED1);
    gd_eval_led_off(LED2);
    gd_eval_led_off(LED3);
}

/*!
    \brief      turn on led
    \param[in]  led_num: led number
    \param[out] none
    \retval     none
*/
void turn_on_led(uint8_t led_num)
{
    switch(led_num) {
    case 0:
        /* turn on LED1 */
        gd_eval_led_on(LED1);
        break;
    case 1:
        /* turn on LED2 */
        gd_eval_led_on(LED2);
        break;
    case 2:
        /* turn on LED3 */
        gd_eval_led_on(LED3);
        break;
    default:
        /* turn on all leds */
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);
        gd_eval_led_on(LED3);
        break;
    }
}

/*!
    \brief      memory compare function
    \param[in]  src: source data pointer
    \param[in]  dst: destination data pointer
    \param[in]  length: the compare data length
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus memory_compare(uint8_t *src, uint8_t *dst, uint16_t length)
{
    while(length --) {
        if(*src++ != *dst++) {
            return ERROR;
        }
    }
    return SUCCESS;
}
