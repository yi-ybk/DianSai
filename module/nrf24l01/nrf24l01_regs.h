/**
 * @file    nrf24l01_regs.h
 * @brief   nRF24L01+ SPI command, register and bit definitions
 */
#pragma once

#include <stdint.h>

#define NRF24_MAX_PAYLOAD_SIZE       32U
#define NRF24_MIN_ADDRESS_WIDTH      3U
#define NRF24_MAX_ADDRESS_WIDTH      5U
#define NRF24_MAX_RF_CHANNEL         125U
#define NRF24_REGISTER_MASK          0x1FU

/* SPI commands */
#define NRF24_CMD_R_REGISTER         0x00U
#define NRF24_CMD_W_REGISTER         0x20U
#define NRF24_CMD_R_RX_PAYLOAD       0x61U
#define NRF24_CMD_W_TX_PAYLOAD       0xA0U
#define NRF24_CMD_FLUSH_TX           0xE1U
#define NRF24_CMD_FLUSH_RX           0xE2U
#define NRF24_CMD_REUSE_TX_PL        0xE3U
#define NRF24_CMD_ACTIVATE           0x50U
#define NRF24_CMD_R_RX_PL_WID        0x60U
#define NRF24_CMD_W_ACK_PAYLOAD      0xA8U
#define NRF24_CMD_W_TX_PAYLOAD_NOACK 0xB0U
#define NRF24_CMD_NOP                0xFFU
#define NRF24_ACTIVATE_DATA          0x73U

/* Register addresses */
#define NRF24_REG_CONFIG             0x00U
#define NRF24_REG_EN_AA              0x01U
#define NRF24_REG_EN_RXADDR          0x02U
#define NRF24_REG_SETUP_AW           0x03U
#define NRF24_REG_SETUP_RETR         0x04U
#define NRF24_REG_RF_CH              0x05U
#define NRF24_REG_RF_SETUP           0x06U
#define NRF24_REG_STATUS             0x07U
#define NRF24_REG_OBSERVE_TX         0x08U
#define NRF24_REG_RPD                0x09U
#define NRF24_REG_RX_ADDR_P0         0x0AU
#define NRF24_REG_RX_ADDR_P1         0x0BU
#define NRF24_REG_RX_ADDR_P2         0x0CU
#define NRF24_REG_RX_ADDR_P3         0x0DU
#define NRF24_REG_RX_ADDR_P4         0x0EU
#define NRF24_REG_RX_ADDR_P5         0x0FU
#define NRF24_REG_TX_ADDR            0x10U
#define NRF24_REG_RX_PW_P0           0x11U
#define NRF24_REG_RX_PW_P1           0x12U
#define NRF24_REG_RX_PW_P2           0x13U
#define NRF24_REG_RX_PW_P3           0x14U
#define NRF24_REG_RX_PW_P4           0x15U
#define NRF24_REG_RX_PW_P5           0x16U
#define NRF24_REG_FIFO_STATUS        0x17U
#define NRF24_REG_DYNPD              0x1CU
#define NRF24_REG_FEATURE            0x1DU

/* CONFIG */
#define NRF24_CONFIG_MASK_RX_DR      (1U << 6)
#define NRF24_CONFIG_MASK_TX_DS      (1U << 5)
#define NRF24_CONFIG_MASK_MAX_RT     (1U << 4)
#define NRF24_CONFIG_EN_CRC          (1U << 3)
#define NRF24_CONFIG_CRCO            (1U << 2)
#define NRF24_CONFIG_PWR_UP          (1U << 1)
#define NRF24_CONFIG_PRIM_RX         (1U << 0)

/* EN_AA and EN_RXADDR */
#define NRF24_PIPE_0_MASK            (1U << 0)
#define NRF24_PIPE_1_MASK            (1U << 1)
#define NRF24_PIPE_2_MASK            (1U << 2)
#define NRF24_PIPE_3_MASK            (1U << 3)
#define NRF24_PIPE_4_MASK            (1U << 4)
#define NRF24_PIPE_5_MASK            (1U << 5)
#define NRF24_ALL_PIPES_MASK         0x3FU

/* SETUP_AW */
#define NRF24_SETUP_AW_3_BYTES       0x01U
#define NRF24_SETUP_AW_4_BYTES       0x02U
#define NRF24_SETUP_AW_5_BYTES       0x03U

/* SETUP_RETR */
#define NRF24_SETUP_RETR_ARD_SHIFT   4U
#define NRF24_SETUP_RETR_ARC_MASK    0x0FU

/* RF_SETUP */
#define NRF24_RF_SETUP_CONT_WAVE     (1U << 7)
#define NRF24_RF_SETUP_RF_DR_LOW     (1U << 5)
#define NRF24_RF_SETUP_PLL_LOCK      (1U << 4)
#define NRF24_RF_SETUP_RF_DR_HIGH    (1U << 3)
#define NRF24_RF_SETUP_RF_PWR_MASK   (3U << 1)

/* STATUS */
#define NRF24_STATUS_RX_DR           (1U << 6)
#define NRF24_STATUS_TX_DS           (1U << 5)
#define NRF24_STATUS_MAX_RT          (1U << 4)
#define NRF24_STATUS_RX_P_NO_MASK    (7U << 1)
#define NRF24_STATUS_RX_P_NO_SHIFT   1U
#define NRF24_STATUS_TX_FULL         (1U << 0)
#define NRF24_STATUS_IRQ_MASK        (NRF24_STATUS_RX_DR | \
                                      NRF24_STATUS_TX_DS | \
                                      NRF24_STATUS_MAX_RT)

/* FIFO_STATUS */
#define NRF24_FIFO_STATUS_TX_REUSE   (1U << 6)
#define NRF24_FIFO_STATUS_TX_FULL    (1U << 5)
#define NRF24_FIFO_STATUS_TX_EMPTY   (1U << 4)
#define NRF24_FIFO_STATUS_RX_FULL    (1U << 1)
#define NRF24_FIFO_STATUS_RX_EMPTY   (1U << 0)

/* FEATURE */
#define NRF24_FEATURE_EN_DPL         (1U << 2)
#define NRF24_FEATURE_EN_ACK_PAY     (1U << 1)
#define NRF24_FEATURE_EN_DYN_ACK     (1U << 0)

