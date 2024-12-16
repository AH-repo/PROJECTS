#pragma once
extern "C" {
#include "stm32f4xx.h"
#include "stm32f446xx.h"
#include <stdio.h>
}

struct uart_config
{
    USART_TypeDef *UART;
    DMA_TypeDef *dma;
    uint8_t dma_stream_tx;
    uint8_t dma_stream_rx;
    uint8_t dma_channel_tx;
    uint8_t dma_channel_rx;
    uint8_t BR;
    GPIO_TypeDef *RX_Port;
    uint16_t RX_Pin;
    GPIO_TypeDef *TX_Port;
    uint16_t TX_Pin;

};
class UART_DMA {
public: 
    UART_DMA(const uart_config& u_conf);
    void init();
    void transmit(uint8_t *data, uint16_t size);
    void receive();
    void send(uint8_t data);
    uint8_t read();
    bool isTXbusy();
    bool isRXbusy();
    void clearTXbusy();
    void clearRXbusy();

private:
    USART_TypeDef *m_UART;
    DMA_TypeDef *m_dma;
    uint8_t m_dma_stream_tx;
    uint8_t m_dma_stream_rx;
    uint8_t m_dma_channel_tx;
    uint8_t m_dma_channel_rx;
    uint32_t m_BR;
    GPIO_TypeDef *m_RX_Port;
    uint16_t m_RX_Pin;
    GPIO_TypeDef *m_TX_Port;
    uint16_t m_TX_Pin;
    uint32_t m_fpclk;
    volatile bool m_TX_busy;
    volatile bool m_RX_busy;
};