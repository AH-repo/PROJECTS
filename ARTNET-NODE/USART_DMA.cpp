#pragma once
#include "UART_DMA.hpp"

UART_DMA::UART_DMA(const uart_config& u_conf)
    : m_UART(u_conf.UART), 
      m_dma(u_conf.dma),
      m_dma_stream_tx(u_conf.dma_stream_tx),
      m_dma_stream_rx(u_conf.dma_stream_rx),
      m_dma_channel_tx(u_conf.dma_channel_tx),
      m_dma_channel_rx(u_conf.dma_channel_rx),
      m_BR(u_conf.BR),
      m_RX_Port(u_conf.RX_Port),
      m_RX_Pin(u_conf.RX_Pin),
      m_TX_Port(u_conf.TX_Port),
      m_TX_Pin(u_conf.TX_Pin) {
            // UART1: PA9 (TX), PA10 (RX) lub PB6 (TX), PB7 (RX)
     if (u_conf.UART == USART1) {
        if ((((u_conf.TX_Port == GPIOA && u_conf.TX_Pin == 9) || (u_conf.TX_Port == GPIOB && u_conf.TX_Pin == 6)) &&
            ((u_conf.RX_Port == GPIOA && u_conf.RX_Pin == 10) ||( u_conf.RX_Port == GPIOB && u_conf.RX_Pin == 7)))){
        m_RX_Pin = u_conf.RX_Pin;
        m_RX_Port = u_conf.RX_Port;
        m_TX_Pin = u_conf.TX_Pin;
        m_RX_Port = u_conf.TX_Port;
        u_conf.UART == m_UART;
        }   
        }
        else{
            printf("Wrong UART RX/TX");
            __disable_irq();
            while(1);
        }
    }

     
     // UART2: PA2 (TX), PA3 (RX) lub PD5 (TX), PD6 (RX)
     else if (u_conf.UART == USART2) {
        if (!((u_conf.TX_Port == GPIOA && u_conf.TX_Pin == 2) || (u_conf.TX_Port == GPIOD && u_conf.TX_Pin == 5)) ||
            ((u_conf.RX_Port == GPIOA && u_conf.RX_Pin == 3) || (u_conf.RX_Port == GPIOD && u_conf.RX_Pin == 6))){
        m_RX_Pin = u_conf.RX_Pin;
        m_RX_Port = u_conf.RX_Port;
        m_TX_Pin = u_conf.TX_Pin;
        m_RX_Port = u_conf.TX_Port;
        u_conf.UART == m_UART;
        }
        else{
        printf("Wrong UART RX/TX");
        __disable_irq();
        while(1);
        }
        }
    
    // UART3: PB10 (TX), PB11 (RX) lub PC10 (TX), PC11 (RX) lub PD8 (TX), PD9 (RX)
    else if (u_conf.UART == USART3) {
        if (!((u_conf.TX_Port == GPIOB && u_conf.TX_Pin == 10) || (u_conf.TX_Port == GPIOC && u_conf.TX_Pin == 10) || (u_conf.TX_Port == GPIOD && u_conf.TX_Pin == 8 )) &&
              ((u_conf.RX_Port == GPIOB && u_conf.RX_Pin == 11) || (u_conf.RX_Port == GPIOC && u_conf.RX_Pin == 11) || (u_conf.RX_Port == GPIOD && u_conf.RX_Pin == 9)))

        m_RX_Pin = u_conf.RX_Pin;
        m_RX_Port = u_conf.RX_Port;
        m_TX_Pin = u_conf.TX_Pin;
        m_RX_Port = u_conf.TX_Port;
        u_conf.UART == m_UART;
        }
        else{
            printf("Wrong UART RX/TX");
            __disable_irq();
            while(1);
        }
    
    // UART4: PA0 (TX), PA1 (RX) lub PC10 (TX), PC11 (RX)
    else if (u_conf.UART == UART4) {
        if (!((u_conf.TX_Port == GPIOA && u_conf.TX_Pin == 0) || (u_conf.TX_Port == GPIOC && u_conf.TX_Pin == 10 )) &&
              ((u_conf.RX_Port == GPIOA && u_conf.RX_Pin == 1) || (u_conf.RX_Port == GPIOC && u_conf.RX_Pin == 11))) {
        m_RX_Pin = u_conf.RX_Pin;
        m_RX_Port = u_conf.RX_Port;
        m_TX_Pin = u_conf.TX_Pin;
        m_RX_Port = u_conf.TX_Port;
        u_conf.UART == m_UART;
        }
        else{
            printf("Wrong UART RX/TX");
            __disable_irq();
            while(1);
        }
    }
    // UART5: PC12 (TX), PD2 (RX)
    else if (u_conf.UART == UART5) {
        if (!(u_conf.TX_Port == GPIOC && u_conf.TX_Pin == 12 && u_conf.RX_Port == GPIOD && u_conf.RX_Pin == 2)){            
        m_RX_Pin = u_conf.RX_Pin;
        m_RX_Port = u_conf.RX_Port;
        m_TX_Pin = u_conf.TX_Pin;
        m_RX_Port = u_conf.TX_Port;
        u_conf.UART == m_UART;
        }
        else{
            printf("Wrong UART RX/TX");
            __disable_irq();
            while(1);
        }
    
    // UART6: PC6 (TX), PC7 (RX)
    else if (u_conf.UART == USART6) {
        if (!(u_conf.TX_Port == GPIOC && u_conf.TX_Pin == 6 && u_conf.RX_Port == GPIOC && u_conf.RX_Pin == 7)){

        m_RX_Pin = u_conf.RX_Pin;
        m_RX_Port = u_conf.RX_Port;
        m_TX_Pin = u_conf.TX_Pin;
        m_RX_Port = u_conf.TX_Port;
        u_conf.UART == m_UART;
        }
        else{
            printf("Wrong UART RX/TX");
            __disable_irq();
            while(1);
        }
    }
    
        //1
        if(u_conf.UART = USART1){
        if( u_conf.dma = DMA2 && u_conf.dma_channel_tx == 4 && u_conf.dma_stream_tx == 7 &&
         ((u_conf.dma_channel_rx == 4 && u_conf.dma_stream_rx == 2) || (u_conf.dma_channel_rx == 4 && u_conf.dma_stream_rx == 5))){
        m_dma_stream_tx = u_conf.dma_stream_tx;
        m_dma_stream_rx = u_conf.dma_stream_rx;
        m_dma_channel_tx = u_conf.dma_channel_tx;
        m_dma_channel_rx = u_conf.dma_channel_rx;
         }
        else{
            printf("Wrong stream/channel");
            __disable_irq();
            while(1);
        }
        }
         //2
        if(u_conf.UART = USART2){
        if(u_conf.dma = DMA1 && u_conf.dma_channel_tx == 4 && u_conf.dma_stream_tx == 6 &&
        u_conf.dma_channel_rx == 4 && u_conf.dma_stream_rx == 5) {
        m_dma_stream_tx = u_conf.dma_stream_tx;
        m_dma_stream_rx = u_conf.dma_stream_rx;
        m_dma_channel_tx = u_conf.dma_channel_tx;
        m_dma_channel_rx = u_conf.dma_channel_rx;
        }
        else{
            printf("Wrong stream/channel");
            __disable_irq();
            while(1);
        }
        }
        //3
        if(u_conf.UART = USART3){
        if(u_conf.dma = DMA1 &&( u_conf.dma_channel_tx == 4 && u_conf.dma_stream_tx == 3 || u_conf.dma_channel_tx == 7 && u_conf.dma_stream_tx == 4 )&&
        u_conf.dma_channel_rx == 4 && u_conf.dma_stream_rx == 1) {
        m_dma_stream_tx = u_conf.dma_stream_tx;
        m_dma_stream_rx = u_conf.dma_stream_rx;
        m_dma_channel_tx = u_conf.dma_channel_tx;
        m_dma_channel_rx = u_conf.dma_channel_rx;
        }
        else{
            printf("Wrong stream/channel");
            __disable_irq();
            while(1);
        }
        }
        //4
        if(u_conf.UART = UART4){
        if(u_conf.dma = DMA1 && u_conf.UART = UART4 && u_conf.dma_channel_tx == 4 && u_conf.dma_stream_tx == 4 &&
        u_conf.dma_channel_rx == 4 && u_conf.dma_stream_rx == 2){
        m_dma_stream_tx = u_conf.dma_stream_tx;
        m_dma_stream_rx = u_conf.dma_stream_rx;
        m_dma_channel_tx = u_conf.dma_channel_tx;
        m_dma_channel_rx = u_conf.dma_channel_rx;
        }
        else{
            printf("Wrong stream/channel");
            __disable_irq();
            while(1);
        }
        }
        //5
        if(u_conf.UART = UART5){
        if(u_conf.dma = DMA1 && u_conf.dma_channel_tx == 4 && u_conf.dma_stream_tx == 7 &&
        u_conf.dma_channel_rx == 4 && u_conf.dma_stream_rx == 0) {
        m_dma_stream_tx = u_conf.dma_stream_tx;
        m_dma_stream_rx = u_conf.dma_stream_rx;
        m_dma_channel_tx = u_conf.dma_channel_tx;
        m_dma_channel_rx = u_conf.dma_channel_rx;
        }
        else{
            printf("Wrong stream/channel");
            __disable_irq();
            while(1);
        }
        }
        //6    
        if(u_conf.UART = USART6 ){             
        if(u_conf.dma = DMA2 && ((u_conf.dma_channel_tx == 5 && u_conf.dma_stream_tx == 6) || (u_conf.dma_channel_tx == 5 && u_conf.dma_stream_tx == 7)) &&
        ((u_conf.dma_channel_rx == 5 && u_conf.dma_stream_rx == 1) || (u_conf.dma_channel_rx == 5 && u_conf.dma_stream_rx == 2))) {
        m_dma_stream_tx = u_conf.dma_stream_tx;
        m_dma_stream_rx = u_conf.dma_stream_rx;
        m_dma_channel_tx = u_conf.dma_channel_tx;
        m_dma_channel_rx = u_conf.dma_channel_rx;
        }
        else{
            printf("Wrong stream/channel");
            __disable_irq();
            while(1);
        }
        }
        

    }

void UART_DMA::init() {

    // Włącz zegar dla odpowiedniego UART
    if (m_dma == DMA1) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    } else if (m_dma == DMA2) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    }
    //tx dla dma
    DMA_Stream_TypeDef* txStream = reinterpret_cast<DMA_Stream_TypeDef*>(m_dma_stream_tx);
    txStream->CR = 0; 
    txStream->PAR = reinterpret_cast<uint32_t>(&m_UART->DR); 
    txStream->CR |= (m_dma_channel_tx << DMA_SxCR_CHSEL_Pos); 
    txStream->CR |= DMA_SxCR_MINC; 
    txStream->CR |= DMA_SxCR_DIR_0; 
    txStream->CR |= DMA_SxCR_PL_1;

    //rx dla dma
    DMA_Stream_TypeDef* rxStream = reinterpret_cast<DMA_Stream_TypeDef*>(m_dma_stream_rx);
    rxStream->CR = 0; 
    rxStream->PAR = reinterpret_cast<uint32_t>(&m_UART->DR); 
    rxStream->CR |= (m_dma_channel_rx << DMA_SxCR_CHSEL_Pos);  
    rxStream->CR |= DMA_SxCR_MINC;  
    rxStream->CR |= DMA_SxCR_PL_1;  
    //KONIEC INITA DMA


    //Odpalenie clocka dla gpio
    if (m_RX_Port == GPIOA || m_TX_Port == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if (m_RX_Port == GPIOB || m_TX_Port == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if (m_RX_Port == GPIOC || m_TX_Port == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    }

    //podpięcie rxa i txa pod gpio
    m_RX_Port->MODER |= (2U << (m_RX_Pin * 2)); 
    m_TX_Port->MODER |= (2U << (u_conf.TX_Pin * 2)); 

    m_RX_Port->AFR[m_RX_Pin / 8] |= (7U << ((m_RX_Pin % 8) * 4)); 
    m_TX_Port->AFR[u_conf.TX_Pin / 8] |= (7U << ((u_conf.TX_Pin % 8) * 4)); 

    m_RX_Port->OSPEEDR |= (3U << (m_RX_Pin * 2));
    m_TX_Port->OSPEEDR |= (3U << (u_conf.TX_Pin * 2));

    m_RX_Port->PUPDR |= (1U << (m_RX_Pin * 2));
    m_TX_Port->PUPDR |= (1U << (u_conf.TX_Pin * 2));

    //zegar dla uarta
    if (m_UART == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    } else if (m_UART == USART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    } else if (m_UART == USART3) {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    } else if (m_UART == UART4) {
        RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
    } else if (m_UART == UART5) {
        RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
    }

    if (m_BR == 0 || m_UART == nullptr) {
        return -1
    }
    const uint32_t over8 = 0; 
    
    if (m_UART == USART1) {
        clock = 9000000;
    } 
    else {
        clock = 4500000;
    }

    uint32_t usartdiv = clock / (m_BR * (16 >> over8));
    uint32_t mantissa = usartdiv / 16; 
    uint32_t fraction = usartdiv % 16; 

    if (mantissa == 0 || mantissa > 0xFFF) {
        return -1
    }
    m_UART->BRR = (mantissa << 4) | fraction; 


    m_UART->CR1 |= USART_CR1_TE | USART_CR1_RE;
    m_UART->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
    m_UART->CR1 |= USART_CR1_UE; 

    bool UART_DMA::transmit() {
    const char* data; 
    uint16_t size = 10; 

    DMA_Stream_TypeDef* txStream = reinterpret_cast<DMA_Stream_TypeDef*>(m_dma_stream_tx);
    txStream->CR &= ~DMA_SxCR_EN; 
    txStream->NDTR = size; 
    txStream->M0AR = reinterpret_cast<uint32_t>(data); 
    txStream->PAR = reinterpret_cast<uint32_t>(&m_UART->DR); 
    txStream->CR = (m_dma_channel_tx << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
    txStream->CR |= DMA_SxCR_EN; 

    while (!(m_UART->SR & USART_SR_TC)); 
    return true;
}
}

void UART_DMA::receive() {
    char buffer[20]; 
    uint16_t size = 20; 

    DMA_Stream_TypeDef* rxStream = reinterpret_cast<DMA_Stream_TypeDef*>(m_dma_stream_rx);
    rxStream->CR &= ~DMA_SxCR_EN; 
    rxStream->NDTR = size;
    rxStream->M0AR = reinterpret_cast<uint32_t>(buffer);
    rxStream->PAR = reinterpret_cast<uint32_t>(&m_UART->DR); 
    rxStream->CR = (m_dma_channel_rx << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC;
    rxStream->CR |= DMA_SxCR_EN; 

    // Czekaj na zakończenie odbioru
    while (!(rxStream->CR & DMA_SxCR_EN));
    return true;
}

void UART_DMA::send() {
    const char* data;  //  ###################################################################
    while (*data) {
        while (!(m_UART->SR & USART_SR_TXE)); 
        m_UART->DR = *data++; 
    return true;
}


void UART_DMA::read() {
    while (!(m_UART->SR & USART_SR_RXNE)); 
    char received = m_UART->DR;
    printf(%c, received);
    return true;
}
return 1;
}