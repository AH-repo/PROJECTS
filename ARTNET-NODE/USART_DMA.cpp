#pragma once
#include "UART_DMA.hpp"

UART_DMA::UART_DMA(const uart_config& u_conf) : m_TX_busy(true), m_RX_busy(true)  {
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

    


    //Odpalenie clocka dla gpio
    if (m_RX_Port == GPIOA || m_TX_Port == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if (m_RX_Port == GPIOB || m_TX_Port == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if (m_RX_Port == GPIOC || m_TX_Port == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    } else if (m_RX_Port == GPIOD || m_TX_Port == GPIOD) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    }

    //podpięcie rxa i txa pod gpio
    m_CS_Port->MODER &= ~(0x3 << (m_TX_Pin*2));     // Clear MODE setting
    m_CS_Port->MODER |= (0x2 << (m_TX_Pin*2));      // Output MODE   

    
    if(m_TX_Pin < 8){
        m_TX_Port->AFR[0]    &=      ~(0xf << (m_TX_Pin*4));
        if(m_UART = USART1 ||m_UART = USART2 ||m_UART = USART3){
            m_TX_Port->AFR[0]    |=      (0x7 << (m_TX_Pin*4));
        }else{
            m_TX_Port->AFR[0]    |=      (0x8 << (m_TX_Pin*4));
        }
    }else{
        m_TX_Port->AFR[1]     &=     ~(0xf << ((m_TX_Pin-8)*4));
        if(m_UART = USART1 ||m_UART = USART2 ||m_UART = USART3){
            m_TX_Port->AFR[1]    |=      (0x7 << (m_TX_Pin-8)*4);
        }else{
            m_TX_Port->AFR[1]    |=      (0x8 << (m_TX_Pin-8)*4);
        }
    }
    

    if(m_RX_Pin < 8){
        m_RX_Port->AFR[0]    &=      ~(0xf << (m_RX_Pin*4));
        if(m_UART = USART1 ||m_UART = USART2 ||m_UART = USART3){
            m_RX_Port->AFR[0]    |=      (0x7 << (m_RX_Pin*4));
        }else{
            m_RX_Port->AFR[0]    |=      (0x8 << (m_RX_Pin*4));
        }
    }else{
        m_RX_Port->AFR[1]     &=     ~(0xf << ((m_RX_Pin-8)*4));
        if(m_UART = USART1 ||m_UART = USART2 ||m_UART = USART3){
            m_RX_Port->AFR[1]    |=      (0x7 << (m_RX_Pin-8)*4);
        }else{
            m_RX_Port->AFR[1]    |=      (0x8 << (m_RX_Pin-8)*4);
        }
    }
    


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

    if (m_BR == 0) {
         printf("Wrong BR");
        __disable_irq();
        while(1);
    }

    
    if (m_UART == USART1) {
        clock = 9000000;
    } 
    else {
        clock = 4500000;
    }

    double usartdiv = clock / (m_BR * (16));
    uint16_t mantissa = static_cast<uint16_t>(usartdiv / 16);  
    if(((usartdiv - mantissa)*16)-(static_cast<uint8_t>((usartdiv - mantissa)*16)) >= 0.5){
        uint8_t fraction = static_cast<uint8_t>((usartdiv - mantissa)*16+1); 
    }else{
    uint8_t fraction = static_cast<uint8_t>((usartdiv - mantissa)*16); 
    };

    if (mantissa == 0 || mantissa > 0xFFF) {
        printf("Wrong BR");
         __disable_irq();
        while(1);
    }
    m_UART->BRR = mantissa << 4 | fraction; 


    m_UART->CR1 |= USART_CR1_TE | USART_CR1_RE;
    m_UART->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
    m_UART->CR1 |= USART_CR1_UE; 

    bool UART_DMA::transmit(uint8_t *data, uint16_t size) {
    const char* data; 
    uint16_t size = 10; 

    DMA_Stream_TypeDef* txStream = reinterpret_cast<DMA_Stream_TypeDef*>(m_dma_stream_tx);
    txStream->CR &= ~DMA_SxCR_EN; 
    txStream->NDTR = size; 
    txStream->M0AR = reinterpret_cast<uint32_t>(data); 
    txStream->PAR = reinterpret_cast<uint32_t>(&m_UART->DR); 
    txStream->CR = (m_dma_channel_tx << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
    txStream->CR |= DMA_SxCR_EN; 
}
}


void UART_DMA::transmit(const uint8_t* txData, uint16_t size) {

    // Czekaj na zakończenie poprzedniej transmisji
    while (m_TX_busy);
    m_TX_busy = true;

    // Wyłącz DMA Stream przed konfiguracją
    m_dma_stream_tx->CR &= ~DMA_SxCR_EN;
    while (m_dma_stream_tx->CR & DMA_SxCR_EN); // Czekaj na dezaktywację

    // Konfiguracja źródła, celu i długości danych
    m_dma_stream_tx->M0AR = reinterpret_cast<uint32_t>(txData); // Źródło - adres bufora
    m_dma_stream_tx->PAR = reinterpret_cast<uint32_t>(&m_UART->DR); // Cel - rejestr danych SPI
    m_dma_stream_tx->NDTR = size; // Długość danych

    // Konfiguracja trybu DMA
    m_dma_stream_tx->CR &= ~DMA_SxCR_DIR; // Kierunek: pamięć -> peryferium
    m_dma_stream_tx->CR |= DMA_SxCR_DIR_1; // Kierunek: pamięć -> peryferium
    m_dma_stream_tx->CR |= DMA_SxCR_MINC; // Inkrementacja adresu pamięci
    m_dma_stream_tx->CR &= ~DMA_SxCR_PINC; // Brak inkrementacji adresu peryferium


    // Włącz DMA Stream
    m_dma_stream_tx->CR |= DMA_SxCR_EN;

    // Aktywuj SPI, aby rozpocząć transmisję
    m_UART->CR1 |= USART_CR1_TE;

}

void UART_DMA::receive(uint8_t* rxData, uint16_t size) {
    while (m_RX_busy);
    m_RX_busy = true;

    // Wyłączamy DMA Stream przed konfiguracją
    m_dma_stream_rx->CR &= ~DMA_SxCR_EN;
    while (m_dma_stream_rx->CR & DMA_SxCR_EN); // Czekamy na dezaktywację

    // Konfigurujemy źródło, cel i długość danych
    m_dma_stream_rx->M0AR = reinterpret_cast<uint32_t>(rxData); // Cel - adres bufora
    m_dma_stream_rx->PAR = reinterpret_cast<uint32_t>(&m_UART->DR); // Źródło - rejestr danych SPI
    m_dma_stream_rx->NDTR = size; // Długość danych

    //DMA_CR
    m_dma_stream_rx->CR &= ~DMA_SxCR_DIR; // Peryferium -> pamięć
    m_dma_stream_rx->CR |= DMA_SxCR_MINC; // Inkrementacja adresu pamięci
    m_dma_stream_rx->CR &= ~DMA_SxCR_PINC; // Brak inkrementacji adresu peryferium


    //Stream Enable
    m_dma_stream_rx->CR |= DMA_SxCR_EN;

    //uart Active
    m_UART->CR1 |= USART_CR1_RE;

}

void UART_DMA::send(uint8_t data) {
        m_UART->DR = data; 
}


uint8_t UART_DMA::read() {
    return static_cast <uint8_t>(m_UART -> DR);
}

bool UART_DMA::isTXbusy(){
    return m_TX_busy;
}
bool UART_DMA::isRXbusy(){
    return m_RX_busy;
}
void UART_DMA clearTXbusy(){
    m_TX_busy = false;
}

void UART_DMA clearRXbusy(){
    m_RX_busy = false;
}
