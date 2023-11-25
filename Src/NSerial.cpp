//==============================================================================
#include "NSerial.h"
#include "DRV_CPU.h"
#include "DRV_SSR.h"
#include "DRV_IO.h"

//------------------------------------------------------------------------------
	#define dummy						__attribute__((unused))

	#define USART_ENABLE            	(USARTx->CR1 |= USART_CR1_UE)
	#define USART_DISABLE            	(USARTx->CR1 &= ((uint32_t)~USART_CR1_UE))
	#define USART_IS_ENABLED            (bool)(USARTx->CR1 & USART_CR1_UE)
	#define USART_TX_EMPTY              (USARTx->SR & USART_SR_TXE)
	#define USART_RX_FULL               (USARTx->SR & USART_SR_RXNE)
	#define USART_RX_REGISTER           (USARTx->DR)

	#define USART_TXDMA_ENABLE          (USARTx->CR3 |= USART_CR3_DMAT)
	#define USART_TXDMA_DISABLE         (USARTx->CR3 &= ~USART_CR3_DMAT)
	#define USART_RXDMA_ENABLE          (USARTx->CR3 |= USART_CR3_DMAR)
	#define USART_RXDMA_DISABLE         (USARTx->CR3 &= ~USART_CR3_DMAR)
	#define USART_ERROR_MASK			(USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)
	#define USART_ERROR_FLAGS           (USARTx->SR & USART_ERROR_MASK)

	#define USART_INT_ERRORS_CLEAR		{dummy uint8_t d = USARTx->SR; d = USARTx->DR;}
	#define USART_INT_ERRORS_ENABLE		{ USARTx->CR3 |= USART_CR3_EIE;}
	#define USART_INT_ERRORS_DISABLE	{ USARTx->CR3 &= ~USART_CR3_EIE;}

	#define USART_INT_RXNE_CLEAR        (USARTx->SR &= ~USART_SR_RXNE)
	#define USART_INT_RXNE_ENABLE       (USARTx->CR1 |= USART_CR1_RXNEIE)
	#define USART_INT_RXNE_DISABLE      (USARTx->CR1 &= ~USART_CR1_RXNEIE)

	#define USART_INT_IDLE_CLEAR		{dummy uint8_t d = USARTx->SR; d = USARTx->DR;}
	#define USART_INT_IDLE_ENABLE       (USARTx->CR1 |= USART_CR1_IDLEIE)
	#define USART_INT_IDLE_DISABLE      (USARTx->CR1 &= ~USART_CR1_IDLEIE)

	#define USART_INT_TC_ENABLE   		(USARTx->CR1 |= USART_CR1_TCIE)
	#define USART_INT_TC_DISABLE		(USARTx->CR1 &= ~USART_CR1_TCIE)
	#define USART_INT_TC_CLEAR          (USARTx->SR &= ~USART_SR_TC)

	#define USART_INT_CTS_CLEAR         (USARTx->SR &= ~USART_SR_CTS)
	#define USART_INT_CTS_ENABLE		(USARTx->CR3 |= USART_CR3_CTSIE)

	#define NSERIAL_BUFFERSIZE_MIN     	((uint32_t)  8)
	#define NSERIAL_BUFFERSIZE_DEFAULT	((uint32_t) 64)
	#define NSERIAL_BUFFERSIZE_MAX      ((uint32_t)255)

	#define PA							0x0000
	#define PB							0x0100
	#define PC							0x0200
	#define PD							0x0300
	#define PE							0x0400
	#define PF							0x0500
	#define PG							0x0600
	#define NO_PIN						0xFFFF

	#define USART1_STANDARD				0
	#define USART1_REMAPPED				1
	#define USART2_STANDARD				2
	#define USART2_REMAPPED				3
	#define USART3_STANDARD				4
	#define USART3_REMAPPED				5
	#define UART4_STANDARD				6
	#define UART4_REMAPPED				7
	#define UART5_STANDARD				8
	#define UART5_REMAPPED				9


	//--------------------------------------------------------------------------
	/*
	 * @brief Packets buffer class for NSerial.
	 * - This class implements all the basic functionalities for the communication\n
	 * buffers needed by the NSerial component to provide buffered packets for both\n
	 * incoming and outgoing serial data streams.\n
	 */
	//--------------------------------------------------------------------------
    class NSERIAL_BUFFER {
        public:
            uint8_t* Buffer;
            uint8_t  Size;
            uint8_t  Length;
            uint8_t  NextByte;

		NSERIAL_BUFFER(){
		  	Resize(NSERIAL_BUFFERSIZE_DEFAULT);
            NextByte = 0; Length = 0;
		}

		void Resize(uint8_t sz){
		  	Size = sz;
			Buffer = new uint8_t[Size];
			NextByte = Length = 0;
		}

		uint8_t Put(uint8_t dat){
			Buffer[NextByte++] = dat; return(++Length);
		}

		uint8_t Get(){
			uint8_t result = 0;
			if(NextByte < Length){ result = Buffer[NextByte++];}
 			return(result);
		}

		void Reset(){
			for(uint8_t c = 0; c < Size; c++){ Buffer[c]=0;}
			Length = 0; NextByte = 0;
		}

		uint8_t Import(uint8_t* ExternalBuffer, uint8_t ExternalSize){
			if(ExternalSize > Size){ ExternalSize = Size;}
			Length = ExternalSize; NextByte = 0;
			for(uint8_t c = 0; c < Length; c++){Buffer[c] = ExternalBuffer[c];}
			return(Length);
		}

		uint8_t Export(uint8_t* ExternalBuffer){
			for(uint8_t c = 0; c < Length; c++){ ExternalBuffer[c] = Buffer[c];}
			return(Length);
		}

		void Export(NSERIAL_BUFFER* Destination){
			if(Destination->Size < Length){ return;}
			for(uint8_t c = 0; c < Length; c++){ Destination->Buffer[c] = Buffer[c];}
			Destination->Length = Length; Destination->NextByte = NextByte;
		}
	};

//------------------------------------------------------------------------------
uint16_t USART_CONFIGS[10][4]{
	// USART1 STANDARD
	//   TX        RX        RTS        CTS
	{(PA | 9), (PA | 10), (PA | 12), (PA | 11)},
	// USART1 REMAPPED
	//   TX       RX         RTS        CTS
	{(PB | 6), (PB | 7),  (NO_PIN), (NO_PIN)},

	// USART2 STANDARD
	//   TX        RX        RTS        CTS
	{(PA | 2), (PA | 3), (PA | 1), (PA | 0)},
	// USART2 REMAPPED
	//   TX       RX         RTS        CTS
	{(PD | 5), (PD | 6),  (NO_PIN), (NO_PIN)},

	// USART3 STANDARD
	//   TX        RX        RTS        CTS
	{(PB | 10), (PB | 11), (PB | 14), (PB | 13)},
	// USART3 REMAPPED
	//   TX       RX         RTS        CTS
	{(PD | 8), (PD | 9),  (PD | 12), (PD | 11)},

	// UART4 STANDARD
	//   TX        RX        RTS        CTS
	{(PC | 10), (PC | 11), (NO_PIN), (NO_PIN)},
	// UART4 REMAPPED
	//   TX       RX         RTS        CTS
	{(PC | 10), (PC | 11), (NO_PIN), (NO_PIN)},

	// UART5 STANDARD
	//   TX        RX        RTS        CTS
	{(PC | 12), (PD | 12), (NO_PIN), (NO_PIN)},
	// UART5 REMAPPED
	//   TX       RX         RTS        CTS
	{(PC | 12), (PD | 12), (NO_PIN), (NO_PIN)}
};

//------------------------------------------------------------------------------
struct USART_Pinout{
    uint16_t tx;
    uint16_t rx;
    uint16_t rts;
    uint16_t cts;
};

//------------------------------------------------------------------------------
void NSerial::Pinout(uint32_t i){
    IO_Config PinConfig;
    USART_Pinout pinout;
    pinout.tx  = USART_CONFIGS[i][0];
    pinout.rx  = USART_CONFIGS[i][1];
    pinout.rts = USART_CONFIGS[i][2];
    pinout.cts = USART_CONFIGS[i][3];

    //--------------------------------------------
    PinConfig.Mode = io_Alt_PushPull_50MHz;

    PinConfig.Pin = pinout.tx & __MASK_PORT;
    IO_PinInit(IO_PortClockOn(pinout.tx), &PinConfig);

    if((flowcontrol == seFlowRTS)||(flowcontrol == seFlowFull)){
		if(pinout.rts != NO_PIN){
			PinConfig.Pin = pinout.rts & __MASK_PORT;
			IO_PinInit(IO_PortClockOn(pinout.rts), &PinConfig);
		}
    }

    //--------------------------------------------
    PinConfig.Mode = io_In_PullUp;

    PinConfig.Pin = pinout.rx & __MASK_PORT;
    IO_PinInit(IO_PortClockOn(pinout.rx), &PinConfig);

    if((flowcontrol == seFlowCTS)||(flowcontrol == seFlowFull)){
		if(pinout.cts != NO_PIN){
			PinConfig.Pin = pinout.cts & __MASK_PORT;
			IO_PinInit(IO_PortClockOn(pinout.cts), &PinConfig);
		}
    }
}

//------------------------------------------------------------------------------
NSerial::~NSerial(){
	if(USART_IS_ENABLED){ Close();}
    if(PacoteLocalTx != NULL){ delete[] PacoteLocalTx;}
    if(PacoteLocalRx != NULL){ delete[] PacoteLocalRx;}
    if(PacoteDisponivelRx != NULL){ delete[] PacoteDisponivelRx;}
}

//------------------------------------------------------------------------------
NSerial::NSerial(USART_TypeDef* USARTn, seRemapping Remap){

    USARTx = USARTn;
    OnError = NULL; OnWriteCompleted = NULL; OnReadCompleted = NULL;
    OnTimeout = NULL; OnPacket = NULL; OnCTS = NULL; OnIdleDetected = NULL;
    OnEnterTransmission = NULL; OnLeaveTransmission = NULL;

    Port.setOwner(this);
    Port.get(&NSerial::GetPort);

    BaudRate.setOwner(this);
    BaudRate.set(&NSerial::SetBaudRate);
    BaudRate.get(&NSerial::GetBaudRate);

    WordLength.setOwner(this);
    WordLength.set(&NSerial::SetWordLength);
    WordLength.get(&NSerial::GetWordLength);

    Parity.setOwner(this);
    Parity.set(&NSerial::SetParity);
    Parity.get(&NSerial::GetParity);

    StopBits.setOwner(this);
    StopBits.set(&NSerial::SetStopBits);
    StopBits.get(&NSerial::GetStopBits);

    RxMode.setOwner(this);
    RxMode.set(&NSerial::SetRxMode);
    RxMode.get(&NSerial::GetRxMode);

    TxMode.setOwner(this);
    TxMode.set(&NSerial::SetTxMode);
    TxMode.get(&NSerial::GetTxMode);

    FlowControl.setOwner(this);
    FlowControl.set(&NSerial::SetFlowControl);
    FlowControl.get(&NSerial::GetFlowControl);

    PacketSize.setOwner(this);
    PacketSize.set(&NSerial::SetPacketSize);
    PacketSize.get(&NSerial::GetPacketSize);

    Timeout.setOwner(this);
    Timeout.set(&NSerial::SetTimeout);
    Timeout.get(&NSerial::GetTimeout);

    TransmissionTimeout.setOwner(this);
    TransmissionTimeout.set(&NSerial::SetTransmissionTimeout);
    TransmissionTimeout.get(&NSerial::GetTransmissionTimeout);

    BufferSizeRx.setOwner(this);
    BufferSizeRx.set(&NSerial::SetBufferSizeRx);
    BufferSizeRx.get(&NSerial::GetBufferSizeRx);

    BufferSizeTx.setOwner(this);
    BufferSizeTx.set(&NSerial::SetBufferSizeTx);
    BufferSizeTx.get(&NSerial::GetBufferSizeTx);

    receiving_counter = 0;
    reception_timeout = 0;
    transmitting_counter = 0;

    baud = 115200;
    wordlength = seBits8;
    parity = seNoParity;
    stopbits = seStop1;
    flowcontrol = seFlowNone;

    rxmode = seDMA;
    txmode = seDMA;
    parent_rx = false;
    parent_tx = false;
    vector_index = 0;
	Priority = nNormal;
    LastSize = packetsize;
    packet_timeout = 0;

	#if defined(UART5)
    if(USARTx == UART5){
        rxmode = seNoDMA;
        txmode = seNoDMA;
    }
	#endif


	PacoteLocalRx = new NSERIAL_BUFFER();
	PacoteDisponivelRx = new NSERIAL_BUFFER();
    PacoteLocalTx = new NSERIAL_BUFFER();
    PacoteDisponivelTx = NULL;

    packetsize = NSERIAL_BUFFERSIZE_DEFAULT;
	transmission_timeout = TRANSMISSION_TIMEOUT_MAX;

    device_pinout = 0;

    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    if((uint32_t)USARTx == (uint32_t)USART1){
        // (USART1 = 37)
        USARTx_IRQn = USART1_IRQn; vector_index = NV_UART1;

        if(Remap != seStandard){
        	device_pinout = USART1_REMAPPED;
        	AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
        } else {
        	device_pinout = USART1_STANDARD;
            AFIO->MAPR &= ~AFIO_MAPR_USART1_REMAP;
        }
        //----------------------------------------------------
        DMAChannelTx = DMA1_Channel4; IRQChannelTx = DMA1_Channel4_IRQn; vector_dma_tx = NV_DMA1_CH4;
        DMAChannelRx = DMA1_Channel5; IRQChannelRx = DMA1_Channel5_IRQn; vector_dma_rx = NV_DMA1_CH5;
        //----------------------------------------------------
    } else if((uint32_t)USARTx == (uint32_t)USART2){
        // (USART2 = 38)
        USARTx_IRQn = USART2_IRQn; vector_index = NV_UART2;
        if(Remap != seStandard){
        	device_pinout = USART2_REMAPPED;  //TX: PD5, RX: PD6
        	AFIO->MAPR |= AFIO_MAPR_USART2_REMAP;
        } else {
        	device_pinout = USART2_STANDARD;  //TX: PA2, RX: PA3
        	AFIO->MAPR &= ~AFIO_MAPR_USART2_REMAP;
        }
        //----------------------------------------------------
        DMAChannelTx = DMA1_Channel7; IRQChannelTx = DMA1_Channel7_IRQn; vector_dma_tx = NV_DMA1_CH7;
        DMAChannelRx = DMA1_Channel6; IRQChannelRx = DMA1_Channel6_IRQn; vector_dma_rx = NV_DMA1_CH6;
        //----------------------------------------------------
	#if defined(USART3)
    } else if((uint32_t)USARTx == (uint32_t)USART3){
        // (USART3 = 39)
        USARTx_IRQn = USART3_IRQn; vector_index = NV_UART3;
        if(Remap != seStandard){
        	device_pinout = USART3_REMAPPED;
        	AFIO->MAPR |= AFIO_MAPR_USART3_REMAP;
        } else {
        	device_pinout = USART3_STANDARD;
        	AFIO->MAPR &= ~AFIO_MAPR_USART3_REMAP;
        }
        //----------------------------------------------------
        DMAChannelTx = DMA1_Channel2; IRQChannelTx = DMA1_Channel2_IRQn; vector_dma_tx = NV_DMA1_CH2;
        DMAChannelRx = DMA1_Channel3; IRQChannelRx = DMA1_Channel3_IRQn; vector_dma_rx = NV_DMA1_CH3;
        //----------------------------------------------------
    }
	#endif
	#if defined(UART4)
    else if((uint32_t)USARTx == (uint32_t)UART4){
        // (UART4 = 59)
        USARTx_IRQn = UART4_IRQn; vector_index = NV_UART4;
        if(Remap != seStandard){
        	device_pinout = UART4_REMAPPED;
        	AFIO->MAPR |= AFIO_MAPR_USART4_REMAP;
        } else {
        	device_pinout = UART4_STANDARD;
        	AFIO->MAPR &= ~AFIO_MAPR_USART4_REMAP;
        }
        //----------------------------------------------------
        DMAChannelTx = DMA2_Channel5; IRQChannelTx = DMA2_Channel5_IRQn; vector_dma_tx = NV_DMA2_CH5;
        DMAChannelRx = DMA2_Channel3; IRQChannelRx = DMA2_Channel3_IRQn; vector_dma_rx = NV_DMA2_CH3;
        //----------------------------------------------------

    }
	#endif

	#if defined(UART5)
    else{
        // (UART5 = 60)
        USARTx_IRQn = UART5_IRQn; vector_index = NV_UART5;
        if(Remap != seStandard){
        	device_pinout = UART5_REMAPPED;
        	AFIO->MAPR |= AFIO_MAPR_USART5_REMAP;
        } else {
        	device_pinout = UART5_STANDARD;
        	AFIO->MAPR &= ~AFIO_MAPR_USART5_REMAP;
        }
    }
	#endif

	} else { };
    Pinout(device_pinout);

    //-----------------------------------------
    uint32_t pri = NVIC_EncodePriority(NVIC_PriorityGroup_4, SYS_PRIORITY_NORMAL, 0);
    NVIC_SetPriority(USARTx_IRQn, pri);
    NVIC_EnableIRQ(USARTx_IRQn);
}

//------------------------------------------------------------------------------
void NSerial::Open(){
    //---------------------------------------
    if(USARTx == NULL){
        SSR_ThrowException(NX_PARAMETER_NULL);
        return;
    }

    //---------------------------------------
    SetPacketSize(packetsize);
    SetTimeout(packet_timeout);

    //---------------------------------------
    CPU_PeripheralClockEnable(USARTx);

    //---------------------------------------
    USART_DISABLE;
    USART_TXDMA_DISABLE;
    USART_RXDMA_DISABLE;

	//---------------------------------------
    USART_INT_CTS_CLEAR;
    USART_INT_CTS_ENABLE;
    
    //---------------------------------------
    SSR_InstallCallback((uint32_t)Handle, vector_index);

    //---------------------------------------
	flagReading = false;
	flagWriting = false;
	flagReceiving = false;
	flagTransmitting = false;
    
    SetBaudRate(baud);
    SetWordLength(wordlength);
    SetParity(parity);
    SetStopBits(stopbits);
    SetFlowControl(flowcontrol);
    SetRxMode(rxmode);
    SetTxMode(txmode);

    //RestartPacket();
	USART_ENABLE;
    Flush();
}

//------------------------------------------------------------------------------
void NSerial::Close(){
	USART_DISABLE;
    if(parent_rx){
        SSR_InstallCallback(NV_NULL, vector_index);
    }

    CPU_PeripheralClockDisable(USARTx);
}

//------------------------------------------------------------------------------
USART_TypeDef* NSerial::GetPort(){
    return(USARTx);
}

//------------------------------------------------------------------------------
bool NSerial::GetIsOpen(){ return(USART_IS_ENABLED);}

//------------------------------------------------------------------------------
void NSerial::SetBaudRate(uint32_t baudrate){
    if((baudrate >= ((uint32_t)1200))&&(baudrate <= ((uint32_t)115200))){

        baud = baudrate;
        // 36.000.000 / (115200 *16) = 19.5
        // Mantissa: 19 or 13h
        // Fraction: 0.5 * 16 = 8 or 08h
        // BRR: 138h
        
        uint32_t pclk = 0;

        if(USARTx == USART1){ pclk = CPU_GetFrequencyAPB2();}
        else { pclk = CPU_GetFrequencyAPB1();}
        uint32_t cycles = baud * 16;

        if(pclk < cycles){
            SSR_ThrowException(NX_NSERIAL_BAUDRATE_UNREACHABLE);
            return;
        }

        uint32_t mantissa = pclk / cycles;
        uint32_t fraction = (pclk -(mantissa*cycles))/(baud+8);

        if(CPU_PeripheralClockStatus(USARTx)==true){
        	USARTx->BRR =  (mantissa & 0x0FFF)<<4 | (fraction & 0x000F);  //138h
            USARTx->SR = 0;
        }
    }
}

//------------------------------------------------------------------------------
uint32_t NSerial::GetBaudRate(){
    return(baud);
}

//------------------------------------------------------------------------------
void NSerial::SetWordLength(seWordLengths wlen){
    wordlength = wlen;
    if(CPU_PeripheralClockStatus(USARTx)==true){
		if(wlen == seBits9){ USARTx->CR1 |= USART_CR1_M;}
		else { USARTx->CR1 &= ~USART_CR1_M;}
        USARTx->SR = 0;
    }
}

//------------------------------------------------------------------------------
seWordLengths NSerial::GetWordLength(){ return(wordlength);}

//------------------------------------------------------------------------------
void NSerial::SetParity(seParities par){
	parity = par;
	if(CPU_PeripheralClockStatus(USARTx)==true){
		USARTx->CR1 &= ~USART_CR1_PCE;
		if(par == seParityOdd){ USARTx->CR1 |= (USART_CR1_PS | USART_CR1_PCE);}
		else if(par == seParityEven){
			USARTx->CR1 = ((USARTx->CR1 & ~USART_CR1_PS) | USART_CR1_PCE);}
	}
}

//------------------------------------------------------------------------------
seParities NSerial::GetParity(){ return(parity);}

//------------------------------------------------------------------------------
void NSerial::SetStopBits(seStopBits stops){

	stopbits = stops;
	if(CPU_PeripheralClockStatus(USARTx)==true){
		USARTx->CR2 &= ~USART_CR2_STOP_Msk;
		USARTx->CR2 |= ((uint32_t)stops << USART_CR2_STOP_Pos);
	}
}

//------------------------------------------------------------------------------
seStopBits NSerial::GetStopBits(){ return(stopbits);}

//------------------------------------------------------------------------------
void NSerial::SetFlowControl(seFlowControls flowc){
	USARTx->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
   	if((flowc == seFlowCTS)||(flowc == seFlowFull)){ USARTx->CR3 |= USART_CR3_CTSE;}
   	if((flowc == seFlowRTS)||(flowc == seFlowFull)){ USARTx->CR3 |= USART_CR3_RTSE;}
   	if(flowcontrol != flowc){ Pinout(device_pinout);}
	flowcontrol = flowc;

}

//------------------------------------------------------------------------------
seFlowControls NSerial::GetFlowControl(){ return(flowcontrol);}

//------------------------------------------------------------------------------
void NSerial::SetRxMode(seTransferModes rmode){

		rxmode = rmode;

		#if defined(UART5)
		if(USARTx == UART5){ rmode = seNoDMA;}
		#endif

	if(rxmode == seOff) {
		NVIC_DisableIRQ((IRQn_Type)IRQChannelRx);
		SSR_InstallCallback((uint32_t)NV_NULL, vector_dma_rx);
		USARTx->CR1 &= ~(USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_IDLEIE);
	} else {
		if(rxmode == seDMA){
			#if defined(UART4)
				if(USARTx == UART4){ CPU_PeripheralClockEnable(DMA2);}
			#endif
			#if defined(UART3)
			if(USARTx == USART3)){ CPU_PeripheralClockEnable(DMA1);}
			#endif

			if((USARTx == USART1)||(USARTx == USART2)){
				CPU_PeripheralClockEnable(DMA1);
			}

			//-----------------------------------------
			uint32_t pri = NVIC_EncodePriority(NVIC_PriorityGroup_4, SYS_PRIORITY_NORMAL, 0);
			NVIC_SetPriority((IRQn_Type)IRQChannelRx, pri);
			NVIC_EnableIRQ((IRQn_Type)IRQChannelRx);
			SSR_InstallCallback((uint32_t)Handle, vector_dma_rx);
			//---------------------------------------
			USART_RXDMA_ENABLE;

		} else if(rxmode == seNoDMA){
			USART_RXDMA_DISABLE;
			NVIC_DisableIRQ((IRQn_Type)IRQChannelRx);
			SSR_InstallCallback((uint32_t)NV_NULL, vector_dma_rx);
		}
		USARTx->CR1 |=  (USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_IDLEIE);
	}
}

//------------------------------------------------------------------------------
seTransferModes NSerial::GetRxMode(){ return(rxmode);}

//------------------------------------------------------------------------------
void NSerial::SetTxMode(seTransferModes tmode){
    txmode = tmode;

	#if defined(UART5)
    if(USARTx == UART5){ return;}
	#endif

	if(txmode == seOff) {
		NVIC_DisableIRQ((IRQn_Type)IRQChannelTx);
		SSR_InstallCallback((uint32_t)NV_NULL, vector_dma_tx);
		USARTx->CR1 &= ~(USART_CR1_TE);
	} else {
		if(txmode == seDMA){
			#if defined(UART4) && defined(DMA2)
			 if(USARTx ==UART4){ CPU_PeripheralClockEnable(DMA2);}
			#endif
			#if defined(UART3)
			if(USARTx == USART3)){ CPU_PeripheralClockEnable(DMA1);}
			#endif

			if((USARTx == USART1)||(USARTx == USART2)){
				CPU_PeripheralClockEnable(DMA1);
			}

			//-----------------------------------------
			uint32_t pri = NVIC_EncodePriority(NVIC_PriorityGroup_4, SYS_PRIORITY_NORMAL, 0);
			NVIC_SetPriority((IRQn_Type)IRQChannelTx, pri);
			NVIC_EnableIRQ((IRQn_Type)IRQChannelTx);
			SSR_InstallCallback((uint32_t)Handle, vector_dma_tx);
			//---------------------------------------
			USART_TXDMA_ENABLE;

		} else if(txmode == seNoDMA){
			USART_TXDMA_DISABLE;
			NVIC_DisableIRQ((IRQn_Type)IRQChannelTx);
			SSR_InstallCallback((uint32_t)NV_NULL, vector_dma_tx);
		}
		USARTx->CR1 |= (USART_CR1_TE);

	}
}

//------------------------------------------------------------------------------
seTransferModes NSerial::GetTxMode(){ return(txmode);}

//------------------------------------------------------------------------------
void NSerial::SetPacketSize(uint32_t psize){
	if(psize == 0){ psize = 1;}
	else if(psize > PacoteLocalRx->Size){ psize = PacoteLocalRx->Size;}
    packetsize = psize;
    LastSize = psize;
    RestartPacket();
}

//------------------------------------------------------------------------------
uint32_t NSerial::GetPacketSize(){ return(packetsize);}

//------------------------------------------------------------------------------
void NSerial::SetTimeout(uint32_t new_timeout){
    if(new_timeout>  PACKET_TIMEOUT_MAX){
        new_timeout = PACKET_TIMEOUT_MAX;
    }
	packet_timeout = new_timeout;
}

//------------------------------------------------------------------------------
uint32_t NSerial::GetTimeout(){ return(packet_timeout);}

//------------------------------------------------------------------------------
void NSerial::SetBufferSizeRx(uint32_t new_size){
	bool e = false;
    if(new_size < NSERIAL_BUFFERSIZE_MIN) { new_size = NSERIAL_BUFFERSIZE_MIN; e=true;}
    if(new_size > NSERIAL_BUFFERSIZE_MAX) { new_size = NSERIAL_BUFFERSIZE_MAX; e=true;}
    if(e){ SSR_ThrowException(NX_PROPERTY_VALUE_OUT_OF_RANGE);}
	PacoteLocalRx->Resize(new_size);
    PacoteDisponivelRx->Resize(new_size);
}

//------------------------------------------------------------------------------
uint32_t NSerial::GetBufferSizeRx(){ return(PacoteLocalRx->Size);}

//------------------------------------------------------------------------------
void NSerial::SetBufferSizeTx(uint32_t new_size){
	bool e = false;
    if(new_size < NSERIAL_BUFFERSIZE_MIN) { new_size = NSERIAL_BUFFERSIZE_MIN; e=true;}
    if(new_size > NSERIAL_BUFFERSIZE_MAX) { new_size = NSERIAL_BUFFERSIZE_MAX; e=true;}
    if(e){ SSR_ThrowException(NX_PROPERTY_VALUE_OUT_OF_RANGE);}
	PacoteLocalTx->Resize(new_size);
	if(PacoteDisponivelTx!=NULL){ PacoteDisponivelTx->Resize(new_size);}
}

//------------------------------------------------------------------------------
uint32_t NSerial::GetBufferSizeTx(){ return(PacoteLocalTx->Size);}

//------------------------------------------------------------------------------
void NSerial::SetTransmissionTimeout(uint32_t new_timeout){
    if(new_timeout == 0) { new_timeout = 1;}
    else if(new_timeout >  TRANSMISSION_TIMEOUT_MAX){
        new_timeout = TRANSMISSION_TIMEOUT_MAX;
    }
	transmission_timeout = new_timeout;
}

//------------------------------------------------------------------------------
uint32_t NSerial::GetTransmissionTimeout(){ return(transmission_timeout);}


//------------------------------------------------------------------------------
bool NSerial::ReloadTxFifo(){

    while(PacoteLocalTx->NextByte < PacoteLocalTx->Length){
         if(USART_TX_EMPTY){
             USARTx->DR = PacoteLocalTx->Buffer[PacoteLocalTx->NextByte++];
         } else break;
    }

    if(PacoteLocalTx->NextByte >= PacoteLocalTx->Length){
        PacoteLocalTx->NextByte = PacoteLocalTx->Length = 0;
        if(busy){ busy = false;}
    }

    USARTx->CR1 |= USART_CR1_TXEIE;
    
    return(!(bool)PacoteLocalTx->Length);
}

//------------------------------------------------------------------------------
void NSerial::StartTimeout(uint32_t Ti){
    if(USART_IS_ENABLED){
		//----------------------------------------
		if(Ti > 0){
			if(Ti > PACKET_TIMEOUT_MAX){ Ti = PACKET_TIMEOUT_MAX;}
			receiving_counter = 0;
			reception_timeout = Ti;
			flagReceiving = true;
		}
    }
}

//------------------------------------------------------------------------------
void NSerial::StopTimeout(){
    receiving_counter=0;
    flagReceiving = false;
}

//------------------------------------------------------------------------------
void NSerial::RestartPacket(){

    if(!flagReading){
        packetsize = LastSize;
        receiving_counter = 0;
        reception_timeout = packet_timeout;
        pacoteInicializado = false;
        flagReceiving = false;

		PacoteLocalRx->Reset();

		//---------------------------------------
		USART_INT_ERRORS_CLEAR;
        USART_INT_RXNE_CLEAR;
		USART_INT_IDLE_CLEAR;

        USART_INT_ERRORS_ENABLE;
        USART_INT_RXNE_ENABLE;

		//---------------------------------------
        USART_RXDMA_DISABLE;

        if(rxmode == seDMA){
        	//USART_INT_IDLE_ENABLE;
        	USART_RXDMA_ENABLE;
			Read(PacoteLocalRx->Size, 0);
        } else {
        	USART_RXDMA_DISABLE;
        	//USART_INT_IDLE_DISABLE;
        }
        USART_INT_IDLE_ENABLE;
    }
    return;
}

//------------------------------------------------------------------------------
bool NSerial::PackData(uint8_t  inData){
	bool result = false;
    //-------------------------------------
	PacoteLocalRx->Put(inData);

    //-------------------------------------
    if((bool)pacoteInicializado == false){
        pacoteInicializado = true;
        if(!flagReading) StartTimeout(packet_timeout);
    }

    //-------------------------------------
	if(PacoteLocalRx->NextByte == packetsize){
		StopTimeout();
		PacoteLocalRx->Export(PacoteDisponivelRx);
		PacoteLocalRx->Reset();
		pacoteInicializado = false;
		result = true;
	}
    return(result);
}

//------------------------------------------------------------------------------
void NSerial::Flush(){
    flagReading = false;
	if(rxmode == seNoDMA){
	    while(USART_RX_FULL){
	    	dummy uint8_t val = USART_RX_REGISTER;
	    }
	} else {
        DMAChannelRx->CCR &= ~DMA_CCR_EN;
    }
	RestartPacket();
}

//------------------------------------------------------------------------------
void NSerial::SetDmaReception(uint8_t* rxBuffer, uint32_t rxSize){

  	if(rxSize > 0){
        DMAChannelRx->CCR = 0;
        DMAChannelRx->CPAR = (uint32_t)(&USARTx->DR);
        DMAChannelRx->CMAR = (uint32_t)(rxBuffer);
        DMAChannelRx->CNDTR = rxSize;
        DMAChannelRx->CCR |= (DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_MINC);
        DMAChannelRx->CCR |= DMA_CCR_EN;
    	USART_RXDMA_ENABLE;
	}
}

//------------------------------------------------------------------------------
void NSerial::Read(uint8_t PacketLength, uint32_t TimeToWait){
    if((USART_IS_ENABLED)&&(!flagReading)){
		if(PacketLength > 0){
	        //----------------------------------------
			StartTimeout(TimeToWait);
	       	if(TimeToWait > 0){ flagReading = true;}
			
			//----------------------------------------
	        if(PacketLength > PacoteLocalRx->Size){
	        	packetsize = PacoteLocalRx->Size;
			} else { packetsize = PacketLength;}

			PacoteLocalRx->Reset();

			//----------------------------------------
	        if(rxmode == seNoDMA){
	            pacoteInicializado = false;
	            USART_INT_RXNE_ENABLE;
	            USART_INT_ERRORS_ENABLE;
	        } else {
				PacoteLocalRx->Length = PacketLength;
				SetDmaReception(PacoteLocalRx->Buffer, PacketLength);
	        }
		}
    }
}

//------------------------------------------------------------------------------
void NSerial::SetDmaTransmission(uint8_t* txBuffer, uint32_t txSize){

	if(txSize > 0){
		DMAChannelTx->CCR = 0;
		DMAChannelTx->CPAR = (uint32_t)(&USARTx->DR);
		DMAChannelTx->CMAR = (uint32_t)(txBuffer);
		DMAChannelTx->CNDTR = txSize;
		DMAChannelTx->CCR |= (DMA_CCR_DIR | DMA_CCR_MINC);
		DMAChannelTx->CCR |= (DMA_CCR_TCIE | DMA_CCR_TEIE);
		DMAChannelTx->CCR |= DMA_CCR_EN;
		USART_TXDMA_ENABLE;
	}
}

//------------------------------------------------------------------------------
bool NSerial::Write(uint8_t* pkt, uint32_t n){
	if((n==0)||(pkt==NULL)){ return(false);}

    if((USART_IS_ENABLED)&&(!flagWriting)){
        flagWriting = true; flagTransmitting = true;

        PacoteLocalTx->Import(pkt, n);

		transmitting_counter = transmission_timeout;
		if(OnEnterTransmission != NULL){ OnEnterTransmission();}

		USART_INT_TC_CLEAR;

        if(txmode == seNoDMA){
            busy = true;
            return(ReloadTxFifo());
        } else {
        	SetDmaTransmission(PacoteLocalTx->Buffer, PacoteLocalTx->Length);
            busy = false;
            return(true);
        }
    } else {
        return(false);
    }
}

//------------------------------------------------------------------------------
bool NSerial::PacketNotification(uint8_t*, uint16_t){
	if(flagReading){ flagReading = false;}
	RestartPacket();
	return(true);
}

//------------------------------------------------------------------------------
void NSerial::DiscardNotification(uint8_t*, uint16_t){}

//------------------------------------------------------------------------------
void NSerial::Notify(NMESSAGE* pmsg){
    if(!USART_IS_ENABLED){
        pmsg->message = NM_NULL;
        return;
    }

    switch(pmsg->message){

        case NM_UARTERROR:
			pmsg->message = NM_NULL;
            if(USARTx == (USART_TypeDef*)pmsg->data1){
                if(OnError != NULL) OnError((uint16_t)pmsg->data2);
				flagReceiving = false; flagReading = false;
				RestartPacket();
			}
            break;

		case NM_UARTRXIDLE:
			pmsg->message = NM_NULL;
			if(rxmode == seDMA){
				if((DMAChannelRx->CCR & DMA_CCR_EN) == false) { return;}
				flagReceiving = false;
				PacoteLocalRx->Length = (packetsize - DMAChannelRx->CNDTR);
				if(PacoteLocalRx->Length > 0){
					PacoteLocalRx->Export(PacoteDisponivelRx);
				}
				if(parent_rx){
					PacketNotification(PacoteDisponivelRx->Buffer, PacoteDisponivelRx->Length);
				} else {
					if(flagReading){
						if(OnReadCompleted != NULL) OnReadCompleted(PacoteDisponivelRx->Buffer, PacoteDisponivelRx->Length);
					} else {
						if(OnPacket != NULL){ OnPacket(PacoteDisponivelRx->Buffer, PacoteDisponivelRx->Length);}
					}
				}
				flagReading = false;
				RestartPacket();
            }
			//else {
                if(OnIdleDetected != NULL){ OnIdleDetected();}
            //}
			break;

        case NM_UARTRX:
			pmsg->message = NM_NULL;
			if(PackData((unsigned char)pmsg->data2)){
				if(parent_rx){
					if(PacketNotification(PacoteDisponivelRx->Buffer, PacoteDisponivelRx->Length)){
						flagReading = false;
						RestartPacket();
					}
				} else {
					if(flagReading){
						flagReading = false;
						if(OnReadCompleted != NULL) OnReadCompleted(PacoteDisponivelRx->Buffer, PacoteDisponivelRx->Length);
					} else {
						if(OnPacket != NULL) OnPacket(PacoteDisponivelRx->Buffer, PacoteDisponivelRx->Length);
					}
					RestartPacket();
				}
			} else {
				USART_INT_RXNE_ENABLE;
				USART_INT_ERRORS_ENABLE;
			}
			break;

        case NM_UARTTX:
			pmsg->message = NM_NULL;
			if(busy){ ReloadTxFifo();}
			else {
				if(pmsg->data2 == USART_SR_TC){
					if(DMAChannelTx->CCR & DMA_CCR_EN){DMAChannelTx->CCR = 0;}
					if(OnLeaveTransmission != NULL){ OnLeaveTransmission();}
					if(parent_tx){ pmsg->message = NM_UARTTX;}
					else { if(OnWriteCompleted != NULL) OnWriteCompleted();}
					flagTransmitting = false; flagWriting = false;
				} else {
				  USART_INT_TC_ENABLE;
				}
			}
            break;

        case NM_UARTCTS:
			pmsg->message = NM_NULL;
            if(OnCTS != NULL){ OnCTS();}
            break;

        case NM_DMA_OK:
			pmsg->message = NM_NULL;
            if(DMAChannelTx == (DMA_Channel_TypeDef*)pmsg->data2){
                DMAChannelTx->CCR &= ~DMA_CCR_TCIE;
                USART_INT_TC_ENABLE;
            } else if(DMAChannelRx == (DMA_Channel_TypeDef*)pmsg->data2){
                DMAChannelRx->CCR = 0;
                StopTimeout();
                PacoteLocalRx->Length = PacketSize;
				PacoteLocalRx->Export(PacoteDisponivelRx);
				if(parent_rx){
					PacketNotification(PacoteDisponivelRx->Buffer, PacoteDisponivelRx->Length);
				} else {
					if(OnReadCompleted != NULL){ OnReadCompleted(PacoteDisponivelRx->Buffer, PacoteDisponivelRx->Length);}
				}
				flagReading = false;
				RestartPacket();
            }
            break;

        case NM_DMA_ERR:
            pmsg->message = NM_NULL;
            if(DMAChannelTx == (DMA_Channel_TypeDef*)pmsg->data2){
            	DMAChannelTx->CCR = 0;
				flagTransmitting = false; flagWriting = false;
				if(OnLeaveTransmission != NULL){ OnLeaveTransmission();}
                if(OnError != NULL){ OnError((uint16_t)USART_ERROR_FLAGS);}
            } else if(DMAChannelRx == (DMA_Channel_TypeDef*)pmsg->data2){
            	DMAChannelRx->CCR = 0;
                StopTimeout();
				flagReading = false;
                RestartPacket();
                if(OnError != NULL){ OnError((uint16_t)USART_ERROR_FLAGS);}
            }
            break;

        case NM_TIMETICK:
		  	if(!parent_tx){ pmsg->message = NM_NULL;}
            if(flagReceiving){
                if(++receiving_counter >= reception_timeout){
					flagReading = false;
    	            RestartPacket();
                    USART_INT_RXNE_ENABLE;
					if(parent_rx){
						DiscardNotification(PacoteDisponivelRx->Buffer, PacoteDisponivelRx->Length);
					} else {
					  	if(OnTimeout != NULL){ OnTimeout();}
                    }
					PacoteDisponivelRx->Reset();
                }
            } else if(flagTransmitting){
                if(transmitting_counter > 0){ transmitting_counter--;}
				else {
					flagTransmitting = false; flagWriting = false;
					if(OnLeaveTransmission != NULL){ OnLeaveTransmission();}
				}
			}
            break;

        case NM_SYSCLOCKCHANGE_PCLK:
            SetBaudRate(baud);
            pmsg->message = NM_NULL;
            break;

        default: pmsg->message = NM_NULL; break;
    }
}

//------------------------------------------------------------------------------
void NSerial::InterruptCallBack(NMESSAGE* msg){
    Notify(msg);
}

//==============================================================================
