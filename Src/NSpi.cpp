//==============================================================================
#include "NSpi.h"
#include "DRV_CPU.h"
#include "NTinyOutput.h"

//------------------------------------------------------------------------------

	#define SPI_ENABLE              (SPIx->CR1 |= SPI_CR1_SPE)
	#define SPI_DISABLE             (SPIx->CR1 &= ~SPI_CR1_SPE)
	#define SPI_IS_ENABLED          (SPIx->CR1 & SPI_CR1_SPE)

	#define SPI_ISR_DISABLE        	(SPIx->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE))
	#define SPI_ISR_ENABLE        	(SPIx->CR2 |= (SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE))

    #define SPI_TXDMA_ENABLE        (SPIx->CR2 |= SPI_CR2_TXDMAEN)
	#define SPI_TXDMA_DISABLE       (SPIx->CR2 &= ~SPI_CR2_TXDMAEN)

	#define SPI_RXDMA_ENABLE        (SPIx->CR2 |= SPI_CR2_RXDMAEN)
    #define SPI_RXDMA_DISABLE       (SPIx->CR2 &= ~SPI_CR2_RXDMAEN)

	#define SPI_DMA_DISABLE         (SPIx->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN))
    #define SPI_CR1_CONFIG          (SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_MSTR)
    #define SPI_CR2_CONFIG          ((uint32_t) 0x0000)

	#define DATASIZE_8BITS			(SPIx->CR1 &= ~SPI_CR1_DFF)
	#define DATASIZE_16BITS			(SPIx->CR1 |= SPI_CR1_DFF)
	#define DATASIZE_IS_16BITS		((bool)(SPIx->CR1 & SPI_CR1_DFF))

    #define MAX_SPI_PACKET_SIZE  	((uint32_t) 1024)

//------------------------------------------------------------------------------
	#define PA						0x0000
	#define PB						0x0100
	#define PC						0x0200
	#define PD						0x0300
	#define PE						0x0400
	#define PF						0x0500
	#define PG						0x0600
	#define NO_PIN					0xFFFF

	#define SPI1_STANDARD			0
	#define SPI1_REMAPPED			1
	#define SPI2_STANDARD			2
	#define SPI2_REMAPPED			2
	#define SPI3_STANDARD			3
	#define SPI3_REMAPPED			4

//------------------------------------------------------------------------------
uint16_t SPI_CONFIGS[5][3]{
	// SPI1 STANDARD
	//  MOSI      MISO       SCK
	{(PA | 7), (PA | 6), (PA | 5)},
	// SPI1 REMAPPED
	//  MOSI      MISO       SCK
	{(PB | 5), (PB | 4),  (PB | 3)},

	// SPI2 STANDARD & SPI2 REMAPPED
	//  MOSI      MISO       SCK
	{(PB | 15), (PB | 14), (PB | 13)},

	// SPI3 STANDARD
	//  MOSI      MISO       SCK
	{(PB | 5), (PB | 4), (PB | 3)},
	// SPI3 REMAPPED
	//  MOSI      MISO       SCK
	{(PC | 12), (PC | 11),  (PD | 10),},
};

//------------------------------------------------------------------------------
struct SPI_Pinout{
    uint16_t mosi;
    uint16_t miso;
    uint16_t sck;
};

//------------------------------------------------------------------------------
void NSpi::Pinout(uint32_t i){
    IO_Config PinConfig;
    SPI_Pinout pinout;
    pinout.mosi  = SPI_CONFIGS[i][0];
    pinout.miso  = SPI_CONFIGS[i][1];
    pinout.sck   = SPI_CONFIGS[i][2];

    //--------------------------------------------
    PinConfig.Mode = io_Alt_PushPull_50MHz;

    PinConfig.Pin = pinout.mosi & __MASK_PORT;
    IO_PinInit(IO_PortClockOn(pinout.mosi), &PinConfig);

    //--------------------------------------------
    PinConfig.Mode = io_In_PullUp;

    PinConfig.Pin = pinout.miso & __MASK_PORT;
    IO_PinInit(IO_PortClockOn(pinout.miso), &PinConfig);

	PinConfig.Pin = pinout.sck & __MASK_PORT;
	IO_PinInit(IO_PortClockOn(pinout.sck), &PinConfig);
}

//------------------------------------------------------------------------------
NSpi::NSpi(SPI_TypeDef* SPIn, spRemapping Remap){

    uint16_t device_pinout = 0;

    SPIx = SPIn;

    OnError = NULL; OnWriteCompleted = NULL; OnReadCompleted = NULL;

    Port.setOwner(this);
    Port.get(&NSpi::GetPort);

    ClockRate.setOwner(this);
    ClockRate.get(&NSpi::GetClockRate);
	ClockRate.set(&NSpi::SetClockRate);

    ClockPolarity.setOwner(this);
    ClockPolarity.get(&NSpi::GetClockPolarity);
	ClockPolarity.set(&NSpi::SetClockPolarity);

	CPU_PeripheralClockDisable(SPIx);

    vector_index = 0; vector_dma_rx = 0; vector_dma_tx = 0;
    IRQChannelTx = 0; IRQChannelRx = 0; SPIx_IRQn = 0;

    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    if(SPIx == SPI1){
        vector_index = NV_SPI1; vector_dma_rx = NV_DMA1_CH2; vector_dma_tx = NV_DMA1_CH3;
        if(Remap==spRemapped){
        	device_pinout = SPI1_REMAPPED;
        	AFIO->MAPR |= AFIO_MAPR_SPI1_REMAP;
        } else {
        	device_pinout = SPI1_STANDARD;
        	AFIO->MAPR &= ~AFIO_MAPR_SPI1_REMAP;
        }
        DMA_RxChannel = DMA1_Channel2; DMA_TxChannel = DMA1_Channel3;
        IRQChannelRx = DMA1_Channel2_IRQn; IRQChannelTx = DMA1_Channel3_IRQn; SPIx_IRQn = SPI1_IRQn;
	#ifdef SPI2

    } else if(SPIx == SPI2){
        vector_index = NV_SPI2; vector_dma_rx = NV_DMA1_CH4; vector_dma_tx = NV_DMA1_CH5;
        if(Remap==spRemapped){
        	device_pinout = SPI2_REMAPPED;
        }  else {
        	device_pinout = SPI2_STANDARD;
        }
        DMA_RxChannel = DMA1_Channel4; DMA_TxChannel = DMA1_Channel5;
        IRQChannelRx = DMA1_Channel4_IRQn; IRQChannelTx = DMA1_Channel5_IRQn; SPIx_IRQn = SPI2_IRQn;
	#endif

	#if defined(SPI3)
	} else if(SPIx == SPI3){
		vector_index = NV_SPI3; vector_dma_rx = NV_DMA2_CH1; vector_dma_tx = NV_DMA2_CH2;
		if(Remap==spRemapped){
			device_pinout = SPI3_REMAPPED;
			AFIO->MAPR |= AFIO_MAPR_SPI3_REMAP;
		} else {
			device_pinout = SPI3_STANDARD;
			AFIO->MAPR &= ~AFIO_MAPR_SPI3_REMAP;
		}
		DMA_RxChannel = DMA2_Channel1; DMA_TxChannel = DMA2_Channel2;
		IRQChannelRx = DMA2_Channel1_IRQn; IRQChannelTx = DMA2_Channel2_IRQn; SPIx_IRQn = SPI3_IRQn;
	#endif

    }

    //IO_PinConfig(device_pinout);
    Pinout(device_pinout);

    MasterTxInProgress = false;
    MasterRxInProgress = false;

    AssistedReception = false;
    AssistedTransmission = false;
    Blocking = true;
	ModeRx = spNoDMA;
    ModeTx = spNoDMA;

	Priority = nTimeCritical;

	SSR_InstallCallback((uint32_t)Handle, vector_index);

    ControlRegister1 = SPI_CR1_CONFIG;
    ControlRegister2 = SPI_CR2_CONFIG;

	ClockRate = spPCLK_Div8;
}

//------------------------------------------------------------------------------
void NSpi::Open(){

    CPU_PeripheralClockEnable(SPIx);

    SPI_ISR_DISABLE;
    SPI_DISABLE;

    SSR_InstallCallback((uint32_t)Handle, vector_index);
	
    SPIx->CR1 = ControlRegister1;
    SPIx->CR2 = ControlRegister2;
}

//------------------------------------------------------------------------------
void NSpi::Close(){
	
	while(MasterRxInProgress){}
	while(MasterTxInProgress){}

    SPI_ISR_DISABLE;
    SPI_DISABLE;
	SSR_InstallCallback((uint32_t)NULL, vector_index);

	CPU_PeripheralClockDisable(SPIx);
}

//------------------------------------------------------------------------------
SPI_TypeDef* NSpi::GetPort(){ return(SPIx);}

//------------------------------------------------------------------------------
spClockRates NSpi::GetClockRate(){
	baud = (spClockRates)((ControlRegister1 & SPI_CR1_BR_Msk)>>SPI_CR1_BR_Pos);
	return(baud);
}

//------------------------------------------------------------------------------
void NSpi::SetClockRate(spClockRates new_baud){
	baud = new_baud;
	ControlRegister1 &= (~SPI_CR1_BR_Msk);
	ControlRegister1 |= ((baud<<SPI_CR1_BR_Pos) & SPI_CR1_BR_Msk);
	if(!SPI_IS_ENABLED){ SPIx->CR1 = ControlRegister1;}
}

//------------------------------------------------------------------------------
spClockPolarities NSpi::GetClockPolarity(){
	return( (spClockPolarities)(ControlRegister1 & 3));
}

//------------------------------------------------------------------------------
void NSpi::SetClockPolarity(spClockPolarities new_pol){
	ControlRegister1 &= ~3;
	ControlRegister1 |= ((uint32_t)new_pol & 3);
	if(!SPI_IS_ENABLED){ SPIx->CR1 = ControlRegister1;}
}

//------------------------------------------------------------------------------
bool NSpi::WriteNoDMA(void* wdata, uint32_t wlen){
	__attribute__((unused)) uint16_t dummy;
    bool result = true;

	if(MasterTxInProgress || MasterRxInProgress){ return(false);}
    
	if((wlen>0L)&&(wlen <= MAX_SPI_PACKET_SIZE)){

		if(Blocking){
			MasterTxInProgress = true;
			SPI_ENABLE;

			for(uint32_t c=0; c< wlen; c++){
				while((SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY){};
				if(DATASIZE_IS_16BITS){
					*((uint16_t*)&SPIx->DR) = ((uint16_t*)wdata)[c];
				} else {
					*((uint8_t*)&SPIx->DR) = ((uint8_t*)wdata)[c]; //Keil /IAR
				}
				while((SPIx->SR & SPI_SR_TXE)==0){};
			}

			while((SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY){};
			while((SPIx->SR & SPI_SR_RXNE) == SPI_SR_RXNE){ dummy = SPIx->DR;}

			while((bool)(SPIx->SR & SPI_SR_BSY)){};
			SPI_DISABLE;
			MasterTxInProgress = false;
			if(!AssistedTransmission){
				if(OnWriteCompleted != NULL){ OnWriteCompleted();}
			}
		} else {
			// non-blocking mode (ISR)
			NVIC_SetPriority((IRQn_Type)SPIx_IRQn, NVIC_EncodePriority(NVIC_PriorityGroup_4, SYS_PRIORITY_NORMAL,0));
			NVIC_EnableIRQ((IRQn_Type)SPIx_IRQn);

			SSR_InstallCallback((uint32_t)Handle, vector_index);

			MasterTxInProgress = true;
			SPI_ENABLE;
			bytes_pointer = wdata;
			bytes_to_write = wlen;
			bytes_written = 0;

			while(((SPIx->SR & SPI_SR_TXE) == SPI_SR_TXE) && (bytes_written < bytes_to_write)){
	            if(DATASIZE_IS_16BITS){
	                *((uint16_t*)&SPIx->DR) = ((uint16_t*)bytes_pointer)[bytes_written++];
	            } else {
	                *((uint8_t*)&SPIx->DR) = ((uint8_t*)bytes_pointer)[bytes_written++];
	            }
			}
			SPIx->CR2 |= (SPI_CR2_ERRIE | SPI_CR2_TXEIE);

		}
	}
    return(result);
}

//------------------------------------------------------------------------------
bool NSpi::WriteDMA(void* ptPacket, uint32_t szPacket){
	__attribute__((unused)) uint8_t dummy;

    if(MasterTxInProgress || MasterRxInProgress){ return(false);}

	if(DMA_TxChannel != NULL){
		MasterTxInProgress = true;

		DMA_TxChannel->CCR = (DMA_CCR_DIR | DMA_CCR_MINC);
		DMA_TxChannel->CPAR = (uint32_t)(&SPIx->DR);
		DMA_TxChannel->CMAR = (uint32_t)(ptPacket);
		DMA_TxChannel->CNDTR = szPacket;

        if(DATASIZE_IS_16BITS){
        	DMA_TxChannel->CCR |= (DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0);
        }

		if(!Blocking){
			// non-blocking transfers must turn-on DMA interrupts
			SSR_InstallCallback((uint32_t)Handle, vector_dma_tx);

			DMA_TxChannel->CCR |= (DMA_CCR_TCIE | DMA_CCR_TEIE);

			NVIC_SetPriority((IRQn_Type)IRQChannelTx, NVIC_EncodePriority(NVIC_PriorityGroup_4, SYS_PRIORITY_NORMAL,0));
			NVIC_EnableIRQ((IRQn_Type)IRQChannelTx);

		}

		SPI_TXDMA_ENABLE;
		SPI_ENABLE;
		DMA_TxChannel->CCR |= DMA_CCR_EN;

		if(Blocking){
			while(!DMA_CheckInterrupts(DMA_TxChannel, DMA_ISR_TCIF1)){};

			while((SPIx->SR & SPI_SR_BSY)== SPI_SR_BSY){};

			SPI_DISABLE;
			if(DATASIZE_IS_16BITS){	*((uint16_t*)&dummy) = *((uint16_t*)&SPIx->DR);}
			else { *((uint8_t*)&dummy) = *((uint8_t*)&SPIx->DR);}

			SPI_TXDMA_DISABLE;
			SSR_InstallCallback((uint32_t)NULL, vector_dma_tx);
			MasterTxInProgress = false;
		}
	}
    return(true);
}

//------------------------------------------------------------------------------
bool NSpi::ReadNoDMA(void* rdata, uint32_t rlen){
	__attribute__((unused)) uint16_t dummy=0;
    bool result = true;
	//uint32_t i=0;

	if(MasterTxInProgress || MasterRxInProgress){ return(false);}

	bytes_pointer = rdata;
	bytes_to_write = rlen;
	bytes_written = 0;
	bytes_read = 0;

	if(Blocking){

		MasterRxInProgress = true;
		SPI_ENABLE;

		while((SPIx->SR & SPI_SR_RXNE)== SPI_SR_RXNE){
			if(DATASIZE_IS_16BITS){	*((uint16_t*)&dummy) = *((uint16_t*)&SPIx->DR);}
			else {	*((uint8_t*)&dummy) = *((uint8_t*)&SPIx->DR);}
			while((SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY){};
		}

		while(bytes_written < bytes_to_write){

			// push dummy values to pull new data from device
			if(DATASIZE_IS_16BITS){	*((uint16_t*)&SPIx->DR) = (uint16_t)0xFFFF;}
			else {	*((uint8_t*)&SPIx->DR) = (uint8_t)0xFF;}
			bytes_written++;

			while((SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY){};

			while((SPIx->SR & SPI_SR_RXNE)==0){}
			if(DATASIZE_IS_16BITS){ ((uint16_t*)bytes_pointer)[bytes_read] = *((uint16_t*)&SPIx->DR);}
			else { ((uint8_t*)bytes_pointer)[bytes_read] = *((uint8_t*)&SPIx->DR);}
			bytes_read++;

			while((SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY){};
		}
		
		while((SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY);
		SPI_DISABLE;
		MasterRxInProgress = false;
		if(!AssistedReception){
			if(OnReadCompleted != NULL){ OnReadCompleted();}
		}
	} else { // non-blocking (ISR)

		if((rlen > 0) && (rlen <= MAX_SPI_PACKET_SIZE)){

			NVIC_SetPriority((IRQn_Type)SPIx_IRQn, NVIC_EncodePriority(NVIC_PriorityGroup_4, SYS_PRIORITY_NORMAL,0));
			NVIC_EnableIRQ((IRQn_Type)SPIx_IRQn);

			SSR_InstallCallback((uint32_t)Handle, vector_index);

			MasterRxInProgress = true;
			SPI_ENABLE;

			while(!(bool)(SPIx->SR & SPI_SR_TXE)){};

			if(DATASIZE_IS_16BITS){ *((uint16_t*)&SPIx->DR) = ((uint16_t)0xFFFF);}
			else { *((uint8_t*)&SPIx->DR) = ((uint8_t)0xFF);}
			bytes_written++;
			result = true;
			SPIx->CR2 |= (SPI_CR2_ERRIE | SPI_CR2_TXEIE); // | SPI_CR2_RXNEIE);
		}
		
	}
    return(result);
}

//------------------------------------------------------------------------------
bool NSpi::ReadDMA(void* ptPacket, uint32_t szPacket){
	__attribute__((unused)) static uint16_t dummy = 0xFFFF;
	
    if(MasterTxInProgress || MasterRxInProgress){ return(false);}

    // Rx channel setup
	DMA_ClearInterrupts(DMA_RxChannel, DMA_IFCR_ALL);
	DMA_RxChannel->CCR = DMA_CCR_MINC;
	DMA_RxChannel->CPAR = (uint32_t)(&SPIx->DR);
	DMA_RxChannel->CMAR = (uint32_t)(ptPacket);
	DMA_RxChannel->CNDTR = szPacket;
    if(DATASIZE_IS_16BITS){	DMA_RxChannel->CCR |= (DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0);}
	MasterRxInProgress = true;

    // Tx channel setup
	DMA_ClearInterrupts(DMA_TxChannel, DMA_IFCR_ALL);
	DMA_EnableInterrupts(DMA_TxChannel, DMA_IFCR_ALL);
	DMA_TxChannel->CCR = (DMA_CCR_DIR);
	DMA_TxChannel->CPAR = (uint32_t)(&SPIx->DR);
	DMA_TxChannel->CMAR = (uint32_t)(&dummy);
	DMA_TxChannel->CNDTR = szPacket;
	if(DATASIZE_IS_16BITS){	DMA_TxChannel->CCR |= (DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0);}

	DMA_RxChannel->CCR |= DMA_CCR_EN;
	if(!Blocking){
		SSR_InstallCallback((uint32_t)Handle, vector_dma_tx);

		NVIC_SetPriority((IRQn_Type)IRQChannelTx, NVIC_EncodePriority(NVIC_PriorityGroup_4, SYS_PRIORITY_NORMAL,0));
		NVIC_EnableIRQ((IRQn_Type)IRQChannelTx);

		// non-blocking transfers must turn-on DMA interrupts
		DMA_TxChannel->CCR |= (DMA_CCR_TCIE | DMA_CCR_TEIE);
	}

	SPI_TXDMA_ENABLE;
	SPI_RXDMA_ENABLE;
	SPI_ENABLE;
	DMA_TxChannel->CCR |= DMA_CCR_EN;

	if(Blocking){

		while(!DMA_CheckInterrupts(DMA_TxChannel, DMA_ISR_TCIF1)){};
		while((SPIx->SR & SPI_SR_TXE) != SPI_SR_TXE){}
		while((SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY){};

		SPI_DISABLE;
		//if(DATASIZE_IS_16BITS){ *((uint16_t*)&dummy) = *((uint16_t*)&SPIx->DR);}
		//else { *((uint8_t*)&dummy) = *((uint8_t*)&SPIx->DR);}
		//while((SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY){};

		SPI_TXDMA_DISABLE;
		SPI_RXDMA_DISABLE;
		MasterRxInProgress = false;
	}
    return(true);
}

//------------------------------------------------------------------------------
bool NSpi::Write(void* ptOutPacket, uint32_t szOutPacket){
	bool result = false;
	if(ModeTx == spDMA){
		result = WriteDMA(ptOutPacket, szOutPacket);
	} else if(ModeTx == spNoDMA){
		result = WriteNoDMA(ptOutPacket, szOutPacket);
	}
	return(result);
}

//------------------------------------------------------------------------------
bool NSpi::Write(uint8_t* ptOutPacket, uint32_t szOutPacket){
	DATASIZE_8BITS;
    return(Write((void*)ptOutPacket, szOutPacket));
}

//------------------------------------------------------------------------------
bool NSpi::Write(uint16_t* ptOutPacket, uint32_t szOutPacket){
	DATASIZE_16BITS;
    return(Write((void*)ptOutPacket, szOutPacket));
}

//------------------------------------------------------------------------------
bool NSpi::Read(void* ptInPacket, uint32_t szInPacket){
	bool result;
	if(ModeRx == spDMA){
		result = ReadDMA(ptInPacket, szInPacket);
	} else if(ModeRx == spNoDMA){
		result = ReadNoDMA(ptInPacket, szInPacket);
	}
	return(result);
}

//------------------------------------------------------------------------------
bool NSpi::Read(uint8_t* ptInPacket, uint32_t szInPacket){
	DATASIZE_8BITS;
    return(Read((void*)ptInPacket, szInPacket));
}

//------------------------------------------------------------------------------
bool NSpi::Read(uint16_t* ptInPacket, uint32_t szInPacket){
	DATASIZE_16BITS;
    return(Read((void*)ptInPacket, szInPacket));
}

//------------------------------------------------------------------------------
void NSpi::InterruptCallBack(NMESSAGE* pmsg){
    __attribute__((unused)) uint16_t dummy;
	
    switch(pmsg->message){
		case NM_SPIMRXNE:
			pmsg->message = NM_NULL;
			break;
		
		case NM_SPIMTXE:
            pmsg->message = NM_NULL;
            if(MasterTxInProgress){
				if(bytes_written < bytes_to_write){

//					while(bytes_written < bytes_to_write){
						dummy = SPIx->DR;
						while((SPIx->SR & SPI_SR_BSY)== SPI_SR_BSY){}

						if(DATASIZE_IS_16BITS){
							*((uint16_t*)&SPIx->DR) = ((uint16_t*)bytes_pointer)[bytes_written];
						} else {
							*((uint8_t*)&SPIx->DR) = ((uint8_t*)bytes_pointer)[bytes_written];
						}
						bytes_written++;
	//				}
				} else {
					dummy = SPIx->DR;
					while((SPIx->SR & SPI_SR_BSY)== SPI_SR_BSY){};
					SPIx->CR2 &= ~(SPI_CR2_ERRIE | SPI_CR2_TXEIE);
					NVIC_DisableIRQ((IRQn_Type)SPIx_IRQn);
					SPI_DISABLE;

					MasterTxInProgress = false;
					if(!AssistedTransmission){
						if(OnWriteCompleted != NULL){ OnWriteCompleted();}
					}
				}
			} else if(MasterRxInProgress){
				while((SPIx->SR & SPI_SR_TXE) != SPI_SR_TXE){}
				while((SPIx->SR & SPI_SR_RXNE) != SPI_SR_RXNE){}
				while((SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY){};
				if(DATASIZE_IS_16BITS){ ((uint16_t*)bytes_pointer)[bytes_read++] = *((uint16_t*)&SPIx->DR);}
				else { ((uint8_t*)bytes_pointer)[bytes_read++] = *((uint8_t*)&SPIx->DR);}

				if(bytes_written < bytes_to_write){
					if(DATASIZE_IS_16BITS){ *((uint16_t*)&SPIx->DR) = ((uint16_t)0xFFFF);}
					else { *((uint8_t*)&SPIx->DR) = ((uint8_t)0xFF);}
					bytes_written++;
				} else {
					while((SPIx->SR & SPI_SR_BSY)== SPI_SR_BSY){};
					NVIC_DisableIRQ((IRQn_Type)SPIx_IRQn);
					SPIx->CR2 &= ~(SPI_CR2_ERRIE | SPI_CR2_TXEIE);
					SPI_DISABLE;
					MasterRxInProgress = false;
					if(!AssistedReception){	if(OnReadCompleted != NULL){ OnReadCompleted();}}
				}
			}
			break;

        case NM_SPIERROR:
            pmsg->message = NM_NULL;
			NVIC_DisableIRQ((IRQn_Type)SPIx_IRQn);
			SPIx->CR2 &= ~(SPI_CR2_ERRIE | SPI_CR2_TXEIE);
			SPI_DISABLE;
            MasterTxInProgress = false;
            MasterRxInProgress = false;
            if(OnError != NULL) OnError(pmsg->data2);
            break;

        case NM_DMA_OK:
            pmsg->message = NM_NULL;
            if(pmsg->data2 == (unsigned int)DMA_TxChannel){

				while((SPIx->SR & SPI_SR_BSY)== SPI_SR_BSY){};
        		while((SPIx->SR & SPI_SR_TXE) != SPI_SR_TXE){}

				SPI_DISABLE;
				SPI_TXDMA_DISABLE;
				SPI_RXDMA_DISABLE;
				SPI_DISABLE;

				//---------------------------------------
                if(MasterTxInProgress){
    				NVIC_DisableIRQ((IRQn_Type)IRQChannelTx);
					MasterTxInProgress = false;
					if(!AssistedTransmission){
						if(OnWriteCompleted != NULL){ OnWriteCompleted();}
					}
				}

				//---------------------------------------
                if(MasterRxInProgress){
    				NVIC_DisableIRQ((IRQn_Type)IRQChannelRx);
					MasterRxInProgress = false;
					if(!AssistedReception){
						if(OnReadCompleted != NULL){ OnReadCompleted();}
					}
				}
			}
			break;

        case NM_DMA_ERR:
            pmsg->message = NM_NULL;
			SPI_TXDMA_DISABLE;
			SPI_RXDMA_DISABLE;
            if(pmsg->data2 == (unsigned int)DMA_TxChannel){
				NVIC_DisableIRQ((IRQn_Type)IRQChannelTx);
				MasterTxInProgress = false;
            } else if(pmsg->data2 == (unsigned int)DMA_RxChannel){
				NVIC_DisableIRQ((IRQn_Type)IRQChannelRx);
                MasterRxInProgress = false;
            }
            if(OnError != NULL) OnError(NM_DMA_ERR);
            break;

        default: pmsg->message = NM_NULL; break;
    }
}

//------------------------------------------------------------------------------
void NSpi::Notify(NMESSAGE* nmsg){
	InterruptCallBack(nmsg);
}


//==============================================================================
