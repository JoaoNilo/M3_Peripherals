//==============================================================================
// Title: EDROS: NIic - Master I2C abstraction class
// Author: Joao Nilo Rodrigues      nilo@pobox.com
//------------------------------------------------------------------------------
#include "NIic.h"
#include "DRV_CPU.h"
#include "DRV_DMA.h"
#include "DRV_SSR.h"
#include "System.h"

//------------------------------------------------------------------------------
NIic::NIic(I2C_TypeDef* I2Cn, iiRemapping Remap){
    uint16_t device_pinout = 0L;

    I2Cx = I2Cn;

    OnError = NULL; OnWriteCompleted = NULL; OnReadCompleted = NULL;

    Port.setOwner(this);
    Port.get(&NIic::GetPort);
	
    ClockRate.setOwner(this);
    ClockRate.get(&NIic::GetClockRate);
	ClockRate.set(&NIic::SetClockRate);
	
    vector_index = 0;
    GPIO_TypeDef* IOport;
	IOport = GPIOB;
    IO_Config IOpin;
    IOpin.Mode = io_Alt_OpenDrain_50MHz;
    //---------------------------------------
    if(I2Cx == I2C1){
        if(Remap == iiRemapped){
        	IOpin.Pin = 8; // SCL
        	IO_PinInit(IOport, &IOpin);
        	IOpin.Pin = 9; // SDA
        	IO_PinInit(IOport, &IOpin);
        } else {
        	IOpin.Pin = 6; // SCL
        	IO_PinInit(IOport, &IOpin);
        	IOpin.Pin = 7; // SDA
        	IO_PinInit(IOport, &IOpin);
        }
        DMA_TxChannel = DMA1_Channel6;
        DMA_RxChannel = DMA1_Channel7;
		vector_index = NV_I2C1;
	#ifdef I2C2
    } else if(I2Cx == I2C2){
    	IOpin.Pin = 10; // SCL
    	IO_PinInit(IOport, &IOpin);
    	IOpin.Pin = 11; // SDA
    	IO_PinInit(IOport, &IOpin);
        DMA_TxChannel = DMA1_Channel4;
        DMA_RxChannel = DMA1_Channel5;
		vector_index = NV_I2C2;
	#endif
    }

    //---------------------------------------
    IO_PinConfig(device_pinout);
	
    MasterTxInProgress = false;
    MasterRxInProgress = false;
	MasterTxRxInProgress = false;
	
    AssistedReception = false;
    AssistedTransmission = false;
    Blocking = true;
	EnableDMA = false;
	baud = ii100kHz;
	Priority = nTimeCritical;
	
	device_address[0]=0;
	device_address[1]=0;
	bytes_pointer = new uint8_t[255];
	disable = false;
	open = false;
}

//------------------------------------------------------------------------------
void NIic::Open(){

    //---------------------------------------
	CPU_PeripheralClockEnable(I2Cx);
	
    //---------------------------------------
    I2C_DISABLE;
    I2C_INTS_DISABLE;
    I2C_DMA_DISABLE;

	SSR_InstallCallback((uint32_t)Handle, vector_index);

	open = true;

	Reset();
	ClockRate = baud;
	//I2Cx->CR2 |= device_id;
	I2C_ENABLE;
}

//------------------------------------------------------------------------------
void NIic::Close(){
    open = false;
    
	//---------------------------------------
    I2C_INTS_DISABLE;
    I2C_DISABLE;
	SSR_InstallCallback((uint32_t)NULL, vector_index);

    //---------------------------------------
	CPU_PeripheralClockDisable(I2Cx);		
}

//------------------------------------------------------------------------------
I2C_TypeDef* NIic::GetPort(){ return(I2Cx);}

//------------------------------------------------------------------------------
iiClockRates NIic::GetClockRate(){ return(baud);}

//------------------------------------------------------------------------------
void NIic::SetClockRate(iiClockRates new_baud){
	#define MHz		((uint32_t) 1000000)
	bool status = false;
	uint32_t ccr;
	baud = new_baud;

	if(open){
		uint32_t f_SCL = 100000;
		uint32_t tr_SCL = 1000;  	// (ns) - from datasheet

		if(I2C_IS_ENABLED){ status = true;}
		I2C_DISABLE;


		I2Cx->CR2 &= ~I2C_CR2_FREQ_Msk;
		I2Cx->CCR = 0;
		I2Cx->TRISE = 0;

		uint32_t f_APB1 = CPU_GetFrequencyAPB1();
		if(baud == ii400kHz){
			f_SCL = 400000;
			tr_SCL = 300;			// (ns) - from datasheet
			I2Cx->CCR = I2C_CCR_FS;
			ccr = (f_APB1 / (f_SCL * 3 ));
		} else {
			ccr = (f_APB1 / (f_SCL * 2 ));
		}

		uint32_t t_APB1 = 10e9 / f_APB1; 	// (10 ns) for "rounding improvement"
		uint32_t trise = ((tr_SCL * 10) / t_APB1) + 1; // HERE

		I2Cx->CR2 |= ((f_APB1 / MHz) & I2C_CR2_FREQ_Msk);
		I2Cx->CCR |= ccr;
		I2Cx->TRISE = trise;

		//-------------------------------------------------
		// reference values for 400kHz @ 36Mhz APB1 (source: CubeMx)
		// I2Cx->CR2   = 36;		// APB1 (em MHz)
		// I2Cx->CCR   = 0x801E;	// 36MHz / (400kHz * 3) = 30 or 1Eh
		// I2Cx->TRISE = 0x0B;		// (1000ns / (1/36MHz)) + 1
		//-------------------------------------------------
	
		if(status){ I2C_ENABLE;}
	}
}

//------------------------------------------------------------------------------
void NIic::SetDeviceId(uint8_t slave_id){
	device_id = (uint16_t)((slave_id & 0x7F)<<1);
	
/*	I2Cx->CR2 = I2Cx->CR2 & (~(I2C_CR2_SADD_Msk |I2C_CR2_ADD10_Msk));
	I2Cx->CR2 |= device_id;*/
}

//------------------------------------------------------------------------------
uint8_t NIic::GetDeviceId(){ return((uint8_t)device_id);}

//------------------------------------------------------------------------------
void NIic::SetDeviceId10(uint16_t slave_id){
	device_id = (uint16_t)(slave_id & 0x3FF);
/*	I2Cx->CR2 = ((I2Cx->CR2 & (~(I2C_CR2_SADD_Msk))) | I2C_CR2_ADD10_Msk);
	I2Cx->CR2 |= device_id;*/
}

//------------------------------------------------------------------------------
uint16_t NIic::GetDeviceId10(){	return((uint16_t)device_id);}

//------------------------------------------------------------------------------
void NIic::Reset(){
	I2C_RESET_ON;
	I2C_RESET_OFF;
}

//------------------------------------------------------------------------------
void NIic::Start(){
	if(!open){ return;}

	I2C_SET_ACK;
	I2C_START;
	while(I2C_NOT_STARTED){ }
}

//------------------------------------------------------------------------------
void NIic::Stop(){
	I2C_STOP;
}

//------------------------------------------------------------------------------
bool NIic::Address(uint8_t addr){
	uint32_t result;
	I2C_TX_DATA = addr;

	while(I2C_NOT_ADDRESSED && I2C_NO_TIMEOUT){ }
	result = I2Cx->SR2 & 0XFF;
	return((bool)result);
}

//------------------------------------------------------------------------------
bool NIic::Write(uint8_t data){
	timeout = 10;
	while((I2Cx->SR1 & I2C_SR1_TXE)==0){ }
	I2C_TX_DATA = data;
	while(I2C_TX_NOT_COMPLETED && I2C_NO_TIMEOUT){ }
	return(true);
}

//------------------------------------------------------------------------------
bool NIic::Write(uint8_t* data, uint8_t size){

	if(!EnableDMA){
		while(size > 0){
			while((I2Cx->SR1 & I2C_SR1_TXE)==0){};
			I2Cx->DR = *data++;
			while(I2C_TX_NOT_COMPLETED){ }
			size--;
		}
		while(I2C_TX_NOT_COMPLETED){}

	} else {

		DMA_TxChannel->CCR = (DMA_CCR_DIR | DMA_CCR_MINC);
		DMA_TxChannel->CPAR = (uint32_t)(&I2C_TX_DATA);
		DMA_TxChannel->CMAR = (uint32_t)(data);
		DMA_TxChannel->CNDTR = size;

		I2C_TXDMA_ENABLE;
		DMA_TxChannel->CCR |= DMA_CCR_EN;

		//while(!DMA_CheckInterrupts(DMA_RxChannel, DMA_ISR_TCIF1)){};
		//while(!DMA_CheckInterrupts(DMA_TxChannel, DMA_ISR_TCIF1)){};
		while(I2C_TX_NOT_COMPLETED){ }
		DMA_TxChannel->CCR = 0;
	}
	return(true);
}

//------------------------------------------------------------------------------
bool NIic::Read(uint8_t deviceAddress, uint8_t* ptInPacket, uint32_t szInPacket){
	uint32_t bytes_remaining = szInPacket;
	uint32_t bytes_received = 0;

	if(bytes_remaining == 1){

		I2C_TX_DATA = deviceAddress;
		while(I2C_NOT_ADDRESSED){ }

		//I2C_CLEAR_ACK;
		I2Cx->CR1 |= I2C_CR1_ACK;
		__attribute((unused)) uint8_t dump = I2Cx->SR1 | I2Cx->SR2;
		I2C_STOP;


		while(I2C_RX_IS_EMPTY){ }
		ptInPacket[bytes_received++] = I2C_RX_DATA;

	} else {
		I2C_TX_DATA = deviceAddress;

		while(I2C_NOT_ADDRESSED){ }

		__attribute((unused)) uint8_t dump = I2Cx->SR1 | I2Cx->SR2;

		//--------------------------------------------------
		// this section CAN be done by both DMA or IRQs
		if((!EnableDMA)||(szInPacket < 5)){
			while(bytes_remaining > 2){

				while(I2C_RX_IS_EMPTY){ }
				ptInPacket[bytes_received++] = I2C_RX_DATA;
				I2C_SET_ACK;
				bytes_remaining--;
			}
		} else {
			bytes_received = (szInPacket - 2);
			DMA_RxChannel->CCR = (DMA_CCR_MINC);
			DMA_RxChannel->CPAR = (uint32_t)(&I2C_RX_DATA);
			DMA_RxChannel->CMAR = (uint32_t)(ptInPacket);
			DMA_RxChannel->CNDTR = bytes_received;

			I2C_RXDMA_ENABLE;
			DMA_RxChannel->CCR |= DMA_CCR_EN;

			//while(!DMA_CheckInterrupts(DMA_RxChannel, DMA_ISR_TCIF1)){};
			//while(I2C_TX_NOT_COMPLETED){ }
			DMA_RxChannel->CCR = 0;
		}

		while(I2C_RX_IS_EMPTY){ }
		ptInPacket[bytes_received++] = I2C_RX_DATA;
		I2C_CLEAR_ACK;
		I2C_STOP;
		bytes_remaining--;

		while(I2C_RX_IS_EMPTY){ }
		ptInPacket[bytes_received] = I2C_RX_DATA;
		//--------------------------------------------------
	}
	return(true);
}

//------------------------------------------------------------------------------
bool NIic::Write(uint8_t deviceAddress, uint8_t* ptOutPacket, uint32_t szOutPacket){
	bool result = false;
	if( Address(deviceAddress)){
		if(szOutPacket==1){	result = Write(ptOutPacket[0]);}
		else { result = Write(ptOutPacket, szOutPacket);}
	}
	return(result);
}

//------------------------------------------------------------------------------
// Msg1 rules: 		message = NM_I2CMTXE, NM_I2CMRXNE, NM_I2CERROR ou NM_NULL
//                  data1  = vector index (NV_I2Cx, NV_DMAx_CHy)
//                  data2  = I2Cx->ISR, I2Cx->DR, etc...
//                  tag     = not used
//------------------------------------------------------------------------------
void NIic::InterruptCallBack(NMESSAGE* pmsg){
	
    switch(pmsg->message){
        case NM_I2CERROR:
            break;

		case NM_I2CMTXE:
			break;

		case NM_I2CMTC:
			break;
		
		case NM_I2CMRXNE:
			break;

        case NM_DMA_OK:
			break;

        case NM_DMA_ERR:
			break;

        default:
        	break;
    }
    pmsg->message = NM_NULL;
}

//------------------------------------------------------------------------------
void NIic::Notify(NMESSAGE* msg){
	msg->message = NM_NULL;
	/*if(msg->message == NM_TIMETICK){
		if((open) && (timeout > 0)){ timeout--;}
		msg->message = NM_NULL;
	} else if(msg->message == NM_SYSCLOCKCHANGE_PCLK){
        SetClockRate(baud);
        msg->message = NM_NULL;
    } else { 
        //InterruptCallBack(msg);
    	msg->message = NM_NULL;
    }*/
}

//==============================================================================
