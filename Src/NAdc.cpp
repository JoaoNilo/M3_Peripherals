//============================================================================//
#include "NAdc.h"
#include "DRV_CPU.h"
#include "DRV_IO.h"

#include "DRV_SSR.h"

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void NAdc::SetSampleTime(adChannels chan, uint8_t new_sample_time){
	uint32_t tmpreg1 = 0;
	uint32_t tmpreg2 = 0;

	uint16_t ch = ( ((uint16_t)chan >> 5) | ((uint16_t)chan & 0x1F) ) & 0x1F;

	if (chan > adCH9){
		tmpreg1 = ADCx->SMPR1;
		tmpreg2 = ADC_SMPR1_SMP10_Msk << (3 * (ch - 10));
		tmpreg1 &= ~tmpreg2;
		tmpreg2 = (uint32_t)new_sample_time << (3 * (ch - 10));
		tmpreg1 |= tmpreg2;
		ADCx->SMPR1 = tmpreg1;
	} else {
		tmpreg1 = ADCx->SMPR2;
		tmpreg2 = ADC_SMPR2_SMP0_Msk << (3 * ch);
		tmpreg1 &= ~tmpreg2;
		tmpreg2 = (uint32_t)new_sample_time << (3 * ch);
		tmpreg1 |= tmpreg2;
		ADCx->SMPR2 = tmpreg1;
	}
}

//------------------------------------------------------------------------------
void NAdc::IncludeRegularChannel(uint8_t channels_count, adChannels inc_ch){
	uint32_t tmpreg1 = 0;
	uint32_t tmpreg2 = 0;

	uint16_t ch = ( ((uint16_t)inc_ch >> 5) | ((uint16_t)inc_ch & 0x1F) ) & 0x1F;

	if(channels_count < 7){
		tmpreg1 = ADCx->SQR3;
		tmpreg2 = ADC_SQR3_SQ1 << (5 * (channels_count - 1));
		tmpreg1 &= ~tmpreg2;
		tmpreg2 = (uint32_t)ch << (5 * (channels_count - 1));
		tmpreg1 |= tmpreg2;
		ADCx->SQR3 = tmpreg1;
	} else if (channels_count < 13){
		tmpreg1 = ADCx->SQR2;
		tmpreg2 = ADC_SQR2_SQ7 << (5 * (channels_count - 7));
		tmpreg1 &= ~tmpreg2;
		tmpreg2 = (uint32_t)ch << (5 * (channels_count - 7));
		tmpreg1 |= tmpreg2;
		ADCx->SQR2 = tmpreg1;
	} else {
		tmpreg1 = ADCx->SQR1;
		tmpreg2 = ADC_SQR1_SQ13 << (5 * (channels_count - 13));
		tmpreg1 &= ~tmpreg2;
		tmpreg2 = (uint32_t)ch << (5 * (channels_count - 13));
		tmpreg1 |= tmpreg2;
		ADCx->SQR1 = tmpreg1;
	}
}

//------------------------------------------------------------------------------
bool NAdc::AddChannel(adChannels new_channel){
    GPIO_TypeDef* channel_port;
    IO_Config PinConfig;

    if(activechannels < 16){
        if(new_channel >= (uint16_t)adVref) return(false);
        channel = new_channel; activechannels++;

        if(channel == (uint16_t)adThrm){
            ADCx->CR2 |= ADC_CR2_TSVREFE;
        }

		//uint32_t ch = Channels[newchannel];
		channel_port = IO_GetPort(channel);
		PinConfig.Pin = channel & 0x000F;
		PinConfig.Mode = io_In_Analog;
		IO_PinInit(channel_port, &PinConfig);
		SetSampleTime(channel, ADC_SampleTime_13p5Cycles);
		IncludeRegularChannel(activechannels, channel);
		//ADC_RegularChannelConfig(ADCx, newchannel, ++activechannels, ADC_SampleTime_13Cycles5);

		ADCx->CR1 &= ~ADC_CR1_SCAN;
		ADCx->SQR1 &= ~ADC_SQR1_L_Msk;
		ADCx->SQR1 |= ((uint32_t)(activechannels-1) & 0x0000000F) << ADC_SQR1_L_Pos;

        if(activechannels > 1){
            channel = adNoCH;
            ADCx->CR1 |= ADC_CR1_SCAN;
        }
        return(true);
    } else return(false);
}

//------------------------------------------------------------------------------
void NAdc::FlushChannels(){
    ADCx->SQR1 = 0; ADCx->SQR2 = 0; ADCx->SQR3 = 0;
    ADCx->CR2 &= ~ADC_CR2_TSVREFE;
    ADCx->CR1 &= ~ADC_CR1_SCAN;
    activechannels = 0; channel = adNoCH;
}

//------------------------------------------------------------------------------
void NAdc::SetTimebase(){
    bool stat = false;

    if((mode==adSingle) || (mode==adContinuous1)){
    	ADC_EXTERNAL_TRIGGER_OFF;
        return;
    }

    if(ADCx->CR2 & ADC_CR2_ADON){ stat = true;}
    ADCx->CR2 &= ~ADC_CR2_ADON;
	ADCx->CR2 &= ~ADC_CR2_EXTSEL;

    if((ADCx == ADC1)||(ADCx == ADC2)){
		#if defined(TIM4)
		ADCx->CR2 |= (ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_0); 	// TIM4 CC4
		#else
		ADCx->CR2 |= (ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_2); 						// TIM3 TRGO
		#endif
    #if defined(ADC3)
    } else if(ADCx == ADC3){
    	ADCx->CR2 |= (ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_0); // TIM5 CC1
    #endif
    } else return;

    //----------------------------------------------
    if(samplingrate < 10){ samplingrate = 10;}
    CPU_PeripheralClockEnable(TIMx);
    TIMx->CR1 = 0; TIMx->CR2 = 0;
    TIMx->ARR = samplingrate - 1;
    //TIMx->PSC = (CPU_GetTimerFrequencyAPB1()/1000000) - 1;
    TIMx->PSC = (CPU_GetFrequencyAPB1()/1000000) - 1;

    //----------------------------------------------
    TIMx->CCER =0;
    if((ADCx == ADC1) || (ADCx == ADC2)){
        TIMx->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0);
        TIMx->CCER |= TIM_CCER_CC4E;
        TIMx->CCR4 = samplingrate-1;
	#if defined(ADC3)
    } else if(ADCx == ADC3){
        TIMx->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
    	TIMx->CCER |= TIM_CCER_CC1E;
        TIMx->CCR1 = samplingrate-1;
		#endif
    }

    //----------------------------------------------
    if(stat){
    	TIMx->CR1 |= TIM_CR1_CEN;
    	ADCx->CR2 |= ADC_CR2_ADON;
    }
}

//------------------------------------------------------------------------------
void NAdc::SetDataBuffer(unsigned short* ptBuffer, unsigned short szBuffer){
	uint32_t dma_vector;
	uint32_t dma_irqn;

    if((uint32_t)ADCx == (uint32_t)ADC2) return;

    if(mode == adContinuous3){
        if(szBuffer&0x01) szBuffer--;
        if((ptBuffer==NULL)||(szBuffer<2)) return;

        buffer[0].data = ptBuffer;
        buffer[0].size = szBuffer/2;
        buffer[1].data = ptBuffer + buffer[0].size;
        buffer[1].size = szBuffer/2;

        //--------------------------------------
        if(DMA_AdcChannel == NULL) return;

        if((uint32_t)ADCx == (uint32_t)ADC1){
        	dma_irqn = DMA1_Channel1_IRQn;
        	dma_vector = NV_DMA1_CH1;
        	CPU_PeripheralClockEnable(DMA1);
        } else {
			#if defined(ADC3)
        	dma_irqn = DMA2_Channel5_IRQn;
        	dma_vector = NV_DMA2_CH5;
        	CPU_PeripheralClockEnable(DMA2);
			#endif
        }

        SSR_InstallCallback((uint32_t)Handle, dma_vector);
        uint32_t pri = NVIC_EncodePriority((IRQn_Type)dma_irqn, SYS_PRIORITY_NORMAL, 0);
    	NVIC_SetPriority((IRQn_Type)dma_irqn, pri);
    	NVIC_EnableIRQ((IRQn_Type)dma_irqn);

        ADCx->CR2 |= ADC_CR2_DMA;

        DMA_AdcChannel->CCR = (DMA_CCR_CIRC | DMA_CCR_MINC);
        DMA_AdcChannel->CCR |= (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0);
        DMA_AdcChannel->CMAR = (uint32_t)ptBuffer;
        DMA_AdcChannel->CPAR = (uint32_t)&ADCx->DR;
        DMA_AdcChannel->CNDTR = szBuffer;

	    if(ADCx==ADC1){ DMA1->ISR &= ~DMA_AdcChannelFlags;}
		#if defined(ADC3)
	    else { DMA2->ISR &= ~DMA_AdcChannelFlags;}
		#endif

	    DMA_AdcChannel->CCR |= (DMA_CCR_TEIE | DMA_CCR_HTIE | DMA_CCR_TCIE);

	    DMA_AdcChannel->CCR |= DMA_CCR_EN;
    }
}

//------------------------------------------------------------------------------
NAdc::NAdc(ADC_TypeDef* AD){
	 uint32_t pri = 0;

    OnData = NULL; OnDataBlock = NULL;
    OnUpperThreshold = NULL; OnLowerThreshold = NULL;
    buffer[0].data = NULL; buffer[1].data = NULL;
    buffer[0].size = 0; buffer[1].size = 0;

    Mode.setOwner(this);
    Mode.set(&NAdc::SetMode);
    Mode.get(&NAdc::GetMode);

    Alignment.setOwner(this);
    Alignment.set(&NAdc::SetAlignment);
    Alignment.get(&NAdc::GetAlignment);

    Thresholds.setOwner(this);
    Thresholds.set(&NAdc::SetThresholds);
    Thresholds.get(&NAdc::GetThresholds);

    LowerLimit.setOwner(this);
    LowerLimit.set(&NAdc::SetLowerLimit);
    LowerLimit.get(&NAdc::GetLowerLimit);

    UpperLimit.setOwner(this);
    UpperLimit.set(&NAdc::SetUpperLimit);
    UpperLimit.get(&NAdc::GetUpperLimit);

    Channel.setOwner(this);
    Channel.set(&NAdc::SetChannel);
    Channel.get(&NAdc::GetChannel);

    ADCx = AD;

    RCC->CFGR &= ~RCC_CFGR_ADCPRE_Msk;
    uint32_t pclk2 = CPU_GetFrequencyAPB2();
    if(pclk2 >= 56000000){
    	RCC->CFGR |= (RCC_CFGR_ADCPRE_1); // PCLK2/6
	} else if(pclk2 >= 28000000){
		RCC->CFGR |= (RCC_CFGR_ADCPRE_0); // PCLK2/4
	}

    if(ADCx == ADC1){
		#if defined(TIM4)
    	TIMx = TIM4;
		#else
    	TIMx = TIM3;
		#endif
        CPU_PeripheralClockEnable(ADC1);
        vector_index = NV_ADC1;
        ADCx_IRQn = ADC1_2_IRQn;
        DMA_AdcChannel = DMA1_Channel1;
        DMA_AdcChannelFlags = (DMA_ISR_TEIF1 | DMA_ISR_HTIF1 | DMA_ISR_TCIF1);
    } else if( ADCx == ADC2){
		#if defined(TIM4)
		TIMx = TIM4;
		#else
		TIMx = TIM3;
		#endif
    	CPU_PeripheralClockEnable(ADC2);
        vector_index = NV_ADC2;
        ADCx_IRQn = ADC1_2_IRQn;
        DMA_AdcChannel = (DMA_Channel_TypeDef *)NULL;
        DMA_AdcChannelFlags = 0;
    #if defined(ADC3)
    } else  if( ADCx == ADC3){
    	TIMx = TIM5;
        CPU_PeripheralClockEnable(ADC3);
        vector_index = NV_ADC3;
        ADCx_IRQn = ADC3_IRQn;
        DMA_AdcChannel = DMA2_Channel5;
        DMA_AdcChannelFlags = (DMA_ISR_TEIF5 | DMA_ISR_HTIF5 | DMA_ISR_TCIF5);
    #endif
    } else { return;}

    //---------------------------
    SSR_InstallCallback((uint32_t)Handle, vector_index);
    pri = NVIC_EncodePriority(ADCx_IRQn, SYS_PRIORITY_NORMAL, 0);
	NVIC_SetPriority(ADCx_IRQn, pri);
	NVIC_EnableIRQ(ADCx_IRQn);

    //---------------------------
    status = adStopped;
    mode = (adModes)10;
    activechannels = 0;
    samplingrate = 1000;
    alignment = adRight;
    lowerlimit = 0x0000;
    upperlimit = 0x0FFF;
    threshold = adNone;
    channel = adNoCH;

    //---------------------------
    convertion_counter = 0L;

    //---------------------------
    ADCx->CR1 = 0; ADCx->CR2 = 0;

    SetMode(adSingle);
}

//------------------------------------------------------------------------------
void NAdc::SetMode(adModes new_mode){
    if(new_mode == mode) return;

    if((ADCx == ADC2) && (new_mode == adContinuous3)){
        //#pragma message("ERROR: Invalid Mode for ADC2!")
        return;
    }

    Stop();

    ADCx->SR &= ~(ADC_SR_EOC | ADC_SR_AWD);
    ADCx->CR1 |= (ADC_CR1_EOCIE | ADC_CR1_AWDIE);

    if((ADCx != ADC2) && (mode == adContinuous3)){
        DMA_AdcChannel->CCR &= ~DMA_CCR_EN;
        DMA_AdcChannel->CCR &= ~DMA_AdcChannelFlags;
    }

    mode = new_mode;
    switch(mode){
        case adSingle:
        case adContinuous1:
            ADC_EXTERNAL_TRIGGER_OFF;
			Priority = nRelaxed;
            break;

        case adContinuous2:
			Priority = nNormal;
            break;

        case adContinuous3:
            if(adc != 2){
				Priority = nNormal;
			    ADCx->CR1 &= ~(ADC_CR1_EOCIE);

			    if(ADCx==ADC1){ DMA1->ISR &= ~DMA_AdcChannelFlags;}
				#if defined(ADC3)
			    else { DMA2->ISR &= ~DMA_AdcChannelFlags;}
				#endif

			    DMA_AdcChannel->CCR |= (DMA_CCR_TEIE | DMA_CCR_HTIE | DMA_CCR_TCIE);

                DMA_AdcChannel->CCR |= DMA_CCR_EN;

            } else mode = adContinuous2;
            break;
    }
    SetTimebase();
}

//------------------------------------------------------------------------------
adModes NAdc::GetMode(){ return(mode);}

//------------------------------------------------------------------------------
void NAdc::SetAlignment(adAlignments new_alignment){
    alignment = new_alignment;
    if(alignment == adRight){ ADCx->CR2 &= ~ADC_CR2_ALIGN;}
    else { ADCx->CR2 |= ADC_CR2_ALIGN;}
}

//------------------------------------------------------------------------------
adAlignments NAdc::GetAlignment(){ return(alignment);}

//------------------------------------------------------------------------------
void NAdc::SetThresholds(adThresholds new_threshold){
    threshold = new_threshold;
    switch(threshold){
        case adNone:
            ADCx->CR1 &= ~(ADC_CR1_AWDEN | ADC_CR1_JAWDEN | ADC_CR1_AWDSGL);
            ADCx->LTR = ADC_THRESHOLD_MIN; ADCx->HTR = ADC_THRESHOLD_MAX;
            break;
        case adLower:
            ADCx->CR1 &= ~(ADC_CR1_AWDEN | ADC_CR1_JAWDEN | ADC_CR1_AWDSGL);
            ADCx->CR1 |= ADC_CR1_AWDEN;
            ADCx->LTR = lowerlimit; ADCx->HTR = ADC_THRESHOLD_MAX;
            break;
        case adUpper:
            ADCx->CR1 &= ~(ADC_CR1_AWDEN | ADC_CR1_JAWDEN | ADC_CR1_AWDSGL);
            ADCx->CR1 |= ADC_CR1_AWDEN;
            ADCx->LTR = ADC_THRESHOLD_MIN; ADCx->HTR = upperlimit;
            break;
        case adBoth:
            ADCx->CR1 &= ~(ADC_CR1_AWDEN | ADC_CR1_JAWDEN | ADC_CR1_AWDSGL);
            ADCx->CR1 |= ADC_CR1_AWDEN;
            ADCx->LTR = lowerlimit; ADCx->HTR = upperlimit;
            break;
    }
}

//------------------------------------------------------------------------------
adThresholds NAdc::GetThresholds(){
    return(threshold);
}

//------------------------------------------------------------------------------
void NAdc::SetLowerLimit(unsigned short newlower){
    newlower &= 0x0FFF;
    if(newlower < upperlimit){
        lowerlimit = newlower;
        if((threshold == adBoth)||(threshold==adLower)){
            ADCx->LTR = lowerlimit;
        }
    }
}

//------------------------------------------------------------------------------
unsigned short NAdc::GetLowerLimit(){ return(lowerlimit);}

//------------------------------------------------------------------------------
void NAdc::SetUpperLimit(unsigned short newupper){
    newupper &= 0x0FFF;
    if(newupper > lowerlimit){
        upperlimit = newupper;
        if((threshold == adBoth)||(threshold==adUpper)){
            ADCx->HTR = upperlimit;
        }
    }
}

//------------------------------------------------------------------------------
unsigned short NAdc::GetUpperLimit(){ return(upperlimit);}

//------------------------------------------------------------------------------
void NAdc::SetChannel(adChannels nChannel){
    FlushChannels();
    AddChannel(nChannel);
}

//------------------------------------------------------------------------------
adChannels NAdc::GetChannel(){ return(channel);}

//------------------------------------------------------------------------------
void NAdc::Start(){
    if(activechannels==0){ return;}
	if((ADCx->CR2 & ADC_CR2_ADON)==false){
		ADCx->CR2 |= ADC_CR2_ADON;
		ADCx->CR2 |= ADC_CR2_CAL;
		while(ADCx->CR2 & ADC_CR2_CAL){}
	}
    if(mode == adSingle){
    	ADCx->CR2 |= ADC_CR2_ADON;
    	ADCx->CR2 |= ADC_CR2_SWSTART;
    }
}

//------------------------------------------------------------------------------
void NAdc::Start(uint16_t Tsampling){
    if(activechannels==0){ return;}
	ADCx->CR2 |= ADC_CR2_ADON;
	ADCx->CR2 |= ADC_CR2_CAL;
	while(ADCx->CR2 & ADC_CR2_CAL){}
    switch(mode){
        case adSingle: Start(); return;
        case adContinuous1:
        	samplingrate = Tsampling;
            if(status==adStopped){
                status=adSampling1;
            	ADCx->CR2 |= ADC_CR2_ADON;
            	ADCx->CR2 |= ADC_CR2_SWSTART;
            }
            break;
        case adContinuous2:
            if(status==adStopped){
                samplingrate = Tsampling;
                SetTimebase();
                TIMx->CR1 |= TIM_CR1_CEN;
                ADCx->CR2 |= ADC_CR2_ADON;
                status=adSampling2;
            }
            break;
        case adContinuous3:
            if(status==adStopped){
                samplingrate = Tsampling;
                SetTimebase();
                DMA_AdcChannel->CCR |= DMA_CCR_EN;
                TIMx->CR1 |= TIM_CR1_CEN;
                ADCx->CR2 |= ADC_CR2_ADON;
                status=adSampling3;
            }
            break;
    }
}

//------------------------------------------------------------------------------
void NAdc::Stop(){
    switch(mode){
        case adSingle:
        	ADCx->CR2 &= ~ADC_CR2_ADON;
        	return;
        case adContinuous1: break;
        case adContinuous2:
        case adContinuous3:
        	DMA_AdcChannel->CCR &= ~DMA_CCR_EN;
        	ADCx->CR2 &= ~ADC_CR2_ADON;
        	TIMx->CR1 &= ~TIM_CR1_CEN;
            break;
    }
    status = adStopped;
}

//------------------------------------------------------------------------------
void NAdc::Notify(NMESSAGE* msg){

    switch(msg->message){

        case NM_DMA_OK:
        	if((HANDLE)SSR_GetCallback(msg->data1) == Handle){
				if(mode==adContinuous3) {
					if(OnDataBlock !=NULL) OnDataBlock(buffer[1L].data, buffer[1L].size);
				}
        	}
            break;

        case NM_DMA_MOK:
        	if((HANDLE)SSR_GetCallback(msg->data1) == Handle){
				if(mode==adContinuous3){
					if(OnDataBlock !=NULL) OnDataBlock(buffer[0L].data, buffer[0L].size);
				}
        	}
			break;

        case NM_ADCEOC:
        	if(msg->data1 == vector_index){
        		if(OnData != NULL) OnData(ADCx->DR);
        	}
            break;

        case NM_ADCAWD:
        	if(msg->data1 == vector_index){
				if((msg->data2 > upperlimit)&&(OnUpperThreshold != NULL)){
					OnUpperThreshold(msg->data2);
				} else if((msg->data2 < lowerlimit)&&(OnLowerThreshold != NULL)){
						OnLowerThreshold(msg->data2);
				}
        	}
            break;

        case NM_DMA_ERR:
            break;

        case NM_ADCJEOC:
            break;

        case NM_TIMETICK:
            if((mode == adContinuous1)&&(status==adSampling1)){
            	if(convertion_counter>0L){ convertion_counter--;}
                else {
                    convertion_counter = samplingrate;
                    ADCx->CR2 |= ADC_CR2_SWSTART;
                    ADCx->CR2 |= ADC_CR2_ADON;
                }
            }
            break;
        default: break;
    }
    msg->message = NM_NULL;
}

//------------------------------------------------------------------------------
void NAdc::InterruptCallBack(NMESSAGE* msg){
    switch(msg->message){
        case NM_DMA_OK:
            if(OnDataBlock !=NULL){ OnDataBlock(buffer[1L].data, buffer[1L].size);} break;

        case NM_DMA_MOK:
            if(OnDataBlock !=NULL){ OnDataBlock(buffer[0L].data, buffer[0L].size);} break;

        case NM_ADCEOC:
        	if(msg->data1 == vector_index){
        		if(OnData != NULL){ OnData(ADCx->DR);} break;
        	}
        	// No break;

        case NM_ADCAWD:
        	if(msg->data1 == vector_index){
				if((msg->data2 > upperlimit)&&(OnUpperThreshold != NULL)){
					OnUpperThreshold(msg->data2);
				} else if((msg->data2 < lowerlimit)&&(OnLowerThreshold != NULL)){
						OnLowerThreshold(msg->data2);
				}
        	}
            break;

        default: break;
    }
    msg->message = NM_NULL;
}

//==============================================================================
