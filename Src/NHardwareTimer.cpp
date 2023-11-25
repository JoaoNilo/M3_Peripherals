//============================================================================//
#include "NHardwareTimer.h"
//------------------------------------------------------------------------------
void NHardwareTimer::ReloadRegisters(){
    TIMx->CR1 = config1;
    TIMx->CR2 = config2;
    TIMx->PSC = prescaler;
    TIMx->ARR = reload;
}

//------------------------------------------------------------------------------
void NHardwareTimer::SaveRegisters(){
    config1 = TIMx->CR1;
    config2 = TIMx->CR2;
    prescaler = TIMx->PSC;
    reload = TIMx->ARR;
}

//------------------------------------------------------------------------------
void NHardwareTimer::SetPrescaler(){
    uint32_t divider = 1000000;
    uint32_t APBClock = 0;
    bool status = (bool) (TIMx->CR1 & ~TIM_CR1_CEN);
	TIMx->CR1 &= ~TIM_CR1_CEN;

	if(TIMx == TIM1){ APBClock = CPU_GetTimerFrequencyAPB2();}
	#if defined(TIM8)
	else if(TIMx == TIM8){ APBClock = CPU_GetTimerFrequencyAPB2();}
	#endif
	#if defined(TIM9)
	else if(TIMx == TIM9){ APBClock = CPU_GetTimerFrequencyAPB2();}
	#endif
	#if defined(TIM10)
	else if(TIMx == TIM10){ APBClock = CPU_GetTimerFrequencyAPB2();}
	#endif
	#if defined(TIM11)
	else if(TIMx == TIM11){ APBClock = CPU_GetTimerFrequencyAPB2();}
	#endif
	else { APBClock = CPU_GetTimerFrequencyAPB1();}
    
    prescaler = ((APBClock/divider)-1);
    TIMx->PSC = prescaler;
	if(status){ TIMx->CR1 |= TIM_CR1_CEN;}
}

//------------------------------------------------------------------------------
//NHardwareTimer::NHardwareTimer(){}

//------------------------------------------------------------------------------
NHardwareTimer::NHardwareTimer(TIM_TypeDef* TIMy){

    OnTimer = NULL;

    Enabled.setOwner(this);
    Enabled.set(&NHardwareTimer::SetEnabled);
    Enabled.get(&NHardwareTimer::GetEnabled);

    Mode.setOwner(this);
    Mode.set(&NHardwareTimer::SetMode);
    Mode.get(&NHardwareTimer::GetMode);

    Scale.setOwner(this);
    Scale.set(&NHardwareTimer::SetScale);
    Scale.get(&NHardwareTimer::GetScale);

    IrqPriority.setOwner(this);
    IrqPriority.set(&NHardwareTimer::SetIrqPriority);
    IrqPriority.get(&NHardwareTimer::GetIrqPriority);

    Interval.setOwner(this);
    Interval.set(&NHardwareTimer::SetInterval);
    Interval.get(&NHardwareTimer::GetInterval);

    Buffered.setOwner(this);
    Buffered.set(&NHardwareTimer::SetBuffered);
    Buffered.get(&NHardwareTimer::GetBuffered);

    //------------------------------------------
    TIMx = TIMy;
    if(TIMx==TIM1){
        TIM_IRQn = TIM1_BRK_IRQn;
        vector_index = NV_TIM1;
    } else if(TIMx==TIM2){
        TIM_IRQn = TIM2_IRQn;
        vector_index = NV_TIM2;
    } else if(TIMx==TIM3){
        TIM_IRQn = TIM3_IRQn;
        vector_index = NV_TIM3;
    }
	#if defined(TIM4)
    else if(TIMx==TIM4){
        TIM_IRQn = TIM4_IRQn;
        vector_index = NV_TIM4;
    }
	#endif

	#if defined(TIM5)
    else if(TIMx==TIM5){
        TIM_IRQn = TIM5_IRQn;
        vector_index = NV_TIM5;
    }
	#endif

	#if defined(TIM8)
    else if(TIMx==TIM8){
		TIM_IRQn = TIM8_BRK_IRQn;
		vector_index = NV_TIM8;
    }
	#endif

	#if defined(TIM9)
	else if(TIMx==TIM9){
		TIM_IRQn = TIM9_IRQn;
		vector_index = NV_TIM9;

	}
	#endif

	#if defined(TIM10)
	else if(TIMx==TIM10){
		TIM_IRQn = TIM10_IRQn;
		vector_index = NV_TIM10;

	}
	#endif

	#if defined(TIM11)
	else if(TIMx==TIM11){
		TIM_IRQn = TIM11_IRQn;
		vector_index = NV_TIM11;

	}
	#endif

	#if defined(TIM12)
	else if(TIMx==TIM12){
		TIM_IRQn = TIM12_IRQn;
		vector_index = NV_TIM12;

	}
	#endif

	#if defined(TIM13)
	else if(TIMx==TIM13){
		TIM_IRQn = TIM13_IRQn;
		vector_index = NV_TIM13;

	}
	#endif

	#if defined(TIM14)
	else if(TIMx==TIM14){
		TIM_IRQn = TIM14_IRQn;
		vector_index = NV_TIM14;

	}
	#endif

    else {
        SSR_ThrowException(NX_CONSTRUCTOR_PARAMETER_INVALID);
        return;
    }

    //------------------------------------------
	SSR_InstallCallback((uint32_t)Handle, vector_index);

    //------------------------------------------
    CPU_PeripheralClockEnable(TIMx);

    //------------------------------------------
	config1 = (TIM_CR1_ARPE | TIM_CR1_DIR); 	// 0x0090;
    TIMx->CR1 = config1; TIMx->CR2 = 0;

    //------------------------------------------
    SetPrescaler();

    //------------------------------------------
	reload = INTERVAL_DEFAULT;
    TIMx->ARR = reload;

    //------------------------------------------
	Blocking = false;
	scale = htMicroseconds;
	irq_priority  = htPriorityLevel10;
	
	SetIrqPriority(irq_priority);

    //-----------------------------------------
    TIMx->SR &= ~TIM_SR_UIF;
    SaveRegisters();
    CPU_PeripheralClockDisable(TIMx);
    Priority = nTimeCritical;
}

//------------------------------------------------------------------------------
NHardwareTimer::~NHardwareTimer(){

    //------------------------------------------
	TIMx->CR1 = 0;

    //------------------------------------------
	SSR_InstallCallback(NV_NULL, vector_index);

    //-----------------------------------------
	if(TIMx==TIM1){
		NVIC_DisableIRQ(TIM1_BRK_IRQn); NVIC_DisableIRQ(TIM1_UP_IRQn);
		NVIC_DisableIRQ(TIM1_TRG_COM_IRQn); NVIC_DisableIRQ(TIM1_CC_IRQn);
	} else
	#ifdef TIM8
	if(TIMx==TIM8){
		NVIC_DisableIRQ(TIM8_BRK_IRQn); NVIC_DisableIRQ(TIM8_UP_IRQn);
		NVIC_DisableIRQ(TIM8_TRG_COM_IRQn); NVIC_DisableIRQ(TIM8_CC_IRQn);
	} else
	#endif
	{	NVIC_DisableIRQ(TIM_IRQn);}

    //-----------------------------------------
    CPU_PeripheralClockDisable(TIMx);
}

//------------------------------------------------------------------------------
void NHardwareTimer::SetEnabled(bool tE){
    if(tE){ Start(reload+1);}
    else { Stop();}
}

//------------------------------------------------------------------------------
bool NHardwareTimer::GetEnabled(){ return((bool)(TIMx->CR1&TIM_CR1_CEN));}

//------------------------------------------------------------------------------
void NHardwareTimer::SetBuffered(bool tB){
    if(tB){ TIMx->CR1 |= TIM_CR1_ARPE;}
    else { TIMx->CR1 &= ~TIM_CR1_ARPE;}
    config1 = TIMx->CR1;
}

//------------------------------------------------------------------------------
bool NHardwareTimer::GetBuffered(){ return(TIMx->CR1 &= TIM_CR1_ARPE);}

//------------------------------------------------------------------------------
void NHardwareTimer::SetMode(htModes tM){
    if(tM == htContinuous){ TIMx->CR1 &= ~TIM_CR1_OPM;}
    else { TIMx->CR1 |= TIM_CR1_OPM;}
    config1 = TIMx->CR1;
}

//------------------------------------------------------------------------------
htModes NHardwareTimer::GetMode(){
    return((htModes)(TIMx->CR1 & TIM_CR1_OPM));
}

//------------------------------------------------------------------------------
void NHardwareTimer::SetScale(htScales tS){
	if((tS == htMicroseconds)|| (tS == htMilliseconds)){ scale = tS;}
}

//------------------------------------------------------------------------------
htScales NHardwareTimer::GetScale(){ return(scale);}

//------------------------------------------------------------------------------
// @brief sets the timer´s IRQn priority level
void  NHardwareTimer::SetIrqPriority(htPriorities sPL){

	irq_priority = sPL;
	if((uint32_t)irq_priority > (uint32_t)htPriorityLevel10){ irq_priority = htPriorityLevel10;}
	uint32_t raw_pri = ((uint32_t)irq_priority + 3);
	uint32_t pri = NVIC_EncodePriority(NVIC_PriorityGroup_4, raw_pri, 0);

	if(TIMx==TIM1){
		NVIC_SetPriority( TIM1_BRK_IRQn, pri);
		NVIC_EnableIRQ( TIM1_BRK_IRQn);
		NVIC_SetPriority( TIM1_UP_IRQn, pri);
		NVIC_EnableIRQ( TIM1_UP_IRQn);
		NVIC_SetPriority( TIM1_TRG_COM_IRQn, pri);
		NVIC_EnableIRQ( TIM1_TRG_COM_IRQn);
		NVIC_SetPriority( TIM1_CC_IRQn, pri);
		NVIC_EnableIRQ( TIM1_CC_IRQn);
	}
	#if defined(TIM8)
	else if(TIMx==TIM8){
		NVIC_SetPriority( TIM8_BRK_IRQn, pri);
		NVIC_EnableIRQ( TIM8_BRK_IRQn);
		NVIC_SetPriority( TIM8_UP_IRQn, pri);
		NVIC_EnableIRQ( TIM8_UP_IRQn);
		NVIC_SetPriority( TIM8_TRG_COM_IRQn, pri);
		NVIC_EnableIRQ( TIM8_TRG_COM_IRQn);
		NVIC_SetPriority( TIM8_CC_IRQn, pri);
		NVIC_EnableIRQ( TIM8_CC_IRQn);
	}
	#endif
	else {
		NVIC_SetPriority(TIM_IRQn, pri);
		NVIC_EnableIRQ(TIM_IRQn);
	}
}

//------------------------------------------------------------------------------
// @brief sets the timer´s IRQn priority level
htPriorities NHardwareTimer::GetIrqPriority(){ return(irq_priority);}

//------------------------------------------------------------------------------
// @brief avoid very low intervals for stability ( >30us @ 72MHz for good results)
void NHardwareTimer::SetInterval(uint16_t new_interval){
    if(new_interval == 0){ new_interval = 1;}
    reload = new_interval-1;
    if(scale == htMilliseconds){ reload = 1000;}
    TIMx->CNT = 0;
	TIMx->ARR = reload;
}

//------------------------------------------------------------------------------
uint16_t NHardwareTimer::GetInterval(){return(reload+1);}

//------------------------------------------------------------------------------
void NHardwareTimer::Start(uint16_t T){
    CPU_PeripheralClockEnable(TIMx);
    ReloadRegisters();
    Factors.load(T);
		
    TIMx->CR1 &= ~TIM_CR1_CEN;
    SetInterval(T);
	TIMx->SR &= ~TIM_SR_UIF;

	if(Blocking){
        TIMx->DIER &= ~TIM_DIER_UIE;
	  	TIMx->CNT = 0;
        TIMx->CR1 |= TIM_CR1_CEN;
		while((TIMx->SR & TIM_SR_UIF)==0){}
		//if(OnTimer != NULL){ OnTimer();}
		TIMx->CR1 &= ~TIM_CR1_CEN;
	} else {
    	TIMx->DIER |= TIM_DIER_UIE;
    	TIMx->CNT = 0;
        TIMx->CR1 |= TIM_CR1_CEN;
	}
}

//------------------------------------------------------------------------------
void NHardwareTimer::Stop(){
    TIMx->CR1 &= ~TIM_CR1_CEN;
    TIMx->SR &= ~TIM_SR_UIF;
    config1 = TIMx->CR1;
    SaveRegisters();
    CPU_PeripheralClockDisable(TIMx);
}

//------------------------------------------------------------------------------
void NHardwareTimer::Wait(uint16_t Tw){
    uint16_t last_interval = reload+1;
  	bool last_block = Blocking;
  	Blocking = true;
	Start(Tw);
    Blocking = last_block;
    Start(last_interval);
}

//------------------------------------------------------------------------------
bool NHardwareTimer::ProcessEvent(){ return(false);}

//------------------------------------------------------------------------------
void NHardwareTimer::InterruptCallBack(NMESSAGE* tmsg){
	bool result = true;
	uint32_t dier = TIMx->DIER;
	TIMx->DIER = 0;
    if((tmsg->data1 == vector_index)&&(tmsg->message == NM_HARDTICK)){
    	if(scale == htMilliseconds){ result = Factors.count();}
    	if(result){
    		if(OnTimer != NULL){ OnTimer();} else { ProcessEvent();}
    	}
    }
    tmsg->message = NM_NULL;
    TIMx->SR &= ~TIM_SR_UIF;
    TIMx->DIER = dier;
}

//------------------------------------------------------------------------------
void NHardwareTimer::Notify(NMESSAGE* msg){
    if(msg->message == NM_SYSCLOCKCHANGE_PCLK){
        SetPrescaler();
	    msg->message = NM_NULL;
    } else { InterruptCallBack(msg);}
}

//==============================================================================
