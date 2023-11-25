//============================================================================//
#include "NMco.h"
#include "DRV_CPU.h"
#include "DRV_SSR.h"
#include "DRV_IO.h"

//------------------------------------------------------------------------------
NMco::NMco(){}

//------------------------------------------------------------------------------
NMco::~NMco(){}

//------------------------------------------------------------------------------
#define __PORT_WIDTH 16L
NMco::NMco(mcoPinouts pinout){

	//-------------------------
    IO_Config PinConfig;
    PinConfig.Mode = io_Alt_PushPull_50MHz;
    GPIO_TypeDef* port;

    
    switch(pinout){
        case mcoStandard:
        case mcoRemapped:
            PinConfig.Pin = MCO_STANDARD_PIN;
            port = MCO_STANDARD_PORT;
        	break;
        default:
            SSR_ThrowException(NX_CONSTRUCTOR_PARAMETER_INVALID);
            return;
    }
    
    //-------------------------
    Enabled.setOwner(this);
    Enabled.set(&NMco::SetEnabled);
    Enabled.get(&NMco::GetEnabled);


    Source.setOwner(this);
    Source.set(&NMco::SetSource);
    Source.get(&NMco::GetSource);
    
    CPU_PeripheralClockEnable(port);
    IO_PinInit(port, &PinConfig);

     //---------------------------
    source = mcoNone;
    enabled = false;
    
}

//------------------------------------------------------------------------------
void NMco::SetEnabled(bool E){
    enabled = E;
    if(!enabled){ RCC->CFGR &= ~RCC_CFGR_MCO_Msk;}
    else {
        SetSource(source);
    }
}

//------------------------------------------------------------------------------
bool NMco::GetEnabled(){
    return(enabled);
}

//------------------------------------------------------------------------------
void NMco::SetSource(mcoSources new_source){
    if(enabled){
        uint32_t mco = ((uint32_t)new_source << RCC_CFGR_MCO_Pos);
        mco &= RCC_CFGR_MCO_Msk;
        RCC->CFGR &= ~RCC_CFGR_MCO_Msk;
        RCC->CFGR |= mco;
    }
    source = new_source;
}

//------------------------------------------------------------------------------
mcoSources NMco::GetSource(){ return(source);}

//==============================================================================
