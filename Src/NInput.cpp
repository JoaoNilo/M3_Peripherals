//============================================================================//
#include "NInput.h"
#include "DRV_SSR.h"

#define PORT_WIDTH 16
//------------------------------------------------------------------------------
inLevels NInput::ReadPin(){
    bool result = false; 
    if(porta!=NULL){
       if(porta->IDR & ((uint32_t)1 << Pino_InitStructure.Pin)){ result = true;}
    }
	return((inLevels)(result ^ inverted));
}

//------------------------------------------------------------------------------
NInput::NInput(GPIO_TypeDef* Port, uint32_t Pin){
    OnLevelChange = NULL; OnRisingEdge = NULL; OnFallingEdge = NULL;

    porta = Port;

    IO_PortClockOn(IO_PortIndex(porta));

    *((uint32_t*)&Pino_InitStructure) = 0;

    if(Pin < PORT_WIDTH){
        Pino_InitStructure.Pin = Pin & 0x0F;

        Pino_InitStructure.Mode = io_In_Floating;
        IO_PinInit (porta, &Pino_InitStructure);
    }

    Level.setOwner(this);
    Level.get(&NInput::GetLevel);

    Access.setOwner(this);
    Access.set(&NInput::SetAccess);
    Access.get(&NInput::GetAccess);

    Edge.setOwner(this);
    Edge.set(&NInput::SetEdge);
    Edge.get(&NInput::GetEdge);

    Inverted.setOwner(this);
    Inverted.set(&NInput::SetInverted);
    Inverted.get(&NInput::GetInverted);

    Bias.setOwner(this);
    Bias.set(&NInput::SetBias);

    //---------------------------
    PreviousVector = NULL;
    Enabled = true;
    access = inQueued;
    edge = inFallingEdge;
	inverted = false;

    SetAccess(access);
    SetBias(inFloating);

    //---------------------------
    level = ReadPin();
}

//------------------------------------------------------------------------------
NInput::~NInput(){}

//------------------------------------------------------------------------------
NV_ID NInput::GetVectorIndex(uint32_t pin_number){
    uint32_t result = NV_LAST + 1;
    if(pin_number<5){ result = pin_number + NV_EXTINT0;}
    else if(pin_number < 10){ result = (pin_number-5) + NV_EXTINT5;}
    else if(pin_number < 16){ result = (pin_number-10) + NV_EXTINT10;}
    return((NV_ID)result);
}

//------------------------------------------------------------------------------
void NInput::SetupInterrupt(){
    uint32_t vector_index = GetVectorIndex(Pino_InitStructure.Pin);

    if(access == inInterrupt){
        if(PreviousVector == NULL){

            if(Handle != (HANDLE)SSR_GetCallback(vector_index)){
                PreviousVector = (HANDLE)SSR_InstallCallback((uint32_t)Handle, vector_index);
            }
        }
        IO_SetExtendedIT(porta, &Pino_InitStructure);
		Priority = nTimeCritical;
    } else {
        if(PreviousVector != NULL){

            SSR_InstallCallback((uint32_t)PreviousVector, vector_index);
            PreviousVector = NULL;
        } else {
            //IO_ResetExtendedIT(porta, &Pino_InitStructure);
        }
		if(access == inImmediate){ Priority = nNormal;}
		else { Priority = nRelaxed;}
    }
}

//------------------------------------------------------------------------------
inLevels NInput::GetLevel(){
    if(access != inQueued){ level = ReadPin();}
    return(level);
}

//------------------------------------------------------------------------------
void NInput::SetAccess(inAccesses A){

    //Pino_InitStructure.Mode = io_In_PullUp;
    
    if(A!=inInterrupt){ Pino_InitStructure.Int = 0;}
    else { Pino_InitStructure.Int = 1;}

    IO_PinInit (porta, &Pino_InitStructure);

    if(access != A){
        SetupInterrupt();
        access = A;
    }
}

//------------------------------------------------------------------------------
inAccesses NInput::GetAccess(){
    return(access);
}

//------------------------------------------------------------------------------
void NInput::SetEdge(inEdges new_edge){
    edge = new_edge;
    Pino_InitStructure.Rise = 0; Pino_InitStructure.Fall = 0;
    if((edge== inRisingEdge)||(edge==inRiseAndFall)){ Pino_InitStructure.Rise = 1;}
    if((edge== inFallingEdge)||(edge==inRiseAndFall)){ Pino_InitStructure.Fall = 1;}
    SetupInterrupt();
}

//------------------------------------------------------------------------------
inEdges NInput::GetEdge(){
    return(edge);
}

//------------------------------------------------------------------------------
void NInput::SetInverted(bool new_inv){
    inverted = new_inv;
	level = ReadPin();
}

//------------------------------------------------------------------------------
bool NInput::GetInverted(){
    return(inverted);
}

//------------------------------------------------------------------------------
void NInput::SetBias(inBiases B){

    if(B == inFloating){ Pino_InitStructure.Mode = io_In_Floating;}
    else if(B == inPullDown){ Pino_InitStructure.Mode = io_In_PullDown;}
    else if(B == inPullUp){ Pino_InitStructure.Mode = io_In_PullUp;}
    IO_PinInit (porta, &Pino_InitStructure);
}


//------------------------------------------------------------------------------
inLevels NInput::ReadInput(){
    if(Enabled){
        inLevels new_level = ReadPin();
        if(level != new_level){
            level = new_level;
            if((level == inHigh)&&((edge==inRisingEdge)||(edge==inRiseAndFall))){
                if(OnRisingEdge != NULL){ OnRisingEdge();}
            } else if((level == inLow)&&((edge==inFallingEdge)||(edge==inRiseAndFall))){
                if(OnFallingEdge != NULL){ OnFallingEdge();}
            }
            if(OnLevelChange != NULL){ OnLevelChange();}
        }
    }
    return(level);
}

//------------------------------------------------------------------------------
void NInput::Notify(NMESSAGE* msg){
    if(msg->message == NM_KEYSCAN){

        IO_MaskExtendedIT(&Pino_InitStructure);

        ReadInput();

        IO_UnmaskExtendedIT(&Pino_InitStructure);
    }
    msg->message = NM_NULL;
}

//------------------------------------------------------------------------------
/// direct notification
/// Msg1:  	message = NM_EXTINT
///   		data1  = component NV_ID
///   		data2  = 32bit encoded port_pin (0x0000PPpp)
///   		tag     = not used
//------------------------------------------------------------------------------
void NInput::InterruptCallBack(NMESSAGE* msg){
    if(msg->message == NM_EXTINT){
        GPIO_TypeDef* Port = IO_GetPort(msg->data2);
        uint32_t Pin = (msg->data2 & 0x0000000F);
        if((Pino_InitStructure.Pin = Pin)&&(porta = Port)){

            ReadInput();
        } else {
            NComponent* prevInput = (NComponent*)PreviousVector;
            if(prevInput != NULL){ prevInput->InterruptCallBack(msg);}
        }
    }
}

//==============================================================================
