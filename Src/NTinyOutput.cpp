//==============================================================================
#include "NTinyOutput.h"

//------------------------------------------------------------------------------
NTinyOutput::NTinyOutput(){}

//------------------------------------------------------------------------------
NTinyOutput::~NTinyOutput(){}

//------------------------------------------------------------------------------
void NTinyOutput::WritePin(toLevels l){
    if(porta!=NULL){
        if((bool)l){ porta->BSRR = 1 << Pino_InitStructure.Pin;}
        else { porta->BRR = 1 << Pino_InitStructure.Pin;}
    }
}

//------------------------------------------------------------------------------
NTinyOutput::NTinyOutput(GPIO_TypeDef* Porta, uint32_t Pino){
 
    Level.setOwner(this);
    Level.set(&NTinyOutput::SetLevel);
    Level.get(&NTinyOutput::GetLevel);

    Bias.setOwner(this);
    Bias.set(&NTinyOutput::SetBias);
    Bias.get(&NTinyOutput::GetBias);

    porta = Porta;

    IO_PortClockOn(IO_PortIndex(porta));

    *((uint32_t*)&Pino_InitStructure) = 0;

    if(Pino < __PORT_WIDTH){
        Pino_InitStructure.Pin = Pino;

        if((porta==GPIOC) && (Pino_InitStructure.Pin>12)){
            Pino_InitStructure.Mode = io_Out_PushPull_2MHz;
        } else {
            Pino_InitStructure.Mode = io_Out_PushPull_50MHz;
        }
        IO_PinInit (porta, &Pino_InitStructure);
    }

    
    //---------------------------
    level = toLow;
    SetEnabled(true);

    //---------------------------
    WritePin(level);
}

//------------------------------------------------------------------------------
void NTinyOutput::SetEnabled(bool E){
    enabled = E;
    Refresh(level);
}

//------------------------------------------------------------------------------
bool NTinyOutput::GetEnabled(){
    return(enabled);
}

//------------------------------------------------------------------------------
void NTinyOutput::SetLevel(toLevels oL){
    level = oL;
    Refresh(level);
}

//------------------------------------------------------------------------------
toLevels NTinyOutput::GetLevel(){
    return(level);
}

//------------------------------------------------------------------------------
void NTinyOutput::SetBias(toBiases oL){
    bias = oL;
}

//------------------------------------------------------------------------------
toBiases NTinyOutput::GetBias(){
    return(bias);
}

//------------------------------------------------------------------------------
void NTinyOutput::Toggle(){
    if(level==toLow){ level = toHigh;}
    else { level = toLow;}
    SetLevel(level);
}

//------------------------------------------------------------------------------
void NTinyOutput::Refresh(toLevels new_level){
    if(!enabled){ return;}
    if(new_level == toTris){
        Pino_InitStructure.Mode = io_In_Floating;
        IO_PinInit (porta, &Pino_InitStructure);
    } else {
        if(Pino_InitStructure.Mode == io_In_Floating){
            if((porta==GPIOC) && (Pino_InitStructure.Pin>12)){
                Pino_InitStructure.Mode = io_Out_PushPull_2MHz;
            } else {
                Pino_InitStructure.Mode = io_Out_PushPull_50MHz;
            }
        }
        WritePin(new_level);
    }
    level = new_level;
    return;
}

//==============================================================================
