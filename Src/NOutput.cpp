//==============================================================================
#include "NOutput.h"

//------------------------------------------------------------------------------
NOutput::NOutput(){}

//------------------------------------------------------------------------------
NOutput::~NOutput(){}

//------------------------------------------------------------------------------
void NOutput::WritePin(ouLevels l){
    if(porta!=NULL){
        if((bool)l){ porta->BSRR = 1 << Pino_InitStructure.Pin;}
        else { porta->BRR = 1 << Pino_InitStructure.Pin;}
    }
}

//------------------------------------------------------------------------------
NOutput::NOutput(GPIO_TypeDef* Porta, uint32_t Pino){

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

    Level.setOwner(this);
    Level.set(&NOutput::SetLevel);
    Level.get(&NOutput::GetLevel);

    Access.setOwner(this);
    Access.set(&NOutput::SetAccess);
    Access.get(&NOutput::GetAccess);

    //---------------------------
    level = ouLow;
    access = ouQueued;
    SetEnabled(true);

    //---------------------------
    current_level = level;
}

//------------------------------------------------------------------------------
void NOutput::SetEnabled(bool E){
    enabled = E;
    if(access == ouImmediate) Refresh(level);
}

//------------------------------------------------------------------------------
bool NOutput::GetEnabled(){
    return(enabled);
}

//------------------------------------------------------------------------------
void NOutput::SetLevel(ouLevels oL){
    level = oL;
    if(access == ouImmediate) Refresh(level);
}

//------------------------------------------------------------------------------
ouLevels NOutput::GetLevel(){
    return(level);
}

//------------------------------------------------------------------------------
void NOutput::SetAccess(ouAccesses oA){
    access = oA;
}

//------------------------------------------------------------------------------
ouAccesses NOutput::GetAccess(){
    return(access);
}

//------------------------------------------------------------------------------
void NOutput::Toggle(){
    if(level==ouLow) level = ouHigh;
    else level = ouLow;
    SetLevel(level);
}

//------------------------------------------------------------------------------
void NOutput::Refresh(ouLevels new_level){
    if(new_level == ouTris){
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

//------------------------------------------------------------------------------
void NOutput::Notify(NMESSAGE* msg){
    if(enabled){
        switch(msg->message){
            case NM_REPAINT: Refresh(level); break;
            default: break;
        }
    }
    msg->message = NM_NULL;
}

void NOutput::InterruptCallBack(NMESSAGE*){};

//==============================================================================
