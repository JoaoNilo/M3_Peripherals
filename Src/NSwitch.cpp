//==============================================================================
#include "NSwitch.h"

//------------------------------------------------------------------------------
NSwitch::NSwitch(GPIO_TypeDef* Porta, uint32_t Pino):NInput(Porta, Pino){

    OnChange = NULL; OnPress = NULL; OnRelease = NULL;

    Status.setOwner(this);
    Status.get(&NSwitch::GetStatus);
    
    Mode.setOwner(this);
    Mode.set(&NSwitch::SetMode);
    Mode.get(&NSwitch::GetMode);

    Debouncing.setOwner(this);
    Debouncing.set(&NSwitch::SetDebouncing);
    Debouncing.get(&NSwitch::GetDebouncing);

    //---------------------------
    NInput::Inverted = true;
    Enabled = true;
    mode = swButton;
    SetDebouncing(swStandard);
    status = (swStatus)NInput::ReadPin();
    
    //---------------------------
    change_counter = 0;
}

//------------------------------------------------------------------------------
NSwitch::~NSwitch(){}

//------------------------------------------------------------------------------
swStatus NSwitch::GetStatus(){
    return(status);
}

//------------------------------------------------------------------------------
void NSwitch::SetMode(swModes sT){
    mode = sT;
}

//------------------------------------------------------------------------------
swModes NSwitch::GetMode(){
    return(mode);
}

//------------------------------------------------------------------------------
void NSwitch::SetDebouncing(swDebouncings newValue){
    debounce = 2 + ((uint32_t)newValue & 0x03) * 5;
}

//------------------------------------------------------------------------------
swDebouncings NSwitch::GetDebouncing(){
	swDebouncings result = swShort;
	if(debounce > 10){ result = swLong;}
	else if(debounce > 5){ result = swStandard;}
    return(result);
}

//------------------------------------------------------------------------------
void NSwitch::Notify(NMESSAGE* msg){

    if(Enabled){    
        switch(msg->message){
            case NM_KEYSCAN:
                pin_status = (swStatus)NInput::ReadPin();

                if(status != pin_status){
                    if(++change_counter < debounce){}
                    else{
                        change_counter = 0;
                        status = pin_status;
                        if(status== swOn){
                            if((mode==swButton)&&(OnPress)) OnPress();
                        } else {
                            if((mode==swButton)&&(OnRelease)) OnRelease();
                        }
                        if((mode==swSwitch)&&(OnChange)) OnChange();
                    }
                } else change_counter = 0;
                break;
            default:break;
        }
    }
    msg->message = NM_NULL;
}

//==============================================================================

