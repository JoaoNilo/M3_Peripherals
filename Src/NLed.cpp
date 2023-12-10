//==============================================================================
#include "NLed.h"

#define LED_INTERVAL_DEFAULT	 	100 	// ms
#define LED_DUTY_DEFAULT	 		50 		// %
#define LED_INTERVAL_MAX	 		10000 	// ms
#define LED_INTERVAL_MIN	 		25 		// ms
#define LED_BURST_MIN	 			2
#define LED_BURST_MAX	 			100

//------------------------------------------------------------------------------
NLed::NLed(GPIO_TypeDef* Porta, uint32_t Pino):NOutput(Porta, Pino){

    OnBurstFinished = NULL;
    
    Status.setOwner(this);
    Status.set(&NLed::SetStatus);
    Status.get(&NLed::GetStatus);

    Driver.setOwner(this);
    Driver.set(&NLed::SetDriver);
    Driver.get(&NLed::GetDriver);

    Access.setOwner(this);
    Access.set(&NLed::SetAccess);
    Access.get(&NLed::GetAccess);

    Interval.setOwner(this);
    Interval.set(&NLed::SetInterval);
    Interval.get(&NLed::GetInterval);

    Duty.setOwner(this);
    Duty.set(&NLed::SetDuty);
    Duty.get(&NLed::GetDuty);

    Burst.setOwner(this);
    Burst.set(&NLed::SetBurst);
    Burst.get(&NLed::GetBurst);

    //---------------------------
    // "default" values
    status = ldOff;
	previous_status = ldOff;
    driver = ldNormal;
    burst = 0;
    pin_status = 0;
    enabled = true;
    access = ldImmediate;
    duty = LED_DUTY_DEFAULT;

    SetInterval(LED_INTERVAL_DEFAULT);
}

//------------------------------------------------------------------------------
void NLed::SetStatus(ldStatus S){
	if(S == ldSignaling){ S = ldBlinking;}
	previous_status = status;
	if(status == ldSignaling){ status = ldBlinking;}
	status = S;	burst = 0;
    if(access==ldImmediate){ Repaint(status);}
}

//------------------------------------------------------------------------------
ldStatus NLed::GetStatus(){ return(status);}

//------------------------------------------------------------------------------
void NLed::SetDriver(ldDrivers D){
    driver = D;
    if(access == ldImmediate) Repaint(status);
}

//------------------------------------------------------------------------------
ldDrivers NLed::GetDriver(){
    return(driver);
}

//------------------------------------------------------------------------------
void NLed::SetAccess(ldAccesses A){ access = A;}

//------------------------------------------------------------------------------
ldAccesses NLed::GetAccess(){ return(access);}

//------------------------------------------------------------------------------
void NLed::SetInterval(uint32_t T){
	if(T < LED_INTERVAL_MIN){ T = LED_INTERVAL_MIN;}
	else if(T > LED_INTERVAL_MAX){ T = LED_INTERVAL_MAX;}
    interval = T;
    on_time = ((T * duty) / 100);
    off_time = T - on_time;

    if(status == ldSignaling){ counter = interval;}
    else { counter = on_time;}
}

//------------------------------------------------------------------------------
uint32_t NLed::GetInterval(){
    return(interval);
}

//------------------------------------------------------------------------------
void NLed::SetDuty(uint32_t D){
	if(D < 1){ D = 1;}
	else if(D > 100){ D = 100;}
    duty = D;
    SetInterval(interval);
}

//------------------------------------------------------------------------------
uint32_t NLed::GetDuty(){
    return(duty);
}

//------------------------------------------------------------------------------
void NLed::SetBurst(uint32_t B){
    if(B > LED_BURST_MAX){ B = LED_BURST_MAX;}
    if(B < LED_BURST_MIN){ B = LED_BURST_MIN;}
	previous_status = status;
	burst = (B*2) - 1; status = ldSignaling;
	Repaint(ldOn);
}

//------------------------------------------------------------------------------
uint32_t NLed::GetBurst(){  return(burst);}

//------------------------------------------------------------------------------
void NLed::Toggle(){

    if(status==ldOff){ status = ldOn; previous_status = ldOff;}
    else if(status==ldOn){ status = ldOff;  previous_status = ldOn;}
}

//------------------------------------------------------------------------------
void NLed::Repaint(ldStatus St){

    switch((uint32_t)St){
		case ldPrevious: 
			St = previous_status;
			//No break
        case ldOn:
        case ldOff:
        	pin_status = St;
            if(driver == ldInverted){ pin_status = !pin_status;}
            break;
        case ldBlinking:
            break;
        default: break;
    }
    NOutput::Refresh((ouLevels)pin_status);
}

//------------------------------------------------------------------------------
void NLed::Notify(NMESSAGE* msg){
    if(enabled){
        switch(msg->message){
            case NM_REPAINT:
                Repaint(status);
                break;
            case NM_TIMETICK:

                if(status ==ldBlinking){
                    if(counter>0){ counter--;}
                    else {
                        if(pin_status == ldOn){
                            pin_status = ldOff;
                            counter = off_time;
                        } else {
                            pin_status = ldOn;
                            counter = on_time;
                        }
                    }
                    if(access == ldImmediate){ Repaint(status);}

                } else if(status == ldSignaling){
                    if(counter > 0){ counter--;}
                    else {
                        counter = interval;
						if(burst > 0){
							pin_status = !pin_status; burst--;
						} else {
							status = previous_status;
							if(OnBurstFinished != NULL){ OnBurstFinished();}
						}
                    }
                    if(access == ldImmediate){ Repaint(status);}
                }
                break;
            default: break;
        }
    }
    msg->message = NM_NULL;
}

//==============================================================================
