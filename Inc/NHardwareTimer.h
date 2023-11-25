//==============================================================================
/**
 * @file NHardwareTimer.h
 * @brief Hardware timer abstraction class\n
 * This class provides resources to setup and use general purpose timer in the\n
 * MCU hardware for precision delays specified in microseconds and milliseconds.
 * @version 1.0.0
 * @author Joao Nilo Rodrigues - nilo@pobox.com
 *
 *------------------------------------------------------------------------------
 *
 * <h2><center>&copy; Copyright (c) 2020 Joao Nilo Rodrigues
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by "Joao Nilo Rodrigues" under BSD 3-Clause
 * license, the "License".
 * You may not use this file except in compliance with the License.
 *               You may obtain a copy of the License at:
 *                 opensource.org/licenses/BSD-3-Clause
 */
//------------------------------------------------------------------------------
#ifndef NHARDWARETIMER_H
    #define NHARDWARETIMER_H

    #include "DRV_CPU.h"
    #include "NComponent.h"

//------------------------------------------------------------------------------

    /**
     * @enum htModes
     * @brief This enumeration defines the options for the @ref Mode property.
     */
    enum htModes { htContinuous,	//!< multiple OnTimer events, every "Interval"
    			   htSingleRun		//!< single OnTimer event after the first "Interval"
    			 };

    /**
     * @enum htScales
     * @brief This enumeration defines the options for the @ref Scale property.
     */
    enum htScales { htMicroseconds,			//!< @ref Interval property is specified in microseconds
    				htMilliseconds			//!< @ref Interval property is specified in milliseconds
    			  };

    /**
     * @enum htPriorities
     * @brief This enumeration defines the options for the @ref IrqPriority property.
     * - Hardware priority assigned to peripheral´s IRQn line:\n
     * htPriorityLevel1 is the highest priority
     */
    enum htPriorities{ htPriorityLevel1,	//!< highest priority
    				   htPriorityLevel2,	//!< 2nd highest priority
					   htPriorityLevel3,	//!< 3rd highest priority
    				   htPriorityLevel4,	//!< 1st standard priority
					   htPriorityLevel5,	//!< 2nd standard priority
					   htPriorityLevel6,	//!< 3rd standard priority
					   htPriorityLevel7,	//!< 4th standard priority
					   htPriorityLevel8,	//!< 3rd lowest priority
					   htPriorityLevel9,	//!< 2nd lowest priority
					   htPriorityLevel10	//!< lowest priority
    				};

    class factorize{
    	public:

    		uint32_t reload_mili_1;
    		uint32_t count_mili_1;

    		void load(uint32_t value){
    			if(value == 0){ value = 1;}	else { value--;}
    			reload_mili_1 = value;
    			count_mili_1 = reload_mili_1;
    		}

    		bool count(){
    			uint16_t result = false;
    			if(count_mili_1 > 0){ count_mili_1--;}
    			else {count_mili_1 = reload_mili_1; result = true;}
    			return(result);
    		}
    };

	//--------------------------------------------------------------------------
    /** @brief Hardware timer abstraction class\n
    * - This class provides resources to setup and use general purpose timer in the\n
    * MCU hardware for precision delays specified in microseconds and milliseconds.
	*/
    class NHardwareTimer: public NComponent{
        private:
			#define PRESCALER_DEFAULT 	((uint16_t)  71)
			#define INTERVAL_DEFAULT  	((uint16_t) 1000)

    		factorize Factors;
            uint16_t vector_index;
            htScales scale;
            htPriorities irq_priority;

        //-----------------------------
        protected:
            TIM_TypeDef* TIMx;
            IRQn_Type TIM_IRQn;
            uint32_t prescaler;
            uint32_t reload;
            uint32_t config1;
            uint32_t config2;

        //-----------------------------
        private:
            void SetPrescaler();
            void SetEnabled(bool);
            bool GetEnabled();
            void SetMode(htModes);
            htModes GetMode();
            void SetIrqPriority(htPriorities);
            htPriorities GetIrqPriority();
            void SetInterval(uint16_t);
            uint16_t GetInterval();
            void SetScale(htScales);
            htScales GetScale();
            void SetBuffered(bool);
            bool GetBuffered();

            void SaveRegisters();
            void ReloadRegisters();

        //-------------------------------
        protected:
			virtual bool ProcessEvent();

        //-----------------------------------------------
        public:
            // METHODS
            /**
             * @brief Constructor for this component.
             * @arg Timer: hardware timer to be used (TIM2, TIM3, etc.)
             * @note Be sure to specify a hardware timer available on the target MCU and
             * not in used by the application.
             */
            NHardwareTimer(TIM_TypeDef* Timer);

            /**
             * @brief Standard destructor for this component.
             */
            ~NHardwareTimer();

            /**
             * @brief This method is used as a system callback function for message dispatching.
             * @note This callback method should not be called by the application.
             */
            virtual void Notify(NMESSAGE*);

            /**
             * @brief This method is used as a system callback function for time critical
             * events dispatching.
             */
            virtual void InterruptCallBack(NMESSAGE*);

            /**
             * @brief This method can be used to start the timer operation.
             * @arg newInterval:
             * required time interval between @ref OnTimer events.
             * @note This method updates the value of the @ref Interval property.
             * @warning The @ref Scale property affects how the newInterval is interpreted.
             */
            void Start(uint16_t newInterval);

            /**
             * @brief This method can be used to stop the timer operation.
            */
            void Stop();

            /**
             * @brief This method can be used to start the timer for a single "blocking run".
             * @arg newInterval:
             * required time interval to wait for (blocked).
             * @note This method updates the value of the @ref Interval property.
             * @warning The @ref Scale property affects how the newInterval is interpreted.
             */
			void Wait(uint16_t  newInterval);

            //-------------------------------------------
            // EVENTS
            /**
             * @brief This is the event handler for timing events.
             * - This event handler is called periodically at every interval expiration
             * if property @ref Mode is set to @ref htContinuos, or just once if property @ref Mode
             * is set to htSingleRun.
             */
            void (*OnTimer)(void);

            //-------------------------------------------
            // PROPERTIES
            /**
             * @brief This property can be used to start and stop the timer.
             * The Enabled property cannot be used to pause the timer since the timer
             * is reset every time this property is set to false.
             * This property provides READ and WRITE access.
             * - Default value: false
             */
            property<NHardwareTimer, bool, propReadWrite> Enabled;

            /**
             * @brief This property is used to set the time-base for the interval specification.\n
             * - The available options are detailed in the @ref htScales enumeration description.
             * This property provides READ and WRITE access.
             * - Default value: @ref htMicroseconds
             */
            property<NHardwareTimer, enum htScales, propReadWrite> Scale;

            /**
             * @brief This property is used to set the required time interval between @ref OnTimer events.
             * - This property is affected by the @ref Scale property as follows:
             * htMicroseconds: interval defines the amount of microseconds between OnTimer events;
             * htMilliseconds: interval defines milliseconds between OnTimer events.
             * This property provides READ and WRITE access.
             * - Default value: 100  (us)
             *
             * @note This property can be changed by the @ref Start method.
             * @warning Avoid very low intervals for stability ( >30us @ 72MHz for good results)
             */
            property<NHardwareTimer, uint16_t, propReadWrite> Interval;

            /**
             * @brief This property is used to set the operation mode. The available options are
             * detailed in the @ref htModes enumeration description.
             * This property provides READ and WRITE access.
             * - Default value: @ref htContinuous
             */
            property<NHardwareTimer, enum htModes, propReadWrite> Mode;

            /**
             * @brief This property is sets the "buffered reload" mode.
             * Check the manufacturer Reference Manual for additional information.
             * This property provides READ and WRITE access.
             * - Default value: true (Buffered Reload)
             */
            property<NHardwareTimer, bool, propReadWrite> Buffered;            		// true (Buffered Reload)

            /**
             * @brief This property is used to select the hardware interrupt priority for this timer.
             * - The available options are detailed in the @ref htPriorities enumeration description.
             * This property provides READ and WRITE access.
             * - Default value: @ref htPriorityLevel10 (lowest)
             */
            property<NHardwareTimer, enum htPriorities, propReadWrite> IrqPriority;

            /**
             * @brief This property can be used in very special cases where "blocking"
             * mode is required. When the @ref Start method is called with this property is set,
             * the program execution will be stuck on the @ref Start() line for the specified amount
             * of microseconds (or milliseconds, depending on @ref Scale).
             * This property provides only READ and WRITE access.
             * - Default value: false
             */
			bool Blocking;
    };

#endif
//==============================================================================
