//==============================================================================
/**
 * @file NLed.h
 * @brief LED abstraction class\n
 * This class provides resources to setup and use general purpose IO pin as\n
 * an LED driver pin.\n
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
#ifndef NLED_H
    #define NLED_H

    #include "NOutput.h"

	/**
	 * @enum ldStatus
	 * This enumeration defines all the possible states for a NLed component.\n
	 * @note Names were carefully chosen to be self-explanatory.
	 */
	enum ldStatus { ldOff,					//<! LED "logically" OFF (check "detailed description")
	    				ldOn,				//<! LED "logically" ON  (same as above)
						ldBlinking,			//<! LED toggling states On/Off every [Interval] (ms)\n
											//<! \n
						ldSignaling,		//<! internal state (not to be assigned by the application)
						ldPrevious			//<! internal state (not to be assigned by the application)
	    			};

	/**
	 * @enum ldAccesses
	 * This enumeration defines two possibilities for updating the NLed components\n
	 * depending on the context. If many components are used and the rate of change is\n
	 * not so high, queued update may provide a better result.
	 */
	enum ldAccesses { ldQueued,			//<! all queued NLeds are updated simultaneously
				    ldImmediate			//<! changes in Status are updated immediately
			      };

	/**
	 * @enum ldDrivers
	 * This enumeration defines two possible driver topologies.\n
	 * The correct choice depends greatly on the hardware driver.
	 */
	enum ldDrivers { ldNormal,			//<! NLed is ON when "pin" is high
					 ldInverted			//<! NLed is ON when "pin" is low
				  };

    //-----------------------------------
	/**
	 * @brief The NLed class provides an abstraction layer for GPIOs connected\n
	 * to LED drivers.
	 */
    class NLed: private NOutput{
        private:
            //-------------------------
            static const unsigned int __PORT_WIDTH = 16L;
            
            //-------------------------
            ldStatus status;
            ldAccesses access;
            ldDrivers driver;
            uint32_t interval;
            uint32_t duty;
            uint32_t burst;

            //-------------------------
            ldStatus previous_status;
            uint8_t pin_status;
            uint32_t counter;
            uint32_t on_time;
            uint32_t off_time;

        //-------------------------------------------------
        private:
            void Repaint(ldStatus);
            
            //---------------------------------------------
            void SetStatus(ldStatus);
            ldStatus GetStatus();
            void SetAccess(ldAccesses);
            ldAccesses GetAccess();
            void SetDriver(ldDrivers);
            ldDrivers GetDriver();
            void SetInterval(uint32_t);
            uint32_t GetInterval();
            void SetDuty(uint32_t);
            uint32_t GetDuty();
            void SetBurst(uint32_t);
            uint32_t GetBurst();

        //------------------------------------------------
        public:
            //-------------------------------------------
            // METHODS
            /**
             * @brief Constructor for this component.
             * @arg Port: GPIO to be used (GPIOA, GPIOB, etc.)
             * @arg Pin: pin number (0 to 15)
             * @note Make sure the specified GPIO and pin number are valid for
             * the chosen target MCU.
             */
            NLed(GPIO_TypeDef* Port, uint32_t Pin);

            /**
             * @brief Standard destructor for this component.
             */
            ~NLed(void);
            virtual void Notify(NMESSAGE*);

            //--------------------------------------------
            /**
             * Toggle is used to alternate the NLed component status between ldOff and ldOn.\n
             */
            void Toggle();

            //--------------------------------------------
            /**
             * OnBurstFinished event is fired when the NLed component finishes counting\n
             * the number of "blinks" previously assigned to the Burst property.
             */
            void (*OnBurstFinished)(void);
                
            //--------------------------------------------
            // PROPERTIES
			using NComponent::Tag;

            /**
             * @brief This property is used to change the LED status. The available options are
             * detailed in the @ref ldStatus enumeration description.\n
             * This property provides READ and WRITE access.
             * - Default value: @ref ldOff
             */
            property<NLed, enum ldStatus, propReadWrite> Status;   // ldOff

            /**
             * @brief This property sets the responsiveness of this component. The available options are
             * detailed in the @ref ldAccesses enumeration description.\n
             * This property provides READ and WRITE access.
             * - Default value: @ref ouQueued
             */
            property<NLed, enum ldAccesses, propReadWrite> Access;   // ldQueued

            /**
             * @brief This property is used adjust logic pin level (internal) to the observable
             * LED state (on /off), regardless the driver topology used.
             * The option for this property are detailed in the @ref ldDrivers enumeration description.\n
             * This property provides READ and WRITE access.
             * - Default value: @ref ldNormal
             */
            property<NLed, enum ldDrivers, propReadWrite> Driver;   // ldNormal

            /**
             * @brief This property defines the "blink interval", i.e. the time the LED remains on each state when blinking.
             * The "blink" interval is given in milliseconds and is limited in the range of 25 to 10000 (ms).
             * This property provides READ and WRITE access.
             * - Default value: 100 (ms)
             */
            property<NLed, uint32_t, propReadWrite>      Interval; // 100 (ms)

            /**
             * @brief This property defines the "ON-PHASE of blink interval", i.e. the time the LED remains ON during 'Interval'.
             * The "duty" time is given in percentage of the 'Interval' time (%).
             * This property provides READ and WRITE access.
             * - Default value: 50 (%)
             */
            property<NLed, uint32_t, propReadWrite>      Duty; // 50 (%)

            /**
             * @brief This property assigns a number of blinks the LED is forced execute.
             * This property is limited in the range of 2 to 100 (blinks).
             * This property provides READ and WRITE access.
             * - Default value: 0 (off)
             * @note After the assigned number of blinks is accomplished the last Led status is restored.
             */
            property<NLed, uint32_t, propReadWrite>      Burst;    // 0

    };

#endif
//==============================================================================
