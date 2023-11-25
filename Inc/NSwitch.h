//==============================================================================
/**
 * @file NSwitch.h
 * @brief Switch abstraction driver class\n
 * This class provides resources to setup and use general purpose IO pin as\n
 * a "Switch" (permanent state) or "Push-Button" (momentary state).
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
 *
 *///------------------------------------------------------------------------------
#ifndef NSWITCH_H
    #define NSWITCH_H

    #include "NInput.h"

    //-----------------------------------
	/**
	 * @enum swStatus
	 * @brief This enumeration defines the possible return values for the @ref Status property.
	 */
    enum swStatus { swReleased = 0,			//!< Push-button is released
    	            swOff = 0,				//!< Switch is OFF (IO pin is low)
					swPressed = 1,			//!< Push-button is pressed
					swOn = 1				//!< Switch is ON (IO pin is high)
    			  };

	/**
	 * @enum swModes
	 * @brief This enumeration defines the options for the @ref Mode property.
	 */
    enum swModes { swButton,				//!< Button: momentary contact
    	           swSwitch					//!< Switch: permanent contact
    			 };

	/**
	 * @enum swDebouncings
	 * @brief This enumeration defines the options for the @ref Debouncing property.
	 */
    enum swDebouncings { swShort,				//!< short de-bouncing count, takes ~2ms
    	          	     swStandard,			//!< standard de-bouncing count, takes ~7ms
					     swLong					//!< standard de-bouncing count ~12ms
    				   };

    //-----------------------------------
    /** @brief Switch abstraction class\n
     *  - This component provides features for configuring and using a general purpose IO pin as\n
     *  an abstraction for a switch or push-button connected to it.
     */
    class NSwitch : private NInput{

        private:
			//#define __PORT_WIDTH   		16
			#define __DEFAULT_DEBOUNCE  2

            //-------------------------
            swStatus status;
            swModes mode;
            uint32_t debounce;

            //-------------------------
            uint32_t change_counter;
            swStatus pin_status;

            //-------------------------
            void SetStatus(swStatus);
            swStatus GetStatus();
            void SetMode(swModes);
            swModes GetMode();
            swDebouncings GetDebouncing();
            void SetDebouncing(swDebouncings);

        //-------------------------------------------
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
            NSwitch(GPIO_TypeDef* Port, uint32_t Pin);

            /**
             * @brief Standard destructor for this component.
             */
            virtual ~NSwitch(void);

            /**
             * @brief This method is used as a system callback function for message dispatching.
             */
            virtual void Notify(NMESSAGE*);

            //---------------------------------------
            // EVENTS
            /**
             * @brief This is the event handler for state changes.
             * - This event handler is called every time a state change is detected.
             * @note This event can be used when @ref swSwitch mode is selected.
             */
            void (*OnChange)(void);

            /**
             * @brief This is the event handler for "button press".
             * @note This event can be used when @ref swButton mode is selected.
             */
            void (*OnPress)(void);

            /**
             * @brief This is the event handler for "button release".
             * @note This event can be used when @ref swButton mode is selected.
             */
            void (*OnRelease)(void);

            //---------------------------------------
            // PROPERTIES
            using NComponent::Tag;

            /**
             * @brief This property is used select one of the bias options available, according to
             * specific hardware needs. The options are described in the @ref inBiases enumeration.\n
             * This property provides READ and WRITE access.
             * - Default value: @ref inFloating
             */
			using NInput::Bias;

            /**
             * @brief This property is used adjust the logic level (internal) to the observable
             * (external) status of the connected device (switch, push-button, etc.).\n
             * This property provides READ and WRITE access.
             * - Default value: true
             */
			using NInput::Inverted;

            /**
             * @brief As the name suggests, this property can be used to enable/disable the component.
             * When the Enabled property is set to false, the @ref Status property is no longer updated and
             * none of the events are fired.\n
             * This property provides READ and WRITE access.\n
             * - Default value: true
             */
            bool Enabled;

            /**
             * @brief This property is used to check the pin level (switch/button status).
             * The possible return values are given by the @ref swStatus enumeration.\n
             * This property provides READ ONLY access.
             * - Default value: ? (unknown)
             */
            property<NSwitch, enum swStatus, propRead> Status;

            /**
             * @brief This property selects the "abstracted device" for this component. The available options are
             * detailed in the @ref swStyles enumeration description.\n
             * This property provides READ and WRITE access.
             * - Default value: @ref swButton
             */
            property<NSwitch, enum swModes, propReadWrite> Mode;

            /**
             * @brief This property selects de-bouncing "effort" to provide safe transition detections.
             * - Normally, switches and buttons have noisy state transitions due to mechanical characteristics,
             * sometimes resulting in bursts of high frequency spikes known as bouncing effect.
             * This property provides READ and WRITE access.
             * - Default value: @ref swStandard
             */
            property<NSwitch, swDebouncings, propReadWrite> Debouncing;

    };

#endif
//==============================================================================
