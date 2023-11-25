//==============================================================================
/** @file NOutput.h
 *  @brief Output pin abstraction class.
 *  - This class provides resources to setup and use a general purpose IO pin as\n
 *  a standard output pin.
 *  @version 1.0.0
 *  @author Joao Nilo Rodrigues - nilo@pobox.com
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
#ifndef NOUTPUT_H
    #define NOUTPUT_H

    #include "DRV_IO.h"
	#include "NComponent.h"

    //-----------------------------------
	/**
	 * @enum ouLevels
	 * This enumeration defines the options for the @ref Level property.
	 */
    enum ouLevels    { ouLow,			//!< Output pin in low "logic level" (check @ref Mode)
    				  ouHigh,			//!< Output pin in high "logic level" (check @ref Mode)
					  ouTris			//!< Output pin in high impedance or tri-state.
    				};

	/**
	 * @enum ouAccesses
	 * @brief This enumeration defines the options for the @ref Access property.
	 */
    enum ouAccesses   { ouQueued,			//!< Changes to @ref Level will not update pin level instantly.
    				  ouImmediate			//!< Changes to @ref Level will update pin level instantly, but asynchronously.
    				};

    //-----------------------------------
	/**
	 *  @brief Output pin abstraction class.
	 *  - This component provides features to setup and use a general purpose IO pin as\n
	 *  a standard output pin.
	 */
    class NOutput: public NComponent{

        private:
            
            const uint32_t __PORT_WIDTH = 16;

            //-------------------------
            ouLevels  level;
            ouAccesses access;
            
            //-------------------------
            IO_Config Pino_InitStructure;
            GPIO_TypeDef* porta;
            
            uint32_t pino;
            ouLevels current_level;
    
        protected:            
            bool enabled;

        //------------------------------
        private:
            void WritePin(ouLevels);
              
            //---------------------------------------
            void SetEnabled(bool);
            bool GetEnabled();
            void SetLevel(ouLevels);
            ouLevels GetLevel();
            void SetAccess(ouAccesses);
            ouAccesses GetAccess();
            
        protected:
            void Refresh(ouLevels);
            
        //------------------------------
        public:
            //-------------------------------------------
            // METHODS
            /**
             * @brief Standard constructor for this component.
             */
            NOutput();

            /**
             * @brief Constructor for this component.
             * @arg Port: GPIO to be used (GPIOA, GPIOB, etc.)
             * @arg Pin: pin number (0 to 15)
             * @note Make sure the specified GPIO and pin number are valid for
             * the chosen target MCU.
             */
            NOutput(GPIO_TypeDef*, uint32_t);

            /**
             * @brief Standard destructor for this component.
             */
            ~NOutput(void);

            /**
             * @brief This method is used to invert the @ref Level property between toHigh and toLow.
             * @note Calling this method has no effect when @ref Level property is on toTris.
             */
            void Toggle();

            /**
             * @brief This method is used as a system callback function for message dispatching.
             */
            virtual void Notify(NMESSAGE*);

            /**
             * @brief This method is used as a system callback function for time critical
             * events dispatching. Not used by this component!
             */
            virtual void InterruptCallBack(NMESSAGE*);


            //---------------------------------------
            // PROPERTIES
            /**
             * @brief This property can be used to enable and disable the component and
             * physically, the current pin level remain unchanged.\n
             * This property provides READ and WRITE access.
             * - Default value: true (enabled)
             */
            property<NOutput, bool, propReadWrite> Enabled;

            /**
             * @brief This property is used to set the pin level. The available options are
             * detailed in the @ref ouLevels enumeration description, as well as some considerations
             * about the pin logic and physical levels.\n
             * This property provides READ and WRITE access.
             * - Default value: @ref ouLow
             */
            property<NOutput, enum ouLevels, propReadWrite> Level;

            /**
             * @brief This property sets the responsiveness of this component. The available options are
             * detailed in the @ref ouAccesses enumeration description.\n
             * This property provides READ and WRITE access.
             * - Default value: @ref ouQueued
             */
            property<NOutput, enum ouAccesses, propReadWrite> Access;

    };

#endif
//==============================================================================
