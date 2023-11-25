//==============================================================================
/**
 * @file NTinyOutput.h
 *  @brief Output pin abstraction class.
 *  - This class provides resources to setup and use a general purpose IO pin as\n
 *  a standard output pin.
 * @version 1.0.0
 * @author J. Nilo Rodrigues
 *
 *------------------------------------------------------------------------------
 *
 * <h2><center>&copy; Copyright (c) 2020 Joao Nilo Rodrigues </center></h2>
 * <h2><center> All rights reserved. </center></h2>
 *
 * This software component is licensed by "Joao Nilo Rodrigues" under BSD 3-Clause
 * license, the "License".
 * You may not use this file except in compliance with the License.
 *               You may obtain a copy of the License at:
 *                 opensource.org/licenses/BSD-3-Clause
 *
 *///---------------------------------------------------------------------------
#ifndef NTINYOUTPUT_H
    #define NTINYOUTPUT_H

	#include "DRV_IO.h"
    #include "Property.h"	

    //-----------------------------------
	/**
	 * @enum toLevels
	 * @brief This enumeration defines the options for the @ref Level property.
	 */
    enum toLevels    {	toLow,			//!< Output pin in low "logic level" (check @ref Mode)
    					toHigh,			//!< Output pin in high "logic level" (check @ref Mode)
						toTris			//!< Output pin in high impedance or tri-state.
    				};

	/**
	 * @enum toModes
	 * @brief This enumeration defines the options for the @ref Mode property.
	 */
    enum toModes    { 	toModeNormal,	//!< Output pin "logic level" equals "physical level"
    					toModeInverted	//!< Output pin "logic level" inverts "physical level"
    				};

	/**
	 * @enum toBiases
	 * @brief This enumeration defines the options for the @ref Bias property.
	 */
    enum toBiases   {   toFloating,		//!< No polarization connected to pin
    					toPullUp,		//!< Internal pull-up resistor connected to pin
						toPullDown		//!< Internal pull-down resistor connected to pin
    				};

    //------------------------------------------------
	/** @brief Simplified output pin abstraction class.
	 *  - This class provides features to setup and use a general purpose IO pin as\n
	 *  a standard output pin, consuming less system resources than NOutput.
	 */
    class NTinyOutput {

        private:
            
			#define __PORT_WIDTH		16

            //-------------------------
            toLevels  level;
            toModes  mode;
            toBiases bias;
           
            //-------------------------
            IO_Config Pino_InitStructure;
            GPIO_TypeDef* porta;
    
        protected:            
            bool enabled;

        //------------------------------
        private:
            void WritePin(toLevels);

            //---------------------------------------
            void SetEnabled(bool);
            bool GetEnabled();
            void SetLevel(toLevels);
            toLevels GetLevel();
            void SetMode(toModes);
            toModes GetMode();
            void SetBias(toBiases);
            toBiases GetBias();

        protected:
            void Refresh(toLevels);
            
        //------------------------------
        public:
            //-------------------------------------------
            // METHODS
            /**
             * @brief Standard constructor for this component.
             */
            NTinyOutput();

            /**
             * @brief Constructor for this component.
             * @arg Port: GPIO to be used (GPIOA, GPIOB, etc.)
             * @arg Pin: pin number (0 to 15)
             * @note Make sure the specified GPIO and pin number are valid for
             * the chosen target MCU.
             */
            NTinyOutput(GPIO_TypeDef* Port, uint32_t Pin);

            /**
             * @brief Standard destructor for this component.
             */
            ~NTinyOutput(void);

            /**
             * @brief This method is used to invert the @ref Level property between toHigh and toLow.
             * @note Calling this method has no effect when @ref Level property is on toTris.
             */
            void Toggle();            

            //---------------------------------------
            // EVENTS

            //---------------------------------------
            // PROPERTIES
            /**
             * @brief This property can be used to enable and disable the component and
             * physically, the current pin level remain unchanged.\n
             * This property provides READ and WRITE access.
             * - Default value: true (enabled)
             */
            property<NTinyOutput, bool, propReadWrite> Enabled;

            /**
             * @brief This property is used to set the pin level. The available options are
             * detailed in the @ref toLevels enumeration description, as well as some considerations
             * about the pin logic and physical levels.\n
             * This property provides READ and WRITE access.
             * - Default value: @ref toLow
             */
            property<NTinyOutput, enum toLevels, propReadWrite> Level;

            /**
             * @brief This property is used adjust logic pin level (internal) to some observable
             * (external) device connected to the pin (LED, Relay, etc.). The available options are
             * detailed in the @ref toModes enumeration description.\n
             * This property provides READ and WRITE access.
             * - Default value: @ref toModeNormal
             */
            property<NTinyOutput, enum toModes, propReadWrite> Mode;

            /**
             * @brief This property is used set the pin polarization.
             * The available options are described in the @ref toBiases enumeration description.\n
             * This property provides READ and WRITE access.
             * - Default value: @ref toPullUp
             */
            property<NTinyOutput, enum toBiases, propReadWrite> Bias;
    };

#endif
//==============================================================================
