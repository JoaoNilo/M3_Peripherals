//==============================================================================
/** @file NMco.h
 *  @brief MCU Output Clock (MCO) abstraction class.\n
 *  This class can be used to setup the MCO pin of the MCU.\n
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
#ifndef NMCO_H
    #define NMCO_H

	#include <stdint.h>
	#include "stm32f1xx.h"
    #include "Property.h"

	#define MCO_STANDARD_PORT	GPIOA
	#define MCO_STANDARD_PIN	8
	#define MCO_REMAPPED_PORT	GPIOA
	#define MCO_REMAPPED_PIN	8

    //-----------------------------------
	/**
	 * @enum mcoPinouts
	 * @brief This enumeration defines the options for the @ref NMco constructor.
	 */
    enum mcoPinouts { mcoStandard,		//!< standard output pin (PA8 for STM321xx)
    				  mcoRemapped		//!< alternate output pin (not available for this line)
    				};

	/**
	 * @enum mcoPinouts
	 * @brief This enumeration defines the options for the @ref Source property.
	 */
    enum mcoSources { mcoNone=0,		//!< no clock source
    				  mcoSysClk=4,		//!< System Clock
					  mcoHsi=5,			//!< High Speed Internal Oscillator
					  mcoHse=6,			//!< High Speed External Oscillator
					  mcoPll=7,			//!< Phase-Locked Loop (main)
					  mcoPll2=8,		//!< Phase-Locked Loop 2
					  mcoPll3=9,		//!< Phase-Locked Loop 3
					  mcoExternal=10,	//!< External clock
					  mcoPll3Eth=11		//!< Ethernet Phase-Locked Loop
    				};

    //-----------------------------------
	/** @brief MCU Output Clock (MCO) abstraction class.
	 *  - This class can be used to setup the MCO pin of the MCU.\n
	 */
    class NMco {

        private:
            
            //-------------------------
            mcoSources  source;
    
        protected:            
            bool enabled;

        //------------------------------
        private:

            //---------------------------------------
            void SetEnabled(bool);
            bool GetEnabled();
            void SetSource(mcoSources);
            mcoSources GetSource();
            
        protected:
            
        //--------------------------------------------
        public:
            //---------------------------------------
            // METHODS
            /**
             * @brief Standard constructor for this class.
             */
            NMco();

            /**
             * @brief Constructor for this class.
             * @arg Pinout: @ref mcoPinouts option to be used as clock output.
             */
            NMco(mcoPinouts Pinout);

            /**
             * @brief Standard destructor for this class.
             */
            ~NMco(void);

            //---------------------------------------
            // EVENTS

            //---------------------------------------
            // PROPERTIES               							< DEFAULT VALUES >
            /**
             * @brief This property can be used to enable and disable the clock output.\n
             * This property provides READ and WRITE access.\n
             * - Default value: false (disabled)
             */
            property<NMco, bool, propReadWrite> Enabled;            // false

            /**
             * @brief This property is used set the clock source to drive the MCO pin.
             * The available options are described in the @ref mcoSources enumeration description.\n
             * This property provides READ and WRITE access.\n
             * - Default value: @ref mcoNone
             */
            property<NMco, enum mcoSources, propReadWrite> Source;   // mcNone
    };

#endif
//==============================================================================
