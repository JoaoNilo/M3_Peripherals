//==============================================================================
/**
 * @file NInput.h
 * @brief Input pin abstraction class\n
 * This class provides resources to setup and use general purpose IO pin as\n
 * a standard input pin or an external interrupt pin.
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
#ifndef NINPUT_H
    #define NINPUT_H

    #include "DRV_IO.h"
    #include "NComponent.h"

//------------------------------------------------------------------------------
	/**
	 * @enum inLevels
	 * This enumeration defines the possible @ref Level property return values.
	 */
    enum inLevels        { inLow,			//!< Input pin in low "logic level"
    					   inHigh			//!< Input pin in high "logic level"
    					};


	/**
	 * @enum inAccesses
	 * @brief This enumeration defines the options for the @ref Access property.
	 */
    enum inAccesses     { inQueued,			//!< @ref Level property is updated periodically.
    					  inImmediate,		//!< @ref Level property is updated immediately, when read.
						  inInterrupt		//!< @ref Level property is updated by hardware interrupt, when pin level changes.
    					};

	/**
	 * @enum inEdges
	 * @brief This enumeration defines the options for the @ref Edges property.
	 */
    enum inEdges        { inNoEdge,			//!< Disables edge detection.
    					  inRisingEdge,		//!< Enables rising-edge detection.
						  inFallingEdge,	//!< Enables falling-edge detection.
						  inRiseAndFall		//!< Enables rising-edge and falling-edge detection.
    					};

	/**
	 * @enum inBiases
	 * @brief This enumeration defines the options for the @ref Bias property.
	 */
    enum inBiases       { inFloating = 0,	//!< Input pin not biased (high impedance).
    					  inPullDown = 2,	//!< Input pin is pulled down (internal resistor to ground).
						  inPullUp   = 3    //!< Input pin is pulled up (internal resistor to Vdd_io).
    					};

    //-----------------------------------
    /**
     * @brief Input pin abstraction class.
     * - This component provides resources to setup and use general purpose IO pin as\n
     * a standard input pin or an external interrupt pin.
     *
     */
    class NInput : public NComponent{

        private:
            //-------------------------
            inLevels level;
            inAccesses access;
            inEdges  edge;
			bool inverted;

            //-------------------------
            IO_Config Pino_InitStructure;
            HANDLE PreviousVector;

            //-------------------------------------------
            inLevels GetLevel();
            void SetAccess(inAccesses);
            inAccesses GetAccess();
            void SetEdge(inEdges);
            inEdges GetEdge();
            void SetInverted(bool);
            bool GetInverted();
            void SetBias(inBiases);

        protected:
            GPIO_TypeDef* porta;

            //-------------------------
            void SetupInterrupt();
            inLevels ReadPin();
            NV_ID GetVectorIndex(uint32_t);

        //-----------------------------------------------
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
            NInput(GPIO_TypeDef* Port, uint32_t Pin);

            /**
             * @brief Standard destructor for this component.
             */
            virtual ~NInput(void);

            /**
             * @brief This method is used as a system callback function for message dispatching.
             */
            virtual void Notify(NMESSAGE*);

            /**
             * @brief This method is used as a system callback function for time critical
             * events dispatching.
             */
            virtual void InterruptCallBack(NMESSAGE*);

            /**
             * @brief This method is used to return the pin level.
             * This method forces an physical read of the input pin and updates
             * the internal status. If any level changes are detected, the corresponding event handler is called.
             * @return Check @ref inLevels for details on the possible return values.
             */
            inLevels ReadInput();

            //-------------------------------------------
            // EVENTS
            /**
             * @brief This is the event handler for level changes.
             * - This event handler is called every time a level change is detected.
             * @note This event is not related to @ref Edge or @ref Access properties.
             */
            void (*OnLevelChange)(void);

            /**
             * @brief This is the event handler for "low to high" level changes.
             * - This event handler is called when the pin level goes high and the @ref Edge property
             * is assigned inRisingEdge or inRiseAndFall.
             * @note This event is not related to @ref Access property.
             */
            void (*OnRisingEdge)(void);

            /**
             * @brief This is the event handler for "high to low" level changes.
             * - This event handler is called when the pin level goes low and the @ref Edge property
             * is assigned inFallingEdge or inRiseAndFall.
             * @note This event is not related to @ref Access property.
             */
            void (*OnFallingEdge)(void);

            //-------------------------------------------
            // PROPERTIES
            /**
             * @brief As the name suggests, this property can be used to enable/disable the input pin.
             * When the Enabled property is set to false, the @ref Level property is no longer updated and
             * none of the events are fired.\n
             * This property provides READ and WRITE access.\n
             * - Default value: true
             */
            bool Enabled;

            /**
             * @brief This property is used to check the pin level.
             * The possible return values are given by the @ref inLevels enumeration.\n
             * This property provides READ ONLY access.
             * - Default value: ? (unknown)
             */
            property<NInput, enum inLevels, propRead> Level;         	// (?)

            /**
             * @brief This property sets the responsiveness of this component. The available options are
             * detailed in the @ref inAccesses enumeration description.\n
             * This property provides READ and WRITE access.
             * - Default value: @ref inQueued
             */
            property<NInput, enum inAccesses, propReadWrite> Access;

            /**
             * @brief This property is used to select the "active edges".
             * In this context edges are level transitions and the available options are
             * detailed in the @ref inEdges enumeration description.\n
             * This property provides READ and WRITE access.
             * - Default value: @ref inFallingEdge
             */
            property<NInput, enum inEdges, propReadWrite> Edge;     	// inFallingEdge

            /**
             * @brief This property is used adjust the logic level (internal) to the observable
             * (external) status of the connected device (switch, push-button, etc.).\n
             * This property provides READ and WRITE access.
             * - Default value: false
             */
            property<NInput, bool, propReadWrite> Inverted;

            /**
             * @brief This property is used select one of the bias options available, according to
             * specific hardware needs. The options are described in the @ref inBiases enumeration.\n
             * This property provides READ and WRITE access.
             * - Default value: @ref inFloating
             */
            property<NInput, enum inBiases, propReadWrite> Bias;

    };

#endif
//==============================================================================
