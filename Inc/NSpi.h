//==============================================================================
/**
 * @file NSpi.h
 * @brief Serial Peripheral Interface (SPI) abstraction class\n
 * - This class provides resources to setup and use SPI as synchronous\n
 * communication interfaces.
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
#ifndef NSPI_H
    #define NSPI_H

    #include "DRV_DMA.h"
    #include "NComponent.h"

	//--------------------------------------------------------------------------
	/**
	 * @enum spRemapping
	 * @brief Predefined options for the SPI pinout:
	 *
	 *  SPI1 pinout options
	 *  Standard: SCK:PA5, MISO: PA6, MOSI: PA7\n
	 * 	Remapped: SCK:PB3, MISO: PB4, MOSI: PB5\n
	 *
	 *  SPI2 pinout options
	 *  Standard: SCK:PB13, MISO: PB14, MOSI: PB15\n
	 *
	 *  SPI3 pinout options
	 *  Standard: SCK:PB3, MISO: PB4, MOSI: PB5\n
	 *  Remapped: SCK:PC10, MISO: PC11, MOSI: PC12\n
	 */
	enum spRemapping {  spStandard,			//!< Standard pinout
    					spRemapped,			//!< Remapped (alternate) pinout
					 };
    
	/**
	 * @brief spClockRates
	 *
	 *  These are the set of values available for the ClockRate property.\n
	 *  Basically, it defines the prescaler to be used by the SPI engine to\n
	 *  generate clock, when configured in "master" mode.\n
	 *  @note It is worth to recall that the resulting clock frequency will be\n
	 *  the corresponding APB Bus frequency divided by the selected fraction below.\n
	 */
	enum spClockRates {  spPCLK_Div2,		/**< APB clock freq. divided by 2*/
						 spPCLK_Div4,		/**< APB clock freq. divided by 4*/
						 spPCLK_Div8,		/**< APB clock freq. divided by 8*/
						 spPCLK_Div16,		/**< APB clock freq. divided by 16*/
						 spPCLK_Div32,		/**< APB clock freq. divided by 32*/
						 spPCLK_Div64,		/**< APB clock freq. divided by 64*/
						 spPCLK_Div128,		/**< APB clock freq. divided by 128*/
						 spPCLK_Div256		/**< APB clock freq. divided by 256*/
					  };
    
    enum spTransferModes {  spNoDMA,		/**< Data transfer without DMA*/
    						spDMA			/**< Data transfer will use DMA*/
    					 };

    enum spClockPolarities { sp_CPol0_CPha0, /**< Clock polarity 0, clock phase 0*/
    						 sp_CPol0_CPha1, /**< Clock polarity 0, clock phase 1*/
							 sp_CPol1_CPha0, /**< Clock polarity 1, clock phase 0*/
							 sp_CPol1_CPha1  /**< Clock polarity 1, clock phase 1*/
    					   };

    //----------------------------------------
    /**
     * @brief Serial Peripheral Interface (SPI) abstraction class
     * - This class provides resources to setup and use SPI as synchronous\n
     * communication interfaces.
     */
    class NSpi: public NComponent{
        private:
			bool MasterTxInProgress;
			bool MasterRxInProgress;
			uint32_t ControlRegister1;
			uint32_t ControlRegister2;
			uint32_t bytes_to_write;
			uint32_t bytes_written;
			uint32_t bytes_read;
			void* bytes_pointer;

			SPI_TypeDef* SPIx;
			DMA_Channel_TypeDef * DMA_TxChannel;
			DMA_Channel_TypeDef * DMA_RxChannel;
			uint32_t SPIx_IRQn;
			uint32_t IRQChannelTx;
			uint32_t IRQChannelRx;
			uint16_t vector_index;
			uint16_t vector_dma_tx;
			uint16_t vector_dma_rx;
			spClockRates baud;

        protected:
            bool AssistedReception;                                 // false
            bool AssistedTransmission;                              // false

        //-------------------------------------------
        private:
            void Pinout(uint32_t);
            SPI_TypeDef* GetPort();
			spClockRates GetClockRate();
			void SetClockRate(spClockRates);
			void SetClockPolarity(spClockPolarities);
			spClockPolarities GetClockPolarity();

        //-------------------------------------------
		protected:
			bool Write(void*, uint32_t);
			bool Read(void*, uint32_t);
			bool WriteNoDMA(void*, uint32_t);
			bool WriteDMA(void*, uint32_t);
			bool ReadNoDMA(void*, uint32_t wlen);
			bool ReadDMA(void*, uint32_t);
		
        //-------------------------------------------
        public:
            //-------------------------------------------
            // METHODS
            /**
             * @brief Constructor for this component.
             * @arg SPIn: SPI interface to be used (SPI1, SPI2 or SPI3 when available)
             * @arg Pinout: check the @ref spRemapping enumeration for predefined pinout options.\n
             * @note Be sure to specify a valid SPI, available on the target MCU and
             * not in use by the application.
             */
            NSpi(SPI_TypeDef* SPIn, spRemapping Pinout);

            /**
             * @brief Standard destructor for this component.
             */
            ~NSpi(void);

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
             * @brief This method is used to start the data communication using the component.
             * - Before this method is called, the read and write methods will not work as well as\n
             * no events will be fired. All the potential properties adjusts, event handlers assignments,\n
             * etc., might be done before calling this method.
             */
            void Open();

            /**
             * @brief This method is used to stop the data communication.
             * - This method restores the component´s condition before the Open method was called.
            */
            void Close();

            /**
             * @brief This method is used to send a block 8-bit data through the SPI interface.
             * - This method can be used with or without DMA. Check the @ref TxMode property\n
             * for further details.
             * @arg ptExtData: pointer to the buffer with data bytes to be sent.
             * @arg szExtData: number of data bytes in the buffer (packet size).             *
             * @return  depends on the @ref Blocking property:\n
             *  - Blocking = false: return true if the data transfer <b>started</b> successfully\n
             *  - Blocking = true: return true if the data transfer <b>completed</b> successfully\n
            */
			bool Write(uint8_t* ptExtData, uint32_t szExtData);

            /**
             * @brief This method is used to read a block of 8-bit data through the SPI interface.
             * - This method can be used with or without DMA. Check the @ref RxMode property\n
             * for further details.
             * @arg ptExtData: pointer to a buffer to receive the data bytes.
             * @arg szExtData: number of data bytes to read an store in the buffer.
             * @return  depends on the @ref Blocking property:\n
             *  - Blocking = false: return true if the data transfer <b>started</b> successfully\n
             *  - Blocking = true: return true if the data transfer <b>completed</b> successfully\n
            */
			bool Read(uint8_t* ptExtData, uint32_t szExtData);

            /**
             * @brief This method is used to send a block 16-bit data through the SPI interface.
             * - This method can be used with or without DMA. Check the @ref TxMode property\n
             * for further details.
             * @arg ptExtData: pointer to the buffer with data words to send.
             * @arg szExtData: number of data words in the buffer (packet size).             *
             * @return  depends on the @ref Blocking property:\n
             *  - Blocking = false: return true if the data transfer <b>started</b> successfully\n
             *  - Blocking = true: return true if the data transfer <b>completed</b> successfully\n
            */
			bool Write(uint16_t*, uint32_t);

            /**
             * @brief This method is used to read a block of 16-bit data through the SPI interface.
             * - This method can be used with or without DMA. Check the @ref RxMode property\n
             * for further details.
             * @arg ptExtData: pointer to a buffer to receive the data words.
             * @arg szExtData: number of data words to read an store in the buffer.
             * @return  depends on the @ref Blocking property:\n
             *  - Blocking = false: return true if the data transfer <b>started</b> successfully\n
             *  - Blocking = true: return true if the data transfer <b>completed</b> successfully\n
            */
			bool Read(uint16_t*, uint32_t);

            //---------------------------------------
            // EVENTS
            /**
             * @brief This event is triggered when any communication error is detected.\n
             * @arg errCode: error flags when the fault occurred.\n\n
             * <KBD>    MSB ................................. LSB</KBD>\n
             * <KBD>   | 00 | OVR| 00 | CRC| 00 | 00 | 00 | 00 |</KBD>\n\n
             *
             * <p>
             * Flags meaning:
             * - OVR: Overrun Error
             * - CRC: CRC Error
             *</p>
             *<p>
             * For detailed information, check SPI_SR in the datasheet.
             *</p>
             */
            void (*OnError)(uint32_t errCode);

            /**
            * @brief This event notifies the completion of the @ref Read operation.
            * <p>
            * As a non-blocking communication component, the Read method just starts the\n
            * read process of given number of data bytes (or words).\n
            * When the expected data is finally received, this event is fired.
            * </p>
            */
            void (*OnReadCompleted)(void);

            /**
            * @brief This event notifies the completion of the @ref Write operation.
            * <p>
            * As a non-blocking communication component, the Write method just starts the transmission of \n
            * certain amount of data.\n
            * When all the specified data is finally sent this event is then triggered.
            * </p>
            */
            void (*OnWriteCompleted)(void);

            //---------------------------------------
            // PROPERTIES
            /**
             * @brief This property can is used to enable the "blocking mode" for both @ref Read\n
             * and @ref Write operations.\n
             * When set to false, the program execution continues after these functions are called,\n
             * and when finished, the corresponding notification event (@ref OnReadCompleted or\n
             * @ref OnWriteCompleted) is fired.\n
             *
             * This property provides READ and WRITE access.\n
             * - Default value: true
             */
            bool Blocking;

            /**
             * @brief This property enables or disables the use of a DMA for @ref Read operations.
             * The possible values for this property can be found in the @ref spTransferModes.\n
             * This property provides READ and WRITE access.\n
             * - Default value: @ref spNoDMA
             */
            spTransferModes ModeRx;

            /**
             * @brief This property enables or disables the use of a DMA for @ref Write operations.
             * The possible values for this property can be found in the @ref spTransferModes.\n
             * This property provides READ and WRITE access.\n
             * - Default value: @ref spNoDMA
             */
            spTransferModes ModeTx;

            /**
             * @brief This property can be used to access the SPI handler.
             * The SPI to be used is passed to the component´s constructor when instantiating.\n
             * The assignment cannot be changed after the component is instantiated.\n
             * This property allows the implementation of some feature not offered by the component.
             * This property provides READ ONLY access.\n
             * - Default value: unknown (depends on the chosen SPI, passed to the constructor)
             */
            property<NSpi, SPI_TypeDef*, propRead> Port;

            /**
             * @brief This property is used to select the SPI clock "polarity" and clock "phase".
             * Polarity defines what level the SCK signal should remain at when idle and Phase\n
             * defines which edge of the SCK signal is used to sample the input data,\n
             * at the MISO input.\n
             * The possible values for this property can be found in the @ref spClockPolarities.\n
             * This property provides READ and WRITE access.\n
             * - Default value:  @ref sp_CPol1_CPha1
             */
			property<NSpi, spClockPolarities, propReadWrite> ClockPolarity;

            /**
             * @brief This property is used to set the SPI bit rate.
             * The value of this property is specified as fractions of the APB frequency.\n
             * The possible values for this property can be found in the @ref spClockRates.\n
             * This property provides READ and WRITE access.\n
             * - Default value: @ref spPCLK_Div8
             */
			property<NSpi, spClockRates, propReadWrite> ClockRate;
};

#endif
//==============================================================================
