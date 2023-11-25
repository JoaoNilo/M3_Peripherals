//==============================================================================
/**
 * @file NSerial.h
 * @brief Asynchronous Serial Interface abstraction class\n
 * - This class provides resources to setup and use USARTS as asynchronous\n
 * serial communication interfaces.
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
#ifndef NSERIAL_H
    #define NSERIAL_H

	#include "NComponent.h"

	#ifdef STM32F10X_HD
		#define DMA2_Channel5_IRQn		DMA2_Channel4_5_IRQn
	#endif


	class NSERIAL_BUFFER;
    
    //--------------------------------------------------------------------------
	/**
	 * @enum seRemapping
	 * @brief Predefined options for the USART pinout:
	 *
	 *  USART1 pinout options\n
	 *  Standard: TX: PA9, RX: PA10, RTS: PA12, CTS: PA11\n
	 * 	Remapped: TX: PB6, RX: PB7, RTS: NONE, CTS: NONE\n\n
	 *
	 *  USART2 pinout options\n
	 *  Standard: TX: PA2, RX: PA3, RTS: PA1, CTS: PA0\n
	 * 	Remapped: TX: PD5, RX: PD6, RTS: NONE, CTS:NONE\n\n
	 *
	 *  USART3 pinout options\n
	 *  Standard: TX: PB10, RX: PB11, RTS: PB14  CTS: PB13\n
	 * 	Remapped: TX: PD8, RX: PD9, RTS: PD12, CTS: PD11\n\n
	 *
	 */
	enum seRemapping {  seStandard,			//!< Standard pinout
						seRemapped			//!< Remapped (alternate) pinout
					 };

	/**
	 * @enum seParities
     * @brief This enumeration defines the options for the @ref Parity property.
     * For detailed information on using parity, see the datasheet.
	 */
    enum seParities {   seNoParity=0,			//!< parity not used
    					seParityEven=2,			//!< even parity in use
						seParityOdd=3			//!< odd parity in use
    				};

	/**
	 * @enum seStopBits
     * @brief This enumeration provides the options for the @ref StopBits property.
	 */
    enum seStopBits {	seStop1 = 0,			//!< 1 stop bit
    					seStop05 = 1,			//!< 0.5 stop bit
						seStop2 = 2,			//!< 2 stop bits
						seStop15 = 3			//!< 1.5 stop bits
    				};

	/**
	 * @enum seWordLengths
     * @brief This enumeration provides the options for the @ref WordLength property.
     */
    enum seWordLengths {seBits8,				//!< 8 bits data
    					seBits9					//!< 9 bits data
    				   };

	/**
	 * @enum seTransferModes
     * @brief This enumeration provides the options for the @ref RxMode and @ref TxMode properties.
     *<p>
     * @ref seNoDMA:
     * - Pros: Less system resources (no DMA channel allocation)
     * - Cons: Performance is compromised due to higher interrupt rate.
     *</p>
     *<p>
     * @ref seDMA:
     * - Pros: Increased overall efficiency (fewer interruptions)
     * - Cons: More system resources (DMA channel allocation)\n
     *         Not available for all serial interfaces (check datasheet).
     *</p>
     *<p>
     * @ref seOff:
     * - Data communication is disabled.
     *</p>
     */
    enum seTransferModes {	seNoDMA,			//!< data transfer without DMA
    						seDMA,				//!< data transfer using DMA
							seOff				//!< data transfer disabled
    					 };

	/**
	 * @enum seFlowControls
     * @brief This enumeration provides the options for the @ref FlowControl property.
     * @note If flow control is used, the additional pins (RTS/CTS) must be available in the hardware.
     */
    enum seFlowControls { 	seFlowNone,			//!< flow control disabled
    						seFlowRTS,			//!< only "request to send" (reception)
							seFlowCTS,			//!< only "clear to send" (transmission)
							seFlowFull			//!< full flow control (tx / rx)
    					};

    //--------------------------------------------------------------------------
    /**
     * @brief Asynchronous Serial Interface abstraction class
     * - This component provides resources to setup and use USARTS as asynchronous\n
     * serial communication interfaces.
     */
    class NSerial: public NComponent{

		#define PACKET_TIMEOUT_MAX				((uint32_t) 1000)
		#define TRANSMISSION_TIMEOUT_MAX 		((uint32_t) 1000)


		protected:
			USART_TypeDef* USARTx;
            void RestartPacket();
            bool parent_rx;
            bool parent_tx;
            
        private:
            //-------------------------
            uint32_t device_pinout;
            uint32_t baud;
            seWordLengths wordlength;
            seStopBits stopbits;
            seParities parity;
            seFlowControls flowcontrol;
            seTransferModes rxmode;
            seTransferModes txmode;
            uint32_t packetsize;

            uint16_t vector_index;
			uint16_t vector_dma_tx;
			uint16_t vector_dma_rx;

            //-------------------------
            IRQn_Type   USARTx_IRQn;
            DMA_Channel_TypeDef* DMAChannelTx;
            DMA_Channel_TypeDef* DMAChannelRx;

            uint32_t LastSize;
            uint32_t IRQChannelTx;
            uint32_t IRQChannelRx;

            //-------------------------
            bool flagReceiving;
            int32_t receiving_counter;
            int32_t reception_timeout;
            int32_t packet_timeout;

            //-------------------------
			bool flagTransmitting;
			int32_t transmitting_counter;
			int32_t transmission_timeout;

            //-------------------------
            bool flagReading;
            bool flagWriting;
			
            //-------------------------
            bool pacoteFinalizado;
            bool pacoteInicializado;
            NSERIAL_BUFFER* PacoteLocalRx;
            NSERIAL_BUFFER* PacoteDisponivelRx;

            //-------------------------
            NSERIAL_BUFFER *PacoteLocalTx;
            NSERIAL_BUFFER* PacoteDisponivelTx;
            bool busy;

        //-------------------------------------------
        private:
            void Pinout(uint32_t);
            void SetDmaReception(uint8_t*, uint32_t);
            void SetDmaTransmission(uint8_t*, uint32_t);
            bool ReloadTxFifo();
            void StartTimeout(uint32_t);
            void StopTimeout();
            bool PackData(uint8_t);

            //---------------------------------------
            bool GetIsOpen();
            void SetBaudRate(uint32_t);
            uint32_t GetBaudRate();
            void SetWordLength(seWordLengths);
            seWordLengths GetWordLength();
            void SetParity(seParities);
            seParities GetParity();
            void SetStopBits(seStopBits);
            seStopBits GetStopBits();
            void SetRxMode(seTransferModes);
            seTransferModes GetRxMode();
            void SetTxMode(seTransferModes);
            seTransferModes GetTxMode();
            void SetFlowControl(seFlowControls);
            seFlowControls GetFlowControl();
            void SetPacketSize(uint32_t);
            uint32_t GetPacketSize();
            void SetTimeout(uint32_t);
            uint32_t GetTimeout();
            USART_TypeDef* GetPort();
            void SetTransmissionTimeout(uint32_t);
            uint32_t GetTransmissionTimeout();
            void SetBufferSizeRx(uint32_t);
            uint32_t GetBufferSizeRx();
            void SetBufferSizeTx(uint32_t);
            uint32_t GetBufferSizeTx();

        protected:
			virtual bool PacketNotification(uint8_t*, uint16_t);
			virtual void DiscardNotification(uint8_t*, uint16_t);

        //-------------------------------------------
        public:
            //-------------------------------------------
            // METHODS
            /**
             * @brief Constructor for this component.
             * @arg USARTn: USART to be used (USART1, USART2, UART4, etc)
             * @arg Pinout: check the @ref seRemapping enumeration for predefined pinout options.\n
             * @note Be sure to specify a valid USART, available on the target MCU and
             * not in use by the application.
             */
            NSerial(USART_TypeDef* USARTn, seRemapping Pinout);

            /**
             * @brief Standard destructor for this component.
             */
            ~NSerial(void);

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
             * @brief This method is used to clear the internal buffers and registers.
             * - This method is commonly used by the component itself to recover from communication errors, but can be used by the application as well.
            */
            void Flush();

            /**
             * @brief This method can be used to start a "timed" data reception.
             * - This method can be useful when implementing "master-side" communication, as the master\n
             * must wait for a response from the slave for a certain period of time, right after the transfer\n
             * of the data packet from the master.
             * @arg szReadData: number of bytes to read
             * @arg smTimeout: time to wait for the whole data packet (in milliseconds)
            */
            void Read(uint8_t szReadData, uint32_t msTimeout);

            /**
             * @brief This method can be used to send data through the serial interface.
             * - Internally, this method can operate with or without DMA. Check the @ref TxMode property\n
             * for further details.
             * @arg ptExtData: pointer to the buffer with data to be sent.
             * @arg szExtData: number of data bytes in the buffer (packet size).
             * @return  true if the data transfer <b>started</b> successfully
            */
            bool Write(uint8_t* ptExtData , uint32_t szExtData);

            //-------------------------------------------
            // EVENTS
            /**
             * @brief This event is triggered when any communication error is detected.\n
             * @arg errCode: error flags when the fault occurred.\n\n
             * <KBD>    MSB ................................. LSB</KBD>\n
             * <KBD>   | 00 | 00 | 00 | 00 | ORE | NE | FE | PE |</KBD>\n\n
             *
             * <p>
             * Flags meaning:
             * - PE: Parity error
             * - FE: Framing error
             * - NE: Noise error
             * - ORE: Overrun error
             *</p>
             */
            void (*OnError)(uint16_t errCode);

            /**
             * TODO: analyze all scenarios that triggers timeout event.
             * @brief This event is triggered when the data reception exceeds the specified timeout interval.
             * <p>
             *  This timeout event can occur in the following situations:\n
             *  - After calling the @ref Read method and no data received within the specified interval.
             *  - When Timeout property is not 0 and the data packet received is smaller than the specified in @ref PacketSize property.
             *  </p>
             */
            void (*OnTimeout)(void);

            /**
             * @brief This event is triggered when new data packet is received.
             * <p>
             * When the number of received bytes reaches the @ref PacketSize number, this event is fired.
             *  @arg ptRecData: pointer to a local buffer with the received data
             *  @arg szRecData: number of bytes in the buffer
             *  </p>
             */
            void (*OnPacket)(uint8_t* ptRecData, uint8_t szRecData);

            /**
            * @brief This event is triggered when any change in the CTS line is detected.
            * <p>
            * It is important to note that the @ref FlowControl property must be set to either @ref seFlowCts \n
            * or @ref seFlowFull and the selected pinout (passed to the @ref NSerial constructor) provides \n
            * access to the input pin to be used as CTS.
            * </p>
            */
            void (*OnCTS)(void);

            /**
            * @brief This event is triggered when receiver line remains stable.
            * <p>
            * The receiver line is considered idle when no data transfer is detected for a period of time\n
            * longer than 1 byte time, which is baud rate dependent.
            * </p>
            */
			void (*OnIdleDetected)(void);

            /**
            * @brief This event notifies the completion of the @ref Read operation.
            * <p>
            * As a non-blocking communication component, the Read method just starts the reception of certain\n
            * amount of data for a specific time interval.\n
            * When the expected data is received this event is fired to provide access to this new data.
            * @arg ptRecData: pointer to a local buffer with the received data
            * @arg szRecData: number of bytes in the buffer
            * </p>
            */
            void (*OnReadCompleted)(uint8_t* ptRecData, uint8_t szRecData);

            /**
            * @brief This event notifies the completion of the @ref Write operation.
            * <p>
            * As a non-blocking communication component, the Write method just starts the transmission of \n
            * certain amount of data.\n
            * When all the specified data is finally sent this event is then triggered.
            * </p>
            */
            void (*OnWriteCompleted)(void);

            /**
            * @brief This event right before the start of any data transmission.
            * <p>
            * This event can be useful when half-duplex transceiver have to be "manually" switched between\n
            * transmission-mode and reception-mode.
            * </p>
            */
            void (*OnEnterTransmission)(void);

            /**
            * @brief This event right after the completion of any data transmission.
            * <p>
            * This event can be useful when half-duplex transceiver have to be "manually" switched between\n
            * transmit mode and receive mode.
            * </p>
            */
            void (*OnLeaveTransmission)(void);

            //--------------------------------------
            // PROPERTIES
            /**
             * @brief This property can be used to check is the communication is open.
             * The "open" state is initiate by calling the @ref Open method and finished by calling @ref Close.
             * This property provides READ ONLY access.\n
             * - Default value: false
             */
			property<NSerial, bool, propRead> IsOpen;

            /**
             * @brief This property can be used to access the USART handler.
             * The USART to be used is passed to the component´s constructor when instantiating.\n
             * The assignment cannot be changed after the component is instantiated.\n
             * This property allows the implementation of some feature not offered by the component.
             * This property provides READ ONLY access.\n
             * - Default value: unknown (depends on the chosen USART, passed to the constructor)
             */
            property<NSerial, USART_TypeDef*, propRead> Port;

            /**
             * @brief This property is used to set the USART baud rate.
             * The value of this property is specified is bits per second or bps.\n
             * This property provides READ and WRITE access.\n
             * - Default value: 115200 (bps)
             */
            property<NSerial, uint32_t, propReadWrite> BaudRate;

            /**
             * @brief This property is used to set the number of data bits in a serial frame.
             * The possible values for this property can be found in the @ref seWordLengths.\n
             * It is important to note that this number should include the parity bit, if used.\n
             * This property provides READ and WRITE access.\n
             * - Default value: @ref se8Bits
             */
            property<NSerial, enum seWordLengths, propReadWrite> WordLength;

            /**
             * @brief This property is used to set the parity bit of the serial frame.
             * The possible values for this property can be found in the @ref seParities.\n
             * It is important to note that changes in the parity bit may need word length adjustment.\n
             * This property provides READ and WRITE access.\n
             * - Default value: @ref seNoParity
             */
            property<NSerial, enum seParities, propReadWrite> Parity;

            /**
             * @brief This property is used to set the number of stop bits of the serial frames.
             * The possible values for this property can be found in the @ref seStopBits.\n
             * This property provides READ and WRITE access.\n
             * - Default value: @ref seStop1
             */
            property<NSerial, enum seStopBits, propReadWrite> StopBits;

            /**
             * @brief This property enables or disables the use of a DMA for data reception.
             * The possible values for this property can be found in the @ref seTransferModes.\n
             * This property provides READ and WRITE access.\n
             * - Default value: @ref seDMA
             */
            property<NSerial, enum seTransferModes, propReadWrite> RxMode;      // seDMA

            /**
             * @brief This property enables or disables the use of a DMA for data transmission.
             * The possible values for this property can be found in the @ref seTransferModes.\n
             * This property provides READ and WRITE access.\n
             * - Default value: @ref seDMA
             */
            property<NSerial, enum seTransferModes, propReadWrite> TxMode;      // seDMA

            /**
             * @brief This property is used to set the data flow control or handshake.
             * The possible values for this property can be found in the @ref seFlowControls.\n
             * This property provides READ and WRITE access.\n
             * - Default value: @ref seNoFlowControl
             */
            property<NSerial, enum seFlowControls, propReadWrite> FlowControl;  // seNoFlowControl

            /**
             * @brief This property is used to set the number of bytes needed to trigger the OnPacket event.
             * The value of this property is limited in the range of 1 to @BufferSizeRx.\n
             * This property provides READ and WRITE access.\n
             * - Default value: 1 (byte)
             */
            property<NSerial, uint32_t, propReadWrite> PacketSize;

            /**
             * @brief This property is used to set the time to wait for @ref PacketSize bytes.
             * It is important to know that this timeout is started as the first byte of the\n
             * packet is received. If the entire packet is not received until timeout expires,\n
             * an @ref OnTimeout event is fired.
             * The value of this property is limited in the range of 0 to 1000 (milliseconds).\n
             * This property provides READ and WRITE access.\n
             * - Default value: 0 (milliseconds)
             */
            property<NSerial, uint32_t, propReadWrite> Timeout;

            /**
             * @brief This property sets the @ref Write expiration timeout.
             * This property defines the time to wait for a write operation to complete.\n
             * If the last write operation does not complete until this timeout expires,\n
             * the write operation is canceled and the @ref OnLeaveTransmission event is fired.\n
             * The value of this property is limited in the range of 0 to 1000 (milliseconds).\n
             * This property provides READ and WRITE access.\n
             * - Default value: 1000 (milliseconds)
             */
            property<NSerial, uint32_t, propReadWrite> TransmissionTimeout;

            /**
             * @brief This property sets the size of 2 internal buffers used for reception.
             * Be sure to adjust this buffer sizes based on the size of largest data\n
             * packet expected to be received.\n
             * Keep in mind that data memory space is a valuable resource in embedded systems.\n
             * The value of this property is limited in the range of 8 to 255 (bytes).\n
             * This property provides READ and WRITE access.\n
             * - Default value: 64 (bytes)
             */
			property<NSerial, uint32_t, propReadWrite> BufferSizeRx;

            /**
             * @brief This property sets the size of the internal buffer used for transmission.
             * Be sure to adjust this buffer sizes based on the size of largest data packet\n
             * expected to be transmitted.\n
             * The value of this property is limited in the range of 8 to 255 (bytes).\n
             * This property provides READ and WRITE access.\n
             * - Default value: 64 (bytes)
             */
            property<NSerial, uint32_t, propReadWrite> BufferSizeTx;
    };

#endif
//==============================================================================
