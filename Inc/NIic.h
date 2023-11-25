//==============================================================================
/**
 * @brief NIic\n
 * Master-mode I2C abstraction class\n
 * This class provides read and write methods for I2C communication with one or\n
 * more slave device connected to the same bus, as long as they have different\n
 * device identification numbers (device IDs).\n
 * @author Joao Nilo Rodrigues - nilo@pobox.com
 *
 *------------------------------------------------------------------------------
 *
 * @copyright   Copyright (c) 2020 Joao Nilo Rodrigues\n
 * 					   All rights reserved.\n
 * This software component is licensed by "Joao Nilo Rodrigues" under BSD 3-Clause\n
 * license, the "License";\n
 * You may not use this file except in compliance with the License.\n
 *               You may obtain a copy of the License at:\n
 *                 opensource.org/licenses/BSD-3-Clause\n
 */
//------------------------------------------------------------------------------
#ifndef NI2C_H
    #define NI2C_H

	#include "NTinyOutput.h"
    #include "DRV_DMA.h"
    #include "SysMessages.h"
    #include "NComponent.h"

//------------------------------------------------------------------------------

    #define I2C_ENABLE              (I2Cx->CR1 |= I2C_CR1_PE)
    #define I2C_DISABLE             (I2Cx->CR1 &= ~I2C_CR1_PE)
    #define I2C_IS_ENABLED          ((I2Cx->CR1 & I2C_CR1_PE)==I2C_CR1_PE)
	#define I2C_START				(I2Cx->CR1 |= I2C_CR1_START)
	#define I2C_STOP				(I2Cx->CR1 |= I2C_CR1_STOP)
	#define I2C_RESET_ON			(I2Cx->CR1 |= I2C_CR1_SWRST)
	#define I2C_RESET_OFF			(I2Cx->CR1 &= ~I2C_CR1_SWRST)

	#define	I2C_NOT_STARTED			((I2Cx->SR1 & I2C_SR1_SB)!=I2C_SR1_SB)
	#define	I2C_TX_NOT_EMPTY		((I2Cx->SR1 & I2C_SR1_TXE)!=I2C_SR1_TXE)
	#define	I2C_TX_NOT_COMPLETED	((I2Cx->SR1 & I2C_SR1_BTF)!=I2C_SR1_BTF)
	#define	I2C_NOT_ADDRESSED		((I2Cx->SR1 & I2C_SR1_ADDR)!=I2C_SR1_ADDR)
	#define	I2C_TX_DATA				(I2Cx->DR)

	#define	I2C_RX_IS_EMPTY			((I2Cx->SR1 & I2C_SR1_RXNE)!=I2C_SR1_RXNE)
	#define	I2C_RX_DATA				(I2Cx->DR)

	#define	I2C_IS_BUSY				((I2Cx->SR2 & I2C_SR2_BUSY)==I2C_SR2_BUSY)

    #define I2C_SET_ACK          	(I2Cx->CR1 |= I2C_CR1_ACK)
	#define I2C_CLEAR_ACK           (I2Cx->CR1 &= ~I2C_CR1_ACK)


	#define I2C_INTS_DISABLE       	(I2Cx->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN))
	#define I2C_INTS_ENABLE       	(I2Cx->CR2 |=  (I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN))

	#define I2C_TXDMA_ENABLE        (I2Cx->CR2 |= I2C_CR2_DMAEN)
	#define I2C_RXDMA_ENABLE        (I2Cx->CR2 |= I2C_CR2_DMAEN)
	#define I2C_TXDMA_DISABLE       (I2Cx->CR2 &= ~I2C_CR2_DMAEN)
	#define I2C_RXDMA_DISABLE       (I2Cx->CR2 &= ~I2C_CR2_DMAEN)
	#define I2C_DMA_DISABLE         (I2Cx->CR2 &= ~I2C_CR2_DMAEN)

	#define I2C_PACKET_SIZE_MAX  	((uint32_t) 255)

	#define I2C_NO_TIMEOUT          (timeout > SSR_GetSystemTime() - ticks)

/**
 *  @enum NIic::iiRemaping
 * @{
 * @brief This enumeration defines options of pinout for the I2C lines.\n
 * Check the table below for more detailed description.
 *
 */
    enum iiRemapping { iiStandard,	//!< I2C1: SCL = PB6,  SDA = PB7
    								//!< I2C2: SCL = PB10, SDA = PB11
    				   iiRemapped,	//!< I2C1: SCL = PB8,  SDA = PB9
					   	   	   	   	//!< I2C2: NOT AVAILABLE
    				 };
/**
* @}
*/

/** @enum NIic::iiClockRates
 * @{
 * @brief This enumeration defines values for the SCL line (serial clock speed)
 */
	enum iiClockRates { ii100kHz,	//!< standard clock frequency: 100kHz
						ii400kHz	//!< "fast mode" clock frequency: 400kHz
					  };
/**
* @}
*/
	/**
	 * @class NIic
	 * This component is an abstraction class driver for I2C communications.\n
	 * Only "master" mode is available at the moment.\n
	 * @{
	 */
    //----------------------------------------
    class NIic: public NComponent{
		//!@internal
        private:
			bool open;
			bool disable;
			bool MasterTxInProgress;
			bool MasterRxInProgress;
			bool MasterTxRxInProgress;
			uint32_t bytes_to_write;
			uint32_t bytes_written;
			uint32_t bytes_to_read;
			uint32_t bytes_read;
			uint8_t* bytes_pointer;
			uint8_t device_address[2];
			uint16_t device_id;
			uint8_t* external_buffer;
			uint32_t timeout;
			uint32_t ticks;

			I2C_TypeDef* I2Cx;
			DMA_Channel_TypeDef * DMA_TxChannel;
			DMA_Channel_TypeDef * DMA_RxChannel;
			uint16_t vector_index;
			iiClockRates baud;

        //-------------------------------------------
        private:
            I2C_TypeDef* GetPort();
			iiClockRates GetClockRate();
			void SetClockRate(iiClockRates);
			uint8_t GetDeviceId();
			void SetDeviceId(uint8_t);
			uint16_t GetDeviceId10();
			void SetDeviceId10(uint16_t);
			void Reset();

        //-------------------------------------------
		protected:
            bool AssistedReception;                                 // false
            bool AssistedTransmission;                              // false
    		//!@endinternal

        //-------------------------------------------
        public:
            NIic(I2C_TypeDef*, iiRemapping);
            ~NIic(void);

			//---------------------------------------
			/// METHODS
			/**
			 * @brief Open must be called before any access to the I2C interface\n
			 * The main purpose is to update all the relevant registers with the
			 * appropriate configuration values.             *
			 */
			void Open();

			/**
			 * @brief This method might me used to release some hardware resources\n
			 * allocated to this component. In addition, the peripheral clock is
			 * turned off to reduce power consumption.
			 */
			void Close();

			/**
			 * @brief This method is a callback function used exclusively by the framework.\n
			 * @param Msg is a structure of type NMESSAGE and is received from the framework.\n
			 * Users should never care about this parameter.
			 * @warning Do not call this method from the application!
			 */
			virtual void Notify(NMESSAGE* Msg);

			/**
			 * @brief This method is a callback function used exclusively by the framework.\n
			 * @param Msg is a structure of type NMESSAGE and is received from the framework.\n
			 * Users should never care about this parameter.
			 * @warning Do not call this method from the application!
			 */
			virtual void InterruptCallBack(NMESSAGE* Msg);

			/**
			 * @brief Sets a "start" condition on the I2C communication bus.\n
			 * Please refer to the specific documentation for details on how the I2C communication
			 * frames are generated.
			 */
			void Start();

			/**
			 * @brief Sets a "stop" condition on the I2C communication bus.\n
			 * Please refer to the specific documentation for details on how the I2C communication
			 * frames are generated.
			 */
			void Stop();

			/**
			 * @brief This method is used to send the address of the destinatio device, since in the\n
			 * same I2C bus it may contain many devices, as long as they have different addresses.
			 * @param Addr is a 7-bit address of the targer device.
			 * @return True if successful or false if failed.
			 * @note This version of NIic does not support 10-bit addressed devices.
			 */
			bool Address(uint8_t Addr);

			/**
			 * @brief This method is used to send a  "data" byte to the destinatio device.
			 * @param 8-bit data to be written.
			 * @return True if successful or false if failed.
			 */
			bool Write(uint8_t);

			/**
			 * @brief This method is used to send a "data" block to the destinatio device.
			 * @param address of the first data to be written.
			 * @param number of bytes to write.
			 * @return True if successful or false if failed.
			 */
			bool Write(uint8_t*, uint8_t);


			bool Write(uint8_t, uint8_t*, uint32_t);
			bool Read(uint8_t, uint8_t*, uint32_t);

            //---------------------------------------
            /** @event OnError
             *  This event is fired when a communication error is detected\n
             *  @param ErrCode is the value of the SR1 register when the error ocuured.\n
             *  Please refer to the datasheet for details on each error flag in this register.\n
             */
            void (*OnError)(uint32_t);


            void (*OnReadCompleted)(uint8_t*, uint8_t);
            void (*OnWriteCompleted)(void);

            /***********************************************************************************
             * @defgroup PROPERTIES                						< DEFAULT VALUES >
             * @{
			 *
             *
             * @property Port
             * This property can be used to check the physical I2C interface associated\n
             * to this component, i. e., the same assigned to the constructor.
             * @note This is a "read only" property.
             */
            property<NIic, I2C_TypeDef*, propRead> Port;            // (?)

            /**
             * @property ClockRate
             * This property is used to select clock rate, according to slave device constraints.\n
             * @note The clock rate should not change during read or write operations.
             */
			property<NIic, iiClockRates, propReadWrite> ClockRate;	// ii100kHz

            /**
             * @property Blocking
             * When Blocking is true (default), methods Read and Write will lock the program\n
             * execution until the data transfer is completed. This option is not recommended\n
             * for large data blocks.
             * If the Blocking property is changed to false, the program execution will leave the\n
             * Read or Write method immediately and, as soon as the current data transfer is completed, \n
             * an event will be generated, depending on the last operation. (OnReadCompleted or OnWriteCompleted)\n
             */
            bool Blocking;                                          // true

            /**
             * @property EnableDMA
             * As the name says it specifies if the DMA engine is to be used by the I2C interface.\n
             * @note This property affects both Read and Write operations.
             */
			bool EnableDMA;											// false

			/********************************************************************************************
			 * *}
			 */
    };
    //-----------------------------------------------
    /**
     * @}
     */
#endif
//==============================================================================
