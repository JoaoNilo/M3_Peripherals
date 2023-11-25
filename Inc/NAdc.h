//==============================================================================
/**
 *  @file NAdc.h
 *  @brief Analog to Digital Converters (ADCs) abstraction class
 *  @version 1.0.0
 *  @author J. Nilo Rodrigues
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
 *------------------------------------------------------------------------------
 *<A id="INSTRUCT"> Instructions: </A>
 *
 * 1 - Call the component's constructor specifying the appropriate converter (ADC1 or ADC2).\n
 * 2 - Select one of the operating modes, through the @ref Mode property.\n
 * Possible options are: @ref adSingle, @ref adContinuous1, @ref adContinuous2 and @ref adContinuous3 .\n
 * Description of operating modes:
 *
 *       2.1 - @ref adSingle: sampling cycles are started by calling the Start() method.
 *       	  At the "end of the conversion", the OnData event is fired the sample can be processed
 *       	  by the event handler.
 *            Pros: can be associated with non-periodic events (e.g. a pressed button).
 *            Cons: worst sampling efficiency.
 *
 *      2.2 - adContinuous1: continuous sampling is started by Start(M), where 'M' is the time interval between samples
 *            (in milliseconds). This mode has a built-in software timer, resulting in low timing accuracy.
 *            At the end of each conversion, the OnData event is fired and the "current" sample is available.
 *            The sampling process can be terminated at any time by Stop();
 *            Pros: does not allocate a "physical" (hardware) timer.
 *            Cons: jitter above 50us is expected in the sampling interval.
 *
 *      2.3 - adContinuous2: continuous sampling is started by Start(U), ​​where 'U' is the required time interval between
 *      	  samples (in microseconds).
 *            At the end of "each" conversion, the OnData event is fired "by interrupt".
 *            The sampling process can be terminated at any time by Stop();
 *
 *      	  Pros: greater accuracy in sampling time.
 *            Cons: this mode allocates "HARDWARE" resources:
 *             - Timer TIM4 for ADC1 and timer TIM2 for ADC2.
 *            ATTENTION: If the sample rate is too high, the system performance will be seriously compromised.
 *
 *      2.4 - adContinuous3: continuous "block" sampling is started by Start(U), ​​where 'U' is the required sampling time
 *            interval (in microseconds).
 * 			  At the end of each conversion, the samples are available in the OnDataBlock(ptSamples, szSamples) event.
 * 			  Then, a number of szSamples, can be read from memory location ptSamples.
 * 			  The sampling process can be terminated at any time by Stop();
 * 			  Pros:	High sample rate conversion capability without compromising the efficiency of the rest of the system.
 * 			  (provided the sample buffer is correctly sized (see "SetDataBuffer(P, S)")).
 *            Cons:
 *            	- this mode "CAN ONLY" be used by the ADC1.
 *           	- this mode needs TIM4 to generate the sampling time base.
 *           	- this mode also allocates DMA1 CHANNEL_1 to transfer samples from ADC1 to memory.
 *
 *
 * 3 - Assign the address of the event handler to OnData when using @ref adSingle, r@e adContinuous1 or @ref adContinuous2, or\n
 *     assign the address of the event handler to OnDataBlock when using @ref adContinuous3.\n\n
 *
 * 4 - Select the channel to be sampled using the "Channel" property, if only one channel is used. The "Channel" property\n
 * can be "changed" inside the "OnData" event handler if you want to change the sampled channel.\n
 * When more than one channel is used ("scan" mode), add the channels to be sampled using "AddChannel()". If no channels are\n
 * added to the sampling table, no channels will be sampled and no events will be "fired".\n
 * NOTE: The "adThermo" and "adVref" channels can only be read by the ADC1.
 *
 * 5 - When the @ref Mode is set to @ref adContinuous3, allocate a buffer for samples by calling the "SetDataBuffer" method.\n
 * Keep in mind that the call rate for the OnDataBlock event will be N/2 * Tsampling, where N is the allocated buffer size\n
 * and Tsampling is the time interval between consecutive samples, in microseconds, as specified in the "Start" method.\n\n
 * This trade-off between buffer size and interrupt rate is also discussed <A href="class_n_adc.html#a8725406d940ac0a900aa8ef7cee0380a">HERE</A>.\n
 *
 * 6 - Start the sampling process by calling "Start".\n\n
 *
 */
///-----------------------------------------------------------------------------
#ifndef NADC_H
    #define NADC_H


	#include "NComponent.h"

    /**
     * @enum adModes
     * @brief This enumeration defines the options for the @ref Mode property.
     */
    enum adModes        { adSingle,			//!< single, asynchronous sampling
    	                  adContinuous1,	//!< continuous, software-based timer sampling
						  adContinuous2,	//!< continuous, precise hardware-based timer sampling
						  adContinuous3		//!< continuous, precise hardware-based timer block sampling
    					};

    /**
     * @enum adAlignments
     * @brief This enumeration defines the options for the @ref Alignment property.
     */
    enum adAlignments   { adRight,			//!< right-aligned, 12 lest-significant bits.
    	                  adLeft			//!< left-aligned,  12 most-significant bits.
    					};

    /**
     * @enum adThresholds
     * @brief This enumeration defines the options for the @ref Thresholds property.
     */
    enum adThresholds   { adNone,			//!< Analog watchdog not in use.
    	                  adLower,			//!< Analog watchdog guarding the lower limit.
						  adUpper,			//!< Analog watchdog guarding the upper limit.
						  adBoth			//!< Analog watchdog on upper and lower limits.
    					};

    /**
     * @enum adChannels
     * @brief This enumeration defines the options for the @ref Channel property.
     */
    enum adChannels     { adCH0 =  0x0000,			//!< Analog input 0.
    					  adCH1 =  0x0001,			//!< Analog input 1.
						  adCH2 =  0x0002,			//!< Analog input 2.
						  adCH3 =  0x0003,			//!< Analog input 3.
						  adCH4 =  0x0004,			//!< Analog input 4.
						  adCH5 =  0x0005,			//!< Analog input 5.
						  adCH6 =  0x0006,			//!< Analog input 6.
						  adCH7 =  0x0007,			//!< Analog input 7.
						  adCH8 =  0x0100,			//!< Analog input 8.
						  adCH9 =  0x0101,			//!< Analog input 9.
						  adCH10 = 0x0200,			//!< Analog input 10.
						  adCH11 = 0x0201,			//!< Analog input 11.
						  adCH12 = 0x0202,			//!< Analog input 12.
						  adCH13 = 0x0203,			//!< Analog input 13.
						  adCH14 = 0x0204,			//!< Analog input 14.
                          adCH15 = 0x0205,			//!< Analog input 15.
						  adThrm = 0x1010,			//!< Core Thermometer.
						  adVref = 0x1011,			//!< Internal Voltage Reference.
						  adNoCH = 0x1012			//!< No channel.
    					};

    //-----------------------------------------------------
    enum adStates { adStopped,
    				adSampling1,
    				adSampling2,
    				adSampling3Locked,
    				adSampling3
    };

    //-----------------------------------------------------
    struct SAMPLE_BUFFERS{
        uint16_t* data;
        uint16_t size;
    };

	#define ADC_SampleTime_1p5Cycles                   	((uint8_t)0x00)
	#define ADC_SampleTime_7p5Cycles                   	((uint8_t)0x01)
	#define ADC_SampleTime_13p5Cycles                  	((uint8_t)0x02)
	#define ADC_SampleTime_28p5Cycles                  	((uint8_t)0x03)
	#define ADC_SampleTime_41p5Cycles                  	((uint8_t)0x04)
	#define ADC_SampleTime_55p5Cycles                  	((uint8_t)0x05)
	#define ADC_SampleTime_71p5Cycles                  	((uint8_t)0x06)
	#define ADC_SampleTime_239p5Cycles                 	((uint8_t)0x07)

	#define ADC_EXTERNAL_TRIGGER_OFF					(ADCx->CR2 &= ~ADC_CR2_EXTTRIG)
	#define ADC_EXTERNAL_TRIGGER_ON						(ADCx->CR2 |= ADC_CR2_EXTTRIG)
	#define ADC_THRESHOLD_MAX							((uint32_t) 0x00000FFF)
	#define ADC_THRESHOLD_MIN							((uint32_t) 0x00000000)

    //-----------------------------------------------------
    /** @brief Analog to Digital Converters (ADCs) abstraction class
     * - This component provides resources to setup and use the Analog to Digital converters\n
     * in the MCU hardware for analog signals sampling and processing.
     */
    class NAdc : public NComponent{
        private:

            //--------------------------------------------
            adModes mode;
            adAlignments alignment;
            adThresholds threshold;
            adChannels channel;
            uint16_t samplingrate;
            uint16_t activechannels;
            uint16_t lowerlimit;
            uint16_t upperlimit;

            //--------------------------------------------
            uint16_t adc;
            uint16_t vector_index;
            SAMPLE_BUFFERS buffer[2];
            uint16_t pino;
            GPIO_TypeDef* porta;
            TIM_TypeDef*                TIMx;
            ADC_TypeDef*                ADCx;
            DMA_Channel_TypeDef*        DMA_AdcChannel;
            uint32_t                	DMA_AdcChannelFlags;
            IRQn_Type 					ADCx_IRQn;

            //--------------------------------------------
            adStates status;
            uint16_t convertion_counter;

        private:

            //-------------------------------------------
            void SetMode(adModes);
            adModes GetMode();
            void SetAlignment(adAlignments);
            adAlignments GetAlignment();
            void SetThresholds(adThresholds);
            adThresholds GetThresholds();
            void SetChannel(adChannels);
            adChannels GetChannel();

            void SetLowerLimit(uint16_t);
            uint16_t GetLowerLimit();
            void SetUpperLimit(uint16_t);
            uint16_t GetUpperLimit();
            uint16_t GetSamplingRate();

            //-------------------------------------------
            void SetupInterrupt();
            void SetTimebase();
            void SetSampleTime(adChannels, uint8_t);
            void IncludeRegularChannel(uint8_t, adChannels);

        //-----------------------------------------------
        public:

            //-------------------------------------------
            // METHODS
            /**
             * @brief Constructor for this component.
             * @arg ADCn: ADC to be used (ADC1 or ADC2 when available)
             * @note Be sure to specify a valid ADC, available on the target MCU and
             * not in used by the application.
             */
            NAdc(ADC_TypeDef* ADCn);

            /**
             * @brief Standard destructor for this component.
             */
            ~NAdc(void);

            /**
             * @brief This method is used as a system callback function for message dispatching.
             * @note This callback method should not be called by the application.
             */
            void Notify(NMESSAGE*);

            /**
             * @brief This method is used as a system callback function for time critical
             * events dispatching.
             */
            void InterruptCallBack(NMESSAGE*);

            /**
             * @brief Adds channel to the conversion table.
             * The convertion table can handle up to 16 channels, repeated or not.
             * @arg CHn: channel number  (@ref adCH0, @ref adCH1, etc.)
             * @return true if channel is successfully added to the table.
             */
            bool AddChannel(adChannels CHn);

            /**
             * @brief Clears the conversion table.
             * The convertion table can handle up to 16 channels, repeated or not.
             * @arg CHn: channel number  (@ref adCH0, @ref adCH1, etc.)
             * @return true if channel is successfully added to the table.
             */
            void FlushChannels();

            /**
             * @brief Sets data structure for samples storage.
             * @arg ptExtSamplesBuffer: pointer to the buffer to receive the samples.
             * @arg szExtSamplesBuffer: size of the buffer (always an even number).\n
             * <A id="SETDATABUFFER">Considerations for correct buffer allocation</A>:\n
             *
             * Lets suppose the following requirements as an example:
             *   - Sampling rate: 10us (100kHz or 100ksps);
             *   - Samples needed on every OnDataBlock event: 50 samples.\n
             *     Lets call this number the Nspe or Number of "samples per event".\n
             *
             *<p>
             * For a single variable (channel):\n
             *    szExtSamplesBuffer =  (Nchannels * 2) * Nspe = (1 * 2) * 50 = 100\n
             *    So, the buffer should have 100 positions.\n
             * 	  For 10us sampling rate the event OnDataBlock will occur every 500us or\n
             * 	  at a rate of 2kHz. For lower rates the buffer size should be increased.
             *</p>
             *
             * For multiple variables (channels):\n
             *    1 - Keep in mind that samples will be interleaved in the buffer;\n
             *    2 - Same amount of samples, for each channel, are expected in the buffer;\n
             *        - szExtSamplesBuffer =  Nchannels * 2 * 50\n
             *
             *<p> In case of 3 channels: szExtSamplesBuffer(3) =  (3 * 2) * 50 = 300 </p>
             *
             *<p>
             * 	  In this case, for 10us sampling rate the event OnDataBlock will occur\n
             * 	  every 1.5ms or at a rate of ~666Hz.\n
             * 	  Samples in the buffer have the following order:\n
             * 	  | CH1[0] | CH2[0] | CH3[0] | CH1[1] | CH2[1] | CH3[1] | CH1[2] | ...\n
             *</p>
             * @warning As the ADC resolution is 12 bits, samples are stored in 16-bit buffers.
             */
            void SetDataBuffer(uint16_t* ptExtSamplesBuffer, uint16_t szExtSamplesBuffer);

            /**
             * @brief Starts a single conversion.
             * This method can only be used when @ref Mode property is set to @ref asSingle.
             */
            void Start();

            /**
             * @brief Starts continuous conversions.
             * This method can only be used when @ref Mode property is set to any "Continuous"\n
             * option of the asModes enumeration.
             * @arg Tsampling: sampling time interval.
             * This argument is in milliseconds for modes @ref adContinuous1 and @ref adContiouus2;
             * and in microseconds for mode adContiuous3.
             */
            void Start(uint16_t Tsampling);

            /**
             * @brief Stops continuous conversions.
             * This method can stop conversions in any continuous mode.
             */
            void Stop();

            //------------------------------------------------------------------
            // EVENTS
            /**
             * @brief This is the handler placeholder for OnData events.
             * - This event handler is called when a new sample is available.\n
             * @arg sample: last value sampled by the ADC converter,
             *
             * @note This event is not fired when Mode property is set to @ref adContinuous3.
             */
            void (*OnData)(uint16_t sample);

            /**
             * @brief This is the handler placeholder for OnDataBlock events.
             * - This event handler is called when a new sample is available.\n
             * @arg ptSamples: pointer to the buffer with the last converted samples,
             * @arg szSamples: number of samples in the buffer.
             *
             * @note This event is fired ONLY when Mode property is set to @ref adContinuous3.
             */
            void (*OnDataBlock)(uint16_t* ptSamples, uint16_t szSamples);

            /**
             * @brief This is the handler placeholder for OnUpperThreshold events.
             * - This event handler is called when the value of a new sample is bigger than \n
             * the @ref UpperLimit property.\n
             * @arg sample: value of the last conversion, which may have triggered the event.
             *
             * This event can be raised on any @ref adModes, unless the @ref Thresholds property\n
             * is set to @ref adLower.
             */
            void (*OnUpperThreshold)(uint16_t sample);

            /**
             * @brief This is the handler placeholder for OnLowerThreshold events.
             * - This event handler is called when the value of a new sample is smaller than \n
             * the @ref LowerLimit property.\n
             * @arg sample: value of the last conversion, which may have triggered the event.
             *
             * This event can be generated on any @ref adModes, unless the @ref Thresholds property\n
             * is set to @ref adUpper.
             */
            void (*OnLowerThreshold)(uint16_t sample);

            //------------------------------------------------------------------
            // PROPERTIES
            /**
             * @brief This property sets the operating @ref Mode.
             * The four basic operating modes are available in the adModes enumeration.\n
             * A detailed description of each one of these modes can be found in the <A href="_n_adc_8h.html#INSTRUCT"> Instructions </A> section.
             * This property provides READ and WRITE access.
             * - Default value: adSingle
             */
            property<NAdc, enum adModes, propReadWrite> Mode;

            /**
             * @brief This property can be used to select a different alignment for the converter samples.\n
             * The available options are listed in the @ref adAlingments enumeration.\n
             * For detailed description about sample alignment, please check the MCU datasheet.
             * This property provides READ and WRITE access.
             * - Default value: adRight
             */
            property<NAdc, enum adAlignments, propReadWrite> Alignment;

            /**
             * @brief This property can be used to control the "analog watchdog".\n
             * The @ref adThresholds enumeration provides options for this property.\n
             * This property provides READ and WRITE access.
             * - Default value: adNone (disabled)
             */
            property<NAdc, enum adThresholds, propReadWrite> Thresholds;

            /**
             * @brief This property sets the lower threshold for the "analog watchdog".\n
             * When @ref Thresholds property is set to adLower or adBoth and the value of the\n
             * last sample is smaller than LowerLimit, the OnLowerTheshold event is triggered.\n
             * This property provides READ and WRITE access.
             * - Default value: 0 (0x0000)
             */
            property<NAdc, uint16_t, propReadWrite> LowerLimit;

            /**
             * @brief This property sets the upper threshold for the "analog watchdog".\n
             * When @ref Thresholds property is set to adUpper or adBoth and the value of the\n
             * last sample is bigger than UpperLimit, the OnUpperTheshold event is triggered.\n
             * This property provides READ and WRITE access.
             * - Default value: 4095 (0x0FFF)
             */
            property<NAdc, uint16_t, propReadWrite> UpperLimit;

            /**
             * @brief This property can be used to check the number of channels registered in the\n
             * conversion table. It may not be the number of channels in use as a channel may be\n
             * registered more than once.
             * This property provides READ ONLY access.
             * - Default value: 0
             */
            property<NAdc, uint16_t, propRead> ActiveChannels;

            /**
             * @brief If only one analog input (A/D channel) is supposed to be used,\n
             * this property sets the required channel for conversions.\n
             * Note, however, that this property clears the convertion table before\n
             * registering the assigned channel. For multiple channels conversions one\n
             * should call @ref AddChannel method multiple times.
             * This property provides READ and WRITE access.
             * - Default value: adNoCH
             */
            property<NAdc, adChannels, propReadWrite> Channel;

    };

#endif
//==============================================================================
