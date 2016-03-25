#include "Headers\adc_driver.h"

#define GPIO30 _SFR_MEM8(0x1E)  // = r30
#define GPIO31 _SFR_MEM8(0x1F)	// = r31


/*  This function get the calibration data from the production calibration.
 *
 *  The calibration data is loaded from flash and stored in the calibration
 *  register. The calibration data reduces the gain error in the adc.
 *
 *  \param  adc          Pointer to ADC module register section.
 */
 void ADC_CalibrationValues_Set(ADC_t * adc)
{
	if(&ADCA == adc){
		 // Get ADCCAL0 from byte address 0x20 (Word address 0x10.  
		adc->CAL = SP_ReadCalibrationByte(0x20);
	}else {
		// Get ADCCAL0 from byte address 0x24 (Word address 0x12.  
		adc->CAL = SP_ReadCalibrationByte(0x24);
	}
} 


/*  This function clears the interrupt flag and returns the coversion result.
 *
 *	This function should be used together with the ADC_Ch_Conversion_Complete.
 *      When the conversion result is ready this funciton reads out the result.
 *
 *  \param  adc_ch  Pointer to ADC channel register section.
 *  \param  offset  Offset value that is compansated for.
 *  \return  Conversion result.
 */
uint16_t ADC_ResultCh_GetWord(ADC_CH_t * adc_ch, uint8_t offset)
{
  	uint16_t answer;
	uint16_t signedOffset = (uint16_t) offset;

	// Append 16-bit signed value if offset (signed) is negative. 
	if (offset >= 128){
		signedOffset |= 0xFF00;
	}

	// Clear interrupt flag. 
	adc_ch->INTFLAGS = ADC_CH_CHIF_bm;

	// Return result register contents  
	answer = adc_ch->RES - signedOffset;

	return answer;
}


/*  This function clears the interrupt flag and returns the low byte of the coversion result.
 *
 *	This funtion should be used together with the ADC_Ch_Conversion_Complete.
 *      When the conversion result is ready this funciton reads out the result.
 *
 *  \note  If this function is used with 12-bit right adjusted results, it
 *         returns the 8 LSB only.
 *
 *  \param  adc_ch  Pointer to ADC channel register section.
 *  \param  offset  Compansation offset value.
 *
 *  \return  Low byte of conversion result.
 */
uint8_t ADC_ResultCh_GetLowByte(ADC_CH_t * adc_ch, uint8_t offset)
{
  	uint8_t answer;

	// Clear interrupt flag.
	adc_ch->INTFLAGS = ADC_CH_CHIF_bm;
	// Return result register contents 
	answer = adc_ch->RESL - offset;

	return answer;
}

/*  This function clears the interrupt flag and returns the high byte of the coversion result.
 *
 *	This funtion should be used together with the ADC_ResultCh_ConversionComplete.
 *      When the conversion result is ready this funciton reads out the result.
 *
 *  \note  If this function is used with 12-bit right adjusted results, it
 *         returns the 8 LSB only. Offset is not compensated.
 *
 *  \note  If ADC interrupts are enabled, the interrupt handler will clear the
 *         interrupt flag before the polling loop can detect it, and this function
 *         will hang forever. Therefore, do not use interrupts with this function.
 *         Instead, read the result registers directly.
 *
 *  \param  adc_ch  Pointer to ADC channel register section.
 *
 *  \return  High byte of conversion result.
 */
uint8_t ADC_ResultCh_GetHighByte(ADC_CH_t * adc_ch)
{
	// Clear interrupt flag. 
	adc_ch->INTFLAGS = ADC_CH_CHIF_bm;

	// Return low ?? byte result register contents. 
	return adc_ch->RESH;
}

/*  This function waits until the adc common mode is settled.
 *
 *  After the ADC clock has been turned on, the common mode voltage in the ADC
 *  needs some time to settle. The time it takes equals one dummy conversion.
 *  Instead of doing a dummy conversion this function waits until the common
 *  mode is settled.
 *
 *  \note The function sets the prescaler to the minimum value to minimize the
 *        time it takes the common mode to settle. If the clock speed is higher
 *        than 8 MHz use the ADC_wait_32MHz function.
 *
 *  \param  adc Pointer to ADC module register section.
 */
void ADC_Wait_8MHz(ADC_t * adc)
{
  	// Store old prescaler value.  
  	uint8_t prescaler_val = adc->PRESCALER;

	// Set prescaler value to minimum value.  
	adc->PRESCALER = ADC_PRESCALER_DIV4_gc;

	// Wait 4*COMMEN_MODE_CYCLES for common mode to settle.  
	delay_us(4*COMMEN_MODE_CYCLES);

	// Set prescaler to old value 
	adc->PRESCALER = prescaler_val;
}


/*  This function waits until the adc common mode is settled.
 *
 *  After the ADC clock has been turned on, the common mode voltage in the ADC
 *  need some time to settle. The time it takes equals one dummy conversion.
 *  Instead of doing a dummy conversion this function waits until the common
 *  mode is settled.
 *
 *  \note The function sets the prescaler to the minimum value possible when the
 *        clock speed is larger than 8 MHz to minimize the time it takes the
 *        common mode to settle.
 *
 *  \note The ADC clock is turned off every time the ADC i disabled or the
 *        device goes into sleep (not Idle sleep mode).
 *
 *  \param  adc Pointer to ADC module register section.
 */
void ADC_Wait_32MHz(ADC_t * adc)
{
  	// Store old prescaler value.  
  	uint8_t prescaler_val = adc->PRESCALER;

	// Set prescaler value to minimum value.  
	adc->PRESCALER = ADC_PRESCALER_DIV8_gc;

	// wait 8*COMMEN_MODE_CYCLES for common mode to settle 
	delay_us(8*COMMEN_MODE_CYCLES);

	// Set prescaler to old value 
	adc->PRESCALER = prescaler_val;
}

/*  This function get the offset of the ADC
 *
 *   This function makes an internal coupling to the same pin and calculate
 *   the internal offset in the ADC.
 *
 *  \note This function only return the low byte of the 12-bit convertion,
 *        because the offset should never be more than +-8 LSB off.
 *
 *  \param adc Pointer to the ADC to calculate offset from.
 *
 *  \return Offset on the selected ADC
 */
uint8_t ADC_Offset_Get(ADC_t * adc)
{
	uint8_t offset;

  	// Set up ADC to get offset.  
  	ADC_ConvMode_and_Resolution_Config(adc, true, ADC_RESOLUTION_12BIT_gc);

	ADC_Prescaler_Config(adc , ADC_PRESCALER_DIV8_gc);

	ADC_Referance_Config(adc , ADC_REFSEL_INT1V_gc);

	ADC_Ch_InputMode_and_Gain_Config(&(adc->CH0),
	                                 ADC_CH_INPUTMODE_DIFF_gc,
	                                 ADC_CH_GAIN_1X_gc);

	ADC_Ch_InputMux_Config(&(adc->CH0), ADC_CH_MUXPOS_PIN0_gc, ADC_CH_MUXNEG_PIN0_gc);

	// Enable ADC.  
	ADC_Enable(adc);

	// Wait until ADC is ready.  
	ADC_Wait_32MHz(adc);

	// Do one conversion to find offset.  
	ADC_Ch_Conversion_Start(&(adc->CH0));

	do{
	}while(!ADC_Ch_Conversion_Complete(&(adc->CH0)));
	offset = ADC_ResultCh_GetLowByte(&(adc->CH0), 0x00);

	// Disable ADC.  
	ADC_Disable(adc);

	return offset;
}

#ifdef __GNUC__

/*  Function for GCC to read out calibration byte.
 *
 *  \note This inline assembly only works for GCC. For IAR support, include the
 *        adc_driver_asm.S90 file in your project.
 *
 *  \param index The index to the calibration byte.
 *
 *  \return Calibration byte.
*/
/* */ 
uint8_t SP_ReadCalibrationByte( uint8_t index )
{
	uint8_t result;
	/* __asm__ __volatile__ (
		"ldi r20, %3    ; Load command into temp register." "\n\t"
		"mov r30, %2	; Move index to Z pointer."         "\n\t"
		"clr r31        ; Clear ZH."                        "\n\t"
		"sts %1, r20    ; Store command to CMD register."   "\n\t"
		"lpm %0, r30      ; Load Program memory to result"    "\n\t"
		"ldi r20, %4    ; Clean up CMD register."           "\n\t"
		"sts %1, r20"
		: "=r" (result)
		: "m" (NVM_CMD),
		  "r" (index),
		  "M" (NVM_CMD_READ_CALIB_ROW_gc),
		  "M" (NVM_CMD_NO_OPERATION_gc)
		: "r20", "r30", "r31"
		); */

		GPIO30 = index;  // r30 - LSB of Z pointer to EEPROM
		GPIO31 = 0;
		NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
		result = GPIO30;
		NVM_CMD = NVM_CMD_NO_OPERATION_gc;


		return result;

}
/* */
#endif
