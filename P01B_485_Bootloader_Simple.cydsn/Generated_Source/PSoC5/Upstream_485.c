/*******************************************************************************
* File Name: Upstream_485.c
* Version 2.50
*
* Description:
*  This file provides all API functionality of the UART component
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Upstream_485.h"
#if (Upstream_485_INTERNAL_CLOCK_USED)
    #include "Upstream_485_IntClock.h"
#endif /* End Upstream_485_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 Upstream_485_initVar = 0u;

#if (Upstream_485_TX_INTERRUPT_ENABLED && Upstream_485_TX_ENABLED)
    volatile uint8 Upstream_485_txBuffer[Upstream_485_TX_BUFFER_SIZE];
    volatile uint8 Upstream_485_txBufferRead = 0u;
    uint8 Upstream_485_txBufferWrite = 0u;
#endif /* (Upstream_485_TX_INTERRUPT_ENABLED && Upstream_485_TX_ENABLED) */

#if (Upstream_485_RX_INTERRUPT_ENABLED && (Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED))
    uint8 Upstream_485_errorStatus = 0u;
    volatile uint8 Upstream_485_rxBuffer[Upstream_485_RX_BUFFER_SIZE];
    volatile uint8 Upstream_485_rxBufferRead  = 0u;
    volatile uint8 Upstream_485_rxBufferWrite = 0u;
    volatile uint8 Upstream_485_rxBufferLoopDetect = 0u;
    volatile uint8 Upstream_485_rxBufferOverflow   = 0u;
    #if (Upstream_485_RXHW_ADDRESS_ENABLED)
        volatile uint8 Upstream_485_rxAddressMode = Upstream_485_RX_ADDRESS_MODE;
        volatile uint8 Upstream_485_rxAddressDetected = 0u;
    #endif /* (Upstream_485_RXHW_ADDRESS_ENABLED) */
#endif /* (Upstream_485_RX_INTERRUPT_ENABLED && (Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED)) */


/*******************************************************************************
* Function Name: Upstream_485_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  Upstream_485_Start() sets the initVar variable, calls the
*  Upstream_485_Init() function, and then calls the
*  Upstream_485_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The Upstream_485_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time Upstream_485_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the Upstream_485_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Upstream_485_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(Upstream_485_initVar == 0u)
    {
        Upstream_485_Init();
        Upstream_485_initVar = 1u;
    }

    Upstream_485_Enable();
}


/*******************************************************************************
* Function Name: Upstream_485_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call Upstream_485_Init() because
*  the Upstream_485_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Upstream_485_Init(void) 
{
    #if(Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED)

        #if (Upstream_485_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(Upstream_485_RX_VECT_NUM, &Upstream_485_RXISR);
            CyIntSetPriority(Upstream_485_RX_VECT_NUM, Upstream_485_RX_PRIOR_NUM);
            Upstream_485_errorStatus = 0u;
        #endif /* (Upstream_485_RX_INTERRUPT_ENABLED) */

        #if (Upstream_485_RXHW_ADDRESS_ENABLED)
            Upstream_485_SetRxAddressMode(Upstream_485_RX_ADDRESS_MODE);
            Upstream_485_SetRxAddress1(Upstream_485_RX_HW_ADDRESS1);
            Upstream_485_SetRxAddress2(Upstream_485_RX_HW_ADDRESS2);
        #endif /* End Upstream_485_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        Upstream_485_RXBITCTR_PERIOD_REG = Upstream_485_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        Upstream_485_RXSTATUS_MASK_REG  = Upstream_485_INIT_RX_INTERRUPTS_MASK;
    #endif /* End Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED*/

    #if(Upstream_485_TX_ENABLED)
        #if (Upstream_485_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(Upstream_485_TX_VECT_NUM, &Upstream_485_TXISR);
            CyIntSetPriority(Upstream_485_TX_VECT_NUM, Upstream_485_TX_PRIOR_NUM);
        #endif /* (Upstream_485_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (Upstream_485_TXCLKGEN_DP)
            Upstream_485_TXBITCLKGEN_CTR_REG = Upstream_485_BIT_CENTER;
            Upstream_485_TXBITCLKTX_COMPLETE_REG = ((Upstream_485_NUMBER_OF_DATA_BITS +
                        Upstream_485_NUMBER_OF_START_BIT) * Upstream_485_OVER_SAMPLE_COUNT) - 1u;
        #else
            Upstream_485_TXBITCTR_PERIOD_REG = ((Upstream_485_NUMBER_OF_DATA_BITS +
                        Upstream_485_NUMBER_OF_START_BIT) * Upstream_485_OVER_SAMPLE_8) - 1u;
        #endif /* End Upstream_485_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (Upstream_485_TX_INTERRUPT_ENABLED)
            Upstream_485_TXSTATUS_MASK_REG = Upstream_485_TX_STS_FIFO_EMPTY;
        #else
            Upstream_485_TXSTATUS_MASK_REG = Upstream_485_INIT_TX_INTERRUPTS_MASK;
        #endif /*End Upstream_485_TX_INTERRUPT_ENABLED*/

    #endif /* End Upstream_485_TX_ENABLED */

    #if(Upstream_485_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        Upstream_485_WriteControlRegister( \
            (Upstream_485_ReadControlRegister() & (uint8)~Upstream_485_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(Upstream_485_PARITY_TYPE << Upstream_485_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End Upstream_485_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: Upstream_485_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call Upstream_485_Enable() because the Upstream_485_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Upstream_485_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void Upstream_485_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        Upstream_485_RXBITCTR_CONTROL_REG |= Upstream_485_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        Upstream_485_RXSTATUS_ACTL_REG  |= Upstream_485_INT_ENABLE;

        #if (Upstream_485_RX_INTERRUPT_ENABLED)
            Upstream_485_EnableRxInt();

            #if (Upstream_485_RXHW_ADDRESS_ENABLED)
                Upstream_485_rxAddressDetected = 0u;
            #endif /* (Upstream_485_RXHW_ADDRESS_ENABLED) */
        #endif /* (Upstream_485_RX_INTERRUPT_ENABLED) */
    #endif /* (Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED) */

    #if(Upstream_485_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!Upstream_485_TXCLKGEN_DP)
            Upstream_485_TXBITCTR_CONTROL_REG |= Upstream_485_CNTR_ENABLE;
        #endif /* End Upstream_485_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        Upstream_485_TXSTATUS_ACTL_REG |= Upstream_485_INT_ENABLE;
        #if (Upstream_485_TX_INTERRUPT_ENABLED)
            Upstream_485_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            Upstream_485_EnableTxInt();
        #endif /* (Upstream_485_TX_INTERRUPT_ENABLED) */
     #endif /* (Upstream_485_TX_INTERRUPT_ENABLED) */

    #if (Upstream_485_INTERNAL_CLOCK_USED)
        Upstream_485_IntClock_Start();  /* Enable the clock */
    #endif /* (Upstream_485_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Upstream_485_Stop
********************************************************************************
*
* Summary:
*  Disables the UART operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Upstream_485_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED)
        Upstream_485_RXBITCTR_CONTROL_REG &= (uint8) ~Upstream_485_CNTR_ENABLE;
    #endif /* (Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED) */

    #if (Upstream_485_TX_ENABLED)
        #if(!Upstream_485_TXCLKGEN_DP)
            Upstream_485_TXBITCTR_CONTROL_REG &= (uint8) ~Upstream_485_CNTR_ENABLE;
        #endif /* (!Upstream_485_TXCLKGEN_DP) */
    #endif /* (Upstream_485_TX_ENABLED) */

    #if (Upstream_485_INTERNAL_CLOCK_USED)
        Upstream_485_IntClock_Stop();   /* Disable the clock */
    #endif /* (Upstream_485_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED)
        Upstream_485_RXSTATUS_ACTL_REG  &= (uint8) ~Upstream_485_INT_ENABLE;

        #if (Upstream_485_RX_INTERRUPT_ENABLED)
            Upstream_485_DisableRxInt();
        #endif /* (Upstream_485_RX_INTERRUPT_ENABLED) */
    #endif /* (Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED) */

    #if (Upstream_485_TX_ENABLED)
        Upstream_485_TXSTATUS_ACTL_REG &= (uint8) ~Upstream_485_INT_ENABLE;

        #if (Upstream_485_TX_INTERRUPT_ENABLED)
            Upstream_485_DisableTxInt();
        #endif /* (Upstream_485_TX_INTERRUPT_ENABLED) */
    #endif /* (Upstream_485_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Upstream_485_ReadControlRegister
********************************************************************************
*
* Summary:
*  Returns the current value of the control register.
*
* Parameters:
*  None.
*
* Return:
*  Contents of the control register.
*
*******************************************************************************/
uint8 Upstream_485_ReadControlRegister(void) 
{
    #if (Upstream_485_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(Upstream_485_CONTROL_REG);
    #endif /* (Upstream_485_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: Upstream_485_WriteControlRegister
********************************************************************************
*
* Summary:
*  Writes an 8-bit value into the control register
*
* Parameters:
*  control:  control register value
*
* Return:
*  None.
*
*******************************************************************************/
void  Upstream_485_WriteControlRegister(uint8 control) 
{
    #if (Upstream_485_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       Upstream_485_CONTROL_REG = control;
    #endif /* (Upstream_485_CONTROL_REG_REMOVED) */
}


#if(Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED)
    /*******************************************************************************
    * Function Name: Upstream_485_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      Upstream_485_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      Upstream_485_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      Upstream_485_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      Upstream_485_RX_STS_BREAK            Interrupt on break.
    *      Upstream_485_RX_STS_OVERRUN          Interrupt on overrun error.
    *      Upstream_485_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      Upstream_485_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void Upstream_485_SetRxInterruptMode(uint8 intSrc) 
    {
        Upstream_485_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: Upstream_485_ReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Returns the next byte of received data. This function returns data without
    *  checking the status. You must check the status separately.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Received data from RX register
    *
    * Global Variables:
    *  Upstream_485_rxBuffer - RAM buffer pointer for save received data.
    *  Upstream_485_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Upstream_485_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Upstream_485_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Upstream_485_ReadRxData(void) 
    {
        uint8 rxData;

    #if (Upstream_485_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        Upstream_485_DisableRxInt();

        locRxBufferRead  = Upstream_485_rxBufferRead;
        locRxBufferWrite = Upstream_485_rxBufferWrite;

        if( (Upstream_485_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = Upstream_485_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= Upstream_485_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            Upstream_485_rxBufferRead = locRxBufferRead;

            if(Upstream_485_rxBufferLoopDetect != 0u)
            {
                Upstream_485_rxBufferLoopDetect = 0u;
                #if ((Upstream_485_RX_INTERRUPT_ENABLED) && (Upstream_485_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( Upstream_485_HD_ENABLED )
                        if((Upstream_485_CONTROL_REG & Upstream_485_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            Upstream_485_RXSTATUS_MASK_REG  |= Upstream_485_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        Upstream_485_RXSTATUS_MASK_REG  |= Upstream_485_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end Upstream_485_HD_ENABLED */
                #endif /* ((Upstream_485_RX_INTERRUPT_ENABLED) && (Upstream_485_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = Upstream_485_RXDATA_REG;
        }

        Upstream_485_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = Upstream_485_RXDATA_REG;

    #endif /* (Upstream_485_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Upstream_485_ReadRxStatus
    ********************************************************************************
    *
    * Summary:
    *  Returns the current state of the receiver status register and the software
    *  buffer overflow status.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Current state of the status register.
    *
    * Side Effect:
    *  All status register bits are clear-on-read except
    *  Upstream_485_RX_STS_FIFO_NOTEMPTY.
    *  Upstream_485_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  Upstream_485_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   Upstream_485_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   Upstream_485_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 Upstream_485_ReadRxStatus(void) 
    {
        uint8 status;

        status = Upstream_485_RXSTATUS_REG & Upstream_485_RX_HW_MASK;

    #if (Upstream_485_RX_INTERRUPT_ENABLED)
        if(Upstream_485_rxBufferOverflow != 0u)
        {
            status |= Upstream_485_RX_STS_SOFT_BUFF_OVER;
            Upstream_485_rxBufferOverflow = 0u;
        }
    #endif /* (Upstream_485_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: Upstream_485_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. Upstream_485_GetChar() is
    *  designed for ASCII characters and returns a uint8 where 1 to 255 are values
    *  for valid characters and 0 indicates an error occurred or no data is present.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Character read from UART RX buffer. ASCII characters from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    * Global Variables:
    *  Upstream_485_rxBuffer - RAM buffer pointer for save received data.
    *  Upstream_485_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Upstream_485_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Upstream_485_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Upstream_485_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (Upstream_485_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        Upstream_485_DisableRxInt();

        locRxBufferRead  = Upstream_485_rxBufferRead;
        locRxBufferWrite = Upstream_485_rxBufferWrite;

        if( (Upstream_485_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = Upstream_485_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= Upstream_485_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            Upstream_485_rxBufferRead = locRxBufferRead;

            if(Upstream_485_rxBufferLoopDetect != 0u)
            {
                Upstream_485_rxBufferLoopDetect = 0u;
                #if( (Upstream_485_RX_INTERRUPT_ENABLED) && (Upstream_485_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( Upstream_485_HD_ENABLED )
                        if((Upstream_485_CONTROL_REG & Upstream_485_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            Upstream_485_RXSTATUS_MASK_REG |= Upstream_485_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        Upstream_485_RXSTATUS_MASK_REG |= Upstream_485_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end Upstream_485_HD_ENABLED */
                #endif /* Upstream_485_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = Upstream_485_RXSTATUS_REG;
            if((rxStatus & Upstream_485_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = Upstream_485_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (Upstream_485_RX_STS_BREAK | Upstream_485_RX_STS_PAR_ERROR |
                                Upstream_485_RX_STS_STOP_ERROR | Upstream_485_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        Upstream_485_EnableRxInt();

    #else

        rxStatus =Upstream_485_RXSTATUS_REG;
        if((rxStatus & Upstream_485_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = Upstream_485_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (Upstream_485_RX_STS_BREAK | Upstream_485_RX_STS_PAR_ERROR |
                            Upstream_485_RX_STS_STOP_ERROR | Upstream_485_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (Upstream_485_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Upstream_485_GetByte
    ********************************************************************************
    *
    * Summary:
    *  Reads UART RX buffer immediately, returns received character and error
    *  condition.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  MSB contains status and LSB contains UART RX data. If the MSB is nonzero,
    *  an error has occurred.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint16 Upstream_485_GetByte(void) 
    {
        
    #if (Upstream_485_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        Upstream_485_DisableRxInt();
        locErrorStatus = (uint16)Upstream_485_errorStatus;
        Upstream_485_errorStatus = 0u;
        Upstream_485_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | Upstream_485_ReadRxData() );
    #else
        return ( ((uint16)Upstream_485_ReadRxStatus() << 8u) | Upstream_485_ReadRxData() );
    #endif /* Upstream_485_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: Upstream_485_GetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of received bytes available in the RX buffer.
    *  * RX software buffer is disabled (RX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty RX FIFO or 1 for not empty RX FIFO.
    *  * RX software buffer is enabled: returns the number of bytes available in 
    *    the RX software buffer. Bytes available in the RX FIFO do not take to 
    *    account.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  uint8: Number of bytes in the RX buffer. 
    *    Return value type depends on RX Buffer Size parameter.
    *
    * Global Variables:
    *  Upstream_485_rxBufferWrite - used to calculate left bytes.
    *  Upstream_485_rxBufferRead - used to calculate left bytes.
    *  Upstream_485_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 Upstream_485_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (Upstream_485_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        Upstream_485_DisableRxInt();

        if(Upstream_485_rxBufferRead == Upstream_485_rxBufferWrite)
        {
            if(Upstream_485_rxBufferLoopDetect != 0u)
            {
                size = Upstream_485_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(Upstream_485_rxBufferRead < Upstream_485_rxBufferWrite)
        {
            size = (Upstream_485_rxBufferWrite - Upstream_485_rxBufferRead);
        }
        else
        {
            size = (Upstream_485_RX_BUFFER_SIZE - Upstream_485_rxBufferRead) + Upstream_485_rxBufferWrite;
        }

        Upstream_485_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((Upstream_485_RXSTATUS_REG & Upstream_485_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (Upstream_485_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Upstream_485_ClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the receiver memory buffer and hardware RX FIFO of all received data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Upstream_485_rxBufferWrite - cleared to zero.
    *  Upstream_485_rxBufferRead - cleared to zero.
    *  Upstream_485_rxBufferLoopDetect - cleared to zero.
    *  Upstream_485_rxBufferOverflow - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may
    *  have remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM or FIFO buffer will be lost.
    *
    *******************************************************************************/
    void Upstream_485_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        Upstream_485_RXDATA_AUX_CTL_REG |= (uint8)  Upstream_485_RX_FIFO_CLR;
        Upstream_485_RXDATA_AUX_CTL_REG &= (uint8) ~Upstream_485_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (Upstream_485_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        Upstream_485_DisableRxInt();

        Upstream_485_rxBufferRead = 0u;
        Upstream_485_rxBufferWrite = 0u;
        Upstream_485_rxBufferLoopDetect = 0u;
        Upstream_485_rxBufferOverflow = 0u;

        Upstream_485_EnableRxInt();

    #endif /* (Upstream_485_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: Upstream_485_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  Upstream_485__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  Upstream_485__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  Upstream_485__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  Upstream_485__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  Upstream_485__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Upstream_485_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  Upstream_485_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void Upstream_485_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(Upstream_485_RXHW_ADDRESS_ENABLED)
            #if(Upstream_485_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* Upstream_485_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = Upstream_485_CONTROL_REG & (uint8)~Upstream_485_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << Upstream_485_CTRL_RXADDR_MODE0_SHIFT);
                Upstream_485_CONTROL_REG = tmpCtrl;

                #if(Upstream_485_RX_INTERRUPT_ENABLED && \
                   (Upstream_485_RXBUFFERSIZE > Upstream_485_FIFO_LENGTH) )
                    Upstream_485_rxAddressMode = addressMode;
                    Upstream_485_rxAddressDetected = 0u;
                #endif /* End Upstream_485_RXBUFFERSIZE > Upstream_485_FIFO_LENGTH*/
            #endif /* End Upstream_485_CONTROL_REG_REMOVED */
        #else /* Upstream_485_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End Upstream_485_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Upstream_485_SetRxAddress1
    ********************************************************************************
    *
    * Summary:
    *  Sets the first of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #1 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void Upstream_485_SetRxAddress1(uint8 address) 
    {
        Upstream_485_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: Upstream_485_SetRxAddress2
    ********************************************************************************
    *
    * Summary:
    *  Sets the second of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #2 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void Upstream_485_SetRxAddress2(uint8 address) 
    {
        Upstream_485_RXADDRESS2_REG = address;
    }

#endif  /* Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED*/


#if( (Upstream_485_TX_ENABLED) || (Upstream_485_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: Upstream_485_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   Upstream_485_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   Upstream_485_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   Upstream_485_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   Upstream_485_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void Upstream_485_SetTxInterruptMode(uint8 intSrc) 
    {
        Upstream_485_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: Upstream_485_WriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Places a byte of data into the transmit buffer to be sent when the bus is
    *  available without checking the TX status register. You must check status
    *  separately.
    *
    * Parameters:
    *  txDataByte: data byte
    *
    * Return:
    * None.
    *
    * Global Variables:
    *  Upstream_485_txBuffer - RAM buffer pointer for save data for transmission
    *  Upstream_485_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  Upstream_485_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  Upstream_485_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Upstream_485_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Upstream_485_initVar != 0u)
        {
        #if (Upstream_485_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            Upstream_485_DisableTxInt();

            if( (Upstream_485_txBufferRead == Upstream_485_txBufferWrite) &&
                ((Upstream_485_TXSTATUS_REG & Upstream_485_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                Upstream_485_TXDATA_REG = txDataByte;
            }
            else
            {
                if(Upstream_485_txBufferWrite >= Upstream_485_TX_BUFFER_SIZE)
                {
                    Upstream_485_txBufferWrite = 0u;
                }

                Upstream_485_txBuffer[Upstream_485_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                Upstream_485_txBufferWrite++;
            }

            Upstream_485_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            Upstream_485_TXDATA_REG = txDataByte;

        #endif /*(Upstream_485_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: Upstream_485_ReadTxStatus
    ********************************************************************************
    *
    * Summary:
    *  Reads the status register for the TX portion of the UART.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Contents of the status register
    *
    * Theory:
    *  This function reads the TX status register, which is cleared on read.
    *  It is up to the user to handle all bits in this return value accordingly,
    *  even if the bit was not enabled as an interrupt source the event happened
    *  and must be handled accordingly.
    *
    *******************************************************************************/
    uint8 Upstream_485_ReadTxStatus(void) 
    {
        return(Upstream_485_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: Upstream_485_PutChar
    ********************************************************************************
    *
    * Summary:
    *  Puts a byte of data into the transmit buffer to be sent when the bus is
    *  available. This is a blocking API that waits until the TX buffer has room to
    *  hold the data.
    *
    * Parameters:
    *  txDataByte: Byte containing the data to transmit
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Upstream_485_txBuffer - RAM buffer pointer for save data for transmission
    *  Upstream_485_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  Upstream_485_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  Upstream_485_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void Upstream_485_PutChar(uint8 txDataByte) 
    {
    #if (Upstream_485_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((Upstream_485_TX_BUFFER_SIZE > Upstream_485_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            Upstream_485_DisableTxInt();
        #endif /* (Upstream_485_TX_BUFFER_SIZE > Upstream_485_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = Upstream_485_txBufferWrite;
            locTxBufferRead  = Upstream_485_txBufferRead;

        #if ((Upstream_485_TX_BUFFER_SIZE > Upstream_485_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            Upstream_485_EnableTxInt();
        #endif /* (Upstream_485_TX_BUFFER_SIZE > Upstream_485_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(Upstream_485_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((Upstream_485_TXSTATUS_REG & Upstream_485_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            Upstream_485_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= Upstream_485_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            Upstream_485_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((Upstream_485_TX_BUFFER_SIZE > Upstream_485_MAX_BYTE_VALUE) && (CY_PSOC3))
            Upstream_485_DisableTxInt();
        #endif /* (Upstream_485_TX_BUFFER_SIZE > Upstream_485_MAX_BYTE_VALUE) && (CY_PSOC3) */

            Upstream_485_txBufferWrite = locTxBufferWrite;

        #if ((Upstream_485_TX_BUFFER_SIZE > Upstream_485_MAX_BYTE_VALUE) && (CY_PSOC3))
            Upstream_485_EnableTxInt();
        #endif /* (Upstream_485_TX_BUFFER_SIZE > Upstream_485_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (Upstream_485_TXSTATUS_REG & Upstream_485_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                Upstream_485_SetPendingTxInt();
            }
        }

    #else

        while((Upstream_485_TXSTATUS_REG & Upstream_485_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        Upstream_485_TXDATA_REG = txDataByte;

    #endif /* Upstream_485_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Upstream_485_PutString
    ********************************************************************************
    *
    * Summary:
    *  Sends a NULL terminated string to the TX buffer for transmission.
    *
    * Parameters:
    *  string[]: Pointer to the null terminated string array residing in RAM or ROM
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Upstream_485_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void Upstream_485_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(Upstream_485_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                Upstream_485_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Upstream_485_PutArray
    ********************************************************************************
    *
    * Summary:
    *  Places N bytes of data from a memory array into the TX buffer for
    *  transmission.
    *
    * Parameters:
    *  string[]: Address of the memory array residing in RAM or ROM.
    *  byteCount: Number of bytes to be transmitted. The type depends on TX Buffer
    *             Size parameter.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Upstream_485_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void Upstream_485_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(Upstream_485_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                Upstream_485_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Upstream_485_PutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Writes a byte of data followed by a carriage return (0x0D) and line feed
    *  (0x0A) to the transmit buffer.
    *
    * Parameters:
    *  txDataByte: Data byte to transmit before the carriage return and line feed.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Upstream_485_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Upstream_485_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(Upstream_485_initVar != 0u)
        {
            Upstream_485_PutChar(txDataByte);
            Upstream_485_PutChar(0x0Du);
            Upstream_485_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: Upstream_485_GetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of bytes in the TX buffer which are waiting to be 
    *  transmitted.
    *  * TX software buffer is disabled (TX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty TX FIFO, 1 for not full TX FIFO or 4 for full TX FIFO.
    *  * TX software buffer is enabled: returns the number of bytes in the TX 
    *    software buffer which are waiting to be transmitted. Bytes available in the
    *    TX FIFO do not count.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Number of bytes used in the TX buffer. Return value type depends on the TX 
    *  Buffer Size parameter.
    *
    * Global Variables:
    *  Upstream_485_txBufferWrite - used to calculate left space.
    *  Upstream_485_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 Upstream_485_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (Upstream_485_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        Upstream_485_DisableTxInt();

        if(Upstream_485_txBufferRead == Upstream_485_txBufferWrite)
        {
            size = 0u;
        }
        else if(Upstream_485_txBufferRead < Upstream_485_txBufferWrite)
        {
            size = (Upstream_485_txBufferWrite - Upstream_485_txBufferRead);
        }
        else
        {
            size = (Upstream_485_TX_BUFFER_SIZE - Upstream_485_txBufferRead) +
                    Upstream_485_txBufferWrite;
        }

        Upstream_485_EnableTxInt();

    #else

        size = Upstream_485_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & Upstream_485_TX_STS_FIFO_FULL) != 0u)
        {
            size = Upstream_485_FIFO_LENGTH;
        }
        else if((size & Upstream_485_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (Upstream_485_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: Upstream_485_ClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears all data from the TX buffer and hardware TX FIFO.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Upstream_485_txBufferWrite - cleared to zero.
    *  Upstream_485_txBufferRead - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may have
    *  remained in the RAM.
    *
    * Side Effects:
    *  Data waiting in the transmit buffer is not sent; a byte that is currently
    *  transmitting finishes transmitting.
    *
    *******************************************************************************/
    void Upstream_485_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        Upstream_485_TXDATA_AUX_CTL_REG |= (uint8)  Upstream_485_TX_FIFO_CLR;
        Upstream_485_TXDATA_AUX_CTL_REG &= (uint8) ~Upstream_485_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (Upstream_485_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        Upstream_485_DisableTxInt();

        Upstream_485_txBufferRead = 0u;
        Upstream_485_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        Upstream_485_EnableTxInt();

    #endif /* (Upstream_485_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: Upstream_485_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   Upstream_485_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   Upstream_485_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   Upstream_485_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   Upstream_485_SEND_WAIT_REINIT - Performs both options: 
    *      Upstream_485_SEND_BREAK and Upstream_485_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Upstream_485_initVar - checked to identify that the component has been
    *     initialized.
    *  txPeriod - static variable, used for keeping TX period configuration.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  SendBreak function initializes registers to send 13-bit break signal. It is
    *  important to return the registers configuration to normal for continue 8-bit
    *  operation.
    *  There are 3 variants for this API usage:
    *  1) SendBreak(3) - function will send the Break signal and take care on the
    *     configuration returning. Function will block CPU until transmission
    *     complete.
    *  2) User may want to use blocking time if UART configured to the low speed
    *     operation
    *     Example for this case:
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     SendBreak(1);     - complete Break operation
    *  3) Same to 2) but user may want to initialize and use the interrupt to
    *     complete break operation.
    *     Example for this case:
    *     Initialize TX interrupt with "TX - On TX Complete" parameter
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     When interrupt appear with Upstream_485_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The Upstream_485_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void Upstream_485_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(Upstream_485_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(Upstream_485_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == Upstream_485_SEND_BREAK) ||
                (retMode == Upstream_485_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                Upstream_485_WriteControlRegister(Upstream_485_ReadControlRegister() |
                                                      Upstream_485_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                Upstream_485_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = Upstream_485_TXSTATUS_REG;
                }
                while((tmpStat & Upstream_485_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == Upstream_485_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == Upstream_485_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = Upstream_485_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & Upstream_485_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == Upstream_485_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == Upstream_485_REINIT) ||
                (retMode == Upstream_485_SEND_WAIT_REINIT) )
            {
                Upstream_485_WriteControlRegister(Upstream_485_ReadControlRegister() &
                                              (uint8)~Upstream_485_CTRL_HD_SEND_BREAK);
            }

        #else /* Upstream_485_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == Upstream_485_SEND_BREAK) ||
                (retMode == Upstream_485_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (Upstream_485_PARITY_TYPE != Upstream_485__B_UART__NONE_REVB) || \
                                    (Upstream_485_PARITY_TYPE_SW != 0u) )
                    Upstream_485_WriteControlRegister(Upstream_485_ReadControlRegister() |
                                                          Upstream_485_CTRL_HD_SEND_BREAK);
                #endif /* End Upstream_485_PARITY_TYPE != Upstream_485__B_UART__NONE_REVB  */

                #if(Upstream_485_TXCLKGEN_DP)
                    txPeriod = Upstream_485_TXBITCLKTX_COMPLETE_REG;
                    Upstream_485_TXBITCLKTX_COMPLETE_REG = Upstream_485_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = Upstream_485_TXBITCTR_PERIOD_REG;
                    Upstream_485_TXBITCTR_PERIOD_REG = Upstream_485_TXBITCTR_BREAKBITS8X;
                #endif /* End Upstream_485_TXCLKGEN_DP */

                /* Send zeros */
                Upstream_485_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = Upstream_485_TXSTATUS_REG;
                }
                while((tmpStat & Upstream_485_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == Upstream_485_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == Upstream_485_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = Upstream_485_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & Upstream_485_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == Upstream_485_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == Upstream_485_REINIT) ||
                (retMode == Upstream_485_SEND_WAIT_REINIT) )
            {

            #if(Upstream_485_TXCLKGEN_DP)
                Upstream_485_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                Upstream_485_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End Upstream_485_TXCLKGEN_DP */

            #if( (Upstream_485_PARITY_TYPE != Upstream_485__B_UART__NONE_REVB) || \
                 (Upstream_485_PARITY_TYPE_SW != 0u) )
                Upstream_485_WriteControlRegister(Upstream_485_ReadControlRegister() &
                                                      (uint8) ~Upstream_485_CTRL_HD_SEND_BREAK);
            #endif /* End Upstream_485_PARITY_TYPE != NONE */
            }
        #endif    /* End Upstream_485_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: Upstream_485_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       Upstream_485_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       Upstream_485_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears Upstream_485_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void Upstream_485_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( Upstream_485_CONTROL_REG_REMOVED == 0u )
            Upstream_485_WriteControlRegister(Upstream_485_ReadControlRegister() |
                                                  Upstream_485_CTRL_MARK);
        #endif /* End Upstream_485_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( Upstream_485_CONTROL_REG_REMOVED == 0u )
            Upstream_485_WriteControlRegister(Upstream_485_ReadControlRegister() &
                                                  (uint8) ~Upstream_485_CTRL_MARK);
        #endif /* End Upstream_485_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndUpstream_485_TX_ENABLED */

#if(Upstream_485_HD_ENABLED)


    /*******************************************************************************
    * Function Name: Upstream_485_LoadRxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the receiver configuration in half duplex mode. After calling this
    *  function, the UART is ready to receive data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the transmitter
    *  configuration.
    *
    *******************************************************************************/
    void Upstream_485_LoadRxConfig(void) 
    {
        Upstream_485_WriteControlRegister(Upstream_485_ReadControlRegister() &
                                                (uint8)~Upstream_485_CTRL_HD_SEND);
        Upstream_485_RXBITCTR_PERIOD_REG = Upstream_485_HD_RXBITCTR_INIT;

    #if (Upstream_485_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        Upstream_485_SetRxInterruptMode(Upstream_485_INIT_RX_INTERRUPTS_MASK);
    #endif /* (Upstream_485_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: Upstream_485_LoadTxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the transmitter configuration in half duplex mode. After calling this
    *  function, the UART is ready to transmit data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the receiver configuration.
    *
    *******************************************************************************/
    void Upstream_485_LoadTxConfig(void) 
    {
    #if (Upstream_485_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        Upstream_485_SetRxInterruptMode(0u);
    #endif /* (Upstream_485_RX_INTERRUPT_ENABLED) */

        Upstream_485_WriteControlRegister(Upstream_485_ReadControlRegister() | Upstream_485_CTRL_HD_SEND);
        Upstream_485_RXBITCTR_PERIOD_REG = Upstream_485_HD_TXBITCTR_INIT;
    }

#endif  /* Upstream_485_HD_ENABLED */


/* [] END OF FILE */
