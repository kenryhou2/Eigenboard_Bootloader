/*******************************************************************************
* File Name: Upstream_485INT.c
* Version 2.50
*
* Description:
*  This file provides all Interrupt Service functionality of the UART component
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Upstream_485.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (Upstream_485_RX_INTERRUPT_ENABLED && (Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED))
    /*******************************************************************************
    * Function Name: Upstream_485_RXISR
    ********************************************************************************
    *
    * Summary:
    *  Interrupt Service Routine for RX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Upstream_485_rxBuffer - RAM buffer pointer for save received data.
    *  Upstream_485_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  Upstream_485_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  Upstream_485_rxBufferOverflow - software overflow flag. Set to one
    *     when Upstream_485_rxBufferWrite index overtakes
    *     Upstream_485_rxBufferRead index.
    *  Upstream_485_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when Upstream_485_rxBufferWrite is equal to
    *    Upstream_485_rxBufferRead
    *  Upstream_485_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  Upstream_485_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(Upstream_485_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef Upstream_485_RXISR_ENTRY_CALLBACK
        Upstream_485_RXISR_EntryCallback();
    #endif /* Upstream_485_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START Upstream_485_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = Upstream_485_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in Upstream_485_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (Upstream_485_RX_STS_BREAK | 
                            Upstream_485_RX_STS_PAR_ERROR |
                            Upstream_485_RX_STS_STOP_ERROR | 
                            Upstream_485_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                Upstream_485_errorStatus |= readStatus & ( Upstream_485_RX_STS_BREAK | 
                                                            Upstream_485_RX_STS_PAR_ERROR | 
                                                            Upstream_485_RX_STS_STOP_ERROR | 
                                                            Upstream_485_RX_STS_OVERRUN);
                /* `#START Upstream_485_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef Upstream_485_RXISR_ERROR_CALLBACK
                Upstream_485_RXISR_ERROR_Callback();
            #endif /* Upstream_485_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & Upstream_485_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = Upstream_485_RXDATA_REG;
            #if (Upstream_485_RXHW_ADDRESS_ENABLED)
                if(Upstream_485_rxAddressMode == (uint8)Upstream_485__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & Upstream_485_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & Upstream_485_RX_STS_ADDR_MATCH) != 0u)
                        {
                            Upstream_485_rxAddressDetected = 1u;
                        }
                        else
                        {
                            Upstream_485_rxAddressDetected = 0u;
                        }
                    }
                    if(Upstream_485_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        Upstream_485_rxBuffer[Upstream_485_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    Upstream_485_rxBuffer[Upstream_485_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                Upstream_485_rxBuffer[Upstream_485_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (Upstream_485_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(Upstream_485_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        Upstream_485_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    Upstream_485_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(Upstream_485_rxBufferWrite >= Upstream_485_RX_BUFFER_SIZE)
                    {
                        Upstream_485_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(Upstream_485_rxBufferWrite == Upstream_485_rxBufferRead)
                    {
                        Upstream_485_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (Upstream_485_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            Upstream_485_RXSTATUS_MASK_REG  &= (uint8)~Upstream_485_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(Upstream_485_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (Upstream_485_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & Upstream_485_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START Upstream_485_RXISR_END` */

        /* `#END` */

    #ifdef Upstream_485_RXISR_EXIT_CALLBACK
        Upstream_485_RXISR_ExitCallback();
    #endif /* Upstream_485_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (Upstream_485_RX_INTERRUPT_ENABLED && (Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED)) */


#if (Upstream_485_TX_INTERRUPT_ENABLED && Upstream_485_TX_ENABLED)
    /*******************************************************************************
    * Function Name: Upstream_485_TXISR
    ********************************************************************************
    *
    * Summary:
    * Interrupt Service Routine for the TX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Upstream_485_txBuffer - RAM buffer pointer for transmit data from.
    *  Upstream_485_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  Upstream_485_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(Upstream_485_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef Upstream_485_TXISR_ENTRY_CALLBACK
        Upstream_485_TXISR_EntryCallback();
    #endif /* Upstream_485_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START Upstream_485_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((Upstream_485_txBufferRead != Upstream_485_txBufferWrite) &&
             ((Upstream_485_TXSTATUS_REG & Upstream_485_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(Upstream_485_txBufferRead >= Upstream_485_TX_BUFFER_SIZE)
            {
                Upstream_485_txBufferRead = 0u;
            }

            Upstream_485_TXDATA_REG = Upstream_485_txBuffer[Upstream_485_txBufferRead];

            /* Set next pointer */
            Upstream_485_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Upstream_485_TXISR_END` */

        /* `#END` */

    #ifdef Upstream_485_TXISR_EXIT_CALLBACK
        Upstream_485_TXISR_ExitCallback();
    #endif /* Upstream_485_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (Upstream_485_TX_INTERRUPT_ENABLED && Upstream_485_TX_ENABLED) */


/* [] END OF FILE */
