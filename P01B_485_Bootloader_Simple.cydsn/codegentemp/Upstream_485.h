/*******************************************************************************
* File Name: Upstream_485.h
* Version 2.50
*
* Description:
*  Contains the function prototypes and constants available to the UART
*  user module.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_UART_Upstream_485_H)
#define CY_UART_Upstream_485_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */


/***************************************
* Conditional Compilation Parameters
***************************************/

#define Upstream_485_RX_ENABLED                     (1u)
#define Upstream_485_TX_ENABLED                     (1u)
#define Upstream_485_HD_ENABLED                     (0u)
#define Upstream_485_RX_INTERRUPT_ENABLED           (1u)
#define Upstream_485_TX_INTERRUPT_ENABLED           (1u)
#define Upstream_485_INTERNAL_CLOCK_USED            (1u)
#define Upstream_485_RXHW_ADDRESS_ENABLED           (0u)
#define Upstream_485_OVER_SAMPLE_COUNT              (8u)
#define Upstream_485_PARITY_TYPE                    (0u)
#define Upstream_485_PARITY_TYPE_SW                 (0u)
#define Upstream_485_BREAK_DETECT                   (0u)
#define Upstream_485_BREAK_BITS_TX                  (13u)
#define Upstream_485_BREAK_BITS_RX                  (13u)
#define Upstream_485_TXCLKGEN_DP                    (1u)
#define Upstream_485_USE23POLLING                   (1u)
#define Upstream_485_FLOW_CONTROL                   (0u)
#define Upstream_485_CLK_FREQ                       (0u)
#define Upstream_485_TX_BUFFER_SIZE                 (128u)
#define Upstream_485_RX_BUFFER_SIZE                 (128u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(Upstream_485_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define Upstream_485_CONTROL_REG_REMOVED            (0u)
#else
    #define Upstream_485_CONTROL_REG_REMOVED            (1u)
#endif /* End Upstream_485_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct Upstream_485_backupStruct_
{
    uint8 enableState;

    #if(Upstream_485_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End Upstream_485_CONTROL_REG_REMOVED */

} Upstream_485_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void Upstream_485_Start(void) ;
void Upstream_485_Stop(void) ;
uint8 Upstream_485_ReadControlRegister(void) ;
void Upstream_485_WriteControlRegister(uint8 control) ;

void Upstream_485_Init(void) ;
void Upstream_485_Enable(void) ;
void Upstream_485_SaveConfig(void) ;
void Upstream_485_RestoreConfig(void) ;
void Upstream_485_Sleep(void) ;
void Upstream_485_Wakeup(void) ;

/* Only if RX is enabled */
#if( (Upstream_485_RX_ENABLED) || (Upstream_485_HD_ENABLED) )

    #if (Upstream_485_RX_INTERRUPT_ENABLED)
        #define Upstream_485_EnableRxInt()  CyIntEnable (Upstream_485_RX_VECT_NUM)
        #define Upstream_485_DisableRxInt() CyIntDisable(Upstream_485_RX_VECT_NUM)
        CY_ISR_PROTO(Upstream_485_RXISR);
    #endif /* Upstream_485_RX_INTERRUPT_ENABLED */

    void Upstream_485_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void Upstream_485_SetRxAddress1(uint8 address) ;
    void Upstream_485_SetRxAddress2(uint8 address) ;

    void  Upstream_485_SetRxInterruptMode(uint8 intSrc) ;
    uint8 Upstream_485_ReadRxData(void) ;
    uint8 Upstream_485_ReadRxStatus(void) ;
    uint8 Upstream_485_GetChar(void) ;
    uint16 Upstream_485_GetByte(void) ;
    uint8 Upstream_485_GetRxBufferSize(void)
                                                            ;
    void Upstream_485_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define Upstream_485_GetRxInterruptSource   Upstream_485_ReadRxStatus

#endif /* End (Upstream_485_RX_ENABLED) || (Upstream_485_HD_ENABLED) */

/* Only if TX is enabled */
#if(Upstream_485_TX_ENABLED || Upstream_485_HD_ENABLED)

    #if(Upstream_485_TX_INTERRUPT_ENABLED)
        #define Upstream_485_EnableTxInt()  CyIntEnable (Upstream_485_TX_VECT_NUM)
        #define Upstream_485_DisableTxInt() CyIntDisable(Upstream_485_TX_VECT_NUM)
        #define Upstream_485_SetPendingTxInt() CyIntSetPending(Upstream_485_TX_VECT_NUM)
        #define Upstream_485_ClearPendingTxInt() CyIntClearPending(Upstream_485_TX_VECT_NUM)
        CY_ISR_PROTO(Upstream_485_TXISR);
    #endif /* Upstream_485_TX_INTERRUPT_ENABLED */

    void Upstream_485_SetTxInterruptMode(uint8 intSrc) ;
    void Upstream_485_WriteTxData(uint8 txDataByte) ;
    uint8 Upstream_485_ReadTxStatus(void) ;
    void Upstream_485_PutChar(uint8 txDataByte) ;
    void Upstream_485_PutString(const char8 string[]) ;
    void Upstream_485_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void Upstream_485_PutCRLF(uint8 txDataByte) ;
    void Upstream_485_ClearTxBuffer(void) ;
    void Upstream_485_SetTxAddressMode(uint8 addressMode) ;
    void Upstream_485_SendBreak(uint8 retMode) ;
    uint8 Upstream_485_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define Upstream_485_PutStringConst         Upstream_485_PutString
    #define Upstream_485_PutArrayConst          Upstream_485_PutArray
    #define Upstream_485_GetTxInterruptSource   Upstream_485_ReadTxStatus

#endif /* End Upstream_485_TX_ENABLED || Upstream_485_HD_ENABLED */

#if(Upstream_485_HD_ENABLED)
    void Upstream_485_LoadRxConfig(void) ;
    void Upstream_485_LoadTxConfig(void) ;
#endif /* End Upstream_485_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Upstream_485) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    Upstream_485_CyBtldrCommStart(void) CYSMALL ;
    void    Upstream_485_CyBtldrCommStop(void) CYSMALL ;
    void    Upstream_485_CyBtldrCommReset(void) CYSMALL ;
    cystatus Upstream_485_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus Upstream_485_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Upstream_485)
        #define CyBtldrCommStart    Upstream_485_CyBtldrCommStart
        #define CyBtldrCommStop     Upstream_485_CyBtldrCommStop
        #define CyBtldrCommReset    Upstream_485_CyBtldrCommReset
        #define CyBtldrCommWrite    Upstream_485_CyBtldrCommWrite
        #define CyBtldrCommRead     Upstream_485_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Upstream_485) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define Upstream_485_BYTE2BYTE_TIME_OUT (25u)
    #define Upstream_485_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define Upstream_485_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define Upstream_485_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define Upstream_485_SET_SPACE      (0x00u)
#define Upstream_485_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (Upstream_485_TX_ENABLED) || (Upstream_485_HD_ENABLED) )
    #if(Upstream_485_TX_INTERRUPT_ENABLED)
        #define Upstream_485_TX_VECT_NUM            (uint8)Upstream_485_TXInternalInterrupt__INTC_NUMBER
        #define Upstream_485_TX_PRIOR_NUM           (uint8)Upstream_485_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Upstream_485_TX_INTERRUPT_ENABLED */

    #define Upstream_485_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define Upstream_485_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define Upstream_485_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(Upstream_485_TX_ENABLED)
        #define Upstream_485_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (Upstream_485_HD_ENABLED) */
        #define Upstream_485_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (Upstream_485_TX_ENABLED) */

    #define Upstream_485_TX_STS_COMPLETE            (uint8)(0x01u << Upstream_485_TX_STS_COMPLETE_SHIFT)
    #define Upstream_485_TX_STS_FIFO_EMPTY          (uint8)(0x01u << Upstream_485_TX_STS_FIFO_EMPTY_SHIFT)
    #define Upstream_485_TX_STS_FIFO_FULL           (uint8)(0x01u << Upstream_485_TX_STS_FIFO_FULL_SHIFT)
    #define Upstream_485_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << Upstream_485_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (Upstream_485_TX_ENABLED) || (Upstream_485_HD_ENABLED)*/

#if( (Upstream_485_RX_ENABLED) || (Upstream_485_HD_ENABLED) )
    #if(Upstream_485_RX_INTERRUPT_ENABLED)
        #define Upstream_485_RX_VECT_NUM            (uint8)Upstream_485_RXInternalInterrupt__INTC_NUMBER
        #define Upstream_485_RX_PRIOR_NUM           (uint8)Upstream_485_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Upstream_485_RX_INTERRUPT_ENABLED */
    #define Upstream_485_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define Upstream_485_RX_STS_BREAK_SHIFT             (0x01u)
    #define Upstream_485_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define Upstream_485_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define Upstream_485_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define Upstream_485_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define Upstream_485_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define Upstream_485_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define Upstream_485_RX_STS_MRKSPC           (uint8)(0x01u << Upstream_485_RX_STS_MRKSPC_SHIFT)
    #define Upstream_485_RX_STS_BREAK            (uint8)(0x01u << Upstream_485_RX_STS_BREAK_SHIFT)
    #define Upstream_485_RX_STS_PAR_ERROR        (uint8)(0x01u << Upstream_485_RX_STS_PAR_ERROR_SHIFT)
    #define Upstream_485_RX_STS_STOP_ERROR       (uint8)(0x01u << Upstream_485_RX_STS_STOP_ERROR_SHIFT)
    #define Upstream_485_RX_STS_OVERRUN          (uint8)(0x01u << Upstream_485_RX_STS_OVERRUN_SHIFT)
    #define Upstream_485_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << Upstream_485_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define Upstream_485_RX_STS_ADDR_MATCH       (uint8)(0x01u << Upstream_485_RX_STS_ADDR_MATCH_SHIFT)
    #define Upstream_485_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << Upstream_485_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define Upstream_485_RX_HW_MASK                     (0x7Fu)
#endif /* End (Upstream_485_RX_ENABLED) || (Upstream_485_HD_ENABLED) */

/* Control Register definitions */
#define Upstream_485_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define Upstream_485_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define Upstream_485_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define Upstream_485_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define Upstream_485_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define Upstream_485_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define Upstream_485_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define Upstream_485_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define Upstream_485_CTRL_HD_SEND               (uint8)(0x01u << Upstream_485_CTRL_HD_SEND_SHIFT)
#define Upstream_485_CTRL_HD_SEND_BREAK         (uint8)(0x01u << Upstream_485_CTRL_HD_SEND_BREAK_SHIFT)
#define Upstream_485_CTRL_MARK                  (uint8)(0x01u << Upstream_485_CTRL_MARK_SHIFT)
#define Upstream_485_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << Upstream_485_CTRL_PARITY_TYPE0_SHIFT)
#define Upstream_485_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << Upstream_485_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define Upstream_485_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define Upstream_485_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define Upstream_485_SEND_BREAK                         (0x00u)
#define Upstream_485_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define Upstream_485_REINIT                             (0x02u)
#define Upstream_485_SEND_WAIT_REINIT                   (0x03u)

#define Upstream_485_OVER_SAMPLE_8                      (8u)
#define Upstream_485_OVER_SAMPLE_16                     (16u)

#define Upstream_485_BIT_CENTER                         (Upstream_485_OVER_SAMPLE_COUNT - 2u)

#define Upstream_485_FIFO_LENGTH                        (4u)
#define Upstream_485_NUMBER_OF_START_BIT                (1u)
#define Upstream_485_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define Upstream_485_TXBITCTR_BREAKBITS8X   ((Upstream_485_BREAK_BITS_TX * Upstream_485_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define Upstream_485_TXBITCTR_BREAKBITS ((Upstream_485_BREAK_BITS_TX * Upstream_485_OVER_SAMPLE_COUNT) - 1u)

#define Upstream_485_HALF_BIT_COUNT   \
                            (((Upstream_485_OVER_SAMPLE_COUNT / 2u) + (Upstream_485_USE23POLLING * 1u)) - 2u)
#if (Upstream_485_OVER_SAMPLE_COUNT == Upstream_485_OVER_SAMPLE_8)
    #define Upstream_485_HD_TXBITCTR_INIT   (((Upstream_485_BREAK_BITS_TX + \
                            Upstream_485_NUMBER_OF_START_BIT) * Upstream_485_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define Upstream_485_RXBITCTR_INIT  ((((Upstream_485_BREAK_BITS_RX + Upstream_485_NUMBER_OF_START_BIT) \
                            * Upstream_485_OVER_SAMPLE_COUNT) + Upstream_485_HALF_BIT_COUNT) - 1u)

#else /* Upstream_485_OVER_SAMPLE_COUNT == Upstream_485_OVER_SAMPLE_16 */
    #define Upstream_485_HD_TXBITCTR_INIT   ((8u * Upstream_485_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define Upstream_485_RXBITCTR_INIT      (((7u * Upstream_485_OVER_SAMPLE_COUNT) - 1u) + \
                                                      Upstream_485_HALF_BIT_COUNT)
#endif /* End Upstream_485_OVER_SAMPLE_COUNT */

#define Upstream_485_HD_RXBITCTR_INIT                   Upstream_485_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 Upstream_485_initVar;
#if (Upstream_485_TX_INTERRUPT_ENABLED && Upstream_485_TX_ENABLED)
    extern volatile uint8 Upstream_485_txBuffer[Upstream_485_TX_BUFFER_SIZE];
    extern volatile uint8 Upstream_485_txBufferRead;
    extern uint8 Upstream_485_txBufferWrite;
#endif /* (Upstream_485_TX_INTERRUPT_ENABLED && Upstream_485_TX_ENABLED) */
#if (Upstream_485_RX_INTERRUPT_ENABLED && (Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED))
    extern uint8 Upstream_485_errorStatus;
    extern volatile uint8 Upstream_485_rxBuffer[Upstream_485_RX_BUFFER_SIZE];
    extern volatile uint8 Upstream_485_rxBufferRead;
    extern volatile uint8 Upstream_485_rxBufferWrite;
    extern volatile uint8 Upstream_485_rxBufferLoopDetect;
    extern volatile uint8 Upstream_485_rxBufferOverflow;
    #if (Upstream_485_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 Upstream_485_rxAddressMode;
        extern volatile uint8 Upstream_485_rxAddressDetected;
    #endif /* (Upstream_485_RXHW_ADDRESS_ENABLED) */
#endif /* (Upstream_485_RX_INTERRUPT_ENABLED && (Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define Upstream_485__B_UART__AM_SW_BYTE_BYTE 1
#define Upstream_485__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define Upstream_485__B_UART__AM_HW_BYTE_BY_BYTE 3
#define Upstream_485__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define Upstream_485__B_UART__AM_NONE 0

#define Upstream_485__B_UART__NONE_REVB 0
#define Upstream_485__B_UART__EVEN_REVB 1
#define Upstream_485__B_UART__ODD_REVB 2
#define Upstream_485__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define Upstream_485_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define Upstream_485_NUMBER_OF_STOP_BITS    (1u)

#if (Upstream_485_RXHW_ADDRESS_ENABLED)
    #define Upstream_485_RX_ADDRESS_MODE    (0u)
    #define Upstream_485_RX_HW_ADDRESS1     (0u)
    #define Upstream_485_RX_HW_ADDRESS2     (0u)
#endif /* (Upstream_485_RXHW_ADDRESS_ENABLED) */

#define Upstream_485_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << Upstream_485_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << Upstream_485_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << Upstream_485_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << Upstream_485_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << Upstream_485_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << Upstream_485_RX_STS_BREAK_SHIFT) \
                                        | (0 << Upstream_485_RX_STS_OVERRUN_SHIFT))

#define Upstream_485_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << Upstream_485_TX_STS_COMPLETE_SHIFT) \
                                        | (1 << Upstream_485_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << Upstream_485_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << Upstream_485_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef Upstream_485_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Upstream_485_CONTROL_REG \
                            (* (reg8 *) Upstream_485_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define Upstream_485_CONTROL_PTR \
                            (  (reg8 *) Upstream_485_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End Upstream_485_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Upstream_485_TX_ENABLED)
    #define Upstream_485_TXDATA_REG          (* (reg8 *) Upstream_485_BUART_sTX_TxShifter_u0__F0_REG)
    #define Upstream_485_TXDATA_PTR          (  (reg8 *) Upstream_485_BUART_sTX_TxShifter_u0__F0_REG)
    #define Upstream_485_TXDATA_AUX_CTL_REG  (* (reg8 *) Upstream_485_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Upstream_485_TXDATA_AUX_CTL_PTR  (  (reg8 *) Upstream_485_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Upstream_485_TXSTATUS_REG        (* (reg8 *) Upstream_485_BUART_sTX_TxSts__STATUS_REG)
    #define Upstream_485_TXSTATUS_PTR        (  (reg8 *) Upstream_485_BUART_sTX_TxSts__STATUS_REG)
    #define Upstream_485_TXSTATUS_MASK_REG   (* (reg8 *) Upstream_485_BUART_sTX_TxSts__MASK_REG)
    #define Upstream_485_TXSTATUS_MASK_PTR   (  (reg8 *) Upstream_485_BUART_sTX_TxSts__MASK_REG)
    #define Upstream_485_TXSTATUS_ACTL_REG   (* (reg8 *) Upstream_485_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define Upstream_485_TXSTATUS_ACTL_PTR   (  (reg8 *) Upstream_485_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(Upstream_485_TXCLKGEN_DP)
        #define Upstream_485_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) Upstream_485_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Upstream_485_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) Upstream_485_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Upstream_485_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) Upstream_485_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define Upstream_485_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) Upstream_485_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define Upstream_485_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) Upstream_485_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Upstream_485_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) Upstream_485_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Upstream_485_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Upstream_485_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Upstream_485_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Upstream_485_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Upstream_485_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) Upstream_485_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define Upstream_485_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) Upstream_485_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* Upstream_485_TXCLKGEN_DP */

#endif /* End Upstream_485_TX_ENABLED */

#if(Upstream_485_HD_ENABLED)

    #define Upstream_485_TXDATA_REG             (* (reg8 *) Upstream_485_BUART_sRX_RxShifter_u0__F1_REG )
    #define Upstream_485_TXDATA_PTR             (  (reg8 *) Upstream_485_BUART_sRX_RxShifter_u0__F1_REG )
    #define Upstream_485_TXDATA_AUX_CTL_REG     (* (reg8 *) Upstream_485_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define Upstream_485_TXDATA_AUX_CTL_PTR     (  (reg8 *) Upstream_485_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Upstream_485_TXSTATUS_REG           (* (reg8 *) Upstream_485_BUART_sRX_RxSts__STATUS_REG )
    #define Upstream_485_TXSTATUS_PTR           (  (reg8 *) Upstream_485_BUART_sRX_RxSts__STATUS_REG )
    #define Upstream_485_TXSTATUS_MASK_REG      (* (reg8 *) Upstream_485_BUART_sRX_RxSts__MASK_REG )
    #define Upstream_485_TXSTATUS_MASK_PTR      (  (reg8 *) Upstream_485_BUART_sRX_RxSts__MASK_REG )
    #define Upstream_485_TXSTATUS_ACTL_REG      (* (reg8 *) Upstream_485_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Upstream_485_TXSTATUS_ACTL_PTR      (  (reg8 *) Upstream_485_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End Upstream_485_HD_ENABLED */

#if( (Upstream_485_RX_ENABLED) || (Upstream_485_HD_ENABLED) )
    #define Upstream_485_RXDATA_REG             (* (reg8 *) Upstream_485_BUART_sRX_RxShifter_u0__F0_REG )
    #define Upstream_485_RXDATA_PTR             (  (reg8 *) Upstream_485_BUART_sRX_RxShifter_u0__F0_REG )
    #define Upstream_485_RXADDRESS1_REG         (* (reg8 *) Upstream_485_BUART_sRX_RxShifter_u0__D0_REG )
    #define Upstream_485_RXADDRESS1_PTR         (  (reg8 *) Upstream_485_BUART_sRX_RxShifter_u0__D0_REG )
    #define Upstream_485_RXADDRESS2_REG         (* (reg8 *) Upstream_485_BUART_sRX_RxShifter_u0__D1_REG )
    #define Upstream_485_RXADDRESS2_PTR         (  (reg8 *) Upstream_485_BUART_sRX_RxShifter_u0__D1_REG )
    #define Upstream_485_RXDATA_AUX_CTL_REG     (* (reg8 *) Upstream_485_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Upstream_485_RXBITCTR_PERIOD_REG    (* (reg8 *) Upstream_485_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Upstream_485_RXBITCTR_PERIOD_PTR    (  (reg8 *) Upstream_485_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Upstream_485_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Upstream_485_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Upstream_485_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Upstream_485_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Upstream_485_RXBITCTR_COUNTER_REG   (* (reg8 *) Upstream_485_BUART_sRX_RxBitCounter__COUNT_REG )
    #define Upstream_485_RXBITCTR_COUNTER_PTR   (  (reg8 *) Upstream_485_BUART_sRX_RxBitCounter__COUNT_REG )

    #define Upstream_485_RXSTATUS_REG           (* (reg8 *) Upstream_485_BUART_sRX_RxSts__STATUS_REG )
    #define Upstream_485_RXSTATUS_PTR           (  (reg8 *) Upstream_485_BUART_sRX_RxSts__STATUS_REG )
    #define Upstream_485_RXSTATUS_MASK_REG      (* (reg8 *) Upstream_485_BUART_sRX_RxSts__MASK_REG )
    #define Upstream_485_RXSTATUS_MASK_PTR      (  (reg8 *) Upstream_485_BUART_sRX_RxSts__MASK_REG )
    #define Upstream_485_RXSTATUS_ACTL_REG      (* (reg8 *) Upstream_485_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Upstream_485_RXSTATUS_ACTL_PTR      (  (reg8 *) Upstream_485_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (Upstream_485_RX_ENABLED) || (Upstream_485_HD_ENABLED) */

#if(Upstream_485_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define Upstream_485_INTCLOCK_CLKEN_REG     (* (reg8 *) Upstream_485_IntClock__PM_ACT_CFG)
    #define Upstream_485_INTCLOCK_CLKEN_PTR     (  (reg8 *) Upstream_485_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define Upstream_485_INTCLOCK_CLKEN_MASK    Upstream_485_IntClock__PM_ACT_MSK
#endif /* End Upstream_485_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(Upstream_485_TX_ENABLED)
    #define Upstream_485_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End Upstream_485_TX_ENABLED */

#if(Upstream_485_HD_ENABLED)
    #define Upstream_485_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End Upstream_485_HD_ENABLED */

#if( (Upstream_485_RX_ENABLED) || (Upstream_485_HD_ENABLED) )
    #define Upstream_485_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (Upstream_485_RX_ENABLED) || (Upstream_485_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define Upstream_485_WAIT_1_MS      Upstream_485_BL_CHK_DELAY_MS   

#define Upstream_485_TXBUFFERSIZE   Upstream_485_TX_BUFFER_SIZE
#define Upstream_485_RXBUFFERSIZE   Upstream_485_RX_BUFFER_SIZE

#if (Upstream_485_RXHW_ADDRESS_ENABLED)
    #define Upstream_485_RXADDRESSMODE  Upstream_485_RX_ADDRESS_MODE
    #define Upstream_485_RXHWADDRESS1   Upstream_485_RX_HW_ADDRESS1
    #define Upstream_485_RXHWADDRESS2   Upstream_485_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define Upstream_485_RXAddressMode  Upstream_485_RXADDRESSMODE
#endif /* (Upstream_485_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define Upstream_485_initvar                    Upstream_485_initVar

#define Upstream_485_RX_Enabled                 Upstream_485_RX_ENABLED
#define Upstream_485_TX_Enabled                 Upstream_485_TX_ENABLED
#define Upstream_485_HD_Enabled                 Upstream_485_HD_ENABLED
#define Upstream_485_RX_IntInterruptEnabled     Upstream_485_RX_INTERRUPT_ENABLED
#define Upstream_485_TX_IntInterruptEnabled     Upstream_485_TX_INTERRUPT_ENABLED
#define Upstream_485_InternalClockUsed          Upstream_485_INTERNAL_CLOCK_USED
#define Upstream_485_RXHW_Address_Enabled       Upstream_485_RXHW_ADDRESS_ENABLED
#define Upstream_485_OverSampleCount            Upstream_485_OVER_SAMPLE_COUNT
#define Upstream_485_ParityType                 Upstream_485_PARITY_TYPE

#if( Upstream_485_TX_ENABLED && (Upstream_485_TXBUFFERSIZE > Upstream_485_FIFO_LENGTH))
    #define Upstream_485_TXBUFFER               Upstream_485_txBuffer
    #define Upstream_485_TXBUFFERREAD           Upstream_485_txBufferRead
    #define Upstream_485_TXBUFFERWRITE          Upstream_485_txBufferWrite
#endif /* End Upstream_485_TX_ENABLED */
#if( ( Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED ) && \
     (Upstream_485_RXBUFFERSIZE > Upstream_485_FIFO_LENGTH) )
    #define Upstream_485_RXBUFFER               Upstream_485_rxBuffer
    #define Upstream_485_RXBUFFERREAD           Upstream_485_rxBufferRead
    #define Upstream_485_RXBUFFERWRITE          Upstream_485_rxBufferWrite
    #define Upstream_485_RXBUFFERLOOPDETECT     Upstream_485_rxBufferLoopDetect
    #define Upstream_485_RXBUFFER_OVERFLOW      Upstream_485_rxBufferOverflow
#endif /* End Upstream_485_RX_ENABLED */

#ifdef Upstream_485_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Upstream_485_CONTROL                Upstream_485_CONTROL_REG
#endif /* End Upstream_485_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Upstream_485_TX_ENABLED)
    #define Upstream_485_TXDATA                 Upstream_485_TXDATA_REG
    #define Upstream_485_TXSTATUS               Upstream_485_TXSTATUS_REG
    #define Upstream_485_TXSTATUS_MASK          Upstream_485_TXSTATUS_MASK_REG
    #define Upstream_485_TXSTATUS_ACTL          Upstream_485_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(Upstream_485_TXCLKGEN_DP)
        #define Upstream_485_TXBITCLKGEN_CTR        Upstream_485_TXBITCLKGEN_CTR_REG
        #define Upstream_485_TXBITCLKTX_COMPLETE    Upstream_485_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define Upstream_485_TXBITCTR_PERIOD        Upstream_485_TXBITCTR_PERIOD_REG
        #define Upstream_485_TXBITCTR_CONTROL       Upstream_485_TXBITCTR_CONTROL_REG
        #define Upstream_485_TXBITCTR_COUNTER       Upstream_485_TXBITCTR_COUNTER_REG
    #endif /* Upstream_485_TXCLKGEN_DP */
#endif /* End Upstream_485_TX_ENABLED */

#if(Upstream_485_HD_ENABLED)
    #define Upstream_485_TXDATA                 Upstream_485_TXDATA_REG
    #define Upstream_485_TXSTATUS               Upstream_485_TXSTATUS_REG
    #define Upstream_485_TXSTATUS_MASK          Upstream_485_TXSTATUS_MASK_REG
    #define Upstream_485_TXSTATUS_ACTL          Upstream_485_TXSTATUS_ACTL_REG
#endif /* End Upstream_485_HD_ENABLED */

#if( (Upstream_485_RX_ENABLED) || (Upstream_485_HD_ENABLED) )
    #define Upstream_485_RXDATA                 Upstream_485_RXDATA_REG
    #define Upstream_485_RXADDRESS1             Upstream_485_RXADDRESS1_REG
    #define Upstream_485_RXADDRESS2             Upstream_485_RXADDRESS2_REG
    #define Upstream_485_RXBITCTR_PERIOD        Upstream_485_RXBITCTR_PERIOD_REG
    #define Upstream_485_RXBITCTR_CONTROL       Upstream_485_RXBITCTR_CONTROL_REG
    #define Upstream_485_RXBITCTR_COUNTER       Upstream_485_RXBITCTR_COUNTER_REG
    #define Upstream_485_RXSTATUS               Upstream_485_RXSTATUS_REG
    #define Upstream_485_RXSTATUS_MASK          Upstream_485_RXSTATUS_MASK_REG
    #define Upstream_485_RXSTATUS_ACTL          Upstream_485_RXSTATUS_ACTL_REG
#endif /* End  (Upstream_485_RX_ENABLED) || (Upstream_485_HD_ENABLED) */

#if(Upstream_485_INTERNAL_CLOCK_USED)
    #define Upstream_485_INTCLOCK_CLKEN         Upstream_485_INTCLOCK_CLKEN_REG
#endif /* End Upstream_485_INTERNAL_CLOCK_USED */

#define Upstream_485_WAIT_FOR_COMLETE_REINIT    Upstream_485_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_Upstream_485_H */


/* [] END OF FILE */
