/*******************************************************************************
* File Name: Output_Pin_Red.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_Output_Pin_Red_H) /* Pins Output_Pin_Red_H */
#define CY_PINS_Output_Pin_Red_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Output_Pin_Red_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Output_Pin_Red__PORT == 15 && ((Output_Pin_Red__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    Output_Pin_Red_Write(uint8 value);
void    Output_Pin_Red_SetDriveMode(uint8 mode);
uint8   Output_Pin_Red_ReadDataReg(void);
uint8   Output_Pin_Red_Read(void);
void    Output_Pin_Red_SetInterruptMode(uint16 position, uint16 mode);
uint8   Output_Pin_Red_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the Output_Pin_Red_SetDriveMode() function.
     *  @{
     */
        #define Output_Pin_Red_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define Output_Pin_Red_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define Output_Pin_Red_DM_RES_UP          PIN_DM_RES_UP
        #define Output_Pin_Red_DM_RES_DWN         PIN_DM_RES_DWN
        #define Output_Pin_Red_DM_OD_LO           PIN_DM_OD_LO
        #define Output_Pin_Red_DM_OD_HI           PIN_DM_OD_HI
        #define Output_Pin_Red_DM_STRONG          PIN_DM_STRONG
        #define Output_Pin_Red_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define Output_Pin_Red_MASK               Output_Pin_Red__MASK
#define Output_Pin_Red_SHIFT              Output_Pin_Red__SHIFT
#define Output_Pin_Red_WIDTH              1u

/* Interrupt constants */
#if defined(Output_Pin_Red__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in Output_Pin_Red_SetInterruptMode() function.
     *  @{
     */
        #define Output_Pin_Red_INTR_NONE      (uint16)(0x0000u)
        #define Output_Pin_Red_INTR_RISING    (uint16)(0x0001u)
        #define Output_Pin_Red_INTR_FALLING   (uint16)(0x0002u)
        #define Output_Pin_Red_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define Output_Pin_Red_INTR_MASK      (0x01u) 
#endif /* (Output_Pin_Red__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Output_Pin_Red_PS                     (* (reg8 *) Output_Pin_Red__PS)
/* Data Register */
#define Output_Pin_Red_DR                     (* (reg8 *) Output_Pin_Red__DR)
/* Port Number */
#define Output_Pin_Red_PRT_NUM                (* (reg8 *) Output_Pin_Red__PRT) 
/* Connect to Analog Globals */                                                  
#define Output_Pin_Red_AG                     (* (reg8 *) Output_Pin_Red__AG)                       
/* Analog MUX bux enable */
#define Output_Pin_Red_AMUX                   (* (reg8 *) Output_Pin_Red__AMUX) 
/* Bidirectional Enable */                                                        
#define Output_Pin_Red_BIE                    (* (reg8 *) Output_Pin_Red__BIE)
/* Bit-mask for Aliased Register Access */
#define Output_Pin_Red_BIT_MASK               (* (reg8 *) Output_Pin_Red__BIT_MASK)
/* Bypass Enable */
#define Output_Pin_Red_BYP                    (* (reg8 *) Output_Pin_Red__BYP)
/* Port wide control signals */                                                   
#define Output_Pin_Red_CTL                    (* (reg8 *) Output_Pin_Red__CTL)
/* Drive Modes */
#define Output_Pin_Red_DM0                    (* (reg8 *) Output_Pin_Red__DM0) 
#define Output_Pin_Red_DM1                    (* (reg8 *) Output_Pin_Red__DM1)
#define Output_Pin_Red_DM2                    (* (reg8 *) Output_Pin_Red__DM2) 
/* Input Buffer Disable Override */
#define Output_Pin_Red_INP_DIS                (* (reg8 *) Output_Pin_Red__INP_DIS)
/* LCD Common or Segment Drive */
#define Output_Pin_Red_LCD_COM_SEG            (* (reg8 *) Output_Pin_Red__LCD_COM_SEG)
/* Enable Segment LCD */
#define Output_Pin_Red_LCD_EN                 (* (reg8 *) Output_Pin_Red__LCD_EN)
/* Slew Rate Control */
#define Output_Pin_Red_SLW                    (* (reg8 *) Output_Pin_Red__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Output_Pin_Red_PRTDSI__CAPS_SEL       (* (reg8 *) Output_Pin_Red__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Output_Pin_Red_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Output_Pin_Red__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Output_Pin_Red_PRTDSI__OE_SEL0        (* (reg8 *) Output_Pin_Red__PRTDSI__OE_SEL0) 
#define Output_Pin_Red_PRTDSI__OE_SEL1        (* (reg8 *) Output_Pin_Red__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Output_Pin_Red_PRTDSI__OUT_SEL0       (* (reg8 *) Output_Pin_Red__PRTDSI__OUT_SEL0) 
#define Output_Pin_Red_PRTDSI__OUT_SEL1       (* (reg8 *) Output_Pin_Red__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Output_Pin_Red_PRTDSI__SYNC_OUT       (* (reg8 *) Output_Pin_Red__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(Output_Pin_Red__SIO_CFG)
    #define Output_Pin_Red_SIO_HYST_EN        (* (reg8 *) Output_Pin_Red__SIO_HYST_EN)
    #define Output_Pin_Red_SIO_REG_HIFREQ     (* (reg8 *) Output_Pin_Red__SIO_REG_HIFREQ)
    #define Output_Pin_Red_SIO_CFG            (* (reg8 *) Output_Pin_Red__SIO_CFG)
    #define Output_Pin_Red_SIO_DIFF           (* (reg8 *) Output_Pin_Red__SIO_DIFF)
#endif /* (Output_Pin_Red__SIO_CFG) */

/* Interrupt Registers */
#if defined(Output_Pin_Red__INTSTAT)
    #define Output_Pin_Red_INTSTAT            (* (reg8 *) Output_Pin_Red__INTSTAT)
    #define Output_Pin_Red_SNAP               (* (reg8 *) Output_Pin_Red__SNAP)
    
	#define Output_Pin_Red_0_INTTYPE_REG 		(* (reg8 *) Output_Pin_Red__0__INTTYPE)
#endif /* (Output_Pin_Red__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Output_Pin_Red_H */


/* [] END OF FILE */
