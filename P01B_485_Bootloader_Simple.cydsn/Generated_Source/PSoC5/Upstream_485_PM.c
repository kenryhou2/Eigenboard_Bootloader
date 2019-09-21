/*******************************************************************************
* File Name: Upstream_485_PM.c
* Version 2.50
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
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


/***************************************
* Local data allocation
***************************************/

static Upstream_485_BACKUP_STRUCT  Upstream_485_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: Upstream_485_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the Upstream_485_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Upstream_485_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Upstream_485_SaveConfig(void)
{
    #if(Upstream_485_CONTROL_REG_REMOVED == 0u)
        Upstream_485_backup.cr = Upstream_485_CONTROL_REG;
    #endif /* End Upstream_485_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Upstream_485_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the nonretention control register except FIFO.
*  Does not restore the FIFO which is a set of nonretention registers.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Upstream_485_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling Upstream_485_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void Upstream_485_RestoreConfig(void)
{
    #if(Upstream_485_CONTROL_REG_REMOVED == 0u)
        Upstream_485_CONTROL_REG = Upstream_485_backup.cr;
    #endif /* End Upstream_485_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Upstream_485_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The Upstream_485_Sleep() API saves the current component state. Then it
*  calls the Upstream_485_Stop() function and calls 
*  Upstream_485_SaveConfig() to save the hardware configuration.
*  Call the Upstream_485_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Upstream_485_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Upstream_485_Sleep(void)
{
    #if(Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED)
        if((Upstream_485_RXSTATUS_ACTL_REG  & Upstream_485_INT_ENABLE) != 0u)
        {
            Upstream_485_backup.enableState = 1u;
        }
        else
        {
            Upstream_485_backup.enableState = 0u;
        }
    #else
        if((Upstream_485_TXSTATUS_ACTL_REG  & Upstream_485_INT_ENABLE) !=0u)
        {
            Upstream_485_backup.enableState = 1u;
        }
        else
        {
            Upstream_485_backup.enableState = 0u;
        }
    #endif /* End Upstream_485_RX_ENABLED || Upstream_485_HD_ENABLED*/

    Upstream_485_Stop();
    Upstream_485_SaveConfig();
}


/*******************************************************************************
* Function Name: Upstream_485_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  Upstream_485_Sleep() was called. The Upstream_485_Wakeup() function
*  calls the Upstream_485_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  Upstream_485_Sleep() function was called, the Upstream_485_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Upstream_485_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Upstream_485_Wakeup(void)
{
    Upstream_485_RestoreConfig();
    #if( (Upstream_485_RX_ENABLED) || (Upstream_485_HD_ENABLED) )
        Upstream_485_ClearRxBuffer();
    #endif /* End (Upstream_485_RX_ENABLED) || (Upstream_485_HD_ENABLED) */
    #if(Upstream_485_TX_ENABLED || Upstream_485_HD_ENABLED)
        Upstream_485_ClearTxBuffer();
    #endif /* End Upstream_485_TX_ENABLED || Upstream_485_HD_ENABLED */

    if(Upstream_485_backup.enableState != 0u)
    {
        Upstream_485_Enable();
    }
}


/* [] END OF FILE */
