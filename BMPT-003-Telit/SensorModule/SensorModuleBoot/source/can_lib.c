
//******************************************************************************
//! @file file: can_lib.c
//!
//! @brief This file contains the library of functions of:
//!             - CAN (Controller Array Network)
//!             - AT90CAN128/64/32
//!
//******************************************************************************

//_____ I N C L U D E S ________________________________________________________
#include "config.h"
#include "can_lib.h"
#include "can_drv.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//  @fn can_init
//!
//! CAN macro initialization. Reset the CAN peripheral, initialize the bit
//! timing, initialize all the registers mapped in SRAM to put MObs in
//! inactive state and enable the CAN macro.
//!
//! @warning The CAN macro will be enable after seen on CAN bus a receceive
//!          level as long as of an inter frame (hardware feature).
//!
//! @param  Mode (for "can_fixed_baudrate" param not used)
//!         ==0: start CAN bit timing evaluation from faster baudrate
//!         ==1: start CAN bit timing evaluation with CANBTx registers
//!              contents
//!
//! @return Baudrate Status
//!         ==0: research of bit timing configuration failed
//!         ==1: baudrate performed 
//!
//------------------------------------------------------------------------------
U8 can_init(U16 mode)
{
    if ((Can_bit_timing(mode))==0) return (0);  // c.f. macro in "can_drv.h"
    can_clear_all_mob();                        // c.f. function in "can_drv.c"
    Can_enable();                               // c.f. macro in "can_drv.h" 
    return (1);
}

//------------------------------------------------------------------------------
//  @fn can_cmd
//!
//! This function takes a CAN descriptor, analyses the action to do:
//! transmit, receive or abort.
//! This function returns a status (CAN_CMD_ACCEPTED or CAN_CMD_REFUSED) if
//! a MOb for Rx or Tx has been found. If no MOB has been found, the
//! application must be retry at a later date.
//! This function also updates the CAN descriptor status (MOB_PENDING or
//! MOB_NOT_REACHED) if a MOb for Rx or Tx has been found. If aborting
//! is performed, the CAN descriptor status will be set to STATUS_CLEARED.
//!
//! @param  st_cmd_t* - Can_descriptor pointer on CAN descriptor structure
//!         to select the action to do.
//!
//! @return CAN_CMD_ACCEPTED - command is accepted
//!         CAN_CMD_REFUSED  - command is refused
//!
//------------------------------------------------------------------------------
U8 can_cmd(st_cmd_t* cmd)
{
  U8 mob_handle, cpt;
  U32 u32_temp;
  
/*  if (cmd->cmd == CMD_ABORT)
  {
    if (cmd->status == MOB_PENDING)
    {
      // Rx or Tx not yet performed
      Can_set_mob(cmd->handle);
      Can_mob_abort();
      Can_clear_status_mob();       // To be sure !
      cmd->handle = 0;
    }
    cmd->status = STATUS_CLEARED; 
  }
  else
  {*/
    mob_handle = can_get_mob_free();
    if (mob_handle!= NO_MOB)
    {
      cmd->status = MOB_PENDING; 
      cmd->handle = mob_handle;
      Can_set_mob(mob_handle);
      Can_clear_mob();
          
      if(cmd->cmd == CMD_TX_DATA)
      {
        if (cmd->ctrl.ide){ Can_set_ext_id(cmd->id.ext);}
        else              { Can_set_std_id(cmd->id.std);}
        for (cpt=0;cpt<cmd->dlc;cpt++) CANMSG = *(cmd->pt_data + cpt);
        cmd->ctrl.rtr=0; Can_clear_rtr();
        Can_set_dlc(cmd->dlc);
        Can_config_tx();
      }
      else
      {
          // case CMD_NONE or not implemented command
          cmd->status = STATUS_CLEARED;     
      }
      
      /*
      switch (cmd->cmd)
      {
        //------------      

        //------------      
        case CMD_TX_DATA:    
          if (cmd->ctrl.ide){ Can_set_ext_id(cmd->id.ext);}
          else              { Can_set_std_id(cmd->id.std);}
          for (cpt=0;cpt<cmd->dlc;cpt++) CANMSG = *(cmd->pt_data + cpt);
          cmd->ctrl.rtr=0; Can_clear_rtr();
          Can_set_dlc(cmd->dlc);
          Can_config_tx();
          break;
        //------------      

        //------------      
        default:
          // case CMD_NONE or not implemented command
          cmd->status = STATUS_CLEARED; 
          break;
        //------------      
      } // switch (cmd ...*/
    } // if (mob_handle ...
    else
    {
      cmd->status = MOB_NOT_REACHED;
      return CAN_CMD_REFUSED;
    }
  //} // else of no CMD_ABORT
  return CAN_CMD_ACCEPTED;
}

//------------------------------------------------------------------------------
//  @fn can_get_status
//!
//! This function allows to return if the command has been performed or not.
//! In an reception case, all the CAN message is stored in the structure.
//! This function also updates the CAN descriptor status (MOB_TX_COMPLETED,    
//!  MOB_RX_COMPLETED, MOB_RX_COMPLETED_DLCW or MOB_???_ERROR).         
//!
//! @param  st_cmd_t* pointer on CAN descriptor structure.
//!
//! @return CAN_STATUS_COMPLETED     - Rx or Tx is completed
//!         CAN_STATUS_NOT_COMPLETED - Rx or Tx is not completed
//!         CAN_STATUS_ERROR         - Error in configuration or in the
//!                                    CAN communication
//!
//------------------------------------------------------------------------------
U8 can_get_status (st_cmd_t* cmd)
{
    U8 a_status, rtn_val;
     
    a_status = cmd->status;
    if ((a_status==STATUS_CLEARED)||(a_status==MOB_NOT_REACHED)||(a_status==MOB_DISABLE))
    {
        return CAN_STATUS_ERROR;
    }

    Can_set_mob(cmd->handle);
    a_status = can_get_mob_status();
    
    switch (a_status)
    {
        case MOB_NOT_COMPLETED:
            // cmd->status not updated
            rtn_val = CAN_STATUS_NOT_COMPLETED;
            break;
        //---------------      
        case MOB_RX_COMPLETED:     
        case MOB_RX_COMPLETED_DLCW:
            cmd->dlc = Can_get_dlc();
            can_get_data(cmd->pt_data);
            cmd->ctrl.rtr = Can_get_rtr();
            if (Can_get_ide()) // if extended frame
            {
                cmd->ctrl.ide = 1; // extended frame
                Can_get_ext_id(cmd->id.ext);
            }
            else // else standard frame
                {
                    cmd->ctrl.ide = 0;
                    Can_get_std_id(cmd->id.std);
                }
            // Status field of descriptor: 0x20 if Rx completed
            // Status field of descriptor: 0xA0 if Rx completed with DLCWarning    
            cmd->status = a_status;
            Can_mob_abort();        // Freed the MOB
            Can_clear_status_mob(); //   and reset MOb status
            rtn_val = CAN_STATUS_COMPLETED;
            break;
        //---------------      
        case MOB_TX_COMPLETED:     
            // Status field of descriptor: 0x40 if Tx completed
            cmd->status = a_status;
            Can_mob_abort();        // Freed the MOB
            Can_clear_status_mob(); //   and reset MOb status
            rtn_val = CAN_STATUS_COMPLETED;
            break;
        //---------------      
        default:
            // Status field of descriptor: (bin)000b.scfa if MOb error
            cmd->status = a_status;
            Can_mob_abort();        // Freed the MOB
            Can_clear_status_mob(); //   and reset MOb status
            rtn_val = CAN_STATUS_ERROR;
            break;
             
    } // switch (a_status...
 
    return (rtn_val);
}
