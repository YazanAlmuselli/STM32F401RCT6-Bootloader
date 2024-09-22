
#ifndef BOOTLOADR_H
#define BOOTLOADR_H

/* ----------------- Includes -----------------*/
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "usart.h"
#include "crc.h"

/* ----------------- Macro Declarations -----------------*/
#define BL_DEBUG_UART								&huart2
#define BL_Host_Communication_UART	&huart2
#define CRC_ENGINE_OBJ							&hcrc

#define DEBUG_INFO_DISABLE					0x00
#define DEBUG_INFO_ENABLE						0x01
#define BL_DEBUG_ENABLE							DEBUG_INFO_ENABLE

#define BL_UART_DEBUG_MESSAGE_ENABLE 0x00
#define BL_SPI_DEBUG_MESSAGE_ENABLE  0x01
#define BL_CAN_DEBUG_MESSAGE_ENABLE  0x02

#define BL_DEBUGE_MESSAGE_METHOD	(BL_UART_DEBUG_MESSAGE_ENABLE)

#define BL_Host_Buffer_RX_Length	200

#define CBL_GET_VER_CMD              0x10									//Read BL Version Number from MCU
#define CBL_GET_HELP_CMD             0x11 								//All Supported Commands Codes
#define CBL_GET_CID_CMD              0x12 								//Read the MCU Chip Identification Number
/* Get Read Protection Status */
#define CBL_GET_RDP_STATUS_CMD       0x13 								//Read the Flash Read Protection Level
#define CBL_GO_TO_ADDR_CMD           0x14 								//Jump BL to Specefied Address
#define CBL_FLASH_ERASE_CMD          0x15 								//Mass Erase of Sector Erase of User Flash
#define CBL_MEM_WRITE_CMD            0x16 								//Write Data into Different Memories in MCU
/* Enable/Disable Write Protection */
#define CBL_EN_W_PROTECT_CMD         0x17 								//Enable W/R Protect on Different Sectors
#define CBL_MEM_READ_CMD             0x18 								//Read Data From Different Memories in MCU
/* Get Sector Read/Write Protection Status */
#define CBL_READ_SECTOR_STATUS_CMD   0x19 								//Read all Sectors Protection
#define CBL_OTP_READ_CMD             0x20                 //Read OTP Memory
/* Change Read Out Protection Level */
#define CBL_CHANGE_ROP_Level_CMD     0x21									//Change Read Out Protection Level

#define CBL_VENDOR_ID								 100
#define CBL_SW_MAJOR_VERSION				 1
#define CBL_SW_MINOR_VERSION				 1
#define CBL_SW_PATCH_VERSION				 0

#define CRC_TYPE_SIZE_BYTE					 4

#define CRC_VERIFICATION_FAILED			 0x00
#define CRC_VERIFICATION_PASSED			 0x01

#define CBL_SEND_NACK								 0xAB
#define CBL_SEND_ACK								 0xCD

#define FLSAH_SECTOR2_BASE_ADDRESS	 0x08008000U

#define INVALID_ADDRESS							 0x00
#define VALID_ADDRESS								 0x01

#define STM32F401RCT6_FLASH_SIZE		 (256 * 1024)
#define STM32F401RCT6_SRAM1_SIZE	   (64  * 1024)
#define STM32F401RCT6_FLASH_END			 (FLASH_BASE + STM32F401RCT6_FLASH_SIZE)
#define STM32F401RCT6_SRAM1_END			 (SRAM1_BASE + STM32F401RCT6_SRAM1_SIZE)

/* CBL_FLASH_ERASE_CMD */
#define FLASH_MAX_SECTORS_NUMBER		 6
#define INVALID_SECTOR_NUMBER				 0x00
#define VALID_SECTOR_NUMBER				 	 0x01
#define UNSUCCESSFUL_ERASE					 0x02
#define SUCCESSFUL_ERASE					   0x03
#define FLASH_MASS_ERASE						 0xFF
#define HAL_SUCCESSFUL_ERASE				 0xFFFFFFFFU

/* CBL_MEM_WRITE_CMD */
#define FALSH_PAYLOAD_WRITE_FAILED 	 0x00
#define FALSH_PAYLOAD_WRITE_PASSED 	 0x01
#define FALSH_LOCK_WRITE_FAILED 	 	 0x00
#define FALSH_LOCK_WRITE_PASSED 	 	 0x01

/* CBL_GET_RDP_STATUS_CMD */
#define ROP_LEVEL_READ_INVALID			 0x00
#define ROP_LEVEL_READ_VALID		  	 0x01

/* Change Read Out Protection Level */
#define ROP_LEVEL_CHANGE_INVALID		 0x00
#define ROP_LEVEL_CHANGE_VALID  		 0x01

/* ----------------- Macro Functions Declarations -----------------*/


/* ----------------- Data Type Declarations -----------------*/
typedef enum{
	BL_NOK = 0,
	BL_OK
}BL_Status;

typedef void (*pMain_App)(void);
typedef void (*Jump_Ptr)(void);

/* ----------------- Software Interfaces Declarations -----------------*/
void BL_Print_Message(char *Format, ...);
BL_Status BL_UART_Fetch_Host_Command(void);

#endif /*BOOTLOADR.H*/