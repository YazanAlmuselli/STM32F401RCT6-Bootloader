#include "bootloader.h"

/* ----------------- Includes -----------------*/

/* ----------------- Macro Definition -----------------*/

/* ----------------- Static Functions Definition -----------------*/
static void BootLoader_Get_Version(uint8_t *Host_Buffer);
static void BootLoader_Get_Help(uint8_t *Host_Buffer);
static void BootLoader_Get_Chip(uint8_t *Host_Buffer);
static void BootLoader_Read_Protection_Level(uint8_t *Host_Buffer);
static void BootLoader_Jump_to_Address(uint8_t *Host_Buffer);
static void BootLoader_Erase_Flash(uint8_t *Host_Buffer);
static void Bootloader_Memory_Write(uint8_t *Host_Buffer);
static void Bootloader_Enable_RW_Protection(uint8_t *Host_Buffer);
static void BootLoader_Memory_Read(uint8_t *Host_Buffer);
static void BootLoader_Get_Sector_protection_Status(uint8_t *Host_Buffer);
static void BootLoader_Read_OTP(uint8_t *Host_Buffer);
static void Bootloader_Change_Read_Protection_Level(uint8_t *Host_Buffer);

static uint8_t Bootloader_CRC_Verify(uint8_t *pData, uint32_t Data_Len, uint32_t Host_CRC);
static void Bootloader_Send_ACK(uint8_t Reply_Len);
static void Bootloader_Send_NACK(void);
static void Bootloader_Send_Data_to_Host(uint8_t *Host_Buffer, uint32_t Data_Len);
static void BL_Jump_to_User_Application(void);

/* ----------------- Data Type Definition -----------------*/
uint8_t Debug_Var_Mem_Write = 0;
uint8_t Debug_Var_Address_Verification = 0;
uint8_t Debug_Var_Payload_Write = 0;

static uint8_t BL_Host_Buffer[BL_Host_Buffer_RX_Length];

static uint8_t Bootloader_Supported_CMDs[12] = {
    CBL_GET_VER_CMD,
    CBL_GET_HELP_CMD,
    CBL_GET_CID_CMD,
    CBL_GET_RDP_STATUS_CMD,
    CBL_GO_TO_ADDR_CMD,
    CBL_FLASH_ERASE_CMD,
    CBL_MEM_WRITE_CMD,
    CBL_EN_W_PROTECT_CMD,
    CBL_MEM_READ_CMD,
    CBL_READ_SECTOR_STATUS_CMD,
    CBL_OTP_READ_CMD,
    CBL_CHANGE_ROP_Level_CMD
};
/* ----------------- Static API Definition -----------------*/
/* Just to Check the Functionality of "Go to Adress" Command, to Use it Change the Optimization Level to 0 */ 
static void Print_Yazan(void){ /* Address -> 0x080031c8 */
	BL_Print_Message("Yazan Abd-Al-Majeed \n\r");
}

static void BL_Jump_to_User_Application(void){
	/* Save the Main Stack Pointer Value of Main Application */
	uint32_t MSP_Value = *((volatile uint32_t *)FLSAH_SECTOR2_BASE_ADDRESS); /* Beginning of VT */
	
	/* Reset Handler Definition of Our Main Application */
	uint32_t Main_App_Address = *((volatile uint32_t *)(FLSAH_SECTOR2_BASE_ADDRESS + 4));
	
	/* Fetch the Reset Handler Address of User Application */
	pMain_App ResetHandler_Address = (pMain_App)Main_App_Address;
	
	/* Set Main Stack Pointer */
	__set_MSP(MSP_Value);
	
	/* Deinitializaing RCC */
	HAL_RCC_DeInit();
	
	/* Jump to Application Reset Handler */
	ResetHandler_Address();
}

BL_Status BL_UART_Fetch_Host_Command(void){
	BL_Status Status  = BL_NOK;
	HAL_StatusTypeDef HAL_Status = HAL_ERROR;
	uint8_t Data_Length = 0;
	
	/* Set the Whole Buffer with Zero to Recive New Commands */
	memset(BL_Host_Buffer, 0, BL_Host_Buffer_RX_Length);
	/* Recive from UART 1 Byte which Indicates the Length of the Packet, Packet Include Command Name, Command Info, and CRC */
	HAL_Status = HAL_UART_Receive(BL_Host_Communication_UART, BL_Host_Buffer, 1, HAL_MAX_DELAY);
	if(HAL_Status != HAL_OK){
		Status = BL_NOK;
	}
	else{
		Data_Length = BL_Host_Buffer[0];
		HAL_Status = HAL_UART_Receive(BL_Host_Communication_UART, &BL_Host_Buffer[1], Data_Length, HAL_MAX_DELAY);
			if(HAL_Status != HAL_OK){
			Status = BL_NOK;
			}
			else{
				switch(BL_Host_Buffer[1]){
					case CBL_GET_VER_CMD:
						BootLoader_Get_Version(BL_Host_Buffer);
						Status = BL_OK;
						break;
					case CBL_GET_HELP_CMD:
						BootLoader_Get_Help(BL_Host_Buffer);
						Status = BL_OK;
						break;
					case CBL_GET_CID_CMD:
						BootLoader_Get_Chip(BL_Host_Buffer);
						Status = BL_OK;
						break;
					case CBL_GET_RDP_STATUS_CMD:
						BootLoader_Read_Protection_Level(BL_Host_Buffer);
						Status = BL_OK;
						break;
					case CBL_GO_TO_ADDR_CMD:
						BootLoader_Jump_to_Address(BL_Host_Buffer);
						Status = BL_OK;
						break;
					case CBL_FLASH_ERASE_CMD:
						BootLoader_Erase_Flash(BL_Host_Buffer);
						Status = BL_OK;
						break;
					case CBL_MEM_WRITE_CMD:
						Bootloader_Memory_Write(BL_Host_Buffer);
						Status = BL_OK;
						break;
					case CBL_EN_W_PROTECT_CMD:
						Bootloader_Enable_RW_Protection(BL_Host_Buffer);
						Status = BL_OK;
						break;
					case CBL_MEM_READ_CMD:
						BootLoader_Memory_Read(BL_Host_Buffer);
						Status = BL_OK;
						break;
					case CBL_READ_SECTOR_STATUS_CMD:
						BootLoader_Get_Sector_protection_Status(BL_Host_Buffer);
						Status = BL_OK;
						break;
					case CBL_OTP_READ_CMD:
						BootLoader_Read_OTP(BL_Host_Buffer);
						Status = BL_OK;
						break;
					case CBL_CHANGE_ROP_Level_CMD:
						Bootloader_Change_Read_Protection_Level(BL_Host_Buffer);
						Status = BL_OK;
						break;
					default:
						BL_Print_Message("Invalid Command Recived From Host !! \n\r");
						break;
				}
			}
		}
		
		return Status;
}

static uint8_t Bootloader_CRC_Verify(uint8_t *pData, uint32_t Data_Len, uint32_t Host_CRC){
	uint8_t CRC_Status = CRC_VERIFICATION_FAILED;
	uint32_t MCU_CRC_Calculated = 0;
	uint32_t Data_Buffer = 0;
	uint8_t Data_Counter = 0;
	
	/* Calculate the CRC32 */
	for(Data_Counter = 0; Data_Counter < Data_Len; Data_Counter++){
		Data_Buffer = (uint32_t)pData[Data_Counter];
		MCU_CRC_Calculated = HAL_CRC_Accumulate(CRC_ENGINE_OBJ, &Data_Buffer, 1);
	}
	
	/* Reset the CRC32  Calculation Unit*/
	__HAL_CRC_DR_RESET(CRC_ENGINE_OBJ);
	
	/* Compare the Host CRC with the Calculated CRC */
	if(MCU_CRC_Calculated == Host_CRC){
		CRC_Status = CRC_VERIFICATION_PASSED;
	}
	else{
		CRC_Status = CRC_VERIFICATION_FAILED;
	}
	
	return CRC_Status;
}

static void Bootloader_Send_ACK(uint8_t Reply_Len){
	uint8_t ACK_Value[2];
	
	ACK_Value[0] = CBL_SEND_ACK;
	ACK_Value[1] = Reply_Len;
	
	HAL_UART_Transmit(BL_Host_Communication_UART, (uint8_t *)ACK_Value, 2, HAL_MAX_DELAY);
}

static void Bootloader_Send_NACK(void){
	
	uint8_t ACK_Value = CBL_SEND_NACK;
	HAL_UART_Transmit(BL_Host_Communication_UART, &ACK_Value, 1, HAL_MAX_DELAY);
}

static void Bootloader_Send_Data_to_Host(uint8_t *Host_Buffer, uint32_t Data_Len){
	HAL_UART_Transmit(BL_Host_Communication_UART, Host_Buffer, Data_Len, HAL_MAX_DELAY);
}

static void BootLoader_Get_Version(uint8_t *Host_Buffer){
	uint8_t BL_Version[4] = {CBL_VENDOR_ID, CBL_SW_MAJOR_VERSION, CBL_SW_MINOR_VERSION, CBL_SW_PATCH_VERSION};
	uint16_t Host_Command_Packet_Length = 0;
	uint32_t Host_CRC32 = 0;
	
	/* Extract the Host Command Packet Length and CRC32 Sent by User */
	Host_Command_Packet_Length = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t *)((Host_Buffer + Host_Command_Packet_Length) - CRC_TYPE_SIZE_BYTE));

	if(CRC_VERIFICATION_PASSED == Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0], Host_Command_Packet_Length - 4, Host_CRC32)){
		Bootloader_Send_ACK(4);
		Bootloader_Send_Data_to_Host((uint8_t *)BL_Version, 4);
	}
	else{
		Bootloader_Send_NACK();
	}
}

static void BootLoader_Get_Help(uint8_t *Host_Buffer){
	uint16_t Host_Command_Packet_Length = 0;
	uint32_t Host_CRC32 = 0;
	
	/* Extract the Host Command Packet Length and CRC32 Sent by User */
	Host_Command_Packet_Length = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t *)((Host_Buffer + Host_Command_Packet_Length) - CRC_TYPE_SIZE_BYTE));
	
	if(CRC_VERIFICATION_PASSED == Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0], Host_Command_Packet_Length - 4, Host_CRC32)){
		Bootloader_Send_ACK(12);
		Bootloader_Send_Data_to_Host((uint8_t *)(&Bootloader_Supported_CMDs[0]), 12);
	}
	else{
		Bootloader_Send_NACK();
	}
}

static void BootLoader_Get_Chip(uint8_t *Host_Buffer){
	uint16_t Host_Command_Packet_Length = 0;
	uint32_t Host_CRC32 = 0;
	uint16_t MCU_ID_Number = 0;
	
	/* Extract the Host Command Packet Length and CRC32 Sent by User */
	Host_Command_Packet_Length = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t *)((Host_Buffer + Host_Command_Packet_Length) - CRC_TYPE_SIZE_BYTE));
	
	/* CRC Verification */
	if(CRC_VERIFICATION_PASSED == Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0], Host_Command_Packet_Length - 4, Host_CRC32)){
		/* Get the MCU Identification Number */
		MCU_ID_Number = (uint16_t)(DBGMCU->IDCODE & 0x00000FFF);
		/* Report the MCU Identification Number */
		Bootloader_Send_ACK(2);
		Bootloader_Send_Data_to_Host((uint8_t *)&MCU_ID_Number, 2);
	}
	else{
		Bootloader_Send_NACK();
	}
}

static uint8_t Host_Jump_Address_Verification(uint32_t Jump_Address){
	uint8_t Address_Verification = INVALID_ADDRESS;
	
	if((Jump_Address <= STM32F401RCT6_FLASH_END) && (Jump_Address >= FLASH_BASE)){
		Debug_Var_Address_Verification = 1;
		Address_Verification = VALID_ADDRESS;
	}
	else if((Jump_Address <= STM32F401RCT6_SRAM1_END) && (Jump_Address >= SRAM1_BASE)){
		Debug_Var_Address_Verification = 2;
		Address_Verification = VALID_ADDRESS;
	}
	else{
		Debug_Var_Address_Verification = 3;
		Address_Verification = INVALID_ADDRESS;
	}
	
	return Address_Verification;
}

static void BootLoader_Jump_to_Address(uint8_t *Host_Buffer){
	uint16_t Host_Command_Packet_Length = 0;
	uint32_t Host_CRC32 = 0;
	uint32_t Host_Jump_Address = 0;
	uint8_t Address_Verification = INVALID_ADDRESS;
	
	/* Extract the Host Command Packet Length and CRC32 Sent by User */
	Host_Command_Packet_Length = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t *)((Host_Buffer + Host_Command_Packet_Length) - CRC_TYPE_SIZE_BYTE));
	
	/* CRC Verification */
	if(CRC_VERIFICATION_PASSED == Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0], Host_Command_Packet_Length - 4, Host_CRC32)){
		Bootloader_Send_ACK(1);
		/* Extract the Address From the Host Command Packet  */
		Host_Jump_Address = *((uint32_t *)&Host_Buffer[2]);
		/* Check for Address Verification */
		Address_Verification = Host_Jump_Address_Verification(Host_Jump_Address);
		
		if(VALID_ADDRESS == Address_Verification){
			/* Report Address Verification Succeeded */
			Bootloader_Send_Data_to_Host((uint8_t *)&Address_Verification, 1);
			/* Prepare the Address to jump */
			Jump_Ptr Jump_Address = (Jump_Ptr)(Host_Jump_Address + 1);
			Jump_Address();
		}
		else{
			Bootloader_Send_Data_to_Host((uint8_t *)&Address_Verification, 1);
		}
	}
	else{
		Bootloader_Send_NACK();
	}
}

static uint8_t BL_STM32F401RCT6_Get_Read_Protection_Level(void){
	FLASH_OBProgramInitTypeDef FLASH_OBProgram;
	
	HAL_FLASHEx_OBGetConfig(&FLASH_OBProgram);
	
	return (uint8_t)(FLASH_OBProgram.RDPLevel);
}

static void BootLoader_Read_Protection_Level(uint8_t *Host_Buffer){
	uint16_t Host_Command_Packet_Length = 0;
	uint32_t Host_CRC32  = 0;
	uint8_t RDP_Level = 0;
	uint8_t RDP_Level_Error_Status = ROP_LEVEL_READ_INVALID;
	
	/* Extract the Host Command Packet Length and CRC32 Sent by User */
	Host_Command_Packet_Length = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t *)((Host_Buffer + Host_Command_Packet_Length) - CRC_TYPE_SIZE_BYTE));
	
	/* CRC Verification */
	if(CRC_VERIFICATION_PASSED == Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0], Host_Command_Packet_Length - 4, Host_CRC32)){
		Bootloader_Send_ACK(1);
		/* Read_Protection_Level */
		RDP_Level = BL_STM32F401RCT6_Get_Read_Protection_Level();
		Bootloader_Send_Data_to_Host((uint8_t *)&RDP_Level, 1);
	}
	else{
		Bootloader_Send_NACK();
	}
}

static uint8_t Perform_Flash_Erase(uint8_t Sector_Number, uint8_t Numbers_of_Sectors){
	uint8_t Sector_Validity_Status = INVALID_SECTOR_NUMBER;
	FLASH_EraseInitTypeDef pEraseInit;
	uint8_t Remaining_Sectors = 0;
	HAL_StatusTypeDef HAL_Status = HAL_ERROR;
	uint32_t SectorError = 0;
	
	if((Numbers_of_Sectors > FLASH_MAX_SECTORS_NUMBER) || (Numbers_of_Sectors < 0)){
		/* Numbers_of_Sectors is out of Range */
		Sector_Validity_Status = INVALID_SECTOR_NUMBER;
	}
	else{
		if(Sector_Number < (FLASH_MAX_SECTORS_NUMBER - 1) || (FLASH_MASS_ERASE == Sector_Number)){
			if(FLASH_MASS_ERASE == Sector_Number){
				/* Erase the Whole Flash */
				pEraseInit.TypeErase = FLASH_TYPEERASE_MASSERASE;		/*!< Flash Mass erase activation */
			}
			else{
				/* Erase Only Determined Sectors */
				Remaining_Sectors = FLASH_MAX_SECTORS_NUMBER - Sector_Number;
				if(Numbers_of_Sectors > Remaining_Sectors){
					Numbers_of_Sectors = Remaining_Sectors;
				}
				else{ /* Nothing */ }
				pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;	  /*!< Sectors erase only */
				pEraseInit.Sector = Sector_Number;	/*!< Initial FLASH sector to erase when Mass erase is disabled */
				pEraseInit.NbSectors = Numbers_of_Sectors;
			}
			
		pEraseInit.Banks = FLASH_BANK_1; 											/*!< Bank 1   */
		pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;			/*!< Device operating range: 2.7V to 3.6V */
			
		/* Unlock the FLASH control register access */
		HAL_Status = HAL_FLASH_Unlock();
		HAL_Status = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
			if(HAL_SUCCESSFUL_ERASE == SectorError){
				Sector_Validity_Status = SUCCESSFUL_ERASE;
			}
			else{
				Sector_Validity_Status = UNSUCCESSFUL_ERASE;
			}
		/* Lock the FLASH control register access */
		HAL_Status = HAL_FLASH_Lock();
		}
		else{
			Sector_Validity_Status = UNSUCCESSFUL_ERASE;
		}
	}
	
	return Sector_Validity_Status;
}

static void BootLoader_Erase_Flash(uint8_t *Host_Buffer){
	uint16_t Host_Command_Packet_Length = 0;
	uint32_t Host_CRC32  = 0;
	uint8_t Erase_Status = 0;
	
	/* Extract the Host Command Packet Length and CRC32 Sent by User */
	Host_Command_Packet_Length = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t *)((Host_Buffer + Host_Command_Packet_Length) - CRC_TYPE_SIZE_BYTE));
	
	/* CRC Verification */
	if(CRC_VERIFICATION_PASSED == Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0], Host_Command_Packet_Length - 4, Host_CRC32)){
		Bootloader_Send_ACK(1);
		Erase_Status = Perform_Flash_Erase(Host_Buffer[2], Host_Buffer[3]);
		if(SUCCESSFUL_ERASE == Erase_Status){
			/* Report Erase Operation Successful */
			Bootloader_Send_Data_to_Host((uint8_t *)&Erase_Status, 1);
		}
		else{
			/* Report Erase Operation Failure */
			Bootloader_Send_Data_to_Host((uint8_t *)&Erase_Status, 1);
		}
	}
	else{
		Bootloader_Send_NACK();
	}
}

static uint8_t Flash_Memory_Write_Payload(uint8_t *Host_Payload, uint32_t Payload_Start_Address, uint16_t Payload_Len){
	HAL_StatusTypeDef HAL_Status = HAL_ERROR;
	uint8_t Flash_Payload_Write_Status = FALSH_PAYLOAD_WRITE_FAILED;
	uint16_t Payload_Counter = 0;
	
	/* Unlock the FLASH control register access */
  HAL_Status = HAL_FLASH_Unlock();
	
	if(HAL_Status != HAL_OK){
		Debug_Var_Payload_Write = 1;
		Flash_Payload_Write_Status = FALSH_PAYLOAD_WRITE_FAILED;
	}
	else{
		for(Payload_Counter = 0; Payload_Counter < Payload_Len; Payload_Counter++){
			/* Program a byte at a specified address */
			HAL_Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Payload_Start_Address + Payload_Counter, Host_Payload[Payload_Counter]);
			if(HAL_Status != HAL_OK){
				Flash_Payload_Write_Status = FALSH_PAYLOAD_WRITE_FAILED;
				Debug_Var_Payload_Write = 2;
				break;
			}
			else{
				Debug_Var_Payload_Write = 3;
				Flash_Payload_Write_Status = FALSH_PAYLOAD_WRITE_PASSED;
			}
		}
	}
	
	if((FALSH_PAYLOAD_WRITE_PASSED == Flash_Payload_Write_Status) && (HAL_OK == HAL_Status)){
		/* Locks the FLASH control register access */
		HAL_Status = HAL_FLASH_Lock();
		if(HAL_Status != HAL_OK){
			Flash_Payload_Write_Status = FALSH_PAYLOAD_WRITE_FAILED;
			Debug_Var_Payload_Write = 4;
		}
		else{
			Flash_Payload_Write_Status = FALSH_PAYLOAD_WRITE_PASSED;
		}
	}
	else{
		Debug_Var_Payload_Write = 5;
		Flash_Payload_Write_Status = FALSH_PAYLOAD_WRITE_FAILED;
	}
	
	return Flash_Payload_Write_Status;
}

static void Bootloader_Memory_Write(uint8_t *Host_Buffer){
	uint16_t Host_CMD_Packet_Len = 0;
  uint32_t Host_CRC32 = 0;
	uint32_t HOST_Address = 0;
	uint8_t Payload_Len = 0;
	uint8_t Address_Verification = INVALID_ADDRESS;
	uint8_t Flash_Payload_Write_Status = FALSH_PAYLOAD_WRITE_FAILED;
	
	/* Extract the CRC32 and packet length sent by the HOST */
	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t *)((Host_Buffer + Host_CMD_Packet_Len) - CRC_TYPE_SIZE_BYTE));	
	/* CRC Verification */
	//if(CRC_VERIFICATION_PASSED == Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0] , Host_CMD_Packet_Len - 4, Host_CRC32)){

		/* Send acknowledgement to the HOST */
		//Bootloader_Send_ACK(1);
		/* Extract the start address from the Host packet */
		HOST_Address = *((uint32_t *)(&Host_Buffer[2]));

		/* Extract the payload length from the Host packet */
		Payload_Len = Host_Buffer[6];
		/* Verify the Extracted address to be valid address */
		Address_Verification = Host_Jump_Address_Verification(HOST_Address);
		Debug_Var_Mem_Write = 1;
		if(VALID_ADDRESS == Address_Verification){
			/* Write the payload to the Flash memory */
			Debug_Var_Mem_Write = 2;
			Flash_Payload_Write_Status = Flash_Memory_Write_Payload((uint8_t *)&Host_Buffer[7], HOST_Address, Payload_Len);
			if(FALSH_PAYLOAD_WRITE_PASSED == Flash_Payload_Write_Status){
				/* Report payload write passed */
				Debug_Var_Mem_Write = 3;
				Bootloader_Send_Data_to_Host((uint8_t *)&Flash_Payload_Write_Status, 1);
			}
			else{
				Debug_Var_Mem_Write = 4;
				/* Report payload write failed */
				Bootloader_Send_Data_to_Host((uint8_t *)&Flash_Payload_Write_Status, 1);
			}
		}
		else{
			Debug_Var_Mem_Write = 5;
			/* Report address verification failed */
			Address_Verification = INVALID_ADDRESS;
			Bootloader_Send_Data_to_Host((uint8_t *)&Address_Verification, 1);
		}
//	}
//	else{
//		/* Send Not acknowledge to the HOST */
//		Bootloader_Send_NACK();
//	}	
}

static void Bootloader_Enable_RW_Protection(uint8_t *Host_Buffer){

}

static void BootLoader_Memory_Read(uint8_t *Host_Buffer){
	
}

static void BootLoader_Get_Sector_protection_Status(uint8_t *Host_Buffer){
	
}

static void BootLoader_Read_OTP(uint8_t *Host_Buffer){
	
}

static uint8_t Change_ROP_Level(uint32_t ROP_Level){
	HAL_StatusTypeDef HAL_Status = HAL_ERROR;
	FLASH_OBProgramInitTypeDef FLASH_OBProgram;
	uint8_t ROP_Level_Status = ROP_LEVEL_CHANGE_INVALID;
	
	/* Unlock the Flash Option Control Register */
	HAL_Status = HAL_FLASH_OB_Unlock();
	if(HAL_Status != HAL_OK){
		ROP_Level_Status = ROP_LEVEL_CHANGE_INVALID;
	}
	else{
		FLASH_OBProgram.OptionType = OPTIONBYTE_RDP;							/* RDP option byte configuration */
		FLASH_OBProgram.Banks = FLASH_BANK_1;											/* Bank 1 */
		FLASH_OBProgram.RDPLevel = ROP_Level;							        /* RDP Level: 1, 2, or 3 */
		/* Program Option Bytes */
		HAL_Status = HAL_FLASHEx_OBProgram(&FLASH_OBProgram);
		if(HAL_Status != HAL_OK){
			HAL_Status = HAL_FLASH_OB_Lock();
			ROP_Level_Status = ROP_LEVEL_CHANGE_INVALID;
		}
		else{
			/* Launch Option Bytes Setting */
			HAL_Status = HAL_FLASH_OB_Launch();
			if(HAL_Status != HAL_OK){
			ROP_Level_Status = ROP_LEVEL_CHANGE_INVALID;
			}
			else{
				/* Lock the Flash Option Control Register */
				HAL_Status = HAL_FLASH_OB_Lock();
				if(HAL_Status != HAL_OK){
					ROP_Level_Status = ROP_LEVEL_CHANGE_INVALID;
				}
				else{
					ROP_Level_Status = ROP_LEVEL_CHANGE_INVALID;
				}
			}
		}
	}
	
	return ROP_Level_Status;
}

static void Bootloader_Change_Read_Protection_Level(uint8_t *Host_Buffer){
	uint16_t Host_Command_Packet_Length = 0;
	uint32_t Host_CRC32  = 0;
	uint8_t ROP_Level_Status = ROP_LEVEL_CHANGE_INVALID;
	uint8_t Host_ROP_Level = 0;
	
	/* Extract the Host Command Packet Length and CRC32 Sent by User */
	Host_Command_Packet_Length = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t *)((Host_Buffer + Host_Command_Packet_Length) - CRC_TYPE_SIZE_BYTE));
	
	/* CRC Verification */
	if(CRC_VERIFICATION_PASSED == Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0], Host_Command_Packet_Length - 4, Host_CRC32)){
		Bootloader_Send_ACK(1);
		/* Request Change the Read Out Protection Level */
		Host_ROP_Level = Host_Buffer[2];
		if(OB_RDP_LEVEL_2 == Host_ROP_Level){
			ROP_Level_Status = ROP_LEVEL_CHANGE_INVALID;
		}
		else{
			if(0  == Host_ROP_Level){
				Host_ROP_Level = 0xAA;
				}
			else if(1  == Host_ROP_Level){
				Host_ROP_Level = 0x55;
				}
			ROP_Level_Status = Change_ROP_Level((uint32_t) Host_ROP_Level);
			}
		Bootloader_Send_Data_to_Host((uint8_t *)&ROP_Level_Status, 1);
		}
	else{
		Bootloader_Send_NACK();
	}
}


void BL_Print_Message(char *Format, ...){
	char message[100] = {0};
	
	va_list args;
	/* Enable Access to the variable Arguments */
	va_start(args, Format);
	/* Write Formatted Data From Args to Message (Array) */
	vsprintf(message, Format, args);
#if	(BL_DEBUGE_MESSAGE_METHOD == BL_UART_DEBUG_MESSAGE_ENABLE)
	/* Transmit Using Universal Asynchronous Reciver Transmitter (UART) */
	HAL_UART_Transmit(BL_DEBUG_UART, (uint8_t *)message, sizeof(message), HAL_MAX_DELAY);
#elif (BL_DEBUGE_MESSAGE_METHOD == BL_SPI_DEBUG_MESSAGE_ENABLE)
	/* Transmit Using Seial Peripheral Interface (SPI) */
#elif (BL_DEBUGE_MESSAGE_METHOD == BL_CAN_DEBUG_MESSAGE_ENABLE)
	/* Transmit Using Contorller Area Network (CAN) */
#endif
	va_end(args);
}