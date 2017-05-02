#include "bootloader.h"
#include <string.h>

SPI_HandleTypeDef *SpiHandle;

static void wait_for_ack(void);
static uint8_t xor_checksum(const uint8_t pData[], uint8_t len);

/**
  * @brief  Initialize the SPI Bootloader host with the associated SPI handle
  *         and synchronize with target.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void BL_Init(SPI_HandleTypeDef *hspi)
{
  uint8_t sync_byte = BL_SPI_SOF;

  /* Associate SPI handle */
  SpiHandle = hspi;

  /* Send synchronization Byte */
  HAL_SPI_Transmit(SpiHandle, &sync_byte, 1U, 1000U);

  /* Get SYNC Byte ACK*/
  wait_for_ack();
}

/**
  * @brief  Get bootloader version and supported commands
  * @param  pData: pointer to destination data buffer
  * @retval None
  */
void BL_Get_Command(uint8_t *pData)
{
  uint8_t cmd_frame[3];
  uint8_t rx_number_of_bytes;

  /* Send start of frame (0x5A) + GET command frame (0x00 0xFF) */
  cmd_frame[0] = BL_SPI_SOF;
  cmd_frame[1] = GET_CMD_COMMAND;
  cmd_frame[2] = GET_CMD_COMMAND ^ 0xFFU; /*!< Command XOR checksum */
  HAL_SPI_Transmit(SpiHandle, cmd_frame, 3U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Receive data frame */
  HAL_SPI_Receive(SpiHandle, &rx_number_of_bytes, 1U, 1000U); /*!< Dummy read */
  HAL_SPI_Receive(SpiHandle, &rx_number_of_bytes, 1U, 1000U);
  HAL_SPI_Receive(SpiHandle, pData, (uint16_t) rx_number_of_bytes + 1U, 1000U);

  /* Send ACK */
  cmd_frame[0] = 0x79U;
  HAL_SPI_Transmit(SpiHandle, cmd_frame, 1U, 1000U);
}

/**
  * @brief  Get bootloader version
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval bootloader version
  */
uint8_t BL_GetVersion_Command(void)
{
  uint8_t cmd_frame[3];
  uint8_t version = 0x00U;
  uint8_t dummy = 0x00U;

  /* Send start of frame (0x5A) + Get Version command frame (0x01 0xEE) */
  cmd_frame[0] = BL_SPI_SOF;
  cmd_frame[1] = GET_VER_COMMAND;
  cmd_frame[2] = GET_VER_COMMAND ^ 0xFFU;
  HAL_SPI_Transmit(SpiHandle, cmd_frame, 3U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Receive data frame */
  HAL_SPI_Transmit(SpiHandle, &dummy, 1U, 1000U); /*!< Send dummy */
  HAL_SPI_TransmitReceive(SpiHandle, cmd_frame, &version, 1U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  return version;
}

/**
  * @brief  Get the version of the chip ID (identification)
  * @param  None
  * @retval chip ID
  */
uint16_t BL_GetID_Command(void)
{
  uint16_t id;
  uint8_t cmd_frame[3];
  uint8_t rx_number_of_bytes;
  uint8_t dummy = 0x00U;
  uint8_t rx_buffer[2];

  /* Send start of frame (0x5A) + Get ID command frame (0x02 0xFD) */
  cmd_frame[0] = BL_SPI_SOF;
  cmd_frame[1] = GET_ID_COMMAND;
  cmd_frame[2] = GET_ID_COMMAND ^ 0xFFU;
  HAL_SPI_Transmit(SpiHandle, cmd_frame, 3U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Receive data frame */
  HAL_SPI_Transmit(SpiHandle, &dummy, 1U, 1000U);
  HAL_SPI_Receive(SpiHandle, &rx_number_of_bytes, 1U, 1000U);
  HAL_SPI_Receive(SpiHandle, rx_buffer, (uint16_t) rx_number_of_bytes + 1U, 1000U);
  /* received Byte #0 is ID MSB and received Byte #1 is ID LSB */
  id = ((uint16_t) rx_buffer[0] << 8) | (rx_buffer[1]);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  return id;
}

/**
  * @brief  Read data from any valid memory address
  * @param  address: start address
  * @param  nob: number of bytes to be read
  * @param  pData: pointer to destination data buffer
  * @retval None
  */
void BL_ReadMemory_Command(uint32_t address, uint8_t nob, uint8_t *pData)
{
  uint8_t cmd_frame[3];
  uint8_t addr_frame[5];
  uint8_t nob_frame[2];
  uint8_t dummy = 0x00U;

  /* Send start of frame (0x5A) + Read Memory command frame (0x11 0xEE) */
  cmd_frame[0] = BL_SPI_SOF;
  cmd_frame[1] = RMEM_COMMAND;
  cmd_frame[2] = RMEM_COMMAND ^ 0xFFU;
  HAL_SPI_Transmit(SpiHandle, cmd_frame, 3U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Send data frame: start address (4 Bytes) + Checksum (1 Byte) */
  addr_frame[0] = (uint8_t) ((address >> 24) & 0xFFU);
  addr_frame[1] = (uint8_t) ((address >> 16) & 0xFFU);
  addr_frame[2] = (uint8_t) ((address >> 8) & 0xFFU);
  addr_frame[3] = (uint8_t) (address & 0xFFU);
  addr_frame[4] = xor_checksum(addr_frame, 4U);
  HAL_SPI_Transmit(SpiHandle, addr_frame, 5U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Send data frame: number of Bytes to be read (1 Byte) + checksum (1 Byte) */
  nob_frame[0] = (nob - 1U);
  nob_frame[1] = (nob - 1U) ^ 0xFFU;
  HAL_SPI_Transmit(SpiHandle, nob_frame, 2U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Receive data frame: data from the Bootloader */
  HAL_SPI_Transmit(SpiHandle, &dummy, 1U, 1000U);
  HAL_SPI_Receive(SpiHandle, pData, (uint16_t) nob, 1000U);
}

/**
  * @brief  Execute the downloaded code or any other code by branching to a
  *         specified address
  * @param  address: start address - 4
  * @retval None
  */
void BL_Go_Command(uint32_t address)
{
  uint8_t cmd_frame[3];
  uint8_t addr_frame[5];

  /* Send start of frame (0x5A) + Go command frame (0x21 0xDE) */
  cmd_frame[0] = BL_SPI_SOF;
  cmd_frame[1] = GO_COMMAND;
  cmd_frame[2] = GO_COMMAND ^ 0xFFU;
  HAL_SPI_Transmit(SpiHandle, cmd_frame, 3U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Send data frame: Go address (4 Bytes) + Checksum (1 Byte) */
  addr_frame[0] = ((uint8_t) (address >> 24) & 0xFFU);
  addr_frame[1] = ((uint8_t) (address >> 16) & 0xFFU);
  addr_frame[2] = ((uint8_t) (address >> 8) & 0xFFU);
  addr_frame[3] = ((uint8_t) address & 0xFFU);
  addr_frame[4] = xor_checksum(addr_frame, 4U);
  HAL_SPI_Transmit(SpiHandle, addr_frame, 5U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();
}

/**
  * @brief  Write data to any valid memory address
  * @param  address: start address
  * @param  nob: number of bytes to be written
  * @param  pData: pointer to source data pointer
  * @retval None
  */
void BL_WriteMemory_Command(uint32_t address, uint8_t nob, uint8_t *pData)
{
  uint8_t cmd_frame[3];
  uint8_t addr_frame[5];
  uint8_t n = nob - 1U;
  uint8_t checksum = xor_checksum(pData, nob) ^ n;

  /* Send start of frame (0x5A) + Write Memory command frame (0x21 0xDE) */
  cmd_frame[0] = BL_SPI_SOF;
  cmd_frame[1] = WMEM_COMMAND;
  cmd_frame[2] = WMEM_COMMAND ^ 0xFFU;
  HAL_SPI_Transmit(SpiHandle, cmd_frame, 3U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Send data frame: Write Memory start address (4 Bytes) + Checksum (1 Byte) */
  addr_frame[0] = ((uint8_t) (address >> 24) & 0xFFU);
  addr_frame[1] = ((uint8_t) (address >> 16) & 0xFFU);
  addr_frame[2] = ((uint8_t) (address >> 8) & 0xFFU);
  addr_frame[3] = ((uint8_t) address & 0xFFU);
  addr_frame[4] = xor_checksum(addr_frame, 4U);
  HAL_SPI_Transmit(SpiHandle, addr_frame, 5U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Send data frame: N number of Bytes to be written (1 Byte),
     N + 1 data Bytes and a checksum (1 Byte) */
  HAL_SPI_Transmit(SpiHandle, &n, 1U, 1000U);
  HAL_SPI_Transmit(SpiHandle, pData, (uint16_t) nob, 1000U);
  HAL_SPI_Transmit(SpiHandle, &checksum, 1U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();
}

/**
  * @brief  Erase one or more Flash memory pages or sectors
  * @param  nb: number of pages or sectors to be erased
  *         This parameter can also be one of the special erase values:
  *           @arg @ref ERASE_ALL for a global mass erase
  *           @arg @ref ERASE_BANK1 for a Bank 1 mass erase (only for products supporting this feature)
  *           @arg @ref ERASE_BANK2 for a Bank 2 mass erase (only for products supporting this feature)
  *         Values from 0xFFFC to 0xFFF0 are reserved
  * @param  code: memory page or sector
  * @retval None
  */
void BL_EraseMemory_Command(uint16_t nb, uint8_t code)
{
  uint8_t cmd_frame[3];
  uint8_t data_frame[3];

  /* Send start of frame (0x5A) + Erase Memory command frame (0x44 0xBB) */
  cmd_frame[0] = BL_SPI_SOF;
  cmd_frame[1] = EMEM_COMMAND;
  cmd_frame[2] = EMEM_COMMAND ^ 0xFFU;
  HAL_SPI_Transmit(SpiHandle, cmd_frame, 3U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Send data frame: nb (2 Bytes), the number of pages or sectors to be
     erased + checksum (1 Byte)*/
  data_frame[0] = (uint8_t) (nb >> 8) & 0xFFU;
  data_frame[1] = (uint8_t) nb & 0xFFU;
  data_frame[2] = data_frame[0] ^ data_frame[1];
  HAL_SPI_Transmit(SpiHandle, data_frame, 3U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* If non-special erase, send page number (2 Bytes) + checksum (1 Byte)*/
  if ((nb >> 4) != 0xFFF)
  {
    data_frame[0] = (uint8_t) (code >> 8) & 0xFFU;
    data_frame[1] = code & 0xFFU;
    data_frame[2] = data_frame[0] ^ data_frame[1];
    HAL_SPI_Transmit(SpiHandle, data_frame, 3U, 1000U);

    /* Wait for ACK or NACK frame */
    wait_for_ack();
  }
}

/**
  * @brief  Enable write protection for some or all Flash memory sectors
  * @param  nb: number of sectors to be write protected
  * @param  codes: pointer to sectors codes to be write protected
  * @retval None
  */
void BL_WriteProtect_Command(uint8_t nb, uint8_t *codes)
{
  uint8_t cmd_frame[3];
  uint8_t data_frame[3];
  uint8_t n = nb - 1U;
  uint8_t checksum;

  /* Send start of frame (0x5A) + Write Protect command frame (0x63 0x9C) */
  cmd_frame[0] = BL_SPI_SOF;
  cmd_frame[1] = WP_COMMAND;
  cmd_frame[2] = WP_COMMAND ^ 0xFFU;
  HAL_SPI_Transmit(SpiHandle, cmd_frame, 3U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Send data frame: number of sectors to be protected (1 Byte) + checksum (1 Byte) */
  /* The bootloader receives one byte that contains N, the number of sectors to
     be write-protected - 1 (0 <= N <= 255) */
  data_frame[0] = n;
  data_frame[1] = n ^ 0xFFU;
  HAL_SPI_Transmit(SpiHandle, data_frame, 2U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Compute Checksum */
  checksum = xor_checksum(codes, nb);

  /* Send data frame: sector codes (nb Bytes) + checksum (1 Byte) */
  memcpy(data_frame, codes, (uint32_t) nb);
  HAL_SPI_Transmit(SpiHandle, data_frame, (uint16_t) nb, 1000U);
  HAL_SPI_Transmit(SpiHandle, &checksum, 1U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();
}

/**
 * @brief Disable write protection of all the Flash memory sectors
 * @param  None
 * @retval None
 */
void BL_WriteUnprotect_Command(void)
{
  uint8_t cmd_frame[3];

  /* Send start of frame (0x5A) + Write Unprotect command frame (0x73 0x8C) */
  cmd_frame[0] = BL_SPI_SOF;
  cmd_frame[1] = WU_COMMAND;
  cmd_frame[2] = WU_COMMAND ^ 0xFFU;
  HAL_SPI_Transmit(SpiHandle, cmd_frame, 3U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Wait for second ACK or NACK frame after unprotection operation */
  wait_for_ack();
}

/**
 * @brief  Enable Flash memory read protection
 * @note   At the end of the Readout Protect command, the bootloader generates
 *         a system reset to take into account the new configuration of the
 *         option byte.
 * @param  None
 * @retval None
 */
void BL_ReadoutProtect_Command(void)
{
  uint8_t cmd_frame[3];
  uint8_t dummy = 0x00U;

  /* Send dummy */
  HAL_SPI_Transmit(SpiHandle, &dummy, 1U, 1000U);

  /* Send start of frame (0x5A) + Read Pprotect command frame (0x82 0x7D) */
  cmd_frame[0] = BL_SPI_SOF;
  cmd_frame[1] = RP_COMMAND;
  cmd_frame[2] = RP_COMMAND ^ 0xFFU;
  HAL_SPI_Transmit(SpiHandle, cmd_frame, 3U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Wait for ACK or NACK frame after read protection has been set */
  wait_for_ack();
}

/**
 * @brief  Disable Flash memory read protection
 * @note   After first ACK, the bootloader erases all the Flash
 * @param  None
 * @retval None
 */
void BL_ReadoutUnprotect_Command(void)
{
  uint8_t cmd_frame[3];

  /* Send start of frame (0x5A) + Readout Unprotect command frame (0x92 0x6D) */
  cmd_frame[0] = BL_SPI_SOF;
  cmd_frame[1] = RU_COMMAND;
  cmd_frame[2] = RU_COMMAND ^ 0xFFU;
  HAL_SPI_Transmit(SpiHandle, cmd_frame, 3U, 1000U);

  /* Wait for ACK or NACK frame */
  wait_for_ack();

  /* Wait for second ACK or NACK frame after unprotection operation */
  wait_for_ack();
}

static void wait_for_ack(void)
{
  uint8_t resp;
  uint8_t dummy = 0x00U;
  uint8_t ack = BL_ACK;
  uint32_t ack_received = 0U;

  HAL_SPI_Transmit(SpiHandle, &dummy, 1U, 1000U);
  while(ack_received != 1U)
  {
    HAL_SPI_TransmitReceive(SpiHandle, &dummy, &resp, 1U, 1000U);
    if(resp == BL_ACK)
    {
      /* Received ACK: send ACK */
      HAL_SPI_Transmit(SpiHandle, &ack, 1U, 1000U);
      ack_received = 1U;
    }
    else if (resp == BL_NAK)
    {
      /* Received NACK */
      Error_Handler();
    }
    else
    {
      /* Received junk */
    }
  }
}

static uint8_t xor_checksum(const uint8_t pData[], uint8_t len)
{
  uint8_t sum = *pData;

  for (uint8_t i = 1U; i < len; i++)
  {
    sum ^= pData[i];
  }

  return sum;
}
