/**
 ******************************************************************************
  * @file    user_diskio_spi.c
  * @brief   This file contains the implementation of the user_diskio_spi FatFs
  *          driver.
  ******************************************************************************
  * Portions copyright (C) 2014, ChaN, all rights reserved.
  * Portions copyright (C) 2017, kiwih, all rights reserved.
  *
  * This software is a free software and there is NO WARRANTY.
  * No restriction on use. You can use, modify and redistribute it for
  * personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
  * Redistributions of source code must retain the above copyright notice.
  *
  ******************************************************************************
  */

//This code was ported by kiwih from a copywrited (C) library written by ChaN
//available at http://elm-chan.org/fsw/ff/ffsample.zip
//(text at http://elm-chan.org/fsw/ff/00index_e.html)

//This file provides the FatFs driver functions and SPI code required to manage
//an SPI-connected MMC or compatible SD card with FAT

//It is designed to be wrapped by a cubemx generated user_diskio.c file.

//#include "stm32f3xx_hal.h" /* Provide the low-level HAL functions */
#include "stm32wlxx_hal.h"
#include "user_diskio_spi.h"
#include "main.h"
#include "sys_app.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdarg.h>
//Make sure you set #define SD_SPI_HANDLE as some hspix in main.h
//Make sure you set #define SD_CS_GPIO_Port as some GPIO port in main.h
//Make sure you set #define SD_CS_Pin as some GPIO pin in main.
extern SPI_HandleTypeDef hspi1;
#define SD_SPI_HANDLE hspi1
//extern void myprintf(const char *fmt, ...);
//extern UART_HandleTypeDef huart2;
//extern UART_HandleTypeDef huart1;
//extern SPI_HandleTypeDef SD_SPI_HANDLE;
void SD_ResetInterface(void);
/* Function prototypes */
// The prescaler bits are bits [5:3] of CR1 register
// Mask = 0x0038 (bits 5:3)
//#define SPI_BAUDRATE_MASK  0x0038
//(Note that the _256 is used as a mask to clear the prescalar bits as it provides binary 111 in the correct position)
#define FCLK_SLOW() { MODIFY_REG(SD_SPI_HANDLE.Instance->CR1, SPI_BAUDRATEPRESCALER_256, SPI_BAUDRATEPRESCALER_128); }	/* Set SCLK = slow, approx 280 KBits/s*/
#define FCLK_FAST() { MODIFY_REG(SD_SPI_HANDLE.Instance->CR1, SPI_BAUDRATEPRESCALER_256, SPI_BAUDRATEPRESCALER_2); }	/* Set SCLK = fast, approx 4.5 MBits/s */
//#define FCLK_SLOW() { MODIFY_REG(SD_SPI_HANDLE.Instance->CR1, SPI_CR1_BR_Msk, SPI_BAUDRATEPRESCALER_128); }
//#define FCLK_FAST() { MODIFY_REG(SD_SPI_HANDLE.Instance->CR1, SPI_CR1_BR_Msk, SPI_BAUDRATEPRESCALER_2); }
#define CS_HIGH()	{HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);}
#define CS_LOW()	{HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);}

/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

/* MMC/SD command */
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND (MMC) */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT (MMC) */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* ERASE_ER_BLK_START */
#define CMD33	(33)		/* ERASE_ER_BLK_END */
#define CMD38	(38)		/* ERASE */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */

/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK	0x08		/* Block addressing */

static volatile
DSTATUS Stat = STA_NOINIT;	/* Physical drive status */


static
BYTE CardType;			/* Card type flags */

uint32_t spiTimerTickStart;
uint32_t spiTimerTickDelay;

void SPI_Timer_On(uint32_t waitTicks) {
    spiTimerTickStart = HAL_GetTick();
    spiTimerTickDelay = waitTicks;
}

uint8_t SPI_Timer_Status() {
    return ((HAL_GetTick() - spiTimerTickStart) < spiTimerTickDelay);
}



/*-----------------------------------------------------------------------*/
/* SPI controls (Platform dependent)                                     */
/*-----------------------------------------------------------------------*/

/* Exchange a byte */
static
BYTE xchg_spi (
	BYTE dat	/* Data to send */
)
{
	BYTE rxDat;
    HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, &dat, &rxDat, 1, 50);
    return rxDat;
}


/* Receive multiple byte */
static
void rcvr_spi_multi (
	BYTE *buff,		/* Pointer to data buffer */
	UINT btr		/* Number of bytes to receive (even number) */
)
{
//	for(UINT i=0; i<btr; i++) {
//		*(buff+i) = xchg_spi(0xFF);
//	}
//	memset(buff, 0xFF, btr);
//		HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, buff, buff, btr, HAL_MAX_DELAY);
	BYTE dummy[512];
	memset(dummy, 0xFF, btr);
	HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, dummy, buff, btr, HAL_MAX_DELAY);
}


#if _USE_WRITE
/* Send multiple byte */
static
void xmit_spi_multi (
	const BYTE *buff,	/* Pointer to the data */
	UINT btx			/* Number of bytes to send (even number) */
)
{
	HAL_SPI_Transmit(&SD_SPI_HANDLE, buff, btx, HAL_MAX_DELAY);
}
#endif


/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static
int wait_ready (	/* 1:Ready, 0:Timeout */
	UINT wt			/* Timeout [ms] */
)
{
	BYTE d;
	//wait_ready needs its own timer, unfortunately, so it can't use the
	//spi_timer functions
	uint32_t waitSpiTimerTickStart;
	uint32_t waitSpiTimerTickDelay;

	waitSpiTimerTickStart = HAL_GetTick();
	waitSpiTimerTickDelay = (uint32_t)wt;
	do {
		d = xchg_spi(0xFF);
		/* This loop takes a time. Insert rot_rdq() here for multitask envilonment. */
	} while (d != 0xFF && ((HAL_GetTick() - waitSpiTimerTickStart) < waitSpiTimerTickDelay));	/* Wait for card goes ready or timeout */

	return (d == 0xFF) ? 1 : 0;
}



/*-----------------------------------------------------------------------*/
/* Despiselect card and release SPI                                         */
/*-----------------------------------------------------------------------*/

static
void despiselect (void)
{
	CS_HIGH();		/* Set CS# high */
	xchg_spi(0xFF);	/* Dummy clock (force DO hi-z for multiple slave SPI) */
	HAL_Delay(1);  // Small delay for card to enter SPI mode

}




/*-----------------------------------------------------------------------*/
/* Select card and wait for ready                                        */
/*-----------------------------------------------------------------------*/

static
int spiselect (void)	/* 1:OK, 0:Timeout */
{
	CS_LOW();		/* Set CS# low */
	xchg_spi(0xFF);	/* Dummy clock (force DO enabled) */
	if (wait_ready(500)) return 1;	/* Wait for card ready */

	despiselect();
	return 0;	/* Timeout */
}



/*-----------------------------------------------------------------------*/
/* Receive a data packet from the MMC                                    */
/*-----------------------------------------------------------------------*/

static
int rcvr_datablock (	/* 1:OK, 0:Error */
	BYTE *buff,			/* Data buffer */
	UINT btr			/* Data block length (byte) */
)
{
	BYTE token;


	SPI_Timer_On(200);
	do {							/* Wait for DataStart token in timeout of 200ms */
		token = xchg_spi(0xFF);
		/* This loop will take a time. Insert rot_rdq() here for multitask envilonment. */
	} while ((token == 0xFF) && SPI_Timer_Status());
	if(token != 0xFE) return 0;		/* Function fails if invalid DataStart token or timeout */

	rcvr_spi_multi(buff, btr);		/* Store trailing data to the buffer */
	xchg_spi(0xFF); xchg_spi(0xFF);			/* Discard CRC */

	return 1;						/* Function succeeded */
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to the MMC                                         */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
static
int xmit_datablock (	/* 1:OK, 0:Failed */
	const BYTE *buff,	/* Ponter to 512 byte data to be sent */
	BYTE token			/* Token */
)
{
	BYTE resp;


	if (!wait_ready(500)) return 0;		/* Wait for card ready */

	xchg_spi(token);					/* Send token */
	if (token != 0xFD) {				/* Send data if token is other than StopTran */
		xmit_spi_multi(buff, 512);		/* Data */
		xchg_spi(0xFF); xchg_spi(0xFF);	/* Dummy CRC */

		resp = xchg_spi(0xFF);				/* Receive data resp */
		if ((resp & 0x1F) != 0x05) return 0;	/* Function fails if the data packet was not accepted */
	}
	return 1;
}
#endif


/*-----------------------------------------------------------------------*/
/* Send a command packet to the MMC                                      */
/*-----------------------------------------------------------------------*/

static
BYTE send_cmd (		/* Return value: R1 resp (bit7==1:Failed to send) */
	BYTE cmd,		/* Command index */
	DWORD arg		/* Argument */
)
{
	BYTE n, res;


	if (cmd & 0x80) {	/* Send a CMD55 prior to ACMD<n> */
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	/* Select the card and wait for ready except to stop multiple block read */
	if (cmd != CMD12) {
		despiselect();
		if (!spiselect()) return 0xFF;
	}

	/* Send command packet */
	xchg_spi(0x40 | cmd);				/* Start + command index */
	xchg_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
	xchg_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
	xchg_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
	xchg_spi((BYTE)arg);				/* Argument[7..0] */
	n = 0x01;							/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
	xchg_spi(n);

	/* Receive command resp */
	if (cmd == CMD12) xchg_spi(0xFF);	/* Diacard following one byte when CMD12 */
//	n = 10;	/* Wait for response (10 bytes max) */
	/* Wait for response (up to 10 bytes, but with longer timeout for CMD0) */
//	n = (cmd == CMD0) ? 20 : 10;		/* Give CMD0 more time to respond */
//	do {
//		res = xchg_spi(0xFF);
//		HAL_Delay(1); // Small delay between response attempts
//	} while ((res & 0x80) && --n);
	if (cmd == CMD0) {
	    n = 50;		/* More attempts for CMD0 */
	    do {
	        res = xchg_spi(0xFF);
	        if (res & 0x80) {
	            for(volatile int i = 0; i < 1000; i++); /* ~100us delay */
	        }
	    } while ((res & 0x80) && --n);
	} else {
	    n = 10;
	    do {
	        res = xchg_spi(0xFF);
	    } while ((res & 0x80) && --n);
	}

	return res;							/* Return received response */




    // --- Handle ACMD<n> (Application commands) ---
//    if (cmd & 0x80) {
//        cmd &= 0x7F;
//        res = send_cmd(CMD55, 0);
//        if (res > 1) return res;
//    }
//
//    // --- Select the card ---
//    if (cmd != CMD12) {
//        despiselect();
//        if (!spiselect()) {
//            APP_LOG(TS_ON, VLEVEL_L, "send_cmd(CMD%d): spiselect() failed\r\n", cmd);
//            return 0xFF;
//        }
//    }
//
//    // --- Log current CS state before sending ---
//    APP_LOG(TS_ON, VLEVEL_L, "send_cmd(CMD%d): CS before sending = %d\r\n",
//            cmd, (int)HAL_GPIO_ReadPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin));
//
//    // --- Send the command packet ---
//    BYTE crc = 0x01;
//    if (cmd == CMD0) crc = 0x95;
//    if (cmd == CMD8) crc = 0x87;
//
//    APP_LOG(TS_ON, VLEVEL_L, "send_cmd(CMD%d): Sending bytes -> 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X  (CRC=0x%02X)\r\n",
//            cmd, (0x40 | cmd), (BYTE)(arg >> 24), (BYTE)(arg >> 16),
//            (BYTE)(arg >> 8), (BYTE)arg, crc);
//
//    xchg_spi(0x40 | cmd);
//    xchg_spi((BYTE)(arg >> 24));
//    xchg_spi((BYTE)(arg >> 16));
//    xchg_spi((BYTE)(arg >> 8));
//    xchg_spi((BYTE)arg);
//    xchg_spi(crc);
//
//    // --- Wait for response ---
//    if (cmd == CMD12) xchg_spi(0xFF);
//
//    if (cmd == CMD0) {
//        n = 50;
//        do {
//            res = xchg_spi(0xFF);
//        } while ((res & 0x80) && --n);
//    } else {
//        n = 10;
//        do {
//            res = xchg_spi(0xFF);
//        } while ((res & 0x80) && --n);
//    }
//
//    APP_LOG(TS_ON, VLEVEL_L, "send_cmd(CMD%d): Response = 0x%02X after %d tries\r\n", cmd, res, (cmd == CMD0) ? (50 - n) : (10 - n));
//
//    // --- Log CS state after completion ---
//    APP_LOG(TS_ON, VLEVEL_L, "send_cmd(CMD%d): CS after completion = %d\r\n",
//            cmd, (int)HAL_GPIO_ReadPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin));
//
//    return res;
}


/*--------------------------------------------------------------------------

   Public FatFs Functions (wrapped in user_diskio.c)

---------------------------------------------------------------------------*/

//The following functions are defined as inline because they aren't the functions that
//are passed to FatFs - they are wrapped by autogenerated (non-inline) cubemx template
//code.
//If you do not wish to use cubemx, remove the "inline" from these functions here
//and in the associated .h


/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/

inline DSTATUS USER_SPI_initialize (
	BYTE drv		/* Physical drive number (0) */
)
{
	BYTE n, cmd, ty, ocr[4];
	APP_LOG(TS_ON, VLEVEL_L,"SD Init: Starting initialization for drive %d\r\n", drv);

	if (drv != 0) return STA_NOINIT;		/* Supports only drive 0 */
	//assume SPI already init init_spi();	/* Initialize SPI */

	if (Stat & STA_NODISK) {
		APP_LOG(TS_ON, VLEVEL_L,"SD Init: No disk detected\r\n");
		return Stat;	/* Is card existing in the soket? */
	}
	APP_LOG(TS_ON, VLEVEL_L,"SD Init: Setting CS high and stabilizing...\r\n");
	// Ensure CS is high and give card time to stabilize
	CS_HIGH();
	HAL_Delay(250);  // 250ms delay for card to stabilize

	APP_LOG(TS_ON, VLEVEL_L,"SD Init: Setting slow clock and sending dummy clocks...\r\n");

	FCLK_SLOW();
	for (n = 80; n; n--) xchg_spi(0xFF);	/* Send 640 dummy clocks */
	// Additional delay after dummy clocks
	HAL_Delay(50);

	ty = 0;
	APP_LOG(TS_ON, VLEVEL_L,"SD Init: Sending CMD0...\r\n");
//		BYTE cmd0_result = send_cmd(CMD0, 0);
		BYTE cmd0_result = 0xFF;
//		for (n = 0; n < 10 && cmd0_result != 1; n++) {
//		    cmd0_result = send_cmd(CMD0, 0);
//		    myprintf("SD Init: CMD0 attempt %d, response = %d\r\n", n+1, cmd0_result);
//		    if (cmd0_result != 1) {
//		        HAL_Delay(10); // Wait between attempts
//		    }
//		}
//		myprintf("SD Init: CMD0 response = %d (expected 1)\r\n", cmd0_result);
		for (n = 0; n < 20 && cmd0_result != 1; n++) {
		    // Ensure CS is properly cycled
		    CS_HIGH();
		    HAL_Delay(10);
		    CS_LOW();

		    cmd0_result = send_cmd(CMD0, 0);
		    APP_LOG(TS_ON, VLEVEL_L,"SD Init: CMD0 attempt %d, response = 0x%02X\r\n", n+1, cmd0_result);

		    if (cmd0_result != 1) {
		        HAL_Delay(50); // Increased wait between attempts
		    }
		}
		APP_LOG(TS_ON, VLEVEL_L,"SD Init: CMD0 response = 0x%02X (expected 0x01)\r\n", cmd0_result);

		if (cmd0_result == 1) {			/* Put the card SPI/Idle state */
			APP_LOG(TS_ON, VLEVEL_L,"SD Init: CMD0 successful, starting 1sec timeout\r\n");
			SPI_Timer_On(2000);					/* Initialization timeout = 1 sec */

			APP_LOG(TS_ON, VLEVEL_L,"SD Init: Sending CMD8...\r\n");
			BYTE cmd8_result = send_cmd(CMD8, 0x1AA);
			APP_LOG(TS_ON, VLEVEL_L,"SD Init: CMD8 response = %d\r\n", cmd8_result);

			if (cmd8_result == 1) {	/* SDv2? */
				APP_LOG(TS_ON, VLEVEL_L,"SD Init: SD v2 card detected\r\n");
				for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);	/* Get 32 bit return value of R7 resp */
				APP_LOG(TS_ON, VLEVEL_L,"SD Init: CMD8 OCR = %02X %02X %02X %02X\r\n", ocr[0], ocr[1], ocr[2], ocr[3]);

				if (ocr[2] == 0x01 && ocr[3] == 0xAA) {				/* Is the card supports vcc of 2.7-3.6V? */
					APP_LOG(TS_ON, VLEVEL_L,"SD Init: Voltage range supported, sending ACMD41...\r\n");
					while (SPI_Timer_Status() && send_cmd(ACMD41, 1UL << 30))
					{
					    HAL_Delay(10); /* Small delay between ACMD41 attempts */
					}	/* Wait for end of initialization with ACMD41(HCS) */
					if (SPI_Timer_Status() && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
						for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
						ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* Card id SDv2 */
						APP_LOG(TS_ON, VLEVEL_L,"SD Init: ACMD41 successful, card type = %d\r\n", ty);
					} else {
						APP_LOG(TS_ON, VLEVEL_L,"SD Init: ACMD41 failed or timeout\r\n");
					}
				} else {
					APP_LOG(TS_ON, VLEVEL_L,"SD Init: Unsupported voltage range\r\n");
				}
			} else {	/* Not SDv2 card */
				APP_LOG(TS_ON, VLEVEL_L,"SD Init: Not SD v2, trying SD v1 or MMC\r\n");
				if (send_cmd(ACMD41, 0) <= 1) 	{	/* SDv1 or MMC? */
					ty = CT_SD1; cmd = ACMD41;	/* SDv1 (ACMD41(0)) */
					APP_LOG(TS_ON, VLEVEL_L,"SD Init: SD v1 detected\r\n");
				} else {
					ty = CT_MMC; cmd = CMD1;	/* MMCv3 (CMD1(0)) */
					APP_LOG(TS_ON, VLEVEL_L,"SD Init: MMC detected\r\n");
				}
				while (SPI_Timer_Status() && send_cmd(cmd, 0)) ;		/* Wait for end of initialization */
				if (!SPI_Timer_Status() || send_cmd(CMD16, 512) != 0)	/* Set block length: 512 */
					ty = 0;
			}
		} else {
			APP_LOG(TS_ON, VLEVEL_L,"SD Init: CMD0 failed - card not responding\r\n");
		}
	CardType = ty;	/* Card type */
	despiselect();

	if (ty) {			/* OK */
		APP_LOG(TS_ON, VLEVEL_L,"SD Init: Success! Setting fast clock\r\n");
		FCLK_FAST();			/* Set fast clock */
		APP_LOG(TS_ON, VLEVEL_L, "APB2CLK = %u Hz\r\n",
		            (unsigned int)HAL_RCC_GetPCLK2Freq());
		APP_LOG(TS_ON, VLEVEL_L, "CR1 after FCLK_FAST = 0x%04x\r\n", (unsigned int)(SD_SPI_HANDLE.Instance->CR1));
		Stat &= ~STA_NOINIT;	/* Clear STA_NOINIT flag */
	} else {			/* Failed */
		APP_LOG(TS_ON, VLEVEL_L,"SD Init: Failed - final card type = 0\r\n");
		Stat = STA_NOINIT;
	}
	APP_LOG(TS_ON, VLEVEL_L,"SD Init: Completed with status = %d\r\n", Stat);

	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Get disk status                                                       */
/*-----------------------------------------------------------------------*/

inline DSTATUS USER_SPI_status (
	BYTE drv		/* Physical drive number (0) */
)
{
	if (drv) return STA_NOINIT;		/* Supports only drive 0 */

	return Stat;	/* Return disk status */
}



/*-----------------------------------------------------------------------*/
/* Read sector(s)                                                        */
/*-----------------------------------------------------------------------*/

inline DRESULT USER_SPI_read (
	BYTE drv,		/* Physical drive number (0) */
	BYTE *buff,		/* Pointer to the data buffer to store read data */
	DWORD sector,	/* Start sector number (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
//	APP_LOG(TS_ON, VLEVEL_L,"USER_SPI_read: drv %d\r\n",drv);
//	APP_LOG(TS_ON, VLEVEL_L,"USER_SPI_read: drv %d\r\n",count);
//	APP_LOG(TS_ON, VLEVEL_L,"USER_SPI_read: drv %d\r\n",sector);
	if (drv || !count) return RES_PARERR;		/* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check if drive is ready */

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* LBA ot BA conversion (byte addressing cards) */

	if (count == 1) {	/* Single sector read */
		if ((send_cmd(CMD17, sector) == 0)	/* READ_SINGLE_BLOCK */
			&& rcvr_datablock(buff, 512)) {
			count = 0;
		}
	}
	else {				/* Multiple sector read */
		if (send_cmd(CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
			do {
				if (!rcvr_datablock(buff, 512)) break;
				buff += 512;
			} while (--count);
			send_cmd(CMD12, 0);				/* STOP_TRANSMISSION */
		}
	}
	despiselect();

	return count ? RES_ERROR : RES_OK;	/* Return result */
}



/*-----------------------------------------------------------------------*/
/* Write sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
inline DRESULT USER_SPI_write (
	BYTE drv,			/* Physical drive number (0) */
	const BYTE *buff,	/* Ponter to the data to write */
	DWORD sector,		/* Start sector number (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	if (drv || !count) return RES_PARERR;		/* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check drive status */
	if (Stat & STA_PROTECT) return RES_WRPRT;	/* Check write protect */

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* LBA ==> BA conversion (byte addressing cards) */

	if (count == 1) {	/* Single sector write */
		if ((send_cmd(CMD24, sector) == 0)	/* WRITE_BLOCK */
			&& xmit_datablock(buff, 0xFE)) {
			count = 0;
		}
	}
	else {				/* Multiple sector write */
		if (CardType & CT_SDC) send_cmd(ACMD23, count);	/* Predefine number of sectors */
		if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD)) count = 1;	/* STOP_TRAN token */
		}
	}
	despiselect();

	return count ? RES_ERROR : RES_OK;	/* Return result */
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous drive controls other than data read/write               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
inline DRESULT USER_SPI_ioctl (
	BYTE drv,		/* Physical drive number (0) */
	BYTE cmd,		/* Control command code */
	void *buff		/* Pointer to the conrtol data */
)
{
	DRESULT res;
	BYTE n, csd[16];
	DWORD *dp, st, ed, csize;


	if (drv) return RES_PARERR;					/* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check if drive is ready */

	res = RES_ERROR;

	switch (cmd) {
	case CTRL_SYNC :		/* Wait for end of internal write process of the drive */
		if (spiselect()) res = RES_OK;
		break;

	case GET_SECTOR_COUNT :	/* Get drive capacity in unit of sector (DWORD) */
		if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
			if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
				csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
				*(DWORD*)buff = csize << 10;
			} else {					/* SDC ver 1.XX or MMC ver 3 */
				n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
				csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
				*(DWORD*)buff = csize << (n - 9);
			}
			res = RES_OK;
		}
		break;

	case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
		if (CardType & CT_SD2) {	/* SDC ver 2.00 */
			if (send_cmd(ACMD13, 0) == 0) {	/* Read SD status */
				xchg_spi(0xFF);
				if (rcvr_datablock(csd, 16)) {				/* Read partial block */
					for (n = 64 - 16; n; n--) xchg_spi(0xFF);	/* Purge trailing data */
					*(DWORD*)buff = 16UL << (csd[10] >> 4);
					res = RES_OK;
				}
			}
		} else {					/* SDC ver 1.XX or MMC */
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {	/* Read CSD */
				if (CardType & CT_SD1) {	/* SDC ver 1.XX */
					*(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
				} else {					/* MMC */
					*(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
				}
				res = RES_OK;
			}
		}
		break;

	case CTRL_TRIM :	/* Erase a block of sectors (used when _USE_ERASE == 1) */
		if (!(CardType & CT_SDC)) break;				/* Check if the card is SDC */
		if (USER_SPI_ioctl(drv, MMC_GET_CSD, csd)) break;	/* Get CSD */
		if (!(csd[0] >> 6) && !(csd[10] & 0x40)) break;	/* Check if sector erase can be applied to the card */
		dp = buff; st = dp[0]; ed = dp[1];				/* Load sector block */
		if (!(CardType & CT_BLOCK)) {
			st *= 512; ed *= 512;
		}
		if (send_cmd(CMD32, st) == 0 && send_cmd(CMD33, ed) == 0 && send_cmd(CMD38, 0) == 0 && wait_ready(30000)) {	/* Erase sector block */
			res = RES_OK;	/* FatFs does not check result of this command */
		}
		break;

	default:
		res = RES_PARERR;
	}

	despiselect();

	return res;
}
void SD_ResetInterface(void)
{
    uint8_t dummy[10];

    // Deinit and reinit SPI to reset peripheral state
    HAL_SPI_DeInit(&SD_SPI_HANDLE);
    HAL_SPI_Init(&SD_SPI_HANDLE);

    // Set CS high to deselect card
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(2); // let card stabilize

    // Send a few dummy clocks with CS high
    memset(dummy, 0xFF, sizeof(dummy));
    HAL_SPI_Transmit(&SD_SPI_HANDLE, dummy, sizeof(dummy), HAL_MAX_DELAY);
}
#endif
