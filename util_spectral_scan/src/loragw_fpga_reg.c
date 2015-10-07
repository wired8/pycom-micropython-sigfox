/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
	Functions used to handle a single LoRa concentrator.
	Registers are addressed by name.
	Multi-bytes registers are handled automatically.
	Read-modify-write is handled automatically.

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Matthieu Leurent
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>		/* C99 types */
#include <stdbool.h>	/* bool type */
#include <stdio.h>		/* printf fprintf */

#include "loragw_fpga_spi.h"
#include "loragw_fpga_reg.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_REG == 1
	#define DEBUG_MSG(str)				fprintf(stderr, str)
	#define DEBUG_PRINTF(fmt, args...)	fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
	#define CHECK_NULL(a)				if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_REG_ERROR;}
#else
	#define DEBUG_MSG(str)
	#define DEBUG_PRINTF(fmt, args...)
	#define CHECK_NULL(a)				if(a==NULL){return LGW_REG_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

struct lgw_reg_s {
	int8_t		page;		/*!< page containing the register (-1 for all pages) */
	uint8_t		addr;		/*!< base address of the register (7 bit) */
	uint8_t		offs;		/*!< position of the register LSB (between 0 to 7) */
	bool		sign;		/*!< 1 indicates the register is signed (2 complem.) */
	uint8_t		leng;		/*!< number of bits in the register */
	bool		rdon;		/*!< 1 indicates a read-only register */
	int32_t		dflt;		/*!< register default value */
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define PAGE_ADDR		0x00
#define PAGE_MASK		0x03

#define FPGA_SPI_MUX_SX1301     0x00
#define FPGA_SPI_MUX_FPGA_REG   0x01
#define FPGA_SPI_MUX_EEPROM     0x02
#define FPGA_SPI_MUX_SX1272     0x03

/*
auto generated register mapping for C code : 11-Jul-2013 13:20:40
this file contains autogenerated C struct used to access the LoRa register from the Primer firmware
this file is autogenerated from registers description
293 registers are defined
*/
const struct lgw_reg_s fpga_regs[LGW_FPGA_TOTALREGS] = {
	{-1,0,0,0,1,0,0}, /* SOFT_RESET */
	{-1,1,0,0,8,1,18}, /* VERSION */
	{-1,2,0,0,8,1,0}, /* FPGA_STATUS */
	{-1,3,0,0,8,0,0}, /* FPGA_CTRL */
	{-1,4,0,0,8,0,0}, /* HISTO_RAM_ADDR */
	{-1,5,0,0,8,1,0}, /* HISTO_RAM_DATA */
	{-1,6,0,0,16,0,32000}, /* HISTO_TEMPO */
	{-1,8,0,0,16,0,1000}, /* HISTO_NB_READ */
	{-1,10,0,0,32,1,0}, /* TIMESTAMP */
	{-1,127,0,0,8,0,0} /* SPI_MUX_CTRL */
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

void *spi_target = NULL; /*! generic pointer to the SPI device */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_fpga_connect(void) {
	int spi_stat = LGW_SPI_SUCCESS;
	uint8_t u = 0;
	
	if (spi_target != NULL) {
		DEBUG_MSG("WARNING: FPGA was already connected\n");
		lgw_fpga_spi_close(spi_target);
	}
	
	/* open the SPI link */
	spi_stat = lgw_fpga_spi_open(&spi_target);
	if (spi_stat != LGW_SPI_SUCCESS) {
		DEBUG_MSG("ERROR CONNECTING FPGA\n");
		return LGW_REG_ERROR;
	}
	
	/* checking the version register */
	spi_stat = lgw_fpga_spi_r(spi_target, FPGA_SPI_MUX_FPGA_REG, fpga_regs[LGW_FPGA_VERSION].addr, &u);
	if (spi_stat != LGW_SPI_SUCCESS) {
		DEBUG_MSG("ERROR READING VERSION REGISTER\n");
		return LGW_REG_ERROR;
	}
	else if ((u == 0) || (u == 255)) {
		DEBUG_MSG("ERROR: FPGA SEEMS DECONNECTED\n");
		return LGW_REG_ERROR;
	}
	else if (u != fpga_regs[LGW_FPGA_VERSION].dflt) {
		DEBUG_MSG("ERROR: NOT EXPECTED FPGA VERSION\n");
		return LGW_REG_ERROR;
	}
	
	DEBUG_MSG("Note: success connecting the FPGA\n");
	return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_fpga_disconnect(void) {
	if (spi_target != NULL) {
		lgw_fpga_spi_close(spi_target);
		spi_target = NULL;
		DEBUG_MSG("Note: success disconnecting the concentrator\n");
		return LGW_REG_SUCCESS;
	} else {
		DEBUG_MSG("WARNING: concentrator was already disconnected\n");
		return LGW_REG_ERROR;
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Write to a register addressed by name */
int lgw_fpga_reg_w(uint16_t register_id, int32_t reg_value) {
	int spi_stat = LGW_SPI_SUCCESS;
	struct lgw_reg_s r;
	uint8_t buf[4] = "\x00\x00\x00\x00";
	int i, size_byte;
	
	/* check input parameters */
	if (register_id >= LGW_FPGA_TOTALREGS) {
		DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
		return LGW_REG_ERROR;
	}
	
	/* check if SPI is initialised */
	if (spi_target == NULL) {
		DEBUG_MSG("ERROR: CONCENTRATOR UNCONNECTED\n");
		return LGW_REG_ERROR;
	}
	
	/* get register struct from the struct array */
	r = fpga_regs[register_id];
	
	/* reject write to read-only registers */
	if (r.rdon == 1){
		DEBUG_MSG("ERROR: TRYING TO WRITE A READ-ONLY REGISTER\n");
		return LGW_REG_ERROR;
	}
	
	if ((r.leng == 8) && (r.offs == 0)) {
		/* direct write */
		spi_stat += lgw_fpga_spi_w(spi_target, FPGA_SPI_MUX_FPGA_REG, r.addr, (uint8_t)reg_value);
	} else if ((r.offs + r.leng) <= 8) {
		/* single-byte read-modify-write, offs:[0-7], leng:[1-7] */
		spi_stat += lgw_fpga_spi_r(spi_target, FPGA_SPI_MUX_FPGA_REG, r.addr, &buf[0]);
		buf[1] = ((1 << r.leng) - 1) << r.offs; /* bit mask */
		buf[2] = ((uint8_t)reg_value) << r.offs; /* new data offsetted */
		buf[3] = (~buf[1] & buf[0]) | (buf[1] & buf[2]); /* mixing old & new data */
		spi_stat += lgw_fpga_spi_w(spi_target, FPGA_SPI_MUX_FPGA_REG, r.addr, buf[3]);
	} else if ((r.offs == 0) && (r.leng > 0) && (r.leng <= 32)) {
		/* multi-byte direct write routine */
		size_byte = (r.leng + 7) / 8; /* add a byte if it's not an exact multiple of 8 */ 
		for (i=0; i<size_byte; ++i) {
			/* big endian register file for a file on N bytes
			Least significant byte is stored in buf[0], most one in buf[N-1] */
			buf[i] = (uint8_t)(0x000000FF & reg_value);
			reg_value = (reg_value >> 8);
		}
		spi_stat += lgw_fpga_spi_wb(spi_target, FPGA_SPI_MUX_FPGA_REG, r.addr, buf, size_byte); /* write the register in one burst */
	} else {
		/* register spanning multiple memory bytes but with an offset */
		DEBUG_MSG("ERROR: REGISTER SIZE AND OFFSET ARE NOT SUPPORTED\n");
		return LGW_REG_ERROR;
	}
	
	if (spi_stat != LGW_SPI_SUCCESS) {
		DEBUG_MSG("ERROR: SPI ERROR DURING REGISTER WRITE\n");
		return LGW_REG_ERROR;
	} else {
		return LGW_REG_SUCCESS;
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Read to a register addressed by name */
int lgw_fpga_reg_r(uint16_t register_id, int32_t *reg_value) {
	int spi_stat = LGW_SPI_SUCCESS;
	struct lgw_reg_s r;
	uint8_t bufu[4] = "\x00\x00\x00\x00";
	int8_t *bufs = (int8_t *)bufu;
	int i, size_byte;
	uint32_t u = 0;
	
	/* check input parameters */
	CHECK_NULL(reg_value);
	if (register_id >= LGW_FPGA_TOTALREGS) {
		DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
		return LGW_REG_ERROR;
	}
	
	/* check if SPI is initialised */
	if (spi_target == NULL) {
		DEBUG_MSG("ERROR: CONCENTRATOR UNCONNECTED\n");
		return LGW_REG_ERROR;
	}
	
	/* get register struct from the struct array */
	r = fpga_regs[register_id];
	
	if ((r.offs + r.leng) <= 8) {
		/* read one byte, then shift and mask bits to get reg value with sign extension if needed */
		spi_stat += lgw_fpga_spi_r(spi_target, FPGA_SPI_MUX_FPGA_REG, r.addr, &bufu[0]);
		bufu[1] = bufu[0] << (8 - r.leng - r.offs); /* left-align the data */
		if (r.sign == true) {
			bufs[2] = bufs[1] >> (8 - r.leng); /* right align the data with sign extension (ARITHMETIC right shift) */
			*reg_value = (int32_t)bufs[2]; /* signed pointer -> 32b sign extension */
		} else {
			bufu[2] = bufu[1] >> (8 - r.leng); /* right align the data, no sign extension */
			*reg_value = (int32_t)bufu[2]; /* unsigned pointer -> no sign extension */
		}
	} else if ((r.offs == 0) && (r.leng > 0) && (r.leng <= 32)) {
		size_byte = (r.leng + 7) / 8; /* add a byte if it's not an exact multiple of 8 */ 
		spi_stat += lgw_fpga_spi_rb(spi_target, FPGA_SPI_MUX_FPGA_REG, r.addr, bufu, size_byte);
		u = 0;
		for (i=(size_byte-1); i>=0; --i) {
			u = (uint32_t)bufu[i] + (u << 8); /* transform a 4-byte array into a 32 bit word */
		}
		if (r.sign == true) {
			u = u << (32 - r.leng); /* left-align the data */
			*reg_value = (int32_t)u >> (32 - r.leng); /* right-align the data with sign extension (ARITHMETIC right shift) */
		} else {
			*reg_value = (int32_t)u; /* unsigned value -> return 'as is' */
		}
	} else {
		/* register spanning multiple memory bytes but with an offset */
		DEBUG_MSG("ERROR: REGISTER SIZE AND OFFSET ARE NOT SUPPORTED\n");
		return LGW_REG_ERROR;
	}
	
	if (spi_stat != LGW_SPI_SUCCESS) {
		DEBUG_MSG("ERROR: SPI ERROR DURING REGISTER WRITE\n");
		return LGW_REG_ERROR;
	} else {
		return LGW_REG_SUCCESS;
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Point to a register by name and do a burst write */
int lgw_fpga_reg_wb(uint16_t register_id, uint8_t *data, uint16_t size) {
	int spi_stat = LGW_SPI_SUCCESS;
	struct lgw_reg_s r;
	
	/* check input parameters */
	CHECK_NULL(data);
	if (size == 0) {
		DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
		return LGW_REG_ERROR;
	}
	if (register_id >= LGW_FPGA_TOTALREGS) {
		DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
		return LGW_REG_ERROR;
	}
	
	/* check if SPI is initialised */
	if (spi_target == NULL) {
		DEBUG_MSG("ERROR: CONCENTRATOR UNCONNECTED\n");
		return LGW_REG_ERROR;
	}
	
	/* get register struct from the struct array */
	r = fpga_regs[register_id];
	
	/* reject write to read-only registers */
	if (r.rdon == 1){
		DEBUG_MSG("ERROR: TRYING TO BURST WRITE A READ-ONLY REGISTER\n");
		return LGW_REG_ERROR;
	}
	
	/* do the burst write */
	spi_stat += lgw_fpga_spi_wb(spi_target, FPGA_SPI_MUX_FPGA_REG, r.addr, data, size);
	
	if (spi_stat != LGW_SPI_SUCCESS) {
		DEBUG_MSG("ERROR: SPI ERROR DURING REGISTER BURST WRITE\n");
		return LGW_REG_ERROR;
	} else {
		return LGW_REG_SUCCESS;
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Point to a register by name and do a burst read */
int lgw_fpga_reg_rb(uint16_t register_id, uint8_t *data, uint16_t size) {
	int spi_stat = LGW_SPI_SUCCESS;
	struct lgw_reg_s r;
	
	/* check input parameters */
	CHECK_NULL(data);
	if (size == 0) {
		DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
		return LGW_REG_ERROR;
	}
	if (register_id >= LGW_FPGA_TOTALREGS) {
		DEBUG_MSG("ERROR: REGISTER NUMBER OUT OF DEFINED RANGE\n");
		return LGW_REG_ERROR;
	}
	
	/* check if SPI is initialised */
	if (spi_target == NULL) {
		DEBUG_MSG("ERROR: CONCENTRATOR UNCONNECTED\n");
		return LGW_REG_ERROR;
	}
	
	/* get register struct from the struct array */
	r = fpga_regs[register_id];
	
	/* do the burst read */
	spi_stat += lgw_fpga_spi_rb(spi_target, FPGA_SPI_MUX_FPGA_REG, r.addr, data, size);
	
	if (spi_stat != LGW_SPI_SUCCESS) {
		DEBUG_MSG("ERROR: SPI ERROR DURING REGISTER BURST READ\n");
		return LGW_REG_ERROR;
	} else {
		return LGW_REG_SUCCESS;
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_sx1272_reg_w(uint8_t address, uint8_t reg_value) {
	return lgw_fpga_spi_w(spi_target, FPGA_SPI_MUX_SX1272, address, reg_value);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_sx1272_reg_r(uint8_t address, uint8_t *reg_value) {
	return lgw_fpga_spi_r(spi_target, FPGA_SPI_MUX_SX1272, address, reg_value);
}

/* --- EOF ------------------------------------------------------------------ */
