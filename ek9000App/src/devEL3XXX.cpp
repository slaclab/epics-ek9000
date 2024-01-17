/*
 * This file is part of the EK9000 device support module. It is subject to
 * the license terms in the LICENSE.txt file found in the top-level directory
 * of this distribution and at:
 *    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
 * No part of the EK9000 device support module, including this file, may be
 * copied, modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */
//======================================================//
// Name: devEL3XXX.cpp
// Purpose: Device support for EL3xxx modules (analog in)
// Authors: Jeremy L.
// Date Created: July 6, 2019
//======================================================//

/* EPICS includes */
#include <epicsExport.h>
#include <epicsMath.h>
#include <devSup.h>
#include <alarm.h>
#include <aiRecord.h>
#include <recGbl.h>

#include <drvModbusAsyn.h>

#include <stddef.h>
#include <stdint.h>
#include "devEK9000.h"

#include "terminal_types.g.h"

//======================================================//
//
//	Common analog input routines
//
//======================================================//

static long EL3XXX_dev_report(int interest);
static long EL3XXX_init(int after);
static long EL3XXX_init_record(dbCommon* precord);
static long EL3XXX_get_ioint_info(int cmd, dbCommon* prec, IOSCANPVT* iopvt);
static long EL3XXX_linconv(aiRecord* precord, int after);

static long EL3XXX_linconv(aiRecord*, int) {
	return 0;
}

static long EL3XXX_dev_report(int) {
	return 0;
}

static long EL3XXX_init(int) {
	return 0;
}

static long EL3XXX_init_record(dbCommon* precord) {
	aiRecord* pRecord = reinterpret_cast<aiRecord*>(precord);
	pRecord->dpvt = util::allocDpvt();
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(pRecord->dpvt);
	uint16_t termid = 0;

	if (!util::setupCommonDpvt(pRecord, *dpvt)) {
		LOG_ERROR(dpvt->pdrv, "Unable to setup dpvt for record %s\n", pRecord->name);
		return 1;
	}

	// Read and validate terminal ID
	{
		DeviceLock lock(dpvt->pdrv);

		if (!lock.valid()) {
			LOG_ERROR(dpvt->pdrv, "unable to obtain device lock\n");
			return 1;
		}

		/* Check connection to terminal */
		if (!dpvt->pterm->m_device->VerifyConnection()) {
			LOG_ERROR(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(EK_ENOCONN));
			return 1;
		}

		/* Check that slave # is OK */
		termid = dpvt->pdrv->ReadTerminalID(dpvt->pterm->m_terminalIndex);
	}

	/* This is important; if the terminal id is different than what we want, report an error */
	if (termid != dpvt->pterm->m_terminalId || termid == 0) {
		LOG_ERROR(dpvt->pdrv, "%s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), pRecord->name, termid);
		return 1;
	}

	return 0;
}

//======================================================//
//
//	EL30XX Device support
//
//======================================================//
static long EL30XX_read_record(aiRecord* precord);

struct devEL30XX_t {
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
	DEVSUPFUN linconv;
};
aidset devEL30XX = {
	{
		6,
		EL3XXX_dev_report,
		EL3XXX_init,
		EL3XXX_init_record,
		EL3XXX_get_ioint_info,
	},
	EL30XX_read_record,
	EL3XXX_linconv,
};

extern "C"
{
	epicsExportAddress(dset, devEL30XX);
}

#pragma pack(1)
// This PDO type applies to EL30XX, EL31XX and EL32XX. For 31XX and 32XX some align bits are interpreted differently.
struct EL30XXStandardInputPDO_t {
	uint8_t underrange : 1;
	uint8_t overrange : 1;
	uint8_t limit1 : 2;
	uint8_t limit2 : 2;
	uint8_t _r1 : 2; // First bit in this align is error for EL31XX
	uint8_t _r2 : 6; // Last bit in this align is Sync error for EL31XX
	uint8_t txpdo_state : 1;
	uint8_t txpdo_toggle : 1;
	int16_t value;	// Must be signed to accommodate bipolar terminals. Unsigned representation still defines range as 0-32767, so this is safe.
};
#pragma pack()

// Associate our PDO struct with the terminals
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3001);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3002);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3004);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3008);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3012);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3014);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3021);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3022);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3024);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3041);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3042);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3044);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3048);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3051);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3052);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3054);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3058);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3061);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3062);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3064);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3068);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3101);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3102);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3104);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3111);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3112);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3114);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3141);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3142);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3144);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3121);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3122);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3124);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3151);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3152);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3154);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3161);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3162);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3164);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3174);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3202);

static long EL3XXX_get_ioint_info(int cmd, dbCommon* pRecord, IOSCANPVT* iopvt) {
	UNUSED(cmd);
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(pRecord->dpvt);
	if (!util::DpvtValid(dpvt))
		return 1;

	*iopvt = dpvt->pdrv->m_analog_io;
	return 0;
}

static long EL30XX_read_record(aiRecord* prec) {
	struct aiRecord* pRecord = (struct aiRecord*)prec;
	EL30XXStandardInputPDO_t* spdo;
	uint16_t buf[2];
	/*static_assert(sizeof(buf) <= sizeof(EL30XXStandardInputPDO_t),
				  "SEL30XXStandardInputPDO is greater than 4 bytes in size, contact the author about this error!");
		*/
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(pRecord->dpvt);

	/* Check for invalid */
	if (!util::DpvtValid(dpvt))
		return 1;

	int status = dpvt->pterm->getEK9000IO(READ_ANALOG, dpvt->pterm->m_inputStart + ((dpvt->channel - 1) * 2), buf, 2);
	if (status == EK_EOK) {
		spdo = reinterpret_cast<EL30XXStandardInputPDO_t*>(buf);
		pRecord->rval = spdo->value;

		/* For standard PDO types, we have limits, so we should set alarms based on these,
		apparently the error bit is just equal to (overrange || underrange) */
		if (spdo->overrange || spdo->underrange) {
			recGblSetSevr(pRecord, HW_LIMIT_ALARM, MAJOR_ALARM);
		}
	}
	else {
		recGblSetSevr(pRecord, COMM_ALARM, INVALID_ALARM);
		LOG_WARNING(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(status));
		return 1;
	}

	pRecord->udf = FALSE;
	return 0;
}

//======================================================//
//
//	EL36XX Device support
//
//======================================================//
static long EL36XX_read_record(aiRecord* precord);

aidset devEL36XX = {
	{
		6,
		EL3XXX_dev_report,
		EL3XXX_init,
		EL3XXX_init_record,
		EL3XXX_get_ioint_info,
	},
	EL36XX_read_record,
	EL3XXX_linconv,
};

extern "C"
{
	epicsExportAddress(dset, devEL36XX);
}

#pragma pack(1)
struct EL36XXInputPDO_t {
	struct {
		uint8_t underrange : 1;
		uint8_t overrange : 1;
		uint8_t limit1 : 1;
		uint8_t limit2 : 1;
		uint8_t _r0 : 4;
	} status;
	uint8_t _r1; // Corresponds to some data we don't care about
	uint32_t inp;
	uint8_t _r2 : 4;
	uint8_t sai_mode : 4;
	uint8_t sai_range;
};
#pragma pack()

DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL36XXInputPDO_t, EL3681);
DEFINE_DUMMY_OUTPUT_PDO_CHECK(EL3681); // Currently no output support for EL3681 outputs. This is a TODO!

static long EL36XX_read_record(aiRecord* prec) {
	struct aiRecord* pRecord = (struct aiRecord*)prec;
	uint16_t buf[STRUCT_SIZE_TO_MODBUS_SIZE(sizeof(EL36XXInputPDO_t))];
	EL36XXInputPDO_t* pdo = NULL;
	/*static_assert(sizeof(EL36XXInputPDO_t) <= sizeof(buf),
				  "SEL36XXInput is greater than 3 bytes in size! Contact the author regarding this error.");
		*/
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(pRecord->dpvt);

	/* Check for invalid */
	if (!util::DpvtValid(dpvt))
		return 1;

	/* Lock mutex */
	int status = dpvt->pterm->getEK9000IO(READ_ANALOG, dpvt->pterm->m_inputStart + ((dpvt->channel - 1) * 2), buf,
										  ArraySize(buf));
	if (status == EK_EOK) {
		pdo = reinterpret_cast<EL36XXInputPDO_t*>(buf);
		pRecord->rval = pdo->inp;

		/* Check the overrange and underrange flags */
		if ((pdo->status.overrange) || (pdo->status.underrange)) {
			recGblSetSevr(pRecord, HW_LIMIT_ALARM, MAJOR_ALARM);
		}
	}
	else {
		recGblSetSevr(pRecord, COMM_ALARM, INVALID_ALARM);
		LOG_WARNING(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(status));
		return 1;
	}

	pRecord->udf = FALSE;
	return 0;
}

//======================================================//
//
//	EL331X Device support
//
//======================================================//
static long EL331X_read_record(aiRecord* precord);

aidset devEL331X = {
	{
		6,
		EL3XXX_dev_report,
		EL3XXX_init,
		EL3XXX_init_record,
		EL3XXX_get_ioint_info,
	},
	EL331X_read_record,
	EL3XXX_linconv,
};

extern "C"
{
	epicsExportAddress(dset, devEL331X);
}

#pragma pack(1)
struct EL331XInputPDO_t {
	uint8_t underrange : 1;
	uint8_t overrange : 1;
	uint8_t limit1 : 2;
	uint8_t limit2 : 2;
	uint8_t error : 1;
	uint8_t txpdo_state : 1;
	uint8_t txpdo_toggle : 1;
	int16_t value;
};

struct EL3314_0010_InputPDO_t {
	uint8_t underrange : 1;
	uint8_t overrange : 1;
	uint8_t limit1 : 2;
	uint8_t limit2 : 2;
	uint8_t error : 1;
	uint8_t txpdo_state : 1;
	uint8_t txpdo_toggle : 1;
	uint8_t padding1 : 7; // Pad it out to byte boundary
	int16_t value;
};
#pragma pack()

DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL331XInputPDO_t, EL3314);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL331XInputPDO_t, EL3312);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL331XInputPDO_t, EL3311);

static long EL331X_read_record(aiRecord* prec) {
	struct aiRecord* pRecord = (struct aiRecord*)prec;

	uint16_t buf[2];
	EL331XInputPDO_t* pdo = NULL;
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(pRecord->dpvt);

	/* Check for invalid */
	if (!util::DpvtValid(dpvt))
		return 1;

	int loc = dpvt->pterm->m_inputStart + ((dpvt->channel - 1) * 2);
	int status = dpvt->pterm->getEK9000IO(READ_ANALOG, loc, buf, 2);
	if (status == EK_EOK) {
		pdo = reinterpret_cast<EL331XInputPDO_t*>(buf);
		pRecord->rval = pdo->value;

		/* Check the overrange and underrange flags */
		if (pdo->overrange || pdo->underrange) {
			recGblSetSevr(pRecord, HW_LIMIT_ALARM, MAJOR_ALARM);
		}
	}
	else {
		recGblSetSevr(pRecord, COMM_ALARM, INVALID_ALARM);
		LOG_WARNING(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(status));
		return 1;
	}

	pRecord->udf = FALSE;
	return 0;
}
