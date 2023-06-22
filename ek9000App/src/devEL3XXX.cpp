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
//	EL30XX Device support
//
//======================================================//
static long EL30XX_dev_report(int interest);
static long EL30XX_init(int after);
static long EL30XX_init_record(void* precord);
static long EL30XX_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt);
static long EL30XX_read_record(void* precord);
static long EL30XX_linconv(void* precord, int after);

struct devEL30XX_t {
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
	DEVSUPFUN linconv;
} devEL30XX = {
	6,
	(DEVSUPFUN)EL30XX_dev_report,
	(DEVSUPFUN)EL30XX_init,
	(DEVSUPFUN)EL30XX_init_record,
	(DEVSUPFUN)EL30XX_get_ioint_info,
	(DEVSUPFUN)EL30XX_read_record,
	(DEVSUPFUN)EL30XX_linconv,
};

epicsExportAddress(dset, devEL30XX);

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

/* Passed to device private */
struct EL30XXDPVT_t {
	devEK9000Terminal* terminal;
	devEK9000* device;
	int channel;
	TerminalDpvt_t newDpvt;
};

static long EL30XX_dev_report(int) {
	return 0;
}

static long EL30XX_init(int) {
	return 0;
}

static long EL30XX_init_record(void* precord) {
	aiRecord* pRecord = static_cast<aiRecord*>(precord);
	pRecord->dpvt = calloc(1, sizeof(EL30XXDPVT_t));
	EL30XXDPVT_t* dpvt = static_cast<EL30XXDPVT_t*>(pRecord->dpvt);

	dpvt->newDpvt = devEK9000::emptyDpvt();
	devEK9000::setupCommonDpvt<aiRecord>(pRecord, dpvt->newDpvt);

	/* Get the terminal */
	dpvt->terminal = devEK9000Terminal::ProcessRecordName(pRecord->name, &dpvt->channel);
	if (!dpvt->terminal) {
		util::Error("EL30XX_init_record(): Unable to find terminal for record %s\n", pRecord->name);
		return 1;
	}

	dpvt->device = dpvt->terminal->m_device;
	dpvt->device->Lock();

	/* Check connection to terminal */
	if (!dpvt->terminal->m_device->VerifyConnection()) {
		util::Error("EL30XX_init_record(): %s\n", devEK9000::ErrorToString(EK_ENOCONN));
		dpvt->device->Unlock();
		return 1;
	}

	/* Check that slave # is OK */
	uint16_t termid = 0;
	dpvt->terminal->m_device->ReadTerminalID(dpvt->terminal->m_terminalIndex, termid);
	dpvt->device->Unlock();

	/* This is important; if the terminal id is different than what we want, report an error */
	if (termid != dpvt->terminal->m_terminalId || termid == 0) {
		util::Error("EL30XX_init_record(): %s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), pRecord->name,
					termid);
		return 1;
	}

	return 0;
}

static long EL30XX_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt) {
	UNUSED(cmd);
	struct dbCommon* pRecord = static_cast<struct dbCommon*>(prec);
	EL30XXDPVT_t* dpvt = static_cast<EL30XXDPVT_t*>(pRecord->dpvt);
	if (!util::DpvtValid<EL30XXDPVT_t>(dpvt))
		return 1;

	*iopvt = dpvt->device->m_analog_io;
	return 0;
}

static long EL30XX_read_record(void* prec) {
	struct aiRecord* pRecord = (struct aiRecord*)prec;
	EL30XXStandardInputPDO_t* spdo;
	uint16_t buf[2];
	/*static_assert(sizeof(buf) <= sizeof(EL30XXStandardInputPDO_t),
				  "SEL30XXStandardInputPDO is greater than 4 bytes in size, contact the author about this error!");
		*/
	EL30XXDPVT_t* dpvt = static_cast<EL30XXDPVT_t*>(pRecord->dpvt);

	/* Check for invalid */
	if (!dpvt->terminal)
		return 0;

	int status;

	status = dpvt->terminal->getEK9000IO(MODBUS_READ_INPUT_REGISTERS,
										 dpvt->terminal->m_inputStart + ((dpvt->channel - 1) * 2), buf, 2);
	spdo = reinterpret_cast<EL30XXStandardInputPDO_t*>(buf);
	pRecord->rval = spdo->value;

	/* For standard PDO types, we have limits, so we should set alarms based on these,
	   apparently the error bit is just equal to (overrange || underrange) */
	if (spdo->overrange || spdo->underrange) {
		recGblSetSevr(pRecord, READ_ALARM, MAJOR_ALARM);
	}

	/* Check for error */
	if (status) {
		recGblSetSevr(pRecord, READ_ALARM, INVALID_ALARM);
		if (status > 0x100) {
			DevError("EL30XX_read_record(): %s\n", devEK9000::ErrorToString(EK_EMODBUSERR));
			return 0;
		}
		DevError("EL30XX_read_record(): %s\n", devEK9000::ErrorToString(status));
		return 0;
	}

	pRecord->pact = FALSE;
	pRecord->udf = FALSE;
	return 0;
}

static long EL30XX_linconv(void*, int) {
	return 0;
}

//======================================================//
//
//	EL36XX Device support
//
//======================================================//
static long EL36XX_dev_report(int interest);
static long EL36XX_init(int after);
static long EL36XX_init_record(void* precord);
static long EL36XX_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt);
static long EL36XX_read_record(void* precord);
static long EL36XX_linconv(void* precord, int after);

struct devEL36XX_t {
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
	DEVSUPFUN linconv;
} devEL36XX = {
	6,
	(DEVSUPFUN)EL36XX_dev_report,
	(DEVSUPFUN)EL36XX_init,
	(DEVSUPFUN)EL36XX_init_record,
	(DEVSUPFUN)EL36XX_get_ioint_info,
	(DEVSUPFUN)EL36XX_read_record,
	(DEVSUPFUN)EL36XX_linconv,
};
epicsExportAddress(dset, devEL36XX);

struct EL36XXDpvt_t {
	devEK9000Terminal* terminal;
	devEK9000* device;
	int channel;
};

#pragma pack(1)
struct EL36XXInputPDO_t {
	uint16_t status;
	uint32_t inp;
};
#pragma pack()

DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL36XXInputPDO_t, EL3681);
DEFINE_DUMMY_OUTPUT_PDO_CHECK(EL3681); // Currently no output support for EL3681 outputs. This is a TODO!

#define EL36XX_OVERRANGE_MASK 0x2
#define EL36XX_UNDERRANGE_MASK 0x1

static long EL36XX_dev_report(int) {
	return 0;
}

static long EL36XX_init(int) {
	return 0;
}

static long EL36XX_init_record(void* precord) {
	aiRecord* pRecord = static_cast<aiRecord*>(precord);
	pRecord->dpvt = calloc(1, sizeof(EL36XXDpvt_t));
	EL36XXDpvt_t* dpvt = static_cast<EL36XXDpvt_t*>(pRecord->dpvt);

	/* Get the terminal */
	dpvt->terminal = devEK9000Terminal::ProcessRecordName(pRecord->name, &dpvt->channel);
	if (!dpvt->terminal) {
		util::Error("EL36XX_init_record(): Unable to find terminal for record %s\n", pRecord->name);
		return 1;
	}

	dpvt->device = dpvt->terminal->m_device;
	dpvt->device->Lock();

	/* Check connection to terminal */
	if (!dpvt->terminal->m_device->VerifyConnection()) {
		util::Error("EL36XX_init_record(): %s\n", devEK9000::ErrorToString(EK_ENOCONN));
		dpvt->device->Unlock();
		return 1;
	}

	/* Check that slave # is OK */
	uint16_t termid = 0;
	dpvt->terminal->m_device->ReadTerminalID(dpvt->terminal->m_terminalIndex, termid);
	dpvt->device->Unlock();

	/* This is important; if the terminal id is different than what we want, report an error */
	if (termid != dpvt->terminal->m_terminalId || termid == 0) {
		util::Error("EL36XX_init_record(): %s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), pRecord->name,
					termid);
		return 1;
	}

	return 0;
}

static long EL36XX_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt) {
	UNUSED(cmd);
	struct dbCommon* pRecord = static_cast<struct dbCommon*>(prec);
	EL36XXDpvt_t* dpvt = static_cast<EL36XXDpvt_t*>(pRecord->dpvt);
	if (!util::DpvtValid<EL36XXDpvt_t>(dpvt))
		return 1;

	*iopvt = dpvt->device->m_analog_io;
	return 0;
}

static long EL36XX_read_record(void* prec) {
	struct aiRecord* pRecord = (struct aiRecord*)prec;
	uint16_t buf[3];
	EL36XXInputPDO_t* pdo = NULL;
	/*static_assert(sizeof(EL36XXInputPDO_t) <= sizeof(buf),
				  "SEL36XXInput is greater than 3 bytes in size! Contact the author regarding this error.");
		*/
	EL36XXDpvt_t* dpvt = static_cast<EL36XXDpvt_t*>(pRecord->dpvt);

	/* Check for invalid */
	if (!dpvt->terminal)
		return 0;

	/* Lock mutex */
	int status;

	status = dpvt->terminal->getEK9000IO(MODBUS_READ_INPUT_REGISTERS,
										 dpvt->terminal->m_inputStart + ((dpvt->channel - 1) * 2), buf, 2);
	pdo = reinterpret_cast<EL36XXInputPDO_t*>(buf);
	pRecord->rval = pdo->inp;

	/* Check the overrange and underrange flags */
	if ((pdo->status & EL36XX_OVERRANGE_MASK) || (pdo->status & EL36XX_UNDERRANGE_MASK)) {
		recGblSetSevr(pRecord, READ_ALARM, MAJOR_ALARM);
	}

	/* Set props */
	pRecord->pact = FALSE;
	pRecord->udf = FALSE;

	/* Check for error */
	if (status) {
		recGblSetSevr(pRecord, READ_ALARM, INVALID_ALARM);
		if (status > 0x100) {
			DevError("EL36XX_read_record(): %s\n", devEK9000::ErrorToString(EK_EMODBUSERR));
			return 0;
		}
		DevError("EL36XX_read_record(): %s\n", devEK9000::ErrorToString(status));
		return 0;
	}
	return 0;
}

static long EL36XX_linconv(void*, int) {
	return 0;
}

//======================================================//
//
//	EL331X Device support
//
//======================================================//
static long EL331X_dev_report(int interest);
static long EL331X_init(int after);
static long EL331X_init_record(void* precord);
static long EL331X_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt);
static long EL331X_read_record(void* precord);
static long EL331X_linconv(void* precord, int after);

struct devEL331X_t {
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
	DEVSUPFUN linconv;
} devEL331X = {
	6,
	(DEVSUPFUN)EL331X_dev_report,
	(DEVSUPFUN)EL331X_init,
	(DEVSUPFUN)EL331X_init_record,
	(DEVSUPFUN)EL331X_get_ioint_info,
	(DEVSUPFUN)EL331X_read_record,
	(DEVSUPFUN)EL331X_linconv,
};
epicsExportAddress(dset, devEL331X);

struct EL331XDpvt_t {
	devEK9000Terminal* terminal;
	devEK9000* device;
	int channel;
};

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

static long EL331X_dev_report(int) {
	return 0;
}

static long EL331X_init(int) {
	return 0;
}

static long EL331X_init_record(void* precord) {
	aiRecord* pRecord = static_cast<aiRecord*>(precord);
	pRecord->dpvt = calloc(1, sizeof(EL331XDpvt_t));
	EL331XDpvt_t* dpvt = static_cast<EL331XDpvt_t*>(pRecord->dpvt);

	/* Get the terminal */
	dpvt->terminal = devEK9000Terminal::ProcessRecordName(pRecord->name, &dpvt->channel);
	if (!dpvt->terminal) {
		util::Error("EL331X_init_record(): Unable to find terminal for record %s\n", pRecord->name);
		return 1;
	}

	dpvt->device = dpvt->terminal->m_device;
	dpvt->device->Lock();

	/* Check connection to terminal */
	if (!dpvt->terminal->m_device->VerifyConnection()) {
		util::Error("EL331X_init_record(): %s\n", devEK9000::ErrorToString(EK_ENOCONN));
		dpvt->device->Unlock();
		return 1;
	}

	/* Check that slave # is OK */
	uint16_t termid = 0;
	dpvt->terminal->m_device->ReadTerminalID(dpvt->terminal->m_terminalIndex, termid);
	dpvt->device->Unlock();

	/* This is important; if the terminal id is different than what we want, report an error */
	if (termid != dpvt->terminal->m_terminalId || termid == 0) {
		util::Error("EL331X_init_record(): %s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), pRecord->name,
					termid);
		return 1;
	}

	return 0;
}

static long EL331X_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt) {
	UNUSED(cmd);
	struct dbCommon* pRecord = static_cast<struct dbCommon*>(prec);
	EL331XDpvt_t* dpvt = static_cast<EL331XDpvt_t*>(pRecord->dpvt);
	if (!util::DpvtValid<EL331XDpvt_t>(dpvt))
		return 1;

	*iopvt = dpvt->device->m_analog_io;
	return 0;
}

static long EL331X_read_record(void* prec) {
	struct aiRecord* pRecord = (struct aiRecord*)prec;

	uint16_t buf[2];
	EL331XInputPDO_t* pdo = NULL;
	/*static_assert(sizeof(EL331XInputPDO_t) <= sizeof(buf),
				  "SEL331XInput is greater than 2 registers in size! Contact the author regarding this error.");
		*/
	EL331XDpvt_t* dpvt = static_cast<EL331XDpvt_t*>(pRecord->dpvt);

	/* Check for invalid */
	if (!dpvt->terminal)
		return 0;

	int status;

	int loc = dpvt->terminal->m_inputStart + ((dpvt->channel - 1) * 2);
	status = dpvt->terminal->getEK9000IO(MODBUS_READ_INPUT_REGISTERS, loc, buf, 2);

	pdo = reinterpret_cast<EL331XInputPDO_t*>(buf);
	pRecord->rval = pdo->value;

	/* Check the overrange and underrange flags */
	if (pdo->overrange || pdo->underrange) {
		recGblSetSevr(pRecord, READ_ALARM, MAJOR_ALARM);
	}

	/* Set props */
	pRecord->pact = FALSE;
	pRecord->udf = FALSE;

	/* Check for error */
	if (status) {
		recGblSetSevr(pRecord, READ_ALARM, INVALID_ALARM);
		if (status > 0x100) {

			util::Warn("EL331X_read_record(): %s\n", devEK9000::ErrorToString(EK_EMODBUSERR));
			return 0;
		}
		util::Warn("EL331X_read_record(): %s\n", devEK9000::ErrorToString(status));
		return 0;
	}
	recGblSetSevr(pRecord, READ_ALARM, NO_ALARM);
	return 0;
}

static long EL331X_linconv(void*, int) {
	return 0;
}
