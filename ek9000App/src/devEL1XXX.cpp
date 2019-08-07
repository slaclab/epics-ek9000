//======================================================//
// Name: devEL1XXX.cpp
// Purpose: Device support for EL1xxx modules (digital in)
// Authors: Jeremy L.
// Date Created: July 6, 2019
//======================================================//

/* EPICS includes */
#include <epicsExport.h>
#include <epicsMath.h>
#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <epicsAssert.h>
#include <dbAccess.h>
#include <devSup.h>
#include <alarm.h>
#include <epicsString.h>
#include <dbScan.h>
#include <biRecord.h>
#include <iocsh.h>
#include <callback.h>

#include <drvModbusAsyn.h>
#include <asynPortDriver.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "devEK9000.h"

/* STL includes */
#include <unordered_map>
#include <map>

struct SEL10XXSupportData
{
	int m_nChannel;
	CTerminal* m_pTerminal;
	CEK9000Device* m_pDevice;
};

static void EL10XX_ReadCallback(CALLBACK* callback);
static long EL10XX_dev_report(int interest);
static long EL10XX_init(int after);
static long EL10XX_init_record(void* precord);
static long EL10XX_read_record(void* precord);

struct
{
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
} devEL10XX = {
	5,
	(DEVSUPFUN)EL10XX_dev_report,
	(DEVSUPFUN)EL10XX_init,
	(DEVSUPFUN)EL10XX_init_record,
	NULL,
	(DEVSUPFUN)EL10XX_read_record,
};

epicsExportAddress(dset, devEL10XX);


static void EL10XX_ReadCallback(CALLBACK* callback)
{
	void* usr = 0;
	callbackGetUser(usr, callback);
	if(!usr)
		return;
	biRecord* pRecord = (biRecord*)usr;
	SEL10XXSupportData* dpvt = (SEL10XXSupportData*)pRecord->dpvt;

	/* Lock for modbus io */
	int status = dpvt->m_pDevice->Lock();

	if(status != epicsMutexLockOK)
	{
		dpvt->m_pDevice->ReportError(EK_EMUTEXTIMEOUT, "EL10XX_ReadCallback");
		pRecord->pact = 0;
		return;
	}

	/* Do the actual IO */
	uint16_t buf = 0;
	status = dpvt->m_pTerminal->doEK9000IO(MODBUS_READ_DISCRETE_INPUTS, dpvt->m_pTerminal->m_nInputStart +
		(dpvt->m_nChannel-2), &buf, 1);
	
	dpvt->m_pDevice->Unlock();

	/* Error states */
	if(status)
	{
		/* Check type of err */
		if(status > 0x100)
		{
			dpvt->m_pDevice->ReportError(EK_EMODBUSERR, "EL10XX_ReadCallback");
			return;
		}
		dpvt->m_pDevice->ReportError(status);
		return;
	}
	pRecord->pact = 0;
	pRecord->rval = buf;
}

static long EL10XX_dev_report(int interest)
{
	return 0;
}

static long EL10XX_init(int after)
{
	return 0;
}

static long EL10XX_init_record(void* precord)
{
	biRecord* pRecord = (biRecord*)precord;
	pRecord->dpvt = malloc(sizeof(SEL10XXSupportData));
	SEL10XXSupportData* dpvt = (SEL10XXSupportData*)pRecord->dpvt;

	/* Get terminal */
	char* name = NULL;
	dpvt->m_pTerminal = CTerminal::ProcessRecordName(pRecord->name, dpvt->m_nChannel, name);
	
	/* Verify terminal */
	if(!dpvt->m_pTerminal)
	{
		dbgprintf("Error while initializing record: Unable to find terminal.");
		return 1;
	}
	free(name);
	dpvt->m_pDevice = dpvt->m_pTerminal->m_pDevice;

	/* Lock mutex for modbus io */
	int status = dpvt->m_pDevice->Lock();

	/* Verify lock OK */
	if(status != epicsMutexLockOK)
	{
		dpvt->m_pDevice->ReportError(EK_EMUTEXTIMEOUT, "EL10XX_init_record");
		return 1;
	}

	/* Read termid */
	uint16_t termid = 0;
	dpvt->m_pDevice->ReadTerminalID(dpvt->m_pTerminal->m_nTerminalIndex, termid);

	/* Invalid term id */
	if(termid == 0 || termid != dpvt->m_pTerminal->m_nTerminalID)
	{
		dpvt->m_pDevice->Unlock();
		dpvt->m_pDevice->ReportError(EK_ETERMIDMIS, "EL10XX_init_record");
		return 1;
	}

	/* Unlock */
	dpvt->m_pDevice->Unlock();
	
	return 0;
}

static long EL10XX_read_record(void* precord)
{
	biRecord* pRecord = (biRecord*)precord;
	CALLBACK* callback = (CALLBACK*)calloc(1, sizeof(CALLBACK));
	*callback = *(CALLBACK*)EL10XX_ReadCallback;
	callbackSetCallback(EL10XX_ReadCallback, callback);
	callbackSetUser(pRecord, callback);
	callbackSetPriority(priorityHigh, callback);
	/* Indicate processing active */
	pRecord->pact = 1;
	callbackRequest(callback);
	return 0;
}