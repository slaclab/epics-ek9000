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
// Name: devEK9000.cpp
// Purpose: Device support for EK9000 and terminals
// Authors: Jeremy L.
// Date Created: June 10, 2019
// Notes:
//	-	For device specific PDOs and such, refer to the docs:
//		EL1XXX: https://download.beckhoff.com/download/document/io/ethercat-terminals/el10xx_el11xxen.pdf
//		EL2XXX: https://download.beckhoff.com/download/document/io/ethercat-terminals/EL20xx_EL2124en.pdf
//		EL3XXX: https://download.beckhoff.com/download/document/io/ethercat-terminals/el30xxen.pdf
//		EL4XXX: https://download.beckhoff.com/download/document/io/ethercat-terminals/el40xxen.pdf
//======================================================//

/* EPICS includes */
#include <epicsExport.h>
#include <epicsStdlib.h>
#include <epicsAssert.h>
#include <epicsPrint.h>
#include <devSup.h>
#include <epicsString.h>
#include <boRecord.h>
#include <iocsh.h>
#include <callback.h>
#include <errno.h>
#include <sys/time.h>

/* Record includes */
#include <longinRecord.h>
#include <longoutRecord.h>
#include <int64inRecord.h>
#include <int64outRecord.h>
#include <dbStaticLib.h>

/* Modbus or asyn includes */
// #include <drvModbusAsyn.h>
#include <drvAsynIPPort.h>
#include <modbusInterpose.h>

#include "alarm.h"
#include "devEK9000.h"
#include "ekUtil.h"
#include "errlog.h"
#include "recGbl.h"
#include "terminal_types.g.h"

#define EK9000_SLAVE_ID 0

/* Some settings */
#define POLL_DURATION 0.05
#define TIMEOUT_COUNT 50

/* Forward decls */
class devEK9000;

/* Globals */
static epicsThreadId g_PollThread = 0;

/* Global list accessor */
std::list<devEK9000*>& GlobalDeviceList() {
	static std::list<devEK9000*> devices;
	return devices;
}

bool devEK9000::debugEnabled = false;
int devEK9000::pollDelay = 200;

// This is a big hack for safety reasons! This will force you to use the DEFINE_XXX_PDO macro for every terminal type at
// least once, so we can catch mismatches between terminals.json and the in-code PDO structs.
// LTO may remove this symbol in release.
void ek9000_PDOHack() {
	__pdo_check();
}

#ifndef EK9000_MOTOR_SUPPORT
DEFINE_DUMMY_INPUT_PDO_CHECK(EL7047)
DEFINE_DUMMY_OUTPUT_PDO_CHECK(EL7047)
#endif

//==========================================================//
// Utils
//==========================================================//
void PollThreadFunc(void* param);

void Utl_InitThread() {
	g_PollThread = epicsThreadCreate("EK9000_PollThread", priorityHigh, epicsThreadGetStackSize(epicsThreadStackMedium),
									 PollThreadFunc, NULL);
}

// This thread does two things:
//      - Check the connection and reset the watchdog every other poll time.
//      - Poll for new EL1xxx/EL3xxx/EL5xxx data every poll time.
void PollThreadFunc(void*) {
	int cnt = 0;
	struct timeval start, finish, last_read_status;
	memset(&last_read_status, 0, sizeof(last_read_status));
	double duration_ms = -1.0;
	while (true) {
		gettimeofday(&start, NULL);
		// for (auto device : GlobalDeviceList()) {
		for (std::list<devEK9000*>::iterator it = GlobalDeviceList().begin(); it != GlobalDeviceList().end(); ++it) {
			devEK9000* device = *it;
			DeviceLock lock(device);
			if (!lock.valid())
				continue;
			if (!cnt) {
				/* check connection every other loop */
				bool connected = device->VerifyConnection();
				if (!connected && device->m_connected) {
					LOG_WARNING(device, "%s: Link status changed to DISCONNECTED\n", device->m_name.data());
					device->m_connected = false;
				}
				if (connected && !device->m_connected) {
					LOG_WARNING(device, "%s: Link status changed to CONNECTED\n", device->m_name.data());
					device->m_connected = true;
				}
				/* Skip poll if we're not connected */
				if (!device->m_connected) {
					LOG_INFO(device, "%s: device not connected, skipping poll", device->m_name.data());
					continue;
				}
				uint16_t buf = 1;
				if (device->doModbusIO(0, MODBUS_WRITE_SINGLE_REGISTER, 0x1121, &buf, 1)) {
					LOG_WARNING(device, "%s: FAILED TO RESET WATCHDOG!\n", device->m_name.data());
				}
			}

			// Read status registers only after a ~1 second delay
			if (((start.tv_sec + start.tv_usec / 1e6) - (last_read_status.tv_sec + last_read_status.tv_usec / 1e6)) >=
				1.0) {
				device->m_status_status = device->doModbusIO(0, MODBUS_READ_INPUT_REGISTERS, EK9000_STATUS_START,
															 device->m_status_buf, ArraySize(device->m_status_buf));

				bool ebus = device->m_status_buf[EK9000_STATUS_EBUS_STATUS - EK9000_STATUS_START] == 1;
				if (ebus != device->m_ebus_ok) {
					device->m_ebus_ok = ebus;
					LOG_WARNING(device, "%s: E-Bus status switched to %s\n", device->m_name.data(),
								ebus ? "OK" : "FAULT");
				}
				scanIoRequest(device->m_status_io);
				gettimeofday(&last_read_status, NULL);
				// Signal digital/analog error
				if (!ebus)
					device->m_digital_status = device->m_analog_status = asynError;
			}

			/* read EL1xxx/EL3xxx/EL5xxx data */
			if (device->m_digital_cnt && device->m_ebus_ok) {
				device->m_digital_status =
					device->doModbusIO(0, MODBUS_READ_DISCRETE_INPUTS, 0, device->m_digital_buf, device->m_digital_cnt);
				scanIoRequest(device->m_digital_io);
			}
			if (device->m_analog_cnt && device->m_ebus_ok) {
				device->m_analog_status =
					device->doModbusIO(0, MODBUS_READ_INPUT_REGISTERS, 0, device->m_analog_buf, device->m_analog_cnt);
				scanIoRequest(device->m_analog_io);
			}
		}
		cnt = (cnt + 1) % 2;
		gettimeofday(&finish, NULL);
		duration_ms = (finish.tv_sec - start.tv_sec) * 1000. + (finish.tv_usec - start.tv_usec) / 1000.;
		if (duration_ms < devEK9000::pollDelay)
			epicsThreadSleep(((float)devEK9000::pollDelay - duration_ms) / 1000.0f);
	}
}

//==========================================================//
// class devEK9000Terminal
//		Holds important info about the terminals
//==========================================================//

devEK9000Terminal::devEK9000Terminal(devEK9000* device) {
	/* Terminal family */
	m_terminalFamily = 0;
	/* Zero-based index of the terminal */
	m_terminalIndex = 0;
	/* the device */
	m_device = device;
	/* Terminal id, aka the 1124 in EL1124 */
	m_terminalId = 0;
	/* Size of inputs */
	m_inputSize = 0;
	/* Size of outputs */
	m_outputSize = 0;
	/* input image start */
	m_inputStart = 0;
	/* Output image start */
	m_outputStart = 0;
}

void devEK9000Terminal::Init(uint32_t termid, int termindex) {
	int outp = 0, inp = 0;

	/* Create IO areas and set things like record name */
	this->m_terminalIndex = termindex;
	this->m_terminalId = termid;

	if (termid >= 1000 && termid < 3000)
		this->m_terminalFamily = TERMINAL_FAMILY_DIGITAL;
	else if (termid >= 3000 && termid < 8000)
		this->m_terminalFamily = TERMINAL_FAMILY_ANALOG;

	/* Get the process image size for this terminal */
	devEK9000Terminal::GetTerminalInfo((int)termid, inp, outp);
	this->m_inputSize = inp;
	this->m_outputSize = outp;
}

// If multi is true, we do not expect a channel selector at the end of the record name
// In that case, outindex is not set
// This is LEGACY code, to maintain compatibility with older setups
devEK9000Terminal* devEK9000Terminal::ProcessRecordName(const char* recname, int* outindex) {
	char ret[512];
	strncpy(ret, recname, sizeof(ret) - 1);
	ret[sizeof(ret) - 1] = 0;

	size_t len = strlen(ret);

	if (outindex) {
		bool good = false;
		for (size_t i = len; i >= 0; i--) {
			if (ret[i] == ':' && (size_t)i < len) {
				ret[i] = '\0';
				if (util::parseNumber(ret + 1, *outindex, 10))
					good = true;
				break;
			}
		}
		if (!good)
			return NULL;
	}

	// for (auto dev : GlobalDeviceList()) {
	for (std::list<devEK9000*>::iterator it = GlobalDeviceList().begin(); it != GlobalDeviceList().end(); ++it) {
		devEK9000* dev = *it;
		for (int i = 0; i < dev->m_numTerms; i++) {
			if (dev->m_terms[i]->m_recordName.empty())
				continue;
			if (strcmp(dev->m_terms[i]->m_recordName.data(), ret) == 0) {
				return dev->m_terms[i];
			}
		}
	}
	return NULL;
}

void devEK9000Terminal::GetTerminalInfo(int termid, int& inp_size, int& out_size) {
	for (size_t i = 0; i < ArraySize(s_terminalInfos); i++) {
		if (s_terminalInfos[i].id == (uint32_t)termid) {
			inp_size = s_terminalInfos[i].inputSize;
			out_size = s_terminalInfos[i].outputSize;
			return;
		}
	}
}

int devEK9000Terminal::doEK9000IO(int type, int startaddr, uint16_t* buf, int len) {
	if (!this->m_device) {
		return EK_EBADTERM;
	}
	int status = this->m_device->doModbusIO(0, type, startaddr, buf, len);
	if (status) {
		return EK_EMODBUSERR;
	}
	return EK_EOK;
}

int devEK9000Terminal::getEK9000IO(EIOType type, int startaddr, uint16_t* buf, int len) {
	if (!this->m_device)
		return EK_EBADTERM;
	return m_device->getEK9000IO(type, startaddr, buf, len);
}

int devEK9000::getEK9000IO(EIOType type, int startaddr, uint16_t* buf, uint16_t len) {
	int status = 0;
	DeviceLock lock(this);
	if (!lock.valid())
		return EK_EMUTEXTIMEOUT;

	if (type == READ_DIGITAL) { /* digital */
		if (startaddr < 0 || startaddr + len > this->m_digital_cnt)
			status = EK_EBADPARAM;
		else if (this->m_digital_status)
			status = this->m_digital_status;
		else {
			memcpy(buf, this->m_digital_buf + startaddr, len * sizeof(uint16_t));
			status = EK_EOK;
		}
	}
	else if (type == READ_ANALOG) { /* analog */
		if (startaddr < 0 || startaddr + len > this->m_analog_cnt)
			status = EK_EBADPARAM;
		else if (this->m_analog_status)
			status = this->m_analog_status;
		else {
			memcpy(buf, this->m_analog_buf + startaddr, len * sizeof(uint16_t));
			status = EK_EOK;
		}
	}
	else if (type == READ_STATUS) {
		startaddr -= EK9000_STATUS_START;
		if (startaddr < 0 || size_t(startaddr + len) > ArraySize(m_status_buf))
			status = EK_EBADPARAM;
		else if (this->m_status_status)
			status = this->m_status_status;
		else {
			memcpy(buf, this->m_status_buf + startaddr, len * sizeof(uint16_t));
			status = EK_EOK;
		}
	}
	else
		status = EK_EBADPARAM;
	return status;
}

//==========================================================//
// class devEK9000
//		Holds useful vars for interacting with EK9000/EL****
//		hardware
//==========================================================//
devEK9000::devEK9000(const char* portname, const char* octetPortName, int termCount, const char* ip)
	: drvModbusAsyn(portname, octetPortName, 0, 2, -1, 256, dataTypeUInt16, 150, "") {

	/* Initialize members */
	for (int i = 0; i < termCount; i++)
		m_terms.push_back(new devEK9000Terminal(this));

	m_numTerms = termCount;
	m_ip = ip;
	m_connected = false;
	m_init = false;
	m_debug = false;
	m_error = EK_EOK;
	LastADSErr = 0;
	m_name = portname;
	m_readTerminals = false;
	m_octetPortName = octetPortName;
	m_ebus_ok = true;

	this->m_Mutex = epicsMutexCreate();
	m_analog_status = EK_EERR + 0x100; /* No data yet!! */
	m_digital_status = EK_EERR + 0x100;
}

devEK9000::~devEK9000() {
	epicsMutexDestroy(this->m_Mutex);
	for (size_t i = 0; i < m_terms.size(); ++i)
		delete m_terms[i];
}

devEK9000* devEK9000::FindDevice(const char* name) {
	// for (auto dev : GlobalDeviceList()) {
	for (std::list<devEK9000*>::iterator it = GlobalDeviceList().begin(); it != GlobalDeviceList().end(); ++it) {
		devEK9000* dev = *it;
		if (!strcmp(name, dev->m_name.data()))
			return dev;
	}
	// return nullptr;
	return NULL;
}

devEK9000* devEK9000::Create(const char* name, const char* ip, int terminal_count) {
	if (terminal_count < 0 || !name || !ip)
		return NULL;

	std::string octetPortName = PORT_PREFIX;
	octetPortName.append(name);

	int status = drvAsynIPPortConfigure(octetPortName.data(), ip, 0, 0, 0);

	if (status) {
		epicsPrintf("devEK9000::Create(): Unable to configure drvAsynIPPort.");
		return NULL;
	}

	status = modbusInterposeConfig(octetPortName.data(), modbusLinkTCP, 5000, 0);

	if (status) {
		epicsPrintf("devEK9000::Create(): Unable to configure modbus driver.");
		return NULL;
	}

	/* check connection */
	asynUser* usr = pasynManager->createAsynUser(NULL, NULL);
	pasynManager->connectDevice(usr, octetPortName.data(), 0);
	int conn = 0;
	pasynManager->isConnected(usr, &conn);
	pasynManager->disconnect(usr);
	pasynManager->freeAsynUser(usr);

	if (!conn) {
		epicsPrintf("devEK9000::Create(): Error while connecting to device %s.", name);
		return NULL;
	}

	devEK9000* pek = new devEK9000(name, octetPortName.c_str(), terminal_count, ip);

	/* Copy IP */
	pek->m_ip = ip;

	/* wdt =  */
	uint16_t buf = 1;
	pek->doModbusIO(0, MODBUS_WRITE_SINGLE_REGISTER, 0x1122, &buf, 1);

	if (!pek->ComputeTerminalMapping()) {
		epicsPrintf("devEK9000::Create(): Unable to compute terminal mapping\n");
		delete pek;
		return NULL;
	}

	GlobalDeviceList().push_back(pek);
	return pek;
}

int devEK9000::AddTerminal(const char* name, uint32_t type, int position) {
	if (position > m_numTerms || !name)
		return EK_EBADPARAM;

	m_terms[position - 1]->Init(type, position);
	m_terms[position - 1]->SetRecordName(name);
	return EK_EOK;
}

/* Verifies that terminals have the correct ID */
/* Sets the process image size */
int devEK9000::InitTerminal(int term) {
	if (term < 0 || term >= m_numTerms)
		return EK_EBADPARAM;

	/* Read ther terminal's id */
	uint16_t tid = this->ReadTerminalID(term);

	/* Verify that the terminal has the proper id */
	devEK9000Terminal* terminal = this->m_terms[term];

	if (tid != terminal->m_terminalId)
		return EK_ETERMIDMIS;

	return EK_EOK;
}

/* This will configure process image locations in each terminal */
/* It will also verify that terminals have the correct type (reads terminal type from the device then yells if its not
the same as the user specified.) */
bool devEK9000::ComputeTerminalMapping() {
	if (m_init) {
		epicsPrintf("devEK9000: Already initialized.\n");
		return false;
	}
	m_init = true;

	/* Gather a buffer of connected terminals */
	uint16_t railLayout[0xFF];
	memset(railLayout, 0, sizeof(railLayout));
	for (int i = 0; i < 0xFF; i += 64) {
		if (this->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, 0x6001 + i, railLayout + i, 64) != asynSuccess) {
			epicsPrintf("%s: Failed to read rail layout from the device\n", __FUNCTION__);
			return false;
		}
	}

	assert(size_t(m_numTerms) <= ArraySize(railLayout));

	/* Figure out the register map */
	int coil_in = 1, coil_out = 1;
	int reg_in = 0, reg_out = 0x800;
	/* in = holding regs, out = inp regs */
	/* analog terms are mapped FIRST */
	/* then digital terms are mapped */
	/* holding registers can have bit offsets */
	for (int i = 0; i < this->m_numTerms; i++) {
		devEK9000Terminal* term = m_terms[i];
		term->Init(railLayout[i], i);
		if (term->m_terminalFamily == TERMINAL_FAMILY_ANALOG) {
			DevInfo("Mapped %u: inp_start(0x%X) out_start(0x%X) inp_size(0x%X) outp_size(0x%X)\n", term->m_terminalId,
					reg_in, reg_out, term->m_inputSize, term->m_outputSize);
			term->m_inputStart = reg_in;
			term->m_outputStart = reg_out;
			reg_in += term->m_inputSize;
			reg_out += term->m_outputSize;
		}
		if (term->m_terminalFamily == TERMINAL_FAMILY_DIGITAL) {
			DevInfo("Mapped %u: inp_start(0x%X) out_start(0x%X) inp_size(0x%X) outp_size(0x%X)\n", term->m_terminalId,
					coil_in, coil_out, term->m_inputSize, term->m_outputSize);
			term->m_inputStart = coil_in;
			term->m_outputStart = coil_out;
			coil_in += term->m_inputSize;
			coil_out += term->m_outputSize;
		}
	}
	/* Now that we have counts, allocate buffer space! */
	scanIoInit(&m_analog_io);
	scanIoInit(&m_digital_io);
	scanIoInit(&m_status_io);
	m_analog_cnt = reg_in;
	if (m_analog_cnt)
		m_analog_buf = (uint16_t*)calloc(m_analog_cnt, sizeof(uint16_t)); /* We read status bits too! */
	else
		m_analog_buf = NULL;
	m_digital_cnt = coil_in - 1;
	if (coil_in != 1)
		// Despite being 1-bit inputs, the modbus driver gives us one digital input per 16-bit int in the output buffer
		m_digital_buf = (uint16_t*)calloc(m_digital_cnt, sizeof(uint16_t));
	else
		m_digital_buf = NULL;
	return true;
}

int devEK9000::VerifyConnection() const {
	/* asynUsers should be pretty cheap to create */
	asynUser* usr = pasynManager->createAsynUser(NULL, NULL);
	usr->timeout = 0.5; /* 500ms timeout */

	/* Try for connection */
	pasynManager->connectDevice(usr, this->m_octetPortName.data(), 0);
	int yn = 0;
	pasynManager->isConnected(usr, &yn);
	pasynManager->disconnect(usr);

	pasynManager->freeAsynUser(usr);

	return yn;
}

int devEK9000::CoEVerifyConnection(uint16_t termid) {
	uint16_t dat;
	if (this->doCoEIO(0, (uint16_t)termid, 1008, 1, &dat, 0) != EK_EOK) {
		return 0;
	}
	return 1;
}

/* 1 for write and 0 for read */
/* LENGTH IS IN REGISTERS */
int devEK9000::doCoEIO(int rw, uint16_t term, uint16_t index, uint16_t len, uint16_t* data, uint16_t subindex,
					   uint16_t reallen) {
	/* write */
	if (rw) {
		uint16_t tmp_data[512] = {
			1,
			(uint16_t)(term | 0x8000), /* Bit 15 needs to be 1 to indicate a write */
			index,
			subindex,
			reallen ? reallen : (uint16_t)(len * 2),
			0, /* Error code */
		};

		memcpy(tmp_data + 6, data, len * sizeof(uint16_t));
		this->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, 0x1400, tmp_data, len + 7);
		if (!this->Poll(0.005, TIMEOUT_COUNT)) {
			this->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, 0x1400, tmp_data, 6);
			/* Write tmp data */
			if ((tmp_data[0] & 0x400) != 0x400) {
				LastADSErr = tmp_data[5];
				return EK_EADSERR;
			}
		}
		else
			return EK_EERR;
		return EK_EOK;
	}
	/* read */
	else {
		uint16_t tmp_data[512] = {
			1,		  /* 0x1400 = exec */
			term,	  /* 0x1401 = term id */
			index,	  /* 0x1402 = obj */
			subindex, /* 0x1403 = subindex */
			0,		  /* 0x1404 = len = 0 */
			0,		  0, 0, 0,
		};

		/* tell it what to do */
		this->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, 0x1400, tmp_data, 9);

		/* poll */
		if (this->Poll(0.005, TIMEOUT_COUNT)) {
			uint16_t dat = 0;
			this->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, 0x1405, &dat, 1);
			if (dat != 0) {
				data[0] = dat;
				return EK_EADSERR;
			}
			return EK_EERR;
		}
		epicsThreadSleep(0.05);
		/* read result */
		int res = this->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, 0x1406, data, len);
		if (res)
			return EK_EERR;
		return EK_EOK;
	}
}

int devEK9000::doEK9000IO(int rw, uint16_t addr, uint16_t len, uint16_t* data) {
	int status = 0;
	/* write */
	if (rw) {
		status = this->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, addr, data, len);
		if (status) {
			return status + 0x100;
		}
		return EK_EOK;
	}
	/* read */
	else {
		status = this->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, addr, data, len);
		if (status) {
			return status + 0x100;
		}
		return EK_EOK;
	}
	return EK_EBADPARAM;
}

/* Read terminal type */
int devEK9000::ReadTerminalType(uint16_t termid, int& id) {
	UNUSED(termid);
	uint16_t dat = 0;
	this->doEK9000IO(0, 0x6000, 1, &dat);
	id = dat;
	return EK_EOK;
}

/* Reads coupler id */
int devEK9000::ReadCouplerID(char* outbuf, size_t& outbufsize) {
	if (!outbuf || outbufsize < sizeof(uint16_t) * 7 + 1)
		return EK_EBADPARAM;

	/* Coupler id len is 6 words */
	memset(outbuf, 0, sizeof(uint16_t) * 7 + 1);
	uint16_t id[7];
	int status = this->doEK9000IO(0, 0x1008, 7, id);

	if (!status) {
		memcpy(outbuf, id, sizeof(uint16_t) * 7);
		outbufsize = sizeof(uint16_t) * 7 + 1;
		return EK_EOK;
	}
	this->m_error = id[0];
	return status;
}

/* Reads the length of the process image */
int devEK9000::ReadProcessImageSize(uint16_t& anal_out, uint16_t& anal_in, uint16_t& dig_out, uint16_t& dig_in) {
	uint16_t image[4];
	int status = this->doEK9000IO(0, 0x1010, 4, image);
	if (!status) {
		anal_out = image[0];
		anal_in = image[1];
		dig_out = image[2];
		dig_in = image[3];
		return EK_EOK;
	}
	this->m_error = image[0];
	return status;
}

/* Reads the current watchdog time */
int devEK9000::ReadWatchdogTime(uint16_t& out) {
	uint16_t tmp = 0;
	int stat = this->doEK9000IO(0, 0x1020, 1, &tmp);
	if (!stat) {
		out = tmp;
		return EK_EOK;
	}
	this->m_error = tmp;
	return stat;
}

/* Read the number of fallbacks triggered */
int devEK9000::ReadNumFallbacksTriggered(uint16_t& out) {
	uint16_t tmp = 0;
	int stat = this->doEK9000IO(0, 0x1021, 1, &tmp);
	if (!stat) {
		out = tmp;
		return EK_EOK;
	}
	this->m_error = tmp;
	return stat;
}

/* Read the number of tcp connections */
int devEK9000::ReadNumTCPConnections(uint16_t& out) {
	uint16_t tmp = 0;
	int stat = this->doEK9000IO(0, 0x1022, 1, &tmp);
	if (!stat) {
		out = tmp;
		return EK_EOK;
	}
	this->m_error = tmp;
	return stat;
}

/* Read hardware/software versions */
int devEK9000::ReadVersionInfo(uint16_t& hardver, uint16_t& softver_major, uint16_t& softver_minor,
							   uint16_t& softver_patch) {
	uint16_t ver[4];
	int status = this->doEK9000IO(0, 0x1030, 4, ver);
	if (!status) {
		hardver = ver[0];
		softver_major = ver[1];
		softver_minor = ver[2];
		softver_patch = ver[3];
		return EK_EOK;
	}
	this->m_error = ver[0];
	return status;
}

/* Read the serial number */
int devEK9000::ReadSerialNumber(uint16_t& sn) {
	uint16_t tmp = 0;
	int stat = this->doEK9000IO(0, 0x1034, 1, &tmp);
	if (!stat) {
		sn = tmp;
		return EK_EOK;
	}
	this->m_error = tmp;
	return stat;
}

/* Read manfacturing date */
int devEK9000::ReadMfgDate(uint16_t& day, uint16_t& mon, uint16_t& year) {
	uint16_t date[3];
	int status = this->doEK9000IO(0, 0x1035, 3, date);
	if (!status) {
		day = date[0];
		mon = date[1];
		year = date[2];
		return EK_EOK;
	}
	this->m_error = date[0];
	return status;
}

/* Read EBus status */
int devEK9000::ReadEBusStatus(uint16_t& status) {
	uint16_t tmp = 0; // to store tmp data
	int stat = this->doEK9000IO(0, 0x1040, 1, &tmp);
	if (!stat) {
		status = tmp;
		return EK_EOK;
	}
	this->m_error = tmp;
	return stat;
}

/* Write the watcdog time */
int devEK9000::WriteWatchdogTime(uint16_t time) {
	return this->doEK9000IO(1, 0x1120, 1, &time);
}

/* Reset watchdog timer */
int devEK9000::WriteWatchdogReset() {
	uint16_t data = 1;
	// return this->doEK9000IO(1, 0, 1, 0x1121, &data);
	this->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, 0x1121, &data, 1);
	return EK_EOK;
}

/* Write watchdog type */
int devEK9000::WriteWatchdogType(uint16_t type) {
	return this->doEK9000IO(1, 0x1122, 1, &type);
}

/* Write fallback mode */
int devEK9000::WriteFallbackMode(uint16_t mode) {
	return this->doEK9000IO(1, 0x1123, 1, &mode);
}

/* Enable/disable writing to second modbus client */
int devEK9000::WriteWritelockMode(uint16_t mode) {
	return this->doEK9000IO(1, 0x1124, 1, &mode);
}

/* Read terminal ID */
uint16_t devEK9000::ReadTerminalID(uint16_t index) {
	assert(index < 0xFF);
	if (m_readTerminals)
		return m_terminals[index];

	memset(m_terminals, 0, sizeof(m_terminals));
	// Read the terminal register space. 0x6000 will contain 9000, corresponding to the bus coupler.
	// registers thereafter will contain a numeric ID corresponding to the terminal type. i.e. 0x6001 will contain 3064
	// if the second terminal is an EL3064 read 125 registers in a loop, as that's the max number that can be read in a
	// single modbus transaction
	for (int off = 0; off < TERMINAL_REGISTER_COUNT; off += 125) {
		// Check if there are no further terminals on the rail, otherwise continue reading
		if (off > 0 && m_terminals[off] == 0)
			break;
		const size_t toRead = util::clamp(ArraySize(m_terminals) - off, 0, 125);
		if (this->doModbusIO(0, MODBUS_READ_INPUT_REGISTERS, 0x6000, m_terminals + off, toRead) != asynSuccess) {
			LOG_WARNING(this, "%s: Failed to read terminal layout\n", m_name.data());
			break;
		}
	}

	m_readTerminals = true;
	return m_terminals[index];
}

int devEK9000::Poll(float duration, int timeout) {
	uint16_t dat = 0;
	this->doModbusIO(EK9000_SLAVE_ID, MODBUS_READ_HOLDING_REGISTERS, 0x1400, &dat, 1);
	while ((dat | 0x200) == 0x200 && timeout > 0) {
		epicsThreadSleep(duration);
		timeout--;
		this->doModbusIO(EK9000_SLAVE_ID, MODBUS_READ_HOLDING_REGISTERS, 0x1400, &dat, 1);
	}

	return timeout <= 0 ? 1 : 0;
}

int devEK9000::LastError() {
	int tmp = m_error;
	m_error = EK_EOK;
	return tmp;
}

const char* devEK9000::LastErrorString() {
	return ErrorToString(LastError());
}

const char* devEK9000::ErrorToString(int i) {
	switch (i) {
		case EK_EOK:
			return "No error";
		case EK_EERR:
			return "Unspecified error";
		case EK_EBADTERM:
			return "Invalid terminal or slave";
		case EK_ENOCONN:
			return "No connection";
		case EK_EBADPARAM:
			return "Invalid parameter";
		case EK_EBADPTR:
			return "Invalid pointer";
		case EK_ENODEV:
			return "Invalid device";
		case EK_ENOENT:
			return "No entry";
		case EK_EWTCHDG:
			return "Watchdog error";
		case EK_EBADTYP:
			return "Invalid type";
		case EK_EBADIP:
			return "Invalid IP address";
		case EK_EBADPORT:
			return "Invalid port";
		case EK_EADSERR:
			return "ADS error";
		case EK_ETERMIDMIS:
			return "Terminal ID mismatch";
		case EK_EBADMUTEX:
			return "Invalid mutex";
		case EK_EMUTEXTIMEOUT:
			return "Mutex operation timeout";
		case EK_EBADTERMID:
			return "Invalid terminal ID";
		case EK_EMODBUSERR:
			return "Modbus driver error";
		default:
			assert(!"Invalid parameter passed to ErrorToString");
			return "Unknown";
	}
}

//==========================================================//
// IOCsh functions here
//==========================================================//
void ek9000Configure(const iocshArgBuf* args) {
	const char* name = args[0].sval;
	const char* ip = args[1].sval;
	int port = args[2].ival;
	int num = args[3].ival;

	if (!name) {
		epicsPrintf("Invalid name passed.\n");
		return;
	}

	if (!ip) {
		epicsPrintf("Invalid IP passed.\n");
		return;
	}

	// Clamp num to a valid range, we can only have 255 terminals on this device.
	if (num < 0 || num >= 0xFF) {
		epicsPrintf("Invalid terminal count passed.\n");
		return;
	}

	if (port <= 0) {
		epicsPrintf("The port %i is invalid.\n", port);
		return;
	}

	devEK9000* dev;

	char ipbuf[64];
	(void)snprintf(ipbuf, sizeof(ipbuf), "%s:%i", ip, port);

	dev = devEK9000::Create(name, ipbuf, num);

	if (!dev) {
		epicsPrintf("Unable to create device: Unspecified error.\n");
		return;
	}
}

void ek9000ConfigureTerminal(const iocshArgBuf* args) {
	const char* ek = args[0].sval;
	const char* name = args[1].sval;
	const char* type = args[2].sval;
	int id = args[3].ival;

	if (!ek || !name || !type || id < 0) {
		epicsPrintf("Invalid parameter passed!\n");
		return;
	}

	devEK9000* dev = devEK9000::FindDevice(ek);

	/* If we cant find the device :( */
	if (!dev) {
		epicsPrintf("Unable to create terminal \"%s\": Device by the name of \"%s\" not found.\n", name, ek);
		return;
	}

	uint32_t tid = 0;
	for (size_t i = 0; i < ArraySize(s_terminalInfos); i++) {
		if (strcmp(s_terminalInfos[i].str, type) == 0) {
			tid = s_terminalInfos[i].id;
			break;
		}
	}
	if (tid == 0) {
		epicsPrintf("Unable to create terminal %s: No terminal with the ID %s found.\n", name, type);
		return;
	}

	/* Check for out of boundries */
	if (id > dev->m_numTerms) {
		epicsPrintf("Unable to create terminal \"%s\": Terminal index out of range.\n", name);
		return;
	}

	int status = dev->AddTerminal(name, tid, id);

	if (status) {
		epicsPrintf("ek9000ConfigureTerminal(): Failed to create terminal.");
		return;
	}
}

void ek9000Stat(const iocshArgBuf* args) {
	const char* ek9k = args[0].sval;
	if (!ek9k) {
		epicsPrintf("Invalid parameter.\n");
		return;
	}
	devEK9000* dev = devEK9000::FindDevice(ek9k);

	if (!dev) {
		epicsPrintf("Invalid device.\n");
		return;
	}

	DeviceLock lock(dev);
	if (!lock.valid()) {
		LOG_WARNING(dev, "ek9000Stat(): unable to obtain device lock");
		return;
	}

	bool connected = false;
	if (dev->VerifyConnection())
		connected = true;

	uint16_t ao = 0, ai = 0, bo = 0, bi = 0, tcp = 0, sn = 0, wtd = 0;
	uint16_t hver = 0, svermaj = 0, svermin = 0, sverpat = 0;
	uint16_t day = 0, month = 0, year = 0;

	dev->ReadProcessImageSize(ao, ai, bo, bi);
	dev->ReadNumTCPConnections(tcp);
	dev->ReadSerialNumber(sn);
	dev->ReadVersionInfo(hver, svermaj, svermin, sverpat);
	dev->ReadNumFallbacksTriggered(wtd);
	dev->ReadMfgDate(day, month, year);

	epicsPrintf("Device: %s\n", ek9k);
	if (connected)
		epicsPrintf("\tStatus: CONNECTED\n");
	else
		epicsPrintf("\tStatus: NOT CONNECTED\n");
	epicsPrintf("\tIP: %s\n", dev->m_ip.data());
	epicsPrintf("\tAsyn Port Name: %s\n", dev->m_octetPortName.data());
	epicsPrintf("\tAO size: %u [bytes]\n", ao);
	epicsPrintf("\tAI size: %u [bytes]\n", ai);
	epicsPrintf("\tBI size: %u [bits]\n", bi);
	epicsPrintf("\tBO size: %u [bits]\n", bo);
	epicsPrintf("\tTCP connections: %u\n", tcp);
	epicsPrintf("\tSerial number: %u\n", sn);
	epicsPrintf("\tHardware Version: %u\n", hver);
	epicsPrintf("\tSoftware Version: %u.%u.%u\n", svermaj, svermin, sverpat);
	epicsPrintf("\tFallbacks triggered: %u\n", wtd);
	epicsPrintf("\tMfg date: %u/%u/%u\n", month, day, year);

	for (int i = 0; i < dev->m_numTerms; i++) {
		if (dev->m_terms[i]->m_recordName.empty())
			continue;
		epicsPrintf("\tSlave #%i:\n", i + 1);
		epicsPrintf("\t\tType: %u\n", dev->m_terms[i]->m_terminalId);
		epicsPrintf("\t\tRecord Name: %s\n", dev->m_terms[i]->m_recordName.data());
		epicsPrintf("\t\tOutput Size: %u\n", dev->m_terms[i]->m_outputSize);
		epicsPrintf("\t\tOutput Start: %u\n", dev->m_terms[i]->m_outputStart);
		epicsPrintf("\t\tInput Size: %u\n", dev->m_terms[i]->m_inputSize);
		epicsPrintf("\t\tInput Start: %u\n", dev->m_terms[i]->m_inputStart);
	}
}

void ek9000EnableDebug(const iocshArgBuf*) {
	devEK9000::debugEnabled = true;
	epicsPrintf("Debug enabled.\n");
}

void ek9000DisableDebug(const iocshArgBuf*) {
	devEK9000::debugEnabled = false;
	epicsPrintf("Debug disabled.\n");
}

void ek9000List(const iocshArgBuf*) {
	// for (auto dev : GlobalDeviceList()) {
	for (std::list<devEK9000*>::iterator it = GlobalDeviceList().begin(); it != GlobalDeviceList().end(); ++it) {
		devEK9000* dev = *it;
		epicsPrintf("Device: %s\n\tSlave Count: %i\n", dev->m_name.data(), dev->m_numTerms);
		epicsPrintf("\tIP: %s\n", dev->m_ip.data());
		epicsPrintf("\tConnected: %s\n", dev->VerifyConnection() ? "TRUE" : "FALSE");
	}
}

void ek9000SetWatchdogTime(const iocshArgBuf* args) {
	const char* ek9k = args[0].sval;
	int time = args[1].ival;
	if (!ek9k)
		return;
	if (time < 0 || time > 60000)
		return;
	devEK9000* dev = devEK9000::FindDevice(ek9k);
	if (!dev)
		return;
	dev->WriteWatchdogTime((uint16_t)time);
}

void ek9000SetWatchdogType(const iocshArgBuf* args) {
	const char* ek9k = args[0].sval;
	int type = args[1].ival;
	if (!ek9k)
		return;
	if (type < 0 || type > 2) {
		epicsPrintf("2 = disable watchdog\n");
		epicsPrintf("1 = enable on telegram\n");
		epicsPrintf("0 = enable on write\n");
		return;
	}
	devEK9000* dev = devEK9000::FindDevice(ek9k);
	if (!dev)
		return;
	dev->WriteWatchdogType(type);
}

void ek9000SetPollTime(const iocshArgBuf* args) {
	const char* ek9k = args[0].sval;
	int time = args[1].ival;
	if (!ek9k)
		return;
	if (time < 10 || time > 1000)
		return;
	devEK9000* dev = devEK9000::FindDevice(ek9k);
	if (!dev)
		return;
	devEK9000::pollDelay = time;
}

int ek9000RegisterFunctions() {

	/* ek9000SetWatchdogTime(ek9k, time[int]) */
	{
		static const iocshArg arg1 = {"Name", iocshArgString};
		static const iocshArg arg2 = {"Time", iocshArgInt};
		static const iocshArg* const args[] = {&arg1, &arg2};
		static const iocshFuncDef func = {"ek9000SetWatchdogTime", 2, args};
		static const iocshFuncDef func2 = {"ek9kSetWdTime", 2, args};
		iocshRegister(&func, ek9000SetWatchdogTime);
		iocshRegister(&func2, ek9000SetWatchdogTime);
	}

	/* ek9000SetWatchdogType(ek9k, type[int]) */
	{
		static const iocshArg arg1 = {"Name", iocshArgString};
		static const iocshArg arg2 = {"Type", iocshArgInt};
		static const iocshArg* const args[] = {&arg1, &arg2};
		static const iocshFuncDef func = {"ek9000SetWatchdogType", 2, args};
		static const iocshFuncDef func2 = {"ek9kSetWdType", 2, args};
		iocshRegister(&func, ek9000SetWatchdogType);
		iocshRegister(&func2, ek9000SetWatchdogType);
	}

	/* ek9000SetPollTime(ek9k, type[int]) */
	{
		static const iocshArg arg1 = {"Name", iocshArgString};
		static const iocshArg arg2 = {"Type", iocshArgInt};
		static const iocshArg* const args[] = {&arg1, &arg2};
		static const iocshFuncDef func = {"ek9000SetPollTime", 2, args};
		static const iocshFuncDef func2 = {"ek9kSetPollTime", 2, args};
		iocshRegister(&func, ek9000SetPollTime);
		iocshRegister(&func2, ek9000SetPollTime);
	}

	/* ek9000Configure(name, ip, termcount) */
	{
		static const iocshArg arg1 = {"Name", iocshArgString};
		static const iocshArg arg2 = {"IP", iocshArgString};
		static const iocshArg arg3 = {"Port", iocshArgInt};
		static const iocshArg arg4 = {"# of Terminals", iocshArgInt};
		static const iocshArg* const args[] = {&arg1, &arg2, &arg3, &arg4};
		static const iocshFuncDef func = {"ek9000Configure", 4, args};
		static const iocshFuncDef func2 = {"ek9kConfigure", 4, args};
		iocshRegister(&func, ek9000Configure);
		iocshRegister(&func2, ek9000Configure);
	}

	/* ek9000ConfigureTerminal(ek9000, name, type, position) */
	{
		static const iocshArg arg1 = {"EK9000 Name", iocshArgString};
		static const iocshArg arg2 = {"Record Name", iocshArgString};
		static const iocshArg arg3 = {"Type", iocshArgString};
		static const iocshArg arg4 = {"Positon", iocshArgInt};
		static const iocshArg* const args[] = {&arg1, &arg2, &arg3, &arg4};
		static const iocshFuncDef func = {"ek9000ConfigureTerminal", 4, args};
		static const iocshFuncDef func2 = {"ek9kConfigureTerm", 4, args};
		iocshRegister(&func, ek9000ConfigureTerminal);
		iocshRegister(&func2, ek9000ConfigureTerminal);
	}

	/* ek9000Stat */
	{
		static const iocshArg arg1 = {"EK9000 Name", iocshArgString};
		static const iocshArg* const args[] = {&arg1};
		static const iocshFuncDef func = {"ek9000Stat", 1, args};
		static const iocshFuncDef func2 = {"ek9kStat", 1, args};
		iocshRegister(&func, ek9000Stat);
		iocshRegister(&func2, ek9000Stat);
	}

	/* ek9000EnableDebug */
	{
		static const iocshArg arg1 = {"EK9k", iocshArgString};
		static const iocshArg* const args[] = {&arg1};
		static const iocshFuncDef func = {"ek9000EnableDebug", 1, args};
		static const iocshFuncDef func2 = {"ek9kEnableDbg", 1, args};
		iocshRegister(&func, ek9000EnableDebug);
		iocshRegister(&func2, ek9000EnableDebug);
	}

	/* ek9000DisableDebug */
	{
		static const iocshArg arg1 = {"EK9K", iocshArgString};
		static const iocshArg* const args[] = {&arg1};
		static const iocshFuncDef func = {"ek9kDisableDebug", 1, args};
		static const iocshFuncDef func2 = {"ek9kDisableDbg", 1, args};
		iocshRegister(&func, ek9000DisableDebug);
		iocshRegister(&func2, ek9000DisableDebug);
	}

	/* ek9000List */
	{
		static iocshFuncDef func = {"ek9000List", 0, NULL};
		static iocshFuncDef func2 = {"ek9kList", 0, NULL};
		iocshRegister(&func, ek9000List);
		iocshRegister(&func2, ek9000List);
	}

	return 0;
}
epicsExportRegistrar(ek9000RegisterFunctions);

//======================================================//
//
// Fake device support module so that we can just do some
// initialization for the ek9000
//
//======================================================//
static long ek9000_init(int);
static long ek9000_init_record(dbCommon* prec);

bidset devEK9000 = {
	{5, NULL, ek9000_init, ek9000_init_record, NULL},
	NULL,
};

extern "C"
{
	epicsExportAddress(dset, devEK9000);
}

static long ek9000_init_record(dbCommon*) {
	epicsPrintf("FATAL ERROR: You should not use devEK9000 on any records!\n");
	epicsAssert(__FILE__, __LINE__, "FATAL ERROR: You should not use devEK9000 on any records!\n", "Jeremy L.");
	return 0;
}

static long ek9000_init(int after) {
	if (after == 0) {
		epicsPrintf("Initializing EK9000 Couplers.\n");
		// for (auto dev : GlobalDeviceList()) {
		for (std::list<class devEK9000*>::iterator it = GlobalDeviceList().begin(); it != GlobalDeviceList().end();
			 ++it) {
			class devEK9000* dev = (*it);
			if (!dev->m_init && !dev->ComputeTerminalMapping()) {
				epicsPrintf("Unable to compute terminal mapping\n");
				return 1;
			}
		}
		epicsPrintf("Initialization Complete.\n");
		Utl_InitThread();
	}
	return 0;
}

//======================================================//
//
// Device support types for CoE configuration
// 		- Strings for CoE configuration should be like
//			this: @CoE ek9k,terminal,index,subindex
//
//======================================================//
// Common funcs/defs
struct ek9k_coe_param_t {
	class devEK9000* ek9k;
	devEK9000Terminal* pterm;
	int index;
	int subindex;
	enum {
		COE_TYPE_BOOL,
		COE_TYPE_INT8,
		COE_TYPE_INT16,
		COE_TYPE_INT32,
		COE_TYPE_INT64
	} type;
};

struct ek9k_param_t {
	class devEK9000* ek9k;
	int reg;
	int flags;
};

struct ek9k_conf_pvt_t {
	ek9k_coe_param_t param;
};

bool CoE_ParseString(const char* str, ek9k_coe_param_t* param);
//======================================================//

//-----------------------------------------------------------------//
// Configuration CoE RO parameter
static long ek9k_confli_init(int pass);
static long ek9k_confli_init_record(dbCommon* prec);
static long ek9k_confli_read_record(int64inRecord* prec);

int64indset devEK9KCoERO = {
	{5, NULL, ek9k_confli_init, ek9k_confli_init_record, NULL},
	ek9k_confli_read_record,
};

extern "C"
{
	epicsExportAddress(dset, devEK9KCoERO);
}

static long ek9k_confli_init(int) {
	return 0;
}

static long ek9k_confli_init_record(dbCommon* prec) {
	int64inRecord* precord = reinterpret_cast<int64inRecord*>(prec);
	ek9k_coe_param_t param;
	precord->dpvt = calloc(1, sizeof(ek9k_conf_pvt_t));
	ek9k_conf_pvt_t* dpvt = static_cast<ek9k_conf_pvt_t*>(precord->dpvt);

	if (!CoE_ParseString(precord->inp.value.instio.string, &param)) {
		epicsPrintf("ek9k_confli_init_record: Malformed input link string for record %s\n", precord->name);
		return 1;
	}
	dpvt->param = param;
	return 0;
}

static long ek9k_confli_read_record(int64inRecord* precord) {
	ek9k_conf_pvt_t* dpvt = static_cast<ek9k_conf_pvt_t*>(precord->dpvt);

	if (!dpvt || !dpvt->param.ek9k)
		return 1;

	DeviceLock lock(dpvt->param.ek9k);

	int err = EK_EBADPARAM;
	switch (dpvt->param.type) {
		case ek9k_coe_param_t::COE_TYPE_BOOL:
			{
				uint16_t buf = 0;
				err = dpvt->param.ek9k->doCoEIO(0, dpvt->param.pterm->m_terminalIndex, dpvt->param.index, 1, &buf,
												dpvt->param.subindex);
				precord->val = static_cast<epicsInt64>(buf);
				break;
			}
		case ek9k_coe_param_t::COE_TYPE_INT8:
			{
				uint16_t buf = 0;
				err = dpvt->param.ek9k->doCoEIO(0, dpvt->param.pterm->m_terminalIndex, dpvt->param.index, 1, &buf,
												dpvt->param.subindex);
				precord->val = static_cast<epicsInt64>(buf);
				break;
			}
		case ek9k_coe_param_t::COE_TYPE_INT16:
			{
				uint16_t buf = 0;
				err = dpvt->param.ek9k->doCoEIO(0, dpvt->param.pterm->m_terminalIndex, dpvt->param.index, 1, &buf,
												dpvt->param.subindex);
				precord->val = static_cast<epicsInt64>(buf);
				break;
			}
		case ek9k_coe_param_t::COE_TYPE_INT32:
			{
				uint32_t buf = 0;
				err = dpvt->param.ek9k->doCoEIO(0, dpvt->param.pterm->m_terminalIndex, dpvt->param.index, 2,
												reinterpret_cast<uint16_t*>(&buf), dpvt->param.subindex);
				precord->val = static_cast<epicsInt64>(buf);
				break;
			}
		case ek9k_coe_param_t::COE_TYPE_INT64:
			{
				uint64_t buf = 0;
				err = dpvt->param.ek9k->doCoEIO(0, dpvt->param.pterm->m_terminalIndex, dpvt->param.index, 4,
												reinterpret_cast<uint16_t*>(&buf), dpvt->param.subindex);
				precord->val = static_cast<epicsInt64>(buf);
				break;
			}
		default: // Really should not get here
			assert(0);
			break;
	}
	if (err != EK_EOK) {
		recGblSetSevr(precord, COMM_ALARM, INVALID_ALARM);
		return 1;
	}
	return 0;
}

//-----------------------------------------------------------------//

//-----------------------------------------------------------------//
// Configuration RW CoE parameter
static long ek9k_conflo_init(int pass);
static long ek9k_conflo_init_record(dbCommon* prec);
static long ek9k_conflo_write_record(int64outRecord* prec);

struct devEK9KCoERW_t {
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record; /*returns: (-1,0)=>(failure,success)*/
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN write_longout; /*(-1,0)=>(failure,success*/
};
int64outdset devEK9KCoERW = {
	{5, NULL, ek9k_conflo_init, ek9k_conflo_init_record, NULL},
	ek9k_conflo_write_record,
};

extern "C"
{
	epicsExportAddress(dset, devEK9KCoERW);
}

static long ek9k_conflo_init(int) {
	return 0;
}

static long ek9k_conflo_init_record(dbCommon* prec) {
	int64outRecord* precord = reinterpret_cast<int64outRecord*>(prec);
	ek9k_coe_param_t param;
	precord->dpvt = calloc(1, sizeof(ek9k_conf_pvt_t));
	ek9k_conf_pvt_t* dpvt = static_cast<ek9k_conf_pvt_t*>(precord->dpvt);

	if (!CoE_ParseString(precord->out.value.instio.string, &param)) {
		epicsPrintf("ek9k_conflo_init_record: Malformed input link string for record %s\n", precord->name);
		return 1;
	}
	dpvt->param = param;
	return 0;
}

static long ek9k_conflo_write_record(int64outRecord* precord) {
	ek9k_conf_pvt_t* dpvt = static_cast<ek9k_conf_pvt_t*>(precord->dpvt);
	int ret = EK_EOK;

	if (!dpvt || !dpvt->param.ek9k)
		return 1;

	DeviceLock lock(dpvt->param.ek9k);

	switch (dpvt->param.type) {
		case ek9k_coe_param_t::COE_TYPE_BOOL:
			{
				uint16_t buf = static_cast<epicsInt16>(precord->val);
				ret = dpvt->param.ek9k->doCoEIO(1, dpvt->param.pterm->m_terminalIndex, dpvt->param.index, 1, &buf,
												dpvt->param.subindex, 1);
				break;
			}
		case ek9k_coe_param_t::COE_TYPE_INT8:
			{
				uint16_t buf = static_cast<epicsInt16>(precord->val);
				ret = dpvt->param.ek9k->doCoEIO(1, dpvt->param.pterm->m_terminalIndex, dpvt->param.index, 1, &buf,
												dpvt->param.subindex, 1);
				break;
			}
		case ek9k_coe_param_t::COE_TYPE_INT16:
			{
				uint16_t buf = static_cast<epicsInt16>(precord->val);
				ret = dpvt->param.ek9k->doCoEIO(1, dpvt->param.pterm->m_terminalIndex, dpvt->param.index, 1, &buf,
												dpvt->param.subindex, 2);
				break;
			}
		case ek9k_coe_param_t::COE_TYPE_INT32:
			{
				uint32_t buf = static_cast<epicsInt32>(precord->val);
				ret = dpvt->param.ek9k->doCoEIO(1, dpvt->param.pterm->m_terminalIndex, dpvt->param.index, 2,
												reinterpret_cast<uint16_t*>(&buf), dpvt->param.subindex, 4);
				break;
			}
		case ek9k_coe_param_t::COE_TYPE_INT64:
			{
				uint64_t buf = precord->val;
				ret = dpvt->param.ek9k->doCoEIO(1, dpvt->param.pterm->m_terminalIndex, dpvt->param.index, 4,
												reinterpret_cast<uint16_t*>(&buf), dpvt->param.subindex, 8);
				break;
			}
		default:
			break;
	}

	if (ret != EK_EOK) {
		epicsPrintf("ek9k_conflo_write_record(): Error writing data to record.\n");
		recGblSetSevr(precord, COMM_ALARM, INVALID_ALARM);
	}

	return 0;
}

//-----------------------------------------------------------------//

bool CoE_ParseString(const char* str, ek9k_coe_param_t* param) {
	class devEK9000* pcoupler = 0;
	devEK9000Terminal* pterm = 0;
	int termid;
	size_t bufcnt = 0;
	char buf[512];
	char* buffers[5];
	memset(buffers, 0, sizeof(buffers));

	if (str[0] == 0)
		return false;

	memset(buf, 0, sizeof(buf));
	strncpy(buf, str, sizeof(buf) - 1);

	/* Tokenize & separate into substrings */
	for (char* tok = strtok((char*)str, ","); tok; tok = strtok(NULL, ",")) {
		buffers[bufcnt] = tok;
		++bufcnt;
	}

	if (!buffers[0])
		return 1;

	const size_t strl = strlen(str);
	for (size_t i = 0; i < strl; i++)
		if (buf[i] == ',')
			buf[i] = 0;

	/* Finally actually parse the integers, find the ek9k, etc. */
	// for (auto dev : GlobalDeviceList()) {
	for (std::list<class devEK9000*>::iterator it = GlobalDeviceList().begin(); it != GlobalDeviceList().end(); ++it) {
		class devEK9000* dev = *it;
		if (strncmp(dev->m_name.data(), buffers[0], dev->m_name.size()) == 0) {
			pcoupler = dev;
			break;
		}
	}

	if (!pcoupler) {
		epicsPrintf("Coupler not found.\n");
		return false;
	}

	if (bufcnt < 4)
		return false;

	/* Determine the CoE type */
	if (epicsStrnCaseCmp(buffers[4], "bool", 4) == 0)
		param->type = ek9k_coe_param_t::COE_TYPE_BOOL;
	else if (epicsStrnCaseCmp(buffers[4], "int16", 5) == 0 || epicsStrnCaseCmp(buffers[4], "uint16", 6) == 0)
		param->type = ek9k_coe_param_t::COE_TYPE_INT16;
	else if (epicsStrnCaseCmp(buffers[4], "int32", 5) == 0 || epicsStrnCaseCmp(buffers[4], "uint32", 6) == 0)
		param->type = ek9k_coe_param_t::COE_TYPE_INT32;
	else if (epicsStrnCaseCmp(buffers[4], "int64", 5) == 0 || epicsStrnCaseCmp(buffers[4], "uint64", 6) == 0)
		param->type = ek9k_coe_param_t::COE_TYPE_INT64;
	else if (epicsStrnCaseCmp(buffers[4], "int8", 4) == 0 || epicsStrnCaseCmp(buffers[4], "uint8", 5) == 0)
		param->type = ek9k_coe_param_t::COE_TYPE_INT8;
	else
		return false;

	if (!util::parseNumber(buffers[1], termid, 10))
		return 1;
	pterm = pcoupler->m_terms[termid - 1];

	param->pterm = pterm;
	param->ek9k = pcoupler;
	if (!util::parseNumber(buffers[2], param->index, 16))
		return 1;

	if (!util::parseNumber(buffers[3], param->subindex, 16))
		return 1;

	return true;
}

//-----------------------------------------------------------------//
// EK9K RO configuration/status
static long ek9k_status_init(int pass);
template <class T>
static long ek9k_status_init_record(dbCommon* prec); // Common init function for status/config records
static long ek9k_status_read_record(longinRecord* prec);
static long ek9k_status_write_record(longoutRecord* prec);
static long ek9k_status_get_ioint_info(int cmd, dbCommon* prec, IOSCANPVT* iopvt);

static bool ek9k_parse_string(const char* linkValue, ek9k_param_t& outParam);

enum {
	STATUS_RD = 0x1,
	STATUS_WR = 0x2,
	STATUS_RW = STATUS_RD | STATUS_WR,
	STATUS_STATIC = 0x4, /* These registers will never change during runtime, only need to read these once */
};
typedef int StatusFlags;

struct StatusReg {
	const char* configName;
	int addr;
	StatusFlags flags;
};

CONSTEXPR StatusReg status_regs[] = {{"analogOutputs", 0x1010, STATUS_RD | STATUS_STATIC},
									 {"analogInputs", 0x1011, STATUS_RD | STATUS_STATIC},
									 {"digitalOutputs", 0x1012, STATUS_RD | STATUS_STATIC},
									 {"digitalInputs", 0x1013, STATUS_RD | STATUS_STATIC},
									 {"fallbacks", 0x1021, STATUS_RD},
									 {"tcpConnections", 0x1022, STATUS_RD},
									 {"hardwareVer", 0x1030, STATUS_RD | STATUS_STATIC},
									 {"softVerMain", 0x1031, STATUS_RD | STATUS_STATIC},
									 {"softVerSub", 0x1032, STATUS_RD | STATUS_STATIC},
									 {"softVerBeta", 0x1033, STATUS_RD | STATUS_STATIC},
									 {"serialNum", 0x1034, STATUS_RD | STATUS_STATIC},
									 {"prodDay", 0x1035, STATUS_RD | STATUS_STATIC},
									 {"prodMonth", 0x1036, STATUS_RD | STATUS_STATIC},
									 {"prodYear", 0x1037, STATUS_RD | STATUS_STATIC},
									 {"ebusStatus", 0x1040, STATUS_RD},
									 {"wdtTime", 0x1120, STATUS_RW},
									 {"wdtReset", 0x1121, STATUS_RW},
									 {"wdtType", 0x1122, STATUS_RW},
									 {"wdtFallback", 0x1123, STATUS_RW},
									 {"writelock", 0x1124, STATUS_RW},
									 {"ebusMode", 0x1140, STATUS_RW}};

// Read-only status info (tcp connections, fallbacks triggered, etc.)
longindset devEK9000ConfigRO = {
	{
		5,
		NULL,
		ek9k_status_init,
		ek9k_status_init_record<longinRecord>,
		ek9k_status_get_ioint_info,
	},
	ek9k_status_read_record,
};

extern "C"
{
	epicsExportAddress(dset, devEK9000ConfigRO);
}

// Read-write parameters (watchdog type, fallback mode, etc.)
longoutdset devEK9000ConfigRW = {
	{
		5,
		NULL,
		ek9k_status_init,
		ek9k_status_init_record<longoutRecord>,
		ek9k_status_get_ioint_info,
	},
	ek9k_status_write_record,
};

extern "C"
{
	epicsExportAddress(dset, devEK9000ConfigRW);
}

static long ek9k_status_init(int) {
	return 0;
}

static const char* link_value(longinRecord* rec) {
	return rec->inp.value.instio.string;
}

static const char* link_value(longoutRecord* rec) {
	return rec->out.value.instio.string;
}

template <class T> static long ek9k_status_init_record(dbCommon* prec) {
	T* precord = reinterpret_cast<T*>(prec);
	precord->dpvt = calloc(1, sizeof(ek9k_param_t));
	ek9k_param_t* dpvt = static_cast<ek9k_param_t*>(precord->dpvt);
	ek9k_param_t param;

	/* parse the string to obtain various info */
	if (!ek9k_parse_string(link_value(precord), param)) {
		epicsPrintf("Malformed modbus string in record %s\n", precord->name);
		return 1;
	}
	*dpvt = param;
	return 0;
}

static long ek9k_status_write_record(longoutRecord* precord) {
	ek9k_param_t* dpvt = static_cast<ek9k_param_t*>(precord->dpvt);
	class devEK9000* dev = dpvt->ek9k;

	if (!dev)
		return 1;

	DeviceLock lock(dpvt->ek9k);
	uint16_t buf = precord->val;
	if (dev->doEK9000IO(1, dpvt->reg, 1, &buf) != EK_EOK) {
		recGblSetSevr(precord, COMM_ALARM, INVALID_ALARM);
		return 1;
	}
	return 0;
}

static long ek9k_status_read_record(longinRecord* precord) {
	ek9k_param_t* dpvt = static_cast<ek9k_param_t*>(precord->dpvt);
	class devEK9000* dev = dpvt->ek9k;
	uint16_t buf;

	if (!dev)
		return 1;

	if (dpvt->flags & STATUS_STATIC) {
		if (dev->doEK9000IO(0, dpvt->reg, 1, &buf) != EK_EOK) {
			recGblSetSevr(precord, COMM_ALARM, INVALID_ALARM);
			return 1;
		}
	}
	else if (dev->getEK9000IO(READ_STATUS, dpvt->reg, &buf, 1) != EK_EOK) {
		recGblSetSevr(precord, COMM_ALARM, INVALID_ALARM);
		return 1;
	}
	precord->val = buf;
	return 0;
}

static long ek9k_status_get_ioint_info(int cmd, dbCommon* prec, IOSCANPVT* iopvt) {
	ek9k_param_t* param = static_cast<ek9k_param_t*>(prec->dpvt);
	if (!param->ek9k)
		return 1;

	// Static parameters only need to be updated once on init, skip any updates later down the road.
	if (param->flags & STATUS_STATIC)
		return 0;

	*iopvt = param->ek9k->m_status_io;
	return 0;
}

//-----------------------------------------------------------------//
/* The string will be in the format EK9K,0x1002 */
bool ek9k_parse_string(const char* str, ek9k_param_t& param) {

	LinkSpec_t spec;
	if (!util::ParseLinkSpecification(str, INST_IO, spec))
		return false;

	param.reg = 0;
	param.flags = 0;

	for (size_t i = 0; i < spec.size(); ++i) {
		if (spec[i].first == "device") {
			param.ek9k = devEK9000::FindDevice(spec[i].second.c_str());
			if (!param.ek9k) {
				epicsPrintf("Unable to find device '%s' specified in instio string '%s'\n", spec[i].second.c_str(),
							str);
				return false;
			}
		}
		else if (spec[i].first == "type") {

			// Find param
			for (size_t s = 0; s < ArraySize(status_regs); ++s) {
				if (!strcmp(spec[i].second.c_str(), status_regs[s].configName)) {
					param.reg = status_regs[s].addr;
					param.flags = status_regs[s].flags;
					break;
				}
			}
			if (param.reg == 0) {
				epicsPrintf("Malformed instio string '%s', does not specify register\n", spec[i].second.c_str());
				return false;
			}
		}
		else if (spec[i].first == "addr") {
			if (!util::parseNumber(spec[i].second.c_str(), param.reg, 16)) {
				epicsStdoutPrintf("Malformed integer '%s' in instio string for key 'addr'\n", spec[i].second.c_str());
				return false;
			}
		}
		else if (spec[i].first == "flags") {
			for (size_t n = 0; n < spec[i].second.length(); ++n) {
				char c = spec[i].second[n];
				if (c == 'r')
					param.flags |= STATUS_RD;
				else if (c == 'w')
					param.flags |= STATUS_WR;
				else if (c == 's')
					param.flags |= STATUS_STATIC;
				else {
					epicsPrintf("Unknown status flag '%c' in instio string '%s' for key 'flags'\n", c,
								spec[i].second.c_str());
					return false;
				}
			}
		}
		else {
			epicsPrintf("Extraneous key '%s' in instio string '%s'\n", spec[i].first.c_str(), str);
			return false;
		}
	}

	return true;
}
