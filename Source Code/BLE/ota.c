#include "ota.h"
#include "debug.h"
#include <string.h>
#include "delay.h"
#include "ble_slave.h"

#ifndef CONFIG_UART_ENABLE
#include "Debuglog.h"
#endif

#ifdef _OTA_

uint8_t ota_state=0;
uint8_t ota_writecnt=0;

extern void BLSetConnectionUpdate(uint8_t target);

static uint8_t EvtBuf[sizeof(struct hci_evt)];
struct hci_evt *pevt = (struct hci_evt *)EvtBuf;
__align(4) struct ota_write_info ota_w;

static void EvtCommandComplete(uint8_t opcode, uint8_t *ret_parm, uint8_t ret_len)
{
	pevt->evtcode = EVT_COMMAND_COMPLETE;
	pevt->evtlen = ret_len + CMD_COMPLETE_HDR_SZ;
	
	pevt->evtparm.EvtCommandComplete.opcode= opcode;

	memcpy((uint8_t *)&pevt->evtparm.EvtCommandComplete.RetParm, ret_parm, ret_len);
}

static void CmdFwErase()
{
	struct ret_fw_erase_cmd	Ret;

	DBGPRINTF(("CmdFwErase\r\n"));

	if(ota_code_erase())
	{
		Ret.status = ERR_COMMAND_SUCCESS;
	}
	else
	{
		Ret.status = 0x01;
	}
	
	EvtCommandComplete(CMD_FW_ERASE, (uint8_t *)&Ret, sizeof (Ret));
}

static void CmdFwWrite(struct cmd_fw_write *p_cmd)
{
	struct ret_fw_erase_cmd	Ret;
	uint8_t i = 0;

	for(i = 0; i < p_cmd->sz; i++)
	{
		ota_w.buf[ota_w.idx++] = p_cmd->data[i];
		if(ota_w.idx >= 32)
		{
			if(ota_code_write(ota_w.cnt*32, 32, ota_w.buf))
			{
				Ret.status = ERR_COMMAND_SUCCESS;
			}
			else
			{
				Ret.status = 0x01;
			}
			ota_w.idx = 0;
			ota_w.cnt++;
		}
		else
		{
			Ret.status = ERR_COMMAND_SUCCESS;
		}
	}
	//if(((int)ota_w.buf %4) !=0)  DBGPRINTF(("CmdFwWrite error\r\n"));

	EvtCommandComplete(CMD_FW_WRITE, (uint8_t *)&Ret, sizeof (Ret));
}

static void CmdFwUpgrade(struct cmd_fw_upgrade *p_cmd)
{
	struct ret_fw_erase_cmd	Ret;

	if(ota_w.idx != 0)
		ota_code_write(ota_w.cnt*32, ota_w.idx, ota_w.buf);
	
	#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
//		DBGPRINTF(("ota sz:%x checksum:%x \r\n",p_cmd->sz,p_cmd->checksum));
	#endif
	
	if(ota_code_update(NULL, NULL, p_cmd->sz, p_cmd->checksum))
	{
		Ret.status = ERR_COMMAND_SUCCESS;
	}
	else
	{
		Ret.status = 0x01;
	}

	EvtCommandComplete(CMD_FW_UPGRADE, (uint8_t *)&Ret, sizeof (Ret));
}

void ota_cmd(uint8_t *p_cmd, uint8_t sz)
{
	struct hci_cmd *pcmd = (struct hci_cmd*)p_cmd;
	switch(pcmd->opcode)
	{	
		case CMD_FW_ERASE:
			BLSetConnectionUpdate(0);  //ota
			CmdFwErase();
			ota_state=1;
			break;
		case CMD_FW_WRITE:
			CmdFwWrite(&pcmd->cmdparm.CmdFwWrite);
			if(!ota_state) ota_state=2;
			ota_writecnt =0;
			break;
		case CMD_FW_UPGRADE:
			CmdFwUpgrade(&pcmd->cmdparm.CmdFwUpgrade);
			ota_state=3;
			break;
	}
}

void ota_rsp(uint8_t *p_rsp, uint8_t *p_sz)
{
	memcpy(p_rsp, EvtBuf,  pevt->evtlen + 2);

	*p_sz = pevt->evtlen + 2;		
}
#endif

