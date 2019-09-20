#include "otaV30.h"
#include "ble_slave.h"
#include "debug.h"
#include <string.h>
#include "delay.h"
#include "DebugLog.h"
#ifdef _OTA_

uint8_t ota_state=0;
uint8_t ota_writecnt=0;
uint16_t ota_section_size=0,ota_receive_size=0,ota_section_check=0,ota_receive_check=0;
uint32_t ota_section_offset=0;
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

void CmdFwErase(void)
{
	struct ret_fw_erase_cmd	Ret;

//	DBGPRINTF(("CmdFwErase\r\n"));

	if(ota_code_erase())
	{
		Ret.status = ERR_COMMAND_SUCCESS;
	}
	else
	{
		Ret.status = ERR_COMMAND_FAILED;
	}
	

	Ret.status = ERR_COMMAND_SUCCESS;
	
	EvtCommandComplete(CMD_FW_ERASE, (uint8_t *)&Ret, sizeof (Ret));
}

static void CmdFwWrite(struct cmd_fw_write *p_cmd)
{
	struct ret_fw_erase_cmd	Ret;
	uint8_t i = 0;

	DBGPRINTF(("CmdFwWrite\r\n"));
	
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
				Ret.status = ERR_COMMAND_FAILED;
			}
			ota_w.idx = 0;
			ota_w.cnt++;
		}
	}
	if(((int)ota_w.buf %4) !=0)  DBGPRINTF(("CmdFwWrite error\r\n"));

	Ret.status = ERR_COMMAND_SUCCESS;
	
	EvtCommandComplete(CMD_FW_WRITE, (uint8_t *)&Ret, sizeof (Ret));
}

static void CmdFwWriteStart(uint8_t status,uint16_t sz,uint16_t checksum)
{
	struct ret_fw_write_start_cmd	Ret;

	Ret.sz=sz;
	Ret.checksum=checksum;
	Ret.status=status;
	
	EvtCommandComplete(CMD_FW_WRITE_START, (uint8_t *)&Ret, sizeof (Ret));
}

static void CmdFwUpgrade(struct cmd_fw_upgrade *p_cmd)
{
	struct ret_fw_erase_cmd	Ret;
	uint8_t temp = 0;

	if(ota_w.idx != 0)
		ota_code_write(ota_w.cnt*32, ota_w.idx, ota_w.buf);
	
	#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTTCONFIG_UART_ENABLE)
	DBGPRINTF(("ota sz:%x checksum:%x ",p_cmd->sz,p_cmd->checksum));
	#endif
	temp = ota_code_update(NULL, NULL, p_cmd->sz, p_cmd->checksum);
	#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTTCONFIG_UART_ENABLE)
	DBGPRINTF(("CodeUpdate=%d\r\n", temp));
	#endif
	
	if(temp)
	{
		Ret.status = ERR_COMMAND_SUCCESS;
	}
	else
	{
		Ret.status = ERR_COMMAND_FAILED;
	}

	Ret.status = ERR_COMMAND_SUCCESS;
	
	EvtCommandComplete(CMD_FW_UPGRADE, (uint8_t *)&Ret, sizeof (Ret));

}

static void CmdFwUpgradev20(struct cmd_fw_upgrade_V20 *p_cmd)
{
	struct ret_fw_erase_cmd	Ret;
	uint8_t temp = 0;

	if(ota_w.idx != 0)
		ota_code_write(ota_w.cnt*32, ota_w.idx, ota_w.buf);
	
	#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTTCONFIG_UART_ENABLE)
	DBGPRINTF(("ota sz:%x checksum:%x ",p_cmd->sz,p_cmd->checksum));
	#endif
	temp = ota_code_update(NULL, NULL, p_cmd->sz, p_cmd->checksum);
	#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTTCONFIG_UART_ENABLE)
	DBGPRINTF(("CodeUpdatev20=%d\r\n", temp));
	#endif

	if(temp==0)
		Ret.status = ERR_COMMAND_FAILED;
	else 
		Ret.status = ERR_COMMAND_SUCCESS;
	
	EvtCommandComplete(CMD_FW_UPGRADE, (uint8_t *)&Ret, sizeof (Ret));
}

void ota_cmd(uint8_t *p_cmd, uint8_t sz)
{
	struct hci_cmd *pcmd = (struct hci_cmd*)p_cmd;
	if((ota_section_size==0) || (ota_state==0))
	{
		switch(pcmd->opcode)
		{	
			case CMD_FW_ERASE:
				BLSetConnectionUpdate(0);  //ota
				#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTTCONFIG_UART_ENABLE)
					dbg_printf(("CMD_FW_ERASE\r\n"));
				#endif
				CmdFwErase();
			    ota_state=1;
				ota_section_offset=0;
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
			case CMD_FW_UPGRADEV20:
				CmdFwUpgradev20(&pcmd->cmdparm.CmdFwUpgradeV20);
				ota_state=3;
				break;
			case CMD_FW_WRITE_START:
				ota_section_size=pcmd->cmdparm.CmdFwWriteStart.sz;
				ota_section_check=pcmd->cmdparm.CmdFwWriteStart.checksum;
				ota_section_offset=pcmd->cmdparm.CmdFwWriteStart.offset;
				ota_receive_size=0;
				ota_receive_check=0;
				#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTTCONFIG_UART_ENABLE)
				dbg_printf("CMD_FW_WRITE_START offset:%x,size:%x checksum:%x\r\n",pcmd->cmdparm.CmdFwWriteStart.offset,ota_section_size,pcmd->cmdparm.CmdFwWriteStart.checksum);
				#endif
				break;
		}
	}
	else
	{
		uint16_t idx;
		ota_writecnt =0;
		
		for(idx=0;idx < sz ; idx++)
			ota_receive_check += p_cmd[idx];
		
		if(((int)p_cmd % 4)!=0)
		{
			memcpy(ota_w.buf,p_cmd,sz);
			ota_code_write(ota_section_offset+ota_receive_size, sz, ota_w.buf);
//			#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTTCONFIG_UART_ENABLE)
//			dbg_printf("CodeWrite noalign  ");
//			#endif
		}
		else
		{
			ota_code_write(ota_section_offset+ota_receive_size, sz, p_cmd);
//			#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTTCONFIG_UART_ENABLE)
//			dbg_printf("CodeWrite align  ");
//			#endif
		}
		
//		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTTCONFIG_UART_ENABLE)
//		dbg_printf("section_offset:%x receive_size:%x section_size:%x\r\n",ota_section_offset,ota_receive_size,ota_section_size);
//		#endif

		ota_receive_size +=sz;
		if(ota_receive_size>=ota_section_size)
		{
			if(ota_receive_check==ota_section_check)
			{
				CmdFwWriteStart(ERR_COMMAND_SUCCESS,ota_receive_size,ota_receive_check);
				#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTTCONFIG_UART_ENABLE)
				dbg_printf("section OK! ");
				#endif
			}
			else
			{
				CmdFwWriteStart(ERR_COMMAND_FAILED,ota_receive_size,ota_receive_check);
				#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTTCONFIG_UART_ENABLE)
				dbg_printf("section faile! ");
				#endif
			}
			#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTTCONFIG_UART_ENABLE)
			dbg_printf("ota_receive_check:%x ota_section_check:%x\r\n",ota_receive_check,ota_section_check);
			#endif
			
			ota_variable_clear(false);
		}
	}
}

void ota_variable_clear(uint8_t all)
{
	if(all)
	{
		ota_state=0;
		ota_writecnt=0;
	}
	ota_section_size=0;
	ota_section_check=0;
	ota_receive_size=0;
	ota_receive_check=0;
	ota_section_offset=0;
}

void ota_rsp(uint8_t *p_rsp, uint8_t *p_sz)
{
	memcpy(p_rsp, EvtBuf,  pevt->evtlen + 2);

	*p_sz = pevt->evtlen + 2;		
	
	memset(EvtBuf,0,sizeof(EvtBuf));
}
#endif

