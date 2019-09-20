#include "flash.h"
#include "spi.h"
#include "gpio.h"
#include "delay.h"
#include "debug.h"
#include "pad_mux_ctrl.h"
#include <string.h>

void flash_spi_enable(bool en)
{
	if(en){
	  pad_mux_write(SPI_CSN_1,  0);
		gpo_config(SPI_CSN_1,0);
	}
	
	spim_enable(en);
	
	if(!en){
		  gpo_set(SPI_CSN_1);
	    pad_mux_write(SPI_CSN_1,  3);
	}
}

/*******************************************************************************
* Function Name  : SPI_Flash_ReadID
* Description    : ��ȡSPI_FLASH��BY2516��ID��
* Input          : None
* Output         : None
* Return         : Temp��������ID��
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ������ dbg_printf("FlashID:%x\r\n",SPI_Flash_ReadID());
*******************************************************************************/
unsigned long int SPI_Flash_ReadID(void)
{
	unsigned long int Temp = 0;	  	
	__align(4) uint8_t rx[4]={BY25_JedecDeviceID,0xFF,0xFF,0xFF};
	uint8_t size = 4;
	spim_enable(1);//CS = 0
	spim_exchange(size, rx, rx);
	spim_enable(0);//CS = 1
	Temp=rx[1]<<16 | rx[2]<<8 | rx[3];
	return Temp;
}  
/*******************************************************************************
* Function Name  : SPI_Flash_ReadSR
* Description    : SPI_flash��ȡ״̬����
* Input          : None
* Output         : None
* Return         : byte��������״̬�Ĵ���
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ������ SPI_FLASH_Write_Enable();
*******************************************************************************/
unsigned char SPI_Flash_ReadSR(void)   
{  
	unsigned char Temp = 0;	  	
	__align(4) uint8_t rx[4]={0}, tx[4]={BY25_ReadStatusReg,0xFF,0xFF,0xFF};
	spim_enable(1);//CS = 0
	spim_exchange(2, rx, tx);
	spim_enable(0);//CS = 1
	Temp=rx[1];
	return Temp;
} 
/*******************************************************************************
* Function Name  : SPI_Flash_Wait_Busy
* Description    : �ȴ�SPI_flash���к���
* Input          : None
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ������ SPI_Flash_Wait_Busy();
*******************************************************************************/
void SPI_Flash_Wait_Busy(void)   
{   
	while ((SPI_Flash_ReadSR()&0x01)==0x01);   // �ȴ�BUSYλ���
}
/*******************************************************************************
* Function Name  : SPI_FLASH_Write_Enable
* Description    : SPI_flashдʹ�ܺ���
* Input          : None
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ������ SPI_FLASH_Write_Enable();
*******************************************************************************/
void SPI_FLASH_Write_Enable(void)   
{	
	__align(4) uint8_t rx[4]={0}, tx[4]={BY25_WriteEnable,0xFF,0xFF,0xFF};
	spim_enable(1);//CS = 0
	spim_exchange(1, rx, tx);
	spim_enable(0);//CS = 1                          //ȡ��Ƭѡ     	      
} 
/*******************************************************************************
* Function Name  : SPI_Flash_Erase_Sector
* Description    : ����SPI��������
* Input          : Dst_Addr������������ַ
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ������SPI_Flash_Erase_Sector(100);
*******************************************************************************/
void SPI_Flash_Erase_Sector(unsigned long int Dst_Addr)   
{   
	unsigned long int Temp = 0;	  	
	__align(4) uint8_t rx[4]={0}, tx[4]={BY25_SectorErase,0xFF,0xFF,0xFF};
  SPI_FLASH_Write_Enable();                  //SET WEL 	 
	SPI_Flash_Wait_Busy();   
	tx[1]=(unsigned char)((Dst_Addr)>>16);
	tx[2]=(unsigned char)((Dst_Addr)>>8);
	tx[3]=(unsigned char)Dst_Addr;
	spim_enable(1);//CS = 0
	spim_exchange(4, rx, tx);
	spim_enable(0);//CS = 1
	SPI_Flash_Wait_Busy();   				   //�ȴ��������
}  
/*******************************************************************************
* Function Name  : SPI_Flash_Read
* Description    : ��BY2516������
* Input          : pBuffer��Ҫ����������
                   WriteAddr��Ҫ�����ĵ�ַ
                   NumByteToWrite��Ҫ������������ ���Ϊ4096
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ������ SPI_Flash_Read(RX_BUFF,wordaddr,32);
*******************************************************************************/
void SPI_Flash_Read(unsigned char* pBuffer,unsigned long int ReadAddr,unsigned short int NumByteToRead)   
{ 
 	unsigned short int i=0;    	
	unsigned long int secpos=0;
	unsigned short int secoff=0;
	__align(4) uint8_t rx[32]={0}, tx[32]={0};
	secpos=NumByteToRead/32;//������ַ 0~511 for BY2516
	secoff=NumByteToRead%32;//�������ڵ�ƫ��
	flash_spi_enable(1);             //ʹ������   
	tx[0]=BY25_ReadData;
	tx[1]=(unsigned char)((ReadAddr)>>16);
	tx[2]=(unsigned char)((ReadAddr)>>8);
	tx[3]=(unsigned char)ReadAddr;
	spim_exchange(4, rx, tx);
	memset(tx,0xff,32);
	for(i=0;i<secpos;i++)
	{ 
		spim_exchange(32, rx, tx);	
		memcpy(pBuffer+i*32,rx,32);   //ѭ������
	}
	spim_exchange(secoff, rx, tx);
	memcpy(&pBuffer[i*32],rx,secoff);   
	flash_spi_enable(0);             //ȡ��Ƭѡ     	      
}  
/*******************************************************************************
* Function Name  : SPI_Flash_Write_NoCheck
* Description    : ��BY2516дҳ����
* Input          : pBuffer��Ҫд�������
                   WriteAddr��Ҫд��ĵ�ַ
                   NumByteToWrite��Ҫд�������� ���Ϊ4096
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ������ SPI_Flash_Write_Pag(SPI_FLASH_BUF,secpos*4096,4096);
*******************************************************************************/
void SPI_Flash_Write_Page(unsigned char* pBuffer,unsigned long int WriteAddr,unsigned short int NumByteToWrite)
{
// 	unsigned short int i;  
//  SPI_FLASH_Write_Enable();                  //SET WEL 
//	SPI_FLASH_CS_against=1;                            //ʹ������   
//	SPIx_ReadWriteByte(BY25_PageProgram);      //����дҳ����   
//	SPIx_ReadWriteByte((unsigned char)((WriteAddr)>>16)); //����24bit��ַ    
//	SPIx_ReadWriteByte((unsigned char)((WriteAddr)>>8));   
//	SPIx_ReadWriteByte((unsigned char)WriteAddr);   
//	for(i=0;i<NumByteToWrite;i++)SPIx_ReadWriteByte(pBuffer[i]);//ѭ��д��  
//	SPI_FLASH_CS_against=0;                            //ȡ��Ƭѡ 
//	SPI_Flash_Wait_Busy();					   //�ȴ�д�����
	 	
 	unsigned short int i=0;    	
	unsigned long int secpos=0;
	unsigned short int secoff=0;
	__align(4) uint8_t rx[32]={0}, tx[32]={0};
	secpos=NumByteToWrite/32;//������ַ 0~511 for BY2516
	secoff=NumByteToWrite%32;//�������ڵ�ƫ��
	SPI_FLASH_Write_Enable();                  //SET WEL 
	flash_spi_enable(1);             //ʹ������   
	tx[0]=BY25_PageProgram;
	tx[1]=(unsigned char)((WriteAddr)>>16);
	tx[2]=(unsigned char)((WriteAddr)>>8);
	tx[3]=(unsigned char)WriteAddr;
	spim_exchange(4, rx, tx);
	for(i=0;i<secpos;i++)
	{ 
		memcpy(tx,&pBuffer[i*32],32);  
		spim_exchange(32, rx, tx);	
	}
	memcpy(tx,&pBuffer[i*32],secoff);  
	spim_exchange(secoff, rx, tx);
	 
	flash_spi_enable(0);             //ȡ��Ƭѡ    
	SPI_Flash_Wait_Busy();					   //�ȴ�д�����
} 
/*******************************************************************************
* Function Name  : SPI_Flash_Write_NoCheck
* Description    : ��BY2516д��������
* Input          : pBuffer��Ҫд�������
                   WriteAddr��Ҫд��ĵ�ַ
                   NumByteToWrite��Ҫд�������� ���Ϊ4096
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ������ SPI_Flash_Write_NoCheck(SPI_FLASH_BUF,secpos*4096,4096);
*******************************************************************************/
void SPI_Flash_Write_NoCheck(unsigned char* pBuffer,unsigned long int WriteAddr,unsigned short int NumByteToWrite)   
{ 			 		 
	unsigned short int pageremain;	   
	pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
	while(1)
	{	   
		SPI_Flash_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//д�������
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=NumByteToWrite; 	  //����256���ֽ���
		}
	};	    
} 

/*******************************************************************************
* Function Name  : SPI_Flash_Write
* Description    : ��BY2516д����
* Input          : pBuffer��Ҫд�������
                   WriteAddr��Ҫд��ĵ�ַ
                   NumByteToWrite��Ҫд�������� ���Ϊ4096
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ������ SPI_Flash_Write((u8*)codeGB_16[k].Msk,wordaddr,32);
*******************************************************************************/
unsigned char SPI_FLASH_BUF[4096];  //�ݴ�ԭ�����ݵļĴ��� �Ա��ڲ���Ҫ��������ʧ
void SPI_Flash_Write(unsigned char* pBuffer,unsigned long int WriteAddr,unsigned short int NumByteToWrite)   
{ 
	unsigned long int secpos;
	unsigned short int secoff;
	unsigned short int secremain;	   
 	unsigned short int i;  
	secpos=WriteAddr/4096;//������ַ 0~511 for BY2516
	secoff=WriteAddr%4096;//�������ڵ�ƫ��
	secremain=4096-secoff;//����ʣ��ռ��С   
	if(NumByteToWrite<=secremain) secremain=NumByteToWrite;//������4096���ֽ�
	while(1) 
	{	
		SPI_Flash_Read(SPI_FLASH_BUF,secpos*4096,4096);//������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(SPI_FLASH_BUF[secoff+i]!=0XFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//ԭ�������� ��Ҫ����
		{
			SPI_Flash_Erase_Sector(secpos);//�����������
			for(i=0;i<secremain;i++)	   //����
			{
				SPI_FLASH_BUF[i+secoff]=pBuffer[i];	    //�����pBuffer[i];�Ƕ����������� �����i�Ǹղ������ݵ���������׵�ַ�ĳ���
			}
			SPI_Flash_Write_NoCheck(SPI_FLASH_BUF,secpos*4096,4096);//д����������  
		}else SPI_Flash_Write_NoCheck(pBuffer,WriteAddr,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(NumByteToWrite==secremain)break;//д�������
		else//д��δ����
		{
			secpos++;//������ַ��1
			secoff=0;//ƫ��λ��Ϊ0 	 
			pBuffer+=secremain;  //ָ��ƫ��
			WriteAddr+=secremain;//д��ַƫ��	   
			NumByteToWrite-=secremain;				//�ֽ����ݼ�
			if(NumByteToWrite>4096)secremain=4096;	//��һ����������д����
			else secremain=NumByteToWrite;			//��һ����������д����
		}	 
	};	 	 
}
