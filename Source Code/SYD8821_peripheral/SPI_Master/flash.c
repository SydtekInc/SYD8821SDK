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
* Description    : 读取SPI_FLASH即BY2516的ID号
* Input          : None
* Output         : None
* Return         : Temp：器件的ID号
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ：例子 dbg_printf("FlashID:%x\r\n",SPI_Flash_ReadID());
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
* Description    : SPI_flash读取状态函数
* Input          : None
* Output         : None
* Return         : byte：器件的状态寄存器
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ：例子 SPI_FLASH_Write_Enable();
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
* Description    : 等待SPI_flash空闲函数
* Input          : None
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ：例子 SPI_Flash_Wait_Busy();
*******************************************************************************/
void SPI_Flash_Wait_Busy(void)   
{   
	while ((SPI_Flash_ReadSR()&0x01)==0x01);   // 等待BUSY位清空
}
/*******************************************************************************
* Function Name  : SPI_FLASH_Write_Enable
* Description    : SPI_flash写使能函数
* Input          : None
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ：例子 SPI_FLASH_Write_Enable();
*******************************************************************************/
void SPI_FLASH_Write_Enable(void)   
{	
	__align(4) uint8_t rx[4]={0}, tx[4]={BY25_WriteEnable,0xFF,0xFF,0xFF};
	spim_enable(1);//CS = 0
	spim_exchange(1, rx, tx);
	spim_enable(0);//CS = 1                          //取消片选     	      
} 
/*******************************************************************************
* Function Name  : SPI_Flash_Erase_Sector
* Description    : 擦除SPI扇区函数
* Input          : Dst_Addr：数据扇区地址
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ：例子SPI_Flash_Erase_Sector(100);
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
	SPI_Flash_Wait_Busy();   				   //等待擦除完成
}  
/*******************************************************************************
* Function Name  : SPI_Flash_Read
* Description    : 向BY2516读函数
* Input          : pBuffer：要读出的数组
                   WriteAddr：要读出的地址
                   NumByteToWrite：要读出多大的数据 最大为4096
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ：例子 SPI_Flash_Read(RX_BUFF,wordaddr,32);
*******************************************************************************/
void SPI_Flash_Read(unsigned char* pBuffer,unsigned long int ReadAddr,unsigned short int NumByteToRead)   
{ 
 	unsigned short int i=0;    	
	unsigned long int secpos=0;
	unsigned short int secoff=0;
	__align(4) uint8_t rx[32]={0}, tx[32]={0};
	secpos=NumByteToRead/32;//扇区地址 0~511 for BY2516
	secoff=NumByteToRead%32;//在扇区内的偏移
	flash_spi_enable(1);             //使能器件   
	tx[0]=BY25_ReadData;
	tx[1]=(unsigned char)((ReadAddr)>>16);
	tx[2]=(unsigned char)((ReadAddr)>>8);
	tx[3]=(unsigned char)ReadAddr;
	spim_exchange(4, rx, tx);
	memset(tx,0xff,32);
	for(i=0;i<secpos;i++)
	{ 
		spim_exchange(32, rx, tx);	
		memcpy(pBuffer+i*32,rx,32);   //循环读数
	}
	spim_exchange(secoff, rx, tx);
	memcpy(&pBuffer[i*32],rx,secoff);   
	flash_spi_enable(0);             //取消片选     	      
}  
/*******************************************************************************
* Function Name  : SPI_Flash_Write_NoCheck
* Description    : 向BY2516写页函数
* Input          : pBuffer：要写入的数组
                   WriteAddr：要写入的地址
                   NumByteToWrite：要写多大的数据 最大为4096
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ：例子 SPI_Flash_Write_Pag(SPI_FLASH_BUF,secpos*4096,4096);
*******************************************************************************/
void SPI_Flash_Write_Page(unsigned char* pBuffer,unsigned long int WriteAddr,unsigned short int NumByteToWrite)
{
// 	unsigned short int i;  
//  SPI_FLASH_Write_Enable();                  //SET WEL 
//	SPI_FLASH_CS_against=1;                            //使能器件   
//	SPIx_ReadWriteByte(BY25_PageProgram);      //发送写页命令   
//	SPIx_ReadWriteByte((unsigned char)((WriteAddr)>>16)); //发送24bit地址    
//	SPIx_ReadWriteByte((unsigned char)((WriteAddr)>>8));   
//	SPIx_ReadWriteByte((unsigned char)WriteAddr);   
//	for(i=0;i<NumByteToWrite;i++)SPIx_ReadWriteByte(pBuffer[i]);//循环写数  
//	SPI_FLASH_CS_against=0;                            //取消片选 
//	SPI_Flash_Wait_Busy();					   //等待写入结束
	 	
 	unsigned short int i=0;    	
	unsigned long int secpos=0;
	unsigned short int secoff=0;
	__align(4) uint8_t rx[32]={0}, tx[32]={0};
	secpos=NumByteToWrite/32;//扇区地址 0~511 for BY2516
	secoff=NumByteToWrite%32;//在扇区内的偏移
	SPI_FLASH_Write_Enable();                  //SET WEL 
	flash_spi_enable(1);             //使能器件   
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
	 
	flash_spi_enable(0);             //取消片选    
	SPI_Flash_Wait_Busy();					   //等待写入结束
} 
/*******************************************************************************
* Function Name  : SPI_Flash_Write_NoCheck
* Description    : 向BY2516写块区函数
* Input          : pBuffer：要写入的数组
                   WriteAddr：要写入的地址
                   NumByteToWrite：要写多大的数据 最大为4096
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ：例子 SPI_Flash_Write_NoCheck(SPI_FLASH_BUF,secpos*4096,4096);
*******************************************************************************/
void SPI_Flash_Write_NoCheck(unsigned char* pBuffer,unsigned long int WriteAddr,unsigned short int NumByteToWrite)   
{ 			 		 
	unsigned short int pageremain;	   
	pageremain=256-WriteAddr%256; //单页剩余的字节数		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
	while(1)
	{	   
		SPI_Flash_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
			else pageremain=NumByteToWrite; 	  //不够256个字节了
		}
	};	    
} 

/*******************************************************************************
* Function Name  : SPI_Flash_Write
* Description    : 向BY2516写函数
* Input          : pBuffer：要写入的数组
                   WriteAddr：要写入的地址
                   NumByteToWrite：要写多大的数据 最大为4096
* Output         : None
* Return         : None
* Data           : 2014/8/24
* programmer     : piaoran
* remark         ：例子 SPI_Flash_Write((u8*)codeGB_16[k].Msk,wordaddr,32);
*******************************************************************************/
unsigned char SPI_FLASH_BUF[4096];  //暂存原来数据的寄存器 以便于不必要的数据损失
void SPI_Flash_Write(unsigned char* pBuffer,unsigned long int WriteAddr,unsigned short int NumByteToWrite)   
{ 
	unsigned long int secpos;
	unsigned short int secoff;
	unsigned short int secremain;	   
 	unsigned short int i;  
	secpos=WriteAddr/4096;//扇区地址 0~511 for BY2516
	secoff=WriteAddr%4096;//在扇区内的偏移
	secremain=4096-secoff;//扇区剩余空间大小   
	if(NumByteToWrite<=secremain) secremain=NumByteToWrite;//不大于4096个字节
	while(1) 
	{	
		SPI_Flash_Read(SPI_FLASH_BUF,secpos*4096,4096);//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
			if(SPI_FLASH_BUF[secoff+i]!=0XFF)break;//需要擦除  	  
		}
		if(i<secremain)//原来有数据 需要擦除
		{
			SPI_Flash_Erase_Sector(secpos);//擦除这个扇区
			for(i=0;i<secremain;i++)	   //复制
			{
				SPI_FLASH_BUF[i+secoff]=pBuffer[i];	    //这里的pBuffer[i];是读出来的数据 这里的i是刚才有数据到这个扇区首地址的长度
			}
			SPI_Flash_Write_NoCheck(SPI_FLASH_BUF,secpos*4096,4096);//写入整个扇区  
		}else SPI_Flash_Write_NoCheck(pBuffer,WriteAddr,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
		if(NumByteToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;//扇区地址增1
			secoff=0;//偏移位置为0 	 
			pBuffer+=secremain;  //指针偏移
			WriteAddr+=secremain;//写地址偏移	   
			NumByteToWrite-=secremain;				//字节数递减
			if(NumByteToWrite>4096)secremain=4096;	//下一个扇区还是写不完
			else secremain=NumByteToWrite;			//下一个扇区可以写完了
		}	 
	};	 	 
}
