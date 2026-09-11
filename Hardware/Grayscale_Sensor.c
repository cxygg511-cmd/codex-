#if 0  /* === TRACKING CODE DISABLED === */
#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "Grayscale_Sensor.h"

/*------------------------I2C---------------------------*/

uint8_t GRAY_I2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
	Delay_us(GRAY_DELAY_TIME);
	return BitValue;
}

void GRAY_I2C_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB,GPIO_Pin_15,(BitAction)(BitValue));
    Delay_us(GRAY_DELAY_TIME);

}				

void GRAY_I2C_W_SDA(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB,GPIO_Pin_14,(BitAction)(BitValue));
    Delay_us(GRAY_DELAY_TIME);

}	

void GRAY_I2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//开启APB2时钟
	
	//初始化Pin14,Pin15
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;        //开漏输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	GPIO_SetBits(GPIOB,GPIO_Pin_15);

}

void GRAY_I2C_Start(void)
{
	GRAY_I2C_W_SDA(1);
	GRAY_I2C_W_SCL(1);
	GRAY_I2C_W_SDA(0);
	GRAY_I2C_W_SCL(0);
}

void GRAY_I2C_Stop(void)
{
	GRAY_I2C_W_SDA(0);
	GRAY_I2C_W_SCL(1);
	GRAY_I2C_W_SDA(1);
}

void GRAY_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		GRAY_I2C_W_SDA(Byte & (0x80 >> i));
		GRAY_I2C_W_SCL(1);
		GRAY_I2C_W_SCL(0);
	}
}

uint8_t GRAY_I2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;
	GRAY_I2C_W_SDA(1);
	for (i = 0; i < 8; i ++)
	{
		GRAY_I2C_W_SCL(1);
		if (GRAY_I2C_R_SDA() == 1){Byte |= (0x80 >> i);}
		GRAY_I2C_W_SCL(0);
	}
	return Byte;
}

void GRAY_I2C_SendAck(uint8_t AckBit)
{
	GRAY_I2C_W_SDA(AckBit);
	GRAY_I2C_W_SCL(1);
	GRAY_I2C_W_SCL(0);
}

uint8_t GRAY_I2C_ReceiveAck(void)
{
	uint8_t AckBit;
	GRAY_I2C_W_SDA(1);
	GRAY_I2C_W_SCL(1);
	AckBit = GRAY_I2C_R_SDA();
	GRAY_I2C_W_SCL(0);
	return AckBit;
}

/*------------------------灰度传感器---------------------------*/

//初始化
void GRAY_Init(void)
{
	GRAY_I2C_Init();

}

//写寄存器函数
void GRAY_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	GRAY_I2C_Start();
	GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS);
	GRAY_I2C_ReceiveAck();
	GRAY_I2C_SendByte(RegAddress);
	GRAY_I2C_ReceiveAck();
	GRAY_I2C_SendByte(Data);
	GRAY_I2C_ReceiveAck();
	GRAY_I2C_Stop();
}

//读数字量函数
uint8_t GRAY_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	GRAY_I2C_Start();
	GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS);
	GRAY_I2C_ReceiveAck();
	GRAY_I2C_SendByte(RegAddress);
	GRAY_I2C_ReceiveAck();
	
	GRAY_I2C_Start();
	GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS | 0x01);
	GRAY_I2C_ReceiveAck();
	Data = GRAY_I2C_ReceiveByte();
	GRAY_I2C_SendAck(1);
	GRAY_I2C_Stop();
	
	return Data;
}

//只读数字量函数-需要在初始化时调用一次读数字量函数，并在之后没有调用其他函数写入命令
uint8_t GRAY_Only_ReadReg(void)
{
	uint8_t Data;
	
	GRAY_I2C_Start();
	GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS | 0x01);
	GRAY_I2C_ReceiveAck();
	Data = GRAY_I2C_ReceiveByte();
	GRAY_I2C_SendAck(1);
	GRAY_I2C_Stop();
	
	return Data;
}

//地址扫描函数
uint8_t GRAY_ADDR_Scan(void)
{
    	uint8_t Data,i;
	for( i=0;i<255;i++)
    {
        GRAY_I2C_Start();
        GRAY_I2C_SendByte(i<<1);
        GRAY_I2C_ReceiveAck();
        GRAY_I2C_SendByte(0xAA);
        GRAY_I2C_ReceiveAck();
        
        GRAY_I2C_Start();
        GRAY_I2C_SendByte((i<<1) | 0x01);
        GRAY_I2C_ReceiveAck();
        Data = GRAY_I2C_ReceiveByte();
        GRAY_I2C_SendAck(1);
        GRAY_I2C_Stop();
        if(Data==0x66)
            break;
    }


    return i<<1;
}

//等待连接完成函数
//State_Flag:1为OK，2为超时状态
uint8_t GRAY_Ping(void)
{
    uint8_t Data=0,State_Flag=0;
    uint32_t cnt=0;
    while(1)
    {
        GRAY_I2C_Start();
        GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS);
        GRAY_I2C_ReceiveAck();
        GRAY_I2C_SendByte(0xAA);
        GRAY_I2C_ReceiveAck();
        
        GRAY_I2C_Start();
        GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS | 0x01);
        GRAY_I2C_ReceiveAck();
        Data = GRAY_I2C_ReceiveByte();
        GRAY_I2C_SendAck(1);
        GRAY_I2C_Stop();
        cnt++;
        if(cnt>10000)
        {
          State_Flag=2;
            break;
        }
        if(Data==0x66)
				{
          State_Flag=1;
            break;
				}

    }
    return State_Flag;
}
/*--------------------单模拟量读取--------------------*/
//Channel:1-8
void GRAY_Single_Analog_ReadReg(uint8_t Channel,uint8_t *Data)
{
	
	GRAY_I2C_Start();
	GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS);
	GRAY_I2C_ReceiveAck();
	GRAY_I2C_SendByte(0xB0|Channel);
	GRAY_I2C_ReceiveAck();
	
	GRAY_I2C_Start();
	GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS | 0x01);
	GRAY_I2C_ReceiveAck();

	*Data = GRAY_I2C_ReceiveByte();
	GRAY_I2C_SendAck(1);		

	GRAY_I2C_Stop();	
	
}
/*--------------------连续模拟量读取--------------------*/
//读模拟量函数
void GRAY_Analog_ReadReg(uint8_t *Data)
{
	uint8_t i;
	
	GRAY_I2C_Start();
	GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS);
	GRAY_I2C_ReceiveAck();
	GRAY_I2C_SendByte(0xB0);
	GRAY_I2C_ReceiveAck();
	
	GRAY_I2C_Start();
	GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS | 0x01);
	GRAY_I2C_ReceiveAck();
	for(i=0;i<8;i++)
	{
		Data[i] = GRAY_I2C_ReceiveByte();
		GRAY_I2C_SendAck(0);		
	}

	GRAY_I2C_Stop();
	
}

//只读模拟量函数-需要在初始化时调用一次读模拟量函数，并在之后没有调用其他函数写入命令
void GRAY_Only_Analog_ReadReg(uint8_t *Data)
{
	uint8_t i;
	
	GRAY_I2C_Start();
	GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS | 0x01);
	GRAY_I2C_ReceiveAck();
	for(i=0;i<8;i++)
	{
		Data[i] = GRAY_I2C_ReceiveByte();
		GRAY_I2C_SendAck(0);		
	}

	GRAY_I2C_Stop();
}

//固件版本查询函数
void Check_Firmware_Version(uint8_t *Version)
{
	GRAY_I2C_Start();
	GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS);
	GRAY_I2C_ReceiveAck();
	GRAY_I2C_SendByte(0xC1);
	GRAY_I2C_ReceiveAck();
	
	GRAY_I2C_Start();
	GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS | 0x01);
	GRAY_I2C_ReceiveAck();
	*Version = GRAY_I2C_ReceiveByte();
	GRAY_I2C_SendAck(1);		

	GRAY_I2C_Stop();	
	
	
}
//传输通道使能
uint8_t Transmission_Channel(uint8_t R_OR_W,uint8_t Data)
{
	uint8_t Argument;
	GRAY_I2C_Start();
	GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS);
	GRAY_I2C_ReceiveAck();
	GRAY_I2C_SendByte(0xCE);
	GRAY_I2C_ReceiveAck();
	
	if(R_OR_W==0)//0,读取
	{
		GRAY_I2C_Start();
		GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS | 0x01);
		GRAY_I2C_ReceiveAck();
		Argument = GRAY_I2C_ReceiveByte();
		GRAY_I2C_SendAck(1);			
	}
	else//1写入
	{
		GRAY_I2C_SendByte(Data);
		GRAY_I2C_ReceiveAck();		
	}
	

	GRAY_I2C_Stop();		
	
	return Argument;	
	
}





//通道数据归一化
//R_OR_W:0为读取，1为写入
//要使能的数据
uint8_t Channel_Data_Normalization(uint8_t R_OR_W,uint8_t Data)
{
	uint8_t Argument;
	GRAY_I2C_Start();
	GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS);
	GRAY_I2C_ReceiveAck();
	GRAY_I2C_SendByte(0xCF);
	GRAY_I2C_ReceiveAck();
	
	if(R_OR_W==0)//0,读取
	{
		GRAY_I2C_Start();
		GRAY_I2C_SendByte(GRAYSCALE_SENSOR_ADDRESS | 0x01);
		GRAY_I2C_ReceiveAck();
		Argument = GRAY_I2C_ReceiveByte();
		GRAY_I2C_SendAck(1);			
	}
	else//1写入
	{
		GRAY_I2C_SendByte(Data);
		GRAY_I2C_ReceiveAck();		
	}
	

	GRAY_I2C_Stop();		
	
	return Argument;
}


#endif  /* === END TRACKING === */
