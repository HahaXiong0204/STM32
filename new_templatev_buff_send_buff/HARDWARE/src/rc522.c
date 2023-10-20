#include "rc522.h"
#include "stdio.h"

//SPI2_SCK 				PB13
//SPI2_MISO				PB14
//SPI2_MOSI 			PB15
//RCC522_RST(CE)        PB9
//RCC522_NSS(SDA��      PB8
//RCC522_IRQ 			����


uint8_t UID[5], Temp[4];
uint8_t RF_Buffer[18];
uint8_t Password_Buffer[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Mifare One ȱʡ����

void RC522_Init(void)
{
    RC522_SDA_LOW;
    HAL_SPI_Transmit(&hspi2, (uint8_t *)0xaa, sizeof((uint8_t *)0xaa), 0xFF);//��������
    RC522_SDA_HIGH;

    HAL_Delay(50);
    PcdReset();//��λRC522������
    HAL_Delay(10);
    PcdReset();//��λRC522������
    HAL_Delay(10);
    PcdAntennaOff();//�ر����߷���
    HAL_Delay(10);
    PcdAntennaOn();//�������߷���
    printf("RFID-MFRC522 ��ʼ�����\r\nFindCard Starting ...\r\n");  //�������ų�ʼ�����
}


void delay_ns(uint32_t ns)
{
    uint32_t i;
    for(i = 0; i < ns; i++)
    {
        __nop();
        __nop();
        __nop();
    }
}



//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
//***************************************************************************/

char RC522_ReadWriteByte(uint8_t TxData)
{
    while (SPI_CHECK_FLAG(SPI2->SR, ((uint16_t)0x0002)) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
    {
        ;
    }
    SPI2->DR = TxData; 																//ͨ������SPIx����һ������
    while (SPI_CHECK_FLAG(SPI2->SR, ((uint16_t)0x0001)) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
    {
    }
    return SPI2->DR; 															//����ͨ��SPIx������յ�����
}

//******************************************************************/
//��    �ܣ���RC522�Ĵ���
//����˵����Address[IN]:�Ĵ�����ַ
//��    �أ�������ֵ
//******************************************************************/
unsigned char ReadRawRC(unsigned char Address)
{
    uint8_t ucAddr;
    uint8_t ucResult = 0;
    RC522_SDA_LOW;
    HAL_Delay(10);
    ucAddr = ((Address << 1) & 0x7E) | 0x80;
    RC522_ReadWriteByte(ucAddr);
    HAL_Delay(10);
    ucResult = RC522_ReadWriteByte(0);
    HAL_Delay(10);
    RC522_SDA_HIGH;
    return ucResult;
}

//******************************************************************/
//��    �ܣ�дRC522�Ĵ���
//����˵����Address[IN]:�Ĵ�����ַ
//          value[IN]:д���ֵ
//******************************************************************/
void WriteRawRC(unsigned char Address, unsigned char value)
{
    uint8_t ucAddr;
    RC522_SDA_LOW;
    HAL_Delay(10);
    ucAddr = ((Address << 1) & 0x7E) ;
    RC522_ReadWriteByte(ucAddr);
    HAL_Delay(10);
    RC522_ReadWriteByte(value);
    HAL_Delay(10);
    RC522_SDA_HIGH;
}

//******************************************************************/
//��    �ܣ���RC522�Ĵ���λ
//����˵����reg[IN]:�Ĵ�����ַ
//          mask[IN]:��λֵ
//******************************************************************/
void SetBitMask(unsigned char reg, unsigned char mask)
{
    char tmp = 0x0            ;
    tmp = ReadRawRC(reg) | mask;
    WriteRawRC(reg, tmp | mask); // set bit mask
}

//******************************************************************/
//��    �ܣ���RC522�Ĵ���λ
//����˵����reg[IN]:�Ĵ�����ַ
//          mask[IN]:��λֵ
//******************************************************************/
void ClearBitMask(unsigned char reg, unsigned char mask)
{
    char tmp = 0x0              ;
    tmp = ReadRawRC(reg) & (~mask);
    WriteRawRC(reg, tmp)        ;  // clear bit mask
}

//******************************************************************/
//��    �ܣ���λRC522
//��    ��: �ɹ�����MI_OK
//******************************************************************/
char PcdReset()
{
    RC522_RST_HIGH;                             ;
    delay_ns(10)                             ;
    RC522_RST_LOW;                             ;
    delay_ns(100)                             ;
    RC522_RST_HIGH;                             ;
    delay_ns(10)                           ;
    WriteRawRC(CommandReg, PCD_RESETPHASE);
    delay_ns(100)                             ;
    WriteRawRC(ModeReg, 0x3D)             ; //���巢�ͺͽ��ճ���ģʽ ��Mifare��ͨѶ��CRC��ʼֵ0x6363
    WriteRawRC(TReloadRegL, 30)           ; //16λ��ʱ����λ 30
    WriteRawRC(TReloadRegH, 0)            ; //16λ��ʱ����λ
    WriteRawRC(TModeReg, 0x8D)            ; //�����ڲ���ʱ��������
    WriteRawRC(TPrescalerReg, 0x3E)       ; //���ö�ʱ����Ƶϵ��
    WriteRawRC(TxASKReg, 0x40)            ; //���Ʒ����ź�Ϊ100%ASK
    return MI_OK                         ;
}
//
//����RC522�Ĺ�����ʽ
//
char MF522PcdConfigISOType(unsigned char  type)
{
    if (type == 'A')	//ISO14443_A
    {
        ClearBitMask(Status2Reg, 0x08);	//״̬2�Ĵ���
        WriteRawRC(ModeReg, 0x3D);	//3F  //��Mifare��ͨѶ��CRC��ʼֵ0x6363
        WriteRawRC(RxSelReg, 0x86);	//84   ѡ���ڲ����������ã��ڲ�ģ�ⲿ�ֵ����źţ��������ݺ��ӳ�6��λʱ�ӣ�����
        WriteRawRC(RFCfgReg, 0x7F);	//4F  ���ý�����  48dB�������
        WriteRawRC(TReloadRegL, 30);	//tmoLength);TReloadVal = 'h6a =tmoLength(dec)
        WriteRawRC(TReloadRegH, 0);	//ʵ��ֵ��OXD3E �ⲿ����Ҫ�����ö�ʱ���Ĵ���
        WriteRawRC(TModeReg, 0x8D);
        WriteRawRC(TPrescalerReg, 0x3E);
        delay_ns(1000);
        PcdAntennaOn();
    }
    else
    {
        return 0xFE;
    }
    return MI_OK;
}

//******************************************************************/
//�������߷���
//ÿ��������ر����շ���֮��Ӧ������1ms�ļ��
//******************************************************************/
void PcdAntennaOn()
{
    unsigned char i;
    WriteRawRC(TxASKReg, 0x40)       ;
    HAL_Delay(1);
    i = ReadRawRC(TxControlReg)     ;
    if(!(i & 0x03))
        SetBitMask(TxControlReg, 0x03);
    i = ReadRawRC(TxASKReg)       ;
}


//******************************************************************/
//�ر����߷���
//******************************************************************/
void PcdAntennaOff()
{
    ClearBitMask(TxControlReg, 0x03);
}

//******************************************************************/
//��    �ܣ�ͨ��RC522��ISO14443��ͨѶ
//����˵����Command[IN]:RC522������
//          pInData[IN]:ͨ��RC522���͵���Ƭ������
//          InLenByte[IN]:�������ݵ��ֽڳ���
//          pOutData[OUT]:���յ��Ŀ�Ƭ��������
//          *pOutLenBit[OUT]:�������ݵ�λ����
//******************************************************************/
char PcdComMF522(unsigned char Command  , unsigned char *pInData ,
                 unsigned char InLenByte, unsigned char *pOutData,
                 unsigned int  *pOutLenBit                       )
{
    char status = MI_ERR                          ;
    unsigned char irqEn   = 0x00                  ;
    unsigned char waitFor = 0x00                  ;
    unsigned char lastBits                        ;
    unsigned char n                               ;
    unsigned int  i                               ;
    switch (Command)
    {
    case PCD_AUTHENT:
        irqEn   = 0x12                            ;
        waitFor = 0x10                            ;
        break                                     ;
    case PCD_TRANSCEIVE:
        irqEn   = 0x77                            ;
        waitFor = 0x30                            ;
        break                                     ;
    default:
        break                                     ;
    }
    WriteRawRC(ComIEnReg, irqEn | 0x80)              ; //
    ClearBitMask(ComIrqReg, 0x80)                  ;
    WriteRawRC(CommandReg, PCD_IDLE)               ;
    SetBitMask(FIFOLevelReg, 0x80)                 ; // ���FIFO
    for(i = 0; i < InLenByte; i++)
        WriteRawRC(FIFODataReg, pInData[i])          ; // ����д��FIFO
    WriteRawRC(CommandReg, Command)               ; // ����д������Ĵ���
    if(Command == PCD_TRANSCEIVE)
        SetBitMask(BitFramingReg, 0x80)              ; // ��ʼ����
    i = 6000                                      ; //����ʱ��Ƶ�ʵ���������M1�����ȴ�ʱ��25ms
    do
    {
        n = ReadRawRC(ComIrqReg)                    ;
        i--                                         ;
    }
    while((i != 0) && !(n & 0x01) && !(n & waitFor))        ;
    ClearBitMask(BitFramingReg, 0x80)              ;
    if(i != 0)
    {
        if(!(ReadRawRC(ErrorReg) & 0x1B))
        {
            status = MI_OK                            ;
            if (n & irqEn & 0x01)
                status = MI_NOTAGERR                    ;
            if(Command == PCD_TRANSCEIVE)
            {
                n = ReadRawRC(FIFOLevelReg)             ;
                lastBits = ReadRawRC(ControlReg) & 0x07   ;
                if(lastBits)
                    *pOutLenBit = (n - 1) * 8 + lastBits      ;
                else
                    *pOutLenBit = n * 8                     ;
                if(n == 0)
                    n = 1                                 ;
                if(n > MAXRLEN)
                    n = MAXRLEN                           ;
                for (i = 0; i < n; i++)
                    pOutData[i] = ReadRawRC(FIFODataReg)  ;
            }
        }
        else
            status = MI_ERR                           ;
    }
    SetBitMask(ControlReg, 0x80)                   ; // stop timer now
    WriteRawRC(CommandReg, PCD_IDLE)               ;
    return status;
}

//******************************************************************/
//��    �ܣ�Ѱ��                                                    /
//����˵��: req_code[IN]:Ѱ����ʽ                                   /
//                0x52 = Ѱ��Ӧ�������з���14443A��׼�Ŀ�           /
//                0x26 = Ѱδ��������״̬�Ŀ�                       /
//                pTagType[OUT]����Ƭ���ʹ���                       /
//                0x4400 = Mifare_UltraLight                        /
//                0x0400 = Mifare_One(S50)                          /
//                0x0200 = Mifare_One(S70)                          /
//                0x0800 = Mifare_Pro(X)                            /
//                0x4403 = Mifare_DESFire                           /
//��    ��: �ɹ�����MI_OK                                           /
//******************************************************************/
char PcdRequest(unsigned char req_code, unsigned char *pTagType)
{
    char status                                        ;
    unsigned int  unLen                                ;
    unsigned char ucComMF522Buf[MAXRLEN]               ;

    ClearBitMask(Status2Reg, 0x08)                      ; //���MRCrypto1on��Ҫ����������
    WriteRawRC(BitFramingReg, 0x07)                     ; //startsend=0,rxalign=0,��FIFO�д�ŵ�λ�ã�TXlastbit=7
    SetBitMask(TxControlReg, 0x03)                      ; //TX2rfen=1,TX1RFen=1,���ݵ��Ƶ�13.56MHZ���ز��ź�

    ucComMF522Buf[0] = req_code                        ;

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &unLen       );
    if ((status == MI_OK) && (unLen == 0x10))
    {
        *pTagType     = ucComMF522Buf[0]                 ;
        *(pTagType + 1) = ucComMF522Buf[1]                 ;
    }
    else
        status = MI_ERR                                  ;
    return status                                      ;
}

//******************************************************************/
//��    �ܣ�����ײ                                                  /
//����˵��: pSnr[OUT]:��Ƭ���кţ�4�ֽ�                             /
//��    ��: �ɹ�����MI_OK                                           /
//******************************************************************/
char PcdAnticoll(unsigned char *pSnr)
{
    char status;
    unsigned char i, snr_check = 0;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ClearBitMask(Status2Reg, 0x08); //���MRCrypto1on��Ҫ����������
    WriteRawRC(BitFramingReg, 0x00); //��ʾ���һ���ֽ�����λ������
    ClearBitMask(CollReg, 0x80); //CollRegCollReg0��ͻ�������ͻλ������

    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x20;

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &unLen);

    if (status == MI_OK)
    {
        for (i = 0; i < 4; i++)
        {
            *(pSnr + i)  = ucComMF522Buf[i];
            snr_check ^= ucComMF522Buf[i];
        }
        if (snr_check != ucComMF522Buf[i])
        {
            status = MI_ERR;
        }
    }

    SetBitMask(CollReg, 0x80); //CollRegCollReg ��106kbps���õķ���ͻ����¸�λ��1
    return status;
}



//==============================================================================
//��ȡ��������
//��ȡ����ID��
//==============================================================================
uint16_t  ID_Cart;
uint16_t ReaderCard(void)
{
    uint16_t temp_value = 0;
	
    if(PcdRequest(PICC_REQALL, Temp) == MI_OK)	//ѡ��
    {
        if(Temp[0] == 0x04 && Temp[1] == 0x00)
            printf("\r\nMFOne-S50\r\n");
        else if(Temp[0] == 0x02 && Temp[1] == 0x00)
            printf("MFOne-S70");
        else if(Temp[0] == 0x44 && Temp[1] == 0x00)
            printf("MF-UltraLight");
        else if(Temp[0] == 0x08 && Temp[1] == 0x00)
            printf("MF-Pro");
        else if(Temp[0] == 0x44 && Temp[1] == 0x03)
            printf("MF Desire");
        else
            printf("Unknown");
        if(PcdAnticoll(UID) == MI_OK)			//����ײ
        {
            printf("Card Id is:");
            /* ��ȡ��ֵ  */
            printf("%d%d%d%d\r\n", UID[0], UID[1], UID[2], UID[3]);
            temp_value = ((UID[0] >> 4) * 10 + (UID[0] & 0x0f));
            switch(temp_value)
            {
            case 37 :
            {    
				printf("����Ա:%d ��֤ͨ��!\r\n", temp_value);
				ID_Cart = temp_value;
				return temp_value;
			}
			case 133 :
            {    
				printf("����Ա:%d ��֤ͨ��!\r\n", temp_value);
				ID_Cart = temp_value;
				return temp_value;
			}
            default :
                printf("��Ч��:%d ��֤ʧ��!\r\n", temp_value);
				temp_value = 0;
                break;
            }
        }
    }
	
    return 0;
}

