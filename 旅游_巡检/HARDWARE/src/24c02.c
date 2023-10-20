#include "24c02.h"

#define AT24CXX_Write_ADDR 0xA0
#define AT24CXX_Read_ADDR  0xA1
#define AT24CXX_MAX_SIZE   256
#define AT24CXX_PAGE_SIZE  8
#define AT24CXX_PAGE_TOTAL (AT24CXX_MAX_SIZE/AT24CXX_PAGE_SIZE)

#define I2C                &hi2c2


uint8_t AT24C02_Check(void)
{
    if(HAL_I2C_Master_Transmit(&hi2c2, AT24CXX_Write_ADDR, NULL, 0, HAL_MAX_DELAY) == HAL_OK)
    {
        printf("AT24C02 存在\r\n");
        return 0;
    }
    printf("AT24C02 不存在\r\n");
    return 1;
}

// addr 地址
// dataptr 数据指针
// datasize 数据大小
int AT24C02_write(uint8_t addr, uint8_t *dataPtr, uint16_t dataSize)
{
    if (0 == dataSize)
    {
        return -1;
    }

    int res = HAL_OK;

    int selectPage_idx  = addr % AT24CXX_PAGE_SIZE;
    int selectPage_rest = AT24CXX_PAGE_SIZE - selectPage_idx;

    if (dataSize <= selectPage_rest)
    {
        res = HAL_I2C_Mem_Write(I2C,
                                AT24CXX_Write_ADDR,
                                addr,
                                I2C_MEMADD_SIZE_8BIT,
                                dataPtr,
                                dataSize,
                                0xFF);

        if (HAL_OK != res)
        {
            return -1;
        }

        HAL_Delay(10);

    }
    else
    {

        /*! 1 write selectPage rest*/
        res = HAL_I2C_Mem_Write(I2C,
                                AT24CXX_Write_ADDR,
                                addr,
                                I2C_MEMADD_SIZE_8BIT,
                                dataPtr,
                                selectPage_rest,
                                0xFF);

        if (HAL_OK != res)
        {
            return -1;
        }

        addr     += selectPage_rest;
        dataSize -= selectPage_rest;
        dataPtr  += selectPage_rest;

        HAL_Delay(5);

        /*! 2 write nextPage full */
        int fullPage = dataSize / AT24CXX_PAGE_SIZE;
        for (int iPage = 0; iPage < fullPage; ++iPage)
        {
            res = HAL_I2C_Mem_Write(I2C,
                                    AT24CXX_Write_ADDR,
                                    addr,
                                    I2C_MEMADD_SIZE_8BIT,
                                    dataPtr,
                                    AT24CXX_PAGE_SIZE,
                                    0xFF);

            if (HAL_OK != res)
            {
                return -1;
            }

            HAL_Delay(5);

            addr     += AT24CXX_PAGE_SIZE;
            dataSize -= AT24CXX_PAGE_SIZE;
            dataPtr  += AT24CXX_PAGE_SIZE;
        }

        /*! 3 write rest */
        if (0 != dataSize)
        {
            res = HAL_I2C_Mem_Write(I2C,
                                    AT24CXX_Write_ADDR,
                                    addr,
                                    I2C_MEMADD_SIZE_8BIT,
                                    dataPtr,
                                    dataSize,
                                    0xFF);

            if (HAL_OK != res)
            {
                return -1;
            }

            HAL_Delay(5);
        }
    }

    return 0;
}


/*! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// addr 地址
// dataptr 数据指针
// datasize 数据大小
int AT24C02_read(uint8_t addr, uint8_t *dataPtr, uint16_t dataSize)
{
    int res = HAL_I2C_Mem_Read(I2C, AT24CXX_Read_ADDR, addr, I2C_MEMADD_SIZE_8BIT, dataPtr, dataSize, 0xFF);

    if (HAL_OK != res)
    {
        return -1;
    }

    return 0;
}







