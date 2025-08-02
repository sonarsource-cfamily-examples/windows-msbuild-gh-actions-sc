#include <string.h>

#include "bus1553lib.h"
#include "bus1553type.h"
#include "1553com.h"
#include "../service/roundMessageQueue.h"
#include "aaa_obdrv_if/include/rs422_if.h"
#include "../service/tc.h"
#include "SylixOS.h"

#define MAX_SUBADDR_NUM         (20)

extern INT    bus1553_memWrite(UINT16 usAddr, CPVOID pvBuf, UINT uiLen);
extern INT    bus1553_memRead(UINT16 usAddr, PVOID pvBuf, UINT uiLen);
extern INT    bus1553_regWrite(UINT16 usAddr, UINT16 usValue);
extern UINT16 bus1553_regRead(UINT16 usAddr);

extern tm_pkg_all               _G_tm_pkg_all;
extern mem_down_all             _G_md_all;

static uint8_t     _S_ucpBusMem[BUS_MEM_MAX];
static uint8_t     ucpData[256];
static uint16_t    _jjm_gnss[7] = {0xEB90, 0x0009, 0x0B00, 0x0000, 0x0000, 0x0000, 0x0000};
static busManage_t _S_bmBus;
static uint32_t uiLen     = 0;
static uint16_t lastStackPtr;
UINT8   tcFlag = 0;
uint16_t wordValue;
extern LW_OBJECT_HANDLE _G_TCNotify;
extern rmsq_obj_t             *p_1553tc_rmq;

/**
 * @see    : RT_init
 * @brief  : RT初始化
 * @param  : NONE
 * @return : NONE
 */
void RT_init(void)
{
    int32_t  i;
    uint16_t usPtr = 0;
    
    /**
     *@ Software reset
     */
    bus1553_regWrite(START_RESET_REG, 0x1);

    /**
     *@ Use ENHANCED MODE function
     */
    bus1553_regWrite(CONFIG3_REG, 0x8000);

    /**
     *@ Interrupt mask:
     *@ RT CIRCULAR BUFFER ROLLOVER
     *@ RT SUBADDRESS CONTROL WORD EOM
     */
    bus1553_regWrite(INT_MASK_REG, 0x0032);

    /**
     *@ Configuration #2
     *@ ENHANCED INTERRUPTS
     *@ OVERWRITE INVALID DATA
     *@ CLEAR TIME TAG ON SYNCHRONIZE
     *@ LOAD TIME TAG ON SYNCHRONIZE
     *@ INTERRUPT STATUS AUTO CLEAR
     *@ CLEAR SERVICE REQUEST
     *@ ENHANCED RT MEMORY MANAGEMENT
     */
    bus1553_regWrite(CONFIG2_REG, 0xAA9F);//0x8876

    /**
     *@ Configuration #3
     *@ ILLEGALIZATION DISABLED
     *@ ENHANCED MODE CODE HANDLING
     *@ NOTICE: This step must be done after bit15 of CFG #3 has been set to 1
     */
    bus1553_regWrite(CONFIG3_REG, 0x8081);

    /**
     *@ Configuration #4
     *@ LATCH RT ADDRESS WITH CFG REG #5
     */
    bus1553_regWrite(CONFIG4_REG, 0x8);

    /**
     *@ Configuration #5
     *@ EXPANDED CROSSING ENABLED
     *@ GAP CHECK ENABLED
     */
    bus1553_regWrite(CONFIG5_REG, 0x0900);

    /**
     *@ Zero time tag
     */
    bus1553_regWrite(TIME_TAG_REG, 0x0);

    memset(_S_ucpBusMem, 0, sizeof(_S_ucpBusMem));
    bus1553_memWrite(0x0, _S_ucpBusMem, BUS_MEM_MAX);

    for (i = 0; i < 32; ++i) {
        /**
         *@ 区域A所有接收子地址初始指向数据块0x0260
         */
        usPtr = 0x0260;
        bus1553_memWrite(0x0140 + i, (CPVOID) &usPtr, 2);

        /**
         *@ 区域A所有发送子地址初始指向数据块0x0280
         */
        usPtr = 0x0280;
        bus1553_memWrite(0x0160 + i, (CPVOID) &usPtr, 2);

        //   config  boardcast
        usPtr = 0x02A0;
        bus1553_memWrite(0x0180 + i, (CPVOID) &usPtr, 2);

    }



    _S_bmBus.usCurrMode      = RT_MODE;
    _S_bmBus.RT.usCurrBufPtr = RT_DATA_BLK_START;

    memset(_S_bmBus.RT.cwConfigTab, 0, sizeof(CONFIGWORD) * RT_SUBADDR_MAX);

    /**
     *@ Configuration #1
     *@ BUSY*
     *@ SERVICE REQUEST*
     */
    bus1553_regWrite(CONFIG1_REG, 0x8F80);
}

/**
 * @see    : RT_configSa1
 * @brief  : 配置子地址
 * @param  : usSatType -- RX?TX? RX=0, TX=1
 * @param  : usSubAddr -- RT sub address
 * @param  : usBufSize -- data block buffer size
 * @param  : usEom     -- is use end-of-message
 * @param  : usCirc    -- is use circle buffer mode
 * @return : NONE
 */
void RT_configSa1(uint16_t usSaType, uint16_t usSubAddr, uint16_t usBufSize, uint16_t usEom, uint16_t usCirc)
{
    uint16_t   i, j;
    CONFIGWORD configWord;
    rtMode_t  *pRt = &_S_bmBus.RT;
    
    memset (&configWord, 0, sizeof(CONFIGWORD));

    if ( TRUE == pRt->usConfigErr) {
        fprintf(stderr, "[1553] Config Error\r\n");

        return;
    } else if (pRt->usConfigCnt >= MAX_SUBADDR_NUM) {
        /**
         *@ 超出可用最大子地址数
         */
        fprintf(stderr, "[1553] More than max subaddress configuration number\n");
        pRt->usConfigErr = TRUE;

        return;
    } else if ((usBufSize + pRt->usConfigSize) > MAX_BUFFER_SIZE) {
        /**
         *@ 缓存区大小超限
         */
        fprintf(stderr, "[1553] Buffer is over the limit:%d\n", (usBufSize + pRt->usConfigSize));
        pRt->usConfigErr = TRUE;

        return;
    }

    /**
     *@ 缓冲区大小
     */
    switch (usBufSize) {

    case 32:
        /**
         *@ 32words
         */
        configWord.sa2 = 0;
        break;

    case 128:
        /**
         *@ 128words
         */
        configWord.sa2 = 1;
        break;

    case 256:
        /**
         *@ 256words
         */
        configWord.sa2 = 2;
        break;

    case 512:
        /**
         *@ 512words
         */
        configWord.sa2 = 3;
        break;

    case 1024:
        /**
         *@ 1024words
         */
        configWord.sa2 = 4;
        break;

    case 2048:
        /**
         *@ 2048words
         */
        configWord.sa2 = 5;
        break;

    case 4096:
        /**
         *@ 4096words
         */
        configWord.sa2 = 6;
        break;

    case 8192:
        /**
         *@ 8192words
         */
        configWord.sa2 = 7;
        break;

    default:
        fprintf(stderr, "[1553] Buffer size is not valid value:%d\n", usBufSize);
        pRt->usConfigErr = TRUE;
        return;
    }

    /**
     *@ 接收/发送模式
     */
    if ( RX == usSaType) {
        configWord.RX1 = 1;
    } else {
        configWord.TX1 = 1;
    }

    configWord.sa1 = usSubAddr;

    for (i = 0; i < pRt->usConfigCnt; i++) {
        /**
         *@ 配置表中已经存在该子地址的配置信息，直接退出配置模式
         */
        if ( (configWord.sa1 == pRt->cwConfigTab[i].sa1) &&
               ((configWord.RX1 == pRt->cwConfigTab[i].RX1) ||
                (configWord.TX1 == pRt->cwConfigTab[i].TX1)) ) {
            fprintf(stderr, "[1553] This subaddress has been configed\r\n");
            /**
             *@ 重复配置
             */
            pRt->usConfigErr = TRUE;

            return;
        }
    }

    /**
     *@ 开启消息结束中断
     */
    if ( TRUE == usEom) {
        configWord.EOM = 1;
    }

    /**
     *@ 开启缓冲区回滚中断
     */
    if ( TRUE == usCirc) {
        configWord.CIRC = 1;
    }

    /**
     *@ 按缓存区大小插入配置表
     */
    for (i = 0; i < pRt->usConfigCnt; i++) {
        if ((!pRt->cwConfigTab[i].RX2 && !pRt->cwConfigTab[i].TX2
                && (configWord.sa2 > pRt->cwConfigTab[i].sa2))
                || ((1 == pRt->cwConfigTab[i].RX2)
                        || (1 == pRt->cwConfigTab[i].TX2))) {
            /**
             *@ 依次向后移动一个位置，留出空位插入新的子地址配置表
             */
            for (j = 0; j < (pRt->usConfigCnt - i); j++) {
                pRt->cwConfigTab[pRt->usConfigCnt - j] = pRt->cwConfigTab[pRt->usConfigCnt - j - 1];
            }

            pRt->cwConfigTab[i] = configWord;
            ++pRt->usConfigCnt;
            pRt->usConfigSize += usBufSize;

            return;
        }
    }

    pRt->cwConfigTab[pRt->usConfigCnt] = configWord;
    ++pRt->usConfigCnt;
    pRt->usConfigSize += usBufSize;
}

/**
 * @see    : RT_configSa2
 * @brief  : 子地址配置相同缓存区
 * @param  : wSatType  -- RX?TX? RX=0, TX=1
 * @param  : wSubAddr  -- RT sub address
 * @param  : mSatType  -- RX?TX? RX=0, TX=1
 * @param  : mSubAddr  -- RT sub address
 * @param  : usEom     -- is use end-of-message
 * @param  : usCirc    -- is use circle buffer mode
 * @return : NONE
 */
void RT_configSa2(uint16_t wSaType, uint16_t wSubAddr, uint16_t mSaType, uint16_t mSubAddr, uint16_t usEom, uint16_t usCirc)
{
    uint16_t   i;
    CONFIGWORD configWord;
    rtMode_t  *pRt = &_S_bmBus.RT;
    
    memset (&configWord, 0, sizeof(CONFIGWORD));

    /**
     *@ 检查前序的配置是否出错
     */
    if ( TRUE == pRt->usConfigErr) {
        fprintf(stderr, "[1553] some error has occured\r\n");

        return;
    } else if (pRt->usConfigCnt >= MAX_SUBADDR_NUM) {
        /**
         *@ 超出可用最大子地址数
         */
        pRt->usConfigErr = TRUE;

        return;
    }

    /**
     *@ 配置接收还是发送
     */
    if ( RX == wSaType) {
        configWord.RX1 = 1;
    } else {
        configWord.TX1 = 1;
    }

    configWord.sa1 = wSubAddr;

    for (i = 0; i < pRt->usConfigCnt; i++) {
        /**
         *@ 重复配置
         */
        if ((configWord.sa1 == pRt->cwConfigTab[i].sa1)
                && ((configWord.RX1 == pRt->cwConfigTab[i].RX1)
                        || (configWord.TX1 == pRt->cwConfigTab[i].TX1))) {
            pRt->usConfigErr = TRUE;

            return;
        }
    }

    if ( RX == mSaType) {
        configWord.RX2 = 1;
    } else {
        configWord.TX2 = 1;
    }

    configWord.sa2 = mSubAddr;

    /**
     *@ 开启消息结束中断
     */
    if ( TRUE == usEom) {
        configWord.EOM = 1;
    }

    /**
     *@ 开启缓冲区回滚中断
     */
    if ( TRUE == usCirc) {
        configWord.CIRC = 1;
    }
    
    ++pRt->usConfigCnt;
    pRt->cwConfigTab[pRt->usConfigCnt] = configWord;
}

/**
 * @see    : RT_configLegal
 * @brief  : 设置非法化表,设置的为合法的
 * @param  : usIsBcst  -- 是否配置广播区域非法化表
 * @param  : usTR      -- 发送还是接收
 * @param  : usSubAddr -- RT子地址
 * @param  : usWordCnt -- 字个数
 * @return : NONE
 */
void RT_configLegal(uint16_t usIsBcst, uint16_t usTR, uint16_t usSubAddr, uint16_t usWordCnt)
{
    uint16_t usAddr = RT_ILL_TAB_ADDR;
    uint16_t usData = 0xFFFF;
    
    if ( TRUE == usIsBcst) {
        usAddr &= 0xff7f;
    } else {
        usAddr |= 0x80;
    }
    
    usAddr |= (usTR << 6);
    usAddr |= (usSubAddr << 1);
    usAddr |= ((usWordCnt & 0x1F) >> 4);

    bus1553_memRead(usAddr, (PVOID)&usData, 2);
    
    if ( usWordCnt & 0x0F) {
        usData &= ~(0x01 << (usWordCnt & 0x0F));
    } else {
        usData &= 0xFFFE;
    }

    bus1553_memWrite(usAddr, (CPVOID)&usData, 2);
}

/**
 * @see    : RT_start
 * @brief  : RT启动
 * @param  : NONE
 * @return : 启动失败返回0，否则返回1
 */
uint16_t RT_start(void)
{
    uint16_t  i;
    uint16_t  usCtrlWord1;
    uint16_t  usCtrlWord2;
    uint16_t  usBufSize;
    uint16_t  usBufAddr;
    rtMode_t *pRt = &_S_bmBus.RT;

    /**
     *@ 判断是否已启动
     */
    if ( TRUE == pRt->usIsStarted) {
        fprintf(stderr, "[1553] Has been started\r\n");

        return (FALSE);
    } else if ( TRUE == pRt->usConfigErr) {
        /**
         *@ 配置信息有误
         */
        fprintf(stderr, "[1553] Config information has something wrong, please check it\r\n");

        RT_stop();

        return (FALSE);
    }

    /**
     *@ 启动
     */
    pRt->usIsStarted = TRUE;

    fprintf(stdout, "[1553] There are total %d SubAddress to be configurated\r\n", pRt->usConfigCnt);

    for (i = 0; i < pRt->usConfigCnt; i++) {
        usCtrlWord1 = 0;
        bus1553_memRead(RT_LOOKUPTAB_SACW_START_A + pRt->cwConfigTab[i].sa1, (PVOID) &usCtrlWord1, 2);
        usCtrlWord2 = 0;
        bus1553_memRead(RT_LOOKUPTAB_SACW_START_A + pRt->cwConfigTab[i].sa2, (PVOID) &usCtrlWord2, 2);

        if (1 == pRt->cwConfigTab[i].RX2) {
            /**
             *@ RT-RT RX
             */
            usBufSize = (usCtrlWord2 >> 5) & 0x0007;
            usBufAddr = 0;
            bus1553_memRead(RT_LOOKUPTAB_RX_START_A + pRt->cwConfigTab[i].sa2, (PVOID) &usBufAddr, 2);
            printf("RT-RT RX\r\n");
        } else if (1 == pRt->cwConfigTab[i].TX2) {
            /**
             *@ RT-RT TX
             */
            usBufSize = (usCtrlWord2 >> 10) & 0x0007;
            usBufAddr = 0;
            bus1553_memRead(RT_LOOKUPTAB_TX_START_A + pRt->cwConfigTab[i].sa2, (PVOID) &usBufAddr, 2);
            printf("RT-RT TX\r\n");
        } else {
            /**
             *@ self from/to BC RX/TX
             */
            usBufSize = pRt->cwConfigTab[i].sa2;
            usBufAddr = pRt->usCurrBufPtr;

            if (!usBufSize) {
                pRt->usCurrBufPtr += 32;
            } else {
                pRt->usCurrBufPtr += (64 * (2 << usBufSize));
            }
        }

        if (1 == pRt->cwConfigTab[i].RX1) {
            /**
             *@ Subaddress is used as RX mode
             */
            if(pRt->cwConfigTab[i].sa1 == 14 || pRt->cwConfigTab[i].sa1 == 16) {
                usCtrlWord1 &= ~(0x7 << BCST_BUF_SIZE);
                usCtrlWord1 |= (usBufSize << BCST_BUF_SIZE);
            }
            else {
                usCtrlWord1 &= ~(0x7 << RX_BUF_SIZE);
                usCtrlWord1 |= (usBufSize << RX_BUF_SIZE);
            }

            if (pRt->cwConfigTab[i].EOM) {
                if(pRt->cwConfigTab[i].sa1 == 14 || pRt->cwConfigTab[i].sa1 == 16) {
                    usCtrlWord1 |= BCST_EOM_INT_EN;
                }
                else {
                    usCtrlWord1 |= RX_EOM_INT_EN;
                }

            }

            if (pRt->cwConfigTab[i].CIRC) {
                usCtrlWord1 |= RX_CIRC_BUF_INT_EN;
            }

            /**
             *@ 控制字
             */
            bus1553_memWrite(RT_LOOKUPTAB_SACW_START_A + pRt->cwConfigTab[i].sa1, (CPVOID) &usCtrlWord1, 2);
            fprintf(stderr, "[1553] RX 1553 MEM : %04X , ctrl_word : %04X\r\n",
                                                    RT_LOOKUPTAB_SACW_START_A + pRt->cwConfigTab[i].sa1, usCtrlWord1);

            if(pRt->cwConfigTab[i].sa1 == 14 || pRt->cwConfigTab[i].sa1 == 16) { // 广播子地址
               /**
                 *@ 查找表
                 */
                bus1553_memWrite(RT_LOOPUPTAB_BCST_START_A + pRt->cwConfigTab[i].sa1, (CPVOID) &usBufAddr, 2);
                fprintf(stderr, "[1553] RX 1553 MEM : %04X , BufAddr : %04X\r\n",
                                                    RT_LOOPUPTAB_BCST_START_A + pRt->cwConfigTab[i].sa1, usBufAddr);
            }
            else {
                /**
                 *@ 查找表
                 */
                bus1553_memWrite(RT_LOOKUPTAB_RX_START_A + pRt->cwConfigTab[i].sa1, (CPVOID) &usBufAddr, 2);
                fprintf(stderr, "[1553] RX 1553 MEM : %04X , BufAddr : %04X\r\n",
                                                       RT_LOOKUPTAB_RX_START_A + pRt->cwConfigTab[i].sa1, usBufAddr);
            }

            printf("BC-RT \r\n");
        } else {
            /**
             *@ Subaddress is used as TX mode
             */
            usCtrlWord1 &= ~(0x7 << TX_BUF_SIZE);
            usCtrlWord1 |= (usBufSize << TX_BUF_SIZE);

            if (pRt->cwConfigTab[i].EOM) {
                usCtrlWord1 |= TX_EOM_INT_EN;
            }

            if (pRt->cwConfigTab[i].CIRC) {
                usCtrlWord1 |= TX_CIRC_BUF_INT_EN;
            }

            /**
             *@ 控制字
             */
            bus1553_memWrite(RT_LOOKUPTAB_SACW_START_A + pRt->cwConfigTab[i].sa1, (CPVOID) &usCtrlWord1, 2);
            fprintf(stderr, "[1553] TX 1553 MEM : %04X , ctrl_word : %04X\r\n",
                                        RT_LOOKUPTAB_SACW_START_A + pRt->cwConfigTab[i].sa1, usCtrlWord1);

            /**
             *@ 查找表
             */
            bus1553_memWrite(RT_LOOKUPTAB_TX_START_A + pRt->cwConfigTab[i].sa1, (CPVOID) &usBufAddr, 2);
            fprintf(stderr, "[1553] TX 1553 MEM : %04X , BufAddr : %04X\r\n",
                                       RT_LOOKUPTAB_TX_START_A + pRt->cwConfigTab[i].sa1, usBufAddr);
        }
    }
    
    return (TRUE);
}

/**
 * @see    : RT_stop
 * @brief  : RT停止
 * @param  : NONE
 * @return : NONE
 */
void RT_stop(void)
{
    rtMode_t *pRt   = &_S_bmBus.RT;
    uint16_t  usVal = 0;
    int32_t   i;
    
    pRt->usIsStarted  = FALSE;
    pRt->usConfigErr  = FALSE;
    pRt->usConfigCnt  = 0;
    pRt->usConfigSize = 0;
    pRt->usCurrBufPtr = RT_DATA_BLK_START;

    /**
     *@ 清空配置表
     */
    memset (pRt->cwConfigTab, 0, sizeof(CONFIGWORD) * MAX_SUBADDR_NUM);

    for (i = 0; i < RT_SUBADDRESS_NM; ++i) {
        /**
         *@ 所有接收子地址初始指向数据块0x0240
         */
        usVal = RT_DATA_BLK_FIRST;
        bus1553_memWrite(RT_LOOKUPTAB_RX_START_A + i, (CPVOID) &usVal, 2);

        /**
         *@ 所有发送子地址初始指向数据块0x0260
         */
        usVal += RT_DATA_BLK_SIZE;
        bus1553_memWrite(RT_LOOKUPTAB_TX_START_A + i, (CPVOID) &usVal, 2);
    }
}

/**
 * @see    : RT_readRxData
 * @brief  : RT读接收子地址数据
 * @param  : usSubAddr  -- 读取的RT子地址
 * @param  : uspDataBuf -- 存储读取的数据
 * @param  : usDataLen  -- 读取的数据长度
 * @return : 读取成功返回实际个数，否则返回-1
 */
int32_t RT_readRxData(uint16_t usSubAddr, uint16_t *uspDataBuf, uint16_t usDataLen)
{
    uint16_t i;
    uint16_t usBufAddr = 0;
    
    usBufAddr = RT_getBufAddr(RX, usSubAddr);
    if ( RT_ADDR_INVALID == usBufAddr) {
        return (PX_ERROR);
    }

    for (i = 0; i < usDataLen; i++) {
        bus1553_memRead(usBufAddr + i, (PVOID) &uspDataBuf[i], 2);
    }

    /**
     *@ 取完数据指针复位
     */
//    bus1553_memWrite(RT_LOOKUPTAB_RX_START_A + usSubAddr, (CPVOID)&usBufAddr, 2);

    return (usDataLen);
}

/**
 * @see    : RT_writeTxData
 * @brief  : RT发送子地址数据
 * @param  : usSubAddr  -- 发送的RT子地址
 * @param  : uspDataBuf -- 存储发送的数据
 * @param  : usDataLen  -- 发送的数据长度
 * @return : 发送成功返回实际个数，否则返回-1
 */
int32_t RT_writeTxData(uint16_t usSubAddr, uint16_t *uspBuf, uint16_t usLen)
{
    uint16_t i;
    uint16_t usBufAddr = 0;
    
    usBufAddr = RT_getBufAddr(TX, usSubAddr);
    if ( RT_ADDR_INVALID == usBufAddr) {
        return (PX_ERROR);
    }

    for (i = 0; i < usLen; i++) {
        bus1553_memWrite(usBufAddr + i, (CPVOID) &uspBuf[i], 2);
    }
    
    /**
     *@ 发送完数据指针复位
     */
//    bus1553_memWrite(RT_LOOKUPTAB_TX_START_A + usSubAddr, (CPVOID)&usBufAddr, 2);

    return (usLen);
}

/**
 * @see    : RT_readRxWord
 * @brief  : RT读接收子地址数据(1个字)
 * @param  : usSubAddr  -- 读取的RT子地址
 * @return : 读取成功返回实际读取的数据
 */
int32_t RT_readRxWord(uint16_t usSubAddr)
{
    uint16_t usData = 0;
    
    RT_readRxData(usSubAddr, &usData, 1);
    
    return (usData);
}

/**
 * @see    : RT_writeTxWord
 * @brief  : RT发送子地址数据(1个字)
 * @param  : usSubAddr -- 发送的RT子地址
 * @param  : usData    -- 发送的数据字
 * @return : NONE
 */
void RT_writeTxWord(uint16_t usSubAddr, uint16_t usData)
{
    RT_writeTxData(usSubAddr, &usData, 2);
}

/**
 * @see    : RT_getRtAddr
 * @brief  : 获取自身RT地址
 * @param  : NONE
 * @return : RT地址
 */
uint16_t RT_getRtAddr(void)
{
    return ((bus1553_regRead(CONFIG5_REG) >> 1) & 0x1F);
}

/**
 * @see    : RT_getStatusWord
 * @brief  : RT获取状态字
 * @param  : NONE
 * @return : RT状态字
 */
uint16_t RT_getStatusWord(void)
{
    return (bus1553_regRead(RT_STAT_WORD_REG));
}

/**
 * @see    : RT_getLastCmd
 * @brief  : 获取最新指令字
 * @param  : NONE
 * @return : RT最新指令字
 */
uint16_t RT_getLastCmd(void)
{
    return (bus1553_regRead(RT_LAST_CMD_REG));
}

/**
 * @see    : RT_serviceRequest
 * @brief  : 服务请求
 * @param  : NONE
 * @return : NONE
 */
void RT_serviceRequest(void)
{
    bus1553_regWrite(CONFIG1_REG, bus1553_regRead(CONFIG1_REG) & ~(0x1<<9));
}

/**
 * @see    : RT_getBusAddr
 * @brief  : 获取缓存区首地址
 * @param  : usSaType  -- 接收？发送？接收-0，发送-1
 * @param  : usSubAddr -- 要获取的子地址
 * @return : 返回对应子地址的缓冲区地址
 */
uint16_t RT_getBufAddr(uint16_t usSaType, uint16_t usSubAddr)
{
    uint16_t usCtrlWord = 0;
    uint16_t usBufSize  = 0;
    uint16_t usBufAddr  = 0;

    /**
     *@ 读取子地址控制字
     */
    bus1553_memRead(RT_LOOKUPTAB_SACW_START_A + usSubAddr, (PVOID)&usCtrlWord, 2);
    
    if ( TX == usSaType) {
        bus1553_memRead(RT_LOOKUPTAB_TX_START_A + usSubAddr, (PVOID) &usBufAddr, 2);
        usBufSize = (usCtrlWord >> TX_BUF_SIZE) & CTRL_WORD_BUF_SIZE_MASK;
    } else {
        if(usSubAddr == 14 || usSubAddr == 16) {
            bus1553_memRead(RT_LOOPUPTAB_BCST_START_A + usSubAddr, (PVOID) &usBufAddr, 2);
            usBufSize = (usCtrlWord >> BCST_BUF_SIZE) & CTRL_WORD_BUF_SIZE_MASK;
        }
        else {
            bus1553_memRead(RT_LOOKUPTAB_RX_START_A + usSubAddr, (PVOID) &usBufAddr, 2);
            usBufSize = (usCtrlWord >> RX_BUF_SIZE) & CTRL_WORD_BUF_SIZE_MASK;
        }
    }

    /**
     *@ 接收处理完一条消息后，指针更新为下一次接收处理的地方，
     *@ 需要根据当前设置的缓冲区大小回到前一次处理的地方
     */
    if (usBufSize > 0) {
        usBufAddr = (usBufAddr >> (usBufSize + 6));
        usBufAddr = (usBufAddr << (usBufSize + 6));
    }
    
    return (usBufAddr);
}

/**
 * @see    : getBusRegs
 * @brief  : 获取1553B控制器各寄存器值（适用于BC/RT模式）
 * @param  : NONE
 * @return : 寄存器当前值
 */
uint16_t getBusRegs(uint16_t usReg)
{
    if ( usReg > BUS_REG_MAX) {
        fprintf(stderr, "[553] bus register No invalid, should be between 0~%d\r\n", BUS_REG_MAX);

        return (0);
    }

    return (bus1553_regRead(usReg));
}

/**
 * @see    : RT_modeCodeIntEnable
 * @brief  : 为各模式字使能中断
 * @param  : uiModeCode   -- 模式字代码
 * @param  : usIsWithData -- 是否带有数据
 * @param  : usType       -- 发送类型还是接收，0表示接收，1表示发送
 * @return : NONE
 */
void RT_modeCodeIntEnable(uint32_t uiModeCode, uint16_t usIsWithData, uint16_t usType)
{
    uint16_t usValue = 0;

    if ( RX == usType) {
        bus1553_memRead(0x0109, (PVOID)&usValue, 2);
        usValue = 0x1 << (uiModeCode&0xFFFF);
        bus1553_memWrite(0x0109, (CPVOID)&usValue, 2);
    } else {
        if ( !usIsWithData) {
            bus1553_memRead(0x010A, (PVOID)&usValue, 2);
            usValue = 0x1 << (uiModeCode&0xFFFF);
            bus1553_memWrite(0x010A, (CPVOID)&usValue, 2);
            usValue = 0;
            bus1553_memRead(0x010A, (PVOID)&usValue, 2);
            fprintf(stdout, "[%d] %X\r\n", uiModeCode, usValue);
        } else {
            bus1553_memRead(0x010B, (PVOID)&usValue, 2);
            usValue = 0x1 << (uiModeCode&0xFFFF);
            bus1553_memWrite(0x010B, (CPVOID)&usValue, 2);
        }
    }
}

/**
 * @see    : RT_setVectorWord
 * @brief  : 为RT设置矢量字
 * @param  : usValue -- 新的矢量字
 * @return : 返回之前的矢量字
 */
uint16_t RT_setVectorWord(uint16_t usValue)
{
    uint16_t usVec = 0;

    bus1553_memRead(0x0120,  (PVOID)&usVec,    2);
    bus1553_memWrite(0x0120, (CPVOID)&usValue, 2);

    return (usVec);
}


void rt_handler(int signo)
{
    UINT16 lastCmdWord = 0;
    UINT16 blockStatusWord = 0;
    UINT8 dataTxRxBit = 0;
    UINT8 rtAddress = 0;
    UINT8 subAddress = 0;
    UINT8 dataLen = 0;
    UINT16 curStackPtr = 0;
    UINT16 ptrTemp = 0;
    UINT8 index = 64;
    uint16_t usValue;



//        p_workstatus->RT_1553intrcnt++;

        bus1553_memRead(0x100,&curStackPtr,2); //current stack pointer

        if(curStackPtr == 0)
        {
            ptrTemp = 0x100;
        }
        else
        {
            ptrTemp = curStackPtr;
        }
        bus1553_memRead(ptrTemp-4,&blockStatusWord,2);

        while(index--)
        {
            ptrTemp &= 0xfffc;  //按4对齐

            if(ptrTemp == lastStackPtr) //栈是否读完
            {
                lastStackPtr = curStackPtr;

                return ;
            }

            if(ptrTemp == 0)
             {
                 ptrTemp = 0x100;
             }


            bus1553_memRead(ptrTemp-1,&lastCmdWord,2);
            rtAddress =  (UINT8) ((lastCmdWord>>11) & 0x1f);
            dataTxRxBit = (UINT8) ((lastCmdWord>>10) & 0x01);
            subAddress = (UINT8) ((lastCmdWord>>5) & 0x1f);
            dataLen = lastCmdWord & 0x1f;

            if(rtAddress == 9)
            {

                if(dataTxRxBit == 0)    //RX sub addr
                {
                    switch(subAddress)
                    {
                        case 0x02:  //遥控注数
                        {
                            if (!dataLen) {
//                                printf("tcFlag: %x \n", tcFlag);
                                memset(ucpData, 0, sizeof(ucpData));
                                uint16_t usBufAddr = RT_getBufAddr(RX, 2);
                                uiLen = RT_readRxData(2, (uint16_t *)ucpData, 128);
                                bus1553_memWrite(RT_LOOKUPTAB_RX_START_A + 2, (CPVOID)&usBufAddr, 2);
                                tcFlag++;
                                memcpy(roundMessageQueue_queueIn_position(p_1553tc_rmq), (void *) ucpData, 256);
                                roundMessageQueue_posIn_inc(p_1553tc_rmq);
                                if(tcFlag > 3){
                                    tcFlag = 0;
                                    API_SemaphoreBPost(_G_TCNotify);
                                }
                            } else {
                                  // error --- tc
                                  printf("errlen = %d\r\n",uiLen);
                                  break;
                              }

                            break;
                        }
                        case 0x0f:  //长抱环
                        {
                            uint16_t usBufAddr;
                            usBufAddr = RT_getBufAddr(RX, 15);
                            wordValue = RT_readRxWord(15);
                            bus1553_memWrite(RT_LOOKUPTAB_RX_START_A + 15, (CPVOID)&usBufAddr, 2);
                            usBufAddr = RT_getBufAddr(TX, 15);
                            wordValue = ~wordValue;
                            printf("wordValue:%x\r\n", wordValue);
                            RT_writeTxWord(15, wordValue);
                            bus1553_memWrite(RT_LOOKUPTAB_TX_START_A + 15, (CPVOID)&usBufAddr, 2);
                            break;
                        }
                        default:
                            break;
                    }
                }
                else    //TX sub addr
                {
                    switch(subAddress)
                    {
                        case 0x03:  //遥测
                        {
                            uint16_t usBufAddr = RT_getBufAddr(TX, 3);
                            bus1553_memWrite(RT_LOOKUPTAB_TX_START_A + 3, (CPVOID)&usBufAddr, 2);
                            break;
                        }
                        case 0x04:  //指令/注数返回
                        {

                            break;
                        }


                        case 0x0C:  //内存下卸
                        {
                            uint16_t usBufAddr = RT_getBufAddr(TX, 12);
                            bus1553_memWrite(RT_LOOKUPTAB_TX_START_A + 12, (CPVOID)&usBufAddr, 2);
                            break;
                        }
                        default:    // others
                        {
                            break;  // Do nothing ???
                        }
                    }
                }
            }
            else if (rtAddress == 31)   //brodcast sub addr
            {
                switch(subAddress)
                {

                    case 0x0c:
                    {
                        printf("system sync\r\n");
                        RT_start();

                        break;
                    }

                    case 0x10:  //时间广播
                    {

                        uint16_t time[3];
                        RT_readRxData(16, time, 6);
                        _jjm_gnss[3] = time[0];
                        _jjm_gnss[4] = time[1];
                        _jjm_gnss[5] = time[2];
                        uint16_t value = _jjm_gnss[2] ^ _jjm_gnss[3] ^ _jjm_gnss[4] ^ _jjm_gnss[5];
                        _jjm_gnss[6] = value;

                        uint32_t com_status = RS422_SendBusyState();
                        if (!(com_status & (1 << JJM_RS422_INDEX))) { // 串口不忙时发送GNSS时间码
                            RS422_DataSend(4, _jjm_gnss, sizeof(_jjm_gnss));
                        } else {
                            buginfo_print("JJM Rs422 is busy.");
                        }

                        break;
                    }

                    default:    //others
                    {
                        // Do nothing ???
                        break;
                    }
                }
            }

            usValue = 0;
            bus1553_memWrite(ptrTemp-1, &usValue,2);  //clear cmd word
            ptrTemp = ptrTemp - 4;

        }
}


int bus1553_rt(void)
{

    signal(SIGUSR1, rt_handler);
    bus1553_isrSignalRegister(API_ThreadIdSelf(),SIGUSR1);


    RT_init();

    fprintf(stdout, "Current RT Address: %d\r\n", RT_getRtAddr());

    RT_configSa1(RX, 2, 128,   0, 1);  //Tc

    RT_configSa1(RX, 14, 32,   1, 0);  //系统同步

    RT_configSa1(RX, 15, 32,   0, 0);  //长抱环

    RT_configSa1(RX, 16, 32,   1, 0);  //GNSS时间

    RT_configSa1(TX, 3,  256,   0, 1);  //Tm

    RT_configSa1(TX, 12, 512,   0, 1);  //内存下卸

    RT_start();

    return (0);



}
