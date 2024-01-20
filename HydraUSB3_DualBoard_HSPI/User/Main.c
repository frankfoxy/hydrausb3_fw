/********************************** (C) COPYRIGHT *******************************
* File Name   : Main.c
* Author      : bvernoux
* Version     : V1.1.1
* Date        : 2022/12/11
* Description : Basic example to test HSPI communication between 2x HydraUSB3 boards
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56x_debug_log.h"
#define DEI60F225SOM 1

#undef FREQ_SYS
/* System clock / MCU frequency(HSPI Frequency) in Hz */
#define FREQ_SYS (120000000)

#if(defined DEBUG) // DEBUG=1 to be defined in Makefile DEFINE_OPTS (Example DEFINE_OPTS = -DDEBUG=1)
    #define UART1_BAUD (115200)
//#define UART1_BAUD (921600)
//#define UART1_BAUD (3000000) // Real baud rate 3Mbauds(For Fsys 96MHz or 120MHz) => Requires USB2HS Serial like FTDI C232HM-DDHSL-0
//#define UART1_BAUD (5000000) // Real baud rate is round to 5Mbauds (For Fsys 120MHz) => Requires USB2HS Serial like FTDI C232HM-DDHSL-0
#endif

//Data size
#define DataSize_8bit  0
#define DataSize_16bit 1
#define DataSize_32bit 2
/* HSPI Data Size Selection 8, 16 or 32bits bus */
//#define Data_Size DataSize_8bit
//#define Data_Size DataSize_16bit
#define Data_Size DataSize_32bit

//DMA_Len
#define DMA_Tx_Len   (512)
#define DMA_Tx_Len0   DMA_Tx_Len
#define DMA_Tx_Len1   DMA_Tx_Len

//DMA_Addr0
#define TX_DMA_Addr0   0x20020000
#define RX_DMA_Addr0   0x20020000

//DMA_Addr1
#define TX_DMA_Addr1   0x20020000 + DMA_Tx_Len0
#define RX_DMA_Addr1   0x20020000 + DMA_Tx_Len1

/* Shared variables */
volatile int HSPI_TX_End_Flag; // Send completion flag
volatile int HSPI_RX_End_Flag; // Receive completion flag
volatile int HSPI_RX_End_Err; // 0=No Error else >0 Error code

/* HSPI_IRQHandler variables */
uint32_t Tx_Cnt = 0;
uint32_t Rx_Cnt = 0;
uint32_t rx_cnt_total = 0;
uint32_t rx_cnt_err_crc = 0; 
uint32_t rx_cnt_err_seq = 0;
uint32_t addr_cnt = 0;

/* Blink time in ms */
#define BLINK_ULTRA_FAST  2 // Determine the speed of Packets Sent (It shall be not too fast for the Slave)
/* BLINK_ULTRA_FAST < 2ms do error on HSPI Slave
 * It heavily depends on Master/Slave UART "debug" speed too (if it is too fast => Serial Port have a big impact on speed)
 * */
#define BLINK_SLOW   250
int blink_ms = BLINK_SLOW;

bool_t board_role_rx; /* Return true or false */
bool spi_test_mode;
/* Required for log_init() => log_printf()/cprintf() */
debug_log_buf_t log_buf;


uint8_t rx_seq_list[64];
uint8_t rx_seq_list_idx;

#define BSP_DEI60F100M_PIN_RST_I GPIO_Pin_13
#define BSP_DEI60F100M_PIN_ERR_O GPIO_Pin_14
#define BSP_DEI60F100M_PIN_OK_O GPIO_Pin_15

#define BSP_DEI60F100M_PIN_HSPI_TX_PAUSE GPIO_Pin_14
void bsp_dei60f100_init(void)
{
    // PA13: as reset button in
    GPIOA_ResetBits(BSP_DEI60F100M_PIN_RST_I);
    GPIOA_ModeCfg(BSP_DEI60F100M_PIN_RST_I, GPIO_ModeIN_PU_SMT); // Pull-up input

    // PA14: as hspi pause
    GPIOA_ResetBits(BSP_DEI60F100M_PIN_HSPI_TX_PAUSE);
    GPIOA_ModeCfg(BSP_DEI60F100M_PIN_HSPI_TX_PAUSE, GPIO_Highspeed_PP_8mA);

    // PA15: as ok out
    GPIOA_SetBits(BSP_DEI60F100M_PIN_OK_O);
    GPIOA_ModeCfg(BSP_DEI60F100M_PIN_OK_O, GPIO_Highspeed_PP_8mA);
}

void bsp_dei60f100_spitest_init(void)
{
    // PA13: as SCLK
    GPIOA_ResetBits(BSP_DEI60F100M_PIN_RST_I);
    GPIOA_ModeCfg(BSP_DEI60F100M_PIN_RST_I, GPIO_Highspeed_PP_16mA);

    // PA14: as MOSI
    GPIOA_ResetBits(BSP_DEI60F100M_PIN_HSPI_TX_PAUSE);
    GPIOA_ModeCfg(BSP_DEI60F100M_PIN_HSPI_TX_PAUSE, GPIO_Highspeed_PP_8mA);

    // PA15: as MISO
    GPIOA_SetBits(BSP_DEI60F100M_PIN_OK_O);
    GPIOA_ModeCfg(BSP_DEI60F100M_PIN_OK_O, GPIO_Highspeed_PP_8mA);
}


void bsp_dei60f100_out_err(bool en)
{
    //if (en) {
    //    GPIOA_SetBits(BSP_DEI60F100M_PIN_ERR_O);
    //} else {
    //    GPIOA_ResetBits(BSP_DEI60F100M_PIN_ERR_O);
    //}
}

void bsp_dei60f100_out_ok(bool en)
{
    if (en) {
        GPIOA_SetBits(BSP_DEI60F100M_PIN_OK_O);
    } else {
        GPIOA_ResetBits(BSP_DEI60F100M_PIN_OK_O);
    }
}

void bsp_dei60f100_out_pause(bool en)
{
    if (en) {
        GPIOA_SetBits(BSP_DEI60F100M_PIN_HSPI_TX_PAUSE);
    } else {
        GPIOA_ResetBits(BSP_DEI60F100M_PIN_HSPI_TX_PAUSE);
    }
}

bool bsp_dei60f100_get_reset_in(void)
{
    return GPIOA_ReadPortPin(BSP_DEI60F100M_PIN_RST_I);
}

bool fpga_spi_read(uint32_t addr, uint8_t *buf, uint8_t len)
{
    uint8_t wr_buf[8];

    if (!buf || len > 128) {
        log_printf(" wrong parameter\r\n");
        return false;
    }

    // read 8 bytes at addr=0
    wr_buf[0] = 0xf5; // soft cs
    wr_buf[1] = 0xC1;  // auto inc
    wr_buf[2] = addr & 0xff; // addr
    wr_buf[3] = (addr >> 8) & 0xff;
    wr_buf[4] = (addr >> 16) & 0xff;
    wr_buf[5] = (addr >> 24) & 0xff;
    wr_buf[6] = (len - 1) << 1;

    //GPIOA_ModeCfg(BSP_DEI60F100M_PIN_OK_O, GPIO_Highspeed_PP_8mA);
    R8_SPI0_CTRL_MOD |= RB_SPI_MISO_OE;
    SPI0_MasterTrans(wr_buf, 7);

    R8_SPI0_CTRL_MOD &= ~RB_SPI_MISO_OE;
    //GPIOA_ModeCfg(BSP_DEI60F100M_PIN_OK_O, GPIO_ModeIN_Floating);
    SPI0_MasterRecv(buf, len);

    return true;
}

bool fpga_spi_write(uint32_t addr, uint8_t *buf, uint8_t len)
{
    uint8_t wr_buf[8];

    if (!buf || len > 128) {
        log_printf(" wrong parameter\r\n");
        return false;
    }
    // read 8 bytes at addr=0
    wr_buf[0] = 0xf5; // soft cs
    wr_buf[1] = 0x33;  // auto inc, auto reload
    wr_buf[2] = addr & 0xff; // addr
    wr_buf[3] = (addr >> 8) & 0xff;
    wr_buf[4] = (addr >> 16) & 0xff;
    wr_buf[5] = (addr >> 24) & 0xff;
    wr_buf[6] = (len - 1) << 1;

    R8_SPI0_CTRL_MOD |= RB_SPI_MISO_OE;
    SPI0_MasterTrans(wr_buf, 7);
    SPI0_MasterTrans(buf, len);

    return true;
}
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    uint32_t i;

    /* Configure GPIO In/Out default/safe state for the board */
    bsp_gpio_init();
    bsp_dei60f100_init();

    /* Init BSP (MCU Frequency & SysTick) */
    bsp_init(FREQ_SYS);
    /* Configure serial debugging for printf()/log_printf()... */
    log_init(&log_buf);
#if(defined DEBUG)
    /* Configure serial debugging for printf()/log_printf()... */
    UART1_init(UART1_BAUD, FREQ_SYS);
#endif
    log_printf("\r\n");

    bsp_wait_ms_delay(1000);
    /******************************************/
    /* Start Synchronization between 2 Boards */
    /* J3 MOSI(PA14) & J3 SCS(PA12) signals   */
    /******************************************/
    //if(bsp_switch() == 0)
    //{
    //	board_role_rx = false;
    //	i = bsp_sync2boards(PA14, PA12, BSP_BOARD2);
    //}
    //else
    //{
    //	board_role_rx = true;
    //	i = bsp_sync2boards(PA14, PA12, BSP_BOARD1);
    //}
    //if(i > 0)
    //	log_printf("SYNC %08d\r\n", i);
    //else
    //	log_printf("SYNC Err Timeout\r\n");

    board_role_rx = false;
    spi_test_mode = false;
    log_time_reinit(); // Reinit log time after synchro
    /* Test Synchronization to be checked with Oscilloscope/LA */
    bsp_uled_on();
    bsp_uled_off();
    /****************************************/
    /* End Synchronization between 2 Boards */
    /****************************************/
#if 1
    bsp_dei60f100_spitest_init();
    uint32_t spi_ram_addr = 0x400;

    log_printf("Start SPI test\r\n");
    SPI0_MasterDefInit();
    SPI0_DataMode(Mode0_HighBitINFront);

    R8_SPI0_CTRL_MOD |= RB_SPI_2WIRE_MOD; // two wire mode
    R8_SPI0_CTRL_MOD &= ~RB_SPI_MOSI_OE;
    R8_SPI0_CLOCK_DIV = 3;

    uint8_t buf[32] = { 0 }, buf_rd[32] = { 0 };

#ifdef DEI60F225SOM
    uint8_t cbsel;
    uint32_t *p32; 

    fpga_spi_read(0, buf, 8);
    p32 = (uint32_t *)buf;
    log_printf(" 0x%08x, 0x%08x\r\n", p32[0], p32[1]);

    if (p32[0] == 0x12345678) {
        log_printf("->FPGA bitstream ready, CBSEL to decide mode\r\n");
        log_printf("   - 2'b10: rx mode (FPGA->CH569)\r\n"); 
        log_printf("   - 2'b00: tx mode (CH569->FPGA)\r\n");
        log_printf("   - 2'b?1: spi test mode\r\n");
        // read cbsel0/cbsel1 to decided test mode
        fpga_spi_read(8, &cbsel, 1);
        log_printf("-> Read CBSEL = 0x%x\r\n", cbsel);

        if (cbsel == 2) 
            board_role_rx = true;

        if (cbsel & 1)
            spi_test_mode = true;

    }
#endif

    while (spi_test_mode) {
        bsp_wait_ms_delay(500);

        for (int i = 0; i < sizeof(buf); i++) {
            buf[i] = i + 0x5a;
            buf_rd[i] = 0;
        }
        for (int i = 8; i < sizeof(buf); i++) {
            fpga_spi_write(spi_ram_addr, buf, i + 1);
            fpga_spi_read(spi_ram_addr, buf_rd, i + 1);
            if (memcmp(buf, buf_rd, i + 1) != 0) {

                GPIOA_SetBits(BSP_DEI60F100M_PIN_HSPI_TX_PAUSE);
                GPIOA_ResetBits(BSP_DEI60F100M_PIN_HSPI_TX_PAUSE);
                log_printf("[NG] i=%d\r\n", i);
                for (int j = 0; j < i + 1; j++) {
                    printf("  #%d: %02x\r\n", j, buf_rd[j]);
                }
                break;
            }

            log_printf("[OK]\r\n");
        }

        //log_printf(" %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\r\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

        //spi_ram_addr = spi_ram_addr < 8 ?spi_ram_addr + 1 : 0;
        bsp_wait_ms_delay(500);
    }
    bsp_dei60f100_init();
#endif

    log_printf("Start HSPI test\r\n");
    if (board_role_rx == false) {
        log_printf("HSPI_Tx(Board2 Bot TX) @ChipID=%02X\r\n", R8_CHIP_ID);
    } else {
        log_printf("HSPI_Rx(Board1 Top RX) @ChipID=%02X\r\n", R8_CHIP_ID);
    }
    log_printf("FSYS=%d\r\n", FREQ_SYS);

    if (board_role_rx == false) { // TX Mode
        log_printf("HSPI TX Data_Size=%d\r\n", Data_Size);
        HSPI_DoubleDMA_Init(HSPI_HOST, RB_HSPI_DAT32_MOD, TX_DMA_Addr0, TX_DMA_Addr1, DMA_Tx_Len);

        log_printf("Write RAMX 0x20020000 32K\r\n");
        // Write RAMX
        for (i = 0; i < 8192; i++) { // 8192*4 = 32K
            *(uint32_t *)(0x20020000 + i * 4) = (i + 0x55555555);
        }

        log_printf("Wait 100us\r\n"); /* Wait 100us RX is ready before to TX */
        bsp_wait_us_delay(100);

        log_printf("Start Tx 32K data\r\n");

        bsp_uled_on();
        HSPI_DMA_Tx();
        bsp_uled_off();

        while (HSPI_TX_End_Flag == 0);

        log_printf("Tx 32K data suc\r\n");
        log_printf("Wait 20ms before blink loop\r\n");
        bsp_wait_ms_delay(20);
        while (1) {
            //if (!bsp_dei60f100_get_reset_in()) {
            //    log_printf("Reset after 500ms\r\n");
            //
            //    bsp_wait_ms_delay(500);
            //    SYS_ResetExecute();
            //}
            //if (bsp_ubtn()) {
            if (!bsp_dei60f100_get_reset_in()) {
                bsp_uled_on();

                // Write RAMX
                for (i = 0; i < 8192; i++) { // 8192*4 = 32K
                    *(uint32_t *)(0x20020000 + i * 4) = i + 0x55555555;
                }
                HSPI_TX_End_Flag = 0;
                log_printf("Start Tx 32K\r\n");
                R32_HSPI_UDF0 = DMA_Tx_Len; // UDF0
                R32_HSPI_UDF1 = DMA_Tx_Len; // UDF1
                HSPI_DMA_Tx();

                while (HSPI_TX_End_Flag == 0);
                log_printf("Tx 32K OK\r\n");

                blink_ms = BLINK_ULTRA_FAST;
            } else {
                blink_ms = BLINK_SLOW;
            }
            bsp_uled_on();
            bsp_wait_ms_delay(blink_ms);
            bsp_uled_off();
            bsp_wait_ms_delay(blink_ms);
        }
    } else { // RX mode
        log_printf("HSPI RX Data_Size=%d\r\n", Data_Size);

        log_printf("Clear RAMX 32K\r\n");
        for (i = 0; i < 8192; i++) { // 8192*4 = 32K
            *(uint32_t *)(0x20020000 + i * 4) = 0;
        }
        log_printf("DMA_RX_Addr0[0]=0x%08X [8191]=0x%08X\r\n",
                   ((uint32_t *)RX_DMA_Addr0)[0], ((uint32_t *)RX_DMA_Addr0)[8191]);

        HSPI_RX_End_Flag = 0;  // Receive completion flag
        HSPI_RX_End_Err = 0; // 0=No Error else >0 Error code
        HSPI_DoubleDMA_Init(HSPI_DEVICE, RB_HSPI_DAT32_MOD, RX_DMA_Addr0, RX_DMA_Addr1, 0);

        //R8_HSPI_CFG |= RB_HSPI_HW_ACK;
        //R8_HSPI_AUX |= RB_HSPI_ACK_TX_MOD;

        int Rx_Verify_Flag = 0;
        uint32_t rx_loop_n = 0;

        while (1) {
            uint32_t cur_rx_cnt = 0;

            //log_printf("Wait Rx\r\n");
            bsp_dei60f100_out_pause(false);

            while (HSPI_RX_End_Flag == 0) {
                bsp_wait_ms_delay(10);
                //bsp_wait_ms_delay(1000);
                //if (Rx_Cnt != cur_rx_cnt) {
                //    cur_rx_cnt = Rx_Cnt;
                //    //log_printf(" rx_cnt = %d\r\n", Rx_Cnt);
                //} else {
                //    // reset number
                //    R8_HSPI_RX_SC = 0;
                //    log_printf(".\r\n");
                //}
            }
            //log_printf("Rx_End\r\n");

            Rx_Verify_Flag = HSPI_RX_End_Err;
            if (HSPI_RX_End_Err == 0) {
                //verify
                //for (i = 0; i < 8192; i++) { // 8192*4 = 32K
                //    uint32_t val_u32;
                //
                //    val_u32 = *(uint32_t *)(0x20020000 + i * 4);
                //    if (val_u32 != (i + 0x55555555)) {
                //        Rx_Verify_Flag = 1;
                //        //log_printf("Verify err Rx_End_Err=%d\r\n", HSPI_RX_End_Err);
                //        log_printf("Err addr=0x%08X val=0x%08X expected val=0x%08X\r\n",
                //                   (0x20020000 + i * 4), val_u32, (i + 0x55555555));
                //        break;
                //    }
                //}
            } else {
                if (HSPI_RX_End_Err & 1) {
                    //log_printf(" CRC error\r\n");
                }
                if (HSPI_RX_End_Err & 2) {
                    //log_printf(" SEQ error\r\n");
                    //for (int i = 0; i < 8; i++) {
                    //    log_printf("seq: %x, %x, %x, %x\r\n",rx_seq_list[4 * i + 0],rx_seq_list[4 * i + 1],rx_seq_list[4 * i + 2],rx_seq_list[4 * i + 3]);
                    //}
                }
            }

            //if (Rx_Verify_Flag == 0) {
            //    log_printf("Verify suc\r\n");
            //    log_printf("DMA_RX_Addr0[0]=0x%08X [8191]=0x%08X\r\n",
            //               ((uint32_t *)RX_DMA_Addr0)[0], ((uint32_t *)RX_DMA_Addr0)[8191]);
            //}

            //log_printf("Clear RAMX 32K\r\n");
            for (i = 0; i < 8192; i++) { // 8192*4 = 32K
                *(uint32_t *)(0x20020000 + i * 4) = 0;
            }
            //log_printf("DMA_RX_Addr0[0]=0x%08X [8191]=0x%08X\r\n",
            //           ((uint32_t *)RX_DMA_Addr0)[0], ((uint32_t *)RX_DMA_Addr0)[8191]);

            HSPI_RX_End_Err = 0;
            HSPI_RX_End_Flag = 0;
            bsp_dei60f100_out_pause(false);

            // reinit rx
            {
                // Error reset Rx_Verify_Flag
                Rx_Verify_Flag = 0;
                //log_printf("HSPI_Init\r\n");
                //HSPI_DoubleDMA_Init(HSPI_DEVICE, RB_HSPI_DAT32_MOD, RX_DMA_Addr0, RX_DMA_Addr1, 0);

                //R8_HSPI_CFG |= RB_HSPI_HW_ACK;
                //R8_HSPI_AUX |= RB_HSPI_ACK_TX_MOD;


                rx_seq_list_idx = 0;
                memset(rx_seq_list, 0, sizeof(rx_seq_list));
            }

            if (rx_loop_n % 128 == 0) {
                log_printf("loop: %d, rx_cnt: %u (err: crc/seq=%u/%u)\r\n", rx_loop_n, rx_cnt_total, rx_cnt_err_crc, rx_cnt_err_seq);
            }

            //printf("\r%d", rx_loop_n);
            rx_loop_n++;
        }
    }
}

void HSPI_IRQHandler_ReInitRX(bool en)
{
    R32_HSPI_RX_ADDR0 = RX_DMA_Addr0;
    R32_HSPI_RX_ADDR1 = RX_DMA_Addr1;
    addr_cnt = 0;
    Rx_Cnt = 0;
    //if (en) {
    //    R8_HSPI_CTRL |= RB_HSPI_ENABLE | RB_HSPI_DMA_EN;
    //} else {
    //    R8_HSPI_CTRL &= ~(RB_HSPI_ENABLE | RB_HSPI_DMA_EN);
    //}
}

/*********************************************************************
 * @fn      HSPI_IRQHandler
 *
 * @brief   This function handles HSPI exception.
 *
 * @return  none
 */
__attribute__((interrupt("WCH-Interrupt-fast")))void HSPI_IRQHandler(void)
{

    bsp_dei60f100_out_ok(false); //
    /**************/
    /** Transmit **/
    /**************/
    if (R8_HSPI_INT_FLAG & RB_HSPI_IF_T_DONE) { // Single packet sending completed
        bsp_uled_on();
        R8_HSPI_INT_FLAG = RB_HSPI_IF_T_DONE;  // Clear Interrupt
        if (board_role_rx == false) { // TX Mode
            Tx_Cnt++;
            addr_cnt++;

            // log_printf("Tx_Cnt=%d addr_cnt=%d\r\n", Tx_Cnt, addr_cnt);
            if (Tx_Cnt < 64) { // Send 32K (64*512 bytes)
                if (addr_cnt % 2) {
                    R32_HSPI_TX_ADDR0 += DMA_Tx_Len0 * 2;
                } else {
                    R32_HSPI_TX_ADDR1 += DMA_Tx_Len0 * 2;
                }
                // bsp_wait_us_delay(1); // Send Data each 1us (to let time to the Device/Receiver to retrieve the packet)
                R8_HSPI_CTRL |= RB_HSPI_SW_ACT;  // Software, trigger to send
            } else { // Send completed
                R32_HSPI_TX_ADDR0 = TX_DMA_Addr0;
                R32_HSPI_TX_ADDR1 = TX_DMA_Addr1;
                addr_cnt = 0;
                Tx_Cnt = 0;
                HSPI_TX_End_Flag = 1;
            }
        }
    }
    /*************/
    /** Receive **/
    /*************/
    if (R8_HSPI_INT_FLAG & RB_HSPI_IF_R_DONE) { // Single packet reception completed
        uint8_t status = R8_HSPI_RTX_STATUS;

        R8_HSPI_INT_FLAG = RB_HSPI_IF_R_DONE;  // Clear Interrupt

        bsp_dei60f100_out_ok(true);
        bsp_dei60f100_out_ok(false);
        rx_seq_list[rx_seq_list_idx++] = R8_HSPI_RX_SC; // &RB_HSPI_RX_NUM;
        if (rx_seq_list_idx >= sizeof(rx_seq_list) / sizeof(rx_seq_list[0])) {
            rx_seq_list_idx = 0;
        }

        bsp_uled_on();
        //log_printf("R8_HSPI_INT_FLAG=0x%02X\r\n", R8_HSPI_INT_FLAG);

        // The CRC is correct, the received serial number matches (data is received correctly)
        if ((status & (RB_HSPI_CRC_ERR | RB_HSPI_NUM_MIS)) == 0) {
            Rx_Cnt++;
            rx_cnt_total++;
            addr_cnt++;
            if (Rx_Cnt <= 9) { // Receive 32K (64*512 bytes)
                if (addr_cnt % 2) {
                    //R32_HSPI_RX_ADDR0 += 512 * 2;
                } else {
                    //R32_HSPI_RX_ADDR1 += 512 * 2;
                }
            } else {
                // Receive completed
                HSPI_IRQHandler_ReInitRX(true);
                HSPI_RX_End_Flag = 1;
            }
        }
        // Determine whether the CRC is correct
        if (status & RB_HSPI_CRC_ERR) {
            // CRC check err
            // R8_HSPI_CTRL &= ~RB_HSPI_ENABLE;
            //bsp_dei60f100_out_err(true); // output err pulse
            //bsp_dei60f100_out_err(false);
            bsp_dei60f100_out_ok(true);
            bsp_dei60f100_out_ok(false);
            //log_printf("CRC err\r\n");
            HSPI_IRQHandler_ReInitRX(false);
            HSPI_RX_End_Err |= 1;
            HSPI_RX_End_Flag = 1;
            rx_cnt_err_crc++;
        }

        // Whether the received serial number matches, (does not match, modify the packet serial number)
        if (status & RB_HSPI_NUM_MIS) {
            // Mismatch
            //
            //bsp_dei60f100_out_err(true); // output err pulse
            //bsp_dei60f100_out_err(false);
            //bsp_dei60f100_out_err(true); // output err pulse
            //bsp_dei60f100_out_err(false);
            bsp_dei60f100_out_ok(true);
            bsp_dei60f100_out_ok(false);
            bsp_dei60f100_out_ok(true);
            bsp_dei60f100_out_ok(false);
            //log_printf("NUM_MIS err\r\n");
            HSPI_IRQHandler_ReInitRX(false);
            HSPI_RX_End_Err |= 2;
            HSPI_RX_End_Flag = 1;

            rx_cnt_err_seq++;
        }

        if (HSPI_RX_End_Flag) {
            bsp_dei60f100_out_pause(true);
        }
    }

    /*
    if(R8_HSPI_INT_FLAG & RB_HSPI_IF_FIFO_OV)
    { // FIFO OV
        R8_HSPI_INT_FLAG = RB_HSPI_IF_FIFO_OV; // Clear Interrupt
        log_printf("FIFO OV\r\n");
        HSPI_IRQHandler_ReInitRX();
        Rx_End_Err |= 4;
        Rx_End_Flag = 1;
    }
    */

    bsp_uled_off();
    bsp_dei60f100_out_ok(true);

    //if (HSPI_RX_End_Flag) {
    //    printf(" rx_len = %d/%d\r\n", R16_HSPI_RX_LEN0, R16_HSPI_RX_LEN1);
    //}
}
