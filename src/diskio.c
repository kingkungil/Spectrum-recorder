

/*-----------------------------------------------------------------------*/
/* Low level disk I/O module for FatFs (ATmega128 SPI SD)               */
/*-----------------------------------------------------------------------*/
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "ff.h"
#include "diskio.h"

/* CS 제어 핀 */
#define SD_CS_LOW()    (PORTC &= ~(1<<PC0))
#define SD_CS_HIGH()   (PORTC |= (1<<PC0))

/* SPI 전송 */
static inline uint8_t spi_transfer(uint8_t data)
{
    SPDR = data;
    while(!(SPSR & (1<<SPIF)));
    return SPDR;
}

/* SPI 초기화 */
static void spi_init(void)
{
    DDRB |= (1<<PB2)|(1<<PB1); // MOSI, SCK 출력
    DDRB &= ~(1<<PB3);         // MISO 입력
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1); // SPI, Master, f_osc/64
    SPSR = 0;
}

/* SD 응답 대기 */
static int sd_wait_ready(uint16_t timeout)
{
    uint16_t t = 0;
    while(spi_transfer(0xFF) != 0xFF){
        if(t++ > timeout) return 0; // 타임아웃
        _delay_us(100);
    }
    return 1;
}

/* SD 명령 전송 */
static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t crc = 0x01;
    if(cmd == 0) crc = 0x95; // CMD0
    if(cmd == 8) crc = 0x87; // CMD8

    SD_CS_LOW();
    sd_wait_ready(5000);
    spi_transfer(0x40 | cmd);
    spi_transfer((arg >> 24) & 0xFF);
    spi_transfer((arg >> 16) & 0xFF);
    spi_transfer((arg >> 8) & 0xFF);
    spi_transfer(arg & 0xFF);
    spi_transfer(crc);

    for(uint8_t i=0; i<10; i++){
        uint8_t r = spi_transfer(0xFF);
        if((r & 0x80) == 0){
            return r;
        }
    }
    return 0xFF; // timeout
}

/* SD 초기화 */
static int sd_init_card(void)
{
    uint8_t i, r;
    spi_init();
    SD_CS_HIGH();
    for(i=0;i<10;i++) spi_transfer(0xFF); // 최소 74클럭

    // CMD0: GO_IDLE_STATE
    for(i=0;i<10;i++){
        r = sd_send_cmd(0,0);
        SD_CS_HIGH(); spi_transfer(0xFF);
        if(r == 0x01) break;
        _delay_ms(10);
    }
    if(r != 0x01) return -1;

    // CMD8: CHECK_VOLTAGE (SD v2)
    r = sd_send_cmd(8,0x1AA);
    SD_CS_HIGH(); spi_transfer(0xFF);

    // ACMD41 대기 (HCS = 1)
    for(uint16_t t=0; t<1000; t++){
        sd_send_cmd(55,0); SD_CS_HIGH(); spi_transfer(0xFF);
        r = sd_send_cmd(41, 0x40000000); SD_CS_HIGH(); spi_transfer(0xFF);
        if(r == 0x00) break;
        _delay_ms(1);
    }
    if(r != 0x00) return -1;

    return 0; // 성공
}

/*-----------------------------------------------------------------------*/
/* FatFs 디스크 함수들                                                   */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status(BYTE pdrv) { return 0; }
DSTATUS disk_initialize(BYTE pdrv) { return (sd_init_card() == 0) ? 0 : STA_NOINIT; }

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    for(UINT i=0;i<count;i++){
        if(sd_send_cmd(17, sector) != 0x00) { SD_CS_HIGH(); return RES_ERROR; }
        while(spi_transfer(0xFF)!=0xFE);
        for(int j=0;j<512;j++) buff[j]=spi_transfer(0xFF);
        spi_transfer(0xFF); spi_transfer(0xFF);
        //SD_CS_HIGH(); 
		//spi_transfer(0xFF);
        buff += 512; 
		sector++;
    }
    SD_CS_HIGH();
    spi_transfer(0xFF); // ★ 중요: CS 해제 후 8클럭 더미 전송
    return RES_OK;
}

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    for(UINT i=0;i<count;i++){
        if(sd_send_cmd(24, sector)!=0x00){ SD_CS_HIGH(); return RES_ERROR; }
        spi_transfer(0xFE);
        for(int j=0;j<512;j++) 
		spi_transfer(buff[j]);
        spi_transfer(0xFF); 
		spi_transfer(0xFF);
        if((spi_transfer(0xFF) & 0x1F)!=0x05){ SD_CS_HIGH(); return RES_ERROR; }
        while(spi_transfer(0xFF)==0); 
		//SD_CS_HIGH(); 
		//spi_transfer(0xFF);
        buff += 512; sector++;
    }
	SD_CS_HIGH();
    spi_transfer(0xFF); // ★ 중요: CS 해제 후 8클럭 더미 전송
    return RES_OK;
}
#endif

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    switch(cmd){
        case CTRL_SYNC: return RES_OK;
        case GET_SECTOR_SIZE: *(WORD*)buff = 512; return RES_OK;
        case GET_SECTOR_COUNT: *(DWORD*)buff = 32768; return RES_OK; // 예: 16MB
        case GET_BLOCK_SIZE: *(DWORD*)buff = 1; return RES_OK;
    }
    return RES_PARERR;
}

DWORD get_fattime(void){
    return ((DWORD)(2025-1980)<<25)|((DWORD)11<<21)|((DWORD)23<<16)|((DWORD)15<<11)|((DWORD)12<<5)|(0>>1);
}
