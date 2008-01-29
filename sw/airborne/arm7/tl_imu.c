#include "tl_imu.h"

#include "LPC21xx.h"
#include "interrupt_hw.h" 

uint8_t tl_imu_status;

struct adc_buf buf_gr;

float tl_imu_r;
float tl_imu_hx;
float tl_imu_hy;
float tl_imu_hz;
float tl_imu_z_baro;

static void SPI1_ISR(void) __attribute__((naked));


/* SSPCR0 settings */
#define SSP_DDS  0x07 << 0  /* data size         : 8 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : data captured on first clock transition */  
#define SSP_CPHA 0x00 << 7  /* clock phase       : SCK idles low */
#define SSP_SCR  0x0F << 8  /* serial clock rate : divide by 16  */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  /* slave output disable : don't care when master */

void tl_imu_init(void) {
  adc_buf_channel(ADC_CHANNEL_GR, &buf_gr, DEFAULT_AV_NB_SAMPLE);
  micromag_init();
  tl_baro_init();

  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= 2 << 2 | 2 << 4 | 2 << 6;
  
  /* setup SSP */
  SSPCR0 = SSP_DDS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = 0x20;
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    // address of the ISR 


  tl_imu_r =  0.;
  tl_imu_hx = 1.;
  tl_imu_hy = 0.;
  tl_imu_hz = 1.;

  tl_imu_status = TL_IMU_IDLE;
  
}



void SPI1_ISR(void) {
  ISR_ENTRY();
  switch (tl_imu_status) {
  case TL_IMU_READING_BARO: {
    TlBaroOnSpiIntReading();
    tl_baro_send_req();
    tl_imu_status = TL_IMU_SENDING_BARO_REQ;
  }
    break;
  case TL_IMU_SENDING_BARO_REQ: {
    TlBaroOnSpiIntSending();
    //micromag_read();
    //tl_imu_status = TL_IMU_READING_MICROMAG;
    tl_imu_status = TL_IMU_DATA_AVAILABLE;

  }
    break;
  case TL_IMU_READING_MICROMAG: {
    MicromagOnSpiInt();
    if (micromag_status == MM_DATA_AVAILABLE) {
      tl_imu_status = TL_IMU_DATA_AVAILABLE;
    }
  }
    break;
  }
  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}


