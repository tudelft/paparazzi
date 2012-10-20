/*
 * Copyright (C) 2005-2012 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file arch/stm32/mcu_periph/spi_arch.c
 * Handling of SPI hardware for STM32.
 */

#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/f1/dma.h>

#include "mcu_periph/spi.h"

/*
 * @file spi_arch.c
 *
 * SPI Master code.
 * 
 * This file manages the SPI implementation how it appears to the chip.
 * The public "API" that is used across the modules has different ideas about the
 * numbers used in the spi structures (spi_periph). 
 * 
 * This means that from the outside, a spi_periph 2 may be mapped to SPI2, even though it's
 * not the primary spi peripheral to use. Alternatively, it may as well be spi0 (mcu_periph/spi.c)
 * which connects to the IMU (SPI2), instead of spi2.
 *
 * See the "spix_arch_init()" functions to see where the mapping occurs.
 * 
 * This does require modifications in the makefiles, because the correct arch_init needs to be called
 * for the selection of aspirin v2.1 for example.
 */

#ifdef SPI_MASTER

void spi_rw(struct spi_periph* p, struct spi_transaction  * _trans);

static void process_dma_interrupt( struct spi_periph *spi );

struct spi_periph_dma {
  u32 spi;
  u32 spidr;
  u32 dma;
  u8  rx_chan;
  u8  tx_chan;
  u8  nvic_irq;
};

struct spi_periph_dma spi0_dma;
struct spi_periph_dma spi1_dma;
struct spi_periph_dma spi2_dma;

// spi dma end of rx handler
// XXX: should be provided by libopencm3?
// void dma1_channel4_isr(void);

// SPI2 Slave Selection

// This mapping is related to the mapping of spi(x) structures in the modules and not 
// necessarily to the identifiers as they appear to the processor. 
// The IMU on Lisam2 for example is assigned to the SPI2 bus, but we continue to use
// the mapping to spi(x) structures as in modules here. The actual mapping of pins 
// occurs in "arch_init".
// What this means is that we're effectively 'locking':
// SPI2 to spi2
// SPI1 to spi1
// SPI3 to spi0

#define SPI_SLAVE0_PORT GPIOA
#define SPI_SLAVE0_PIN GPIO15

#define SPI_SLAVE1_PORT GPIOA
#define SPI_SLAVE1_PIN GPIO4

#define SPI_SLAVE2_PORT GPIOB
#define SPI_SLAVE2_PIN GPIO12

static inline void SpiSlaveUnselect(uint8_t slave)
{
  switch(slave) {
#if USE_SPI0
    case 0:
      GPIO_BSRR(SPI_SLAVE0_PORT) = SPI_SLAVE0_PIN;
      break;
#endif // USE_SPI_SLAVE0
#if USE_SPI1
    case 1:
      GPIO_BSRR(SPI_SLAVE1_PORT) = SPI_SLAVE1_PIN;
      break;
#endif //USE_SPI_SLAVE1
#if USE_SPI2
    case 2:
      GPIO_BSRR(SPI_SLAVE2_PORT) = SPI_SLAVE2_PIN;
      break;
#endif //USE_SPI_SLAVE2

    default:
      break;
  }
}


static inline void SpiSlaveSelect(uint8_t slave)
{
  switch(slave) {
#if USE_SPI0
    case 0:
      GPIO_BRR(SPI_SLAVE0_PORT) = SPI_SLAVE0_PIN;
      break;
#endif // USE_SPI_SLAVE0
#if USE_SPI1
    case 1:
      GPIO_BRR(SPI_SLAVE1_PORT) = SPI_SLAVE1_PIN;
      break;
#endif //USE_SPI_SLAVE1
#if USE_SPI2
    case 2:
      GPIO_BRR(SPI_SLAVE2_PORT) = SPI_SLAVE2_PIN;
      break;
#endif //USE_SPI_SLAVE2
    default:
      break;
  }
}

static void spi_arch_int_enable( struct spi_periph *spi ) {
  // Enable DMA1 channel4 IRQ Channel ( SPI RX)
  nvic_set_priority( ((struct spi_periph_dma *)spi->dma)->nvic_irq, 0);
  nvic_enable_irq( ((struct spi_periph_dma *)spi->dma)->nvic_irq );
}

static void spi_arch_int_disable( struct spi_periph *spi ) {
  // Disable DMA1 channel4 IRQ Channel ( SPI RX)
  nvic_disable_irq( ((struct spi_periph_dma *)spi->dma)->nvic_irq );
}

void spi_init_slaves(void) {
#if USE_SPI0
  SpiSlaveUnselect(0);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, SPI_SLAVE0_PIN);
#endif

#if USE_SPI1
  SpiSlaveUnselect(1);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, SPI_SLAVE1_PIN);
#endif

#if USE_SPI2

  //FIXME: do remapping
  //GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //Slave2 is on JTDO pin, so disable JTAG DP
#endif
}


/**
 *  These functions map the publically available "spi" structures to 
 *  specific pins on this processor
 */
#if USE_SPI0
void spi0_arch_init(void) {

  // Enable SPI3 Periph and gpio clocks -------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_SPI3EN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);

  // Configure GPIOs: SCK, MISO and MOSI  --------------------------------
  gpio_set_mode(GPIO_BANK_SPI3_SCK, GPIO_MODE_OUTPUT_50_MHZ,
	        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI3_SCK |
	                                        GPIO_SPI3_MISO |
	                                        GPIO_SPI3_MOSI);

  // reset SPI
  spi_reset(SPI3);

  // Disable SPI peripheral
  spi_disable(SPI3);

  // configure SPI
  spi_init_master(SPI3, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI3);
  spi_set_nss_high(SPI3);

  // Enable SPI3 periph.
  spi_enable(SPI3);

  // Enable SPI_3 DMA clock ---------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA2EN);

  spi0.dma = &spi0_dma;
  spi0_dma.spi = SPI3;
  spi0_dma.spidr = (u32)&SPI3_DR;
  spi0_dma.dma = DMA2;
  spi0_dma.rx_chan = DMA_CHANNEL1;
  spi0_dma.tx_chan = DMA_CHANNEL2;
  spi0_dma.nvic_irq = NVIC_DMA2_CHANNEL1_IRQ;

  spi0.trans_insert_idx = 0;
  spi0.trans_extract_idx = 0;
  spi0.status = SPIIdle;
  
  spi_arch_int_enable( &spi0 );
}
#endif

#if USE_SPI1
void spi1_arch_init(void) {

  // Enable SPI1 Periph and gpio clocks -------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB1ENR_SPI1EN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);

  // Configure GPIOs: SCK, MISO and MOSI  --------------------------------
  gpio_set_mode(GPIO_BANK_SPI1_SCK, GPIO_MODE_OUTPUT_50_MHZ,
	        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_SCK |
	                                        GPIO_SPI1_MISO |
	                                        GPIO_SPI1_MOSI);

  // reset SPI
  spi_reset(SPI1);

  // Disable SPI peripheral
  spi_disable(SPI1);

  // configure SPI
  spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI1);
  spi_set_nss_high(SPI1);

  // Enable SPI1 periph.
  spi_enable(SPI1);

  // Enable SPI_1 DMA clock ---------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);

  spi1.dma = &spi1_dma;
  spi1_dma.spi = SPI1;
  spi1_dma.spidr = (u32)&SPI1_DR;
  spi1_dma.dma = DMA1;
  spi1_dma.rx_chan = DMA_CHANNEL4;
  spi1_dma.tx_chan = DMA_CHANNEL5;
  spi1_dma.nvic_irq = NVIC_DMA1_CHANNEL2_IRQ;

  spi1.trans_insert_idx = 0;
  spi1.trans_extract_idx = 0;
  spi1.status = SPIIdle;
  
  spi_arch_int_enable( &spi1 );
}
#endif

#if USE_SPI2
void spi2_arch_init(void) {

  // Enable SPI2 Periph and gpio clocks -------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_SPI2EN);

  // Configure GPIOs: SCK, MISO and MOSI  --------------------------------
  gpio_set_mode(GPIO_BANK_SPI2_SCK, GPIO_MODE_OUTPUT_50_MHZ,
	        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI2_SCK |
	                                        GPIO_SPI2_MOSI);

  gpio_set_mode(GPIO_BANK_SPI2_MISO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
          GPIO_SPI2_MISO);

  // reset SPI
  //spi_reset(SPI2);

  // Disable SPI peripheral
  //spi_disable(SPI2);

  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_OTGFSEN);


  SpiSlaveUnselect(2);
  gpio_set(GPIOB, SPI_SLAVE2_PIN);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, SPI_SLAVE2_PIN);


  // configure SPI
  SPI2_I2SCFGR = 0;
  spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  //spi_enable_crc( SPI2 );
  //spi_set_next_tx_from_buffer( SPI2 );
  //spi_set_full_duplex_mode( SPI2 );
  //SPI2_CRCPR = 0x07;

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI2);
  spi_set_nss_high(SPI2);
  //spi_enable_ss_output(SPI2);

  // Enable SPI_2 DMA clock ---------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);

  // Enable SPI2 periph.
  spi_enable(SPI2);

  // SpiSlaveUnselect( &spi2 );

  // spi_enable_ss_output( SPI2 );

  spi2.dma = &spi2_dma;
  spi2_dma.spi = SPI2;
  spi2_dma.spidr = (u32)&SPI2_DR;
  spi2_dma.dma = DMA1;
  spi2_dma.rx_chan = DMA_CHANNEL4;
  spi2_dma.tx_chan = DMA_CHANNEL5;
  spi2_dma.nvic_irq = NVIC_DMA1_CHANNEL4_IRQ;

  spi2.trans_insert_idx = 0;
  spi2.trans_extract_idx = 0;
  spi2.status = SPIIdle;
  
  spi_arch_int_enable( &spi2 );
}
#endif


//FIXME: get rid off slave0 and take spi periph as parameter
// GT: done
void spi_rw(struct spi_periph* p, struct spi_transaction  * _trans)
{
  struct spi_periph_dma *dma;

  // Store local copy to notify of the results
  _trans->status = SPITransRunning;
  p->status = SPIRunning;
  SpiSlaveSelect(_trans->slave_idx);

  dma = p->dma;

  // Rx_DMA_Channel configuration ------------------------------------
  dma_channel_reset( dma->dma, dma->rx_chan );
  dma_set_peripheral_address(dma->dma, dma->rx_chan, (u32)dma->spidr);
  dma_set_memory_address(dma->dma, dma->rx_chan, (uint32_t)_trans->input_buf);
  dma_set_number_of_data(dma->dma, dma->rx_chan, _trans->input_length);
  dma_set_read_from_peripheral(dma->dma, dma->rx_chan);
  //dma_disable_peripheral_increment_mode(dma->dma, dma->rx_chan);
  dma_enable_memory_increment_mode(dma->dma, dma->rx_chan);
  dma_set_peripheral_size(dma->dma, dma->rx_chan, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(dma->dma, dma->rx_chan, DMA_CCR_MSIZE_8BIT);
  //dma_set_mode(dma->dma, dma->rx_chan, DMA_???_NORMAL);
  dma_set_priority(dma->dma, dma->rx_chan, DMA_CCR_PL_VERY_HIGH);

  // SPI Tx_DMA_Channel configuration ------------------------------------
  dma_channel_reset(dma->dma, dma->tx_chan);
  dma_set_peripheral_address(dma->dma, dma->tx_chan, (u32)dma->spidr);
  dma_set_memory_address(dma->dma, dma->tx_chan, (uint32_t)_trans->output_buf);
  dma_set_number_of_data(dma->dma, dma->tx_chan, _trans->output_length);
  dma_set_read_from_memory(dma->dma, dma->tx_chan);
  //dma_disable_peripheral_increment_mode(dma->dma, dma->tx_chan);
  dma_enable_memory_increment_mode(dma->dma, dma->tx_chan);
  dma_set_peripheral_size(dma->dma, dma->tx_chan, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(dma->dma, dma->tx_chan, DMA_CCR_MSIZE_8BIT);
  //dma_set_mode(dma->dma, dma->tx_chan, DMA_???_NORMAL);
  dma_set_priority(dma->dma, dma->tx_chan, DMA_CCR_PL_MEDIUM);

  // Enable SPI Rx request
  spi_enable_rx_dma(dma->spi);
  // Enable dma->dma rx channel
  dma_enable_channel(dma->dma, dma->rx_chan);

  // Enable SPI Tx request
  spi_enable_tx_dma(dma->spi);
  // Enable dma->dma tx Channel
  dma_enable_channel(dma->dma, dma->tx_chan);

  // Enable dma->dma rx Channel Transfer Complete interrupt
  dma_enable_transfer_complete_interrupt(dma->dma, dma->rx_chan);
}

bool_t spi_submit(struct spi_periph* p, struct spi_transaction* t)
{
  uint8_t idx;
  idx = p->trans_insert_idx + 1;
  if (idx >= SPI_TRANSACTION_QUEUE_LEN) idx = 0;
  if (idx == p->trans_extract_idx) {
    t->status = SPITransFailed;
    return FALSE; /* queue full */
  }
  t->status = SPITransPending;
  // FIXME: still needed?
  //*(t->ready) = 0;
  //Disable interrupts to avoid race conflict with end of DMA transfer interrupt
  //FIXME
  //__disable_irq();
  //spi_arch_int_disable( p );
  
  // GT: no copy?  There's a queue implying a copy here...
  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = idx;

  /* if peripheral is idle, start the transaction */
  if (p->status == SPIIdle) {
    spi_rw(p, p->trans[p->trans_extract_idx]);
  }
  //FIXME
  //__enable_irq();
  //spi_arch_int_enable( p );
  return TRUE;
}

#ifdef USE_SPI1
// Accel end of DMA transferred
void dma1_channel2_isr(void)
{
  SpiSlaveUnselect(spi1.trans[spi1.trans_extract_idx]->slave_idx );
  if ((DMA1_ISR & DMA_ISR_TCIF2) != 0) {
    // clear int pending bit
    DMA1_IFCR |= DMA_IFCR_CTCIF2;

    // mark as available
    // FIXME: should only be needed in slave mode...
    //spi_message_received = TRUE;
  }
  process_dma_interrupt( &spi1 );
}
#endif

#ifdef USE_SPI2
// Accel end of DMA transferred
void dma1_channel4_isr(void)
{
  SpiSlaveUnselect(spi2.trans[spi2.trans_extract_idx]->slave_idx );
  if ((DMA1_ISR & DMA_ISR_TCIF4) != 0) {
    // clear int pending bit
    DMA1_IFCR |= DMA_IFCR_CTCIF4;

    // mark as available
    // FIXME: should only be needed in slave mode...
    //spi_message_received = TRUE;
  }
  process_dma_interrupt( &spi2 );
}
#endif

#if USE_SPI0
// Accel end of DMA transferred
void dma2_channel1_isr(void)
{
  SpiSlaveUnselect(spi0.trans[spi0.trans_extract_idx]->slave_idx );
  if ((DMA2_ISR & DMA_ISR_TCIF1) != 0) {
    // clear int pending bit
    DMA2_IFCR |= DMA_IFCR_CTCIF1;

    // mark as available
    // FIXME: should only be needed in slave mode...
    //spi_message_received = TRUE;
  }
  process_dma_interrupt( &spi0 );
}
#endif

// GT: This adds some overhead during an interrupt. Should this be 
// undone and kept as is, even if code is slightly duplicated,
// to reduce processing time in interrupt?
void process_dma_interrupt( struct spi_periph *spi ) {
  struct spi_periph_dma *dma = spi->dma;
  struct spi_transaction *trans = spi->trans[spi->trans_extract_idx];

  // disable DMA Channel
  dma_disable_transfer_complete_interrupt( dma->dma, dma->rx_chan );

  // Disable SPI Rx and TX request
  spi_disable_rx_dma( dma->spi );
  spi_disable_tx_dma( dma->spi );

  // Disable DMA1 rx and tx channels
  dma_disable_channel( dma->dma, dma->rx_chan );
  dma_disable_channel( dma->dma, dma->tx_chan );

  trans->status = SPITransSuccess;
  *(trans->ready) = 1;
  spi->trans_extract_idx++;

  // Check if there is another pending SPI transaction
  if (spi->trans_extract_idx >= SPI_TRANSACTION_QUEUE_LEN)
    spi->trans_extract_idx = 0;
  if (spi->trans_extract_idx == spi->trans_insert_idx)
    spi->status = SPIIdle;
  else
    spi_rw(spi, spi->trans[spi->trans_extract_idx]);
}

#endif /** SPI_MASTER */


/*
 *
 * SPI Slave code
 *
 * FIXME implement it
 *
 */
#ifdef SPI_SLAVE

#endif /* SPI_SLAVE */

