/*
 * Copyright (C) 2026 OpenUAS
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file arch/chibios/modules/core/settings_arch.c
 * Persistent settings low level flash routines for ChibiOS,
 * supporting the STM32F1, STM32F4, STM32F7 and STM32H7 families.
 *
 * The settings are stored in the last flash sector, with the
 * following layout (same scheme as the former bare metal stm32 arch):
 *
 * data          sector_addr
 * data_size     sector_end - FSIZ (uint32)
 * checksum      sector_end - FCHK (uint32)
 *
 * The generic part (layout, checksum, verification) is family
 * independent; only four small primitives (geometry detection,
 * erase, programming of one write unit, cache flush) are implemented
 * per flash controller family. The flash registers are accessed
 * directly (CMSIS definitions provided through the ChibiOS HAL
 * headers), so no HAL/EFL driver configuration is required.
 *
 * Note: while the sector erase is in progress (up to ~2s for a large
 * sector) the CPU stalls on any flash fetch, so settings should only
 * be stored/cleared on the ground.
 */

#include "modules/core/settings.h"

#include <hal.h>
#include <string.h>

#if defined(STM32F1XX) || defined(STM32F4XX) || defined(STM32F7XX) || defined(STM32H7XX)

/** Base address of the internal flash on all supported STM32 families. */
#define PFLASH_BASE 0x08000000UL
/** Offset of the stored data size word, counted back from the sector end. */
#define FSIZ 8
/** Offset of the stored checksum word, counted back from the sector end. */
#define FCHK 4

/* ST flash unlock key sequence, identical on all supported families
 * (defined here only if the CMSIS device header does not provide it) */
#ifndef FLASH_UNLOCK_KEY1
#define FLASH_UNLOCK_KEY1 0x45670123UL
#endif
#ifndef FLASH_UNLOCK_KEY2
#define FLASH_UNLOCK_KEY2 0xCDEF89ABUL
#endif

/**
 * Upper bound for busy polling, far above the worst case erase time.
 * It only turns a (theoretical) stuck flash controller into an error
 * return instead of an infinite loop in the autopilot.
 */
#define PFLASH_WAIT_MAX_POLLS 400000000UL

/** Location and geometry of the settings sector, filled by pflash_detect(). */
struct FlashInfo {
  uint32_t addr;      ///< address of the settings sector
  uint32_t page_size; ///< size of the settings sector
  uint32_t snb;       ///< sector number encoding for the SNB register field (unused on F1)
};

/*
 * Family specific primitives:
 *
 * pflash_detect       locate the last flash sector
 * pflash_erase        erase (only) the settings sector
 * pflash_program_unit program one PFLASH_PROG_SIZE byte aligned unit
 * pflash_cache_flush  make cached flash reads coherent again
 */

#if defined(STM32F1XX)

/** F1 flash programming unit: one 16bit half word. */
#define PFLASH_PROG_SIZE 2

/** All F1 FPEC error flags: programming error, write protection error. */
#define PFLASH_F1_SR_ERRORS (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)

/**
 * Detect the flash geometry and locate the last page.
 *
 * Low/medium density parts (<= 128k) have 1k pages, high density and
 * connectivity line parts have 2k pages. XL density parts (> 512k)
 * are not supported, their upper flash bank has a second FPEC with
 * its own registers.
 *
 * @return 0 on success
 */
static int32_t pflash_detect(struct FlashInfo *flash)
{
  /* flash size in kBytes, from the factory programmed size register */
  uint32_t size_kb = *(volatile const uint16_t *)FLASHSIZE_BASE;
  if ((size_kb < 16) || (size_kb > 512)) { return -1; }

  flash->page_size = (size_kb <= 128) ? 0x400 : 0x800;
  flash->addr = PFLASH_BASE + size_kb * 1024 - flash->page_size;
  flash->snb = 0; /* not used, F1 erases by page address */

  return 0;
}

/**
 * Busy-wait until the flash controller is idle.
 * @return 0 when idle, -1 if the poll limit was exceeded
 */
static int32_t pflash_wait(void)
{
  for (uint32_t i = 0; i < PFLASH_WAIT_MAX_POLLS; i++) {
    if (!(FLASH->SR & FLASH_SR_BSY)) { return 0; }
  }
  return -1;
}

/** Unlock the flash control register (no-op when already unlocked). */
static void pflash_unlock(void)
{
  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = FLASH_UNLOCK_KEY1;
    FLASH->KEYR = FLASH_UNLOCK_KEY2;
  }
}

/** Re-lock the flash control register against accidental writes. */
static void pflash_lock(void)
{
  FLASH->CR |= FLASH_CR_LOCK;
}

/**
 * Erase the settings page via the FPEC page erase sequence.
 * The CPU stalls on any flash fetch while the erase is in progress.
 * @param flash settings sector location, from pflash_detect()
 * @return 0 on success
 */
static int32_t pflash_erase(const struct FlashInfo *flash)
{
  pflash_unlock();
  if (pflash_wait()) { pflash_lock(); return -1; }
  FLASH->SR = PFLASH_F1_SR_ERRORS | FLASH_SR_EOP; /* clear pending flags */

  /* page erase */
  FLASH->CR = FLASH_CR_PER;
  FLASH->AR = flash->addr;
  FLASH->CR |= FLASH_CR_STRT;
  int32_t ret = pflash_wait();
  FLASH->CR &= ~FLASH_CR_PER;
  pflash_lock();

  if (ret || (FLASH->SR & PFLASH_F1_SR_ERRORS)) { return -1; }
  return 0;
}

/**
 * Program one 16bit half word (flash must be unlocked by the caller).
 * @param addr destination address in erased flash, half word aligned
 * @param buf  source of PFLASH_PROG_SIZE bytes
 * @return 0 on success
 */
static int32_t pflash_program_unit(const struct FlashInfo *flash __attribute__((unused)),
                                   uint32_t addr, const uint8_t *buf)
{
  uint16_t half;
  memcpy(&half, buf, 2);

  if (pflash_wait()) { return -1; }
  FLASH->CR = FLASH_CR_PG;
  *(volatile uint16_t *)addr = half;
  __DSB();
  int32_t ret = pflash_wait();
  FLASH->CR &= ~FLASH_CR_PG;

  if (ret || (FLASH->SR & PFLASH_F1_SR_ERRORS)) { return -1; }
  return 0;
}

#elif defined(STM32F4XX) || defined(STM32F7XX)

/** F4/F7 flash programming unit: one 32bit word. */
#define PFLASH_PROG_SIZE 4

/** Error flags common to F4/F7, plus the family specific ones. */
static const uint32_t pflash_sr_errors =
  FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR
#ifdef FLASH_SR_PGSERR /* F4 */
  | FLASH_SR_PGSERR
#endif
#ifdef FLASH_SR_ERSERR /* F7 */
  | FLASH_SR_ERSERR
#endif
#ifdef FLASH_SR_RDERR  /* F42x/F43x, F7 */
  | FLASH_SR_RDERR
#endif
  ;

/**
 * Detect the flash geometry and locate the last sector.
 *
 * Each STM32F4 flash bank: 4x 16k + 1x 64k + n x 128k sectors.
 * STM32F7 single bank devices: 512k parts (F72x/F73x) use the F4
 * sector sizes, larger parts (F74x..F77x) use doubled sector sizes
 * (4x 32k + 1x 128k + n x 256k). 2MB F76x/F77x parts can optionally
 * (nDBANK option cleared) run dual bank with the F4 sector sizes.
 *
 * @return 0 on success
 */
static int32_t pflash_detect(struct FlashInfo *flash)
{
  /* flash size in kBytes, from the factory programmed size register */
  uint32_t size_kb = *(volatile const uint16_t *)FLASHSIZE_BASE;
  if ((size_kb < 256) || (size_kb > 2048)) { return -1; }
  uint32_t total = size_kb * 1024;

#if defined(STM32F4XX)
  const uint32_t small = 0x20000; /* 128k sectors beyond the first 128k of a bank */
#else /* STM32F7XX */
  const uint32_t small = (size_kb > 512) ? 0x40000 : 0x20000;
#endif

  /* dual bank organization:
   * F4: always on 2MB parts, optional (DB1M set) on 1MB F42x/F43x parts
   * F7: optional (nDBANK cleared) on 2MB F76x/F77x parts */
  bool dual_bank = false;
#if defined(STM32F4XX)
  dual_bank = (size_kb == 2048);
#ifdef FLASH_OPTCR_DB1M
  if (FLASH->OPTCR & FLASH_OPTCR_DB1M) { dual_bank = true; }
#endif
#elif defined(FLASH_OPTCR_nDBANK)
  if (!(FLASH->OPTCR & FLASH_OPTCR_nDBANK)) { dual_bank = true; }
#endif

  /* F4 dual bank keeps the single bank sector sizes (each bank is
   * simply half the flash), only F7 dual bank halves them */
#if defined(STM32F4XX)
  uint32_t sector_size = small;
#else /* STM32F7XX */
  uint32_t sector_size = dual_bank ? small / 2 : small;
#endif
  uint32_t bank = dual_bank ? total / 2 : total;
  uint32_t last_offset = total - sector_size;
  /* bank relative offset of the last sector */
  uint32_t bank_offset = last_offset - (dual_bank ? bank : 0);

  /* need at least one full sized sector after the small first sectors,
   * which occupy the first sector_size bytes of the bank */
  if ((bank < 2 * sector_size) || (bank % sector_size)) { return -1; }

  flash->page_size = sector_size;
  flash->addr = PFLASH_BASE + last_offset;

  /* sector index: 4 small + 1 medium sector precede the first full
   * sized sector, dual bank sectors of bank2 are indexed from 12 */
  uint32_t sector = 5 + (bank_offset - sector_size) / sector_size
                    + (dual_bank ? 12 : 0);

  /* SNB field encoding: bank2 sectors (12..) map to 16.. */
  flash->snb = (sector < 12) ? sector : sector + 4;

  return 0;
}

/**
 * Busy-wait until the flash controller is idle.
 * @return 0 when idle, -1 if the poll limit was exceeded
 */
static int32_t pflash_wait(void)
{
  for (uint32_t i = 0; i < PFLASH_WAIT_MAX_POLLS; i++) {
    if (!(FLASH->SR & FLASH_SR_BSY)) { return 0; }
  }
  return -1;
}

/** Unlock the flash control register (no-op when already unlocked). */
static void pflash_unlock(void)
{
  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = FLASH_UNLOCK_KEY1;
    FLASH->KEYR = FLASH_UNLOCK_KEY2;
  }
}

/** Re-lock the flash control register against accidental writes. */
static void pflash_lock(void)
{
  FLASH->CR |= FLASH_CR_LOCK;
}

/**
 * Erase the settings sector (SER + SNB sequence).
 * The CPU stalls on any flash fetch while the erase is in progress
 * (up to ~2s for a large sector).
 * @param flash settings sector location, from pflash_detect()
 * @return 0 on success
 */
static int32_t pflash_erase(const struct FlashInfo *flash)
{
  pflash_unlock();
  if (pflash_wait()) { pflash_lock(); return -1; }
  FLASH->SR = pflash_sr_errors; /* clear pending error flags */

  /* sector erase, 32bit parallelism (PSIZE = x32, VDD > 2.7V) */
  FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_SER |
              ((flash->snb << FLASH_CR_SNB_Pos) & FLASH_CR_SNB_Msk);
  FLASH->CR |= FLASH_CR_STRT;
  int32_t ret = pflash_wait();
  FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
  pflash_lock();

  if (ret || (FLASH->SR & pflash_sr_errors)) { return -1; }
  return 0;
}

/**
 * Program one 32bit word (flash must be unlocked by the caller).
 * @param addr destination address in erased flash, word aligned
 * @param buf  source of PFLASH_PROG_SIZE bytes
 * @return 0 on success
 */
static int32_t pflash_program_unit(const struct FlashInfo *flash __attribute__((unused)),
                                   uint32_t addr, const uint8_t *buf)
{
  uint32_t word;
  memcpy(&word, buf, 4);

  if (pflash_wait()) { return -1; }
  FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
  *(volatile uint32_t *)addr = word;
  __DSB();
  int32_t ret = pflash_wait();
  FLASH->CR &= ~FLASH_CR_PG;

  if (ret || (FLASH->SR & pflash_sr_errors)) { return -1; }
  return 0;
}

#elif defined(STM32H7XX)

/** H7 flash programming unit: one 256bit (32 byte) flash word. */
#define PFLASH_PROG_SIZE 32

/** All relevant H7 bank2 error flags. */
#define PFLASH_H7_SR_ERRORS (FLASH_SR_WRPERR | FLASH_SR_PGSERR | \
                             FLASH_SR_STRBERR | FLASH_SR_INCERR | FLASH_SR_OPERR)

/**
 * Detect the flash geometry and locate the last sector.
 *
 * Only the dual bank 2MB parts (e.g. H743) are supported: two banks
 * of 8x 128k sectors, the settings go in the last sector of bank2,
 * which has its own set of flash control registers.
 *
 * @return 0 on success
 */
static int32_t pflash_detect(struct FlashInfo *flash)
{
  uint32_t size_kb = *(volatile const uint16_t *)FLASHSIZE_BASE;
  if (size_kb != 2048) { return -1; }

  flash->page_size = 0x20000;
  flash->addr = PFLASH_BASE + size_kb * 1024 - flash->page_size;
  flash->snb = 7; /* last sector of bank2 */

  return 0;
}

/**
 * Busy-wait until bank2 is idle and its write buffer is drained.
 * @return 0 when idle, -1 if the poll limit was exceeded
 */
static int32_t pflash_wait(void)
{
  for (uint32_t i = 0; i < PFLASH_WAIT_MAX_POLLS; i++) {
    if (!(FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW | FLASH_SR_WBNE))) { return 0; }
  }
  return -1;
}

/** Unlock the bank2 flash control register (no-op when already unlocked). */
static void pflash_unlock(void)
{
  if (FLASH->CR2 & FLASH_CR_LOCK) {
    FLASH->KEYR2 = FLASH_UNLOCK_KEY1;
    FLASH->KEYR2 = FLASH_UNLOCK_KEY2;
  }
}

/** Re-lock the bank2 flash control register against accidental writes. */
static void pflash_lock(void)
{
  FLASH->CR2 |= FLASH_CR_LOCK;
}

/**
 * Erase the settings sector (last sector of bank2).
 * Bank2 has its own control registers and erasing it does not stall
 * code fetched from bank1; still treat this as a ground-only operation.
 * @param flash settings sector location, from pflash_detect()
 * @return 0 on success
 */
static int32_t pflash_erase(const struct FlashInfo *flash)
{
  pflash_unlock();
  if (pflash_wait()) { pflash_lock(); return -1; }
  FLASH->CCR2 = PFLASH_H7_SR_ERRORS; /* clear pending error flags */

  /* sector erase, 32bit write parallelism (PSIZE = x32) */
  FLASH->CR2 = FLASH_CR_PSIZE_1 | FLASH_CR_SER |
               ((flash->snb << FLASH_CR_SNB_Pos) & FLASH_CR_SNB_Msk);
  FLASH->CR2 |= FLASH_CR_START;
  int32_t ret = pflash_wait();
  FLASH->CR2 &= ~(FLASH_CR_SER | FLASH_CR_SNB);
  pflash_lock();

  if (ret || (FLASH->SR2 & PFLASH_H7_SR_ERRORS)) { return -1; }
  return 0;
}

/**
 * Program one 256bit flash word (flash must be unlocked by the caller).
 * All 8 words must be written before the controller starts programming.
 * @param addr destination address in erased flash, 32 byte aligned
 * @param buf  source of PFLASH_PROG_SIZE bytes
 * @return 0 on success
 */
static int32_t pflash_program_unit(const struct FlashInfo *flash __attribute__((unused)),
                                   uint32_t addr, const uint8_t *buf)
{
  if (pflash_wait()) { return -1; }
  FLASH->CR2 = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
  __DSB();

  /* fill the write buffer with the complete flash word */
  for (int i = 0; i < 8; i++) {
    uint32_t word;
    memcpy(&word, buf + 4 * i, 4);
    ((volatile uint32_t *)addr)[i] = word;
  }
  __DSB();
  int32_t ret = pflash_wait();
  FLASH->CR2 &= ~FLASH_CR_PG;

  if (ret || (FLASH->SR2 & PFLASH_H7_SR_ERRORS)) { return -1; }
  return 0;
}

#endif /* family specific primitives */

/**
 * Make cached flash reads coherent again after erase/program, so the
 * verification reads hit the actual flash content instead of stale
 * cache lines (which could false-fail or, worse, false-pass).
 *
 * F4: reset the ART accelerator caches, their only invalidation
 * mechanism (permitted only while the caches are disabled).
 * F7/H7: invalidate the affected L1 data cache lines, if a data
 * cache is present and enabled at runtime.
 */
static void pflash_cache_flush(const struct FlashInfo *flash __attribute__((unused)))
{
#if defined(STM32F4XX)
  /* reset the ART accelerator caches (only allowed while disabled) */
  uint32_t acr = FLASH->ACR;
  FLASH->ACR = acr & ~(FLASH_ACR_DCEN | FLASH_ACR_ICEN);
  FLASH->ACR |= FLASH_ACR_DCRST | FLASH_ACR_ICRST;
  FLASH->ACR &= ~(FLASH_ACR_DCRST | FLASH_ACR_ICRST);
  FLASH->ACR = acr;
#endif
#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
  /* F7/H7 cores: invalidate the L1 data cache lines of the sector */
  if (SCB->CCR & SCB_CCR_DC_Msk) {
    SCB_InvalidateDCache_by_Addr((void *)flash->addr, flash->page_size);
  }
#endif
  __DSB();
  __ISB();
}

/*
 * Generic part, common to all families.
 */

/**
 * Software CRC-32 (polynomial 0x04C11DB7, MSB first) over a byte range.
 *
 * Deliberately bitwise and table free: the settings blob is tiny, so a
 * 1k lookup table or the hardware CRC unit (extra clock and peripheral
 * dependency) would buy nothing.
 *
 * @param ptr  start address (RAM or flash)
 * @param size number of bytes
 * @return CRC-32 of the range
 */
static uint32_t pflash_checksum(uint32_t ptr, uint32_t size)
{
  uint32_t crc = 0xFFFFFFFFUL;
  for (uint32_t i = 0; i < size; i++) {
    crc ^= ((uint32_t)(*(volatile const uint8_t *)(ptr + i))) << 24;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x80000000UL) { crc = (crc << 1) ^ 0x04C11DB7UL; }
      else { crc <<= 1; }
    }
  }
  return crc;
}

/**
 * Size of the trailer block at the sector end holding data size and
 * checksum: at least one programming unit, so the trailer never shares
 * a write unit with the data (on H7 it is one full 32 byte flash word).
 */
#define PFLASH_TRAILER ((FSIZ > PFLASH_PROG_SIZE) ? FSIZ : PFLASH_PROG_SIZE)

/**
 * Maximum usable settings data size.
 * @return sector size minus the trailer block
 */
static uint32_t pflash_max_size(const struct FlashInfo *flash)
{
  return flash->page_size - PFLASH_TRAILER;
}

/**
 * ChibiOS GCC linker symbols used to compute the end of the firmware
 * image in flash: load address of the initialized data section plus its
 * size (the ChibiOS linker rules place .data last in the flash image).
 */
extern uint8_t __textdata_base__, __data_base__, __data_end__;

/**
 * Locate the settings sector and make sure the firmware image does
 * not reach into it, so the driver can never erase its own code as
 * the firmware grows.
 * @param[out] flash filled with the settings sector location
 * @return 0 on success, -1 if unsupported geometry or unsafe overlap
 */
static int32_t pflash_get(struct FlashInfo *flash)
{
  if (pflash_detect(flash)) { return -1; }

  uint32_t image_end = (uint32_t)&__textdata_base__ +
                       ((uint32_t)&__data_end__ - (uint32_t)&__data_base__);
  if (image_end > flash->addr) { return -1; }

  return 0;
}

/**
 * Program a byte buffer as a sequence of programming units, padding the
 * last partial unit with 0xFF (the erased state, so the padding does
 * not disturb anything). Flash must be unlocked by the caller.
 * @param flash settings sector location
 * @param dst   destination flash address, unit aligned
 * @param src   source address in RAM
 * @param size  number of bytes to program
 * @return 0 on success
 */
static int32_t pflash_program_buffer(const struct FlashInfo *flash,
                                     uint32_t dst, uint32_t src, uint32_t size)
{
  uint8_t unit[PFLASH_PROG_SIZE];

  for (uint32_t i = 0; i < size; i += PFLASH_PROG_SIZE) {
    uint32_t n = size - i;
    if (n >= PFLASH_PROG_SIZE) {
      memcpy(unit, (const void *)(src + i), PFLASH_PROG_SIZE);
    } else {
      memset(unit, 0xFF, PFLASH_PROG_SIZE);
      memcpy(unit, (const void *)(src + i), n);
    }
    if (pflash_program_unit(flash, dst + i, unit)) { return -1; }
  }
  return 0;
}

/**
 * Erase the settings sector and verify word by word that it reads back
 * fully erased (flash erases can fail silently).
 * @param flash settings sector location
 * @return 0 on success
 */
static int32_t pflash_erase_verified(const struct FlashInfo *flash)
{
  if (pflash_erase(flash)) { return -1; }

  pflash_cache_flush(flash);

  /* verify erase */
  for (uint32_t i = 0; i < flash->page_size; i += 4) {
    if ((*(volatile uint32_t *)(flash->addr + i)) != 0xFFFFFFFFUL) { return -1; }
  }
  return 0;
}

/**
 * Store a settings blob: erase the sector, program data and trailer,
 * then read everything back for verification (a false "stored OK" is
 * worse than an error).
 * @param flash  settings sector location
 * @param src    RAM address of the data
 * @param size   data size in bytes
 * @param chksum CRC-32 of the data, stored in the trailer
 * @return 0 on success, -1 erase/program failed, -2 data mismatch,
 *         -3 size word mismatch, -4 checksum word mismatch
 */
static int32_t pflash_program_bytes(const struct FlashInfo *flash,
                                    uint32_t src,
                                    uint32_t size,
                                    uint32_t chksum)
{
  uint32_t i;
  int32_t ret;

  /* erase, return with error if not successful */
  if (pflash_erase_verified(flash)) { return -1; }

  pflash_unlock();

  /* write data */
  ret = pflash_program_buffer(flash, flash->addr, src, size);

  /* write trailer: 0xFF padding, then size and checksum in the last 8 bytes */
  if (ret == 0) {
    uint8_t trailer[PFLASH_TRAILER];
    memset(trailer, 0xFF, PFLASH_TRAILER);
    memcpy(&trailer[PFLASH_TRAILER - FSIZ], &size, 4);
    memcpy(&trailer[PFLASH_TRAILER - FCHK], &chksum, 4);
    ret = pflash_program_buffer(flash, flash->addr + flash->page_size - PFLASH_TRAILER,
                                (uint32_t)trailer, PFLASH_TRAILER);
  }

  pflash_lock();
  if (ret) { return -1; }

  pflash_cache_flush(flash);

  /* verify data */
  for (i = 0; i < size; i++) {
    if ((*(volatile uint8_t *)(flash->addr + i)) != (*(const uint8_t *)(src + i))) { return -2; }
  }
  if (*(volatile uint32_t *)(flash->addr + flash->page_size - FSIZ) != size) { return -3; }
  if (*(volatile uint32_t *)(flash->addr + flash->page_size - FCHK) != chksum) { return -4; }

  return 0;
}

/**
 * Store the settings in the last flash sector.
 *
 * Blocking call: while the sector erase is in progress the CPU may
 * stall on flash fetches (up to ~2s), so only use it on the ground.
 *
 * @param ptr  RAM address of the settings data
 * @param size data size in bytes
 * @return 0 on success, -1 no usable/safe sector, -2 invalid size,
 *         other negative values from pflash_program_bytes()
 */
int32_t persistent_write(void *ptr, uint32_t size)
{
  struct FlashInfo flash;
  if (pflash_get(&flash)) { return -1; }
  if ((size > pflash_max_size(&flash)) || (size == 0)) { return -2; }

  return pflash_program_bytes(&flash,
                              (uint32_t)ptr,
                              size,
                              pflash_checksum((uint32_t)ptr, size));
}

/**
 * Load the settings from flash after validating the stored size and
 * checksum, so stale or corrupt data is never loaded.
 *
 * @param ptr  RAM destination for the settings data
 * @param size expected data size in bytes (must equal the stored size)
 * @return 0 on success, -1 no usable/safe sector, -2 invalid size,
 *         -3 stored size mismatch (e.g. sector erased or layout changed),
 *         -4 checksum mismatch
 */
int32_t persistent_read(void *ptr, uint32_t size)
{
  struct FlashInfo flash;
  uint32_t i;

  /* check parameters */
  if (pflash_get(&flash)) { return -1; }
  if ((size > pflash_max_size(&flash)) || (size == 0)) { return -2; }

  /* check consistency */
  if (size != *(volatile uint32_t *)(flash.addr + flash.page_size - FSIZ)) { return -3; }
  if (pflash_checksum(flash.addr, size) !=
      *(volatile uint32_t *)(flash.addr + flash.page_size - FCHK)) {
    return -4;
  }

  /* copy data */
  for (i = 0; i < size; i++) {
    *(uint8_t *)((uint32_t)ptr + i) = *(volatile uint8_t *)(flash.addr + i);
  }

  return 0;
}

/**
 * Erase the settings sector, invalidating any stored settings
 * (a following persistent_read() will fail with -3).
 * @return 0 on success
 */
int32_t persistent_clear(void)
{
  struct FlashInfo flash;
  if (pflash_get(&flash)) { return -1; }

  return pflash_erase_verified(&flash);
}

#else /* unsupported MCU family: dummy implementation so it still links */

/*
 * All dummies return an error; persistent_read() failing means
 * settings_init() falls back to the airframe file defaults instead
 * of silently loading garbage.
 */

int32_t persistent_write(void *ptr __attribute__((unused)), uint32_t size __attribute__((unused)))
{
  return -1;
}

int32_t persistent_read(void *ptr __attribute__((unused)), uint32_t size __attribute__((unused)))
{
  return -1;
}

int32_t persistent_clear(void)
{
  return -1;
}

#endif
