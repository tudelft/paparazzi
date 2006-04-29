/*  $Id$
 *
 * Copyright (C) 2003-2005  Pascal Brisset, Antoine Drouin
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

/** \brief handling of arm7 inter mcu link
 *  
 */

#ifndef LINK_MCU_HW_H
#define LINK_MCU_HW_H

#ifdef FBW
#define TX_BUF ((uint8_t*)&link_mcu_from_fbw_msg)
#define RX_BUF ((uint8_t*)&link_mcu_from_ap_msg)
#define EndTransmit() {}
#endif

#ifdef AP 
#define TX_BUF ((uint8_t*)&link_mcu_from_ap_msg)
#define RX_BUF ((uint8_t*)&link_mcu_from_fbw_msg)
#define EndTransmit() \
  if (link_mcu_tx_idx == FRAME_LENGTH)			\
      SpiDisableTxi();
#endif

#define FRAME_LENGTH sizeof(struct link_mcu_msg)

#define LinkMcuTransmit() {						\
    while (link_mcu_tx_idx < FRAME_LENGTH		\
	   && bit_is_set(SSPSR, TNF)) {					\
      SpiSend(TX_BUF[link_mcu_tx_idx]);	\
      link_mcu_tx_idx++;						\
    }			\
    EndTransmit() \
}

#define LinkMcuReceive() {						\
    while ( bit_is_set(SSPSR, RNE)) {					\
      SpiRead(RX_BUF[link_mcu_rx_idx]);	\
      link_mcu_rx_idx++;						\
    }									\
  }

#define LinkMcuStart() { \
  LinkMcuTransmit();  /* fill fifo */ \
  SpiEnableTxi();     /* enable tx fifo half empty interrupt */ \
  SpiEnableRti();     /* enable rx timeout interrupt         */ \
}

#endif /* LINK_MCU_HW_H */
