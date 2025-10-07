/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Peter Lawrence
 * Copyright (c) 2024-2025 Kenny Tseng
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

/*
  Theory of operation:

  The M55M1/M5531 USBD peripheral has twenty-five "EP"s, but each is simplex,
  so two collectively (peripheral nomenclature of "EP0" and "EP1") are needed to
  implement USB EP0.  PERIPH_EP0 and PERIPH_EP1 are used by this driver for
  EP0_IN and EP0_OUT respectively.  This leaves up to twenty-three for user usage.
*/

#include "tusb_option.h"

#if CFG_TUD_ENABLED && ((TU_CHECK_MCU(OPT_MCU_M55x)))

#include "device/dcd.h"
#include "NuMicro.h"

// Since TinyUSB doesn't use SOF for now, and this interrupt too often (1ms interval)
// We disable SOF for now until needed later on
#ifndef USE_SOF
#define USE_SOF     0
#endif
//#define printf(...)
/* allocation of USBD RAM for Setup, EP0_IN, and and EP_OUT */
#define PERIPH_SETUP_BUF_BASE  0
#define PERIPH_SETUP_BUF_LEN   8
#define PERIPH_EP0_BUF_BASE    (PERIPH_SETUP_BUF_BASE + PERIPH_SETUP_BUF_LEN)
#define PERIPH_EP0_BUF_LEN     CFG_TUD_ENDPOINT0_SIZE
#define PERIPH_EP1_BUF_BASE    (PERIPH_EP0_BUF_BASE + PERIPH_EP0_BUF_LEN)
#define PERIPH_EP1_BUF_LEN     CFG_TUD_ENDPOINT0_SIZE
#define PERIPH_EP2_BUF_BASE    (PERIPH_EP1_BUF_BASE + PERIPH_EP1_BUF_LEN)

#ifndef PERIPH_MAX_EP
#define PERIPH_MAX_EP          (USBD_MAX_EP)
#endif
#if TU_CHECK_MCU(OPT_MCU_M55x)
  #define USBD_BUF_SIZE          (1536)
#else
  #define USBD_BUF_SIZE          (512)
#endif
enum ep_enum
{
  PERIPH_EP0 = 0,
  PERIPH_EP1 = 1,
  PERIPH_EP2 = 2,
  PERIPH_EP3 = 3,
  PERIPH_EP4 = 4,
  PERIPH_EP5 = 5,
  PERIPH_EP6 = 6,
  PERIPH_EP7 = 7,
  PERIPH_EP8 = 8,
  PERIPH_EP9 = 9,
  PERIPH_EP10 = 10,
  PERIPH_EP11 = 11,
  PERIPH_EP12 = 12,
  PERIPH_EP13 = 13,
  PERIPH_EP14 = 14,
  PERIPH_EP15 = 15,
  PERIPH_EP16 = 16,
  PERIPH_EP17 = 17,
  PERIPH_EP18 = 18,
  PERIPH_EP19 = 19,
  PERIPH_EP20 = 20,
  PERIPH_EP21 = 21,
  PERIPH_EP22 = 22,
  PERIPH_EP23 = 23,
  PERIPH_EP24 = 24,
  PERIPH_EP25 = 25,
  PERIPH_EP26 = 26,
  PERIPH_EP27 = 27,
  PERIPH_EP28 = 28,
  PERIPH_EP29 = 29,
  PERIPH_EP30 = 30,
  PERIPH_EP31 = 31,
  PERIPH_ENUM_MAX = PERIPH_MAX_EP,
};

/* reset by dcd_init(), this is used by dcd_edpt_open() to assign USBD peripheral buffer addresses */
static uint32_t bufseg_addr;

/* used by dcd_edpt_xfer() and the ISR to reset the data sync (DATA0/DATA1) in an EP0_IN transfer */
static bool active_ep0_xfer;

/* RAM table needed to track ongoing transfers performed by dcd_edpt_xfer(), dcd_in_xfer(), and the ISR */
static struct xfer_ctl_t
{
  uint8_t *data_ptr;         /* data_ptr tracks where to next copy data to (for OUT) or from (for IN) */
  // tu_fifo_t * ff; // TODO support dcd_edpt_xfer_fifo API
  union {
    uint16_t in_remaining_bytes; /* for IN endpoints, we track how many bytes are left to transfer */
    uint16_t out_bytes_so_far;   /* but for OUT endpoints, we track how many bytes we've transferred so far */
  };
  uint16_t max_packet_size;  /* needed since device driver only finds out this at runtime */
  uint16_t total_bytes;      /* quantity needed to pass as argument to dcd_event_xfer_complete() (for IN endpoints) */
  uint8_t ep_addr;
} xfer_table[PERIPH_MAX_EP];

/*
  local helper functions
*/

static void usb_attach(void)
{
  USBD->SE0 &= ~USBD_SE0_SE0_Msk;
}

static void usb_detach(void)
{
  USBD->SE0 |= USBD_SE0_SE0_Msk;
}

static inline void usb_memcpy(uint8_t *dest, uint8_t *src, uint16_t size)
{
    uint32_t i = 0ul;

    while (i < size)
    {
        //Prevent this code from being optimized.
        __asm volatile("" : : : "memory");
        dest[i] = src[i];
        i++;
    }
}

static void usb_control_send_zlp(void)
{
  USBD->EP[PERIPH_EP0].CFG |= USBD_CFG_DSQSYNC_Msk;
  USBD->EP[PERIPH_EP0].MXPLD = 0;
}

/* map 8-bit ep_addr into peripheral endpoint index (PERIPH_EP0...) */
static USBD_EP_T *ep_entry(uint8_t ep_addr, bool add)
{
  USBD_EP_T *ep;
  enum ep_enum ep_index;
  struct xfer_ctl_t *xfer;

  for (ep_index = PERIPH_EP0, xfer = &xfer_table[PERIPH_EP0], ep = USBD->EP; 
       ep_index < PERIPH_MAX_EP; 
       ep_index++, xfer++, ep++)
  {
    if (add)
    {
      /* take first peripheral endpoint that is unused */
      if (0 == (ep->CFG & USBD_CFG_STATE_Msk)) return ep;
    }
    else
    {
      /* find a peripheral endpoint that matches ep_addr */
      if (xfer->ep_addr == ep_addr) return ep;
    }
  }

  return NULL;
}

/* perform an IN endpoint transfer; this is called by dcd_edpt_xfer() and the ISR  */
static void dcd_in_xfer(struct xfer_ctl_t *xfer, USBD_EP_T *ep)
{
  uint16_t bytes_now = tu_min16(xfer->in_remaining_bytes, xfer->max_packet_size);

#if 0 // TODO support dcd_edpt_xfer_fifo API
  if (xfer->ff)
  {
    tu_fifo_read_n(xfer->ff, (void *) (USBD_BUF_BASE + ep->BUFSEG), bytes_now);
  }
  else
#endif
  {
    // USB SRAM seems to only support byte access and memcpy could possibly do it by words
    usb_memcpy((uint8_t *)(USBD_BUF_BASE + ep->BUFSEG), xfer->data_ptr, bytes_now);
  }

  ep->MXPLD = bytes_now;
}

/* called by dcd_init() as well as by the ISR during a USB bus reset */
static void bus_reset(void)
{
  USBD->STBUFSEG = PERIPH_SETUP_BUF_BASE;

  for (enum ep_enum ep_index = PERIPH_EP0; ep_index < PERIPH_MAX_EP; ep_index++)
  {
    USBD->EP[ep_index].CFG = 0;
    USBD->EP[ep_index].CFGP = 0;
  }

  /* allocate the default EP0 endpoints */

  USBD->EP[PERIPH_EP0].CFG = USBD_CFG_CSTALL_Msk | USBD_CFG_EPMODE_IN;
  USBD->EP[PERIPH_EP0].BUFSEG = PERIPH_EP0_BUF_BASE;
  xfer_table[PERIPH_EP0].max_packet_size = PERIPH_EP0_BUF_LEN;
  xfer_table[PERIPH_EP0].ep_addr = 0x80;

  USBD->EP[PERIPH_EP1].CFG = USBD_CFG_CSTALL_Msk | USBD_CFG_EPMODE_OUT;
  USBD->EP[PERIPH_EP1].BUFSEG = PERIPH_EP1_BUF_BASE;
  xfer_table[PERIPH_EP1].max_packet_size = PERIPH_EP1_BUF_LEN;
  xfer_table[PERIPH_EP1].ep_addr = 0x00;

  /* USB RAM beyond what we've allocated above is available to the user */
  bufseg_addr = PERIPH_EP2_BUF_BASE;

  /* Reset USB device address */
  USBD->FADDR = 0;

  /* reset EP0_IN flag */
  active_ep0_xfer = false;
}

/* centralized location for USBD interrupt enable bit mask */
enum {
  ENABLED_IRQS = USBD_INTSTS_VBDETIF_Msk | USBD_INTSTS_BUSIF_Msk | USBD_INTSTS_SETUP_Msk |
                 USBD_INTSTS_USBIF_Msk   | (USE_SOF ? USBD_INTSTS_SOFIF_Msk : 0)
};

/*
  TinyUSB API driver implementation
*/

bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;

#ifdef SUPPORT_LPM
  USBD->ATTR = 0x7D0 | USBD_LPMACK;
#else
  USBD->ATTR = 0x7D0;
#endif

  usb_detach();

  bus_reset();

  usb_attach();

  USBD->INTSTS = ENABLED_IRQS;
  USBD->INTEN  = ENABLED_IRQS;

  return true;
}

void dcd_int_enable(uint8_t rhport)
{
  (void) rhport;
  NVIC_EnableIRQ(USBD_IRQn);
}

void dcd_int_disable(uint8_t rhport)
{
  (void) rhport;
  NVIC_DisableIRQ(USBD_IRQn);
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  (void) rhport;
  (void) dev_addr;
  usb_control_send_zlp(); /* SET_ADDRESS is the one exception where TinyUSB doesn't use dcd_edpt_xfer() to generate a ZLP */

  // DCD can only set address after status for this request is complete.
  // do it at dcd_edpt0_status_complete()
}

static void remote_wakeup_delay(void)
{
  // try to delay for 1 ms
  uint32_t count = SystemCoreClock / 1000;
  while(count--) __NOP();
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;
  // Enable PHY before sending Resume('K') state
  USBD->ATTR |= USBD_ATTR_PHYEN_Msk;
  USBD->ATTR |= USBD_ATTR_RWAKEUP_Msk;

  // Per specs: remote wakeup signal bit must be clear within 1-15ms
  remote_wakeup_delay();
  USBD->ATTR &=~USBD_ATTR_RWAKEUP_Msk;
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * p_endpoint_desc)
{
  (void) rhport;

  USBD_EP_T *ep = ep_entry(p_endpoint_desc->bEndpointAddress, true);
  TU_ASSERT(ep);

  /* mine the data for the information we need */
  int const dir = tu_edpt_dir(p_endpoint_desc->bEndpointAddress);
  int const size = tu_edpt_packet_size(p_endpoint_desc);
  tusb_xfer_type_t const type = (tusb_xfer_type_t) p_endpoint_desc->bmAttributes.xfer;
  struct xfer_ctl_t *xfer = &xfer_table[ep - USBD->EP];

  /* allocate buffer from USB RAM */
  ep->BUFSEG = bufseg_addr;
  bufseg_addr += size;
    if(bufseg_addr > USBD_BUF_SIZE)
        printf("RAM\n");
  TU_ASSERT(bufseg_addr <= USBD_BUF_SIZE);

  /* construct USB Configuration Register value and then write it */
  uint32_t cfg = tu_edpt_number(p_endpoint_desc->bEndpointAddress);
  cfg |= (TUSB_DIR_IN == dir) ? USBD_CFG_EPMODE_IN : USBD_CFG_EPMODE_OUT;
  if (TUSB_XFER_ISOCHRONOUS == type)
    cfg |= USBD_CFG_TYPE_ISO;
  ep->CFG = cfg;

  /* make a note of the endpoint size */
  xfer->max_packet_size = size;
  xfer->ep_addr = p_endpoint_desc->bEndpointAddress;
  return true;
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
    USBD_EP_T *ep = ep_entry(ep_addr, false);
    
    if (ep != NULL)
    {
        struct xfer_ctl_t *xfer = &xfer_table[ep - USBD->EP];
        // Clear Endpoint information
        memset((void *)xfer, 0, sizeof(xfer));
        // Clear Endpoint configure
        ep->CFG = 0U;
    }
     // Reconfigures the buffer segmentation for all enabled endpoints.
    bufseg_addr = PERIPH_EP2_BUF_BASE;
    enum ep_enum ep_index;
    struct xfer_ctl_t *xfer;
    for (ep_index = PERIPH_EP2, ep = &USBD->EP[PERIPH_EP2],xfer = &xfer_table[PERIPH_EP2]; ep_index < PERIPH_MAX_EP; ep_index++, ep++,xfer++)
    {
        if (0 == (ep->CFG & USBD_CFG_STATE_Msk)) continue;
        ep->BUFSEG = bufseg_addr;
        bufseg_addr += xfer->max_packet_size;
    }

}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
  memset(&xfer_table[PERIPH_EP2], 0, sizeof(xfer_table) - sizeof(xfer_table[0]) * PERIPH_EP2);
  for (enum ep_enum ep_index = PERIPH_EP2; ep_index < PERIPH_MAX_EP; ep_index++)
  {
    USBD->EP[ep_index].CFG = 0;
    USBD->EP[ep_index].CFGP = 0;
  }
  bufseg_addr = PERIPH_EP2_BUF_BASE;
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
  (void) rhport;
  /* mine the data for the information we need */
  tusb_dir_t dir = tu_edpt_dir(ep_addr);
  USBD_EP_T *ep = ep_entry(ep_addr, false);
  struct xfer_ctl_t *xfer = &xfer_table[ep - USBD->EP];

  /* store away the information we'll needing now and later */
  xfer->data_ptr = buffer;
  // xfer->ff       = NULL; // TODO support dcd_edpt_xfer_fifo API
  xfer->in_remaining_bytes = total_bytes;
  xfer->total_bytes = total_bytes;

  /* for the first of one or more EP0_IN packets in a message, the first must be DATA1 */
  if ( (0x80 == ep_addr) && !active_ep0_xfer ) ep->CFG |= USBD_CFG_DSQSYNC_Msk;

  if (TUSB_DIR_IN == dir)
  {
    dcd_in_xfer(xfer, ep);
  }
  else
  {
    xfer->out_bytes_so_far = 0;
    ep->MXPLD = xfer->max_packet_size;
  }

  return true;
}

#if 0 // TODO support dcd_edpt_xfer_fifo API
bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
{
  (void) rhport;

  /* mine the data for the information we need */
  tusb_dir_t dir = tu_edpt_dir(ep_addr);
  USBD_EP_T *ep = ep_entry(ep_addr, false);
  struct xfer_ctl_t *xfer = &xfer_table[ep - USBD->EP];

  /* store away the information we'll needing now and later */
  xfer->data_ptr = NULL;      // Indicates a FIFO shall be used
  xfer->ff       = ff;
  xfer->in_remaining_bytes = total_bytes;
  xfer->total_bytes = total_bytes;

  if (TUSB_DIR_IN == dir)
  {
    dcd_in_xfer(xfer, ep);
  }
  else
  {
    xfer->out_bytes_so_far = 0;
    ep->MXPLD = xfer->max_packet_size;
  }

  return true;
}
#endif

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  USBD_EP_T *ep = ep_entry(ep_addr, false);
  ep->CFGP |= USBD_CFGP_SSTALL_Msk;
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  USBD_EP_T *ep = ep_entry(ep_addr, false);
  // Clear STALL and reset Packet Identifier(PID)
    ep->CFGP &= ~USBD_CFGP_SSTALL_Msk;
    ep->CFG &= ~USBD_CFG_DSQSYNC_Msk;
}

void dcd_int_handler(uint8_t rhport)
{
  (void) rhport;

  // Mask non-enabled irqs, ex. SOF
  uint32_t u32IntSts = USBD->INTSTS & (ENABLED_IRQS | 0xffffff00);
  uint32_t u32EpIntSts = USBD->EPINTSTS;
#ifdef SUPPORT_LPM
  uint32_t u32BusSts = USBD->ATTR & 0x300f;
#else
  uint32_t u32BusSts = USBD->ATTR & 0xf;
#endif

  if(u32IntSts & USBD_INTSTS_VBDETIF_Msk)
  {
    if(USBD->VBUSDET & USBD_VBUSDET_VBUSDET_Msk)
    {
      /* USB connect */
      USBD->ATTR |= USBD_ATTR_USBEN_Msk | USBD_ATTR_PHYEN_Msk;
    }
    else
    {
      /* USB disconnect */
      USBD->ATTR &= ~USBD_ATTR_USBEN_Msk;
    }
  }

  if(u32IntSts & USBD_INTSTS_BUSIF_Msk)
  {
    if(u32BusSts & USBD_ATTR_USBRST_Msk)
    {
      /* USB bus reset */
      USBD->ATTR |= USBD_ATTR_USBEN_Msk | USBD_ATTR_PHYEN_Msk;

      bus_reset();

      dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
    }

    if(u32BusSts & USBD_ATTR_SUSPEND_Msk)
    {
      /* Enable USB but disable PHY */
      USBD->ATTR &= ~USBD_ATTR_PHYEN_Msk;
      dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
    }

    if(u32BusSts & USBD_ATTR_RESUME_Msk)
    {
      /* Enable USB and enable PHY */
      USBD->ATTR |= USBD_ATTR_USBEN_Msk | USBD_ATTR_PHYEN_Msk;
      dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
    }
  }

  if(u32IntSts & USBD_INTSTS_SETUP_Msk)
  {
    /* clear the data ready flag of control endpoints */
    USBD->EP[PERIPH_EP0].CFGP |= USBD_CFGP_CLRRDY_Msk;
    USBD->EP[PERIPH_EP1].CFGP |= USBD_CFGP_CLRRDY_Msk;

    /* get SETUP packet from USB buffer */
    dcd_event_setup_received(0, (uint8_t *)USBD_BUF_BASE, true);
  }

  if(u32IntSts & USBD_INTSTS_USBIF_Msk)
  {
    if (u32EpIntSts & USBD_EPINTSTS_EPEVT0_Msk) /* PERIPH_EP0 (EP0_IN) event: this is treated separately from the rest */
    {
      USBD->EPINTSTS = USBD_EPINTSTS_EPEVT0_Msk;
      uint16_t const available_bytes = USBD->EP[PERIPH_EP0].MXPLD;

      active_ep0_xfer = (available_bytes == xfer_table[PERIPH_EP0].max_packet_size);

      dcd_event_xfer_complete(0, 0x80, available_bytes, XFER_RESULT_SUCCESS, true);
    }

    /* service PERIPH_EP1 through PERIPH_MAX_EP */
    enum ep_enum ep_index;
    uint32_t mask;
    struct xfer_ctl_t *xfer;
    USBD_EP_T *ep;
    for (ep_index = PERIPH_EP1, mask = USBD_EPINTSTS_EPEVT1_Msk, xfer = &xfer_table[PERIPH_EP1], ep = &USBD->EP[PERIPH_EP1]; ep_index < PERIPH_MAX_EP; ep_index++, mask <<= 1, xfer++, ep++)
    {
      if(u32EpIntSts & mask)
      {
        USBD->EPINTSTS = mask;
        uint8_t const ep_addr = xfer->ep_addr;
        uint16_t const available_bytes = ep->MXPLD;
        bool const out_ep = !(ep_addr & TUSB_DIR_IN_MASK);

        if (out_ep)
        {
          /* copy the data from the PC to the previously provided buffer */
#if 0 // TODO support dcd_edpt_xfer_fifo API
          if (xfer->ff)
          {
            tu_fifo_write_n(xfer->ff, (const void *) (USBD_BUF_BASE + ep->BUFSEG), available_bytes);
          }
          else
#endif
          {
            // USB SRAM seems to only support byte access and memcpy could possibly do it by words
            usb_memcpy(xfer->data_ptr, (uint8_t *)(USBD_BUF_BASE + ep->BUFSEG), available_bytes);
            xfer->data_ptr += available_bytes;
          }

          xfer->out_bytes_so_far += available_bytes;

          /* when the transfer is finished, alert TinyUSB; otherwise, accept more data */
          if ( (xfer->total_bytes == xfer->out_bytes_so_far) || (available_bytes < xfer->max_packet_size) )
            dcd_event_xfer_complete(0, ep_addr, xfer->out_bytes_so_far, XFER_RESULT_SUCCESS, true);
          else
            ep->MXPLD = xfer->max_packet_size;
        }
        else
        {
          /* update the bookkeeping to reflect the data that has now been sent to the PC */
          xfer->in_remaining_bytes -= available_bytes;
          xfer->data_ptr += available_bytes;

          /* if more data to send, send it; otherwise, alert TinyUSB that we've finished */
          if (xfer->in_remaining_bytes)
            dcd_in_xfer(xfer, ep);
          else
            dcd_event_xfer_complete(0, ep_addr, xfer->total_bytes, XFER_RESULT_SUCCESS, true);
        }
      }
    }
  }

  if(u32IntSts & USBD_INTSTS_SOFIF_Msk)
  {
    /* Start-Of-Frame event */
    dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
  }

  /* acknowledge all interrupts */
  USBD->INTSTS = u32IntSts & ENABLED_IRQS;
}

// Invoked when a control transfer's status stage is complete.
// May help DCD to prepare for next control transfer, this API is optional.
void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const * request)
{
  (void) rhport;

  if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
      request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
      request->bRequest == TUSB_REQ_SET_ADDRESS )
  {
    uint8_t const dev_addr = (uint8_t) request->wValue;

    // Setting new address after the whole request is complete
    USBD->FADDR = dev_addr;
  }
}

void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  usb_detach();
}

void dcd_connect(uint8_t rhport)
{
  (void) rhport;
  usb_attach();
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

#endif
