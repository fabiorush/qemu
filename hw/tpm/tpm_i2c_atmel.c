/*
 * MAXIM TPM_I2C_ATMEL I2C RTC+NVRAM
 *
 * Copyright (c) 2009 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GNU GPL v2.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "qemu/main-loop.h"
#include "hw/i2c/i2c.h"
#include "qemu/bcd.h"
#include "sysemu/tpm_backend.h"
#include "tpm_int.h"
#include "qapi/error.h"

#define DEBUG_TIS 1

#define DPRINTF(fmt, ...) do { \
    if (DEBUG_TIS) { \
        printf("%s: %d - " fmt,  __FILE__, __LINE__, ## __VA_ARGS__); \
    } \
} while (0);

/* vendor-specific registers */
#define TPM_TIS_REG_DEBUG                 0xf90

#define TPM_TIS_STS_TPM_FAMILY_MASK         (0x3 << 26)/* TPM 2.0 */
#define TPM_TIS_STS_TPM_FAMILY1_2           (0 << 26)  /* TPM 2.0 */
#define TPM_TIS_STS_TPM_FAMILY2_0           (1 << 26)  /* TPM 2.0 */
#define TPM_TIS_STS_RESET_ESTABLISHMENT_BIT (1 << 25)  /* TPM 2.0 */
#define TPM_TIS_STS_COMMAND_CANCEL          (1 << 24)  /* TPM 2.0 */

#define TPM_TIS_STS_VALID                 (1 << 7)
#define TPM_TIS_STS_COMMAND_READY         (1 << 6)
#define TPM_TIS_STS_TPM_GO                (1 << 5)
#define TPM_TIS_STS_DATA_AVAILABLE        (1 << 4)
#define TPM_TIS_STS_EXPECT                (1 << 3)
#define TPM_TIS_STS_SELFTEST_DONE         (1 << 2)
#define TPM_TIS_STS_RESPONSE_RETRY        (1 << 1)

#define TPM_TIS_BURST_COUNT_SHIFT         8
#define TPM_TIS_BURST_COUNT(X) \
    ((X) << TPM_TIS_BURST_COUNT_SHIFT)

#define TPM_TIS_ACCESS_TPM_REG_VALID_STS  (1 << 7)
#define TPM_TIS_ACCESS_ACTIVE_LOCALITY    (1 << 5)
#define TPM_TIS_ACCESS_BEEN_SEIZED        (1 << 4)
#define TPM_TIS_ACCESS_SEIZE              (1 << 3)
#define TPM_TIS_ACCESS_PENDING_REQUEST    (1 << 2)
#define TPM_TIS_ACCESS_REQUEST_USE        (1 << 1)
#define TPM_TIS_ACCESS_TPM_ESTABLISHMENT  (1 << 0)

#define TPM_TIS_CAP_INTERFACE_VERSION1_3 (2 << 28)
#define TPM_TIS_CAP_INTERFACE_VERSION1_3_FOR_TPM2_0 (3 << 28)
#define TPM_TIS_CAP_DATA_TRANSFER_64B    (3 << 9)
#define TPM_TIS_CAP_DATA_TRANSFER_LEGACY (0 << 9)
#define TPM_TIS_CAP_BURST_COUNT_DYNAMIC  (0 << 8)
#define TPM_TIS_CAP_INTERRUPT_LOW_LEVEL  (1 << 4) /* support is mandatory */
#define TPM_TIS_CAPABILITIES_SUPPORTED1_3 \
    (TPM_TIS_CAP_INTERRUPT_LOW_LEVEL | \
     TPM_TIS_CAP_BURST_COUNT_DYNAMIC | \
     TPM_TIS_CAP_DATA_TRANSFER_64B | \
     TPM_TIS_CAP_INTERFACE_VERSION1_3 | \
     TPM_TIS_INTERRUPTS_SUPPORTED)

#define TPM_TIS_CAPABILITIES_SUPPORTED2_0 \
    (TPM_TIS_CAP_INTERRUPT_LOW_LEVEL | \
     TPM_TIS_CAP_BURST_COUNT_DYNAMIC | \
     TPM_TIS_CAP_DATA_TRANSFER_64B | \
     TPM_TIS_CAP_INTERFACE_VERSION1_3_FOR_TPM2_0 | \
     TPM_TIS_INTERRUPTS_SUPPORTED)

#define TPM_TIS_IFACE_ID_INTERFACE_TIS1_3   (0xf)     /* TPM 2.0 */
#define TPM_TIS_IFACE_ID_INTERFACE_FIFO     (0x0)     /* TPM 2.0 */
#define TPM_TIS_IFACE_ID_INTERFACE_VER_FIFO (0 << 4)  /* TPM 2.0 */
#define TPM_TIS_IFACE_ID_CAP_5_LOCALITIES   (1 << 8)  /* TPM 2.0 */
#define TPM_TIS_IFACE_ID_CAP_TIS_SUPPORTED  (1 << 13) /* TPM 2.0 */
#define TPM_TIS_IFACE_ID_INT_SEL_LOCK       (1 << 19) /* TPM 2.0 */

#define TPM_TIS_IFACE_ID_SUPPORTED_FLAGS1_3 \
    (TPM_TIS_IFACE_ID_INTERFACE_TIS1_3 | \
     (~0u << 4)/* all of it is don't care */)

/* if backend was a TPM 2.0: */
#define TPM_TIS_IFACE_ID_SUPPORTED_FLAGS2_0 \
    (TPM_TIS_IFACE_ID_INTERFACE_FIFO | \
     TPM_TIS_IFACE_ID_INTERFACE_VER_FIFO | \
     TPM_TIS_IFACE_ID_CAP_5_LOCALITIES | \
     TPM_TIS_IFACE_ID_CAP_TIS_SUPPORTED)

// /* Size of NVRAM including both the user-accessible area and the
//  * secondary register area.
//  */
// #define NVRAM_SIZE 64

// /* Flags definitions */
// #define SECONDS_CH 0x80
// #define HOURS_12   0x40
// #define HOURS_PM   0x20
// #define CTRL_OSF   0x20

// #define TPM_I2C_ATMEL(obj) OBJECT_CHECK(TPM_I2C_ATMELState, (obj), TYPE_TPM_I2C_ATMEL)

// typedef struct TPM_I2C_ATMELState {
//     I2CSlave parent_obj;

//     int64_t offset;
//     uint8_t wday_offset;
//     uint8_t nvram[NVRAM_SIZE];
//     int32_t ptr;
//     bool addr_byte;
// } TPM_I2C_ATMELState;

static const VMStateDescription vmstate_tpm_i2c_atmel = {
    .name = "tpm",
    .unmigratable = 1,
//     .name = "tpm_i2c_atmel",
//     .version_id = 2,
//     .minimum_version_id = 1,
//     .fields = (VMStateField[]) {
//         VMSTATE_I2C_SLAVE(parent_obj, TPM_I2C_ATMELState),
//         VMSTATE_INT64(offset, TPM_I2C_ATMELState),
//         VMSTATE_UINT8_V(wday_offset, TPM_I2C_ATMELState, 2),
//         VMSTATE_UINT8_ARRAY(nvram, TPM_I2C_ATMELState, NVRAM_SIZE),
//         VMSTATE_INT32(ptr, TPM_I2C_ATMELState),
//         VMSTATE_BOOL(addr_byte, TPM_I2C_ATMELState),
//         VMSTATE_END_OF_LIST()
//     }
};

uint8_t tx_buffer[100];
uint8_t *tx_idx = tx_buffer;
size_t tx_len = 0;

uint8_t rx_buffer[100];
uint8_t *rx_idx = rx_buffer;
size_t rx_len = 0;

static void tpm_i2c_atmel_event(I2CSlave *i2c, enum i2c_event event)
{
    //TPM_I2C_ATMELState *s = TPM_I2C_ATMEL(i2c);

    switch (event) {
    case I2C_START_RECV:
        /* In h/w, capture happens on any START condition, not just a
         * START_RECV, but there is no need to actually capture on
         * START_SEND, because the guest can't get at that data
         * without going through a START_RECV which would overwrite it.
         */
        DPRINTF("I2C_START_RECV\n");
        rx_idx = rx_buffer;
        break;
    case I2C_START_SEND:
        //s->addr_byte = true;
        DPRINTF("I2C_START_SEND\n");
        tx_idx = tx_buffer;
        break;
    case I2C_FINISH:
        DPRINTF("I2C_FINISH\n");
        if (tx_idx != tx_buffer) {
            DPRINTF("Sending %ld bytes to backend\n", tx_idx - tx_buffer);
            tx_idx = tx_buffer;
        }
        if (rx_idx != rx_buffer) {
            DPRINTF("Receiving %ld bytes from backend\n", rx_idx - rx_buffer);
            rx_idx = rx_buffer;
        }
        break;
    case I2C_NACK:
        DPRINTF("I2C_NACK\n");
        break;
    default:
        DPRINTF("default\n");
        break;
    }
}

static int tpm_i2c_atmel_recv(I2CSlave *i2c)
{
    /*TPM_I2C_ATMELState *s = TPM_I2C_ATMEL(i2c);
    uint8_t res;

    res  = s->nvram[s->ptr];
    inc_regptr(s);
    return res;*/
    // DPRINTF("tpm_i2c_atmel_recv\n");
    rx_idx++;
    return 0;
}

static int tpm_i2c_atmel_send(I2CSlave *i2c, uint8_t data)
{
    #if 0
    TPM_I2C_ATMELState *s = TPM_I2C_ATMEL(i2c);

    if (s->addr_byte) {
        s->ptr = data & (NVRAM_SIZE - 1);
        s->addr_byte = false;
        return 0;
    }
    if (s->ptr < 7) {
        /* Time register. */
        struct tm now;
        qemu_get_timedate(&now, s->offset);
        switch(s->ptr) {
        case 0:
            /* TODO: Implement CH (stop) bit.  */
            now.tm_sec = from_bcd(data & 0x7f);
            break;
        case 1:
            now.tm_min = from_bcd(data & 0x7f);
            break;
        case 2:
            if (data & HOURS_12) {
                int tmp = from_bcd(data & (HOURS_PM - 1));
                if (data & HOURS_PM) {
                    tmp += 12;
                }
                if (tmp % 12 == 0) {
                    tmp -= 12;
                }
                now.tm_hour = tmp;
            } else {
                now.tm_hour = from_bcd(data & (HOURS_12 - 1));
            }
            break;
        case 3:
            {
                /* The day field is supposed to contain a value in
                   the range 1-7. Otherwise behavior is undefined.
                 */
                int user_wday = (data & 7) - 1;
                s->wday_offset = (user_wday - now.tm_wday + 7) % 7;
            }
            break;
        case 4:
            now.tm_mday = from_bcd(data & 0x3f);
            break;
        case 5:
            now.tm_mon = from_bcd(data & 0x1f) - 1;
            break;
        case 6:
            now.tm_year = from_bcd(data) + 100;
            break;
        }
        s->offset = qemu_timedate_diff(&now);
    } else if (s->ptr == 7) {
        /* Control register. */

        /* Ensure bits 2, 3 and 6 will read back as zero. */
        data &= 0xB3;

        /* Attempting to write the OSF flag to logic 1 leaves the
           value unchanged. */
        data = (data & ~CTRL_OSF) | (data & s->nvram[s->ptr] & CTRL_OSF);

        s->nvram[s->ptr] = data;
    } else {
        s->nvram[s->ptr] = data;
    }
    inc_regptr(s);
    #endif
    // DPRINTF("tpm_i2c_atmel_send - data: 0x%02x\n", data);
    *tx_idx++ = data;

    return 0;
}
static void tpm_i2c_atmel_receive_cb(TPMState *s, uint8_t locty,
                               bool is_selftest_done)
{
    TPMTISEmuState *tis = &s->s.tis;
    uint8_t l;

    assert(s->locty_number == locty);

    if (is_selftest_done) {
        for (l = 0; l < TPM_TIS_NUM_LOCALITIES; l++) {
            tis->loc[locty].sts |= TPM_TIS_STS_SELFTEST_DONE;
        }
    }

    qemu_bh_schedule(tis->bh);
}


static void tpm_i2c_atmel_realizefn(DeviceState *dev, Error **errp)
{
    TPMState *s = TPM(dev);
    TPMTISEmuState *tis = &s->s.tis;

    DPRINTF("backend %s\n", s->backend);
    s->be_driver = qemu_find_tpm(s->backend);
    if (!s->be_driver) {
        error_setg(errp, "tpm_i2c_atmel: backend driver with id %s could not be "
                   "found", s->backend);
        return;
    }

    s->be_driver->fe_model = TPM_MODEL_TPM_TIS;

    if (tpm_backend_init(s->be_driver, s, tpm_i2c_atmel_receive_cb)) {
        error_setg(errp, "tpm_i2c_atmel: backend driver with id %s could not be "
                   "initialized", s->backend);
        return;
    }

    if (tis->irq_num > 15) {
        error_setg(errp, "tpm_i2c_atmel: IRQ %d for TPM TIS is outside valid range "
                   "of 0 to 15", tis->irq_num);
        return;
    }

    // tis->bh = qemu_bh_new(tpm_tis_receive_bh, s);

    // isa_init_irq(&s->busdev, &tis->irq, tis->irq_num);

    // memory_region_add_subregion(isa_address_space(ISA_DEVICE(dev)),
    //                             TPM_TIS_ADDR_BASE, &s->mmio);
}

static int tpm_i2c_atmel_do_startup_tpm(TPMState *s)
{
    return tpm_backend_startup_tpm(s->be_driver);
}


static int tpm_i2c_atmel_init(I2CSlave *i2c)
{
    return 0;
}

static void tpm_i2c_atmel_reset(DeviceState *dev)
{
    TPMState *s = TPM(dev);
    TPMTISEmuState *tis = &s->s.tis;
    int c;

    s->be_tpm_version = tpm_backend_get_tpm_version(s->be_driver);

    tpm_backend_reset(s->be_driver);

    tis->active_locty = TPM_TIS_NO_LOCALITY;
    tis->next_locty = TPM_TIS_NO_LOCALITY;
    tis->aborting_locty = TPM_TIS_NO_LOCALITY;

    for (c = 0; c < TPM_TIS_NUM_LOCALITIES; c++) {
        tis->loc[c].access = TPM_TIS_ACCESS_TPM_REG_VALID_STS;
        switch (s->be_tpm_version) {
        case TPM_VERSION_UNSPEC:
            break;
        case TPM_VERSION_1_2:
            tis->loc[c].sts = TPM_TIS_STS_TPM_FAMILY1_2;
            tis->loc[c].iface_id = TPM_TIS_IFACE_ID_SUPPORTED_FLAGS1_3;
            break;
        case TPM_VERSION_2_0:
            tis->loc[c].sts = TPM_TIS_STS_TPM_FAMILY2_0;
            tis->loc[c].iface_id = TPM_TIS_IFACE_ID_SUPPORTED_FLAGS2_0;
            break;
        }
        // tis->loc[c].inte = TPM_TIS_INT_POLARITY_LOW_LEVEL;
        // tis->loc[c].ints = 0;
        tis->loc[c].state = TPM_TIS_STATE_IDLE;

        tis->loc[c].w_offset = 0;
        tpm_backend_realloc_buffer(s->be_driver, &tis->loc[c].w_buffer);
        tis->loc[c].r_offset = 0;
        tpm_backend_realloc_buffer(s->be_driver, &tis->loc[c].r_buffer);
    }

    tpm_i2c_atmel_do_startup_tpm(s);
}

static Property tpm_tis_properties[] = {
    DEFINE_PROP_UINT32("irq", TPMState,
                       s.tis.irq_num, TPM_TIS_IRQ),
    DEFINE_PROP_STRING("tpmdev", TPMState, backend),
    DEFINE_PROP_END_OF_LIST(),
};

static void tpm_i2c_atmel_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = tpm_i2c_atmel_init;
    k->event = tpm_i2c_atmel_event;
    k->recv = tpm_i2c_atmel_recv;
    k->send = tpm_i2c_atmel_send;
    dc->realize = tpm_i2c_atmel_realizefn;
    dc->props = tpm_tis_properties;
    dc->reset = tpm_i2c_atmel_reset;
    dc->vmsd = &vmstate_tpm_i2c_atmel;
}

static const TypeInfo tpm_i2c_atmel_info = {
    // .name          = TYPE_TPM_I2C_ATMEL,
    .name          = TYPE_TPM_TIS,
    .parent        = TYPE_I2C_SLAVE,
    // .instance_size = sizeof(TPM_I2C_ATMELState),
    .instance_size = sizeof(TPMState),
    .class_init    = tpm_i2c_atmel_class_init,
};

static void tpm_i2c_atmel_register_types(void)
{
    type_register_static(&tpm_i2c_atmel_info);
    tpm_register_model(TPM_MODEL_TPM_TIS);
}

type_init(tpm_i2c_atmel_register_types)
