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

#define DEBUG_TIS 0

#define DPRINTF(fmt, ...) do { \
    if (DEBUG_TIS) { \
        printf(fmt, ## __VA_ARGS__); \
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

#define TPM_TIS_NO_DATA_BYTE  0xff

static const VMStateDescription vmstate_tpm_i2c_atmel = {
    .name = "tpm",
    .unmigratable = 1,
};

static uint32_t tpm_i2c_atmel_get_size_from_buffer(const TPMSizedBuffer *sb)
{
    return be32_to_cpu(*(uint32_t *)&sb->buffer[2]);
}

static void tpm_i2c_atmel_show_buffer(const TPMSizedBuffer *sb, const char *string)
{
#ifdef DEBUG_TIS
    uint32_t len, i;

    len = tpm_i2c_atmel_get_size_from_buffer(sb);
    DPRINTF("tpm_tis: %s length = %d\n", string, len);
    for (i = 0; i < len; i++) {
        if (i && !(i % 16)) {
            DPRINTF("\n");
        }
        DPRINTF("%.2X ", sb->buffer[i]);
    }
    DPRINTF("\n");
#endif
}

/*
 * Set the given flags in the STS register by clearing the register but
 * preserving the SELFTEST_DONE and TPM_FAMILY_MASK flags and then setting
 * the new flags.
 *
 * The SELFTEST_DONE flag is acquired from the backend that determines it by
 * peeking into TPM commands.
 *
 * A VM suspend/resume will preserve the flag by storing it into the VM
 * device state, but the backend will not remember it when QEMU is started
 * again. Therefore, we cache the flag here. Once set, it will not be unset
 * except by a reset.
 */
static void tpm_i2c_atmel_sts_set(TPMLocality *l, uint32_t flags)
{
    l->sts &= TPM_TIS_STS_SELFTEST_DONE | TPM_TIS_STS_TPM_FAMILY_MASK;
    l->sts |= flags;
}

/*
 * Send a request to the TPM.
 */
static void tpm_i2c_atmel_tpm_send(TPMState *s, uint8_t locty)
{
    TPMTISEmuState *tis = &s->s.tis;

    tpm_i2c_atmel_show_buffer(&tis->loc[locty].w_buffer, "tpm_tis: To TPM");

    s->locty_number = locty;
    s->locty_data = &tis->loc[locty];

    /*
     * w_offset serves as length indicator for length of data;
     * it's reset when the response comes back
     */
    tis->loc[locty].state = TPM_TIS_STATE_EXECUTION;

    tpm_backend_deliver_request(s->be_driver);
}

/* abort -- this function switches the locality */
// static void tpm_i2c_atmel_abort(TPMState *s, uint8_t locty)
// {
//     TPMTISEmuState *tis = &s->s.tis;

//     tis->loc[locty].r_offset = 0;
//     tis->loc[locty].w_offset = 0;

//     DPRINTF("tpm_tis: tis_abort: new active locality is %d\n", tis->next_locty);

//     /*
//      * Need to react differently depending on who's aborting now and
//      * which locality will become active afterwards.
//      */
//     if (tis->aborting_locty == tis->next_locty) {
//         tis->loc[tis->aborting_locty].state = TPM_TIS_STATE_READY;
//         tpm_tis_sts_set(&tis->loc[tis->aborting_locty],
//                         TPM_TIS_STS_COMMAND_READY);
//         tpm_tis_raise_irq(s, tis->aborting_locty, TPM_TIS_INT_COMMAND_READY);
//     }

//     /* locality after abort is another one than the current one */
//     tpm_tis_new_active_locality(s, tis->next_locty);

//     tis->next_locty = TPM_TIS_NO_LOCALITY;
//     /* nobody's aborting a command anymore */
//     tis->aborting_locty = TPM_TIS_NO_LOCALITY;
// }


static void tpm_i2c_atmel_receive_bh(void *opaque)
{
    TPMState *s = opaque;
    TPMTISEmuState *tis = &s->s.tis;
    uint8_t locty = s->locty_number;

    tpm_i2c_atmel_sts_set(&tis->loc[locty],
                    TPM_TIS_STS_VALID | TPM_TIS_STS_DATA_AVAILABLE);
    tis->loc[locty].state = TPM_TIS_STATE_COMPLETION;
    tis->loc[locty].r_offset = 0;
    tis->loc[locty].w_offset = 0;
    DPRINTF("tpm_i2c_atmel: tpm_i2c_atmel_receive_bh locty [%d]\n", locty);

    // if (TPM_TIS_IS_VALID_LOCTY(tis->next_locty)) {
    //     tpm_i2c_atmel_abort(s, locty);
    // }

}

/*
 * Read a byte of response data
 */
static uint32_t tpm_i2c_atmel_data_read(TPMState *s, uint8_t locty)
{
    TPMTISEmuState *tis = &s->s.tis;
    uint32_t ret = TPM_TIS_NO_DATA_BYTE;
    uint16_t len;

    if ((tis->loc[locty].sts & TPM_TIS_STS_DATA_AVAILABLE)) {
        len = tpm_i2c_atmel_get_size_from_buffer(&tis->loc[locty].r_buffer);

        ret = tis->loc[locty].r_buffer.buffer[tis->loc[locty].r_offset++];
        if (tis->loc[locty].r_offset >= len) {
            /* got last byte */
            tpm_i2c_atmel_sts_set(&tis->loc[locty], TPM_TIS_STS_VALID);
// #ifdef RAISE_STS_IRQ
//             tpm_tis_raise_irq(s, locty, TPM_TIS_INT_STS_VALID);
// #endif
        }
        DPRINTF("tpm_i2c_atmel: tpm_i2c_atmel_data_read byte 0x%02x   [%d]\n",
                ret, tis->loc[locty].r_offset-1);
    } else {
        DPRINTF("tpm_i2c_atmel: !TPM_TIS_STS_DATA_AVAILABLE [%d]\n",
                tis->loc[locty].sts);
    }

    return ret;
}

static void tpm_i2c_atmel_event(I2CSlave *i2c, enum i2c_event event)
{
    TPMState *s = TPM(&(i2c->qdev));
    TPMTISEmuState *tis = &s->s.tis;

    switch (event) {
    case I2C_START_RECV:
        DPRINTF("I2C_START_RECV\n");
        //tis->loc[0].r_offset = 0;
        break;
    case I2C_START_SEND:
        DPRINTF("I2C_START_SEND\n");
        tis->loc[0].w_offset = 0;
        tis->loc[0].r_offset = 0;
        break;
    case I2C_FINISH:
        DPRINTF("I2C_FINISH\n");
        if (tis->loc[0].w_offset) {
            tpm_i2c_atmel_tpm_send(s, 0);
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
    // DPRINTF("tpm_i2c_atmel_recv\n");
    TPMState *s = TPM(&(i2c->qdev));
    return tpm_i2c_atmel_data_read(s, 0);
    // rx_idx++;
    // return 0;
}

static int tpm_i2c_atmel_send(I2CSlave *i2c, uint8_t data)
{
    TPMState *s = TPM(&(i2c->qdev));
    TPMTISEmuState *tis = &s->s.tis;
    // DPRINTF("tpm_i2c_atmel_send - data: 0x%02x\n", data);
    tis->loc[0].w_buffer.buffer[tis->loc[0].w_offset++] = data;
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

    tis->bh = qemu_bh_new(tpm_i2c_atmel_receive_bh, s);

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
