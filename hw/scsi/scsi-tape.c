/*
 * SCSI Tape emulation
 *
 * Copyright (c) 2024 Paul Brook
 *
 * This code is licensed under the LGPL.
 *
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qemu/hw-version.h"
#include "qemu/memalign.h"
#include "hw/scsi/scsi.h"
#include "migration/qemu-file-types.h"
#include "migration/vmstate.h"
#include "hw/scsi/emulation.h"
#include "scsi/constants.h"
#include "sysemu/block-backend.h"
#include "sysemu/blockdev.h"
#include "hw/block/block.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "sysemu/dma.h"
#include "sysemu/sysemu.h"
#include "qemu/cutils.h"
#include "trace.h"
#include "qom/object.h"

#define TYPE_SCSI_TAPE "scsi-tape"
OBJECT_DECLARE_SIMPLE_TYPE(SCSITapeState, SCSI_TAPE)

struct SCSITapeState {
    SCSIDevice qdev;
    uint64_t position;
};

typedef struct SCSITapeReq {
    SCSIRequest req;
    int block_count;
    uint8_t *buf;
    int buflen;
} SCSITapeReq;

static void scsi_free_request(SCSIRequest *req)
{
    SCSITapeReq *r = DO_UPCAST(SCSITapeReq, req, req);

    g_free(r->buf);
}

/* Helper function for command completion with sense.  */
static void scsi_check_condition(SCSITapeReq *r, SCSISense sense)
{
    trace_scsi_tape_check_condition(r->req.tag, sense.key, sense.asc,
                                    sense.ascq);
    scsi_req_build_sense(&r->req, sense);
    scsi_req_complete(&r->req, CHECK_CONDITION);
}

static void scsi_tape_read_data(SCSIRequest *req)
{
    SCSITapeReq *r = DO_UPCAST(SCSITapeReq, req, req);
    SCSITapeState *s = DO_UPCAST(SCSITapeState, qdev, req->dev);
    int buflen = r->buflen;

    if (r->block_count) {
        assert(r->buflen == 0);
        trace_scsi_tape_read_block(s->position);
        int blocksize = s->qdev.blocksize;
        blk_pread(s->qdev.conf.blk, s->position, blocksize, r->buf, 0);
        r->block_count--;
        s->position += blocksize;
        scsi_req_data(&r->req, blocksize);
        return;
    } else if (buflen) {
        trace_scsi_tape_read_data_buf(buflen);
        r->buflen = 0;
        scsi_req_data(&r->req, buflen);
        return;
    }

    /* This also clears the sense buffer for REQUEST SENSE.  */
    scsi_req_complete(&r->req, GOOD);
}

static void scsi_tape_write_data(SCSIRequest *req)
{
    SCSITapeReq *r = DO_UPCAST(SCSITapeReq, req, req);
    int buflen = r->buflen;

    if (buflen) {
        trace_scsi_tape_read_data_buf(buflen);
        r->buflen = 0;
        scsi_req_data(&r->req, buflen);
        return;
    }
    scsi_req_complete(&r->req, GOOD);
}

static int scsi_tape_space(SCSITapeState *s, SCSITapeReq *r)
{
    uint8_t *cmd = r->req.cmd.buf;
    uint8_t code = cmd[1] & 7;
    int count = cmd[4] | (cmd[3] << 8) | (cmd[2] << 16);
    if (count & 0x800000) {
        count |= ~0x7fffff;
    }
    trace_scsi_tape_command_SPACE(code, count);
    if (count == 0) {
        return 0;
    }
    return -1;
}
static int32_t scsi_tape_command(SCSIRequest *req, uint8_t *buf)
{
    SCSITapeReq *r = DO_UPCAST(SCSITapeReq, req, req);
    SCSITapeState *s = DO_UPCAST(SCSITapeState, qdev, req->dev);
    bool defer = false;

    switch (req->cmd.buf[0]) {
    case REWIND:
    case INQUIRY:
    case MODE_SENSE:
    case MODE_SENSE_10:
    case RESERVE:
    case RESERVE_10:
    case RELEASE:
    case RELEASE_10:
    case START_STOP:
    case ALLOW_MEDIUM_REMOVAL:
    case GET_CONFIGURATION:
    case GET_EVENT_STATUS_NOTIFICATION:
    case MECHANISM_STATUS:
    case REQUEST_SENSE:
        break;

    case READ_6:
        defer = true;
        /* fallthrough */
    default:
        if (!blk_is_available(s->qdev.conf.blk)) {
            scsi_check_condition(r, SENSE_CODE(NO_MEDIUM));
            return 0;
        }
        break;
    }

    if (defer) {
        r->buflen = 4096;
    } else {
        /*
         * FIXME: we shouldn't return anything bigger than 4k, but the code
         * requires the buffer to be as big as req->cmd.xfer in several
         * places.  So, do not allow CDBs with a very large ALLOCATION
         * LENGTH.  The real fix would be to modify scsi_read_data and
         * dma_buf_read, so that they return data beyond the buflen
         * as all zeros.
         */
        if (req->cmd.xfer > 65536) {
            goto illegal_request;
        }
        r->buflen = MAX(4096, req->cmd.xfer);
    }

    r->buf = blk_blockalign(s->qdev.conf.blk, r->buflen);

    memset(r->buf, 0, r->buflen);
    switch (req->cmd.buf[0]) {
    case TEST_UNIT_READY:
        assert(blk_is_available(s->qdev.conf.blk));
        break;
    case REWIND:
        trace_scsi_tape_command_REWIND();
        s->position = 0;
        break;
#if 0
    case INQUIRY:
        buflen = scsi_disk_emulate_inquiry(req, outbuf);
        if (buflen < 0) {
            goto illegal_request;
        }
        break;
#endif
    case REQUEST_SENSE:
        r->buflen = scsi_convert_sense(NULL, 0, r->buf, r->buflen,
                                       (req->cmd.buf[1] & 1) == 0);
        if (r->buflen < 0) {
            goto illegal_request;
        }
        break;
    case MODE_SELECT:
        trace_scsi_tape_command_MODE_SELECT(r->req.cmd.xfer);
        break;
    case MODE_SELECT_10:
        trace_scsi_tape_command_MODE_SELECT_10(r->req.cmd.xfer);
        break;
    case READ_6:
        if ((req->cmd.buf[1] & 0x01) == 0) {
            /* We only support fixed block sizes */
            goto illegal_request;
        }
        r->block_count = req->cmd.xfer / s->qdev.blocksize;
        trace_scsi_tape_command_READ(r->block_count);
        r->buflen = 0;
        return req->cmd.xfer;
    case SPACE:
        if (scsi_tape_space(s, r) < 0) {
            goto illegal_request;
        }
        break;
    default:
        trace_scsi_disk_emulate_command_UNKNOWN(buf[0],
                                                scsi_command_name(buf[0]));
        scsi_check_condition(r, SENSE_CODE(INVALID_OPCODE));
        return 0;
    }
    assert(!r->req.aiocb);
    r->buflen = MIN(r->buflen, req->cmd.xfer);
    if (r->buflen == 0) {
        scsi_req_complete(&r->req, GOOD);
    }
    if (r->req.cmd.mode == SCSI_XFER_TO_DEV) {
        assert(r->buflen == req->cmd.xfer);
        return -r->buflen;
    } else {
        return r->buflen;
    }

illegal_request:
    if (r->req.status == -1) {
        scsi_check_condition(r, SENSE_CODE(INVALID_FIELD));
    }
    return 0;
}

static void scsi_tape_reset(DeviceState *dev)
{
    SCSIDevice *s = SCSI_DEVICE(dev);

    s->scsi_version = 1;
    scsi_device_purge_requests(s, SENSE_CODE(RESET));
}

static void scsi_tape_drained_begin(void *opaque)
{
    SCSITapeState *s = opaque;

    scsi_device_drained_begin(&s->qdev);
}

static void scsi_tape_drained_end(void *opaque)
{
    SCSITapeState *s = opaque;

    scsi_device_drained_end(&s->qdev);
}

static const BlockDevOps scsi_tape_block_ops = {
    //.change_media_cb  = scsi_tape_change_media_cb,
    .drained_begin    = scsi_tape_drained_begin,
    .drained_end      = scsi_tape_drained_end,
    //.is_medium_locked = scsi_cd_is_medium_locked,
    //.is_tray_open     = scsi_cd_is_tray_open,
};

static void scsi_tape_realize(SCSIDevice *dev, Error **errp)
{
    SCSITapeState *s = DO_UPCAST(SCSITapeState, qdev, dev);
    dev->blocksize = 512;
    dev->type = TYPE_TAPE;
    if (!s->qdev.conf.blk) {
        error_setg(errp, "drive property not set");
        return;
    }

    if (blk_is_sg(s->qdev.conf.blk)) {
        error_setg(errp, "unwanted /dev/sg*");
        return;
    }

    blk_set_dev_ops(s->qdev.conf.blk, &scsi_tape_block_ops, s);
}

static uint8_t *scsi_tape_get_buf(SCSIRequest *req)
{
    SCSITapeReq *r = DO_UPCAST(SCSITapeReq, req, req);

    return r->buf;
}

const SCSIReqOps scsi_tape_req_ops = {
    .size         = sizeof(SCSITapeReq),
    .free_req     = scsi_free_request,
    .send_command = scsi_tape_command,
    .read_data    = scsi_tape_read_data,
    .write_data   = scsi_tape_write_data,
    .get_buf      = scsi_tape_get_buf,
    //.load_request = scsi_generic_load_request,
    //.save_request = scsi_generic_save_request,
};

static SCSIRequest *scsi_tape_new_request(SCSIDevice *d, uint32_t tag, uint32_t lun,
                                          uint8_t *buf, void *hba_private)
{
    return scsi_req_alloc(&scsi_tape_req_ops, d, tag, lun, hba_private);
}

static Property scsi_tape_properties[] = {
    //DEFINE_BLOCK_ERROR_PROPERTIES(SCSIDeviceState, conf),
    DEFINE_PROP_DRIVE("drive", SCSITapeState, qdev.conf.blk),
    DEFINE_PROP_END_OF_LIST(),
};

static void scsi_tape_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SCSIDeviceClass *sc = SCSI_DEVICE_CLASS(klass);

    sc->realize      = scsi_tape_realize;
    sc->alloc_req    = scsi_tape_new_request;
    dc->fw_name = "tape";
    dc->desc = "virtual SCSI tape";
    dc->reset = scsi_tape_reset;
    device_class_set_props(dc, scsi_tape_properties);
    //dc->vmsd  = &vmstate_scsi_disk_state;
}

static const TypeInfo scsi_tape_info = {
    .name          = TYPE_SCSI_TAPE,
    .parent        = TYPE_SCSI_DEVICE,
    .class_init    = scsi_tape_class_initfn,
    .instance_size = sizeof(SCSITapeState),
};

static void scsi_tape_register_types(void)
{
    type_register_static(&scsi_tape_info);
}

type_init(scsi_tape_register_types)
