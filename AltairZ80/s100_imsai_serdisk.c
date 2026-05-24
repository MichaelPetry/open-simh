/*************************************************************************
 *                                                                       *
 * Copyright (c) 2026                                                    *
 *                                                                       *
 * IMSAI Serial Disk Simulator for AltairZ80                             *
 *                                                                       *
 * Simulates the IMSAI serial disk interface as used by IMSAIIO.ASM.     *
 * The CP/M driver communicates over a serial port using a simple         *
 * '@R'/'@W' command protocol to read and write 128-byte sectors.        *
 *                                                                       *
 * I/O Ports (default base 0xB4):                                        *
 *   base+0 (0xB4): Data register (read/write)                           *
 *   base+2 (0xB6): Status register (read only)                          *
 *                                                                       *
 * Status bits:                                                           *
 *   Bit 0: RX data available (1 = byte ready to read)                   *
 *   Bit 2: TX buffer empty (1 = ready to accept byte)                   *
 *                                                                       *
 * Protocol:                                                              *
 *   READ:  '@' 'R' track_hi track_lo sector -> 128 bytes                *
 *   WRITE: '@' 'W' track_hi track_lo sector 128_bytes ->                *
 *                                                                       *
 * Drive select: bit 7 of track_hi (0 = drive 0, 1 = drive 1)            *
 * Track: 15-bit from track_hi[6:0] and track_lo                         *
 * Sector: 8-bit (0-31)                                                  *
 *                                                                       *
 * Geometry: 128 bytes/sector, 32 sectors/track, 2048 tracks             *
 * Capacity: 128 * 32 * 2048 = 8,388,608 bytes (8 MB) per drive         *
 *                                                                       *
 *************************************************************************/

#include "altairz80_defs.h"

/* Debug flags */
#define CMD_MSG     (1 << 0)
#define READ_MSG    (1 << 1)
#define WRITE_MSG   (1 << 2)
#define STATUS_MSG  (1 << 3)
#define VERBOSE_MSG (1 << 4)

/* Device constants */
#define IMSAISD_UNITS           2
#define IMSAISD_SECTOR_SIZE     128
#define IMSAISD_DEFAULT_SPT     32
#define IMSAISD_DEFAULT_TRACKS  2048
#define IMSAISD_DEFAULT_CAP     (IMSAISD_SECTOR_SIZE * IMSAISD_DEFAULT_SPT * IMSAISD_DEFAULT_TRACKS)
#define IMSAISD_IO_PORTS        3       /* base+0, base+1 (unused), base+2 */

/* Unit fields for geometry */
#define IMSAISD_SECTOR_SZ       u5
#define IMSAISD_SPT             u4
#define IMSAISD_TRACKS          u3

/* State machine states */
#define STATE_IDLE              0
#define STATE_PREFIX            1       /* Received '@', waiting for 'R' or 'W' */
#define STATE_READ_TRK_HI      2       /* Waiting for track high byte */
#define STATE_READ_TRK_LO      3       /* Waiting for track low byte */
#define STATE_READ_SECTOR      4       /* Waiting for sector byte */
#define STATE_WRITE_TRK_HI     5       /* Waiting for track high byte */
#define STATE_WRITE_TRK_LO     6       /* Waiting for track low byte */
#define STATE_WRITE_SECTOR     7       /* Waiting for sector byte */
#define STATE_WRITE_DATA       8       /* Receiving 128 bytes of write data */

/* Response buffer size */
#define RESP_BUF_SIZE           256

/* Forward declarations */
static t_stat imsaisd_reset(DEVICE *dptr);
static t_stat imsaisd_attach(UNIT *uptr, CONST char *cptr);
static t_stat imsaisd_detach(UNIT *uptr);
static t_stat imsaisd_set_geom(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
static t_stat imsaisd_show_geom(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
static const char *imsaisd_description(DEVICE *dptr);
int32 imsaisd_io(const int32 port, const int32 io, const int32 data);

extern t_stat set_iobase(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
extern t_stat show_iobase(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
extern uint32 sim_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                               int32 (*routine)(const int32, const int32, const int32),
                               const char* name, uint8 unmap);
extern int32 find_unit_index(UNIT *uptr);

/* Device state */
static int32 imsaisd_state = STATE_IDLE;
static int32 imsaisd_selected_unit = 0;
static uint8 imsaisd_track_hi = 0;
static uint8 imsaisd_track_lo = 0;
static uint16 imsaisd_track = 0;
static uint8 imsaisd_sector = 0;
static int32 imsaisd_data_index = 0;
static uint8 imsaisd_sector_buf[IMSAISD_SECTOR_SIZE];
static uint8 imsaisd_resp_buf[RESP_BUF_SIZE];
static int32 imsaisd_resp_index = 0;
static int32 imsaisd_resp_length = 0;

typedef struct {
    PNP_INFO    pnp;    /* Plug and Play */
} IMSAISD_INFO;

static IMSAISD_INFO imsaisd_info_data = { { 0x0000, 0, 0xB4, IMSAISD_IO_PORTS } };

static UNIT imsaisd_unit[] = {
    { UDATA(NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, IMSAISD_DEFAULT_CAP) },
    { UDATA(NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, IMSAISD_DEFAULT_CAP) }
};

static REG imsaisd_reg[] = {
    { DRDATAD(STATE,    imsaisd_state,          8,  "State machine state")      },
    { DRDATAD(UNIT,     imsaisd_selected_unit,  8,  "Selected unit")            },
    { DRDATAD(TRACK,    imsaisd_track,          16, "Current track")            },
    { DRDATAD(SECTOR,   imsaisd_sector,         8,  "Current sector")           },
    { NULL }
};

#define IMSAISD_NAME    "IMSAI Serial Disk"

static const char *imsaisd_description(DEVICE *dptr) {
    return IMSAISD_NAME;
}

static MTAB imsaisd_mod[] = {
    { MTAB_XTD|MTAB_VDV,    0, "IOBASE",  "IOBASE",
        &set_iobase, &show_iobase, NULL,
        "Sets the I/O base address for " IMSAISD_NAME },
    { MTAB_XTD|MTAB_VUN,    0, "GEOM",    "GEOM",
        &imsaisd_set_geom, &imsaisd_show_geom, NULL,
        "Sets disk geometry T:tracks/N:sectors/S:sectorsize" },
    { 0 }
};

static DEBTAB imsaisd_dt[] = {
    { "CMD",        CMD_MSG,        "Command processing"    },
    { "READ",       READ_MSG,       "Read operations"       },
    { "WRITE",      WRITE_MSG,      "Write operations"      },
    { "STATUS",     STATUS_MSG,     "Status port access"    },
    { "VERBOSE",    VERBOSE_MSG,    "Verbose output"        },
    { NULL,         0                                       }
};

DEVICE imsaisd_dev = {
    "IMSAISD", imsaisd_unit, imsaisd_reg, imsaisd_mod,
    IMSAISD_UNITS, 10, 31, 1, 8, 8,
    NULL, NULL, &imsaisd_reset,
    NULL, &imsaisd_attach, &imsaisd_detach,
    &imsaisd_info_data, (DEV_DISABLE | DEV_DIS | DEV_DEBUG), 0,
    imsaisd_dt, NULL, NULL, NULL, NULL, NULL, &imsaisd_description
};

/* Reset routine */
static t_stat imsaisd_reset(DEVICE *dptr) {
    PNP_INFO *pnp = (PNP_INFO *)dptr->ctxt;

    if (dptr->flags & DEV_DIS) {
        sim_map_resource(pnp->io_base, pnp->io_size, RESOURCE_TYPE_IO,
                        &imsaisd_io, "imsaisd_io", TRUE);
    } else {
        if (sim_map_resource(pnp->io_base, pnp->io_size, RESOURCE_TYPE_IO,
                            &imsaisd_io, "imsaisd_io", FALSE) != 0) {
            sim_printf("%s: error mapping I/O resource at 0x%04x\n",
                      __FUNCTION__, pnp->io_base);
            dptr->flags |= DEV_DIS;
            return SCPE_ARG;
        }
    }

    /* Reset state machine */
    imsaisd_state = STATE_IDLE;
    imsaisd_selected_unit = 0;
    imsaisd_track = 0;
    imsaisd_sector = 0;
    imsaisd_data_index = 0;
    imsaisd_resp_index = 0;
    imsaisd_resp_length = 0;

    return SCPE_OK;
}

/* Attach disk image */
static t_stat imsaisd_attach(UNIT *uptr, CONST char *cptr) {
    const t_stat r = attach_unit(uptr, cptr);
    if (r != SCPE_OK)
        return r;

    /* Determine capacity */
    uptr->capac = sim_fsize(uptr->fileref);
    if (uptr->capac == 0) {
        if (uptr->IMSAISD_SPT == 0) {
            uptr->IMSAISD_SPT = IMSAISD_DEFAULT_SPT;
            uptr->IMSAISD_SECTOR_SZ = IMSAISD_SECTOR_SIZE;
            uptr->IMSAISD_TRACKS = IMSAISD_DEFAULT_TRACKS;
        }
        uptr->capac = (t_addr)uptr->IMSAISD_TRACKS * uptr->IMSAISD_SPT * uptr->IMSAISD_SECTOR_SZ;
    } else {
        if (uptr->IMSAISD_SPT == 0) {
            uptr->IMSAISD_SPT = IMSAISD_DEFAULT_SPT;
            uptr->IMSAISD_SECTOR_SZ = IMSAISD_SECTOR_SIZE;
            uptr->IMSAISD_TRACKS = (uptr->capac + IMSAISD_DEFAULT_SPT * IMSAISD_SECTOR_SIZE - 1)
                                   / (IMSAISD_DEFAULT_SPT * IMSAISD_SECTOR_SIZE);
        }
    }

    sim_debug(VERBOSE_MSG, &imsaisd_dev,
              "IMSAISD%d: attached, capacity=%d, T:%d/N:%d/S:%d\n",
              find_unit_index(uptr), uptr->capac,
              uptr->IMSAISD_TRACKS, uptr->IMSAISD_SPT, uptr->IMSAISD_SECTOR_SZ);

    return SCPE_OK;
}

/* Detach disk image */
static t_stat imsaisd_detach(UNIT *uptr) {
    return detach_unit(uptr);
}

/* Set geometry: T:tracks/N:sectors/S:sectorsize */
static t_stat imsaisd_set_geom(UNIT *uptr, int32 val, CONST char *cptr, void *desc) {
    uint32 tracks, sectors, secsize;
    int result, n;

    if (cptr == NULL)
        return SCPE_ARG;
    if (uptr == NULL)
        return SCPE_IERR;

    result = sscanf(cptr, "T:%u/N:%u/S:%u%n", &tracks, &sectors, &secsize, &n);
    if ((result != 3) || (cptr[n] != 0)) {
        result = sscanf(cptr, "%u/%u/%u%n", &tracks, &sectors, &secsize, &n);
        if ((result != 3) || (cptr[n] != 0))
            return SCPE_ARG;
    }

    uptr->IMSAISD_TRACKS = tracks;
    uptr->IMSAISD_SPT = sectors;
    uptr->IMSAISD_SECTOR_SZ = secsize;
    uptr->capac = tracks * sectors * secsize;
    return SCPE_OK;
}

/* Show geometry */
static t_stat imsaisd_show_geom(FILE *st, UNIT *uptr, int32 val, CONST void *desc) {
    if (uptr == NULL)
        return SCPE_IERR;
    fprintf(st, "T:%d/N:%d/S:%d", uptr->IMSAISD_TRACKS,
            uptr->IMSAISD_SPT, uptr->IMSAISD_SECTOR_SZ);
    return SCPE_OK;
}

/*
 * Queue a response byte to the output buffer.
 */
static void imsaisd_queue_byte(uint8 byte) {
    if (imsaisd_resp_length < RESP_BUF_SIZE) {
        imsaisd_resp_buf[imsaisd_resp_length++] = byte;
    }
}

/*
 * Perform a sector read from the attached disk image.
 * Reads IMSAISD_SECTOR_SIZE bytes at the computed offset into imsaisd_sector_buf.
 * Returns 0 on success, 1 on error.
 */
static int imsaisd_do_read(void) {
    UNIT *uptr = &imsaisd_unit[imsaisd_selected_unit];
    t_addr offset;

    if (!(uptr->flags & UNIT_ATT)) {
        sim_debug(READ_MSG, &imsaisd_dev,
                  "IMSAISD%d: read error - not attached\n", imsaisd_selected_unit);
        return 1;
    }

    offset = ((t_addr)imsaisd_track * IMSAISD_DEFAULT_SPT + imsaisd_sector) * IMSAISD_SECTOR_SIZE;
    if (offset + IMSAISD_SECTOR_SIZE > uptr->capac) {
        sim_debug(READ_MSG, &imsaisd_dev,
                  "IMSAISD%d: read error - track=%d sector=%d beyond capacity\n",
                  imsaisd_selected_unit, imsaisd_track, imsaisd_sector);
        return 1;
    }

    if (sim_fseek(uptr->fileref, offset, SEEK_SET) != 0) {
        sim_debug(READ_MSG, &imsaisd_dev,
                  "IMSAISD%d: read error - seek failed at offset %u\n",
                  imsaisd_selected_unit, (uint32)offset);
        return 1;
    }

    memset(imsaisd_sector_buf, 0xE5, IMSAISD_SECTOR_SIZE);
    if (sim_fread(imsaisd_sector_buf, 1, IMSAISD_SECTOR_SIZE, uptr->fileref)
        != IMSAISD_SECTOR_SIZE) {
        /* Short read OK for sparse files - buffer pre-filled with E5 */
    }

    sim_debug(READ_MSG, &imsaisd_dev,
              "IMSAISD%d: read track=%d sector=%d offset=%u OK\n",
              imsaisd_selected_unit, imsaisd_track, imsaisd_sector, (uint32)offset);
    return 0;
}

/*
 * Perform a sector write to the attached disk image.
 * Writes IMSAISD_SECTOR_SIZE bytes from imsaisd_sector_buf at the computed offset.
 * Returns 0 on success, 1 on error.
 */
static int imsaisd_do_write(void) {
    UNIT *uptr = &imsaisd_unit[imsaisd_selected_unit];
    t_addr offset;

    if (!(uptr->flags & UNIT_ATT)) {
        sim_debug(WRITE_MSG, &imsaisd_dev,
                  "IMSAISD%d: write error - not attached\n", imsaisd_selected_unit);
        return 1;
    }

    if (uptr->flags & UNIT_RO) {
        sim_debug(WRITE_MSG, &imsaisd_dev,
                  "IMSAISD%d: write error - read only\n", imsaisd_selected_unit);
        return 1;
    }

    offset = ((t_addr)imsaisd_track * IMSAISD_DEFAULT_SPT + imsaisd_sector) * IMSAISD_SECTOR_SIZE;
    if (offset + IMSAISD_SECTOR_SIZE > uptr->capac) {
        sim_debug(WRITE_MSG, &imsaisd_dev,
                  "IMSAISD%d: write error - track=%d sector=%d beyond capacity\n",
                  imsaisd_selected_unit, imsaisd_track, imsaisd_sector);
        return 1;
    }

    if (sim_fseek(uptr->fileref, offset, SEEK_SET) != 0) {
        sim_debug(WRITE_MSG, &imsaisd_dev,
                  "IMSAISD%d: write error - seek failed\n", imsaisd_selected_unit);
        return 1;
    }

    if (sim_fwrite(imsaisd_sector_buf, 1, IMSAISD_SECTOR_SIZE, uptr->fileref)
        != IMSAISD_SECTOR_SIZE) {
        sim_debug(WRITE_MSG, &imsaisd_dev,
                  "IMSAISD%d: write error - write failed\n", imsaisd_selected_unit);
        return 1;
    }

    sim_debug(WRITE_MSG, &imsaisd_dev,
              "IMSAISD%d: write track=%d sector=%d offset=%u OK\n",
              imsaisd_selected_unit, imsaisd_track, imsaisd_sector, (uint32)offset);
    return 0;
}

/*
 * Execute a read command: seek and queue 128 bytes.
 */
static void imsaisd_exec_read(void) {
    int i;

    if (imsaisd_do_read() == 0) {
        for (i = 0; i < IMSAISD_SECTOR_SIZE; i++)
            imsaisd_queue_byte(imsaisd_sector_buf[i]);
    } else {
        /* On error, return E5-filled sector */
        for (i = 0; i < IMSAISD_SECTOR_SIZE; i++)
            imsaisd_queue_byte(0xE5);
    }
    imsaisd_state = STATE_IDLE;
}

/*
 * Handle a byte written to the data port.
 * Advances the state machine through the '@R'/'@W' protocol.
 */
static void imsaisd_data_write(uint8 data) {
    switch (imsaisd_state) {
        case STATE_IDLE:
            if (data == '@') {
                imsaisd_state = STATE_PREFIX;
                sim_debug(VERBOSE_MSG, &imsaisd_dev, "IMSAISD: '@' received\n");
            }
            break;

        case STATE_PREFIX:
            if (data == 'R') {
                imsaisd_state = STATE_READ_TRK_HI;
                sim_debug(CMD_MSG, &imsaisd_dev, "IMSAISD: READ command\n");
            } else if (data == 'W') {
                imsaisd_state = STATE_WRITE_TRK_HI;
                sim_debug(CMD_MSG, &imsaisd_dev, "IMSAISD: WRITE command\n");
            } else {
                sim_debug(CMD_MSG, &imsaisd_dev,
                          "IMSAISD: unknown command '%c' (0x%02X)\n", data, data);
                imsaisd_state = STATE_IDLE;
            }
            break;

        case STATE_READ_TRK_HI:
            imsaisd_track_hi = data;
            imsaisd_selected_unit = (data >> 7) & 1;
            imsaisd_state = STATE_READ_TRK_LO;
            break;

        case STATE_READ_TRK_LO:
            imsaisd_track_lo = data;
            imsaisd_track = ((uint16)(imsaisd_track_hi & 0x7F) << 8) | imsaisd_track_lo;
            imsaisd_state = STATE_READ_SECTOR;
            break;

        case STATE_READ_SECTOR:
            imsaisd_sector = data;
            sim_debug(CMD_MSG, &imsaisd_dev,
                      "IMSAISD: READ unit=%d track=%d sector=%d\n",
                      imsaisd_selected_unit, imsaisd_track, imsaisd_sector);
            imsaisd_exec_read();
            break;

        case STATE_WRITE_TRK_HI:
            imsaisd_track_hi = data;
            imsaisd_selected_unit = (data >> 7) & 1;
            imsaisd_state = STATE_WRITE_TRK_LO;
            break;

        case STATE_WRITE_TRK_LO:
            imsaisd_track_lo = data;
            imsaisd_track = ((uint16)(imsaisd_track_hi & 0x7F) << 8) | imsaisd_track_lo;
            imsaisd_state = STATE_WRITE_SECTOR;
            break;

        case STATE_WRITE_SECTOR:
            imsaisd_sector = data;
            sim_debug(CMD_MSG, &imsaisd_dev,
                      "IMSAISD: WRITE unit=%d track=%d sector=%d\n",
                      imsaisd_selected_unit, imsaisd_track, imsaisd_sector);
            imsaisd_data_index = 0;
            imsaisd_state = STATE_WRITE_DATA;
            break;

        case STATE_WRITE_DATA:
            imsaisd_sector_buf[imsaisd_data_index++] = data;
            if (imsaisd_data_index >= IMSAISD_SECTOR_SIZE) {
                imsaisd_do_write();
                imsaisd_state = STATE_IDLE;
            }
            break;

        default:
            sim_debug(VERBOSE_MSG, &imsaisd_dev,
                      "IMSAISD: unexpected write 0x%02X in state %d\n",
                      data, imsaisd_state);
            imsaisd_state = STATE_IDLE;
            break;
    }
}

/*
 * Read a byte from the data port (dequeue from response buffer).
 */
static int32 imsaisd_data_read(void) {
    int32 result = 0;

    if (imsaisd_resp_index < imsaisd_resp_length) {
        result = imsaisd_resp_buf[imsaisd_resp_index++];
        if (imsaisd_resp_index >= imsaisd_resp_length) {
            imsaisd_resp_index = 0;
            imsaisd_resp_length = 0;
        }
    }

    sim_debug(VERBOSE_MSG, &imsaisd_dev,
              "IMSAISD: data read -> 0x%02X\n", result);
    return result;
}

/*
 * Read the status port.
 * Bit 0: RX data available (1 = byte ready to read from response buffer)
 * Bit 2: TX buffer empty (1 = ready to accept byte, always true in sim)
 */
static int32 imsaisd_status_read(void) {
    int32 status = 0;

    /* Bit 2: TX ready - always ready in simulation */
    status |= 0x04;

    /* Bit 0: RX data available if response buffer has data */
    if (imsaisd_resp_length > imsaisd_resp_index)
        status |= 0x01;

    sim_debug(STATUS_MSG, &imsaisd_dev,
              "IMSAISD: status read -> 0x%02X\n", status);
    return status;
}

/*
 * Main I/O dispatcher.
 * port: I/O port address
 * io: 0 = input (read), 1 = output (write)
 * data: byte being written (only valid when io=1)
 */
int32 imsaisd_io(const int32 port, const int32 io, const int32 data) {
    PNP_INFO *pnp = (PNP_INFO *)imsaisd_dev.ctxt;
    int32 offset = port - pnp->io_base;

    if (io == 0) {
        /* INPUT (read) */
        switch (offset) {
            case 0:     /* Data port (base+0 = 0xB4) */
                return imsaisd_data_read();
            case 2:     /* Status port (base+2 = 0xB6) */
                return imsaisd_status_read();
            default:
                return 0xFF;
        }
    } else {
        /* OUTPUT (write) */
        switch (offset) {
            case 0:     /* Data port (base+0 = 0xB4) */
                imsaisd_data_write((uint8)(data & 0xFF));
                break;
            case 2:     /* Status port - writes ignored */
                break;
        }
        return 0;
    }
}
