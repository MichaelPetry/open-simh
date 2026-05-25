/*************************************************************************
 *                                                                       *
 * Copyright (c) 2026                                                    *
 *                                                                       *
 * S100 Dual SD Card Board (ESP32) Simulator for AltairZ80               *
 *                                                                       *
 * Simulates the S100Computers.com Dual SD Card Board which uses an      *
 * ESP32 to interface two SD cards to the S100 bus. The board uses two   *
 * I/O ports (status and data) with a command/response protocol.         *
 *                                                                       *
 * Reference: https://www.s100computers.com/My%20System%20Pages/         *
 *            Dual%20SD%20card%20Board/Dual%20SD%20card%20Board.htm      *
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
#define ESP32SD_UNITS           2
#define ESP32SD_SECTOR_SIZE     512
#define ESP32SD_DEFAULT_SPT     61      /* sectors per track (native format) */
#define ESP32SD_DEFAULT_TRACKS  256     /* tracks per disk (native format) */
#define ESP32SD_DEFAULT_CAP     (ESP32SD_DEFAULT_SPT * ESP32SD_DEFAULT_TRACKS * ESP32SD_SECTOR_SIZE)

/* HDSK-compatible format */
#define ESP32SD_HDSK_SPT        32      /* HDSK sectors per track */
#define ESP32SD_HDSK_SECSIZE    128     /* HDSK sector size */
#define ESP32SD_HDSK_TRACKS     2048    /* HDSK tracks */
#define ESP32SD_HDSK_CAP        (ESP32SD_HDSK_SPT * ESP32SD_HDSK_TRACKS * ESP32SD_HDSK_SECSIZE)

/* Unit fields for geometry */
#define ESP32SD_SECTOR_SZ       u5
#define ESP32SD_SPT             u4
#define ESP32SD_TRACKS          u3
#define ESP32SD_FORMAT          u6

/* Format types */
#define FMT_ESP32SD             0       /* Native: 512-byte sectors, 61/track, 256 tracks */
#define FMT_HDSK                1       /* HDSK: 128-byte sectors, 32/track, 2048 tracks */

/* ESP32 Command codes */
#define CMD_INIT0               0x80
#define CMD_INIT1               0x81
#define CMD_SEL0                0x82
#define CMD_SEL1                0x83
#define CMD_SETTRKSEC           0x84
#define CMD_READ                0x85
#define CMD_WRITE               0x86
#define CMD_FORMAT              0x87
#define CMD_RESET               0x88
#define CMD_FWVER               0x90
#define CMD_SETLBA              0x91
#define CMD_TYPE                0x92
#define CMD_CAP                 0x93
#define CMD_CID                 0x94
#define CMD_CSD                 0x95
#define CMD_DISP                0x96
#define CMD_ECHO                0x97

/* Command preamble */
#define CMD_PREAMBLE            0x33

/* Status register bits */
#define STAT_TX_BUSY            0x01    /* Bit 0: TX buffer full */
#define STAT_SD1_PRESENT        0x02    /* Bit 1: SD card 1 present */
#define STAT_SD2_PRESENT        0x04    /* Bit 2: SD card 2 present */
#define STAT_RX_READY           0x80    /* Bit 7: RX data available */

/* State machine states */
#define STATE_IDLE              0
#define STATE_PREAMBLE          1       /* Received 0x33, waiting for command */
#define STATE_CMD_SETTRKSEC_TRK 2       /* Waiting for track byte */
#define STATE_CMD_SETTRKSEC_SEC 3       /* Waiting for sector byte */
#define STATE_CMD_SETLBA        4       /* Receiving 4 LBA bytes */
#define STATE_CMD_WRITE_DATA    5       /* Receiving 512 bytes of write data */
#define STATE_CMD_READ_DATA     6       /* Sending 512 bytes of read data */
#define STATE_CMD_ECHO_LEN_MSB  7       /* Receiving echo length MSB */
#define STATE_CMD_ECHO_LEN_LSB  8       /* Receiving echo length LSB */
#define STATE_CMD_ECHO_DATA     9       /* Receiving echo data */
#define STATE_CMD_ECHO_SEND     10      /* Sending echo data back */

/* Response buffer size */
#define RESP_BUF_SIZE           1024

/* Firmware version */
#define FW_BOARD_ID             0x01
#define FW_VER_MAJOR            0x02
#define FW_VER_MINOR            0x00

/* Forward declarations */
static t_stat esp32sd_reset(DEVICE *dptr);
static t_stat esp32sd_attach(UNIT *uptr, CONST char *cptr);
static t_stat esp32sd_detach(UNIT *uptr);
static t_stat esp32sd_set_format(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
static t_stat esp32sd_show_format(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
static t_stat esp32sd_set_geom(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
static t_stat esp32sd_show_geom(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
static const char *esp32sd_description(DEVICE *dptr);
int32 esp32sd_io(const int32 port, const int32 io, const int32 data);

extern t_stat set_iobase(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
extern t_stat show_iobase(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
extern uint32 sim_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                               int32 (*routine)(const int32, const int32, const int32),
                               const char* name, uint8 unmap);
extern int32 find_unit_index(UNIT *uptr);

/* Device state */
static int32 esp32sd_state = STATE_IDLE;
static int32 esp32sd_selected_unit = 0;
static uint32 esp32sd_lba = 0;
static int32 esp32sd_current_cmd = 0;
static int32 esp32sd_param_count = 0;
static int32 esp32sd_data_index = 0;
static int32 esp32sd_data_length = 0;
static uint8 esp32sd_sector_buf[ESP32SD_SECTOR_SIZE];
static uint8 esp32sd_resp_buf[RESP_BUF_SIZE];
static int32 esp32sd_resp_index = 0;
static int32 esp32sd_resp_length = 0;
static uint8 esp32sd_track_byte = 0;
static uint8 esp32sd_lba_bytes[4];
static int32 esp32sd_echo_length = 0;
static uint8 esp32sd_echo_buf[RESP_BUF_SIZE];
static int32 esp32sd_echo_index = 0;
static int32 esp32sd_unit_initialized[ESP32SD_UNITS] = { 0, 0 };
static int32 esp32sd_rx_consumed = 0;  /* Set after data read, cleared on next status read */

typedef struct {
    PNP_INFO    pnp;    /* Plug and Play */
} ESP32SD_INFO;

static ESP32SD_INFO esp32sd_info_data = { { 0x0000, 0, 0x80, 2 } };

static UNIT esp32sd_unit[] = {
    { UDATA(NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, ESP32SD_DEFAULT_CAP) },
    { UDATA(NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, ESP32SD_DEFAULT_CAP) }
};

static REG esp32sd_reg[] = {
    { DRDATAD(STATE,    esp32sd_state,          8,  "State machine state")      },
    { DRDATAD(UNIT,     esp32sd_selected_unit,  8,  "Selected SD unit")         },
    { DRDATAD(LBA,      esp32sd_lba,            32, "Current LBA address")      },
    { DRDATAD(CMD,      esp32sd_current_cmd,    8,  "Current command")          },
    { NULL }
};

#define ESP32SD_NAME    "S100 ESP32 Dual SD Card"

static const char *esp32sd_description(DEVICE *dptr) {
    return ESP32SD_NAME;
}

static MTAB esp32sd_mod[] = {
    { MTAB_XTD|MTAB_VDV,    0, "IOBASE",  "IOBASE",
        &set_iobase, &show_iobase, NULL,
        "Sets the I/O base address for " ESP32SD_NAME },
    { MTAB_XTD|MTAB_VUN,    0, "FORMAT",  "FORMAT",
        &esp32sd_set_format, &esp32sd_show_format, NULL,
        "Sets the disk format (ESP32SD or HDSK)" },
    { MTAB_XTD|MTAB_VUN,    0, "GEOM",    "GEOM",
        &esp32sd_set_geom, &esp32sd_show_geom, NULL,
        "Sets disk geometry T:tracks/N:sectors/S:sectorsize" },
    { 0 }
};

static DEBTAB esp32sd_dt[] = {
    { "CMD",        CMD_MSG,        "Command processing"    },
    { "READ",       READ_MSG,       "Read operations"       },
    { "WRITE",      WRITE_MSG,      "Write operations"      },
    { "STATUS",     STATUS_MSG,     "Status port access"    },
    { "VERBOSE",    VERBOSE_MSG,    "Verbose output"        },
    { NULL,         0                                       }
};

DEVICE esp32sd_dev = {
    "ESP32SD", esp32sd_unit, esp32sd_reg, esp32sd_mod,
    ESP32SD_UNITS, 10, 31, 1, 8, 8,
    NULL, NULL, &esp32sd_reset,
    NULL, &esp32sd_attach, &esp32sd_detach,
    &esp32sd_info_data, (DEV_DISABLE | DEV_DIS | DEV_DEBUG), 0,
    esp32sd_dt, NULL, NULL, NULL, NULL, NULL, &esp32sd_description
};

/* Reset routine */
static t_stat esp32sd_reset(DEVICE *dptr) {
    PNP_INFO *pnp = (PNP_INFO *)dptr->ctxt;

    if (dptr->flags & DEV_DIS) {
        sim_map_resource(pnp->io_base, pnp->io_size, RESOURCE_TYPE_IO,
                        &esp32sd_io, "esp32sd_io", TRUE);
    } else {
        if (sim_map_resource(pnp->io_base, pnp->io_size, RESOURCE_TYPE_IO,
                            &esp32sd_io, "esp32sd_io", FALSE) != 0) {
            sim_printf("%s: error mapping I/O resource at 0x%04x\n",
                      __FUNCTION__, pnp->io_base);
            dptr->flags |= DEV_DIS;
            return SCPE_ARG;
        }
    }

    /* Reset state machine */
    esp32sd_state = STATE_IDLE;
    esp32sd_selected_unit = 0;
    esp32sd_lba = 0;
    esp32sd_current_cmd = 0;
    esp32sd_param_count = 0;
    esp32sd_data_index = 0;
    esp32sd_resp_index = 0;
    esp32sd_resp_length = 0;
    esp32sd_unit_initialized[0] = 0;
    esp32sd_unit_initialized[1] = 0;
    esp32sd_rx_consumed = 0;

    return SCPE_OK;
}

/* Attach disk image */
static t_stat esp32sd_attach(UNIT *uptr, CONST char *cptr) {
    const t_stat r = attach_unit(uptr, cptr);
    if (r != SCPE_OK)
        return r;

    /* Determine capacity */
    uptr->capac = sim_fsize(uptr->fileref);
    if (uptr->capac == 0) {
        /* New file: use configured geometry or default */
        if (uptr->ESP32SD_SPT == 0) {
            if (uptr->ESP32SD_FORMAT == FMT_HDSK) {
                uptr->ESP32SD_SPT = ESP32SD_HDSK_SPT;
                uptr->ESP32SD_SECTOR_SZ = ESP32SD_HDSK_SECSIZE;
                uptr->ESP32SD_TRACKS = ESP32SD_HDSK_TRACKS;
            } else {
                uptr->ESP32SD_SPT = ESP32SD_DEFAULT_SPT;
                uptr->ESP32SD_SECTOR_SZ = ESP32SD_SECTOR_SIZE;
                uptr->ESP32SD_TRACKS = ESP32SD_DEFAULT_TRACKS;
            }
        }
        uptr->capac = (t_addr)uptr->ESP32SD_TRACKS * uptr->ESP32SD_SPT * uptr->ESP32SD_SECTOR_SZ;
    } else {
        /* Existing file: determine geometry from size */
        if (uptr->ESP32SD_SPT == 0) {
            if (uptr->capac == ESP32SD_HDSK_CAP) {
                uptr->ESP32SD_FORMAT = FMT_HDSK;
                uptr->ESP32SD_SPT = ESP32SD_HDSK_SPT;
                uptr->ESP32SD_SECTOR_SZ = ESP32SD_HDSK_SECSIZE;
                uptr->ESP32SD_TRACKS = ESP32SD_HDSK_TRACKS;
            } else {
                uptr->ESP32SD_FORMAT = FMT_ESP32SD;
                uptr->ESP32SD_SPT = ESP32SD_DEFAULT_SPT;
                uptr->ESP32SD_SECTOR_SZ = ESP32SD_SECTOR_SIZE;
                uptr->ESP32SD_TRACKS = (uptr->capac + ESP32SD_DEFAULT_SPT * ESP32SD_SECTOR_SIZE - 1)
                                       / (ESP32SD_DEFAULT_SPT * ESP32SD_SECTOR_SIZE);
            }
        }
    }

    sim_debug(VERBOSE_MSG, &esp32sd_dev,
              "ESP32SD%d: attached, capacity=%d, format=%s, T:%d/N:%d/S:%d\n",
              find_unit_index(uptr), uptr->capac,
              uptr->ESP32SD_FORMAT == FMT_HDSK ? "HDSK" : "ESP32SD",
              uptr->ESP32SD_TRACKS, uptr->ESP32SD_SPT, uptr->ESP32SD_SECTOR_SZ);

    return SCPE_OK;
}

/* Detach disk image */
static t_stat esp32sd_detach(UNIT *uptr) {
    int32 idx = find_unit_index(uptr);
    if (idx >= 0 && idx < ESP32SD_UNITS)
        esp32sd_unit_initialized[idx] = 0;
    return detach_unit(uptr);
}

/* Set format: ESP32SD or HDSK */
static t_stat esp32sd_set_format(UNIT *uptr, int32 val, CONST char *cptr, void *desc) {
    if (cptr == NULL)
        return SCPE_ARG;
    if (uptr == NULL)
        return SCPE_IERR;

    if (strcasecmp(cptr, "ESP32SD") == 0) {
        uptr->ESP32SD_FORMAT = FMT_ESP32SD;
        uptr->ESP32SD_SPT = ESP32SD_DEFAULT_SPT;
        uptr->ESP32SD_SECTOR_SZ = ESP32SD_SECTOR_SIZE;
        uptr->ESP32SD_TRACKS = ESP32SD_DEFAULT_TRACKS;
        uptr->capac = ESP32SD_DEFAULT_CAP;
    } else if (strcasecmp(cptr, "HDSK") == 0) {
        uptr->ESP32SD_FORMAT = FMT_HDSK;
        uptr->ESP32SD_SPT = ESP32SD_HDSK_SPT;
        uptr->ESP32SD_SECTOR_SZ = ESP32SD_HDSK_SECSIZE;
        uptr->ESP32SD_TRACKS = ESP32SD_HDSK_TRACKS;
        uptr->capac = ESP32SD_HDSK_CAP;
    } else {
        return SCPE_ARG;
    }
    return SCPE_OK;
}

/* Show format */
static t_stat esp32sd_show_format(FILE *st, UNIT *uptr, int32 val, CONST void *desc) {
    if (uptr == NULL)
        return SCPE_IERR;
    fprintf(st, "%s", uptr->ESP32SD_FORMAT == FMT_HDSK ? "HDSK" : "ESP32SD");
    return SCPE_OK;
}

/* Set geometry: T:tracks/N:sectors/S:sectorsize */
static t_stat esp32sd_set_geom(UNIT *uptr, int32 val, CONST char *cptr, void *desc) {
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

    uptr->ESP32SD_TRACKS = tracks;
    uptr->ESP32SD_SPT = sectors;
    uptr->ESP32SD_SECTOR_SZ = secsize;
    uptr->capac = tracks * sectors * secsize;
    return SCPE_OK;
}

/* Show geometry */
static t_stat esp32sd_show_geom(FILE *st, UNIT *uptr, int32 val, CONST void *desc) {
    if (uptr == NULL)
        return SCPE_IERR;
    fprintf(st, "T:%d/N:%d/S:%d", uptr->ESP32SD_TRACKS,
            uptr->ESP32SD_SPT, uptr->ESP32SD_SECTOR_SZ);
    return SCPE_OK;
}

/*
 * Queue a response byte to the output buffer.
 */
static void esp32sd_queue_byte(uint8 byte) {
    if (esp32sd_resp_length < RESP_BUF_SIZE) {
        esp32sd_resp_buf[esp32sd_resp_length++] = byte;
    }
}

/*
 * Perform a sector read from the attached disk image.
 * Reads ESP32SD_SECTOR_SIZE bytes at the current LBA into esp32sd_sector_buf.
 * Returns 0 on success, 1 on error.
 */
static int esp32sd_do_read(void) {
    UNIT *uptr = &esp32sd_unit[esp32sd_selected_unit];
    t_addr offset;

    if (!(uptr->flags & UNIT_ATT)) {
        sim_debug(READ_MSG, &esp32sd_dev,
                  "ESP32SD%d: read error - not attached\n", esp32sd_selected_unit);
        return 1;
    }

    offset = (t_addr)esp32sd_lba * ESP32SD_SECTOR_SIZE;
    if (offset + ESP32SD_SECTOR_SIZE > uptr->capac) {
        sim_debug(READ_MSG, &esp32sd_dev,
                  "ESP32SD%d: read error - LBA %u beyond capacity\n",
                  esp32sd_selected_unit, esp32sd_lba);
        return 1;
    }

    if (sim_fseek(uptr->fileref, offset, SEEK_SET) != 0) {
        sim_debug(READ_MSG, &esp32sd_dev,
                  "ESP32SD%d: read error - seek failed at offset %u\n",
                  esp32sd_selected_unit, (uint32)offset);
        return 1;
    }

    memset(esp32sd_sector_buf, 0xE5, ESP32SD_SECTOR_SIZE);
    if (sim_fread(esp32sd_sector_buf, 1, ESP32SD_SECTOR_SIZE, uptr->fileref)
        != ESP32SD_SECTOR_SIZE) {
        /* Short read is OK for sparse files - buffer was pre-filled with E5 */
    }

    sim_debug(READ_MSG, &esp32sd_dev,
              "ESP32SD%d: read LBA=%u offset=%u OK\n",
              esp32sd_selected_unit, esp32sd_lba, (uint32)offset);
    return 0;
}

/*
 * Perform a sector write to the attached disk image.
 * Writes ESP32SD_SECTOR_SIZE bytes from esp32sd_sector_buf at the current LBA.
 * Returns 0 on success, 1 on error.
 */
static int esp32sd_do_write(void) {
    UNIT *uptr = &esp32sd_unit[esp32sd_selected_unit];
    t_addr offset;

    if (!(uptr->flags & UNIT_ATT)) {
        sim_debug(WRITE_MSG, &esp32sd_dev,
                  "ESP32SD%d: write error - not attached\n", esp32sd_selected_unit);
        return 1;
    }

    if (uptr->flags & UNIT_RO) {
        sim_debug(WRITE_MSG, &esp32sd_dev,
                  "ESP32SD%d: write error - read only\n", esp32sd_selected_unit);
        return 1;
    }

    offset = (t_addr)esp32sd_lba * ESP32SD_SECTOR_SIZE;
    if (offset + ESP32SD_SECTOR_SIZE > uptr->capac) {
        sim_debug(WRITE_MSG, &esp32sd_dev,
                  "ESP32SD%d: write error - LBA %u beyond capacity\n",
                  esp32sd_selected_unit, esp32sd_lba);
        return 1;
    }

    if (sim_fseek(uptr->fileref, offset, SEEK_SET) != 0) {
        sim_debug(WRITE_MSG, &esp32sd_dev,
                  "ESP32SD%d: write error - seek failed\n", esp32sd_selected_unit);
        return 1;
    }

    if (sim_fwrite(esp32sd_sector_buf, 1, ESP32SD_SECTOR_SIZE, uptr->fileref)
        != ESP32SD_SECTOR_SIZE) {
        sim_debug(WRITE_MSG, &esp32sd_dev,
                  "ESP32SD%d: write error - write failed\n", esp32sd_selected_unit);
        return 1;
    }

    sim_debug(WRITE_MSG, &esp32sd_dev,
              "ESP32SD%d: write LBA=%u offset=%u OK\n",
              esp32sd_selected_unit, esp32sd_lba, (uint32)offset);
    return 0;
}

/*
 * Process a complete command after all parameters have been received.
 */
static void esp32sd_execute_cmd(uint8 cmd) {
    UNIT *uptr;
    uint32 cap_sectors;
    int result;

    sim_debug(CMD_MSG, &esp32sd_dev, "ESP32SD: execute cmd=0x%02X unit=%d\n",
              cmd, esp32sd_selected_unit);

    switch (cmd) {
        case CMD_INIT0:
            uptr = &esp32sd_unit[0];
            if (uptr->flags & UNIT_ATT) {
                esp32sd_unit_initialized[0] = 1;
                esp32sd_selected_unit = 0;
                esp32sd_queue_byte(0x00);   /* success */
            } else {
                esp32sd_queue_byte(0x01);   /* error: not attached */
            }
            esp32sd_state = STATE_IDLE;
            break;

        case CMD_INIT1:
            uptr = &esp32sd_unit[1];
            if (uptr->flags & UNIT_ATT) {
                esp32sd_unit_initialized[1] = 1;
                esp32sd_selected_unit = 1;
                esp32sd_queue_byte(0x00);
            } else {
                esp32sd_queue_byte(0x01);
            }
            esp32sd_state = STATE_IDLE;
            break;

        case CMD_SEL0:
            uptr = &esp32sd_unit[0];
            if ((uptr->flags & UNIT_ATT) && esp32sd_unit_initialized[0]) {
                esp32sd_selected_unit = 0;
                esp32sd_queue_byte(0x00);
            } else {
                esp32sd_queue_byte(0x01);
            }
            esp32sd_state = STATE_IDLE;
            break;

        case CMD_SEL1:
            uptr = &esp32sd_unit[1];
            if ((uptr->flags & UNIT_ATT) && esp32sd_unit_initialized[1]) {
                esp32sd_selected_unit = 1;
                esp32sd_queue_byte(0x00);
            } else {
                esp32sd_queue_byte(0x01);
            }
            esp32sd_state = STATE_IDLE;
            break;

        case CMD_SETTRKSEC:
            /* Need 2 more bytes: track, sector */
            esp32sd_state = STATE_CMD_SETTRKSEC_TRK;
            esp32sd_param_count = 0;
            break;

        case CMD_READ:
            /* Read sector at current LBA, queue 512 bytes + result */
            result = esp32sd_do_read();
            if (result == 0) {
                /* Queue all 512 bytes */
                esp32sd_data_index = 0;
                esp32sd_data_length = ESP32SD_SECTOR_SIZE;
                esp32sd_state = STATE_CMD_READ_DATA;
            } else {
                /* Queue error bytes (fill with E5) then error result */
                esp32sd_data_index = 0;
                esp32sd_data_length = ESP32SD_SECTOR_SIZE;
                memset(esp32sd_sector_buf, 0xE5, ESP32SD_SECTOR_SIZE);
                esp32sd_state = STATE_CMD_READ_DATA;
            }
            break;

        case CMD_WRITE:
            /* Need 512 bytes of data */
            esp32sd_data_index = 0;
            esp32sd_data_length = ESP32SD_SECTOR_SIZE;
            esp32sd_state = STATE_CMD_WRITE_DATA;
            break;

        case CMD_FORMAT:
            /* Fill current sector with 0xE5 */
            memset(esp32sd_sector_buf, 0xE5, ESP32SD_SECTOR_SIZE);
            result = esp32sd_do_write();
            esp32sd_queue_byte(result ? 0x01 : 0x00);
            esp32sd_state = STATE_IDLE;
            break;

        case CMD_RESET:
            esp32sd_state = STATE_IDLE;
            esp32sd_resp_index = 0;
            esp32sd_resp_length = 0;
            break;

        case CMD_FWVER:
            esp32sd_queue_byte(FW_BOARD_ID);
            esp32sd_queue_byte(FW_VER_MAJOR);
            esp32sd_queue_byte(FW_VER_MINOR);
            esp32sd_queue_byte(0x00);   /* result */
            esp32sd_state = STATE_IDLE;
            break;

        case CMD_SETLBA:
            /* Need 4 bytes MSB first */
            esp32sd_param_count = 0;
            esp32sd_state = STATE_CMD_SETLBA;
            break;

        case CMD_TYPE:
            /* Report card type: 2 = SDHC */
            esp32sd_queue_byte(0x02);
            esp32sd_queue_byte(0x00);   /* result */
            esp32sd_state = STATE_IDLE;
            break;

        case CMD_CAP:
            /* Report capacity in sectors (4 bytes MSB first) */
            uptr = &esp32sd_unit[esp32sd_selected_unit];
            cap_sectors = uptr->capac / ESP32SD_SECTOR_SIZE;
            esp32sd_queue_byte((cap_sectors >> 24) & 0xFF);
            esp32sd_queue_byte((cap_sectors >> 16) & 0xFF);
            esp32sd_queue_byte((cap_sectors >> 8) & 0xFF);
            esp32sd_queue_byte(cap_sectors & 0xFF);
            esp32sd_queue_byte(0x00);   /* result */
            esp32sd_state = STATE_IDLE;
            break;

        case CMD_CID:
            /* Return 16 bytes of fake CID data + result */
            {
                int i;
                for (i = 0; i < 16; i++)
                    esp32sd_queue_byte(0x00);
            }
            esp32sd_queue_byte(0x00);
            esp32sd_state = STATE_IDLE;
            break;

        case CMD_CSD:
            /* Return 16 bytes of fake CSD data + result */
            {
                int i;
                for (i = 0; i < 16; i++)
                    esp32sd_queue_byte(0x00);
            }
            esp32sd_queue_byte(0x00);
            esp32sd_state = STATE_IDLE;
            break;

        case CMD_DISP:
            esp32sd_queue_byte(0x00);
            esp32sd_state = STATE_IDLE;
            break;

        case CMD_ECHO:
            /* Need 2-byte length then data */
            esp32sd_state = STATE_CMD_ECHO_LEN_MSB;
            esp32sd_param_count = 0;
            break;

        default:
            sim_debug(CMD_MSG, &esp32sd_dev,
                      "ESP32SD: unknown command 0x%02X\n", cmd);
            esp32sd_queue_byte(0xFF);   /* error: unknown command */
            esp32sd_state = STATE_IDLE;
            break;
    }
}

/*
 * Handle a byte written to the data port.
 * This advances the state machine.
 */
static void esp32sd_data_write(uint8 data) {
    int result;

    switch (esp32sd_state) {
        case STATE_IDLE:
            if (data == CMD_PREAMBLE) {
                esp32sd_state = STATE_PREAMBLE;
                sim_debug(VERBOSE_MSG, &esp32sd_dev, "ESP32SD: preamble received\n");
            }
            break;

        case STATE_PREAMBLE:
            /* This byte is the command */
            esp32sd_current_cmd = data;
            esp32sd_execute_cmd(data);
            break;

        case STATE_CMD_SETTRKSEC_TRK:
            esp32sd_track_byte = data;
            esp32sd_state = STATE_CMD_SETTRKSEC_SEC;
            break;

        case STATE_CMD_SETTRKSEC_SEC:
            /* Compute LBA from track and sector (sector is 1-based) */
            esp32sd_lba = (uint32)esp32sd_track_byte * ESP32SD_DEFAULT_SPT + (data - 1);
            sim_debug(CMD_MSG, &esp32sd_dev,
                      "ESP32SD: SETTRKSEC track=%d sector=%d -> LBA=%u\n",
                      esp32sd_track_byte, data, esp32sd_lba);
            esp32sd_queue_byte(0x00);   /* success */
            esp32sd_state = STATE_IDLE;
            break;

        case STATE_CMD_SETLBA:
            esp32sd_lba_bytes[esp32sd_param_count++] = data;
            if (esp32sd_param_count >= 4) {
                esp32sd_lba = ((uint32)esp32sd_lba_bytes[0] << 24) |
                              ((uint32)esp32sd_lba_bytes[1] << 16) |
                              ((uint32)esp32sd_lba_bytes[2] << 8) |
                              (uint32)esp32sd_lba_bytes[3];
                sim_debug(CMD_MSG, &esp32sd_dev,
                          "ESP32SD: SETLBA -> LBA=%u\n", esp32sd_lba);
                esp32sd_queue_byte(0x00);
                esp32sd_state = STATE_IDLE;
            }
            break;

        case STATE_CMD_WRITE_DATA:
            esp32sd_sector_buf[esp32sd_data_index++] = data;
            if (esp32sd_data_index >= esp32sd_data_length) {
                /* All data received, perform write */
                result = esp32sd_do_write();
                esp32sd_queue_byte(result ? 0x01 : 0x00);
                esp32sd_state = STATE_IDLE;
            }
            break;

        case STATE_CMD_ECHO_LEN_MSB:
            esp32sd_echo_length = (int32)data << 8;
            esp32sd_state = STATE_CMD_ECHO_LEN_LSB;
            break;

        case STATE_CMD_ECHO_LEN_LSB:
            esp32sd_echo_length |= data;
            if (esp32sd_echo_length > RESP_BUF_SIZE)
                esp32sd_echo_length = RESP_BUF_SIZE;
            esp32sd_echo_index = 0;
            if (esp32sd_echo_length == 0) {
                esp32sd_queue_byte(0x00);
                esp32sd_state = STATE_IDLE;
            } else {
                esp32sd_state = STATE_CMD_ECHO_DATA;
            }
            break;

        case STATE_CMD_ECHO_DATA:
            esp32sd_echo_buf[esp32sd_echo_index++] = data;
            if (esp32sd_echo_index >= esp32sd_echo_length) {
                /* Echo all data back */
                int i;
                for (i = 0; i < esp32sd_echo_length; i++)
                    esp32sd_queue_byte(esp32sd_echo_buf[i]);
                esp32sd_queue_byte(0x00);   /* result */
                esp32sd_state = STATE_IDLE;
            }
            break;

        default:
            sim_debug(VERBOSE_MSG, &esp32sd_dev,
                      "ESP32SD: unexpected write 0x%02X in state %d\n",
                      data, esp32sd_state);
            esp32sd_state = STATE_IDLE;
            break;
    }
}

/*
 * Read a byte from the data port (dequeue from response buffer or sector buffer).
 */
static int32 esp32sd_data_read(void) {
    int32 result = 0;

    if (esp32sd_state == STATE_CMD_READ_DATA) {
        /* Streaming sector data */
        result = esp32sd_sector_buf[esp32sd_data_index++];
        if (esp32sd_data_index >= esp32sd_data_length) {
            /* All sector data sent, queue the result byte */
            esp32sd_queue_byte(0x00);   /* success */
            esp32sd_state = STATE_IDLE;
        }
    } else if (esp32sd_resp_index < esp32sd_resp_length) {
        /* Normal response buffer */
        result = esp32sd_resp_buf[esp32sd_resp_index++];
        if (esp32sd_resp_index >= esp32sd_resp_length) {
            /* Response fully consumed, reset buffer */
            esp32sd_resp_index = 0;
            esp32sd_resp_length = 0;
        }
    }

    /* After reading a byte, suppress RX_READY for one status read cycle.
     * This models the real hardware behavior where the ESP32 briefly
     * deasserts the ready signal between bytes. */
    esp32sd_rx_consumed = 1;

    sim_debug(VERBOSE_MSG, &esp32sd_dev,
              "ESP32SD: data read -> 0x%02X\n", result);
    return result;
}

/*
 * Read the status port.
 * Returns status bits based on current device state.
 */
static int32 esp32sd_status_read(void) {
    int32 status = 0;

    /* Bit 0: TX_BUSY - always 0 in simulation (we're always ready) */

    /* Bit 1: SD1 present if unit 0 is attached */
    if (esp32sd_unit[0].flags & UNIT_ATT)
        status |= STAT_SD1_PRESENT;

    /* Bit 2: SD2 present if unit 1 is attached */
    if (esp32sd_unit[1].flags & UNIT_ATT)
        status |= STAT_SD2_PRESENT;

    /* Bit 7: RX_READY if there's data to read.
     * After a data read, suppress RX_READY for one status read cycle
     * to model the real ESP32 inter-byte handshake timing. */
    if (esp32sd_rx_consumed) {
        esp32sd_rx_consumed = 0;
    } else if (esp32sd_state == STATE_CMD_READ_DATA || esp32sd_resp_length > esp32sd_resp_index) {
        status |= STAT_RX_READY;
    }

    sim_debug(STATUS_MSG, &esp32sd_dev,
              "ESP32SD: status read -> 0x%02X\n", status);
    return status;
}

/*
 * Main I/O dispatcher.
 * port: I/O port address
 * io: 0 = input (read), 1 = output (write)
 * data: byte being written (only valid when io=1)
 */
int32 esp32sd_io(const int32 port, const int32 io, const int32 data) {
    PNP_INFO *pnp = (PNP_INFO *)esp32sd_dev.ctxt;
    int32 offset = port - pnp->io_base;

    if (io == 0) {
        /* INPUT (read) */
        switch (offset) {
            case 0:     /* Status port */
                return esp32sd_status_read();
            case 1:     /* Data port */
                return esp32sd_data_read();
            default:
                return 0xFF;
        }
    } else {
        /* OUTPUT (write) */
        switch (offset) {
            case 0:     /* Status port - writes ignored */
                break;
            case 1:     /* Data port */
                esp32sd_data_write((uint8)(data & 0xFF));
                break;
        }
        return 0;
    }
}
