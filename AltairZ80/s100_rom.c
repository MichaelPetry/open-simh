/*************************************************************************
 *                                                                       *
 * Copyright (c) 2026                                                    *
 *                                                                       *
 * Generic ROM Device for AltairZ80                                      *
 *                                                                       *
 * Provides a configurable read-only memory region that loads a binary   *
 * or Intel HEX file. Useful for boot ROMs, monitor programs, etc.       *
 *                                                                       *
 * Configuration:                                                        *
 *   set rom enabled                                                     *
 *   set rom size=800          (hex, must be multiple of 100h/256)       *
 *   set rom membase=F800      (hex, must be aligned to size)            *
 *   attach rom0 firmware.bin  (or firmware.hex for Intel HEX)           *
 *                                                                       *
 *************************************************************************/

#include "altairz80_defs.h"
#include <ctype.h>

/* Debug flags */
#define VERBOSE_MSG (1 << 0)
#define ACCESS_MSG  (1 << 1)

/* Device constants */
#define ROM_MAX_SIZE    65536       /* Maximum ROM size (64K) */
#define ROM_DEFAULT_SIZE 0x0800    /* Default 2K */
#define ROM_DEFAULT_BASE 0xF800   /* Default base address */

/* Forward declarations */
static t_stat rom_reset(DEVICE *dptr);
static t_stat rom_attach(UNIT *uptr, CONST char *cptr);
static t_stat rom_detach(UNIT *uptr);
static t_stat rom_set_size(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
static t_stat rom_show_size(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
static t_stat rom_boot(int32 unitno, DEVICE *dptr);
static const char *rom_description(DEVICE *dptr);
static int32 rom_mem(const int32 Addr, const int32 write, const int32 data);

extern t_stat set_membase(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
extern t_stat show_membase(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
extern uint32 sim_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                               int32 (*routine)(const int32, const int32, const int32),
                               const char* name, uint8 unmap);

/* Device state */
static uint8 rom_buf[ROM_MAX_SIZE];
static uint32 rom_loaded_size = 0;

typedef struct {
    PNP_INFO    pnp;    /* Plug and Play */
} ROM_INFO;

static ROM_INFO rom_info_data = { { ROM_DEFAULT_BASE, ROM_DEFAULT_SIZE, 0, 0 } };

static UNIT rom_unit[] = {
    { UDATA(NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, ROM_DEFAULT_SIZE) }
};

static REG rom_reg[] = {
    { DRDATAD(LOADED, rom_loaded_size, 32, "Bytes loaded from file") },
    { NULL }
};

#define ROM_NAME    "Generic ROM"

static const char *rom_description(DEVICE *dptr) {
    return ROM_NAME;
}

static MTAB rom_mod[] = {
    { MTAB_XTD|MTAB_VDV,    0, "MEMBASE",  "MEMBASE",
        &set_membase, &show_membase, NULL,
        "Sets the memory base address" },
    { MTAB_XTD|MTAB_VDV,    0, "SIZE",     "SIZE",
        &rom_set_size, &rom_show_size, NULL,
        "Sets the ROM size in hex (must be multiple of 100)" },
    { 0 }
};

static DEBTAB rom_dt[] = {
    { "VERBOSE",    VERBOSE_MSG,    "Verbose messages"  },
    { "ACCESS",     ACCESS_MSG,     "Memory accesses"   },
    { NULL,         0                                   }
};

DEVICE rom_dev = {
    "ROM", rom_unit, rom_reg, rom_mod,
    1, 16, 16, 1, 16, 8,
    NULL, NULL, &rom_reset,
    &rom_boot, &rom_attach, &rom_detach,
    &rom_info_data, (DEV_DISABLE | DEV_DIS | DEV_DEBUG), 0,
    rom_dt, NULL, NULL, NULL, NULL, NULL, &rom_description
};

/*
 * Memory access handler.
 * Reads return ROM contents; writes are silently ignored.
 */
static int32 rom_mem(const int32 Addr, const int32 write, const int32 data) {
    PNP_INFO *pnp = (PNP_INFO *)rom_dev.ctxt;
    uint32 offset;

    if (write) {
        sim_debug(ACCESS_MSG, &rom_dev,
                  "ROM: write to 0x%04X ignored (ROM is read-only)\n", Addr);
        return 0;
    }

    offset = Addr - pnp->mem_base;
    if (offset < pnp->mem_size && offset < rom_loaded_size)
        return rom_buf[offset];

    return 0xFF;    /* Unprogrammed EPROM reads as 0xFF */
}

/* Reset routine */
static t_stat rom_reset(DEVICE *dptr) {
    PNP_INFO *pnp = (PNP_INFO *)dptr->ctxt;

    if (dptr->flags & DEV_DIS) {
        sim_map_resource(pnp->mem_base, pnp->mem_size, RESOURCE_TYPE_MEMORY,
                        &rom_mem, "rom_mem", TRUE);
    } else {
        if (sim_map_resource(pnp->mem_base, pnp->mem_size, RESOURCE_TYPE_MEMORY,
                            &rom_mem, "rom_mem", FALSE) != 0) {
            sim_printf("%s: error mapping memory resource at 0x%04x\n",
                      __FUNCTION__, pnp->mem_base);
            dptr->flags |= DEV_DIS;
            return SCPE_ARG;
        }
    }

    sim_debug(VERBOSE_MSG, &rom_dev,
              "ROM: reset, base=0x%04X size=0x%04X loaded=%u\n",
              pnp->mem_base, pnp->mem_size, rom_loaded_size);

    return SCPE_OK;
}

/*
 * Parse an Intel HEX file into rom_buf.
 * Addresses in the HEX file are treated as absolute; only data within
 * [mem_base, mem_base+mem_size) is stored.
 * Returns number of bytes loaded, or 0 on error.
 */
static uint32 rom_load_hex(FILE *fp, uint32 mem_base, uint32 mem_size) {
    char line[256];
    uint32 total = 0;
    uint32 max_offset = 0;

    while (fgets(line, sizeof(line), fp) != NULL) {
        uint32 byte_count, addr, rec_type;
        uint32 i, offset;
        uint8 checksum = 0;
        char *p;

        /* Skip lines that don't start with ':' */
        p = line;
        while (*p && isspace((unsigned char)*p)) p++;
        if (*p != ':') continue;
        p++;

        if (sscanf(p, "%2x%4x%2x", &byte_count, &addr, &rec_type) != 3)
            continue;

        checksum = (uint8)(byte_count + (addr >> 8) + (addr & 0xFF) + rec_type);
        p += 8;    /* skip past byte_count + addr + rec_type */

        if (rec_type == 0x01)   /* EOF record */
            break;

        if (rec_type != 0x00)   /* Only process data records */
            continue;

        for (i = 0; i < byte_count; i++) {
            uint32 byte_val;
            if (sscanf(p, "%2x", &byte_val) != 1)
                break;
            checksum += (uint8)byte_val;
            p += 2;

            /* Store if address falls within ROM range */
            if (addr + i >= mem_base && addr + i < mem_base + mem_size) {
                offset = (addr + i) - mem_base;
                rom_buf[offset] = (uint8)byte_val;
                if (offset + 1 > max_offset)
                    max_offset = offset + 1;
                total++;
            }
        }

        /* Verify checksum */
        {
            uint32 file_checksum;
            if (sscanf(p, "%2x", &file_checksum) == 1) {
                checksum += (uint8)file_checksum;
                if (checksum != 0) {
                    sim_printf("ROM: HEX checksum error at address 0x%04X\n", addr);
                }
            }
        }
    }

    return max_offset > 0 ? max_offset : total;
}

/* Attach routine - load binary or Intel HEX file */
static t_stat rom_attach(UNIT *uptr, CONST char *cptr) {
    PNP_INFO *pnp = (PNP_INFO *)rom_dev.ctxt;
    t_stat r;
    int is_hex = 0;
    const char *ext;
    size_t bytes_read;

    r = attach_unit(uptr, cptr);
    if (r != SCPE_OK)
        return r;

    /* Erase ROM buffer */
    memset(rom_buf, 0xFF, ROM_MAX_SIZE);
    rom_loaded_size = 0;

    /* Determine file type from extension */
    ext = strrchr(cptr, '.');
    if (ext != NULL && (strcasecmp(ext, ".hex") == 0 || strcasecmp(ext, ".ihx") == 0))
        is_hex = 1;

    /* Also detect by first character */
    if (!is_hex) {
        int ch = fgetc(uptr->fileref);
        if (ch == ':')
            is_hex = 1;
        rewind(uptr->fileref);
    }

    if (is_hex) {
        rom_loaded_size = rom_load_hex(uptr->fileref, pnp->mem_base, pnp->mem_size);
        sim_debug(VERBOSE_MSG, &rom_dev,
                  "ROM: loaded Intel HEX, %u bytes at 0x%04X-0x%04X\n",
                  rom_loaded_size, pnp->mem_base, pnp->mem_base + rom_loaded_size - 1);
    } else {
        /* Binary file: read up to ROM size */
        bytes_read = sim_fread(rom_buf, 1, pnp->mem_size, uptr->fileref);
        rom_loaded_size = (uint32)bytes_read;
        sim_debug(VERBOSE_MSG, &rom_dev,
                  "ROM: loaded binary, %u bytes at 0x%04X-0x%04X\n",
                  rom_loaded_size, pnp->mem_base, pnp->mem_base + rom_loaded_size - 1);
    }

    if (rom_loaded_size == 0) {
        sim_printf("ROM: warning - no data loaded from %s\n", cptr);
    } else {
        sim_printf("ROM: %u bytes loaded at 0x%04X-0x%04X from %s\n",
                   rom_loaded_size, pnp->mem_base,
                   pnp->mem_base + rom_loaded_size - 1, cptr);
    }

    return SCPE_OK;
}

/* Detach routine */
static t_stat rom_detach(UNIT *uptr) {
    rom_loaded_size = 0;
    memset(rom_buf, 0xFF, ROM_MAX_SIZE);
    return detach_unit(uptr);
}

/* Boot routine - set PC to ROM base address */
static t_stat rom_boot(int32 unitno, DEVICE *dptr) {
    PNP_INFO *pnp = (PNP_INFO *)dptr->ctxt;

    if (rom_loaded_size == 0) {
        sim_printf("ROM: no image loaded\n");
        return SCPE_IERR;
    }

    *((int32 *)sim_PC->loc) = pnp->mem_base;
    sim_printf("ROM: booting from 0x%04X\n", pnp->mem_base);
    return SCPE_OK;
}

/* Set ROM size (hex value, must be multiple of 256) */
static t_stat rom_set_size(UNIT *uptr, int32 val, CONST char *cptr, void *desc) {
    PNP_INFO *pnp = (PNP_INFO *)rom_dev.ctxt;
    uint32 new_size;
    t_stat r;

    if (cptr == NULL)
        return SCPE_ARG;

    new_size = get_uint(cptr, 16, ROM_MAX_SIZE, &r);
    if (r != SCPE_OK)
        return r;

    if (new_size == 0 || (new_size & 0xFF) != 0) {
        sim_printf("ROM: size must be a non-zero multiple of 100h (256)\n");
        return SCPE_ARG;
    }

    if (new_size > ROM_MAX_SIZE) {
        sim_printf("ROM: maximum size is %Xh (%d) bytes\n", ROM_MAX_SIZE, ROM_MAX_SIZE);
        return SCPE_ARG;
    }

    /* Remap if currently enabled */
    if (!(rom_dev.flags & DEV_DIS)) {
        sim_map_resource(pnp->mem_base, pnp->mem_size, RESOURCE_TYPE_MEMORY,
                        &rom_mem, "rom_mem", TRUE);
    }

    pnp->mem_size = new_size;
    uptr->capac = new_size;

    if (!(rom_dev.flags & DEV_DIS)) {
        sim_map_resource(pnp->mem_base, pnp->mem_size, RESOURCE_TYPE_MEMORY,
                        &rom_mem, "rom_mem", FALSE);
    }

    return SCPE_OK;
}

/* Show ROM size */
static t_stat rom_show_size(FILE *st, UNIT *uptr, int32 val, CONST void *desc) {
    PNP_INFO *pnp = (PNP_INFO *)rom_dev.ctxt;
    fprintf(st, "SIZE=0x%04X (%d bytes)", pnp->mem_size, pnp->mem_size);
    return SCPE_OK;
}
