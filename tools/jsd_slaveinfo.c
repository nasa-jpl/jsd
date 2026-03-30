/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : slaveinfo [ifname] [-sdo] [-map]
 * Ifname is NIC interface, f.e. eth0.
 * Optional -sdo to display CoE object dictionary.
 * Optional -map to display slave PDO mapping
 *
 * This shows the configured slave data.
 *
 * (c)Arthur Ketels 2010 - 2011
 *
 * Modified from the original SOEM slaveinfo.c
 * to support SOEM EC_VER2 used by jsd
 */

#include <inttypes.h>
#include <string.h>

#include "jsd/jsd.h"
#include "jsd_slaveinfo.h"

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/sys/printk.h>
#define JSD_SLAVEINFO_PRINT(...) printk(__VA_ARGS__)
#define printf(...) JSD_SLAVEINFO_PRINT(__VA_ARGS__)
#endif

ec_ODlistt ODlist;
ec_OElistt OElist;
boolean    printSDO = FALSE;
boolean    printMAP = FALSE;
char       usdo[128];
char       hstr[1024];
jsd_t*     jsd;

#ifdef __ZEPHYR__
/*
 * The Zephyr sample parks forever after a successful run, and current packet
 * socket teardown still appears to trip a post-success fault on this platform.
 * Retain the successful context instead of closing/freeing it immediately.
 */
static jsd_t* jsd_retained_on_success;
#endif

#ifdef __ZEPHYR__
#define JSD_SLAVEINFO_POST_INIT_LINK_WAIT_MS 3000
#define JSD_SLAVEINFO_POST_CARRIER_SETTLE_MS 1000
#define JSD_SLAVEINFO_CONFIG_INIT_RETRIES 2
#define JSD_SLAVEINFO_CONFIG_INIT_RETRY_DELAY_MS 250

static void jsd_slaveinfo_log_stack_space(const char* label) {
  size_t unused = 0U;
  int err = k_thread_stack_space_get(k_current_get(), &unused);

  if (err == 0) {
    printf("slaveinfo: stack unused %s: %zu bytes\n", label, unused);
  } else {
    printf("slaveinfo: stack probe failed %s: %d\n", label, err);
  }
}

static struct net_if* jsd_slaveinfo_find_iface_by_name(const char* ifname) {
  STRUCT_SECTION_FOREACH(net_if, iface) {
    char name[IFNAMSIZ] = {0};

    if (net_if_get_name(iface, name, sizeof(name)) > 0 &&
        strcmp(name, ifname) == 0) {
      return iface;
    }
  }

  return NULL;
}

static bool jsd_slaveinfo_wait_for_zephyr_carrier_after_ec_init(
    const char* ifname) {
  struct net_if* iface;
  int64_t        deadline;

  iface = jsd_slaveinfo_find_iface_by_name(ifname);
  if (iface == NULL) {
    printf("Unable to find Zephyr net_if named %s after ec_init().\n", ifname);
    return false;
  }

  if (net_if_is_carrier_ok(iface)) {
    printf("Carrier is already on for %s after ec_init().\n", ifname);
    printf("Waiting %d ms for %s link to settle after carrier.\n",
           JSD_SLAVEINFO_POST_CARRIER_SETTLE_MS, ifname);
    k_msleep(JSD_SLAVEINFO_POST_CARRIER_SETTLE_MS);
    return true;
  }

  printf("Waiting up to %d ms for carrier on %s after ec_init().\n",
         JSD_SLAVEINFO_POST_INIT_LINK_WAIT_MS, ifname);
  deadline = k_uptime_get() + JSD_SLAVEINFO_POST_INIT_LINK_WAIT_MS;

  while (k_uptime_get() < deadline) {
    if (net_if_is_carrier_ok(iface)) {
      printf("Carrier became active on %s after ec_init().\n", ifname);
      printf("Waiting %d ms for %s link to settle after carrier.\n",
             JSD_SLAVEINFO_POST_CARRIER_SETTLE_MS, ifname);
      k_msleep(JSD_SLAVEINFO_POST_CARRIER_SETTLE_MS);
      return true;
    }

    k_msleep(100);
  }

  printf("Carrier is still off on %s after ec_init(); continuing with EtherCAT "
         "discovery.\n",
         ifname);
  return false;
}
#endif

static int jsd_slaveinfo_config_init_with_logging(const char* ifname) {
  int slave_count = -1;
  int attempt_count = 1;

#ifdef __ZEPHYR__
  attempt_count = JSD_SLAVEINFO_CONFIG_INIT_RETRIES;
#endif

  for (int attempt = 1; attempt <= attempt_count; ++attempt) {
    printf("Running ecx_config_init attempt %d on %s.\n", attempt, ifname);
    slave_count = ecx_config_init(&jsd->ecx_context);
    printf("ecx_config_init attempt %d on %s returned %d.\n",
           attempt, ifname, slave_count);

    if (slave_count > 0) {
      return slave_count;
    }

    while (jsd->ecx_context.ecaterror) {
      printf("%s", ecx_elist2string(&jsd->ecx_context));
    }

#ifdef __ZEPHYR__
    if (attempt < attempt_count) {
      printf("Waiting %d ms before retrying ecx_config_init on %s.\n",
             JSD_SLAVEINFO_CONFIG_INIT_RETRY_DELAY_MS, ifname);
      k_msleep(JSD_SLAVEINFO_CONFIG_INIT_RETRY_DELAY_MS);
    }
#endif
  }

  return slave_count;
}

char* dtype2string(uint16 dtype) {
  switch (dtype) {
    case ECT_BOOLEAN:
      sprintf(hstr, "BOOLEAN");
      break;
    case ECT_INTEGER8:
      sprintf(hstr, "INTEGER8");
      break;
    case ECT_INTEGER16:
      sprintf(hstr, "INTEGER16");
      break;
    case ECT_INTEGER32:
      sprintf(hstr, "INTEGER32");
      break;
    case ECT_INTEGER24:
      sprintf(hstr, "INTEGER24");
      break;
    case ECT_INTEGER64:
      sprintf(hstr, "INTEGER64");
      break;
    case ECT_UNSIGNED8:
      sprintf(hstr, "UNSIGNED8");
      break;
    case ECT_UNSIGNED16:
      sprintf(hstr, "UNSIGNED16");
      break;
    case ECT_UNSIGNED32:
      sprintf(hstr, "UNSIGNED32");
      break;
    case ECT_UNSIGNED24:
      sprintf(hstr, "UNSIGNED24");
      break;
    case ECT_UNSIGNED64:
      sprintf(hstr, "UNSIGNED64");
      break;
    case ECT_REAL32:
      sprintf(hstr, "REAL32");
      break;
    case ECT_REAL64:
      sprintf(hstr, "REAL64");
      break;
    case ECT_BIT1:
      sprintf(hstr, "BIT1");
      break;
    case ECT_BIT2:
      sprintf(hstr, "BIT2");
      break;
    case ECT_BIT3:
      sprintf(hstr, "BIT3");
      break;
    case ECT_BIT4:
      sprintf(hstr, "BIT4");
      break;
    case ECT_BIT5:
      sprintf(hstr, "BIT5");
      break;
    case ECT_BIT6:
      sprintf(hstr, "BIT6");
      break;
    case ECT_BIT7:
      sprintf(hstr, "BIT7");
      break;
    case ECT_BIT8:
      sprintf(hstr, "BIT8");
      break;
    case ECT_VISIBLE_STRING:
      sprintf(hstr, "VISIBLE_STRING");
      break;
    case ECT_OCTET_STRING:
      sprintf(hstr, "OCTET_STRING");
      break;
    default:
      sprintf(hstr, "Type 0x%4.4X", dtype);
  }
  return hstr;
}

char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype) {
  int     l = sizeof(usdo) - 1, i;
  uint8*  u8;
  int8*   i8;
  uint16* u16;
  int16*  i16;
  uint32* u32;
  int32*  i32;
  uint64* u64;
  int64*  i64;
  float*  sr;
  double* dr;
  char    es[32];

  memset(&usdo, 0, 128);
  ecx_SDOread(&jsd->ecx_context, slave, index, subidx, FALSE, &l, &usdo,
              EC_TIMEOUTRXM);
  if (jsd->ecx_context.ecaterror) {
    return ecx_elist2string(&jsd->ecx_context);
  } else {
    switch (dtype) {
      case ECT_BOOLEAN:
        u8 = (uint8*)&usdo[0];
        if (*u8)
          sprintf(hstr, "TRUE");
        else
          sprintf(hstr, "FALSE");
        break;
      case ECT_INTEGER8:
        i8 = (int8*)&usdo[0];
        sprintf(hstr, "0x%2.2x %d", *i8, *i8);
        break;
      case ECT_INTEGER16:
        i16 = (int16*)&usdo[0];
        sprintf(hstr, "0x%4.4x %d", *i16, *i16);
        break;
      case ECT_INTEGER32:
      case ECT_INTEGER24:
        i32 = (int32*)&usdo[0];
        sprintf(hstr, "0x%8.8x %d", *i32, *i32);
        break;
      case ECT_INTEGER64:
        i64 = (int64*)&usdo[0];
        sprintf(hstr, "0x%16.16" PRIx64 " %" PRId64, *i64, *i64);
        break;
      case ECT_UNSIGNED8:
        u8 = (uint8*)&usdo[0];
        sprintf(hstr, "0x%2.2x %u", *u8, *u8);
        break;
      case ECT_UNSIGNED16:
        u16 = (uint16*)&usdo[0];
        sprintf(hstr, "0x%4.4x %u", *u16, *u16);
        break;
      case ECT_UNSIGNED32:
      case ECT_UNSIGNED24:
        u32 = (uint32*)&usdo[0];
        sprintf(hstr, "0x%8.8x %u", *u32, *u32);
        break;
      case ECT_UNSIGNED64:
        u64 = (uint64*)&usdo[0];
        sprintf(hstr, "0x%16.16" PRIx64 " %" PRIu64, *u64, *u64);
        break;
      case ECT_REAL32:
        sr = (float*)&usdo[0];
        sprintf(hstr, "%f", *sr);
        break;
      case ECT_REAL64:
        dr = (double*)&usdo[0];
        sprintf(hstr, "%f", *dr);
        break;
      case ECT_BIT1:
      case ECT_BIT2:
      case ECT_BIT3:
      case ECT_BIT4:
      case ECT_BIT5:
      case ECT_BIT6:
      case ECT_BIT7:
      case ECT_BIT8:
        u8 = (uint8*)&usdo[0];
        sprintf(hstr, "0x%x", *u8);
        break;
      case ECT_VISIBLE_STRING:
        strcpy(hstr, usdo);
        break;
      case ECT_OCTET_STRING:
        hstr[0] = 0x00;
        for (i = 0; i < l; i++) {
          sprintf(es, "0x%2.2x ", usdo[i]);
          strcat(hstr, es);
        }
        break;
      default:
        sprintf(hstr, "Unknown type");
    }
    return hstr;
  }
}

int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset) {
  uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
  uint8  subcnt;
  int    wkc, bsize = 0, rdl;
  int32  rdat2;
  uint8  bitlen, obj_subidx;
  uint16 obj_idx;
  int    abs_offset, abs_bit;

  rdl  = sizeof(rdat);
  rdat = 0;
  /* read PDO assign subindex 0 ( = number of PDO's) */
  wkc  = ecx_SDOread(&jsd->ecx_context, slave, PDOassign, 0x00, FALSE, &rdl,
                    &rdat, EC_TIMEOUTRXM);
  rdat = etohs(rdat);
  /* positive result from slave ? */
  if ((wkc > 0) && (rdat > 0)) {
    /* number of available sub indexes */
    nidx  = rdat;
    bsize = 0;
    /* read all PDO's */
    for (idxloop = 1; idxloop <= nidx; idxloop++) {
      rdl  = sizeof(rdat);
      rdat = 0;
      /* read PDO assign */
      wkc = ecx_SDOread(&jsd->ecx_context, slave, PDOassign, (uint8)idxloop,
                        FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
      /* result is index of PDO */
      idx = etohs(rdat);
      if (idx > 0) {
        rdl    = sizeof(subcnt);
        subcnt = 0;
        /* read number of subindexes of PDO */
        wkc    = ecx_SDOread(&jsd->ecx_context, slave, idx, 0x00, FALSE, &rdl,
                          &subcnt, EC_TIMEOUTRXM);
        subidx = subcnt;
        /* for each subindex */
        for (subidxloop = 1; subidxloop <= subidx; subidxloop++) {
          rdl   = sizeof(rdat2);
          rdat2 = 0;
          /* read SDO that is mapped in PDO */
          wkc   = ecx_SDOread(&jsd->ecx_context, slave, idx, (uint8)subidxloop,
                            FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
          rdat2 = etohl(rdat2);
          /* extract bitlength of SDO */
          bitlen = LO_BYTE(rdat2);
          bsize += bitlen;
          obj_idx         = (uint16)(rdat2 >> 16);
          obj_subidx      = (uint8)((rdat2 >> 8) & 0x000000ff);
          abs_offset      = mapoffset + (bitoffset / 8);
          abs_bit         = bitoffset % 8;
          ODlist.Slave    = slave;
          ODlist.Index[0] = obj_idx;
          OElist.Entries  = 0;
          wkc             = 0;
          /* read object entry from dictionary if not a filler (0x0000:0x00) */
          if (obj_idx || obj_subidx)
            wkc = ecx_readOEsingle(&jsd->ecx_context, 0, obj_subidx, &ODlist,
                                   &OElist);
          printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit,
                 obj_idx, obj_subidx, bitlen);
          if ((wkc > 0) && OElist.Entries) {
            printf(" %-12s %s\n", dtype2string(OElist.DataType[obj_subidx]),
                   OElist.Name[obj_subidx]);
          } else
            printf("\n");
          bitoffset += bitlen;
        };
      };
    };
  };
  /* return total found bitlength (PDO) */
  return bsize;
}

int si_map_sdo(int slave) {
  int   wkc, rdl;
  int   retVal = 0;
  uint8 nSM, iSM, tSM;
  int   Tsize, outputs_bo, inputs_bo;
  uint8 SMt_bug_add;

  printf("PDO mapping according to CoE :\n");
  SMt_bug_add = 0;
  outputs_bo  = 0;
  inputs_bo   = 0;
  rdl         = sizeof(nSM);
  nSM         = 0;
  /* read SyncManager Communication Type object count */
  wkc = ecx_SDOread(&jsd->ecx_context, slave, ECT_SDO_SMCOMMTYPE, 0x00, FALSE,
                    &rdl, &nSM, EC_TIMEOUTRXM);
  /* positive result from slave ? */
  if ((wkc > 0) && (nSM > 2)) {
    /* make nSM equal to number of defined SM */
    nSM--;
    /* limit to maximum number of SM defined, if true the slave can't be
     * configured */
    if (nSM > EC_MAXSM) nSM = EC_MAXSM;
    /* iterate for every SM type defined */
    for (iSM = 2; iSM <= nSM; iSM++) {
      rdl = sizeof(tSM);
      tSM = 0;
      /* read SyncManager Communication Type */
      wkc = ecx_SDOread(&jsd->ecx_context, slave, ECT_SDO_SMCOMMTYPE, iSM + 1,
                        FALSE, &rdl, &tSM, EC_TIMEOUTRXM);
      if (wkc > 0) {
        if ((iSM == 2) && (tSM == 2))  // SM2 has type 2 == mailbox out, this is
                                       // a bug in the slave!
        {
          SMt_bug_add = 1;  // try to correct, this works if the types are 0 1 2
                            // 3 and should be 1 2 3 4
          printf("Activated SM type workaround, possible incorrect mapping.\n");
        }
        if (tSM) tSM += SMt_bug_add;  // only add if SMt > 0

        if (tSM == 3)  // outputs
        {
          /* read the assign RXPDO */
          printf(
              "  SM%1d outputs\n     addr b   index: sub bitl data_type    "
              "name\n",
              iSM);
          Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM,
                               (int)(jsd->ecx_context.slavelist[slave].outputs -
                                     (uint8*)&jsd->IOmap[0]),
                               outputs_bo);
          outputs_bo += Tsize;
        }
        if (tSM == 4)  // inputs
        {
          /* read the assign TXPDO */
          printf(
              "  SM%1d inputs\n     addr b   index: sub bitl data_type    "
              "name\n",
              iSM);
          Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM,
                               (int)(jsd->ecx_context.slavelist[slave].inputs -
                                     (uint8*)&jsd->IOmap[0]),
                               inputs_bo);
          inputs_bo += Tsize;
        }
      }
    }
  }

  /* found some I/O bits ? */
  if ((outputs_bo > 0) || (inputs_bo > 0)) retVal = 1;
  return retVal;
}

int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset) {
  uint16         a, w, c, e, er;
  uint8          eectl;
  uint16         obj_idx;
  uint8          obj_subidx;
  uint8          obj_name;
  uint8          obj_datatype;
  uint8          bitlen;
  int            totalsize;
  ec_eepromPDOt  eepPDO;
  ec_eepromPDOt* PDO;
  int            abs_offset, abs_bit;
  char           str_name[EC_MAXNAME + 1];

  eectl         = jsd->ecx_context.slavelist[slave].eep_pdi;
  totalsize     = 0;
  PDO           = &eepPDO;
  PDO->nPDO     = 0;
  PDO->Length   = 0;
  PDO->Index[1] = 0;
  for (c = 0; c < EC_MAXSM; c++) PDO->SMbitsize[c] = 0;
  if (t > 1) t = 1;
  PDO->Startpos = ecx_siifind(&jsd->ecx_context, slave, ECT_SII_PDO + t);
  if (PDO->Startpos > 0) {
    a = PDO->Startpos;
    w = ecx_siigetbyte(&jsd->ecx_context, slave, a++);
    w += (ecx_siigetbyte(&jsd->ecx_context, slave, a++) << 8);
    PDO->Length = w;
    c           = 1;
    /* traverse through all PDOs */
    do {
      PDO->nPDO++;
      PDO->Index[PDO->nPDO] = ecx_siigetbyte(&jsd->ecx_context, slave, a++);
      PDO->Index[PDO->nPDO] +=
          (ecx_siigetbyte(&jsd->ecx_context, slave, a++) << 8);
      PDO->BitSize[PDO->nPDO] = 0;
      c++;
      /* number of entries in PDO */
      e                     = ecx_siigetbyte(&jsd->ecx_context, slave, a++);
      PDO->SyncM[PDO->nPDO] = ecx_siigetbyte(&jsd->ecx_context, slave, a++);
      a++;
      obj_name = ecx_siigetbyte(&jsd->ecx_context, slave, a++);
      a += 2;
      c += 2;
      if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* active and in range SM? */
      {
        str_name[0] = 0;
        if (obj_name)
          ecx_siistring(&jsd->ecx_context, str_name, slave, obj_name);
        if (t)
          printf("  SM%1d RXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO],
                 PDO->Index[PDO->nPDO], str_name);
        else
          printf("  SM%1d TXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO],
                 PDO->Index[PDO->nPDO], str_name);
        printf("     addr b   index: sub bitl data_type    name\n");
        /* read all entries defined in PDO */
        for (er = 1; er <= e; er++) {
          c += 4;
          obj_idx = ecx_siigetbyte(&jsd->ecx_context, slave, a++);
          obj_idx += (ecx_siigetbyte(&jsd->ecx_context, slave, a++) << 8);
          obj_subidx   = ecx_siigetbyte(&jsd->ecx_context, slave, a++);
          obj_name     = ecx_siigetbyte(&jsd->ecx_context, slave, a++);
          obj_datatype = ecx_siigetbyte(&jsd->ecx_context, slave, a++);
          bitlen       = ecx_siigetbyte(&jsd->ecx_context, slave, a++);
          abs_offset   = mapoffset + (bitoffset / 8);
          abs_bit      = bitoffset % 8;

          PDO->BitSize[PDO->nPDO] += bitlen;
          a += 2;

          /* skip entry if filler (0x0000:0x00) */
          if (obj_idx || obj_subidx) {
            str_name[0] = 0;
            if (obj_name)
              ecx_siistring(&jsd->ecx_context, str_name, slave, obj_name);

            printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset,
                   abs_bit, obj_idx, obj_subidx, bitlen);
            printf(" %-12s %s\n", dtype2string(obj_datatype), str_name);
          }
          bitoffset += bitlen;
          totalsize += bitlen;
        }
        PDO->SMbitsize[PDO->SyncM[PDO->nPDO]] += PDO->BitSize[PDO->nPDO];
        c++;
      } else /* PDO deactivated because SM is 0xff or > EC_MAXSM */
      {
        c += 4 * e;
        a += 8 * e;
        c++;
      }
      if (PDO->nPDO >= (EC_MAXEEPDO - 1))
        c = PDO->Length; /* limit number of PDO entries in buffer */
    } while (c < PDO->Length);
  }
  if (eectl)
    ecx_eeprom2pdi(
        &jsd->ecx_context,
        slave); /* if eeprom control was previously pdi then restore */
  return totalsize;
}

int si_map_sii(int slave) {
  int retVal = 0;
  int Tsize, outputs_bo, inputs_bo;

  printf("PDO mapping according to SII :\n");

  outputs_bo = 0;
  inputs_bo  = 0;
  /* read the assign RXPDOs */
  Tsize = si_siiPDO(
      slave, 1,
      (int)(jsd->ecx_context.slavelist[slave].outputs - (uint8*)&jsd->IOmap),
      outputs_bo);
  outputs_bo += Tsize;
  /* read the assign TXPDOs */
  Tsize = si_siiPDO(
      slave, 0,
      (int)(jsd->ecx_context.slavelist[slave].inputs - (uint8*)&jsd->IOmap),
      inputs_bo);
  inputs_bo += Tsize;
  /* found some I/O bits ? */
  if ((outputs_bo > 0) || (inputs_bo > 0)) retVal = 1;
  return retVal;
}

void si_sdo(int cnt) {
  int i, j;

  ODlist.Entries = 0;
  memset(&ODlist, 0, sizeof(ODlist));
  if (ecx_readODlist(&jsd->ecx_context, cnt, &ODlist)) {
    printf(" CoE Object Description found, %d entries.\n", ODlist.Entries);
    for (i = 0; i < ODlist.Entries; i++) {
      ecx_readODdescription(&jsd->ecx_context, i, &ODlist);
      while (jsd->ecx_context.ecaterror)
        printf("%s", ecx_elist2string(&jsd->ecx_context));
      printf(" Index: %4.4x Datatype: %4.4x Objectcode: %2.2x Name: %s\n",
             ODlist.Index[i], ODlist.DataType[i], ODlist.ObjectCode[i],
             ODlist.Name[i]);
      memset(&OElist, 0, sizeof(OElist));
      ecx_readOE(&jsd->ecx_context, i, &ODlist, &OElist);
      while (jsd->ecx_context.ecaterror)
        printf("%s", ecx_elist2string(&jsd->ecx_context));
      for (j = 0; j < ODlist.MaxSub[i] + 1; j++) {
        if ((OElist.DataType[j] > 0) && (OElist.BitLength[j] > 0)) {
          printf(
              "  Sub: %2.2x Datatype: %4.4x Bitlength: %4.4x Obj.access: %4.4x "
              "Name: %s\n",
              j, OElist.DataType[j], OElist.BitLength[j], OElist.ObjAccess[j],
              OElist.Name[j]);
          if ((OElist.ObjAccess[j] & 0x0007)) {
            printf("          Value :%s\n",
                   SDO2string(cnt, ODlist.Index[i], j, OElist.DataType[j]));
          }
        }
      }
    }
  } else {
    while (jsd->ecx_context.ecaterror)
      printf("%s", ecx_elist2string(&jsd->ecx_context));
  }
}

static int slaveinfo(char* ifname) {
  int    cnt, j, nSM;
  uint16 ssigen;
  int    expectedWKC;
  int    rc = 1;

  printf("Starting slaveinfo\n");

  /* initialise SOEM, bind socket to ifname */
  if (ecx_init(&jsd->ecx_context, ifname)) {
    printf("ec_init on %s succeeded.\n", ifname);
#ifdef __ZEPHYR__
    jsd_slaveinfo_wait_for_zephyr_carrier_after_ec_init(ifname);
#endif
    /* find and auto-config slaves */
    int slave_count = jsd_slaveinfo_config_init_with_logging(ifname);

    if (slave_count > 0) {
      printf("%d slaves found and configured.\n", jsd->ecx_context.slavecount);
      rc = 0;

#ifdef __ZEPHYR__
      printf("slaveinfo: configuring PDO map\n");
      jsd_slaveinfo_log_stack_space("before PDO map");
#endif
      int iomap_size = ecx_config_map_group(&jsd->ecx_context, &jsd->IOmap, 0);

#ifdef __ZEPHYR__
      printf("slaveinfo: PDO map size %d bytes\n", iomap_size);
      jsd_slaveinfo_log_stack_space("after PDO map");
      printf("slaveinfo: waiting for SAFE-OP\n");
#endif
      if (iomap_size <= 0 || iomap_size > (int)sizeof(jsd->IOmap)) {
        printf("slaveinfo: invalid PDO map size %d (buffer=%zu)\n",
               iomap_size, sizeof(jsd->IOmap));
        rc = 3;
        printf("End slaveinfo\n");
        return rc;
      }
      ecx_statecheck(&jsd->ecx_context, 0, EC_STATE_SAFE_OP,
                     EC_TIMEOUTSTATE * 3);

#ifdef __ZEPHYR__
      jsd_slaveinfo_log_stack_space("before configdc");
      printf("slaveinfo: configuring distributed clocks\n");
#endif
      ecx_configdc(&jsd->ecx_context);

      expectedWKC = (jsd->ecx_context.grouplist[0].outputsWKC * 2) +
                    jsd->ecx_context.grouplist[0].inputsWKC;

      printf("Calculated workcounter %d\n", expectedWKC);

#ifdef __ZEPHYR__
      jsd_slaveinfo_log_stack_space("before readstate");
      printf("slaveinfo: reading slave states\n");
#endif
      ecx_readstate(&jsd->ecx_context);

      for (cnt = 1; cnt <= jsd->ecx_context.slavecount; cnt++) {
        printf(
            "\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n "
            "State: %d\n Delay: %d[ns]\n Has DC: %d\n",
            cnt, jsd->ecx_context.slavelist[cnt].name,
            jsd->ecx_context.slavelist[cnt].Obits,
            jsd->ecx_context.slavelist[cnt].Ibits,
            jsd->ecx_context.slavelist[cnt].state,
            jsd->ecx_context.slavelist[cnt].pdelay,
            jsd->ecx_context.slavelist[cnt].hasdc);
        if (jsd->ecx_context.slavelist[cnt].hasdc) {
          printf(" DCParentport:%d\n",
                 jsd->ecx_context.slavelist[cnt].parentport);
        }
        printf(" Activeports:%d.%d.%d.%d\n",
               (jsd->ecx_context.slavelist[cnt].activeports & 0x01) > 0,
               (jsd->ecx_context.slavelist[cnt].activeports & 0x02) > 0,
               (jsd->ecx_context.slavelist[cnt].activeports & 0x04) > 0,
               (jsd->ecx_context.slavelist[cnt].activeports & 0x08) > 0);
        printf(" Configured address: %4.4x\n",
               jsd->ecx_context.slavelist[cnt].configadr);
        printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n",
               (int)jsd->ecx_context.slavelist[cnt].eep_man,
               (int)jsd->ecx_context.slavelist[cnt].eep_id,
               (int)jsd->ecx_context.slavelist[cnt].eep_rev);
        for (nSM = 0; nSM < EC_MAXSM; nSM++) {
          if (jsd->ecx_context.slavelist[cnt].SM[nSM].StartAddr > 0)
            printf(" SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n", nSM,
                   etohs(jsd->ecx_context.slavelist[cnt].SM[nSM].StartAddr),
                   etohs(jsd->ecx_context.slavelist[cnt].SM[nSM].SMlength),
                   etohl(jsd->ecx_context.slavelist[cnt].SM[nSM].SMflags),
                   jsd->ecx_context.slavelist[cnt].SMtype[nSM]);
        }
        for (j = 0; j < jsd->ecx_context.slavelist[cnt].FMMUunused; j++) {
          printf(
              " FMMU%1d Ls:%8.8x Ll:%4d Lsb:%d Leb:%d Ps:%4.4x Psb:%d Ty:%2.2x "
              "Act:%2.2x\n",
              j, etohl(jsd->ecx_context.slavelist[cnt].FMMU[j].LogStart),
              etohs(jsd->ecx_context.slavelist[cnt].FMMU[j].LogLength),
              jsd->ecx_context.slavelist[cnt].FMMU[j].LogStartbit,
              jsd->ecx_context.slavelist[cnt].FMMU[j].LogEndbit,
              etohs(jsd->ecx_context.slavelist[cnt].FMMU[j].PhysStart),
              jsd->ecx_context.slavelist[cnt].FMMU[j].PhysStartBit,
              jsd->ecx_context.slavelist[cnt].FMMU[j].FMMUtype,
              jsd->ecx_context.slavelist[cnt].FMMU[j].FMMUactive);
        }
        printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
               jsd->ecx_context.slavelist[cnt].FMMU0func,
               jsd->ecx_context.slavelist[cnt].FMMU1func,
               jsd->ecx_context.slavelist[cnt].FMMU2func,
               jsd->ecx_context.slavelist[cnt].FMMU3func);

        printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n",
               jsd->ecx_context.slavelist[cnt].mbx_l,
               jsd->ecx_context.slavelist[cnt].mbx_rl,
               jsd->ecx_context.slavelist[cnt].mbx_proto);
        ssigen = ecx_siifind(&jsd->ecx_context, cnt, ECT_SII_GENERAL);
        /* SII general section */
        if (ssigen) {
          jsd->ecx_context.slavelist[cnt].CoEdetails =
              ecx_siigetbyte(&jsd->ecx_context, cnt, ssigen + 0x07);
          jsd->ecx_context.slavelist[cnt].FoEdetails =
              ecx_siigetbyte(&jsd->ecx_context, cnt, ssigen + 0x08);
          jsd->ecx_context.slavelist[cnt].EoEdetails =
              ecx_siigetbyte(&jsd->ecx_context, cnt, ssigen + 0x09);
          jsd->ecx_context.slavelist[cnt].SoEdetails =
              ecx_siigetbyte(&jsd->ecx_context, cnt, ssigen + 0x0a);
          if ((ecx_siigetbyte(&jsd->ecx_context, cnt, ssigen + 0x0d) & 0x02) >
              0) {
            jsd->ecx_context.slavelist[cnt].blockLRW = 1;
            jsd->ecx_context.slavelist[0].blockLRW++;
          }
          jsd->ecx_context.slavelist[cnt].Ebuscurrent =
              ecx_siigetbyte(&jsd->ecx_context, cnt, ssigen + 0x0e);
          jsd->ecx_context.slavelist[cnt].Ebuscurrent +=
              ecx_siigetbyte(&jsd->ecx_context, cnt, ssigen + 0x0f) << 8;
          jsd->ecx_context.slavelist[0].Ebuscurrent +=
              jsd->ecx_context.slavelist[cnt].Ebuscurrent;
        }
        printf(
            " CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE "
            "details: %2.2x\n",
            jsd->ecx_context.slavelist[cnt].CoEdetails,
            jsd->ecx_context.slavelist[cnt].FoEdetails,
            jsd->ecx_context.slavelist[cnt].EoEdetails,
            jsd->ecx_context.slavelist[cnt].SoEdetails);

        printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n",
               jsd->ecx_context.slavelist[cnt].Ebuscurrent,
               jsd->ecx_context.slavelist[cnt].blockLRW);

        if ((jsd->ecx_context.slavelist[cnt].mbx_proto & ECT_MBXPROT_COE) &&
            printSDO)
          si_sdo(cnt);
        if (printMAP) {
          if (jsd->ecx_context.slavelist[cnt].mbx_proto & ECT_MBXPROT_COE)
            si_map_sdo(cnt);
          else
            si_map_sii(cnt);
        }
      }
    } else if (slave_count == 0) {
      printf("No slaves found!\n");
      rc = 2;
    } else {
      printf("Slave discovery failed on %s\n", ifname);
    }
    printf("End slaveinfo\n");
  } else {
    printf("No socket connection on %s\nExcecute as root\n", ifname);
  }

  return rc;
}

void jsd_slaveinfo_print_usage(void) {
  ec_adaptert* adapter = NULL;
  ec_adaptert* head = NULL;

  printf("Usage: slaveinfo ifname [options]\n");
  printf("ifname = network interface name, for example eth0\n");
  printf("Options :\n -sdo : print SDO info\n -map : print mapping\n");

  printf("Available adapters\n");
  head = adapter = ec_find_adapters();
  while (adapter != NULL) {
    printf("Description : %s, Device to use for wpcap: %s\n", adapter->desc,
           adapter->name);
    adapter = adapter->next;
  }
  ec_free_adapters(head);
}

int jsd_slaveinfo_run(const char* ifname, bool enable_print_sdo,
                      bool enable_print_map) {
  char ifbuf[1024];
  int  rc = 1;

  if (ifname == NULL || ifname[0] == '\0') {
    jsd_slaveinfo_print_usage();
    return 1;
  }

  printSDO = enable_print_sdo ? TRUE : FALSE;
  printMAP = enable_print_map ? TRUE : FALSE;

  jsd = jsd_alloc();
  snprintf(ifbuf, sizeof(ifbuf), "%s", ifname);
  rc = slaveinfo(ifbuf);

#ifdef __ZEPHYR__
  if (rc == 0) {
    jsd_retained_on_success = jsd;
    jsd = NULL;
    printf("Retaining JSD context on Zephyr after successful slaveinfo.\n");
    return rc;
  }
#endif

  jsd_free(jsd);
  jsd = NULL;

  return rc;
}

#ifndef __ZEPHYR__
int main(int argc, char* argv[]) {
  int  rc        = 0;
  bool print_sdo = false;
  bool print_map = false;

  printf("jsd (Simple Open EtherCAT Master)\nSlaveinfo\n");

  if (argc > 1) {
    if ((argc > 2) && (strncmp(argv[2], "-sdo", sizeof("-sdo")) == 0)) {
      print_sdo = true;
    }
    if ((argc > 2) && (strncmp(argv[2], "-map", sizeof("-map")) == 0)) {
      print_map = true;
    }

    rc = jsd_slaveinfo_run(argv[1], print_sdo, print_map);
  } else {
    jsd_slaveinfo_print_usage();
    rc = 1;
  }

  printf("End program\n");
  return rc;
}
#endif
