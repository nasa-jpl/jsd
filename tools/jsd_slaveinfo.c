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
#include <stdio.h>
#include <string.h>

#include "jsd/jsd.h"

ec_ODlistt ODlist;
ec_OElistt OElist;
boolean    printSDO = FALSE;
boolean    printMAP = FALSE;
char       usdo[128];
char       hstr[1024];
jsd_t*     jsd;
FILE*      json_file = NULL;
char       json_output_path[1024] = "jsd_slaveinfo.json";
int        json_depth             = 0;
int        json_first[64];

static int json_is_enabled(void) {
  return json_file != NULL;
}

static void json_write_indent(void) {
  int depth = 0;

  if (!json_is_enabled()) {
    return;
  }

  for (depth = 0; depth < json_depth; depth++) {
    fputs("  ", json_file);
  }
}

static void json_begin_value(void) {
  if (!json_is_enabled() || json_depth <= 0) {
    return;
  }

  if (json_first[json_depth - 1]) {
    fputc('\n', json_file);
    json_first[json_depth - 1] = 0;
  } else {
    fputs(",\n", json_file);
  }
  json_write_indent();
}

static void json_write_escaped_string(const char* value) {
  const unsigned char* ptr = (const unsigned char*)value;

  if (!json_is_enabled()) {
    return;
  }

  if (value == NULL) {
    fputs("\"\"", json_file);
    return;
  }

  fputc('"', json_file);
  while (*ptr != '\0') {
    switch (*ptr) {
      case '\\':
        fputs("\\\\", json_file);
        break;
      case '"':
        fputs("\\\"", json_file);
        break;
      case '\b':
        fputs("\\b", json_file);
        break;
      case '\f':
        fputs("\\f", json_file);
        break;
      case '\n':
        fputs("\\n", json_file);
        break;
      case '\r':
        fputs("\\r", json_file);
        break;
      case '\t':
        fputs("\\t", json_file);
        break;
      default:
        if (*ptr < 0x20) {
          fprintf(json_file, "\\u%4.4x", *ptr);
        } else {
          fputc(*ptr, json_file);
        }
    }
    ptr++;
  }
  fputc('"', json_file);
}

static void json_write_key(const char* key) {
  json_begin_value();
  json_write_escaped_string(key);
  fputs(": ", json_file);
}

static void json_begin_root_object(void) {
  if (!json_is_enabled()) {
    return;
  }

  json_depth = 0;
  memset(json_first, 0, sizeof(json_first));
  fputc('{', json_file);
  json_first[json_depth++] = 1;
}

static void json_end_container(char close_char) {
  if (!json_is_enabled() || json_depth <= 0) {
    return;
  }

  json_depth--;
  if (!json_first[json_depth]) {
    fputc('\n', json_file);
    json_write_indent();
  }
  fputc(close_char, json_file);
}

static void json_begin_object_property(const char* key) {
  if (!json_is_enabled()) {
    return;
  }

  json_write_key(key);
  fputc('{', json_file);
  json_first[json_depth++] = 1;
}

static void json_begin_array_property(const char* key) {
  if (!json_is_enabled()) {
    return;
  }

  json_write_key(key);
  fputc('[', json_file);
  json_first[json_depth++] = 1;
}

static void json_begin_object_element(void) {
  if (!json_is_enabled()) {
    return;
  }

  json_begin_value();
  fputc('{', json_file);
  json_first[json_depth++] = 1;
}

static void json_end_object(void) {
  json_end_container('}');
}

static void json_end_array(void) {
  json_end_container(']');
}

static void json_property_string(const char* key, const char* value) {
  if (!json_is_enabled()) {
    return;
  }

  json_write_key(key);
  json_write_escaped_string(value);
}

static void json_property_int(const char* key, int value) {
  if (!json_is_enabled()) {
    return;
  }

  json_write_key(key);
  fprintf(json_file, "%d", value);
}

static void json_property_uint(const char* key, unsigned int value) {
  if (!json_is_enabled()) {
    return;
  }

  json_write_key(key);
  fprintf(json_file, "%u", value);
}

static void json_property_bool(const char* key, int value) {
  if (!json_is_enabled()) {
    return;
  }

  json_write_key(key);
  fputs(value ? "true" : "false", json_file);
}

static void json_open_output(const char* ifname) {
  snprintf(json_output_path, sizeof(json_output_path), "jsd_slaveinfo_%s.json",
           ifname);
  json_file = fopen(json_output_path, "w");
  if (!json_file) {
    fprintf(stderr, "Failed to open %s for JSON output\n", json_output_path);
    return;
  }

  json_begin_root_object();
  json_property_string("interface", ifname);
  json_property_bool("print_sdo", printSDO);
  json_property_bool("print_map", printMAP);
}

static void json_close_output(const char* status) {
  if (!json_is_enabled()) {
    return;
  }

  json_property_string("status", status);
  json_end_object();
  fputc('\n', json_file);
  fclose(json_file);
  json_file  = NULL;
  json_depth = 0;
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
  if (*jsd->ecx_context.ecaterror) {
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
          if (json_is_enabled()) {
            json_begin_object_element();
            json_property_int("offset_bytes", abs_offset);
            json_property_int("offset_bits", abs_bit);
            json_property_uint("object_index", obj_idx);
            json_property_uint("subindex", obj_subidx);
            json_property_uint("bit_length", bitlen);
            if ((wkc > 0) && OElist.Entries) {
              json_property_string("data_type",
                                   dtype2string(OElist.DataType[obj_subidx]));
              json_property_string("name", OElist.Name[obj_subidx]);
            }
            json_end_object();
          }
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
  int   used_sm_type_workaround;

  printf("PDO mapping according to CoE :\n");
  if (json_is_enabled()) {
    json_begin_object_property("mapping");
    json_property_string("source", "coe");
  }
  SMt_bug_add = 0;
  outputs_bo  = 0;
  inputs_bo   = 0;
  used_sm_type_workaround = 0;
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
          used_sm_type_workaround = 1;
          printf("Activated SM type workaround, possible incorrect mapping.\n");
        }
        if (tSM) tSM += SMt_bug_add;  // only add if SMt > 0
        if ((iSM == 2) && json_is_enabled()) {
          json_property_bool("used_sm_type_workaround", used_sm_type_workaround);
          json_begin_array_property("sync_managers");
        }

        if (tSM == 3)  // outputs
        {
          /* read the assign RXPDO */
          printf(
              "  SM%1d outputs\n     addr b   index: sub bitl data_type    "
              "name\n",
              iSM);
          if (json_is_enabled()) {
            json_begin_object_element();
            json_property_uint("index", iSM);
            json_property_string("direction", "outputs");
            json_begin_array_property("entries");
          }
          Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM,
                               (int)(jsd->ecx_context.slavelist[slave].outputs -
                                     (uint8*)&jsd->IOmap[0]),
                               outputs_bo);
          if (json_is_enabled()) {
            json_end_array();
            json_end_object();
          }
          outputs_bo += Tsize;
        }
        if (tSM == 4)  // inputs
        {
          /* read the assign TXPDO */
          printf(
              "  SM%1d inputs\n     addr b   index: sub bitl data_type    "
              "name\n",
              iSM);
          if (json_is_enabled()) {
            json_begin_object_element();
            json_property_uint("index", iSM);
            json_property_string("direction", "inputs");
            json_begin_array_property("entries");
          }
          Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM,
                               (int)(jsd->ecx_context.slavelist[slave].inputs -
                                     (uint8*)&jsd->IOmap[0]),
                               inputs_bo);
          if (json_is_enabled()) {
            json_end_array();
            json_end_object();
          }
          inputs_bo += Tsize;
        }
      }
    }
  }
  if (json_is_enabled() && ((wkc <= 0) || (nSM <= 2))) {
    json_property_bool("used_sm_type_workaround", 0);
    json_begin_array_property("sync_managers");
  }

  /* found some I/O bits ? */
  if ((outputs_bo > 0) || (inputs_bo > 0)) retVal = 1;
  if (json_is_enabled()) {
    json_end_array();
    json_end_object();
  }
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
        if (json_is_enabled()) {
          json_begin_object_element();
          json_property_uint("sync_manager", PDO->SyncM[PDO->nPDO]);
          json_property_string("pdo_type", t ? "rxpdo" : "txpdo");
          json_property_uint("pdo_index", PDO->Index[PDO->nPDO]);
          json_property_string("name", str_name);
          json_begin_array_property("entries");
        }
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
            if (json_is_enabled()) {
              json_begin_object_element();
              json_property_int("offset_bytes", abs_offset);
              json_property_int("offset_bits", abs_bit);
              json_property_uint("object_index", obj_idx);
              json_property_uint("subindex", obj_subidx);
              json_property_uint("bit_length", bitlen);
              json_property_string("data_type", dtype2string(obj_datatype));
              json_property_string("name", str_name);
              json_end_object();
            }
          }
          bitoffset += bitlen;
          totalsize += bitlen;
        }
        if (json_is_enabled()) {
          json_end_array();
          json_end_object();
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
  if (json_is_enabled()) {
    json_begin_object_property("mapping");
    json_property_string("source", "sii");
    json_begin_array_property("pdos");
  }

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
  if (json_is_enabled()) {
    json_end_array();
    json_end_object();
  }
  return retVal;
}

void si_sdo(int cnt) {
  int i, j;
  char sdo_value[sizeof(hstr)];

  ODlist.Entries = 0;
  memset(&ODlist, 0, sizeof(ODlist));
  if (json_is_enabled()) {
    json_begin_array_property("sdo_objects");
  }
  if (ecx_readODlist(&jsd->ecx_context, cnt, &ODlist)) {
    printf(" CoE Object Description found, %d entries.\n", ODlist.Entries);
    for (i = 0; i < ODlist.Entries; i++) {
      ecx_readODdescription(&jsd->ecx_context, i, &ODlist);
      while (*jsd->ecx_context.ecaterror)
        printf("%s", ecx_elist2string(&jsd->ecx_context));
      printf(" Index: %4.4x Datatype: %4.4x Objectcode: %2.2x Name: %s\n",
             ODlist.Index[i], ODlist.DataType[i], ODlist.ObjectCode[i],
             ODlist.Name[i]);
      if (json_is_enabled()) {
        json_begin_object_element();
        json_property_uint("index", ODlist.Index[i]);
        json_property_uint("data_type", ODlist.DataType[i]);
        json_property_uint("object_code", ODlist.ObjectCode[i]);
        json_property_string("name", ODlist.Name[i]);
        json_begin_array_property("subentries");
      }
      memset(&OElist, 0, sizeof(OElist));
      ecx_readOE(&jsd->ecx_context, i, &ODlist, &OElist);
      while (*jsd->ecx_context.ecaterror)
        printf("%s", ecx_elist2string(&jsd->ecx_context));
      for (j = 0; j < ODlist.MaxSub[i] + 1; j++) {
        if ((OElist.DataType[j] > 0) && (OElist.BitLength[j] > 0)) {
          printf(
              "  Sub: %2.2x Datatype: %4.4x Bitlength: %4.4x Obj.access: %4.4x "
              "Name: %s\n",
              j, OElist.DataType[j], OElist.BitLength[j], OElist.ObjAccess[j],
              OElist.Name[j]);
          if (json_is_enabled()) {
            json_begin_object_element();
            json_property_int("subindex", j);
            json_property_uint("data_type", OElist.DataType[j]);
            json_property_uint("bit_length", OElist.BitLength[j]);
            json_property_uint("object_access", OElist.ObjAccess[j]);
            json_property_string("name", OElist.Name[j]);
          }
          if ((OElist.ObjAccess[j] & 0x0007)) {
            snprintf(sdo_value, sizeof(sdo_value), "%s",
                     SDO2string(cnt, ODlist.Index[i], j, OElist.DataType[j]));
            printf("          Value :%s\n",
                   sdo_value);
            if (json_is_enabled()) {
              json_property_string("value", sdo_value);
            }
          }
          if (json_is_enabled()) {
            json_end_object();
          }
        }
      }
      if (json_is_enabled()) {
        json_end_array();
        json_end_object();
      }
    }
  } else {
    while (*jsd->ecx_context.ecaterror)
      printf("%s", ecx_elist2string(&jsd->ecx_context));
  }
  if (json_is_enabled()) {
    json_end_array();
  }
}

void slaveinfo(char* ifname) {
  int    cnt, j, nSM;
  uint16 ssigen;
  int    expectedWKC;
  const char* status = "unknown";

  printf("Starting slaveinfo\n");
  json_open_output(ifname);

  /* initialise SOEM, bind socket to ifname */
  if (ecx_init(&jsd->ecx_context, ifname)) {
    printf("ec_init on %s succeeded.\n", ifname);
    /* find and auto-config slaves */
    if (ecx_config_init(&jsd->ecx_context, FALSE) > 0) {
      status = "ok";
      if (json_is_enabled()) {
        json_property_int("slave_count", *jsd->ecx_context.slavecount);
      }
      printf("%d slaves found and configured.\n", *jsd->ecx_context.slavecount);

      ecx_config_map_group(&jsd->ecx_context, &jsd->IOmap, 0);

      ecx_statecheck(&jsd->ecx_context, 0, EC_STATE_SAFE_OP,
                     EC_TIMEOUTSTATE * 3);

      ecx_configdc(&jsd->ecx_context);

      expectedWKC = (jsd->ecx_context.grouplist[0].outputsWKC * 2) +
                    jsd->ecx_context.grouplist[0].inputsWKC;

      printf("Calculated workcounter %d\n", expectedWKC);
      if (json_is_enabled()) {
        json_property_int("expected_workcounter", expectedWKC);
        json_begin_array_property("slaves");
      }

      ecx_readstate(&jsd->ecx_context);

      for (cnt = 1; cnt <= *jsd->ecx_context.slavecount; cnt++) {
        printf(
            "\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n "
            "State: %d\n Delay: %d[ns]\n Has DC: %d\n",
            cnt, jsd->ecx_context.slavelist[cnt].name,
            jsd->ecx_context.slavelist[cnt].Obits,
            jsd->ecx_context.slavelist[cnt].Ibits,
            jsd->ecx_context.slavelist[cnt].state,
            jsd->ecx_context.slavelist[cnt].pdelay,
            jsd->ecx_context.slavelist[cnt].hasdc);
        if (json_is_enabled()) {
          json_begin_object_element();
          json_property_int("index", cnt);
          json_property_string("name", jsd->ecx_context.slavelist[cnt].name);
          json_property_int("output_bits", jsd->ecx_context.slavelist[cnt].Obits);
          json_property_int("input_bits", jsd->ecx_context.slavelist[cnt].Ibits);
          json_property_int("state", jsd->ecx_context.slavelist[cnt].state);
          json_property_int("delay_ns", jsd->ecx_context.slavelist[cnt].pdelay);
          json_property_bool("has_dc", jsd->ecx_context.slavelist[cnt].hasdc);
        }
        if (jsd->ecx_context.slavelist[cnt].hasdc) {
          printf(" DCParentport:%d\n",
                 jsd->ecx_context.slavelist[cnt].parentport);
          if (json_is_enabled()) {
            json_property_int("dc_parent_port",
                              jsd->ecx_context.slavelist[cnt].parentport);
          }
        }
        printf(" Activeports:%d.%d.%d.%d\n",
               (jsd->ecx_context.slavelist[cnt].activeports & 0x01) > 0,
               (jsd->ecx_context.slavelist[cnt].activeports & 0x02) > 0,
               (jsd->ecx_context.slavelist[cnt].activeports & 0x04) > 0,
               (jsd->ecx_context.slavelist[cnt].activeports & 0x08) > 0);
        if (json_is_enabled()) {
          json_begin_object_property("active_ports");
          json_property_bool(
              "port0", (jsd->ecx_context.slavelist[cnt].activeports & 0x01) > 0);
          json_property_bool(
              "port1", (jsd->ecx_context.slavelist[cnt].activeports & 0x02) > 0);
          json_property_bool(
              "port2", (jsd->ecx_context.slavelist[cnt].activeports & 0x04) > 0);
          json_property_bool(
              "port3", (jsd->ecx_context.slavelist[cnt].activeports & 0x08) > 0);
          json_end_object();
        }
        printf(" Configured address: %4.4x\n",
               jsd->ecx_context.slavelist[cnt].configadr);
        if (json_is_enabled()) {
          json_property_uint("configured_address",
                             jsd->ecx_context.slavelist[cnt].configadr);
        }
        printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n",
               (int)jsd->ecx_context.slavelist[cnt].eep_man,
               (int)jsd->ecx_context.slavelist[cnt].eep_id,
               (int)jsd->ecx_context.slavelist[cnt].eep_rev);
        if (json_is_enabled()) {
          json_property_uint("manufacturer",
                             (unsigned int)jsd->ecx_context.slavelist[cnt]
                                 .eep_man);
          json_property_uint("product_id",
                             (unsigned int)jsd->ecx_context.slavelist[cnt]
                                 .eep_id);
          json_property_uint("revision",
                             (unsigned int)jsd->ecx_context.slavelist[cnt]
                                 .eep_rev);
          json_begin_array_property("sync_managers");
        }
        for (nSM = 0; nSM < EC_MAXSM; nSM++) {
          if (jsd->ecx_context.slavelist[cnt].SM[nSM].StartAddr > 0) {
            printf(" SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n", nSM,
                   etohs(jsd->ecx_context.slavelist[cnt].SM[nSM].StartAddr),
                   etohs(jsd->ecx_context.slavelist[cnt].SM[nSM].SMlength),
                   etohl(jsd->ecx_context.slavelist[cnt].SM[nSM].SMflags),
                   jsd->ecx_context.slavelist[cnt].SMtype[nSM]);
            if (json_is_enabled()) {
              json_begin_object_element();
              json_property_int("index", nSM);
              json_property_uint(
                  "start_address",
                  etohs(jsd->ecx_context.slavelist[cnt].SM[nSM].StartAddr));
              json_property_uint(
                  "length",
                  etohs(jsd->ecx_context.slavelist[cnt].SM[nSM].SMlength));
              json_property_uint(
                  "flags", etohl(jsd->ecx_context.slavelist[cnt].SM[nSM].SMflags));
              json_property_int("type", jsd->ecx_context.slavelist[cnt].SMtype[nSM]);
              json_end_object();
            }
          }
        }
        if (json_is_enabled()) {
          json_end_array();
          json_begin_array_property("fmmus");
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
          if (json_is_enabled()) {
            json_begin_object_element();
            json_property_int("index", j);
            json_property_uint(
                "logical_start", etohl(jsd->ecx_context.slavelist[cnt].FMMU[j].LogStart));
            json_property_uint(
                "logical_length",
                etohs(jsd->ecx_context.slavelist[cnt].FMMU[j].LogLength));
            json_property_uint("logical_start_bit",
                               jsd->ecx_context.slavelist[cnt].FMMU[j].LogStartbit);
            json_property_uint("logical_end_bit",
                               jsd->ecx_context.slavelist[cnt].FMMU[j].LogEndbit);
            json_property_uint(
                "physical_start", etohs(jsd->ecx_context.slavelist[cnt].FMMU[j].PhysStart));
            json_property_uint(
                "physical_start_bit",
                jsd->ecx_context.slavelist[cnt].FMMU[j].PhysStartBit);
            json_property_uint("type", jsd->ecx_context.slavelist[cnt].FMMU[j].FMMUtype);
            json_property_uint("active",
                               jsd->ecx_context.slavelist[cnt].FMMU[j].FMMUactive);
            json_end_object();
          }
        }
        if (json_is_enabled()) {
          json_end_array();
        }
        printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
               jsd->ecx_context.slavelist[cnt].FMMU0func,
               jsd->ecx_context.slavelist[cnt].FMMU1func,
               jsd->ecx_context.slavelist[cnt].FMMU2func,
               jsd->ecx_context.slavelist[cnt].FMMU3func);
        if (json_is_enabled()) {
          json_begin_object_property("fmmu_functions");
          json_property_int("fmmu0", jsd->ecx_context.slavelist[cnt].FMMU0func);
          json_property_int("fmmu1", jsd->ecx_context.slavelist[cnt].FMMU1func);
          json_property_int("fmmu2", jsd->ecx_context.slavelist[cnt].FMMU2func);
          json_property_int("fmmu3", jsd->ecx_context.slavelist[cnt].FMMU3func);
          json_end_object();
        }

        printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n",
               jsd->ecx_context.slavelist[cnt].mbx_l,
               jsd->ecx_context.slavelist[cnt].mbx_rl,
               jsd->ecx_context.slavelist[cnt].mbx_proto);
        if (json_is_enabled()) {
          json_begin_object_property("mailbox");
          json_property_int("write_length", jsd->ecx_context.slavelist[cnt].mbx_l);
          json_property_int("read_length", jsd->ecx_context.slavelist[cnt].mbx_rl);
          json_property_uint("protocols", jsd->ecx_context.slavelist[cnt].mbx_proto);
          json_end_object();
        }
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
        if (json_is_enabled()) {
          json_begin_object_property("details");
          json_property_uint("coe", jsd->ecx_context.slavelist[cnt].CoEdetails);
          json_property_uint("foe", jsd->ecx_context.slavelist[cnt].FoEdetails);
          json_property_uint("eoe", jsd->ecx_context.slavelist[cnt].EoEdetails);
          json_property_uint("soe", jsd->ecx_context.slavelist[cnt].SoEdetails);
          json_property_int("ebus_current_ma",
                            jsd->ecx_context.slavelist[cnt].Ebuscurrent);
          json_property_int("only_lrd_lwr",
                            jsd->ecx_context.slavelist[cnt].blockLRW);
          json_end_object();
        }

        printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n",
               jsd->ecx_context.slavelist[cnt].Ebuscurrent,
               jsd->ecx_context.slavelist[cnt].blockLRW);

        if (printSDO) {
          if (jsd->ecx_context.slavelist[cnt].mbx_proto & ECT_MBXPROT_COE)
            si_sdo(cnt);
          else if (json_is_enabled()) {
            json_begin_array_property("sdo_objects");
            json_end_array();
          }
        }
        if (printMAP) {
          if (jsd->ecx_context.slavelist[cnt].mbx_proto & ECT_MBXPROT_COE)
            si_map_sdo(cnt);
          else
            si_map_sii(cnt);
        }
        if (json_is_enabled()) {
          json_end_object();
        }
      }
      if (json_is_enabled()) {
        json_end_array();
      }
    } else {
      status = "no_slaves";
      printf("No slaves found!\n");
      if (json_is_enabled()) {
        json_property_int("slave_count", 0);
        json_begin_array_property("slaves");
        json_end_array();
      }
    }
    printf("End slaveinfo, close socket\n");
    /* stop SOEM, close socket */
    ecx_close(&jsd->ecx_context);
  } else {
    status = "no_socket";
    printf("No socket connection on %s\nExcecute as root\n", ifname);
    if (json_is_enabled()) {
      json_property_int("slave_count", 0);
      json_begin_array_property("slaves");
      json_end_array();
    }
  }
  json_close_output(status);
}

int main(int argc, char* argv[]) {
  ec_adaptert* adapter = NULL;
  printf("jsd (Simple Open EtherCAT Master)\nSlaveinfo\n");
  char ifbuf[1024];
  jsd = jsd_alloc();

  if (argc > 1) {
    if ((argc > 2) && (strncmp(argv[2], "-sdo", sizeof("-sdo")) == 0))
      printSDO = TRUE;
    if ((argc > 2) && (strncmp(argv[2], "-map", sizeof("-map")) == 0))
      printMAP = TRUE;
    /* start slaveinfo */
    strcpy(ifbuf, argv[1]);
    slaveinfo(ifbuf);
  } else {
    printf("Usage: slaveinfo ifname [options]\nifname = eth0 for example\n");
    printf("Options :\n -sdo : print SDO info\n -map : print mapping\n");

    printf("Available adapters\n");
    adapter = ec_find_adapters();
    while (adapter != NULL) {
      printf("Description : %s, Device to use for wpcap: %s\n", adapter->desc,
             adapter->name);
      adapter = adapter->next;
    }
  }

  jsd_free(jsd);
  printf("End program\n");
  return (0);
}
