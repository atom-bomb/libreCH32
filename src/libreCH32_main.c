#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <time.h>
#include <endian.h>

#include "debug_print.h"
#include "libreCH32.h"

int
main(
  int argc,
  char **argv) {

  int ret = 0;
  bool show_help = false;
  char opt;
  extern char *optarg;
  extern int optind, optopt, opterr;

  char *serial_port_dev = NULL;
  char *verify_file = NULL;
  char *write_file = NULL;
  bool reboot = false;
  bool set_config = false;
  uint8_t set_config_bits = 0;

  char buffer[LIBRECH32_MAX_BLOCK_SIZE];
  FILE *fp;
  uint32_t addr = 0;
  bool done = false;
  ssize_t len = 0;
  uint8_t config_bits = 0;

  while ((opt = getopt(argc, argv, "p:V:W:w:b:rvh?")) != -1) {
    switch(opt) {
      case 'p':
        serial_port_dev = optarg;
        break;

      case 'w':
        write_file = optarg;
        break;

      case 'V':
        verify_file = optarg;
        break;

      case 'W':
        write_file = optarg;
        verify_file = optarg;
        break;

      case 'b':
        set_config = true;
        set_config_bits = (uint8_t)strtoul(optarg, NULL, 0);
        break;

      case 'r':
        reboot = true;
        break;

      case 'v':
        debug_print_verbosity++;
        break;

      case '?':
      case 'h':
        show_help = true;
        break;
    } /* switch */
  } /* while */

  if (show_help) {
    fprintf(stderr, "%s usage:\n", argv[0]);
    fprintf(stderr, "----------------------------------------------\n");
    fprintf(stderr, "-p <path>                 : set serial port\n");
    fprintf(stderr, "-b 0xXX                   : set config bits\n");
    fprintf(stderr, "-W <path>                 : write and verify data\n");
    fprintf(stderr, "-V <path>                 : verify data\n");
    fprintf(stderr, "-w <path>                 : write data\n");
    fprintf(stderr, "-r                        : reboot before exiting\n");
    fprintf(stderr, "-v                        : more verbose\n");
    fprintf(stderr, "-h                        : show help\n");
    fprintf(stderr, "----------------------------------------------\n");
    fprintf(stderr, "config bits:\n");
    fprintf(stderr, "  Enable Read Protect       0x01\n");
    fprintf(stderr, "  Disable Long Delay        0x02\n");
    fprintf(stderr, "  Enable USBD Low Speed     0x04\n");
    fprintf(stderr, "  Enable USBD Pull-Up       0x08\n");
    fprintf(stderr, "  Enable Stop Mode Reset    0x10\n");
    fprintf(stderr, "  Enable Standby Mode Reset 0x20\n");
    fprintf(stderr, "  Disable Soft-Control IWDG 0x40\n");
    goto bail;
  } /* if */

  if (serial_port_dev == NULL) {
    fprintf(stderr, "use -p to specify serial port\n");
    ret = -1;
    goto bail;
  } /* if */

  if (LIBRECH32_ERROR_OK != librech32_init(serial_port_dev)) {
    fprintf(stderr, "failed to init using %s\n", serial_port_dev);
    ret = 1;
    goto bail;
  } /* if */

  /* no operation specified -- just check to see if a ch32 is there */
  if ((NULL == write_file) &&
     (NULL == verify_file)) {

    if (LIBRECH32_ERROR_OK != librech32_detect()) {
      fprintf(stderr, "chip detect failed\n");
      ret = 2;
      goto bail;
    } else if (LIBRECH32_ERROR_OK != librech32_read_config(&config_bits)) {
      fprintf(stderr, "read config failed\n");
      ret = 3;
      goto bail;
    } /* if */

    fprintf(stderr, "CH32F103 detected\n");
    fprintf(stderr, "config bits: 0x%02x\n", config_bits);
  } /* if */

  /* set config bits */
  if (set_config) {

    if (LIBRECH32_ERROR_OK != librech32_detect()) {
      fprintf(stderr, "chip detect failed\n");
      ret = 2;
      goto bail;
    } else if (LIBRECH32_ERROR_OK != librech32_read_config(&config_bits)) {
      fprintf(stderr, "read config failed\n");
      ret = 3;
      goto bail;
    } /* if */

    fprintf(stderr, "old config bits: 0x%02x\n", config_bits);

    if (LIBRECH32_ERROR_OK != librech32_write_config(set_config_bits)) {
      fprintf(stderr, "write config failed\n");
      ret = 3;
      goto bail;
    } else if (LIBRECH32_ERROR_OK != librech32_read_config(&config_bits)) {
      fprintf(stderr, "read config failed\n");
      ret = 3;
      goto bail;
    } /* if */

    fprintf(stderr, "new config bits: 0x%02x\n", config_bits);
  } /* if */

  /* write a file to ch32 */
  if (write_file) {
    if (LIBRECH32_ERROR_OK != librech32_detect()) {
      fprintf(stderr, "chip detect failed\n");
      ret = 2;
      goto bail;
    } else if (LIBRECH32_ERROR_OK != librech32_read_config(NULL)) {
      fprintf(stderr, "read config failed\n");
      ret = 3;
      goto bail;
    } else if (LIBRECH32_ERROR_OK != librech32_setup_key()) {
      fprintf(stderr, "setup key failed\n");
      ret = 3;
    } /* if */

    fp = fopen(write_file, "r");

    if (fp) {
      if (LIBRECH32_ERROR_OK != librech32_erase()) {
        fprintf(stderr, "erase failed\n");
        ret = 4;
        done = true;
      } /* if */

      while (!done) {
        len = fread(buffer, sizeof(char), sizeof(buffer), fp);

        if (len < sizeof(buffer))
          done = true;

        if (len) {
          if (LIBRECH32_ERROR_OK != librech32_write(len, addr, buffer)) {
            fprintf(stderr, "write error at 0x%04x\n", addr);
            ret = 5;
            done = true;
          } /* if */

          addr += len;
        } /* if */
      } /* while */

      if ((0 == ret) &&
         (LIBRECH32_ERROR_OK != librech32_write(0, addr, buffer))) {
        fprintf(stderr, "write error at 0x%04x\n", addr);
        ret = 5;
      } /* if */

      fclose(fp);
    } else {
      fprintf(stderr, "failed to open %s\n", verify_file);
      ret = -2;
    } /* if */
  } /* if */

  /* verify ch32 memory */
  if (verify_file) {
    if (LIBRECH32_ERROR_OK != librech32_detect()) {
      fprintf(stderr, "chip detect failed\n");
      ret = 2;
      goto bail;
    } else if (LIBRECH32_ERROR_OK != librech32_read_config(NULL)) {
      fprintf(stderr, "read config failed\n");
      ret = 3;
      goto bail;
    } else if (LIBRECH32_ERROR_OK != librech32_setup_key()) {
      fprintf(stderr, "setup key failed\n");
      ret = 3;
    } /* if */

    fp = fopen(verify_file, "r");

    if (fp) {
      while (!done) {
        len = fread(buffer, sizeof(char), sizeof(buffer), fp);

        if (len < sizeof(buffer)) {
          done = true;
        } /* if */

        if (len) {
          if (LIBRECH32_ERROR_OK != librech32_verify(len, addr, buffer)) {
            fprintf(stderr, "verify error at 0x%04x\n", addr);
            ret = 6;
            done = true;
          } /* if */
          addr += len;
        } /* if */
      } /* while */

      fclose(fp);
    } else {
      fprintf(stderr, "failed to open %s\n", verify_file);
      ret = -2;
    } /* else */
  } /* if */

  /* reboot before exiting */
  if (reboot) {
    if (LIBRECH32_ERROR_OK != librech32_reboot()) {
      fprintf(stderr, "reboot failed\n");
      ret = 3;
      goto bail;
    } /* if */
  } /* if */

bail:
  librech32_fini();
  return ret;
} /* main */
