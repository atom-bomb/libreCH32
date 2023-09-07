#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/file.h>

#include "libreCH32.h"
#include "debug_print.h"

#define LIBRECH32_PORT_IO_TIMEOUT 5 

/* commands are:
   CH32_COMMAND_HEADER
   command byte
   le16 data length
   data
   8 bit checksum of command and data
*/
#define CH32_COMMAND_HEADER \
  0x57, 0xab

/* responses are:
  CH32_RESPONSE_HEADER
  command to which this is a response, 00
  le16 data length
  data
  8 bit checksum of command and data
*/
#define CH32_RESPONSE_HEADER \
  0x55, 0xaa

#define CH32_GENERIC_RESPONSE_LENGTH 9

#define CH32_DETECT_PACKET \
  0x57, 0xab, \
  0xa1, \
  0x12, 0x00, \
  0x32, 0x14, 0x4d, 0x43, \
  0x55, 0x20, 0x49, 0x53, 0x50, 0x20, 0x26, \
  0x20, 0x57, 0x43, 0x48, 0x2e, 0x43, 0x4e, \
  0xf1

/* response has two bytes indicating chip model */
#define CH32_DETECT_RESPONSE_LENGTH 9

#define CH32_KEY \
  0x57, 0xab, 0xa3, \
  0x1e, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0xd1 

/* response has checksum of the key */
#define CH32_KEY_RESPONSE_LENGTH 9

#define CH32_READ_CONFIG_PACKET \
  0x57, 0xab, \
  0xa7, \
  0x02, 0x00, \
  0x1f, 0x00, \
  0xc8 

#define CH32_READ_CONFIG_RESPONSE_LENGTH 33

#define CH32_WRITE_CONFIG_PACKET \
  0x57, 0xab, \
  0xa8, \
  0x0e, 0x00, \
  0x07, 0x00, 0xff, 0x5a, 0xff, \
  0xff, 0x00, 0x00, 0x00, 0x00, \
  0xff, 0xff, 0xff, 0xff, \
  0x10

#define CH32_ENABLE_RRP  0x00
#define CH32_DISABLE_RRP 0x5a

#define CH32_DISABLE_LONG_DELAY      0x20
#define CH32_ENABLE_USBD_LOW_SPEED   0x08
#define CH32_ENABLE_STOP_MODE_RST    0x02
#define CH32_ENABLE_USBD_PU          0x10
#define CH32_ENABLE_STANDBY_MODE_RST 0x04
#define CH32_DISABLE_SOFTCTRL_IWDG   0x01

#define CH32_ERASE_PACKET \
  0x57, 0xab,\
  0xa4,\
  0x01, 0x00,\
  0x32,\
  0xd7

#define CH32_WRITE_CMD 0xa5

#define CH32_WRITE_HEADER_LENGTH 10

#define CH32_VERIFY_CMD 0xa6

#define CH32_VERIFY_HEADER_LENGTH 10

#define CH32_REBOOT_PACKET \
  0x57, 0xab,\
  0xa2,\
  0x01, 0x00,\
  0x01,\
  0xa4

#define CH32_MIN_BLOCK_SIZE 8

/* private stuff that nobody else needs to see */
static struct {
  int fd_serial;

  struct termios old_port_settings;
  struct termios new_port_settings;

  uint8_t chip_model;
  uint8_t chip_id[8];

  uint8_t rrp_config;
  uint8_t hw_config;

  uint8_t key[8];

  uint8_t buffer[CH32_WRITE_HEADER_LENGTH + LIBRECH32_MAX_BLOCK_SIZE + 1];
} librech32;

/*
 * librech32_port_read
 *
 * read a response packet from the serial port
 */
static librech32_error_t
librech32_port_read(
  uint8_t *buffer,
  size_t length) {

  uint8_t *read_ptr = buffer;
  size_t remaining = length;
  ssize_t result;

  while (remaining) {
    result = read(librech32.fd_serial, read_ptr, remaining);   

    if (0 == result)
      return LIBRECH32_ERROR_TIMEOUT;
    else if (0 > result)
      return LIBRECH32_ERROR_UNKNOWN;

    read_ptr += result;
    remaining -= result;
  } /* while */

  return LIBRECH32_ERROR_OK;
} /* librech32_port_read */

/*
 * librech32_port_write
 *
 * write a command buffer to the serial port
 */
static librech32_error_t
librech32_port_write(
  const uint8_t *buffer,
  size_t length) {

  const uint8_t *write_ptr = buffer;
  size_t remaining = length;
  ssize_t result;

  while (remaining) {
    result = write(librech32.fd_serial, write_ptr, remaining);   

    if (0 == result)
      return LIBRECH32_ERROR_TIMEOUT;
    else if (0 > result)
      return LIBRECH32_ERROR_UNKNOWN;

    write_ptr += result;
    remaining -= result;
  } /* while */

  return LIBRECH32_ERROR_OK;
} /* librech32_port_write */

/*
 * librech32_checksum
 *
 * sum the given bytes for an 8 bit checksum
 */
static uint8_t
librech32_checksum(
  const uint8_t *data,
  size_t length) {

  uint8_t sum = 0;
  size_t remaining = length;
  const uint8_t *ptr = data;

  while (remaining) {
    sum += *ptr;
    ptr++;
    remaining--;
  } /* while */

  return sum;
} /* librech32_checksum */

/*
 * librech32_encrypt
 *
 * xor the given bytes with the key
 */
static void
librech32_encrypt(
  uint8_t *data,
  size_t length) {

  int index;

  for (index = 0;
       index < length;
       index++) {
    data[index] ^= librech32.key[index % sizeof(librech32.key)];
  } /* for */
} /* librech32_encrypt */

/*
 * librech32_validate_response
 *
 * sanity check a response packet
 */
static librech32_error_t
librech32_validate_response(
  uint8_t command,
  const uint8_t *response,
  size_t response_length) {

  uint8_t checksum = 0;

  if (response_length < 4) {
    ERROR_PRINTF("response too short\n");
    return LIBRECH32_ERROR_INVALID;
  } /* if */

  if ((response[0] != 0x55) ||
     (response[1] != 0xaa)) {
    ERROR_PRINTF("bad response header\n");
    return LIBRECH32_ERROR_INVALID;
  } /* if */

  if (response[2] != command) {
    ERROR_PRINTF("response command mismatch\n");
    return LIBRECH32_ERROR_INVALID;
  } /* if */

  checksum = librech32_checksum(&response[2], response_length - 3);

  if (checksum != response[response_length - 1]) {
    ERROR_PRINTF("response checksum mismatch 0x%02x\n", checksum);
    return LIBRECH32_ERROR_INVALID;
  } /* if */

  return LIBRECH32_ERROR_OK;
} /* librech32_validate_response */

/*
 * librech32_execute_command
 *
 * send a command, receive and validate a generic response packet
 * check for non-OK return code.
 */
static librech32_error_t
librech32_execute_command(
  const uint8_t *command,
  size_t length) {

  uint8_t response[CH32_GENERIC_RESPONSE_LENGTH];
  librech32_error_t err;

  if (LIBRECH32_ERROR_OK == (err = librech32_port_write(
    command, length))) {

    memset(response, 0, sizeof(response));

    if (LIBRECH32_ERROR_OK == (err = librech32_port_read(
      response, sizeof(response)))) {

      err = librech32_validate_response(
        command[2], response, sizeof(response));

      if ((err == LIBRECH32_ERROR_OK) &&
          (response[6] | response[7]))
        err = LIBRECH32_ERROR_FAILED;

      DEBUG_PRINT_HEX(response, sizeof(response));
    } /* if */
  } /* if */

  return err;
} /* librech32_execute_command */

/*
 * librech32_init
 *
 * open the serial port for communication to the target device
 */
librech32_error_t
librech32_init(
  char* serial_port) {

  speed_t port_baud = B115200;
  tcflag_t port_bits = CS8;
  tcflag_t port_parity = 0;
  tcflag_t port_stop = 0;
  struct termios port_settings;

  TRACE_PRINTF("\n");

  librech32.fd_serial = open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY);

  if (librech32.fd_serial < 0) {
    ERROR_PRINTF("Can\'t open %s\n", serial_port);
    return LIBRECH32_ERROR_IO;
  } /* if */

  if (lockf(librech32.fd_serial, F_TLOCK, 0) != 0) {
    ERROR_PRINTF("%s busy\n", serial_port);
    return LIBRECH32_ERROR_IO;
  } /* if */

  fcntl(librech32.fd_serial, F_SETFL, 0);

  tcgetattr(librech32.fd_serial, &librech32.old_port_settings);
  tcgetattr(librech32.fd_serial, &librech32.new_port_settings);

  cfmakeraw(&librech32.new_port_settings);
  librech32.new_port_settings.c_cflag &= ~(CSIZE | CRTSCTS);
  librech32.new_port_settings.c_cflag &= ~(IXON | IXOFF | IXANY | IGNPAR);
  librech32.new_port_settings.c_cflag &= ~(ECHOK | ECHOCTL | ECHOKE);
  librech32.new_port_settings.c_cflag &= ~(OPOST | ONLCR);

  cfsetispeed(&librech32.new_port_settings, port_baud);
  cfsetospeed(&librech32.new_port_settings, port_baud);
  librech32.new_port_settings.c_cflag |= \
    (port_parity | port_bits | port_stop | CLOCAL | CREAD);

  librech32.new_port_settings.c_cc[VMIN] = 0;
  librech32.new_port_settings.c_cc[VTIME] = LIBRECH32_PORT_IO_TIMEOUT;

  if (0 != 
    tcsetattr(librech32.fd_serial, TCSANOW, &librech32.new_port_settings)) {
    ERROR_PRINTF("failed to setup serial port\n");
    return LIBRECH32_ERROR_UNKNOWN;
  } /* if */

  return LIBRECH32_ERROR_OK;
} /* librech32_init */

/*
 * librech32_fini
 *
 * close the serial port and restore previous serial port settings
 */
void
librech32_fini() {
  TRACE_PRINTF("\n");

  if (librech32.fd_serial) {
    tcsetattr(librech32.fd_serial, TCSANOW, &librech32.old_port_settings);
    lockf(librech32.fd_serial, F_ULOCK, 0);
    close(librech32.fd_serial);
  } /* if */
} /* librech32_fini */

/*
 * librech32_detect
 *
 * check that the target device responds in a reasonable way
 */
librech32_error_t
librech32_detect() {

  librech32_error_t err = LIBRECH32_ERROR_UNKNOWN;
  const uint8_t detect_cmd[] = { CH32_DETECT_PACKET };
  uint8_t detect_response[CH32_DETECT_RESPONSE_LENGTH];

  TRACE_PRINTF("\n");

  memset(detect_response, 0, sizeof(detect_response));

  if (LIBRECH32_ERROR_OK == (err = librech32_port_write(
    detect_cmd, sizeof(detect_cmd)))) {

    err = librech32_port_read(detect_response, sizeof(detect_response));

    DEBUG_PRINT_HEX(detect_response, sizeof(detect_response));

    err = librech32_validate_response(
      detect_cmd[2], detect_response, sizeof(detect_response));

    if (err == LIBRECH32_ERROR_OK)
      librech32.chip_model = detect_response[6];
  } /* if */

  if (LIBRECH32_ERROR_OK != err)
    ERROR_PRINTF("err: 0x%02x\n", err);

  return err;
} /* librech32_detect */

/*
 * librech32_read_config
 *
 * get config data from the target device
 */
librech32_error_t
librech32_read_config(
  uint8_t *config_p) {

  librech32_error_t err = LIBRECH32_ERROR_UNKNOWN;
  const uint8_t config_cmd[] = { CH32_READ_CONFIG_PACKET };
  uint8_t config_response[CH32_READ_CONFIG_RESPONSE_LENGTH];

  TRACE_PRINTF("\n");

  memset(config_response, 0, sizeof(config_response));

  if (LIBRECH32_ERROR_OK == (err = librech32_port_write(
    config_cmd, sizeof(config_cmd)))) {

    err = librech32_port_read(config_response, sizeof(config_response));

    err = librech32_validate_response(
      config_cmd[2], config_response, sizeof(config_response));

    if (err == LIBRECH32_ERROR_OK) {
      memcpy(librech32.chip_id, &config_response[24],
        sizeof(librech32.chip_id));

      librech32.rrp_config = config_response[9];
      librech32.hw_config = config_response[11];

      if (config_p) {
        if (librech32.rrp_config == CH32_ENABLE_RRP)
          *config_p = LIBRECH32_CONFIG_READ_PROTECT;
        else
          *config_p = 0;

        if (librech32.hw_config & CH32_DISABLE_SOFTCTRL_IWDG)
          *config_p |= LIBRECH32_CONFIG_HARDCTRL_IWDG;

        if (librech32.hw_config & CH32_ENABLE_USBD_LOW_SPEED)
          *config_p |= LIBRECH32_CONFIG_USBD_LOW_SPEED;

        if (librech32.hw_config & CH32_ENABLE_USBD_PU)
          *config_p |= LIBRECH32_CONFIG_USBD_PULLUP;

        if (librech32.hw_config & CH32_ENABLE_STOP_MODE_RST)
          *config_p |= LIBRECH32_CONFIG_STOP_MODE_RESET;

        if (librech32.hw_config & CH32_ENABLE_STANDBY_MODE_RST)
          *config_p |= LIBRECH32_CONFIG_STANDBY_MODE_RESET;
      } /* if */
    } /* if */

    DEBUG_PRINT_HEX(config_response, sizeof(config_response));
  } /* if */

  if (LIBRECH32_ERROR_OK != err)
    ERROR_PRINTF("err: 0x%02x\n", err);

  return err;
} /* librech32_read_config */

/*
 * librech32_write_config
 *
 * write config data on the target device
 */
librech32_error_t
librech32_write_config(
  uint8_t config) {

  librech32_error_t err = LIBRECH32_ERROR_UNKNOWN;
  uint8_t config_cmd[] = { CH32_WRITE_CONFIG_PACKET };

  TRACE_PRINTF("\n");

  config_cmd[7] = ~librech32.rrp_config;
  config_cmd[8] = librech32.rrp_config;

  if (config & LIBRECH32_CONFIG_READ_PROTECT) {
    config_cmd[7] = ~CH32_ENABLE_RRP;
  } else {
    config_cmd[7] = ~CH32_DISABLE_RRP;
  } /* else */

  config_cmd[9] = ~librech32.hw_config;
  config_cmd[10] = librech32.hw_config;

  if (config & LIBRECH32_CONFIG_HARDCTRL_IWDG)
    config_cmd[9] &= ~CH32_DISABLE_SOFTCTRL_IWDG;
  else
    config_cmd[9] |= CH32_DISABLE_SOFTCTRL_IWDG;

  if (config & LIBRECH32_CONFIG_SHORT_DELAY)
    config_cmd[9] &= ~CH32_DISABLE_LONG_DELAY;
  else
    config_cmd[9] |= CH32_DISABLE_LONG_DELAY;

  if (config & LIBRECH32_CONFIG_USBD_LOW_SPEED)
    config_cmd[9] &= ~CH32_ENABLE_USBD_LOW_SPEED;
  else
    config_cmd[9] |= CH32_ENABLE_USBD_LOW_SPEED;

  if (config & LIBRECH32_CONFIG_USBD_PULLUP)
    config_cmd[9] &= ~CH32_ENABLE_USBD_PU;
  else
    config_cmd[9] |= CH32_ENABLE_USBD_PU;

  if (config & LIBRECH32_CONFIG_STOP_MODE_RESET)
    config_cmd[9] &= ~CH32_ENABLE_STOP_MODE_RST;
  else
    config_cmd[9] |= CH32_ENABLE_STOP_MODE_RST;

  if (config & LIBRECH32_CONFIG_STANDBY_MODE_RESET)
    config_cmd[9] &= ~CH32_ENABLE_STANDBY_MODE_RST;
  else
    config_cmd[9] |= CH32_ENABLE_STANDBY_MODE_RST;

  config_cmd[sizeof(config_cmd)- 1] = librech32_checksum(&config_cmd[2],
    sizeof(config_cmd) - 3);

  err = librech32_execute_command(config_cmd, sizeof(config_cmd));

  if (LIBRECH32_ERROR_OK != err)
    ERROR_PRINTF("err: 0x%02x\n", err);

  return err;
} /* librech32_write_config */

/*
 * librech32_setup_key
 *
 * setup write session key with device
 */
librech32_error_t
librech32_setup_key() {

  librech32_error_t err = LIBRECH32_ERROR_UNKNOWN;
  uint8_t key_cmd[] = { CH32_KEY};
  uint8_t key_response[CH32_KEY_RESPONSE_LENGTH];
  uint8_t zero_key_val = 0;

  TRACE_PRINTF("\n");

  zero_key_val = librech32_checksum(librech32.chip_id,
    sizeof(librech32.chip_id));
  memset(librech32.key, 0, sizeof(librech32.key));
  librech32.key[7] = librech32.chip_model;

  memset(&key_cmd[5], zero_key_val, sizeof(key_cmd) - 5);
  key_cmd[sizeof(key_cmd) - 1] = librech32_checksum(&key_cmd[2],
    sizeof(key_cmd) - 3);

  memset(key_response, 0, sizeof(key_response));

  DEBUG_PRINT_HEX(key_cmd, sizeof(key_cmd));

  if (LIBRECH32_ERROR_OK == (err = librech32_port_write(
    key_cmd, sizeof(key_cmd)))) {

    err = librech32_port_read(key_response, sizeof(key_response));

    err = librech32_validate_response(
      key_cmd[2], key_response, sizeof(key_response));

    DEBUG_PRINT_HEX(key_response, sizeof(key_response));
  } /* if */

  if (LIBRECH32_ERROR_OK != err)
    ERROR_PRINTF("err: 0x%02x\n", err);

  return err;
} /* librech32_setup_key */

/*
 * librech32_erase
 *
 * erase device
 */
librech32_error_t
librech32_erase() {

  librech32_error_t err = LIBRECH32_ERROR_UNKNOWN;
  uint8_t erase_cmd[] = { CH32_ERASE_PACKET };

  TRACE_PRINTF("\n");

  err = librech32_execute_command(erase_cmd, sizeof(erase_cmd));

  if (LIBRECH32_ERROR_OK != err)
    ERROR_PRINTF("err: 0x%02x\n", err);

  return err;
} /* librech32_erase */

/*
 * librech32_write
 *
 * write a block of data
 */
librech32_error_t
librech32_write(
  uint16_t size,
  uint32_t address,
  uint8_t *data) {

  librech32_error_t err = LIBRECH32_ERROR_UNKNOWN;
  uint16_t payload_size = size + 5;
  uint16_t padded_size;
  uint8_t *write_cmd = librech32.buffer;
  size_t  write_cmd_length = 0;

  TRACE_PRINTF("\n");

  if (size > LIBRECH32_MAX_BLOCK_SIZE) {
    err = LIBRECH32_ERROR_NOMEM;
    goto bail;
  } /* if */

  padded_size = size;

  if (size % CH32_MIN_BLOCK_SIZE)
    padded_size += CH32_MIN_BLOCK_SIZE - (size % CH32_MIN_BLOCK_SIZE);

  payload_size = padded_size + 5;

  memset(librech32.buffer, 0, sizeof(librech32.buffer));

  write_cmd_length = CH32_WRITE_HEADER_LENGTH + padded_size + 1;

  write_cmd[0] = 0x57;
  write_cmd[1] = 0xab;
  write_cmd[2] = CH32_WRITE_CMD;
  write_cmd[3] = payload_size & 0xff;
  write_cmd[4] = (payload_size & 0xff00) >> 8;
  write_cmd[5] = address & 0xff;
  write_cmd[6] = (address & 0xff00) >> 8;
  write_cmd[7] = (address & 0xff0000) >> 16;
  write_cmd[8] = (address & 0xff000000) >> 24;
  write_cmd[9] = 0;

  memcpy(&write_cmd[CH32_WRITE_HEADER_LENGTH], data, size);
  librech32_encrypt(&write_cmd[CH32_WRITE_HEADER_LENGTH], padded_size);

  write_cmd[write_cmd_length - 1] = librech32_checksum(&write_cmd[2],
    write_cmd_length - 3);

  DEBUG_PRINT_HEX(write_cmd, write_cmd_length);

  err = librech32_execute_command(write_cmd, write_cmd_length);

bail:
  if (LIBRECH32_ERROR_OK != err)
    ERROR_PRINTF("err: 0x%02x\n", err);

  return err;
} /* librech32_write */

/*
 * librech32_verify
 *
 * verify a block of data
 */
librech32_error_t
librech32_verify(
  uint16_t size,
  uint32_t address,
  uint8_t *data) {

  librech32_error_t err = LIBRECH32_ERROR_UNKNOWN;
  uint16_t payload_size;
  uint16_t padded_size;
  uint8_t *verify_cmd = librech32.buffer;
  size_t  verify_cmd_length;

  TRACE_PRINTF("\n");

  if (size > LIBRECH32_MAX_BLOCK_SIZE) {
    err = LIBRECH32_ERROR_NOMEM;
    goto bail;
  } /* if */

  padded_size = size;

  if (size % CH32_MIN_BLOCK_SIZE)
    padded_size += CH32_MIN_BLOCK_SIZE - (size % CH32_MIN_BLOCK_SIZE);

  payload_size = padded_size + 5;

  memset(librech32.buffer, 0, sizeof(librech32.buffer));

  verify_cmd_length = CH32_VERIFY_HEADER_LENGTH + padded_size + 1;

  verify_cmd[0] = 0x57;
  verify_cmd[1] = 0xab;
  verify_cmd[2] = CH32_VERIFY_CMD;
  verify_cmd[3] = payload_size & 0xff;
  verify_cmd[4] = (payload_size & 0xff00) >> 8;
  verify_cmd[5] = address & 0xff;
  verify_cmd[6] = (address & 0xff00) >> 8;
  verify_cmd[7] = (address & 0xff0000) >> 16;
  verify_cmd[8] = (address & 0xff000000) >> 24;
  verify_cmd[9] = 0x00;

  memcpy(&verify_cmd[CH32_VERIFY_HEADER_LENGTH], data, size);
  librech32_encrypt(&verify_cmd[CH32_VERIFY_HEADER_LENGTH], padded_size);

  verify_cmd[verify_cmd_length - 1] = librech32_checksum(&verify_cmd[2],
    verify_cmd_length - 3);

  DEBUG_PRINT_HEX(verify_cmd, verify_cmd_length);

  err = librech32_execute_command(verify_cmd, verify_cmd_length);

bail:
  if (LIBRECH32_ERROR_OK != err)
    ERROR_PRINTF("err: 0x%02x\n", err);

  return err;
} /* librech32_verify */

/*
 * librech32_reboot
 *
 * reboot device
 */
librech32_error_t
librech32_reboot() {

  librech32_error_t err = LIBRECH32_ERROR_UNKNOWN;
  uint8_t reboot_cmd[] = { CH32_REBOOT_PACKET };
  uint8_t response[CH32_GENERIC_RESPONSE_LENGTH - 1];

  TRACE_PRINTF("\n");

  if (LIBRECH32_ERROR_OK == (err = librech32_port_write(
    reboot_cmd, sizeof(reboot_cmd)))) {

    memset(response, 0, sizeof(response));

    if (LIBRECH32_ERROR_OK == (err = librech32_port_read(
      response, sizeof(response)))) {

      if ((response[0] != 0x55) ||
         (response[1] != 0xaa)) {
        ERROR_PRINTF("bad response header\n");
        err = LIBRECH32_ERROR_INVALID;
      } /* if */

      if (response[2] != reboot_cmd[2]) {
        ERROR_PRINTF("response command mismatch\n");
        err = LIBRECH32_ERROR_INVALID;
      } /* if */

      if ((err == LIBRECH32_ERROR_OK) &&
          (response[6] | response[7]))
        err = LIBRECH32_ERROR_FAILED;

      DEBUG_PRINT_HEX(response, sizeof(response));
    } /* if */
  } /* if */

  if (LIBRECH32_ERROR_OK != err)
    ERROR_PRINTF("err: 0x%02x\n", err);

  return err;
} /* librech32_reboot */
