#include <stdint.h>

#ifndef LIBRECH32_H
#define LIBRECH32_H 1

#define LIBRECH32_MAX_BLOCK_SIZE 56

typedef enum LIBRECH32_ERROR_E {
  LIBRECH32_ERROR_OK = 0,
  LIBRECH32_ERROR_UNKNOWN,
  LIBRECH32_ERROR_IO,
  LIBRECH32_ERROR_TIMEOUT,
  LIBRECH32_ERROR_INVALID,
  LIBRECH32_ERROR_NOMEM,
  LIBRECH32_ERROR_FAILED
} librech32_error_t;

#define LIBRECH32_CONFIG_READ_PROTECT       0x01
#define LIBRECH32_CONFIG_SHORT_DELAY        0x02
#define LIBRECH32_CONFIG_USBD_LOW_SPEED     0x04
#define LIBRECH32_CONFIG_USBD_PULLUP        0x08
#define LIBRECH32_CONFIG_STOP_MODE_RESET    0x10
#define LIBRECH32_CONFIG_STANDBY_MODE_RESET 0x20
#define LIBRECH32_CONFIG_HARDCTRL_IWDG      0x40

/*
 * librech32_init
 *
 * open the serial port for communication to the target device
 */
librech32_error_t
librech32_init(
  char* serial_port);

/*
 * librech32_detect
 *
 * check that the target device responds in a reasonable way
 */
librech32_error_t
librech32_detect();

/*
 * librech32_read_config
 *
 * get config data from the target device
 */
librech32_error_t
librech32_read_config(
  uint8_t *config_p);

/*
 * librech32_write_config
 *
 * set config data on the target device
 */
librech32_error_t
librech32_write_config(
  uint8_t config);

/*
 * librech32_setup_key
 *
 * setup write session key with device
 */
librech32_error_t
librech32_setup_key();

/*
 * librech32_erase
 *
 * erase device
 */
librech32_error_t
librech32_erase();

/*
 * librech32_write
 *
 * write a block of data
 */
librech32_error_t
librech32_write(
  uint16_t size,
  uint32_t address,
  uint8_t *data);

/*
 * librech32_verify
 *
 * verify a block of data
 */
librech32_error_t
librech32_verify(
  uint16_t size,
  uint32_t address,
  uint8_t *data);

/*
 * librech32_reboot
 *
 * reboot device
 */
librech32_error_t
librech32_reboot();

/*
 * librech32_fini
 *
 * close the serial port and restore previous serial port settings
 */
void
librech32_fini();

#endif /* LIBRECH32_H */
