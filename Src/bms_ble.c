#include "bms_ble.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define HSI_VALUE                    8000000UL

typedef struct {
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
} RCC_TypeDef;

typedef struct {
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
} GPIO_TypeDef;

typedef struct {
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t CRCPR;
  volatile uint32_t RXCRCR;
  volatile uint32_t TXCRCR;
  volatile uint32_t I2SCFGR;
  volatile uint32_t I2SPR;
} SPI_TypeDef;

typedef struct {
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile uint32_t CALIB;
} SysTick_TypeDef;

#define RCC_BASE                     0x40021000UL
#define GPIOA_BASE                   0x48000000UL
#define SPI1_BASE                    0x40013000UL
#define SCB_CPACR_ADDR               0xE000ED88UL
#define SYSTICK_BASE                 0xE000E010UL

#define RCC                          ((RCC_TypeDef *)RCC_BASE)
#define GPIOA                        ((GPIO_TypeDef *)GPIOA_BASE)
#define SPI1                         ((SPI_TypeDef *)SPI1_BASE)
#define SysTick                      ((SysTick_TypeDef *)SYSTICK_BASE)
#define SCB_CPACR                    (*(volatile uint32_t *)SCB_CPACR_ADDR)

#define RCC_AHBENR_GPIOAEN           (1UL << 17)
#define RCC_APB2ENR_SYSCFGEN         (1UL << 0)
#define RCC_APB2ENR_SPI1EN           (1UL << 12)

#define SPI_CR1_CPHA                 (1UL << 0)
#define SPI_CR1_MSTR                 (1UL << 2)
#define SPI_CR1_BR_1                 (1UL << 4)
#define SPI_CR1_SPE                  (1UL << 6)
#define SPI_CR1_SSI                  (1UL << 8)
#define SPI_CR1_SSM                  (1UL << 9)

#define SPI_CR2_DS_Pos               8U
#define SPI_CR2_FRXTH                (1UL << 12)

#define SPI_SR_RXNE                  (1UL << 0)
#define SPI_SR_TXE                   (1UL << 1)

#define SYSTICK_CTRL_ENABLE          (1UL << 0)
#define SYSTICK_CTRL_TICKINT         (1UL << 1)
#define SYSTICK_CTRL_CLKSOURCE       (1UL << 2)

#define BLE_CELL_COUNT              13U
#define BLE_CHAR_PAYLOAD_LEN        (BLE_CELL_COUNT * 2U)

#define BLE_SPI_HEADER_SIZE         5U
#define BLE_MAX_FRAME_SIZE          260U
#define BLE_CMD_TIMEOUT_MS          300U
#define BLE_SPI_TIMEOUT_MS          20U

#define BLE_ADVERT_RETRY_MS         1000U
#define BLE_PUBLISH_PERIOD_MS       1000U

#define HCI_COMMAND_PKT             0x01U
#define HCI_EVENT_PKT               0x04U

#define EVT_DISCONN_COMPLETE        0x05U
#define EVT_CMD_COMPLETE            0x0EU
#define EVT_LE_META_EVENT           0x3EU
#define EVT_VENDOR                  0xFFU

#define EVT_LE_CONN_COMPLETE        0x01U

#define EVT_BLUE_GATT_ATTRIBUTE_MODIFIED 0x0C01U

#define OGF_HOST_CTL                0x03U
#define OGF_VENDOR_CMD              0x3FU

#define OCF_RESET                   0x0003U
#define OCF_HAL_WRITE_CONFIG_DATA   0x000CU
#define OCF_GAP_SET_DISCOVERABLE    0x0083U
#define OCF_GAP_INIT                0x008AU
#define OCF_GATT_INIT               0x0101U
#define OCF_GATT_ADD_SERVICE        0x0102U
#define OCF_GATT_ADD_CHAR           0x0104U
#define OCF_GATT_UPDATE_CHAR_VALUE  0x0106U

#define HCI_OPCODE(ogf, ocf)        (uint16_t)((((uint16_t)(ogf)) << 10) | ((uint16_t)(ocf)))

#define CONFIG_DATA_PUBADDR_OFFSET  0x00U

#define GAP_PERIPHERAL_ROLE         0x01U
#define UUID_TYPE_16                0x01U
#define PRIMARY_SERVICE             0x01U

#define CHAR_PROP_READ              0x02U
#define CHAR_PROP_NOTIFY            0x10U

#define ATTR_PERMISSION_NONE        0x00U
#define GATT_NOTIFY_ATTRIBUTE_WRITE 0x01U

#define BLE_STATUS_SUCCESS          0x00U

#define BNRG_WRITE_HEADER           0x0AU
#define BNRG_READ_HEADER            0x0BU

#define AD_TYPE_COMPLETE_LOCAL_NAME 0x09U

#define BLE_SERVICE_UUID            0xA123U
#define BLE_VOLT_CHAR_UUID          0xA124U
#define BLE_TEMP_CHAR_UUID          0xA125U

/* Pins for X-NUCLEO-BNRG2A1 on NUCLEO-F302R8 */
#define BNRG_IRQ_PORT               GPIOA
#define BNRG_IRQ_PIN                0U   /* A0 */
#define BNRG_CS_PORT                GPIOA
#define BNRG_CS_PIN                 1U   /* A1 */
#define BNRG_RST_PORT               GPIOA
#define BNRG_RST_PIN                8U   /* D7 */

static volatile uint32_t s_tick_ms;

static bool s_connected;
static uint16_t s_connection_handle;
static bool s_voltage_notify_enabled;
static bool s_temp_notify_enabled;
static bool s_advertise_pending;

static uint16_t s_gap_service_handle;
static uint16_t s_dev_name_char_handle;
static uint16_t s_custom_service_handle;
static uint16_t s_voltage_char_handle;
static uint16_t s_temp_char_handle;

static uint16_t s_cell_voltage_mv[BLE_CELL_COUNT];
static int16_t s_cell_temp_deci_c[BLE_CELL_COUNT];
static uint16_t s_wave_step;

static uint32_t s_last_publish_ms;
static uint32_t s_last_adv_attempt_ms;

static const uint8_t s_device_name[] = "BMS-A123";

static uint32_t ble_millis(void);
static bool ble_elapsed(uint32_t start_ms, uint32_t timeout_ms);
static void ble_delay_ms(uint32_t delay_ms);

static void ble_gpio_spi_init(void);
static void ble_spi1_init(void);
static void ble_spi1_select(void);
static void ble_spi1_deselect(void);
static bool ble_spi1_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len);
static bool ble_wait_irq_high(uint32_t timeout_ms);
static bool ble_is_irq_high(void);
static void ble_reset_module(void);

static bool bluenrg_spi_write(const uint8_t *data, uint16_t len);
static uint16_t bluenrg_spi_read(uint8_t *buffer, uint16_t max_len);

static int hci_send_cmd_sync(uint16_t opcode, const uint8_t *params, uint8_t params_len,
                             uint8_t *ret_params, uint8_t *ret_len);
static void ble_handle_event_packet(const uint8_t *packet, uint16_t len);
static void ble_drain_events(void);

static int ble_stack_init(void);
static int ble_set_public_addr(const uint8_t addr[6]);
static int ble_gap_set_discoverable(void);
static int ble_gatt_update_char(uint16_t service_handle, uint16_t char_handle,
                                const uint8_t *data, uint8_t len);

static void ble_update_stub_measurements(void);
static void ble_pack_voltage_payload(uint8_t out[BLE_CHAR_PAYLOAD_LEN]);
static void ble_pack_temperature_payload(uint8_t out[BLE_CHAR_PAYLOAD_LEN]);

void SystemInit(void)
{
#if defined(__FPU_PRESENT) && (__FPU_PRESENT == 1U)
  SCB_CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
#endif

  SysTick->LOAD = (HSI_VALUE / 1000U) - 1U;
  SysTick->VAL = 0U;
  SysTick->CTRL = SYSTICK_CTRL_CLKSOURCE | SYSTICK_CTRL_TICKINT | SYSTICK_CTRL_ENABLE;
}

void SysTick_Handler(void)
{
  s_tick_ms++;
}

int bms_ble_init(void)
{
  memset(s_cell_voltage_mv, 0, sizeof(s_cell_voltage_mv));
  memset(s_cell_temp_deci_c, 0, sizeof(s_cell_temp_deci_c));

  s_connected = false;
  s_connection_handle = 0U;
  s_voltage_notify_enabled = false;
  s_temp_notify_enabled = false;
  s_advertise_pending = true;
  s_last_publish_ms = ble_millis();
  s_last_adv_attempt_ms = ble_millis();
  s_wave_step = 0U;

  ble_gpio_spi_init();
  ble_reset_module();
  ble_delay_ms(10U);

  if (ble_stack_init() != 0) {
    return -1;
  }

  if (ble_gap_set_discoverable() == 0) {
    s_advertise_pending = false;
  } else {
    s_advertise_pending = true;
    s_last_adv_attempt_ms = ble_millis();
  }

  return 0;
}

void bms_ble_process(void)
{
  uint32_t now;
  uint8_t voltage_payload[BLE_CHAR_PAYLOAD_LEN];
  uint8_t temp_payload[BLE_CHAR_PAYLOAD_LEN];

  ble_drain_events();
  now = ble_millis();

  if (!s_connected && s_advertise_pending && ble_elapsed(s_last_adv_attempt_ms, BLE_ADVERT_RETRY_MS)) {
    s_last_adv_attempt_ms = now;
    if (ble_gap_set_discoverable() == 0) {
      s_advertise_pending = false;
    }
  }

  if (!ble_elapsed(s_last_publish_ms, BLE_PUBLISH_PERIOD_MS)) {
    return;
  }

  s_last_publish_ms = now;
  ble_update_stub_measurements();
  ble_pack_voltage_payload(voltage_payload);
  ble_pack_temperature_payload(temp_payload);

  if (s_connected && s_voltage_notify_enabled) {
    (void)ble_gatt_update_char(s_custom_service_handle, s_voltage_char_handle,
                               voltage_payload, BLE_CHAR_PAYLOAD_LEN);
  }

  if (s_connected && s_temp_notify_enabled) {
    (void)ble_gatt_update_char(s_custom_service_handle, s_temp_char_handle,
                               temp_payload, BLE_CHAR_PAYLOAD_LEN);
  }
}

static uint32_t ble_millis(void)
{
  return s_tick_ms;
}

static bool ble_elapsed(uint32_t start_ms, uint32_t timeout_ms)
{
  return (ble_millis() - start_ms) >= timeout_ms;
}

static void ble_delay_ms(uint32_t delay_ms)
{
  uint32_t start = ble_millis();
  while (!ble_elapsed(start, delay_ms)) {
  }
}

static void ble_gpio_spi_init(void)
{
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  (void)RCC->AHBENR;
  (void)RCC->APB2ENR;

  /* PA5/PA6/PA7 as SPI1 SCK/MISO/MOSI */
  GPIOA->MODER &= ~((3U << (5U * 2U)) | (3U << (6U * 2U)) | (3U << (7U * 2U)));
  GPIOA->MODER |= (2U << (5U * 2U)) | (2U << (6U * 2U)) | (2U << (7U * 2U));
  GPIOA->AFR[0] &= ~((0xFU << (5U * 4U)) | (0xFU << (6U * 4U)) | (0xFU << (7U * 4U)));
  GPIOA->AFR[0] |= (5U << (5U * 4U)) | (5U << (6U * 4U)) | (5U << (7U * 4U));
  GPIOA->OSPEEDR |= (3U << (5U * 2U)) | (3U << (6U * 2U)) | (3U << (7U * 2U));

  /* PA1 chip select output */
  GPIOA->MODER &= ~(3U << (BNRG_CS_PIN * 2U));
  GPIOA->MODER |= (1U << (BNRG_CS_PIN * 2U));
  GPIOA->OTYPER &= ~(1U << BNRG_CS_PIN);
  GPIOA->BSRR = (1U << BNRG_CS_PIN);

  /* PA8 reset output */
  GPIOA->MODER &= ~(3U << (BNRG_RST_PIN * 2U));
  GPIOA->MODER |= (1U << (BNRG_RST_PIN * 2U));
  GPIOA->OTYPER &= ~(1U << BNRG_RST_PIN);
  GPIOA->BSRR = (1U << BNRG_RST_PIN);

  /* PA0 IRQ input */
  GPIOA->MODER &= ~(3U << (BNRG_IRQ_PIN * 2U));
  GPIOA->PUPDR &= ~(3U << (BNRG_IRQ_PIN * 2U));

  ble_spi1_init();
}

static void ble_spi1_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  (void)RCC->APB2ENR;

  SPI1->CR1 = 0U;
  SPI1->CR2 = 0U;

  /* SPI mode 1, master, software NSS, <=1 MHz when SYSCLK is 8 MHz. */
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1 | SPI_CR1_CPHA;
  SPI1->CR2 = (7U << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
  SPI1->CR1 |= SPI_CR1_SPE;
}

static void ble_spi1_select(void)
{
  BNRG_CS_PORT->BSRR = (1U << (BNRG_CS_PIN + 16U));
}

static void ble_spi1_deselect(void)
{
  BNRG_CS_PORT->BSRR = (1U << BNRG_CS_PIN);
}

static bool ble_spi1_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
  uint16_t i;
  uint8_t tx_byte;
  uint8_t rx_byte;
  uint32_t start;

  for (i = 0U; i < len; i++) {
    tx_byte = (tx != NULL) ? tx[i] : 0x00U;

    start = ble_millis();
    while ((SPI1->SR & SPI_SR_TXE) == 0U) {
      if (ble_elapsed(start, BLE_SPI_TIMEOUT_MS)) {
        return false;
      }
    }

    *(volatile uint8_t *)&SPI1->DR = tx_byte;

    start = ble_millis();
    while ((SPI1->SR & SPI_SR_RXNE) == 0U) {
      if (ble_elapsed(start, BLE_SPI_TIMEOUT_MS)) {
        return false;
      }
    }

    rx_byte = *(volatile uint8_t *)&SPI1->DR;
    if (rx != NULL) {
      rx[i] = rx_byte;
    }
  }

  return true;
}

static bool ble_wait_irq_high(uint32_t timeout_ms)
{
  uint32_t start = ble_millis();
  while (!ble_is_irq_high()) {
    if (ble_elapsed(start, timeout_ms)) {
      return false;
    }
  }
  return true;
}

static bool ble_is_irq_high(void)
{
  return (BNRG_IRQ_PORT->IDR & (1U << BNRG_IRQ_PIN)) != 0U;
}

static void ble_reset_module(void)
{
  ble_spi1_deselect();
  BNRG_RST_PORT->BSRR = (1U << (BNRG_RST_PIN + 16U));
  ble_delay_ms(5U);
  BNRG_RST_PORT->BSRR = (1U << BNRG_RST_PIN);
  ble_delay_ms(5U);
}

static bool bluenrg_spi_write(const uint8_t *data, uint16_t len)
{
  uint8_t hdr_tx[BLE_SPI_HEADER_SIZE] = {BNRG_WRITE_HEADER, 0x00U, 0x00U, 0x00U, 0x00U};
  uint8_t hdr_rx[BLE_SPI_HEADER_SIZE];
  uint16_t writable;
  bool ok;

  if (!ble_wait_irq_high(BLE_CMD_TIMEOUT_MS)) {
    return false;
  }

  ble_spi1_select();

  ok = ble_spi1_transfer(hdr_tx, hdr_rx, BLE_SPI_HEADER_SIZE);
  if (!ok) {
    ble_spi1_deselect();
    return false;
  }

  writable = (uint16_t)hdr_rx[1] | ((uint16_t)hdr_rx[2] << 8);
  if ((hdr_rx[0] != 0x02U) || (writable < len)) {
    ble_spi1_deselect();
    return false;
  }

  ok = ble_spi1_transfer(data, NULL, len);
  ble_spi1_deselect();
  return ok;
}

static uint16_t bluenrg_spi_read(uint8_t *buffer, uint16_t max_len)
{
  uint8_t hdr_tx[BLE_SPI_HEADER_SIZE] = {BNRG_READ_HEADER, 0x00U, 0x00U, 0x00U, 0x00U};
  uint8_t hdr_rx[BLE_SPI_HEADER_SIZE];
  uint16_t available;
  uint16_t to_copy;
  uint16_t i;
  uint8_t rx_byte;

  if (!ble_is_irq_high()) {
    return 0U;
  }

  ble_spi1_select();

  if (!ble_spi1_transfer(hdr_tx, hdr_rx, BLE_SPI_HEADER_SIZE)) {
    ble_spi1_deselect();
    return 0U;
  }

  if (hdr_rx[0] != 0x02U) {
    ble_spi1_deselect();
    return 0U;
  }

  available = (uint16_t)hdr_rx[3] | ((uint16_t)hdr_rx[4] << 8);
  to_copy = (available < max_len) ? available : max_len;

  for (i = 0U; i < available; i++) {
    if (!ble_spi1_transfer(NULL, &rx_byte, 1U)) {
      ble_spi1_deselect();
      return i;
    }
    if (i < to_copy) {
      buffer[i] = rx_byte;
    }
  }

  ble_spi1_deselect();
  return to_copy;
}

static int hci_send_cmd_sync(uint16_t opcode, const uint8_t *params, uint8_t params_len,
                             uint8_t *ret_params, uint8_t *ret_len)
{
  uint8_t cmd[4U + 255U];
  uint16_t cmd_len;
  uint8_t packet[BLE_MAX_FRAME_SIZE];
  uint8_t event_len;
  uint16_t event_opcode;
  uint8_t return_length;
  uint16_t received;
  uint32_t start;

  cmd[0] = HCI_COMMAND_PKT;
  cmd[1] = (uint8_t)(opcode & 0xFFU);
  cmd[2] = (uint8_t)((opcode >> 8) & 0xFFU);
  cmd[3] = params_len;
  if ((params_len > 0U) && (params != NULL)) {
    memcpy(&cmd[4], params, params_len);
  }
  cmd_len = (uint16_t)(4U + params_len);

  if (!bluenrg_spi_write(cmd, cmd_len)) {
    return -1;
  }

  start = ble_millis();
  while (!ble_elapsed(start, BLE_CMD_TIMEOUT_MS)) {
    received = bluenrg_spi_read(packet, sizeof(packet));
    if (received == 0U) {
      continue;
    }

    if ((received >= 6U) && (packet[0] == HCI_EVENT_PKT) && (packet[1] == EVT_CMD_COMPLETE)) {
      event_len = packet[2];
      if (event_len >= 3U) {
        event_opcode = (uint16_t)packet[4] | ((uint16_t)packet[5] << 8);
        if (event_opcode == opcode) {
          return_length = (uint8_t)(event_len - 3U);
          if ((ret_params != NULL) && (return_length > 0U)) {
            memcpy(ret_params, &packet[6], return_length);
          }
          if (ret_len != NULL) {
            *ret_len = return_length;
          }
          return 0;
        }
      }
    }

    ble_handle_event_packet(packet, received);
  }

  return -2;
}

static void ble_handle_event_packet(const uint8_t *packet, uint16_t len)
{
  uint8_t evt;
  uint8_t plen;
  const uint8_t *data;
  uint16_t ecode;
  uint16_t attr_handle;
  uint16_t attr_data_len;

  if ((packet == NULL) || (len < 3U) || (packet[0] != HCI_EVENT_PKT)) {
    return;
  }

  evt = packet[1];
  plen = packet[2];
  if ((uint16_t)plen > (len - 3U)) {
    plen = (uint8_t)(len - 3U);
  }
  data = &packet[3];

  if ((evt == EVT_DISCONN_COMPLETE) && (plen >= 3U)) {
    s_connected = false;
    s_connection_handle = 0U;
    s_voltage_notify_enabled = false;
    s_temp_notify_enabled = false;
    s_advertise_pending = true;
    return;
  }

  if ((evt == EVT_LE_META_EVENT) && (plen >= 19U) && (data[0] == EVT_LE_CONN_COMPLETE)) {
    if (data[1] == 0x00U) {
      s_connected = true;
      s_connection_handle = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
      s_advertise_pending = false;
    }
    return;
  }

  if ((evt == EVT_VENDOR) && (plen >= 2U)) {
    ecode = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    if ((ecode == EVT_BLUE_GATT_ATTRIBUTE_MODIFIED) && (plen >= 10U)) {
      attr_handle = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
      attr_data_len = (uint16_t)data[8] | ((uint16_t)data[9] << 8);
      if ((attr_data_len >= 1U) && (plen >= (uint8_t)(10U + attr_data_len))) {
        if (attr_handle == (uint16_t)(s_voltage_char_handle + 2U)) {
          s_voltage_notify_enabled = ((data[10] & 0x01U) != 0U);
        } else if (attr_handle == (uint16_t)(s_temp_char_handle + 2U)) {
          s_temp_notify_enabled = ((data[10] & 0x01U) != 0U);
        }
      }
    }
  }
}

static void ble_drain_events(void)
{
  uint8_t packet[BLE_MAX_FRAME_SIZE];
  uint16_t len;

  while (ble_is_irq_high()) {
    len = bluenrg_spi_read(packet, sizeof(packet));
    if (len == 0U) {
      break;
    }
    ble_handle_event_packet(packet, len);
  }
}

static int ble_stack_init(void)
{
  static const uint8_t public_addr[6] = {0xDEU, 0xADU, 0xBEU, 0xEFU, 0x30U, 0x21U};

  uint8_t rp[16];
  uint8_t rp_len;
  uint8_t params[40];
  uint8_t params_len;

  int rc;

  /* Standard HCI reset (opcode 0x0C03). */
  rc = hci_send_cmd_sync(HCI_OPCODE(OGF_HOST_CTL, OCF_RESET), NULL, 0U, rp, &rp_len);
  if (rc != 0) {
    return rc;
  }

  ble_drain_events();

  rc = ble_set_public_addr(public_addr);
  if (rc != 0) {
    return rc;
  }

  rc = hci_send_cmd_sync(HCI_OPCODE(OGF_VENDOR_CMD, OCF_GATT_INIT), NULL, 0U, rp, &rp_len);
  if ((rc != 0) || (rp_len < 1U) || (rp[0] != BLE_STATUS_SUCCESS)) {
    return -3;
  }

  params[0] = GAP_PERIPHERAL_ROLE;
  params[1] = 0x00U;
  params[2] = (uint8_t)(sizeof(s_device_name) - 1U);
  rc = hci_send_cmd_sync(HCI_OPCODE(OGF_VENDOR_CMD, OCF_GAP_INIT), params, 3U, rp, &rp_len);
  if ((rc != 0) || (rp_len < 7U) || (rp[0] != BLE_STATUS_SUCCESS)) {
    return -4;
  }
  s_gap_service_handle = (uint16_t)rp[1] | ((uint16_t)rp[2] << 8);
  s_dev_name_char_handle = (uint16_t)rp[3] | ((uint16_t)rp[4] << 8);

  rc = ble_gatt_update_char(s_gap_service_handle, s_dev_name_char_handle,
                            s_device_name, (uint8_t)(sizeof(s_device_name) - 1U));
  if (rc != 0) {
    return rc;
  }

  params_len = 0U;
  params[params_len++] = UUID_TYPE_16;
  params[params_len++] = (uint8_t)(BLE_SERVICE_UUID & 0xFFU);
  params[params_len++] = (uint8_t)((BLE_SERVICE_UUID >> 8) & 0xFFU);
  params[params_len++] = PRIMARY_SERVICE;
  params[params_len++] = 7U; /* 1 service + 2 chars * (declaration + value + cccd). */
  rc = hci_send_cmd_sync(HCI_OPCODE(OGF_VENDOR_CMD, OCF_GATT_ADD_SERVICE), params, params_len, rp, &rp_len);
  if ((rc != 0) || (rp_len < 3U) || (rp[0] != BLE_STATUS_SUCCESS)) {
    return -5;
  }
  s_custom_service_handle = (uint16_t)rp[1] | ((uint16_t)rp[2] << 8);

  params_len = 0U;
  params[params_len++] = (uint8_t)(s_custom_service_handle & 0xFFU);
  params[params_len++] = (uint8_t)((s_custom_service_handle >> 8) & 0xFFU);
  params[params_len++] = UUID_TYPE_16;
  params[params_len++] = (uint8_t)(BLE_VOLT_CHAR_UUID & 0xFFU);
  params[params_len++] = (uint8_t)((BLE_VOLT_CHAR_UUID >> 8) & 0xFFU);
  params[params_len++] = (uint8_t)(BLE_CHAR_PAYLOAD_LEN & 0xFFU);
  params[params_len++] = (uint8_t)((BLE_CHAR_PAYLOAD_LEN >> 8) & 0xFFU);
  params[params_len++] = CHAR_PROP_NOTIFY | CHAR_PROP_READ;
  params[params_len++] = ATTR_PERMISSION_NONE;
  params[params_len++] = GATT_NOTIFY_ATTRIBUTE_WRITE;
  params[params_len++] = 16U;
  params[params_len++] = 0U;
  rc = hci_send_cmd_sync(HCI_OPCODE(OGF_VENDOR_CMD, OCF_GATT_ADD_CHAR), params, params_len, rp, &rp_len);
  if ((rc != 0) || (rp_len < 3U) || (rp[0] != BLE_STATUS_SUCCESS)) {
    return -6;
  }
  s_voltage_char_handle = (uint16_t)rp[1] | ((uint16_t)rp[2] << 8);

  params_len = 0U;
  params[params_len++] = (uint8_t)(s_custom_service_handle & 0xFFU);
  params[params_len++] = (uint8_t)((s_custom_service_handle >> 8) & 0xFFU);
  params[params_len++] = UUID_TYPE_16;
  params[params_len++] = (uint8_t)(BLE_TEMP_CHAR_UUID & 0xFFU);
  params[params_len++] = (uint8_t)((BLE_TEMP_CHAR_UUID >> 8) & 0xFFU);
  params[params_len++] = (uint8_t)(BLE_CHAR_PAYLOAD_LEN & 0xFFU);
  params[params_len++] = (uint8_t)((BLE_CHAR_PAYLOAD_LEN >> 8) & 0xFFU);
  params[params_len++] = CHAR_PROP_NOTIFY | CHAR_PROP_READ;
  params[params_len++] = ATTR_PERMISSION_NONE;
  params[params_len++] = GATT_NOTIFY_ATTRIBUTE_WRITE;
  params[params_len++] = 16U;
  params[params_len++] = 0U;
  rc = hci_send_cmd_sync(HCI_OPCODE(OGF_VENDOR_CMD, OCF_GATT_ADD_CHAR), params, params_len, rp, &rp_len);
  if ((rc != 0) || (rp_len < 3U) || (rp[0] != BLE_STATUS_SUCCESS)) {
    return -7;
  }
  s_temp_char_handle = (uint16_t)rp[1] | ((uint16_t)rp[2] << 8);

  return 0;
}

static int ble_set_public_addr(const uint8_t addr[6])
{
  uint8_t params[8];
  uint8_t rp[8];
  uint8_t rp_len;
  int rc;

  params[0] = CONFIG_DATA_PUBADDR_OFFSET;
  params[1] = 6U;
  memcpy(&params[2], addr, 6U);

  rc = hci_send_cmd_sync(HCI_OPCODE(OGF_VENDOR_CMD, OCF_HAL_WRITE_CONFIG_DATA),
                         params, 8U, rp, &rp_len);
  if ((rc != 0) || (rp_len < 1U) || (rp[0] != BLE_STATUS_SUCCESS)) {
    return -1;
  }

  return 0;
}

static int ble_gap_set_discoverable(void)
{
  uint8_t params[48];
  uint8_t rp[8];
  uint8_t rp_len;
  uint8_t local_name_len;
  uint8_t service_list_len;
  uint8_t i;
  int rc;

  const uint16_t adv_interval = 0x0800U; /* 1.28 s */
  uint8_t local_name[1U + sizeof(s_device_name)];
  uint8_t service_list[2] = {(uint8_t)(BLE_SERVICE_UUID & 0xFFU), (uint8_t)(BLE_SERVICE_UUID >> 8)};

  local_name[0] = AD_TYPE_COMPLETE_LOCAL_NAME;
  memcpy(&local_name[1], s_device_name, sizeof(s_device_name) - 1U);
  local_name_len = sizeof(s_device_name);
  service_list_len = sizeof(service_list);

  i = 0U;
  params[i++] = 0x00U; /* Connectable undirected advertising. */
  params[i++] = (uint8_t)(adv_interval & 0xFFU);
  params[i++] = (uint8_t)(adv_interval >> 8);
  params[i++] = (uint8_t)(adv_interval & 0xFFU);
  params[i++] = (uint8_t)(adv_interval >> 8);
  params[i++] = 0x00U; /* Public address */
  params[i++] = 0x00U; /* No white list */
  params[i++] = local_name_len;
  memcpy(&params[i], local_name, local_name_len);
  i = (uint8_t)(i + local_name_len);
  params[i++] = service_list_len;
  memcpy(&params[i], service_list, service_list_len);
  i = (uint8_t)(i + service_list_len);
  params[i++] = 0x00U; /* Slave_Conn_Interval_Min */
  params[i++] = 0x00U;
  params[i++] = 0x00U; /* Slave_Conn_Interval_Max */
  params[i++] = 0x00U;

  rc = hci_send_cmd_sync(HCI_OPCODE(OGF_VENDOR_CMD, OCF_GAP_SET_DISCOVERABLE),
                         params, i, rp, &rp_len);
  if ((rc != 0) || (rp_len < 1U) || (rp[0] != BLE_STATUS_SUCCESS)) {
    return -1;
  }

  return 0;
}

static int ble_gatt_update_char(uint16_t service_handle, uint16_t char_handle,
                                const uint8_t *data, uint8_t len)
{
  uint8_t params[6U + BLE_CHAR_PAYLOAD_LEN];
  uint8_t rp[8];
  uint8_t rp_len;
  int rc;

  params[0] = (uint8_t)(service_handle & 0xFFU);
  params[1] = (uint8_t)(service_handle >> 8);
  params[2] = (uint8_t)(char_handle & 0xFFU);
  params[3] = (uint8_t)(char_handle >> 8);
  params[4] = 0x00U; /* offset */
  params[5] = len;
  memcpy(&params[6], data, len);

  rc = hci_send_cmd_sync(HCI_OPCODE(OGF_VENDOR_CMD, OCF_GATT_UPDATE_CHAR_VALUE),
                         params, (uint8_t)(len + 6U), rp, &rp_len);
  if ((rc != 0) || (rp_len < 1U) || (rp[0] != BLE_STATUS_SUCCESS)) {
    return -1;
  }

  return 0;
}

static void ble_update_stub_measurements(void)
{
  uint32_t i;
  uint16_t ripple_v;
  uint16_t ripple_t;

  s_wave_step = (uint16_t)(s_wave_step + 3U);

  for (i = 0U; i < BLE_CELL_COUNT; i++) {
    ripple_v = (uint16_t)((s_wave_step + (i * 11U)) % 90U);
    ripple_t = (uint16_t)((s_wave_step + (i * 17U)) % 70U);
    s_cell_voltage_mv[i] = (uint16_t)(3200U + (i * 4U) + ripple_v);
    s_cell_temp_deci_c[i] = (int16_t)(220 + ripple_t);
  }
}

static void ble_pack_voltage_payload(uint8_t out[BLE_CHAR_PAYLOAD_LEN])
{
  uint32_t i;
  uint16_t mv;

  for (i = 0U; i < BLE_CELL_COUNT; i++) {
    mv = s_cell_voltage_mv[i];
    out[(i * 2U)] = (uint8_t)(mv & 0xFFU);
    out[(i * 2U) + 1U] = (uint8_t)(mv >> 8);
  }
}

static void ble_pack_temperature_payload(uint8_t out[BLE_CHAR_PAYLOAD_LEN])
{
  uint32_t i;
  int16_t deci_c;
  uint16_t raw;

  for (i = 0U; i < BLE_CELL_COUNT; i++) {
    deci_c = s_cell_temp_deci_c[i];
    raw = (uint16_t)deci_c;
    out[(i * 2U)] = (uint8_t)(raw & 0xFFU);
    out[(i * 2U) + 1U] = (uint8_t)(raw >> 8);
  }
}
