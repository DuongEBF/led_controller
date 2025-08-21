
# Embedded LED Controller, UART (IRQ) and MAVLink Receiver

This repository contains a set of small, focused C modules for:
- Driving **front/rear gear LEDs** based on the autopilot (MAVLink) state.
- Receiving **UART** bytes via interrupt (IRQ) into lock-free ring buffers.
- A minimal **ring buffer** utility.
- A **MAVLink** byte-stream parser that updates decoded heartbeat and attitude.

The code is written for STM32-class MCUs using HAL/CMSIS, but is portable to other MCUs with minor changes.
All low-level hardware assumptions are isolated to `led_controller.c` and `uart_irq.c`.

> Target audience: embedded developers who are comfortable with HAL/CMSIS and FreeRTOS.

---

## Project Layout

```
led_controller.c      # LED effects driven by MAVLink state and attitude
led_controller.h
mavlink_from_FC.c     # MAVLink byte-stream parsing (heartbeat/attitude)
mavlink_from_FC.h
ring_buff.c           # Simple lock-free ring buffer
ring_buff.h
uart_irq.c            # UART reception via IRQ -> ring buffers; helper APIs
uart_irq.h
```

## External Dependencies

- **STM32 HAL** headers (e.g., `main.h`, GPIO types, `HAL_GetTick`, UART handles).
- **CMSIS-OS / FreeRTOS** (`cmsis_os.h`) if you use the provided LED controller task.
- **MAVLink library** headers generated for your dialect:
  - The code includes `../../MAVLink_Custom/hypmotion_mavlink_custom/mavlink.h` from `mavlink_from_FC.h`. Adjust this include path to match your project.
- Two UART peripherals are assumed:
  - `huart3` → connected to the **Flight Controller** (MAVLink stream).
  - `huart2` → named `tobufi` in code (a secondary input; optional).

> You can change which UART is used by editing `uart_irq.c` (search for `huart2`, `huart3`).

---

## Quick Start

1. **Wire your LEDs** and provide four LED pin macros in `main.h` (or a central pin header):
   ```c
   // Expected pin macros (names used by led_controller.c)
   #define LED_FRONT_GEAR_GREEN_Pin  /* your pin constant */
   #define LED_FRONT_GEAR_RED_Pin    /* your pin constant */
   #define LED_REAR_GEAR_GREEN_Pin   /* your pin constant */
   #define LED_REAR_GEAR_RED_Pin     /* your pin constant */
   ```

2. **Configure UARTs** in CubeMX (or manually) and ensure these handles exist:
   ```c
   extern UART_HandleTypeDef huart2; // optional secondary receiver
   extern UART_HandleTypeDef huart3; // MAVLink from FC
   ```

3. **Add the sources** from this repository to your build system and ensure the include paths cover your MAVLink headers.

4. **Initialize the modules** at startup:
   ```c
   #include "led_controller.h"
   #include "mavlink_from_FC.h"
   #include "uart_irq.h"

   int main(void) {
       // ... HAL_Init(), clock, GPIO init, UART init, RTOS init, etc.

       // Pass the GPIO port that hosts all LED_* pins (same port for all 4 pins).
       led_controller_init(GPIOA /* or the appropriate GPIO_TypeDef* */);

       // If you also want to receive the secondary stream:
       uart_init_tobufi();

       // The MAVLink parser's init is called inside led_controller_init() via mavlink_init(),
       // which starts UART reception on the FC port.
       // ... start scheduler / main loop ...
   }
   ```

5. **Run the LED controller task** (FreeRTOS):
   ```c
   // led_controller.c defines a task entry 'Led_controller_Task(void*)' (not declared in the .h).
   extern void Led_controller_Task(void*);

   const osThreadAttr_t ledTaskAttr = {
       .name = "LedController",
       .priority = osPriorityNormal,
       .stack_size = 1024
   };

   osThreadId_t ledTaskId = osThreadNew(Led_controller_Task, NULL, &ledTaskAttr);
   ```

   The task continuously:
   - Feeds the MAVLink parser with bytes arriving on `huart3` (via IRQ + ring buffer).
   - Updates rear LED patterns as a function of MAVLink `system_status` and tilt (attitude).
   - Keeps front LEDs in their configured constant state.

---

## Configuration Knobs (in `led_controller.c`)

```c
#define DISARM_TILT_DEG         40.0f   // Tilt threshold (deg) to trigger override when DISARMED
#define ATT_VALID_MS            500     // How recent attitude must be to be considered valid

#define UNINIT_HALF_PERIOD_MS   100     // UNINIT: rear LEDs blink together; full period = 2*UNINIT_HALF_PERIOD_MS
#define STANDBY_PERIOD_MS       1550    // STANDBY: short green pulse every STANDBY_PERIOD_MS
#define STANDBY_ON_MS           100     //         pulse duration
#define ACTIVE_HALF_PERIOD_MS   500     // ACTIVE: rear LEDs alternate each ACTIVE_HALF_PERIOD_MS
#define CRITICAL_HALF_PERIOD_MS 300     // CRITICAL+: red blinks with 50% duty; full period = 2*CRITICAL_HALF_PERIOD_MS
```

**Tilt override** (disarmed only): if `|roll|` or `|pitch|` exceeds `DISARM_TILT_DEG` and the last attitude sample is newer than `ATT_VALID_MS`, rear **red** blinks at the CRITICAL rate regardless of base state.

---

## Runtime Behavior (LEDs)

- **Front LEDs**: fixed — green off, red on.
- **Rear LEDs**: pattern depends on MAVLink `heartbeat.system_status` and tilt override:
  - `MAV_STATE_UNINIT`: both rear blink **together** at 50% duty.
  - `MAV_STATE_STANDBY`: short **green** pulse (width `STANDBY_ON_MS`) every `STANDBY_PERIOD_MS`; red off.
  - `MAV_STATE_ACTIVE`: **alternating** rear green/red; dwell `ACTIVE_HALF_PERIOD_MS` each color.
  - `state >= MAV_STATE_CRITICAL`: **red** blinks at 50% duty; green off.
  - **Tilt override** while **disarmed** and attitude fresh: same as CRITICAL red blink.

GPIO writes are coalesced (only applied when a pin’s target state changes) to reduce redundant `HAL_GPIO_WritePin` calls.

---

## API Reference

### `ring_buff.h`

```c
typedef struct
{
    uint8_t* buffer;
    volatile uint16_t head;
    volatile uint16_t tail;
    uint16_t maxlen;
} Ring_buff_t;

void     ring_buffer_init(Ring_buff_t* ring_buff, uint8_t* buff, uint16_t len);
int8_t   ring_buffer_push(Ring_buff_t* ring_buff, uint8_t data);
int8_t   ring_buffer_pop (Ring_buff_t* ring_buff, uint8_t* data);
uint16_t ring_buffer_available(Ring_buff_t* ring_buff);
```

- **ring_buffer_init**: associate a caller-provided byte array `buff` with the structure and reset indices.
- **ring_buffer_push**: push one byte; returns `0` on success or `-1` if the buffer is full.
- **ring_buffer_pop**: pop one byte into `*data`; returns `0` on success or `-1` if empty.
- **ring_buffer_available**: returns the number of bytes currently stored.

**Concurrency notes:** indices are `volatile`. In this project, **push** happens in the UART IRQ and **pop** happens in tasks; this single-producer/single-consumer pattern is safe without locks. Avoid multiple producers or multiple consumers on the same instance.

**Example:**
```c
#define RX_LEN 256
static uint8_t rx_mem[RX_LEN];
static Ring_buff_t rx_rb;

void app_init(void) {
    ring_buffer_init(&rx_rb, rx_mem, RX_LEN);
}

void on_rx_byte(uint8_t b) {
    (void)ring_buffer_push(&rx_rb, b);
}

void app_poll(void) {
    while (ring_buffer_available(&rx_rb)) {
        uint8_t b;
        if (ring_buffer_pop(&rx_rb, &b) == 0) {
            // process b
        }
    }
}
```

---

### `uart_irq.h`

```c
void     uart_init_fc(void);
void     uart_init_tobufi(void);

uint8_t  uart_read_fc(void);
uint8_t  uart_read_tobufi(void);

uint16_t uart_available_fc(void);
uint16_t uart_available_tobufi(void);

void     receive_rx_fc(uint8_t data_rx);
void     receive_rx_tobufi(uint8_t data_rx);
```

- `uart_init_fc` / `uart_init_tobufi`: start IRQ-driven reception on `huart3` / `huart2` and initialize their internal ring buffers (length 512 bytes each by default).
- `uart_available_*`: number of bytes currently buffered for the respective UART.
- `uart_read_*`: pop one byte from the respective buffer. **Precondition:** `uart_available_*() > 0`.
- `receive_rx_*`: helper called by the IRQ to push an incoming byte; you normally **do not** call these manually.

**IRQ wiring:** `uart_irq.c` implements `HAL_UART_RxCpltCallback`. On each byte, it pushes to the appropriate ring buffer and re-arms `HAL_UART_Receive_IT(huart, &byte, 1)`. Make sure your HAL weak callback is not overridden elsewhere.

**Example (manual polling of secondary port):**
```c
uart_init_tobufi();

for (;;) {
    while (uart_available_tobufi()) {
        uint8_t b = uart_read_tobufi();
        // process b
    }
    // ... other work ...
}
```

---

### `mavlink_from_FC.h`

```c
typedef struct
{
    mavlink_message_t   rx_msg;
    mavlink_status_t    status;
    mavlink_heartbeat_t hb;
    mavlink_attitude_t  att;
    uint32_t            att_last_ms;
} mavlink_from_fc_t;

void mavlink_init(void);
void mavlink_process(mavlink_from_fc_t* mav_fc, uint32_t now);
```

- **mavlink_init**: starts UART reception on the FC port by calling `uart_init_fc()`.
- **mavlink_process**: pull all available bytes from `uart_available_fc()` and feed them to `mavlink_parse_char`. When a full message is parsed, selected messages are decoded into:
  - `hb` (`MAVLINK_MSG_ID_HEARTBEAT`) — used for `base_mode` and `system_status`.
  - `att` (`MAVLINK_MSG_ID_ATTITUDE`) — roll/pitch/yaw, timestamped via `att_last_ms = now`.

Call `mavlink_process(&state, HAL_GetTick())` at a high cadence (e.g., every 5–10 ms in a task) or from your main loop.

**Example:**
```c
mavlink_from_fc_t M;

void app_init(void) {
    mavlink_init();
}

void app_task(void* arg) {
    for(;;) {
        mavlink_process(&M, HAL_GetTick());
        // You can inspect M.hb.system_status, M.hb.base_mode, M.att
        osDelay(5);
    }
}
```

---

### `led_controller.h`

```c
void led_controller_init(GPIO_TypeDef* gpio_x);
```

- **led_controller_init**: stores the GPIO **port** pointer used by all four LED pins and calls `mavlink_init()`.
  - The LED pin **numbers** are taken from the macros `LED_FRONT_GEAR_GREEN_Pin`, `LED_FRONT_GEAR_RED_Pin`, `LED_REAR_GEAR_GREEN_Pin`, `LED_REAR_GEAR_RED_Pin` which must be defined elsewhere (commonly `main.h`). All four are assumed to reside on the **same port** you pass here.

The module also defines a task entry that is not part of the header but exported by the C file:
```c
extern void Led_controller_Task(void* parameter);
```
Create a FreeRTOS thread for it as shown in the Quick Start.

---

## Porting Notes

- If your front/rear LEDs are on **different GPIO ports**, split `led_controller_init` (or extend it) to accept separate ports and update `set_pin_if_changed` calls accordingly.
- If your MAVLink stream uses a different UART, replace `huart3` and `MAVLINK_COMM_0` as needed.
- Timing and patterns are centralized via the `*_PERIOD_MS` macros; adjust them to meet your UX requirements.

---

## Testing Tips

- Use a MAVLink sender (e.g., SITL or a small host tool) to emit `HEARTBEAT` and `ATTITUDE` at typical rates (1 Hz heartbeat, 50–100 Hz attitude).
- Verify that **tilt override** triggers only when disarmed and the attitude timestamp is fresher than `ATT_VALID_MS`.
- Confirm the UART IRQ re-arms properly by observing that `uart_available_fc()` increases during sustained input.

---

## Version

Generated on 2025-08-21.

AUTHORSHIP
----------
Author: Duong

LICENSE
-------
GPL (General Public License)
