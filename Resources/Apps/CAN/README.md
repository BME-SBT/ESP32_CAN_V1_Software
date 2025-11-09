# CAN manager (TWAI) usage guide

## Important disclaimers
- Read this README carefully before reusing the code. It explains the expected usage contract and API.
- Strong rule: you MUST call `can_manager_lock_frame(...)` before reading or modifying any shared `twai_message_t` and call `can_manager_unlock_frame(...)` afterwards. Failing to lock can corrupt data or cause race conditions.
- Register all frames BEFORE calling `can_manager_init(...)` to avoid corrupted frame registry.
- **Only** use pre-defined frames from the list of pre-defined frames below. These are the only standardized frames we are using for this project.
- The `tag` argument to `can_manager_init` controls ESP_LOG output; pass a board or module name string (e.g. "MOTOR_CTRL") or NULL to use the default `CANMAN`.

## What this example does
- Uses a central “CAN manager” that owns the TWAI lifecycle and spawns internal TX/RX tasks.
- You register pointers to your `twai_message_t` frames; the manager:
   - Transmits frames if `tx_enable=true` at a fixed period set at init (default 100ms)
   - Receives all frames and if `rx_enable=true` and ID matches, updates your frame contents.
- Your app can safely modify or access a frame under the manager lock/mutex (lock → edit → unlock).

## Quick start (pattern)
1) Define your frames in your app module (e.g., main.c):
```c
#define MY_FRAME_ID 0x250
static twai_message_t my_frame = {
      .extd = 0,
      .rtr = 0,
      .identifier = MY_FRAME_ID,
      .data_length_code = 8,
      .data = {0}
};
```

2) Register frames BEFORE init and start the manager (fixed TX period applies to all TX-enabled frames). Provide a log TAG string (or NULL to use default "CANMAN"):
```c
ESP_ERROR_CHECK(can_manager_register_frame(&my_frame, /*tx_enable=*/true, /*rx_enable=*/true));
ESP_ERROR_CHECK(can_manager_init(&g_config, &t_config, &f_config, /*tx_period_ms=*/100, "MY_CAN"));
```

3) Modify a frame under lock (always lock/unlock around any access):
```c
can_manager_lock_frame(&my_frame);
my_frame.data[0] = 0xAA;
my_frame.data[1] = 0x55;
can_manager_unlock_frame(&my_frame);
```

4) Check interface health:
```c
bool err = can_manager_error_get();
ESP_LOGI(TAG, "CAN error state: %s", err ? "true" : "false");
```

## API reference 
- `esp_err_t can_manager_register_frame(twai_message_t* frame, bool tx_enable, bool rx_enable)`
   - Register a frame with TX/RX flags. Must be called before `can_manager_init()`.
- `esp_err_t can_manager_init(const twai_general_config_t* g, const twai_timing_config_t* t, const twai_filter_config_t* f, uint8_t tx_period_ms, const char *tag)`
   - Install/start TWAI, spawn TX/RX tasks, and set the fixed TX period for all TX-enabled frames. `tag` is used for ESP_LOG* output; pass NULL for default `"CANMAN"`.
- `esp_err_t can_manager_set_tx_enable(twai_message_t* frame, bool tx_enable)`
   - Enable/disable periodic TX for a registered frame.
- `esp_err_t can_manager_set_rx_enable(twai_message_t* frame, bool rx_enable)`
   - Enable/disable receiving updates into a registered frame.
- `esp_err_t can_manager_lock_frame(twai_message_t* frame)`
   - Lock a frame before reading or modifying it. REQUIRED before any access.
- `esp_err_t can_manager_unlock_frame(twai_message_t* frame)`
   - Unlock a previously locked frame.
- `bool can_manager_error_get(void)`
   - Return the debounced CAN error state (true if recent errors, cleared gradually on success).
- `esp_err_t can_manager_deinit(void)`
   - Stop TWAI and release resources (basic cleanup).

## Manager behavior and policies
- TX task: sends all frames with `tx_enable=true`, then sleeps for `tx_period_ms` (fixed global period set at init).
- RX task: copies incoming messages into matching frames when `rx_enable=true` (match by identifier).
- Thread-safety: each frame has an internal mutex; the manager locks it for RX updates. Your code must lock/unlock for any read/write.
- Registry: auto-grows while you register frames, but registration after init is rejected to avoid runtime races.

## Pre-defined frames 
Below are example project frames to copy as needed; keep them in your app file (not the manager). Define only what you need and register before init. The IDs are already defined in `solar.h` for you.
```c
static twai_message_t motor_temps_frame = {
      .identifier = MotorTemps_ID,
      .extd = 0,
      .rtr = 0,
      .data_length_code = 8,
      .data = {0}
};

static twai_message_t motor_data_frame = {
      .identifier = MotorData_ID,
      .extd = 0,
      .rtr = 0,
      .data_length_code = 8,
      .data = {0}
};

static twai_message_t bms_data_frame = {
      .identifier = BMSData_ID,
      .extd = 0,
      .rtr = 0,
      .data_length_code = 8,
      .data = {0}
};

static twai_message_t bms_extra_frame = {
      .identifier = BMSExtra_ID,
      .extd = 0,
      .rtr = 0,
      .data_length_code = 8,
      .data = {0}
};

static twai_message_t gps_frame = {
      .identifier = GPS_ID,
      .extd = 0,
      .rtr = 0,
      .data_length_code = 8,
      .data = {0}
};

static twai_message_t position_frame = {
      .identifier = Position_ID,
      .extd = 0,
      .rtr = 0,
      .data_length_code = 8,
      .data = {0}
};

static twai_message_t controll_frame = {
      .identifier = Control_ID,
      .extd = 0,
      .rtr = 0,
      .data_length_code = 8,
      .data = {0}
};
```

## Possible issues (caveats for future work)
- Dynamic memory allocation in `can_manager_register_frame` could be unreliable. If issues occur, just use a fixed length array.
- No dynamic registration after init: by design for simplicity. If we need it, we’d add a mutex for the registry.
- No de-registration API and registry memory isn’t freed until deinit: acceptable for embedded use where frames are static. Could be extended.
- Error handling is basic (debounced flag). We might consider adding counters, callbacks, or auto-recovery (e.g., bus-off handling) if needed.
- Removed frame size overwriting in TX task (no longer setting to 8 since all apps are using CAN standard size of 8 byte). If we face issue, we should force set `local.data_length_code = 8` before transmitting in the TX task.
- Task priorities are fixed (currently low). Adjust if system needs stricter timing.
- Memory allocation uses `calloc/realloc` during registration of frames to heap memory. On very tight RAM, a simple pre-registered array might be better.