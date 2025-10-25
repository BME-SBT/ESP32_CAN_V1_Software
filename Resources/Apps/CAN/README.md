*Needs proofreading*


# CAN example — usage & extension guide

This app contains a small TWAI (CAN) example, snippets of which all CAN boards will use. The active code transmits and receives two example frames called ``example_frame1`` and ``example_frame2``. These frame variables are defined and initialized (identifier, DLC, data) in your code and can also be found or derived from `solar.h` where the actual project CAN frames are declared.

This document explains how to use and extend the example, using `example_frame1` as the canonical example.

## Overview
- `example_frame1` and `example_frame2` are `twai_message_t` variables that hold the CAN ID, DLC and payload bytes.
- Each shared frame has a mutex named `m_<frameName>`. For `example_frame1` it is `m_example_frame1`.
- `tx_task` periodically (every 100 ms) copies a frame under its mutex into a local `twai_message_t` and then calls `twai_transmit()` on the local copy (so the mutex is not held during the blocking transmit).
- `rx_task` receives CAN frames and, when the incoming `identifier` matches a known frame ID, updates the shared `example_frame` under the matching mutex (``example_frame = rx_msg;`` — struct copy).
- There is a simple CAN error flag protected by `m_can_error` with helper functions in the code: `can_error_set()`, `can_error_clear_one()`, and `can_error_get()`.

## Where the frames live
- Example frames are declared in `main.c` as:

```c
static twai_message_t example_frame1 = {
    .extd = 0,
    .rtr = 0,
    .identifier = example_frame1_ID, // 0x100 in the example
    .data_length_code = 8,
    .data = {0}
};
```

- `solar.h` contains project frames (MotorTemps, MotorData, BMSData, ...) initialized in a similar way. You can reuse the frames in `solar.h` or define new ones in `main.c` (see "Add a new frame").

## Quick example (how TX uses a shared frame — exact code is already in the project):

```c
// copy under mutex
xSemaphoreTake(m_example_frame1, portMAX_DELAY);
local = example_frame1; // struct copy copies the data[] array too
xSemaphoreGive(m_example_frame1);
// transmit the local copy (mutex not held during transmit)
twai_transmit(&local, pdMS_TO_TICKS(10));
```

## Adding a new frame — checklist
1. Define the frame ID (choose an unused CAN ID):
   ```c
   #define MY_FRAME_ID 0x250
   ```

2. Declare and initialize the shared frame variable (top of file):
   ```c
   static twai_message_t my_frame = {
       .extd = 0,
       .rtr = 0,
       .identifier = MY_FRAME_ID,
       .data_length_code = 8,
       .data = {0}
   };
   ```

3. Declare the mutex for the frame (naming follows the convention used here):
   ```c
   static SemaphoreHandle_t m_my_frame = NULL;
   ```

4. Create the mutex in `app_main()` (after TWAI driver is installed and before tasks that use it are created):
   ```c
   m_my_frame = xSemaphoreCreateMutex();
   if (!m_my_frame) { ESP_LOGE(TAG, "m_my_frame create failed"); return; }
   ```

5. Add TX copy/transmit in `tx_task` (do not hold mutex during twai_transmit):
   ```c
   twai_message_t local;
   xSemaphoreTake(m_my_frame, portMAX_DELAY);
   local = my_frame;
   xSemaphoreGive(m_my_frame);
   local.data_length_code = 8; // enforce sending 8 bytes if desired
   twai_transmit(&local, pdMS_TO_TICKS(10));
   ```

6. Add RX handler in `rx_task` (store received frame into the shared frame under mutex):
   ```c
   case MY_FRAME_ID:
       xSemaphoreTake(m_my_frame, portMAX_DELAY);
       my_frame = rx_msg; // struct copy
       xSemaphoreGive(m_my_frame);
       break;
   ```

7. If a test harness is desired, create a small task that writes into `my_frame` under `m_my_frame` to exercise transmit and receive.

## Remove or modify a frame
- To remove: undo the steps above — remove ID, frame variable and mutex, remove TX transmit copy & RX `case` branch.
- To change ID: update the frame global declaration `#define` and update the `rx_task` `case` branch accordingly.

## Best practices and gotchas
- Always take the frame mutex before reading or writing the shared `twai_message_t`.
- Never call `twai_transmit()` while holding the frame mutex — copy under the mutex and transmit the local copy.
- Keep a consistent lock order if you must lock multiple frame mutexes in one function (e.g., always lock `m_example_frame1` then `m_example_frame2`).
- Use `m_can_error` helper functions to inspect or clear the CAN error state instead of touching the counter directly.

### Example: test randomizer
- The project includes a simple test loop that randomly fills `example_frame1.data[]`. This test exercise can be left in during development or moved into its own task for cleanliness.
