# project1_cpp — Firmware ESP32-S3 điều khiển 6 động cơ khớp robot

Firmware cho **ESP32-S3** (ESP-IDF v6.1, C++26 / `gnu++26`), làm cầu nối giữa máy tính và
**6 động cơ khớp** trên hai bus UART một dây. Máy tính điều khiển qua script
`uart_sender.py` (cổng USB, 921600 baud). Sau này bo **RV1106** sẽ thay máy tính làm bộ điều
khiển ở chế độ thật (real mode) với tần số 100 Hz — phần này **chưa thiết kế**.

```
 PC (uart_sender.py)                ESP32-S3                         Động cơ
 ───────────────────   USB/CP2102   ──────────────────────────────   ─────────────────────
  lệnh dạng khung   ───────────────► pc_link  →  motor_utils          bus 1: UART1 GPIO17
  AA 55 CMD LEN ... ◄─────────────── (dòng chữ: RESULT, ERR, LOG…)    id 1, 2, 3
                       921600 baud            →  motor  →  instruction → UART ─┐
                                                                               ├─ 1 Mbaud, 1 dây
                                                                      bus 2: UART2 GPIO15
                                                                        id 1, 2, 3
```

---

## Mục lục

1. [Phần cứng và kết nối](#1-phần-cứng-và-kết-nối)
2. [Chế độ hoạt động](#2-chế-độ-hoạt-động)
3. [Cấu trúc thư mục](#3-cấu-trúc-thư-mục)
4. [Kiến trúc phân lớp](#4-kiến-trúc-phân-lớp)
5. [Chi tiết từng file](#5-chi-tiết-từng-file)
6. [Giao thức PC → ESP](#6-giao-thức-pc--esp)
7. [Giao thức ESP → động cơ](#7-giao-thức-esp--động-cơ)
8. [Đơn vị và quy đổi](#8-đơn-vị-và-quy-đổi)
9. [uart_sender.py](#9-uart_senderpy)
10. [Build và nạp firmware](#10-build-và-nạp-firmware)
11. [Quy ước code](#11-quy-ước-code)
12. [Lưu ý và vấn đề đã biết](#12-lưu-ý-và-vấn-đề-đã-biết)

---

## 1. Phần cứng và kết nối

| Cổng   | Chân              | Baud      | Dùng cho                                                     |
|--------|-------------------|-----------|--------------------------------------------------------------|
| UART0  | TX 43 / RX 44     | 921600    | Kết nối PC qua CP2102 (dùng chung với console log của ESP)   |
| UART1  | GPIO17 (một dây)  | 1 000 000 | Bus 1: động cơ id 1, 2, 3                                    |
| UART2  | GPIO15 (một dây)  | 1 000 000 | Bus 2: động cơ id 1, 2, 3                                    |
| GPIO38 | LED1              | —         | Sáng liên tục ở config mode, nhấp nháy 1 Hz ở action mode    |
| GPIO37 | LED2              | —         | Như LED1                                                     |

- **Một dây (half-duplex):** chân TX và RX của UART1/UART2 là cùng một chân. ESP chỉ kéo
  đường dây khi đang gửi, gửi xong thì nhả ra (có pull-up) để động cơ trả lời trên chính dây đó.
- **6 khớp** (định nghĩa trong `config.hpp`):

| Bus | id | Kiểu điều khiển | Chiều (direction) | Tỉ số truyền |
|-----|----|-----------------|-------------------|--------------|
| 1   | 1  | Speed (tốc độ)  | −1                | 1.0 (chưa biết) |
| 1   | 2  | Position        | +1                | 7.5          |
| 1   | 3  | Position        | +1                | 9.0          |
| 2   | 1  | Speed (tốc độ)  | +1                | 1.0 (chưa biết) |
| 2   | 2  | Position        | −1                | 7.5          |
| 2   | 3  | Position        | −1                | 9.0          |

---

## 2. Chế độ hoạt động

Ở sim mode (PC điều khiển) có **hai chế độ**, chuyển bằng lệnh `action` / `config`:

| | **Config mode** (khi khởi động) | **Action mode** |
|---|---|---|
| Vòng lặp 100 Hz | Dừng — **không gửi gì** ngoài lệnh PC yêu cầu | **Chỉ đọc phản hồi**: mỗi 10 ms đọc `get_all` của động cơ đang chạy `test_motor*`; không có test thì không gửi gì |
| Lệnh chuyển động | — | Gửi **một lần** khi PC yêu cầu; động cơ giữ lệnh đó, không gửi lặp lại |
| Lệnh chuyển động (`set_cmd_mit`, `set_cmd_pos`, `reset`, `test_motor*`) | Bị từ chối | Chỉ cho động cơ đang bật torque |
| Lệnh cấu hình (`setgains`, `setgainslimit`, `setmode`) | Được | Bị từ chối |
| `settorque`, `torque on/off`, `ping`, `get*` | Được | Được |
| In TX/RX | Mọi khung gửi đi và mọi byte nhận về | TX chỉ khi khung thay đổi, RX chỉ khi lỗi |

**Torque:**

- `settorque <bus> <id> on` — bật một động cơ **tại chỗ**: đọc vị trí hiện tại, lấy vị trí đó làm
  **gốc 0 (offset)** của khớp, rồi ra lệnh giữ ở đó. Động cơ không trả lời thì không được bật.
  Khớp speed được bật với `kp = 0, kd = 36` và vận tốc 0.
- `torque on` — bật tất cả động cơ đang tắt theo cách trên. Nếu một động cơ không trả lời thì
  **tắt hết** (không giữ robot khi có khớp không nhìn thấy).
- `settorque … off` / `torque off` — tắt torque; vòng lặp bỏ qua động cơ đã tắt (vì mỗi lệnh ghi
  vị trí sẽ tự bật lại torque trên động cơ).
- Chuyển từ action về config: kết thúc các test và **tắt torque tất cả**.

Khi khởi động, firmware tắt torque mọi động cơ và đặt mức phản hồi (`0x10 = 1`: chỉ trả lời
PING và READ), rồi đợi `kPcStartDelay` (500 ms) mới nhận lệnh từ PC.

---

## 3. Cấu trúc thư mục

```
project1_cpp/
├── CMakeLists.txt          project ESP-IDF (project1_cpp)
├── sdkconfig               cấu hình ESP-IDF (target esp32s3, FreeRTOS 100 Hz, flash 2 MB)
├── .clangd                 bỏ cờ -f*, -m* của GCC Xtensa cho clangd
├── main/                   toàn bộ mã nguồn firmware (component "main")
│   ├── CMakeLists.txt      danh sách file nguồn và component phụ thuộc
│   ├── main.cpp            app_main: khởi tạo UART, LED, chạy các task
│   ├── config.hpp          chân, UART, thời gian, bảng 6 khớp
│   ├── UART.hpp / .cpp     lớp UART (thường, một dây, RS-485)
│   ├── instruction.hpp/.cpp tầng gói tin của giao thức động cơ
│   ├── motor.hpp / .cpp    hàm cơ bản set/get cho một động cơ
│   ├── motor_utils.hpp/.cpp hàm không cơ bản: khớp, chế độ, vòng 100 Hz, test
│   ├── pc_link.hpp / .cpp  kết nối USB với uart_sender.py
│   ├── little_endian.hpp   đọc/ghi số 16 bit (byte thấp trước)
│   └── MPU9265.hpp / .cpp  driver IMU MPU-9250 qua SPI (chưa dùng trong main)
├── test_motor/             kết quả test do uart_sender.py ghi ra
│   ├── csv/id<id>/         file CSV mỗi lần test
│   └── plot/id<id>/        đồ thị PNG tương ứng
└── build/                  thư mục build (sinh ra bởi idf.py)
```

`uart_sender.py` nằm **ngoài** project (`C:\Users\ngoc\Downloads\uart_sender.py`).

---

## 4. Kiến trúc phân lớp

Mỗi lớp chỉ gọi lớp ngay bên dưới:

```
pc_link        nhận khung từ PC, giải mã, gọi đúng một hàm, trả lời bằng dòng chữ
   │
motor_utils    hàm KHÔNG cơ bản: quy đổi khớp ↔ động cơ, chế độ, torque, vòng 100 Hz, test
   │
motor          hàm CƠ BẢN: set/get một động cơ qua Motor_struct (bản sao các thanh ghi)
   │
instruction    tầng gói tin: FF FF ID LEN INST PARAMS CS, ping/read/write…, khóa bus
   │
UART           cổng phần cứng: gửi, nhận, xử lý một dây
```

**Quy tắc của lớp `motor`:**

- Hàm **set** sửa struct trước, rồi ghi xuống động cơ **từ struct**.
- Hàm **get** đọc động cơ rồi điền **vào struct**.
- Mỗi hàm giữ khóa bus trong suốt một lần trao đổi, nên task nào gọi cũng an toàn.

---

## 5. Chi tiết từng file

### 5.1. `main.cpp`

`app_main()`:
1. Khởi tạo `uart1`, `uart2`, `uart0` và khóa bus (`instruction::init_locks()`).
2. Cấu hình hai LED.
3. `motor_utils::start()` — tạo task vòng lặp động cơ (`motor_loop`, ưu tiên 5).
4. Đợi `kPcStartDelay`, rồi tạo task `pc_rx_task` (ưu tiên 6) đọc lệnh từ PC.

### 5.2. `config.hpp` — cấu hình phần cứng

- **Chân LED**, **hằng thời gian**:

| Hằng số            | Giá trị | Ý nghĩa                                                       |
|--------------------|---------|---------------------------------------------------------------|
| `kControlPeriod`   | 10 ms   | Chu kỳ vòng lặp (100 Hz); phải khớp `CONTROL_HZ` trong Python |
| `kPcStartDelay`    | 500 ms  | Thời gian chờ trước khi nhận lệnh PC                          |
| `kReplyTimeout`    | 20 ms   | Thời gian chờ trả lời của ping và read                        |
| `kReplySettle`     | 20 ms   | Chờ sau một lệnh ghi mà động cơ có thể trả lời                |
| `kPcByteTimeout`   | 50 ms   | Khoảng trống tối đa giữa hai byte trong một khung PC          |
| `kLedToggleCycles` | 25      | 25 chu kỳ × 10 ms = 500 ms → LED nháy 1 Hz                    |
| `kSpeedGains`      | kp 0, kd 36 | Gain của khớp speed khi bật torque                       |

- **`uart0`, `uart1`, `uart2`** (`Uart_class`), **`bus1`, `bus2`** (`instruction::Bus_class`).
- **`Control_enum`** `{ Position, Speed }` — cách dùng một khớp.
- **`Joint_struct`** — một khớp:
  - `motor` — `motor::Motor_struct` (bản sao thanh ghi);
  - `control` — position hay speed;
  - `direction` — +1, hoặc −1 khi lắp ngược;
  - `gear_ratio` — chỉ dùng để chia vận tốc **phản hồi**;
  - `offset` — vị trí động cơ (counts) lúc bật torque.
- **`joints`** — mảng `[2][3]`: `joints[0]` = bus 1, `joints[1]` = bus 2.
- **`find_joint(bus, id)`** — tìm khớp theo số UART và id, không có thì trả `nullptr`.

### 5.3. `UART.hpp` / `UART.cpp` — cổng UART

- **`UartConfig_struct`**: cổng, baud, data/parity/stop bits, chân TX/RX/RTS, kích thước buffer.
- **`Uart_class`**:
  - `init()` — cài driver. `rx_pin == tx_pin` → chế độ **một dây**; có `rts_pin` → RS-485.
  - `write()` — gửi. Trên đường dây dùng chung, hàm đợi gửi xong rồi mới trả về.
  - `read()` — đọc với timeout (hết giờ mà không có dữ liệu vẫn là thành công, `bytes_read = 0`).
  - `flush_input()` — bỏ dữ liệu đã nhận chưa đọc.
- **Xử lý một dây** (quan trọng, xem [mục 12](#12-lưu-ý-và-vấn-đề-đã-biết)):
  - `drive_line()` / `release_line()` — giao chân cho UART TX hoặc nhả ra.
  - `wait_tx_idle()` — **quay vòng** trên bit "TX idle" của phần cứng và nhả dây ngay khi stop bit
    cuối ra xong (~1 µs), thay vì `uart_wait_tx_done` (chờ ngắt, mất hàng chục µs).
  - `drop_echo()` — chân RX nhận lại chính những gì vừa gửi (echo); hàm đọc bỏ **đúng số byte đó**,
    không hơn, để không làm mất phản hồi đến ngay sau.

### 5.4. `instruction.hpp` / `instruction.cpp` — tầng gói tin

- **Mã lệnh**: `kInstPing 0x01`, `kInstRead 0x02`, `kInstWrite 0x03`, `kInstReg 0x04`,
  `kInstAction 0x05`, `kInstReset 0x06`, `kInstReboot 0x64`, `kInstSync 0x83`.
- **`Response_struct`** — một phản hồi đã giải mã: id, length, status, `has_position`, position,
  pre_len, instruction, parameter1, data.
- **`Bus_class`** — các động cơ trên một UART:
  - `ping`, `read`, `write`, `reg_write`, `action`, `reset`, `reboot`, `sync_write`;
  - `lock()` / `unlock()` + **`BusLock_class`** (RAII) — một mutex cho mỗi cổng UART.
- **`send_and_receive`**: xóa buffer nhận → gửi khung → đọc phản hồi.
- **`receive_reply`** — bộ đọc phản hồi chịu lỗi:
  - Tìm `FF`, lấy byte đầu tiên khác `FF` làm ID (chấp nhận mất một `FF` hoặc thừa `FF`).
  - Chấp nhận **hai kiểu phản hồi**: có vị trí (`LEN = M + 7`, theo tài liệu) và không có vị trí
    (`LEN = M + 5`, kiểu của động cơ id 1).
  - ID, Length, CheckSum và các byte nhắc lại lệnh phải khớp; không khớp thì bỏ qua và tìm tiếp
    cho đến hết timeout.
  - Trả `ESP_ERR_TIMEOUT` (không có gì) hoặc `ESP_ERR_INVALID_RESPONSE` (có dữ liệu nhưng sai).
- **Hook gỡ lỗi**:
  - `set_tx_hook` — gọi với mỗi khung vừa gửi;
  - `set_rx_hook` — gọi với các byte nhận được trong lúc chờ phản hồi (tối đa 64 byte) và cờ tìm
    thấy hay không.

### 5.5. `motor.hpp` / `motor.cpp` — hàm cơ bản

- **`reg::`** — địa chỉ thanh ghi (xem [mục 7.3](#73-thanh-ghi-dùng-trong-code)).
- **`status::`** — các cờ lỗi trong byte status.
- **`Mode_enum`** `{ Position = 2, Mit = 3 }`, **`ReplyLevel_enum`** `{ PingOnly, PingAndRead, All }`.
- **`Gains_struct`** `{ kp, kd }` (0.001 N·m/rad, 0.001 N·m·s/rad).
- **`Command_struct`** `{ position, speed, torque }` (counts, RPM, 0.001 N·m).
- **`Motor_struct`** — một động cơ: `bus`, `id`, và bản sao thanh ghi:
  - ghi bởi hàm set: `torque`, `gains`, `command`, `mode`, `limits`;
  - điền bởi hàm get: `status`, `position`, `velocity`, `current`, `voltage`, `temperature`,
    `reg_pending`, `moving`, `eeprom_locked`, `punch`.
- **Hàm set** (một lệnh WRITE, không đọc phản hồi):

| Hàm                      | Thanh ghi          | Ghi chú                                  |
|--------------------------|--------------------|------------------------------------------|
| `set_command`            | 0x1E, 6 byte       | pos vel tff; động cơ tự bật torque       |
| `set_gains`              | 0x1A, 4 byte       | kp kd; torque giữ nguyên                 |
| `set_gains_and_command`  | 0x1A, 10 byte      | kp kd pos vel tff; động cơ tự bật torque |
| `set_torque`             | 0x18, 1 byte       | bật/tắt lực giữ                          |
| `set_mode`               | 0x2D, 1 byte       | position / MIT; mất khi tắt nguồn        |
| `set_gains_limit`        | 0x32, 4 byte       | Kp_max Kd_max — **thực tế là chỉ đọc**   |
| `set_reply_level`        | 0x10, 1 byte       | mức phản hồi                             |

- **Hàm get** (đợi phản hồi tối đa `timeout_ticks`):
  - `ping`, `get_pos_vel` (0x24..0x27), `get_gains` (0x1A..0x1D), `get_mode` (0x2D), `get_torque` (0x18);
  - `get_all` — 30 byte từ 0x18 đến 0x35, điền mọi trường của struct.
  - Mỗi phản hồi cũng cập nhật `status`, và cả `position` nếu phản hồi có vị trí.

### 5.6. `motor_utils.hpp` / `motor_utils.cpp` — hàm không cơ bản

- **Quy đổi khớp ↔ động cơ**: `to_motor`, `to_joint`, `to_joint_feedback` (xem [mục 8](#8-đơn-vị-và-quy-đổi)).
- **Chế độ**: `set_action(bool)`, `in_action()`.
- **Torque**: `set_torque_all(bool)`, `set_torque(bus, id, bool)`.
- **Vòng lặp 100 Hz** (`loop_task`):
  - Khi khởi động: tắt torque tất cả và đặt mức phản hồi.
  - Vòng lặp **chỉ đọc phản hồi**: mọi lệnh được gửi một lần khi PC yêu cầu, động cơ giữ lệnh đó.
  - Ở action mode, mỗi 10 ms: đọc và ghi log các test đang chạy (không có test thì không gửi gì),
    nháy LED.
  - Ở config mode: không gửi gì, LED sáng.
- **Test**: `test_motor(bus, id, gains, command, samples)` và `test_motor_dual(id, …)` (cùng id trên
  hai bus, cùng thời điểm bắt đầu).
  - Mỗi chu kỳ đọc `get_all` của động cơ được test và gửi một dòng `LOG,...`.
  - Mở đầu bằng `LOG_START`, kết thúc bằng `LOG_END` (`reason=done | replaced | torque_off | config`).
- **In TX/RX** cho PC: `echo_tx_frame`, `echo_rx_bytes` (định dạng `TX1: FF FF …`, `RX2: …`).
- **`Lock_class`** — mutex bảo vệ trạng thái (chế độ, test, struct và offset của khớp), tách các
  lần đọc 100 Hz khỏi lệnh của PC.

### 5.7. `pc_link.hpp` / `pc_link.cpp` — kết nối PC

- **`namespace pc`** — mã lệnh PC (xem [mục 6](#6-giao-thức-pc--esp)).
- **`reply(format, …)`** — gửi một dòng chữ kiểu `printf` về PC, có `\n` ở cuối.
- **`PcFrameParser_class`** — ghép từng byte thành khung `AA 55 CMD LEN PAYLOAD CS`, tự đồng bộ lại
  theo header, kiểm tra checksum; payload tối đa 14 byte.
- **`pc_rx_task`** — đọc byte; một khung bị đứt quá `kPcByteTimeout` thì bỏ; khung hợp lệ được
  chuyển cho hàm `handle_*` tương ứng.
- **Các hàm `handle_*`**: kiểm tra độ dài payload, chế độ, bus/id; gọi đúng một hàm của
  `motor_utils::` hoặc `motor::`; trả lời bằng `RESULT` / `ERR` / dòng dữ liệu.

### 5.8. `little_endian.hpp`

`put_u16`, `get_u16`, `get_i16` — số 16 bit dạng 2 byte, **byte thấp trước** (thứ tự của cả
thanh ghi động cơ lẫn khung PC).

### 5.9. `MPU9265.hpp` / `MPU9265.cpp` — IMU (chưa dùng)

Driver IMU họ MPU-9250 (MPU-9250/9255/6500) qua SPI:
- `init()` — khởi tạo SPI, reset, kiểm tra `WHO_AM_I`, lấy mẫu 1 kHz, lọc ~41 Hz.
- `calibrate_gyro()` — lấy trung bình để tính bias gyro (cảm biến phải đứng yên).
- `update()` — đọc một mẫu, chạy bộ lọc **Mahony** để ước lượng hướng trọng lực; trả về vận tốc góc
  (rad/s), hướng trọng lực, gia tốc (g), nhiệt độ.

File được build nhưng **chưa được gọi** trong `main.cpp`.

### 5.10. `main/CMakeLists.txt`

```cmake
idf_component_register(
    SRCS "motor_utils.cpp" "main.cpp" "pc_link.cpp" "instruction.cpp" "motor.cpp" "UART.cpp" "MPU9265.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_uart esp_driver_spi esp_timer
)
```

---

## 6. Giao thức PC → ESP

### 6.1. Khung lệnh

```
AA 55 CMD LEN PAYLOAD... CS          CS = ~(CMD + LEN + PAYLOAD...) & 0xFF
```

- `AA 55` — đánh dấu đầu khung (giúp ESP đồng bộ lại khi mất byte).
- `LEN` — số byte payload (mỗi lệnh có độ dài khác nhau).
- `CS` — checksum: khung sai thì bị bỏ, không bao giờ làm động cơ chuyển động.
- `BUS` là số UART (1 hoặc 2), `ID` là 1, 2 hoặc 3; số 16 bit theo little-endian.

### 6.2. Bảng lệnh

| Mã     | Tên                | Payload                                   | Byte | Chế độ          |
|--------|--------------------|-------------------------------------------|------|-----------------|
| `0x02` | SetSimMode         | `ACTION` (1 action, 0 config)             | 1    | bất kỳ          |
| `0x03` | SetTorqueAll       | `ENABLE`                                  | 1    | bất kỳ          |
| `0x04` | SetCmdMit          | `BUS ID KP KD POS VEL TFF`                | 12   | action          |
| `0x05` | SetCmdPos          | `BUS ID POS VEL TFF` (giữ KP KD)          | 8    | action          |
| `0x06` | TestMotor          | `BUS ID KP KD POS VEL TFF SAMPLES`        | 14   | action          |
| `0x07` | TestMotorDual      | `ID KP KD POS VEL TFF SAMPLES`            | 13   | action          |
| `0x10` | SetGains           | `BUS ID KP KD`                            | 6    | config          |
| `0x11` | SetGainsLimit      | `BUS ID KP_MAX KD_MAX`                    | 6    | config          |
| `0x12` | SetMode            | `BUS ID MODE` (2 position, 3 MIT)         | 3    | config          |
| `0x13` | SetTorque          | `BUS ID ENABLE`                           | 3    | bất kỳ          |
| `0x20` | Ping               | `BUS ID`                                  | 2    | bất kỳ          |
| `0x24` | GetGains           | `BUS ID`                                  | 2    | bất kỳ          |
| `0x25` | GetMode            | `BUS ID`                                  | 2    | bất kỳ          |
| `0x26` | GetPosVel          | `BUS ID`                                  | 2    | bất kỳ          |
| `0x27` | GetTorque          | `BUS ID`                                  | 2    | bất kỳ          |
| `0x28` | GetAll             | `BUS ID` (thanh ghi 0x18..0x35)           | 2    | bất kỳ          |

`KP KD` là `u16`; `POS VEL TFF` là `i16`; `SAMPLES` là `u16` (số chu kỳ 10 ms).

Ví dụ `set_cmd_pos 2 3 30 10 0` (pos 30° = 1365 counts):
```
AA 55 05 08 02 03 55 05 0A 00 00 00 89
      │  │  └──────── payload 8 byte ─────┘ └ CS
      │  └ LEN = 8
      └ CMD = 0x05
```

### 6.3. Phản hồi ESP → PC (dòng chữ)

| Dòng                         | Ý nghĩa                                                          |
|------------------------------|------------------------------------------------------------------|
| `RESULT <lệnh> bus= id=: …`  | Kết quả lệnh một động cơ (`ESP_OK`, `ESP_ERR_TIMEOUT`, …)         |
| `ERR <lệnh>: <lý do>`        | Lệnh bị từ chối                                                  |
| `MODE action` / `MODE config`| Đã chuyển chế độ                                                 |
| `TORQUE on` / `TORQUE off`   | Kết quả `torque on/off`                                          |
| `MOTOR_TORQUE bus= id= on=`  | Kết quả `settorque` / `gettorque`                                |
| `OFFSET bus= id= pos=`       | Gốc 0 mới của một khớp position khi bật torque                   |
| `GAINS …`, `CTRLMODE …`, `POSVEL …` | Kết quả các lệnh get                                      |
| `ALL_CTRL …`, `ALL_STATE …`  | Kết quả `getall`: giá trị đặt và giá trị đo                      |
| `LOG_START …`, `LOG,…`, `LOG_END …` | Log của một lần test                                      |
| `TX1: FF FF …`               | Khung vừa gửi lên bus 1                                          |
| `RX2: …`                     | Các byte nhận được từ bus 2 khi chờ phản hồi                     |

Dòng `LOG`:
```
LOG,bus,id,t_us,kp,kd,cmd_pos,cmd_vel,cmd_tff,pos,spd,cur,volt,temp,status,err
```
Giá trị lệnh là giá trị **khớp** (như PC đặt); giá trị đo là giá trị **thô** của động cơ (PC tự quy đổi).

---

## 7. Giao thức ESP → động cơ

Theo tài liệu *"Robot joint control communication documentation via 1-wire UART V1.0.0"*.
UART half-duplex 1 Mbps, 8N1, little-endian, 16384 counts = 1 vòng trục ra.

### 7.1. Khung lệnh

```
FF FF ID LEN INST PARAMS... CS       LEN = số param + 2
                                     CS  = ~(ID + LEN + INST + PARAMS...) & 0xFF
```

### 7.2. Khung phản hồi

Theo tài liệu (ids 2, 3):
```
FF FF ID LEN STATUS POS_L POS_H PRE_LEN INST PARAM1 DATA... CS      LEN = M + 7
```
Động cơ id 1 (firmware khác, không có vị trí):
```
FF FF ID LEN STATUS PRE_LEN INST PARAM1 DATA... CS                  LEN = M + 5
```
`PRE_LEN`, `INST`, `PARAM1` nhắc lại lệnh vừa gửi; `DATA` chỉ có ở lệnh READ (`M` byte).

**Mức phản hồi (`0x10`)**:
- `0` — chỉ PING;
- `1` — PING và READ (firmware dùng mức này, để lệnh ghi không có phản hồi va chạm trên dây);
- `2` — mọi lệnh (mặc định khi bật nguồn).

### 7.3. Thanh ghi dùng trong code

| Địa chỉ     | Tên                  | Quyền | Ghi chú                                         |
|-------------|----------------------|-------|-------------------------------------------------|
| `0x03`      | ID                   | RW    | mặc định 1                                      |
| `0x05`      | Độ trễ phản hồi      | RW    | × 2 µs, mặc định 250 = 500 µs                   |
| `0x10`      | Mức phản hồi         | RW    | 0 / 1 / 2                                       |
| `0x18`      | Torque enable        | RW    | 0 thả lỏng, 1 giữ lực                           |
| `0x1A..0x1B`| Kp                   | RW    | 0.001 N·m/rad, tối đa Kp_max                    |
| `0x1C..0x1D`| Kd                   | RW    | 0.001 N·m·s/rad, tối đa Kd_max                  |
| `0x1E..0x1F`| Vị trí đích          | RW    | counts, ±2 vòng                                 |
| `0x20..0x21`| Vận tốc đích         | RW    | RPM                                             |
| `0x22..0x23`| Mô-men bù tiến (tff) | RW    | 0.001 N·m                                       |
| `0x24..0x25`| Vị trí hiện tại      | R     | counts                                          |
| `0x26..0x27`| Vận tốc hiện tại     | R     | RPM                                             |
| `0x28..0x29`| Dòng điện            | R     | mA                                              |
| `0x2A`      | Điện áp              | R     | 0.1 V                                           |
| `0x2B`      | Nhiệt độ             | R     | °C                                              |
| `0x2C`      | Cờ lệnh chờ          | R     | REG_WRITE đang chờ ACTION                       |
| `0x2D`      | Chế độ điều khiển    | RW    | 2 position, 3 MIT (mặc định)                    |
| `0x2E`      | Đang di chuyển       | R     | Mode 2                                          |
| `0x2F`      | Khóa EEPROM          | RW    |                                                 |
| `0x30..0x31`| Punch                | RW    | Mode 2                                          |
| `0x32..0x33`| Kp_max               | R     | ghi Kp lớn hơn → bị từ chối, giữ giá trị cũ, cờ 0x08 |
| `0x34..0x35`| Kd_max               | R     | như trên                                        |

**Mô-men MIT**: `τ = Kp·(pos_đích − pos) + Kd·(vel_đích − vel) + tff`.
Ghi khối `0x1A` hoặc `0x1E` có chứa vị trí đích → động cơ **tự bật torque**.

---

## 8. Đơn vị và quy đổi

| Đại lượng       | PC gõ vào        | Gửi xuống động cơ                                                   |
|-----------------|------------------|---------------------------------------------------------------------|
| kp, kd          | N·m/rad, N·m·s/rad | × 1000 (Python)                                                  |
| pos             | độ (deg)         | → counts (Python); `× direction + offset` (ESP, chỉ khớp position)  |
| vel             | RPM của động cơ  | `× direction` (ESP) — **không** nhân tỉ số truyền                   |
| tff             | N·m              | × 1000 (Python); `× direction` (ESP)                                |

**Phản hồi** (để vẽ đồ thị):
- vị trí khớp = `direction × (pos − offset)`;
- vận tốc trục ra = `direction × vel / gear_ratio`;
- dòng điện giữ nguyên dấu (không theo chiều quay).

---

## 9. `uart_sender.py`

Script Python chạy trên PC: gõ lệnh, gửi khung xuống ESP, giải mã và in phản hồi, ghi CSV và vẽ
đồ thị cho các test.

- **Cấu hình đầu file**: `PORT = "COM10"`, `BAUD = 921600` (khớp `uart0`), `CONTROL_HZ = 100`
  (khớp `kControlPeriod`), `GEAR_RATIOS = {1: 1.0, 2: 7.5, 3: 9.0}`, `TEST_DIR`.
- **Mã lệnh** `CMD_*` phải khớp `namespace pc` trong `pc_link.hpp`.

### 9.1. Lệnh

```
  action|config
action mode only, for a motor whose torque is on:
  set_cmd_mit <bus> <id> <kp> <kd> <pos> <vel> <tff>
  set_cmd_pos <bus> <id> <pos> <vel> <tff>
  reset <id>
  test_motor <bus> <id> <kp> <kd> <pos> <vel> <tff> <seconds>
  test_motor_dual <id> <kp> <kd> <pos> <vel> <tff> <seconds>
  test_motor_dual_auto <id> <kp> <kd> <tff> <seconds> <l_pos> <h_pos> <l_vel> <h_vel>
  test_motor_dual_sequential <id> <kp> <kd> <pos> <vel> <tff> <times>
  test_motor_dual_sequential_auto <id> <kp> <kd> <tff> <times> <l_pos> <h_pos> <l_vel> <h_vel>
  stop
config mode only:
  setgains <bus> <id> <kp> <kd>
  setgainslimit <bus> <id> <kp_max> <kd_max>
  setmode <bus> <id> mit|position
either mode:
  torque on|off
  settorque <bus> <id> on|off
  ping / getposvel / getgains / getmode / gettorque / getall <bus> <id>
  help
```

- `reset <id>` — đưa khớp id đó trên cả hai bus về 0 (`set_cmd_pos … 0 RESET_VEL_RPM 0`).
- `test_motor_dual_auto` — quét pos từ `l_pos` đến `h_pos` và vel từ `l_vel` đến `h_vel`, bước
  `AUTO_STEP`; mỗi cặp là một lần test, reset về 0 giữa các lần.
- `test_motor_dual_sequential` — lặp đi lặp lại giữa 0 và `pos` `times` lần.
- `test_motor_dual_sequential_auto` — như trên nhưng quét các cặp (pos, vel), bỏ qua pos = 0 (vel = 0 vẫn chạy: chỉ kp kéo khớp, mỗi lần di chuyển được giả định `ZERO_VEL_MOVE_S` = 3 s).
- `stop` — dừng test tự động: khớp quay về 0 rồi test kết thúc.

### 9.2. Kết quả test

- **CSV**: `test_motor/csv/id<id>/<tên>.csv`, cột:
  `time_s, bus, id, kp, kd, cmd_pos_deg, cmd_vel_rpm, cmd_tff_Nm, pos_deg, pos_counts, vel_rpm,
  vel_motor_rpm, current_mA, voltage_V, temp_C, status, error`.
- **Đồ thị**: `test_motor/plot/id<id>/<tên>.png`, 3 khung: vị trí (deg), vận tốc (RPM trục ra),
  dòng điện (mA); mỗi bus một đường, kèm đường lệnh.
- **Tên file**: `<lệnh>_<bus>_<id>_<kp>_<kd>_<pos>_<vel>_<tff>`, bus là `1+2` với test dual.
  Ví dụ: `test_motor_dual_sequential_auto_1+2_3_30_1_20_20_0`. Tên trùng thì thêm `_2`, `_3`, …

---

## 10. Build và nạp firmware

```bash
idf.py set-target esp32s3      # chỉ lần đầu
idf.py build
idf.py -p COMx flash
python uart_sender.py          # sau khi nạp, chạy script trên PC
```

- Log khởi động của ESP ở 115200 baud; sau `init()` UART0 chạy 921600 — nên các dòng đầu tiên trong
  script có thể là ký tự rác.
- Mỗi lần sửa code phải **build lại và nạp lại**; nếu script báo
  `no reply from the ESP` thì kiểm tra firmware có khớp script không.

---

## 11. Quy ước code

- **Hậu tố kiểu**: `_struct`, `_class`, `_enum`, `_alias` (ví dụ `Motor_struct`, `Bus_class`,
  `Mode_enum`, `TxHook_alias`).
- **Namespace**: `instruction::`, `motor::` (`motor::reg::`, `motor::status::`), `motor_utils::`, `pc::`.
- **Hằng số**: tiền tố `k` (`kControlPeriod`, `kInstPing`).
- **Hàm trả về**:
  - lớp `motor` / `instruction` trả `esp_err_t`;
  - lớp `motor_utils` trả `const char *`: `nullptr` là thành công, khác thì là lý do từ chối.
- **Phân chia**: `motor.*` chỉ chứa hàm cơ bản set/get; mọi thứ khác (quy đổi, chế độ, vòng lặp,
  test) nằm ở `motor_utils.*`; `config.hpp` chứa các khớp.

---

## 12. Lưu ý và vấn đề đã biết

- **Động cơ id 1 trả lời ngay lập tức** (không có độ trễ 500 µs như tài liệu) và **không gửi vị trí**
  trong phản hồi (`LEN = M + 5`). Có thể là firmware cũ hơn. Firmware xử lý được nhờ:
  - nhả dây ngay khi gửi xong (`wait_tx_idle`);
  - bỏ echo theo số byte (`drop_echo`) thay vì xóa buffer;
  - bộ đọc phản hồi chấp nhận thiếu một `FF` và cả hai kiểu phản hồi.

  Cách sửa triệt để: đặt độ trễ phản hồi (`0x05`) của id 1 về 250, hoặc dùng chân open-drain với
  điện trở kéo lên 1–4.7 kΩ. Cần hỏi lại về firmware của động cơ id 1.
- **`setgains` không đổi giá trị**: nếu kp > Kp_max hoặc kd > Kd_max, động cơ từ chối và giữ giá trị
  cũ; vì mức phản hồi là 1 nên lỗi (cờ 0x08) không được báo về. Kp_max/Kd_max là **chỉ đọc**, nên
  `setgainslimit` không có tác dụng trên phần cứng.
- **`torque on` cần đủ 6 động cơ trả lời**; khi chỉ nối một phần động cơ, dùng `settorque` cho từng
  động cơ.
- **Vận tốc lệnh** là RPM của động cơ (trước hộp số), còn vận tốc trên đồ thị là RPM trục ra.
- **Chế độ thật với RV1106**: `rv_link.hpp/.cpp` (giao thức HCR1, lệnh 0x01–0x05), bật bằng
  `kUseRvLink = true` trong `config.hpp`. Mọi giá trị là count / thanh ghi tốc độ, xem `rv_link.hpp`.
