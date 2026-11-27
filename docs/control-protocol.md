# Native Device Service control protocol

`--service-fd FD` takes ownership of an inherited Unix stream socket. The
owner sends a four-byte big-endian length followed by a UTF-8 JSON request
of at most 128 KiB. Replies use server-side WebSocket framing without an
HTTP handshake. `hello` continues to report protocol 1. New operations
are additive, and an older runtime responds with `ok:false`.

A request has `t:"service"`, a caller-assigned unsigned 32-bit `id`, an
`op`, and an `args` string with ASCII whitespace-delimited arguments. Text
replies echo `t` and `id`, and carry `ok` plus `value` for the operations
below. The owner must demultiplex replies and unsolicited serial, display,
audio and stats messages. Samples come from the live machine at a scheduler
boundary.

| Operation | Arguments | Result |
| --- | --- | --- |
| `checkpoint` | empty | `cycle`, `clock_hz`, `instructions`, `interrupts`, `exceptions`. Divide cycle by clock_hz for guest seconds. These are execution counters, not wall-clock pacing estimates. |
| `state-snapshot` | empty | JSON object keyed by absolute device paths. |
| `state-get` | `PATH [PROPERTY]` | The node or one of its properties, with the same shape and names as the snapshot. |
| `state-set` | `/soc/gpio/PIN input_drive true\|false\|null` | Drive a modeled GPIO input or release the host drive. The resolved input is readable under `level`. |
| `state-set` | `/board/imu sample AX AY AZ GX GY GZ` | Place a sample on a board that implements the six-axis input. Acceleration is in g and angular velocity is in degrees per second. |
| `state-set` | `/board/camera auto-increment true\|false` | Advance or freeze the synthetic RGB565 camera pattern. AtomS3R-CAM and M12 also accept `/machine/m5-lcd-cam` for both reads and writes. |
| `state-set` | `/board/sd present true\|false` | Change physical media presence and return card state. |
| `controller-report` | empty | `checkpoint` plus the existing `soc` and `board` diagnostic report strings. CLI checker counters can be sampled before stop. Report strings retain the existing model-specific diagnostic format. |
| `sd-media` | `insert\|eject` | Reinsert or remove the same launch-time card. Identical requests are idempotent. SPI/SDMMC transactions use the new presence state and the card protocol resets on either transition. The media store and its writes survive. |
| `network-fault` | `link\|radio up\|down` | Cut or restore the attached Ethernet data path or virtual Wi-Fi air link and return the resulting `/soc/wifi` state. Radio loss clears virtual association so firmware must recover it. |
| `flash-snapshot` | empty | Atomically replace the configured Flash image and return its byte count and guest cycle. |
| `flash-stop` | empty | Atomically replace the configured Flash image at the final scheduler boundary and exit the emulator. The success receipt is process exit status 0, with no JSON reply. A rejection returns `ok:false` and leaves the guest running. |
| `state-set` | `/board/imu PROPERTY VALUE`, `/board/magnetometer PROPERTY VALUE` | Set a supported scalar sensor input. Other axes retain their current sample. The corresponding guest data registers apply the chip's normal range and encoding. |
| `state-set` | `/board/pmic PROPERTY VALUE`, `/board/ioe1 PROPERTY VALUE`, `/board/ip5306 PROPERTY VALUE` | Set a supported board input. The PMIC and expander read paths use these values for voltage, ADC, charging or pin status. |
| `state-set` | `/board/sht40 temperature-c\|humidity-pct VALUE` | Set one PaperColor environmental input, leaving the other at its current value. |

Paths describe device roles, with `/soc` for controller state and
`/board` for board-wired devices. Model-specific detail can use a child
path. For example `/board/panel` provides frame count, version and rotation,
while `/board/panel/controller` can expose a display controller's BUSY
deadline or pixel counters. GPIO nodes use decimal pin numbers.
`/board/leds` and `/board/leds/NAME` expose RGB output and update count.
Where implemented, `/board/pmic`, `/board/rtc`, `/board/imu` and
`/board/touch` describe current device state. Raw register arrays start at
address zero and reflect stored register bytes. Host reads do not perform
I2C reads or clear IRQ flags. The AtomS3U snapshot adds `/soc/rmt/0` counters for real channel 0 TX
symbols, completions and errors. Board-specific nodes expose the wired
camera identity, e-paper scan counters, and peripheral state for Device
Service compatibility. Unsupported nodes, properties and writes
return `ok:false`.

`/board/sd` describes prepared media with `present`, `blocks`,
`commands`, `blocks_read` and `blocks_written`. Prepare the image through
`--sd` or `--sd-rw`. Add `--sd-ejected` to prepare the image and begin with an
empty slot before the first guest instruction. The socket does not accept
new filenames. SPI slots also report `accepted_cmd17`, `accepted_cmd18`,
`completed_blocks` and `reset_generation`. A read command is accepted once
its first data block is available. A response block counts only after the
guest clocks its last CRC byte, so raising CS during the response never
counts as delivery. Machine reset and each media-presence change increment
the generation and clear the three observation counters. Guest CMD0
reinitializes the card protocol within the current observation generation.

PaperDIY
drives active-low GPIO48, CoreS3 and StackChan drive the AW9523 socket switch,
and PaperColor routes powered card detect through the PMIC. Native SDMMC
and SPI presentations of a Tab5 card share the same media; the ESP-Hosted
SDIO function has its own independent lifecycle.

For Flash export supply `--service-flash-out PATH` at launch. Use a path
inside the lease directory, distinct from the firmware input and owned by
that lease. The emulator stages bytes in the same directory and atomically
renames the complete image. The control socket carries only the receipt.
Serialize snapshot requests per lease when collecting each file. For final
export send `flash-stop` and keep the socket open until the process exits.
Force-kill or closing the socket is an abrupt termination and has no final
export guarantee. The caller handles retention, hashing and deletion of
each image.

Configure networking when spawning the runtime. Omit both radio options
for an offline lease. Use `--wifi SPEC --net nat` for STA with host TCP/UDP
egress, or `--wifi SPEC --net restricted` to keep guest packets inside the
virtual AP, DHCP, DNS and SNTP model with no host NAT or port forwarding.
`--net none` is the existing spelling of the same isolation.
`--net diagnostic` adds virtual AP/network tracing with the same isolation.
Use `--wifi-sta SPEC` for the guest SoftAP and its virtual station. The
control socket cannot change an isolated session into a NAT session.
`/soc/wifi` reports the mode, fault state, frame counters and available
virtual network counters. The Tab5 ESP-Hosted coprocessor can use the CLI
network modes. Its live fault controls and Wi-Fi state are currently
unavailable through the P4 control socket.

## E-paper readiness

PaperColor publishes a new six-color frame when its ED2208 BUSY interval
ends. The 15-second guest interval is at the lower end of the 15–30 seconds
documented for the display. When replaying an input on PaperMono, wait for
the app's first complete frame. A composed UIFlow2 test can still be inside
its synchronous startup drawing calls when the initial frames are visible.
PaperMono's touch node exposes the active-low INT state and report-read
count. Check the pixel after the redraw completes as well as the touch
report count.

- [PaperColor specifications](https://docs.m5stack.com/en/core/PaperColor)
- [PaperMono specifications](https://docs.m5stack.com/en/core/PaperMono)

## Classic and camera compatibility fixtures

On AtomS3R-CAM and M12, `/board/camera` exposes `auto-increment`,
`frame-seed`, `powered` and the source `frames` count. Without an uploaded
image the RGB565 source starts at a static seed of zero. Enabling
auto-increment selects the advancing pattern even when a raw picture is
loaded. Disabling it returns to that picture or holds the current seed.
The raw synthetic source preserves the guest-selected output dimensions.
Encoded JPEG input continues to use the sensor's existing JPEG path.
The guest must drive GPIO18 low before probing SCCB, as on the board.
[M5Stack camera power and wiring](https://docs.m5stack.com/en/core/AtomS3R-CAM)

Use `--grove tmp105` to provide the classic QEMU test fixture at address
`0x48` for a single launch. Atom Echo routes it through SDA26/SCL32 and
Basic v2.7 through SDA21/SCL22. Either ESP32 controller may select that
pin pair. Basic's IP5306 remains on the shared bus at `0x75`. The TMP105
models the temperature, configuration and two threshold registers with
a fixed zero-degree test input. The option is for compatibility runs,
and the standard board remains available with an empty Grove port.
[TI TMP105 register reference](https://www.ti.com/lit/ds/symlink/tmp105.pdf)

Basic's IP5306 preserves reserved bits in the SYS_CTL0 and CHG_DIG_CTL0
registers. After the stock UIFlow2 initialization, these read `0xb9`
and `0x01`. Its disconnected-cell fixture shows `READ0=0x00` and
`READ4=0xff`. Classic QEMU returns `READ0=0x08` in that setup because
it treats the charger as active even with its battery-absent sentinel.
The IP5306 defines bit 3 as charge in progress.
[Injoinic register manual](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/IIC_IP5306_REG_V1.4_cn.pdf)
