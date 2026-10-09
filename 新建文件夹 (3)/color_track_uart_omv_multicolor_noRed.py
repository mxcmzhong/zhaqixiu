
import image
import struct
import time
from machine import UART

# 兼容不同固件：新固件（4.5 起）用 csi，老固件用 sensor
try:
    import csi
    _NEW_API = True
except ImportError:
    import sensor
    _NEW_API = False

# ----------------------------- 可调参数 ------------------------------------
UART_ID = 3            # H7/H7 Plus：UART3 = P4(TX)/P5(RX)
BAUDRATE = 115200

# 颜色表：名字 -> LAB 阈值列表（同一种颜色可给多条，适应不同光照）
# 阈值请用 OpenMV IDE 的"阈值编辑器"对着现场光照标定，下面是估算的起点
COLORS = {
    # 红：暂时不参与识别（见下面 AVOID 的说明），阈值留在这里备用
    "red":    [(30, 100, 20, 127, 10, 90)],
    # 黄：b 明显为正，a 接近 0
    "yellow": [(45, 100, -20, 25, 25, 110)],
    # 蓝：b 明显为负，a 接近 0
    "blue":   [(10, 70, -40, 30, -128, -20)],
    # 绿：a 明显为负
    # 【绿色阈值放宽】原来 L 从 30 起，气球背光那一面（偏暗）会被整片丢掉，
    # 这是"绿色识别不清楚"最常见的原因。OpenMV 官方例程的绿色阈值本来就是
    # (0, 100, -128, -22, -128, 127)，比原来宽得多。这里折中成：
    #   L 0~100  明暗都收（暗面不再丢）
    #   a -110~-15  只收真正偏绿的颜色，避免和黄/蓝打架
    #   b -60~100   蓝气球 b 会更负，靠这个下限和蓝色分开
    "green":  [(0, 100, -110, -15, -60, 100)],
    # 紫（品红）：a 为正但 b 为负 —— 靠 b 的符号和红色区分开
    "purple": [(20, 90, 10, 70, -95, -5)],
}

TARGETS = ["yellow", "blue", "green", "purple"]   # 要戳的 4 种颜色

# 【红色已关闭】不识别红色：只要不把 "red" 放进 DETECT，红色就不在检测阈值
# 列表里，find_blobs 找不到它，红球自然不会被选中。
# 以后想恢复"识别红色并躲开它"，把下面两行换回：
#     AVOID = ["red"]
#     DETECT = TARGETS + AVOID
AVOID = []
DETECT = TARGETS

THRESHOLDS = []
NAME_BY_CODE = []
for _name in DETECT:
    for _t in COLORS[_name]:
        THRESHOLDS.append(_t)
        NAME_BY_CODE.append(_name)
CODE_TO_NAME = {1 << i: n for i, n in enumerate(NAME_BY_CODE)}
TARGET_CODES = {1 << i for i, n in enumerate(NAME_BY_CODE) if n in TARGETS}
AVOID_CODES = {1 << i for i, n in enumerate(NAME_BY_CODE) if n in AVOID}   # 现在恒为空
COLOR_ID = {name: i + 1 for i, name in enumerate(TARGETS)}   # 1=黄 2=蓝 3=绿 4=紫
ID_TO_NAME = {v: k for k, v in COLOR_ID.items()}

# ---- 尺寸/形状体检：把"不像气球的东西"挡在门外 --------------------------
MIN_PIXELS = 200        # 色块像素数下限
MIN_AREA = 200          # 色块面积下限
MAX_AREA_RATIO = 0.15   # 单个色块最多占画面的比例，超过就当背景
MAX_PIXELS = int(320 * 240 * MAX_AREA_RATIO)   # QVGA = 320x240 -> 11520
MIN_DENSITY = 0.45      # pixels/(w*h)；气球接近圆，约 0.6~0.8，太"空"的是背景
ASPECT_MIN = 0.4        # 宽高比下限（扁长条不是气球）
ASPECT_MAX = 2.5        # 宽高比上限

# 只在气球实际出现的区域里找。例如气球都挂在画面上半部分就写 (0, 0, 320, 180)，
# 能把桌面、地面、墙角这些误检源头整块排除掉。None = 全画面
ROI = None

MERGE = False           # 多色时必须为 False，否则 blob.code() 分不出颜色
SMOOTH = 0.3            # 坐标平滑：越大越跟手越抖，越小越稳越滞后
SEND_INTERVAL_MS = 0    # 发送间隔：0 = 每帧都发
DEBUG_PRINT = True      # 接电脑调试时开着；正式脱机前改成 False
USE_LED = True          # 板载三色灯当状态灯：绿=锁定目标，红=目标丢失
HEADER = b"\xAA\x55"
# 【红色已关闭】RED_LAB 只在"识别红色并躲开"时用得到，先停用
# RED_LAB = COLORS["red"][0]


def init_camera():
    if _NEW_API:
        cam = csi.CSI()
        cam.reset()
        cam.pixformat(csi.RGB565)
        cam.framesize(csi.QVGA)
        return lambda: cam.snapshot()

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)
    return lambda: sensor.snapshot()


def xor_checksum(data):
    chk = 0
    for byte in data:
        chk ^= byte
    return chk


def blob_sane(b):
    """像不像一个气球：大小合适、够"实"、不太扁也不太狭长"""
    p = b.pixels()
    if p < MIN_PIXELS or p > MAX_PIXELS:
        return False
    if b.h() == 0:
        return False
    if b.density() < MIN_DENSITY:
        return False
    ratio = b.w() / b.h()
    if ratio < ASPECT_MIN or ratio > ASPECT_MAX:
        return False
    return True


def center_inside(rect, x, y):
    """点 (x, y) 是否落在矩形 rect 内"""
    rx, ry, rw, rh = rect
    return rx <= x < rx + rw and ry <= y < ry + rh


def lab_in(l, a, b, rng):
    return rng[0] <= l <= rng[1] and rng[2] <= a <= rng[3] and rng[4] <= b <= rng[5]


def find_all(img):
    if ROI is None:
        return img.find_blobs(THRESHOLDS,
                              pixels_threshold=MIN_PIXELS,
                              area_threshold=MIN_AREA,
                              merge=MERGE)
    return img.find_blobs(THRESHOLDS,
                          pixels_threshold=MIN_PIXELS,
                          area_threshold=MIN_AREA,
                          merge=MERGE,
                          roi=ROI)


def main():
    snapshot = init_camera()
    uart = UART(UART_ID, baudrate=BAUDRATE)

    # 板载三色灯，脱机时不用电脑也能看出程序在不在工作、有没有找到目标
    led_red = led_green = None
    if USE_LED:
        try:
            from machine import LED
            led_red = LED("LED_RED")
            led_green = LED("LED_GREEN")
        except Exception:
            led_red = led_green = None

    clock = time.clock()
    smooth_x = smooth_y = None
    last_id = 0
    last_send = time.ticks_ms()
    frame = 0

    while True:
        clock.tick()
        img = snapshot()

        status, cx, cy, w, h = 0, 0, 0, 0, 0
        raw = find_all(img)
        blobs = [b for b in raw if blob_sane(b)]

        # ② 【红色已关闭】红球"禁区"不再计算，直接留空。
        #    留成空列表是为了让下面画框、判定的代码即使不删也不会生效；
        #    要恢复红色识别，把下面这行注释换回上面那行即可。
        # red_rects = [b.rect() for b in blobs if b.code() in AVOID_CODES]
        red_rects = []

        cands = []
        for b in blobs:
            if b.code() not in TARGET_CODES:
                continue
            # 【红色已关闭】下面两段"躲红球"的判定一并停用
            # # (a) 中心落在某个红球框里 -> 当成红球，躲开
            # if any(center_inside(rr, b.cx(), b.cy()) for rr in red_rects):
            #     continue
            # # (b) 区域平均色落在红范围 -> 当成红球（挡掉阈值串色）
            # st = img.get_statistics(roi=b.rect())
            # if lab_in(st.l_mean(), st.a_mean(), st.b_mean(), RED_LAB):
            #     continue
            cands.append(b)

        if cands:
            # ③ 选目标：默认挑最大的；想戳最近的改成按 cy 最大（画面下方更近）
            blob = max(cands, key=lambda x: x.pixels())
            # blob = max(cands, key=lambda x: (x.cy(), x.pixels()))
            cx, cy = blob.cx(), blob.cy()
            w, h = blob.w(), blob.h()
            color_name = CODE_TO_NAME.get(blob.code(), "?")
            status = COLOR_ID[color_name]

            # ④ 坐标平滑；换目标（颜色编号变了）时重新起步
            if smooth_x is None or last_id != status or SMOOTH <= 0:
                smooth_x, smooth_y = cx, cy
            else:
                smooth_x = int(smooth_x * (1 - SMOOTH) + cx * SMOOTH)
                smooth_y = int(smooth_y * (1 - SMOOTH) + cy * SMOOTH)
            cx, cy = smooth_x, smooth_y
            last_id = status

            img.draw_rectangle(blob.rect(), color=(0, 255, 0), thickness=2)
            img.draw_cross(cx, cy, color=(255, 255, 0), size=10)
            img.draw_string(2, 2, "%s cx:%d cy:%d" % (color_name, cx, cy),
                            color=(255, 255, 255), scale=1.5)
        else:
            smooth_x = smooth_y = None
            last_id = 0
            img.draw_string(2, 2, "no target", color=(255, 0, 0), scale=1.5)

        # 【红色已关闭】红球框不再画（red_rects 恒为空，这段留着占位）
        # for rr in red_rects:
        #     img.draw_rectangle(rr, color=(255, 0, 0), thickness=2)

        if led_red is not None:
            if status:
                led_green.on()
                led_red.off()
            else:
                led_green.off()
                led_red.on()

        now = time.ticks_ms()
        if SEND_INTERVAL_MS == 0 or time.ticks_diff(now, last_send) >= SEND_INTERVAL_MS:
            payload = struct.pack("<BHHHH", status, cx, cy, w, h)
            uart.write(HEADER + payload + bytes([xor_checksum(payload)]))
            last_send = now

        # 调试输出：raw/kept/dropped/red/cand 这几个数就是排查误检的仪表盘
        if DEBUG_PRINT and frame % 10 == 0:
            print("fps=%.1f color=%s status=%d cx=%d cy=%d w=%d h=%d"
                  % (clock.fps(), ID_TO_NAME.get(status, "-"),
                     status, cx, cy, w, h))
            if raw:
                big = max(raw, key=lambda x: x.pixels())
                note = "  biggest=%s %dpx %dx%d den=%.2f" % (
                    CODE_TO_NAME.get(big.code(), "?"), big.pixels(),
                    big.w(), big.h(), big.density())
            else:
                note = "  biggest=-"
            print("   raw=%d kept=%d dropped=%d red=%d cand=%d%s"
                  % (len(raw), len(blobs), len(raw) - len(blobs),
                     len(red_rects), len(cands), note))
        frame += 1


if __name__ == "__main__":
    main()
