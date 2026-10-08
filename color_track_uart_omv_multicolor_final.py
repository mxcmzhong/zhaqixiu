# color_track_uart_omv_multicolor_final.py —— 五色气球追踪【只按大小过滤 / 识别红色并躲开】
#
# 【版本说明】相关版本（协议完全一样，下位机不用改）：
#   color_track_uart_omv_multicolor.py       形状体检 + 识别红色(躲开)
#   color_track_uart_omv_multicolor_noRed.py 形状体检 + 不识别红色（改自上面那份）
#   color_track_uart_omv_multicolor_final.py 本文件：只按大小过滤 + 识别红色(躲开)
#
# 现场颜色：红、黄、蓝、绿、紫 共 5 种，只戳 黄/蓝/绿/紫，不戳红。
#
# v3 筛选策略（按需求简化）：不做形状判断，只丢掉"太小"的干扰项。
#   门槛 = 普通气球放在 50cm 处时占的像素数，比它小的色块一律不认。
#   尺寸参考见下面 MIN_PIXELS / MIN_AREA 的注释，换镜头要重新实测。
#
# 帧格式（12 字节定长，小端）：
#   [0]=0xAA  [1]=0x55
#   [2] status/color : 0 = 没找到；1..4 = 黄/蓝/绿/紫
#   [3][4]=cx  [5][6]=cy  [7][8]=w  [9][10]=h   （小端 uint16）
#   [11] checksum = 第 2~10 字节逐字节异或
#
# 脱机运行：IDE 菜单 工具 -> 保存打开的脚本到 OpenMV Cam(作为 main.py)，
#   上电后自动运行，不需要电脑。脱机前把 DEBUG_PRINT 改成 False。

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
    # 红：a 明显为正、b 也为正（偏橙红一侧）
    "red":    [(30, 100, 20, 127, 10, 90)],
    # 黄：b 明显为正，a 接近 0
    "yellow": [(45, 100, -20, 25, 25, 110)],
    # 蓝：b 明显为负，a 接近 0
    "blue":   [(10, 70, -40, 30, -128, -20)],
    # 绿：a 明显为负
    "green":  [(30, 100, -70, -5, -40, 40)],
    # 紫（品红）：a 为正但 b 为负 —— 靠 b 的符号和红色区分开
    "purple": [(20, 90, 10, 70, -95, -5)],
}

TARGETS = ["yellow", "blue", "green", "purple"]   # 要戳的 4 种颜色
AVOID = ["red"]                                   # 不要戳的颜色

# 由上面两张表自动生成检测用的阈值列表，并记下每个下标对应哪个颜色
DETECT = TARGETS + AVOID
THRESHOLDS = []
NAME_BY_CODE = []
for _name in DETECT:
    for _t in COLORS[_name]:
        THRESHOLDS.append(_t)
        NAME_BY_CODE.append(_name)
# 【重要】blob.code() 返回的不是下标，而是位掩码 1 << 下标！
# OpenMV 源码里是：lnk_blob.code = 1 << code;（多个阈值合并时按位或）
# 所以：第 0 条阈值 -> code=1，第 1 条 -> 2，第 2 条 -> 4，第 3 条 -> 8。
# 以前这里把 code 当下标用，绿色(第2条)会算成下标 4 -> IndexError，就是这个报错。
CODE_TO_NAME = {1 << i: n for i, n in enumerate(NAME_BY_CODE)}
TARGET_CODES = {1 << i for i, n in enumerate(NAME_BY_CODE) if n in TARGETS}
AVOID_CODES = {1 << i for i, n in enumerate(NAME_BY_CODE) if n in AVOID}
COLOR_ID = {name: i + 1 for i, name in enumerate(TARGETS)}   # 1=黄 2=蓝 3=绿 4=紫
ID_TO_NAME = {v: k for k, v in COLOR_ID.items()}

# --------------------- 大小门槛：只按"太小"过滤 --------------------------
# 普通气球充气后直径约 25cm。OpenMV Cam H7 默认 2.8mm 镜头 + QVGA(320x240) 下，
# 25cm 的气球在各个距离大约占多少像素：
#     30cm -> 直径约 188px   圆盘约 27600px   外接框约 35200px
#     50cm -> 直径约 113px   圆盘约  9950px   外接框约 12700px
#     80cm -> 直径约  70px   圆盘约  3900px   外接框约  4950px
#    100cm -> 直径约  56px   圆盘约  2500px   外接框约  3200px
#    200cm -> 直径约  28px   圆盘约   620px   外接框约   790px
# 门槛就取在"50cm 的球"和"80cm 的球"中间：
# 50cm 的球稳稳通过（球上有一块白色高光反光匹配不上颜色，实际匹配到的像素
# 只有圆盘的六七成，所以门槛不能贴着圆盘面积取，否则会误伤它自己），
# 80cm 以外的球即使一个像素不漏全匹配上也只有 3900px，照样被丢掉。
# 换镜头 / 换分辨率 / 想让更远的球也能认，跑 measure_balloon.py 实测后照着改。
MIN_PIXELS = 5000       # 色块像素数下限（太小 -> 当干扰丢掉）
MIN_AREA = 6000         # 外接矩形面积下限（双保险）

# 可选的上限：默认 None = 完全不限制，只按"太小"过滤。
# 如果画面里又出现大片背景被误检（之前那种"一大片框"），把它设成 40000 打开。
MAX_PIXELS = None

# 只在气球实际出现的区域里找，能把桌面/地面/墙角整块排除。None = 全画面。
# 例如气球都挂在画面上半部分：ROI = (0, 0, 320, 180)
ROI = None

MERGE = False           # 多色时必须为 False，否则 blob.code() 分不出颜色
SMOOTH = 0.3            # 坐标平滑：越大越跟手越抖，越小越稳越滞后
SEND_INTERVAL_MS = 0    # 发送间隔：0 = 每帧都发
DEBUG_PRINT = True      # 接电脑调试时开着；正式脱机前改成 False
USE_LED = True          # 板载三色灯当状态灯：绿=锁定目标，红=目标丢失
HEADER = b"\xAA\x55"
RED_LAB = COLORS["red"][0]


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

        # ① 找色块。比"50cm 处普通气球"还小的已经在 find_blobs 里被丢掉了，
        #    这里最多再按可选的 MAX_PIXELS 砍一刀上限（默认不开）。
        raw = find_all(img)
        if MAX_PIXELS is None:
            blobs = raw
        else:
            blobs = [b for b in raw if b.pixels() <= MAX_PIXELS]

        # ② 红球区域（要躲开的）
        red_rects = [b.rect() for b in blobs if b.code() in AVOID_CODES]

        cands = []
        for b in blobs:
            if b.code() not in TARGET_CODES:
                continue
            # (a) 中心落在某个红球框里 -> 当成红球，躲开
            if any(center_inside(rr, b.cx(), b.cy()) for rr in red_rects):
                continue
            # (b) 区域平均色落在红范围 -> 当成红球（挡掉阈值串色）
            st = img.get_statistics(roi=b.rect())
            if lab_in(st.l_mean(), st.a_mean(), st.b_mean(), RED_LAB):
                continue
            cands.append(b)

        if cands:
            # ③ 选目标：默认挑最大的（离得最近的那个）
            blob = max(cands, key=lambda x: x.pixels())
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

        # 红球框画成红色（要躲开的目标）
        for rr in red_rects:
            img.draw_rectangle(rr, color=(255, 0, 0), thickness=2)

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

        # 调试输出：看 biggest 的 px 就知道当前门槛卡得合不合适
        if DEBUG_PRINT and frame % 10 == 0:
            print("fps=%.1f color=%s status=%d cx=%d cy=%d w=%d h=%d"
                  % (clock.fps(), ID_TO_NAME.get(status, "-"),
                     status, cx, cy, w, h))
            if raw:
                big = max(raw, key=lambda x: x.pixels())
                print("   blobs=%d red=%d cand=%d  biggest=%s %dpx %dx%d"
                      % (len(raw), len(red_rects), len(cands),
                         CODE_TO_NAME.get(big.code(), "?"), big.pixels(),
                         big.w(), big.h()))
            else:
                print("   blobs=0 red=0 cand=0")
        frame += 1


if __name__ == "__main__":
    main()
