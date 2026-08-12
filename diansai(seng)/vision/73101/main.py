from maix import camera, display, image, nn, app, time, sys, uart, touchscreen
import struct

from ball_position import (
    AdaptiveAlphaBetaFilter,
    axis_point,
    project_onto_axis,
)

# 输入打印. 关掉后提升帧率
DEBUG_LOG=False

def print_debug(s):
    if DEBUG_LOG:
        print(s)

# 位置传感器更重视"采集到结果"的延迟，而不是最高吞吐帧率。
# 低延迟模式下 dual_buff=False，每次检测结果都对应本次传入的图像。
# 只有在测试最高检测帧率时，才建议把该开关改为 False。
LOW_LATENCY_MODE = True

# 使能畸变校准
LENS_CORR_ENABLE=False

# 畸变校准强度
LENS_CORR_STRENGTH=0.6

DETECTION_CONFIDENCE = 0.50


MAIXCAM2_MODEL_PATH = "model_312644.mud"

def model_path_for_device(device_name):
    """Return the checked-in model matching a Maix device name."""
    normalized_name = device_name.strip().lower()
    if normalized_name == "maixcam2":
        return MAIXCAM2_MODEL_PATH
    raise ValueError("unsupported device: {}".format(device_name))


model = model_path_for_device(sys.device_name())

detector = nn.YOLOv5(
    model=model,
    dual_buff=not LOW_LATENCY_MODE,
)

AXIS_START_PX = (5, detector.input_height() // 2)
AXIS_END_PX = (detector.input_width() - 5, detector.input_height() // 2)

cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
disp = display.Display()

UART_DEV = "/dev/ttyS4"
serial = None
try:
    serial = uart.UART(UART_DEV, 115200)
    print(f"UART4 已连接: {UART_DEV}")
except Exception as e:
    print(f"警告: 无法打开 {UART_DEV}, 错误: {e}")

position_filter = AdaptiveAlphaBetaFilter()

# 触摸屏: 目标中心点在左边(-100)时, 点一下屏幕切换回右边(+100)
ts = None
try:
    ts = touchscreen.TouchScreen()
except Exception as e:
    print(f"警告: 无法初始化触摸屏, 错误: {e}")
touch_last = False
if ts is not None:
    if ts.available():
        _, _, touch_last = ts.read()

print(detector.input_width(), detector.input_height())

axis_color = image.Color.from_rgb(0, 220, 255)
zero_color = image.Color.from_rgb(255, 220, 0)
panel_color = image.Color.from_rgb(0, 0, 0)

# 动态中心点偏移 (像素)
CENTER_OFFSET = 85

# 左侧目标点偏移 (像素)
CENTER_OFFSET_LEFT = 85

# 全局状态: 是否已到达过目标中心点并锁定
target_reached = False

# 上一帧小球滤波 x, 用于检测小球从左往右越过切换阈值,
# 防止小球一开始就在阈值右侧导致目标点一直停在左边
filtered_x_prev = None

# 预先计算一些常量
AXIS_CENTER_X = (AXIS_START_PX[0] + AXIS_END_PX[0]) // 2
AXIS_CENTER_Y = (AXIS_START_PX[1] + AXIS_END_PX[1]) // 2
CAM_CENTER_X = detector.input_width() / 2


def draw_calibration_axis(img):
    img.draw_line(
        AXIS_START_PX[0], AXIS_START_PX[1],
        AXIS_END_PX[0], AXIS_END_PX[1],
        axis_color, 2,
    )
    img.draw_cross(AXIS_START_PX[0], AXIS_START_PX[1], axis_color, 7, 2)
    img.draw_cross(AXIS_END_PX[0], AXIS_END_PX[1], axis_color, 7, 2)
    cam_center_color = image.Color.from_rgb(255, 0, 0)
    img.draw_cross(int(CAM_CENTER_X), AXIS_CENTER_Y, cam_center_color, 9, 2)

def send_uart(serial, arrived, pixel_diff):
    if serial is None:
        return
    frame = struct.pack("<BBf", 0x39, arrived, pixel_diff)
    serial.write(frame)

last_ms = time.ticks_ms()
loop_ms = 1000
while not app.need_exit():
    loop_ms = time.ticks_ms() - last_ms
    print_debug("loop cost " + str(loop_ms) + "ms")
    last_ms = time.ticks_ms()

    img = cam.read()
    t = time.ticks_ms()

    # 目标在 -100 时点一下屏幕, 切换回 +100 (仅在有触摸事件时读取, 避免阻塞显示)
    if ts is not None:
        if ts.available():
            _, _, tpressed = ts.read()
            if tpressed and not touch_last and target_reached:
                target_reached = False

            touch_last = tpressed

    if LENS_CORR_ENABLE:
        img = img.lens_corr(strength=LENS_CORR_STRENGTH)
    print_debug("lens_corr cost " + str(time.ticks_ms() - t) + "ms")

    roi_y = 0
    roi_h = detector.input_height()

    t = time.ticks_ms()
    roi_img = img.crop(0, roi_y, detector.input_width(), roi_h)
    objs = detector.detect(roi_img, conf_th=DETECTION_CONFIDENCE, iou_th=0.45)
    for obj in objs:
        obj.y += roi_y
    print_debug("detect cost " + str(time.ticks_ms() - t) + "ms")

    t = time.ticks_ms()
    now_ms = time.ticks_ms()
    draw_calibration_axis(img)

    # 模型只有 steel_ball 一个类别。如果一帧出现多个框，只选择置信度最高的框，
    # 避免多个误检框同时干扰位置滤波器。
    ball = max(objs, key=lambda obj: obj.score) if objs else None
    if ball is not None:
        raw_x = ball.x + ball.w * 0.5
        raw_y = ball.y + ball.h * 0.5
        filtered_x, filtered_y = position_filter.update(raw_x, raw_y, now_ms)
        axis_ratio, _axis_distance = project_onto_axis(
            (filtered_x, filtered_y),
            AXIS_START_PX,
            AXIS_END_PX,
        )
        projected_x, projected_y = axis_point(
            axis_ratio, AXIS_START_PX, AXIS_END_PX
        )

        # 目标点状态机:
        # 初始目标在中心右边 +CENTER_OFFSET, 小球从左侧真正到达
        # "目标点左侧10px"处(从左往右越过阈值)时, 目标点才切到中心左边 -CENTER_OFFSET
        if not target_reached:
            cur_target_x = AXIS_CENTER_X + CENTER_OFFSET
            switch_threshold = cur_target_x - 10
            if (
                filtered_x_prev is not None
                and filtered_x_prev < switch_threshold
                and filtered_x >= switch_threshold
            ):
                target_reached = True
        if target_reached:
            cur_target_x = AXIS_CENTER_X - CENTER_OFFSET_LEFT
        filtered_x_prev = filtered_x

        pixel_diff = filtered_x - cur_target_x

        # 目标点为中心点左边时, 小球到达目标点右边10像素内 -> arrived=1, 否则 0
        arrived = 1 if (target_reached and filtered_x <= cur_target_x + 10) else 0

        img.draw_cross(int(cur_target_x), AXIS_CENTER_Y, zero_color, 9, 2)

        img.draw_rect(ball.x, ball.y, ball.w, ball.h, color=image.COLOR_GREEN, thickness=2)

        if DEBUG_LOG:
            img.draw_cross(
                int(filtered_x), int(filtered_y), image.COLOR_GREEN, 12, 2
            )
            img.draw_cross(
                int(projected_x), int(projected_y), zero_color, 7, 2
            )
            msg = (
                f"{detector.labels[ball.class_id]}: {ball.score:.2f} "
                f"px=({filtered_x:.1f},{filtered_y:.1f})"
            )
            img.draw_string(ball.x, ball.y, msg, color=image.COLOR_RED)
            img.draw_rect(0, 0, detector.input_width(), 35, panel_color, -1)
        img.draw_string(
            8, 5, f"X {pixel_diff:+.1f} ARR{arrived}",
            color=image.COLOR_WHITE, scale=1.4, thickness=2,
        )

        send_uart(serial, arrived, pixel_diff)
    else:
        position_filter.mark_missing(now_ms)

        img.draw_rect(0, 0, detector.input_width(), 35, panel_color, -1)
        img.draw_string(
            8, 5, "BALL LOST",
            color=image.COLOR_RED, scale=1.4, thickness=2,
        )
        lost_target_x = AXIS_CENTER_X - CENTER_OFFSET_LEFT if target_reached else AXIS_CENTER_X + CENTER_OFFSET
        img.draw_cross(lost_target_x, AXIS_CENTER_Y, zero_color, 9, 2)

        send_uart(serial, 0, 0.0)

    fps_str = "FPS:" + str(0 if loop_ms == 0 else 1000//loop_ms)
    img.draw_string(
        img.width() - image.string_size(fps_str, scale=1.4, thickness=2).width(), 5, fps_str,
        color=image.COLOR_GREEN, scale=1.4, thickness=2,
    )
    print_debug("draw and get position cost " + str(time.ticks_ms() - t) + "ms")
    img = img.crop(0, roi_y, detector.input_width(), roi_h)
    img = img.resize(disp.width(), disp.height())
    disp.show(img)