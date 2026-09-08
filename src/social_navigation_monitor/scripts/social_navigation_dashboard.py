#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dashboard SII/SGI — cửa sổ riêng và/hoặc panel ảnh trong RViz (/social_dashboard/image)."""

import queue
from collections import deque

import matplotlib as mpl
import numpy as np
import rospy
from sensor_msgs.msg import Image
from social_navigation_monitor.msg import SII, SGI

LABEL_FS = 15
TITLE_FS = 17
TITLE_WEIGHT = "bold"
TICK_FS = 13
ANIM_INTERVAL_MS = 120
DEFAULT_WINDOW_WIDTH = 531
DEFAULT_WINDOW_HEIGHT = 619
DEFAULT_WINDOW_X = 72
DEFAULT_WINDOW_Y = 84
# Tỉ lệ cao/rộng khi image_height:=0 (auto)
# horizontal: thấp + rộng tối đa (strip SII|SGI dưới ô 3D)
AUTO_ASPECT_HORIZONTAL = 0.18
AUTO_ASPECT_VERTICAL = 1.15    # xếp dọc (real POS)


sii_queue = queue.Queue()
sgi_queue = queue.Queue()
social_start_time = None


def _apply_window_geometry(fig):
    w = int(rospy.get_param("~window_width", DEFAULT_WINDOW_WIDTH))
    h = int(rospy.get_param("~window_height", DEFAULT_WINDOW_HEIGHT))
    x = int(rospy.get_param("~window_x", DEFAULT_WINDOW_X))
    y = int(rospy.get_param("~window_y", DEFAULT_WINDOW_Y))
    mgr = fig.canvas.manager
    for setter in (
        lambda: mgr.window.setGeometry(x, y, w, h),
        lambda: mgr.window.geometry("%dx%d+%d+%d" % (w, h, x, y)),
        lambda: mgr.window.wm_geometry("%dx%d+%d+%d" % (w, h, x, y)),
    ):
        try:
            setter()
            return
        except Exception:
            continue
    rospy.logwarn("Khong dat duoc geometry cua so dashboard")


def _apply_axis_font(ax):
    ax.title.set_fontsize(TITLE_FS)
    ax.title.set_fontweight(TITLE_WEIGHT)
    ax.xaxis.label.set_fontsize(LABEL_FS)
    ax.yaxis.label.set_fontsize(LABEL_FS)
    ax.tick_params(axis="both", labelsize=TICK_FS)


def _resolve_image_size(layout):
    """Full chiều rộng (image_width); cao auto nếu image_height<=0."""
    width_px = int(rospy.get_param("~image_width", 1840 if layout == "horizontal" else 560))
    height_px = int(rospy.get_param("~image_height", 0))
    dpi = float(rospy.get_param("~image_dpi", 100.0))
    width_px = max(320, width_px)
    dpi = max(50.0, dpi)
    if height_px <= 0:
        aspect = AUTO_ASPECT_HORIZONTAL if layout == "horizontal" else AUTO_ASPECT_VERTICAL
        height_px = int(round(width_px * aspect))
    # horizontal: giữ dải thấp (không để cao quá)
    if layout == "horizontal":
        height_px = min(height_px, max(140, int(round(width_px * 0.22))))
    height_px = max(120, height_px)
    return width_px, height_px, dpi


def _configure_figure_pixels(fig, width_px, height_px, dpi):
    fig.set_dpi(dpi)
    fig.set_size_inches(width_px / dpi, height_px / dpi, forward=True)
    try:
        fig.set_constrained_layout_pads(w_pad=0.02, h_pad=0.04, wspace=0.04, hspace=0.06)
    except Exception:
        pass
    # Ép canvas Agg đúng pixel (tránh lệch DPI)
    try:
        fig.canvas.resize(width_px, height_px)
    except Exception:
        pass


def get_message_time_sec(msg):
    if hasattr(msg, "header") and msg.header and msg.header.stamp:
        stamp_sec = msg.header.stamp.to_sec()
        if stamp_sec > 0:
            return stamp_sec
    return rospy.Time.now().to_sec()


def sii_callback(msg):
    global social_start_time
    msg_time_sec = get_message_time_sec(msg)
    if social_start_time is None:
        social_start_time = msg_time_sec
    t = max(0.0, msg_time_sec - social_start_time)
    sii_queue.put((t, float(msg.sii)))


def sgi_callback(msg):
    global social_start_time
    msg_time_sec = get_message_time_sec(msg)
    if social_start_time is None:
        social_start_time = msg_time_sec
    t = max(0.0, msg_time_sec - social_start_time)
    sgi_queue.put((t, float(msg.sgi)))


def _fig_to_image_msg(fig):
    fig.canvas.draw()
    try:
        buf = np.asarray(fig.canvas.buffer_rgba())
        rgb = buf[:, :, :3]
    except Exception:
        w, h = fig.canvas.get_width_height()
        buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        rgb = buf.reshape((h, w, 3))
    h, w = rgb.shape[:2]
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "social_dashboard"
    msg.height = h
    msg.width = w
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = w * 3
    msg.data = np.ascontiguousarray(rgb).tobytes()
    return msg


def main():
    rospy.init_node("social_navigation_dashboard", anonymous=True)

    max_points = int(rospy.get_param("~max_plot_points", 6000))
    publish_image = bool(rospy.get_param("~publish_image", True))
    show_window = bool(rospy.get_param("~show_window", True))
    image_topic = str(rospy.get_param("~image_topic", "/social_dashboard/image"))
    layout = str(rospy.get_param("~layout", "vertical")).strip().lower()
    if layout not in ("horizontal", "vertical"):
        rospy.logwarn("layout=%s khong hop le; dung vertical.", layout)
        layout = "vertical"

    width_px, height_px, dpi = _resolve_image_size(layout)

    if show_window:
        for backend in ("Qt5Agg", "TkAgg"):
            try:
                mpl.use(backend)
                break
            except Exception:
                continue
        else:
            rospy.logwarn("Khong co backend GUI; chi publish anh RViz.")
            mpl.use("Agg")
            show_window = False
    else:
        mpl.use("Agg")

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    sii_data = deque(maxlen=max_points)
    sgi_data = deque(maxlen=max_points)

    image_pub = None
    if publish_image:
        image_pub = rospy.Publisher(image_topic, Image, queue_size=1, latch=True)

    mpl.rcParams["figure.autolayout"] = False
    figsize = (width_px / dpi, height_px / dpi)
    if layout == "horizontal":
        fig, axes = plt.subplots(
            1, 2, figsize=figsize, constrained_layout=True, sharey=True
        )
    else:
        fig, axes = plt.subplots(
            2, 1, figsize=figsize, constrained_layout=True, sharex=True
        )
    _configure_figure_pixels(fig, width_px, height_px, dpi)

    try:
        fig.canvas.manager.set_window_title("Social Dashboard (SII/SGI)")
    except Exception:
        pass
    if show_window:
        _apply_window_geometry(fig)

    ax_sii, ax_sgi = axes

    ax_sii.set_xlabel("Time (s)")
    ax_sii.set_ylabel("SII")
    ax_sii.set_title("Social Individual Index (SII)")
    ax_sii.grid(True, alpha=0.3)
    ax_sii.set_ylim(0, 1)
    sii_line, = ax_sii.plot([], [], "b-", label="SII", linewidth=1.5)
    ax_sii.axhline(y=0.14, color="cyan", linestyle="--", linewidth=1, label="Tc 0.14")
    ax_sii.axhline(y=0.54, color="red", linestyle="--", linewidth=1, label="Tp 0.54")
    ax_sii.legend(loc="upper right", fontsize=TICK_FS)
    _apply_axis_font(ax_sii)

    ax_sgi.set_xlabel("Time (s)")
    ax_sgi.set_ylabel("SGI")
    ax_sgi.set_title("Social Group Index (SGI)")
    ax_sgi.grid(True, alpha=0.3)
    ax_sgi.set_ylim(0, 1)
    sgi_line, = ax_sgi.plot([], [], "r-", label="SGI", linewidth=1.5)
    ax_sgi.axhline(y=0.14, color="cyan", linestyle="--", linewidth=1, label="Tg 0.14")
    ax_sgi.legend(loc="upper right", fontsize=TICK_FS)
    _apply_axis_font(ax_sgi)

    rospy.Subscriber("/sii", SII, sii_callback, queue_size=50)
    rospy.Subscriber("/sgi", SGI, sgi_callback, queue_size=50)

    def update_plot(_frame=None):
        if rospy.is_shutdown():
            return
        while not sii_queue.empty():
            sii_data.append(sii_queue.get())
        while not sgi_queue.empty():
            sgi_data.append(sgi_queue.get())

        if sii_data:
            tx, ty = zip(*sii_data)
            sii_line.set_data(tx, ty)
            t0, t1 = min(tx), max(tx)
            ax_sii.set_xlim(t0, t1 if t1 > t0 else t0 + 1e-6)

        if sgi_data:
            tx, ty = zip(*sgi_data)
            sgi_line.set_data(tx, ty)
            t0, t1 = min(tx), max(tx)
            ax_sgi.set_xlim(t0, t1 if t1 > t0 else t0 + 1e-6)

        if image_pub is not None:
            try:
                image_pub.publish(_fig_to_image_msg(fig))
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "Publish dashboard image that bai: %s", exc)

    rospy.loginfo(
        "social_navigation_dashboard: layout=%s size=%dx%d dpi=%.0f "
        "show_window=%s publish_image=%s topic=%s",
        layout,
        width_px,
        height_px,
        dpi,
        show_window,
        publish_image,
        image_topic if publish_image else "(off)",
    )

    if show_window:
        _ani = FuncAnimation(fig, update_plot, interval=ANIM_INTERVAL_MS, cache_frame_data=False)
        plt.show(block=True)
    else:
        rospy.Timer(rospy.Duration(ANIM_INTERVAL_MS / 1000.0), update_plot)
        update_plot()
        rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
