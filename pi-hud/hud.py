import math
import sys
import threading
import time

import pygame

try:
    from sensors import Sensors
except Exception:
    Sensors = None

sensor_reader = None

# ==================== COLORS ====================
COLOR_GROUND = (90, 60, 30)
COLOR_SKY = (0, 100, 200)
COLOR_TAPE = (28, 28, 28)
COLOR_BORDER = (80, 80, 80)
COLOR_DIM = (120, 120, 120)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
YELLOW = (255, 255, 0)
GREEN = (0, 200, 0)
RED = (220, 30, 30)
CYAN = (0, 200, 200)

# ==================== STATE ====================
state = {
    "pitch": 0.0,
    "roll": 0.0,
    "heading": 0.0,
    "altitude_m": 300.0,
    "speed_mph": 45.0,
    "gps_fix": False,
    "sats": 0,
    "lat": 45.678900,
    "lon": -110.123400,
    "qnh": 1013.25,
    "imu_ok": True,
    "mag_ok": True,
    "bmp_ok": True,
    "gps_ok": True,
}

current_screen = 0  # 0=Horizon 1=GPS 2=Info 3=Calibrate
menu_active = False
menu_items = ["Horizon", "GPS", "Info", "Calibrate"]


def update_simulated_data(t):
    """Fallback data source, used when no I2C sensors are found (e.g. on a dev machine)."""
    state["pitch"] = 15 * math.sin(t * 0.3)
    state["roll"] = 25 * math.sin(t * 0.5 + 1)
    state["heading"] = (t * 10) % 360
    state["altitude_m"] = 300 + 50 * math.sin(t * 0.1)
    state["speed_mph"] = 45 + 10 * math.sin(t * 0.2)
    state["gps_fix"] = t > 3
    state["sats"] = 8 if state["gps_fix"] else 0
    state["lat"] = 45.678900 + 0.0005 * math.sin(t * 0.05)
    state["lon"] = -110.123400 + 0.0005 * math.cos(t * 0.05)


def sensor_loop(reader):
    """Runs on a background thread, continuously refreshing `state` from the I2C sensors."""
    while True:
        reader.update()
        state["pitch"] = reader.pitch
        state["roll"] = reader.roll
        state["heading"] = reader.heading
        state["altitude_m"] = reader.altitude_m
        state["speed_mph"] = reader.speed_mph
        state["gps_fix"] = reader.gps_fix
        state["sats"] = reader.sats
        state["lat"] = reader.lat
        state["lon"] = reader.lon
        state["qnh"] = reader.sea_level_hpa
        state["imu_ok"] = reader.imu_ok
        state["mag_ok"] = reader.mag_ok
        state["bmp_ok"] = reader.bmp_ok
        state["gps_ok"] = reader.gps_ok
        time.sleep(0.02)


def draw_menu_button(surf, font_big, W, H):
    bw, bh = W * 0.13, H * 0.09
    r = pygame.Rect(0, H - bh, bw, bh)
    pygame.draw.rect(surf, (30, 30, 30), r, border_radius=10)
    pygame.draw.rect(surf, WHITE, r, width=2, border_radius=10)
    label = font_big.render("MENU", True, WHITE)
    surf.blit(label, (r.x + 20, r.y + bh / 2 - label.get_height() / 2))

    bw2, bh2 = W * 0.13, H * 0.08
    r2 = pygame.Rect(W - bw2 - 10, 10, bw2, bh2)
    pygame.draw.rect(surf, (40, 40, 40), r2, border_radius=10)
    pygame.draw.rect(surf, WHITE, r2, width=2, border_radius=10)
    label2 = font_big.render("< BACK", True, WHITE)
    surf.blit(label2, (r2.x + 10, r2.y + bh2 / 2 - label2.get_height() / 2))


def draw_menu(surf, font_menu, W, H):
    surf.fill(BLACK)
    row_h = H * 0.14
    top = H * 0.09
    for i, item in enumerate(menu_items):
        y = top + i * row_h
        if i == current_screen:
            pygame.draw.rect(surf, (30, 30, 30), (0, y, W, row_h))
        pygame.draw.line(surf, (60, 60, 60), (0, y), (W, y), 1)
        color = YELLOW if i == current_screen else WHITE
        label = font_menu.render(item, True, color)
        surf.blit(label, (W * 0.11, y + row_h / 2 - label.get_height() / 2))
    pygame.draw.line(surf, (60, 60, 60), (0, top + len(menu_items) * row_h), (W, top + len(menu_items) * row_h), 1)


def draw_horizon(surf, font_sm, W, H):
    body_h = H * 0.86
    cx, cy = W / 2, body_h / 2

    r = math.radians(state["roll"])
    p = state["pitch"] * (H / 105.0)

    max_angle = math.radians(85)
    r = max(-max_angle, min(max_angle, r))
    slope = math.tan(r)

    y1 = cy + p + (0 - cx) * slope
    y2 = cy + p + (W - cx) * slope

    horizon_poly_sky = [(0, 0), (W, 0), (W, y2), (0, y1)]
    horizon_poly_ground = [(0, y1), (W, y2), (W, body_h), (0, body_h)]
    surf.fill(COLOR_SKY)
    pygame.draw.polygon(surf, COLOR_GROUND, horizon_poly_ground)
    pygame.draw.line(surf, WHITE, (0, y1), (W, y2), 3)

    px_per_deg = H / 60.0
    half_len = W * 0.09
    gap = W * 0.03

    for pv in range(-30, 31, 5):
        if pv == 0:
            continue
        y_at_cx = cy + p - pv * px_per_deg
        if y_at_cx < 10 or y_at_cx > body_h - 10:
            continue
        major = (pv % 10 == 0)
        length = half_len if major else half_len / 2

        l_o = y_at_cx + (-length) * slope
        r_o = y_at_cx + (length) * slope
        l_inner = y_at_cx + (-gap) * slope
        r_inner = y_at_cx + (gap) * slope

        if pv < 0:
            # dashed for below-horizon (ground) marks
            steps = int(length // 8)
            for s in range(steps):
                s1 = s * 8
                s2 = min(s1 + 4, length)
                ly1 = y_at_cx + (-length + s1) * slope
                ly2 = y_at_cx + (-length + s2) * slope
                ry1 = y_at_cx + (length - s2) * slope
                ry2 = y_at_cx + (length - s1) * slope
                pygame.draw.line(surf, WHITE, (cx - length + s1, ly1), (cx - length + s2, ly2), 2)
                pygame.draw.line(surf, WHITE, (cx + length - s2, ry1), (cx + length - s1, ry2), 2)
        else:
            pygame.draw.line(surf, WHITE, (cx - length, l_o), (cx - gap, l_inner), 2)
            pygame.draw.line(surf, WHITE, (cx + gap, r_inner), (cx + length, r_o), 2)

        if major:
            label = font_sm.render(str(abs(pv)), True, WHITE)
            surf.blit(label, (cx - length - 30, l_o - 8))
            surf.blit(label, (cx + length + 6, r_o - 8))

    # center aircraft symbol
    wl = W * 0.06
    pygame.draw.line(surf, YELLOW, (cx - wl - 30, cy), (cx - 30, cy), 3)
    pygame.draw.line(surf, YELLOW, (cx + 30, cy), (cx + 30 + wl, cy), 3)
    pygame.draw.line(surf, YELLOW, (cx - 30, cy), (cx - 30, cy + 8), 3)
    pygame.draw.line(surf, YELLOW, (cx + 30, cy), (cx + 30, cy + 8), 3)
    pygame.draw.rect(surf, YELLOW, (cx - 3, cy - 1, 6, 4))


def draw_roll_tape(surf, font_sm, font_big, W, H):
    tape_w = W * 0.12
    tape_h = H * 0.86
    cy = tape_h / 2
    scale = tape_h / 168.0

    pygame.draw.rect(surf, COLOR_TAPE, (0, 0, tape_w, tape_h))
    pygame.draw.line(surf, COLOR_BORDER, (tape_w, 0), (tape_w, tape_h), 1)

    r = state["roll"]
    for deg in range(-90, 91, 5):
        y = cy - (deg - r) * scale
        if y < 0 or y >= tape_h:
            continue
        major = (deg % 10 == 0)
        tick_len = tape_w * 0.35 if major else tape_w * 0.18
        pygame.draw.line(surf, COLOR_BORDER, (tape_w - tick_len, y), (tape_w, y), 2)
        if major:
            label = font_sm.render(str(deg), True, WHITE)
            surf.blit(label, (tape_w - tick_len - 34, y - 8))

    pygame.draw.polygon(surf, WHITE, [(tape_w + 1, cy), (tape_w + 14, cy - 8), (tape_w + 14, cy + 8)])

    roll_val = round(r)
    label = font_big.render(str(roll_val), True, GREEN)
    box_h = label.get_height() + 16
    box = pygame.Rect(0, cy - box_h / 2, tape_w, box_h)
    pygame.draw.rect(surf, BLACK, box)
    pygame.draw.rect(surf, WHITE, box, 2)
    surf.blit(label, (tape_w / 2 - label.get_width() / 2, cy - label.get_height() / 2))

    tag = font_sm.render("ROLL", True, WHITE)
    surf.blit(tag, (tape_w / 2 - tag.get_width() / 2, box.top - tag.get_height() - 4))


def draw_altitude_tape(surf, font_sm, font_big, W, H):
    tape_w = W * 0.12
    tape_h = H * 0.86
    tape_x = W - tape_w
    cy = tape_h / 2
    scale = tape_h / 240.0

    pygame.draw.rect(surf, COLOR_TAPE, (tape_x, 0, tape_w, tape_h))
    pygame.draw.line(surf, COLOR_BORDER, (tape_x - 1, 0), (tape_x - 1, tape_h), 1)

    altitude_ft = state["altitude_m"] * 3.28084
    start_ft = (int(altitude_ft / 20)) * 20 - 120

    for ft in range(start_ft, start_ft + 241, 20):
        y = cy - (ft - altitude_ft) * scale
        if y < 0 or y >= tape_h:
            continue
        major = (ft % 100 == 0)
        tick_len = tape_w * 0.35 if major else tape_w * 0.18
        pygame.draw.line(surf, COLOR_BORDER, (tape_x, y), (tape_x + tick_len, y), 2)
        if major:
            label = font_sm.render(str(ft), True, WHITE)
            surf.blit(label, (tape_x + tick_len + 4, y - 8))

    pygame.draw.polygon(surf, WHITE, [(tape_x - 2, cy), (tape_x - 15, cy - 8), (tape_x - 15, cy + 8)])

    alt_val = round(altitude_ft)
    label = font_big.render(str(alt_val), True, GREEN)
    box_h = label.get_height() + 16
    box = pygame.Rect(tape_x, cy - box_h / 2, tape_w, box_h)
    pygame.draw.rect(surf, BLACK, box)
    pygame.draw.rect(surf, WHITE, box, 2)
    surf.blit(label, (tape_x + 8, cy - label.get_height() / 2))

    tag = font_sm.render("ALT", True, WHITE)
    surf.blit(tag, (tape_x + tape_w / 2 - tag.get_width() / 2, box.top - tag.get_height() - 4))


def draw_heading_tape(surf, font_sm, font_big, W, H):
    bar_y = H * 0.86
    bar_h = H * 0.14
    cx = W / 2
    px_per_deg = W / 200.0

    pygame.draw.rect(surf, COLOR_TAPE, (0, bar_y, W, bar_h))
    pygame.draw.line(surf, COLOR_BORDER, (0, bar_y), (W, bar_y), 1)

    heading = state["heading"]
    for b in range(-60, 61):
        deg = int(heading + b + 360) % 360
        x = cx + b * px_per_deg
        if x < 0 or x >= W:
            continue
        if deg % 30 == 0:
            pygame.draw.line(surf, WHITE, (x, bar_y + 2), (x, bar_y + 18), 2)
            labels = {0: "N", 90: "E", 180: "S", 270: "W"}
            if deg in labels:
                label = font_sm.render(labels[deg], True, GREEN)
            else:
                label = font_sm.render(str(deg), True, WHITE)
            surf.blit(label, (x - label.get_width() / 2, bar_y + 20))
        elif deg % 10 == 0:
            pygame.draw.line(surf, COLOR_BORDER, (x, bar_y + 2), (x, bar_y + 10), 2)

    pygame.draw.polygon(surf, WHITE, [(cx, bar_y + 1), (cx - 8, bar_y - 8), (cx + 8, bar_y - 8)])

    label = font_big.render("%03d" % (int(heading) % 360), True, GREEN)
    box_w = label.get_width() + 20
    box_h = label.get_height() + 12
    box_y = bar_y + 18 + (bar_h - 18 - box_h) / 2
    box = pygame.Rect(cx - box_w / 2, box_y, box_w, box_h)
    pygame.draw.rect(surf, BLACK, box)
    pygame.draw.rect(surf, WHITE, box, 2)
    surf.blit(label, (cx - label.get_width() / 2, box.centery - label.get_height() / 2))


def draw_overlay(surf, font_big, font_sm, W, H):
    if abs(state["roll"]) > 35 or abs(state["pitch"]) > 35:
        label = font_big.render("! DANGER !", True, RED)
        surf.blit(label, (W / 2 - label.get_width() / 2, 10))

    box = pygame.Rect(W * 0.03, H - H * 0.11, W * 0.09, H * 0.07)
    pygame.draw.rect(surf, (40, 40, 40), box, border_radius=6)
    pygame.draw.rect(surf, COLOR_BORDER, box, width=2, border_radius=6)
    label = font_big.render("CAL", True, WHITE)
    surf.blit(label, (box.x + 8, box.y + box.h / 2 - label.get_height() / 2))

    gps_label = font_sm.render("GPS" if state["gps_fix"] else "---", True, GREEN if state["gps_fix"] else COLOR_DIM)
    surf.blit(gps_label, (W - 90, H - 55))
    mag_label = font_sm.render("MAG" if state["mag_ok"] else "GYR", True, GREEN if state["mag_ok"] else COLOR_DIM)
    surf.blit(mag_label, (W - 130, H - 55))


def draw_gps_screen(surf, font_title, font_body, W, H):
    surf.fill(BLACK)
    title = font_title.render("GPS", True, CYAN)
    surf.blit(title, (40, 20))

    if state["gps_fix"]:
        fix = font_title.render("FIX", True, GREEN)
        surf.blit(fix, (220, 20))
        lines = [
            "LAT: %.6f" % state["lat"],
            "LON: %.6f" % state["lon"],
            "SPD: %.1f mph" % state["speed_mph"],
            "ALT: %d ft" % int(state["altitude_m"] * 3.28084),
        ]
    else:
        fix = font_title.render("NO FIX", True, RED)
        surf.blit(fix, (220, 20))
        lines = []

    y = 120
    for line in lines:
        label = font_body.render(line, True, WHITE)
        surf.blit(label, (40, y))
        y += 55

    sats = font_body.render("SATS: %d" % state["sats"], True, WHITE)
    surf.blit(sats, (40, H - H * 0.22))

    draw_menu_button(surf, font_body, W, H)


def draw_info_screen(surf, font_title, font_body, font_small, W, H):
    surf.fill(BLACK)
    title = font_title.render("INFO", True, CYAN)
    surf.blit(title, (40, 20))

    lines = [
        "Pitch: %.1f" % state["pitch"],
        "Roll:  %.1f" % state["roll"],
        "Hdg:   %.0f" % state["heading"],
        "Alt:   %d ft" % int(state["altitude_m"] * 3.28084),
        "QNH:   %.1f hPa" % state["qnh"],
    ]
    y = 110
    for line in lines:
        label = font_body.render(line, True, WHITE)
        surf.blit(label, (40, y))
        y += 50

    status = "IMU:%s  MAG:%s  BARO:%s  GPS:%s" % (
        "OK" if state["imu_ok"] else "--",
        "OK" if state["mag_ok"] else "--",
        "OK" if state["bmp_ok"] else "--",
        "OK" if state["gps_ok"] else "--",
    )
    label = font_small.render(status, True, COLOR_DIM)
    surf.blit(label, (40, H - H * 0.11))

    draw_menu_button(surf, font_body, W, H)


def draw_calibrate_screen(surf, font_title, font_body, font_small, W, H):
    surf.fill(BLACK)
    title = font_title.render("CALIBRATE", True, YELLOW)
    surf.blit(title, (40, 10))
    pygame.draw.line(surf, (60, 60, 60), (0, 80), (W, 80), 1)

    r1 = pygame.Rect(10, 88, W * 0.48, 74)
    pygame.draw.rect(surf, (50, 15, 15), r1, border_radius=10)
    pygame.draw.rect(surf, RED, r1, width=2, border_radius=10)
    r2 = pygame.Rect(W * 0.51, 88, W * 0.48, 74)
    pygame.draw.rect(surf, (15, 50, 15), r2, border_radius=10)
    pygame.draw.rect(surf, GREEN, r2, width=2, border_radius=10)

    surf.blit(font_body.render("QNH  -", True, WHITE), (r1.x + 20, r1.y + 10))
    surf.blit(font_body.render("QNH  +", True, WHITE), (r2.x + 20, r2.y + 10))
    qnh_label = font_small.render("%.1f hPa" % state["qnh"], True, YELLOW)
    surf.blit(qnh_label, (W / 2 - qnh_label.get_width() / 2, 140))

    pygame.draw.line(surf, (60, 60, 60), (0, 180), (W, 180), 1)
    box2 = pygame.Rect(10, 188, W - 20, 94)
    pygame.draw.rect(surf, (20, 20, 60), box2, border_radius=10)
    pygame.draw.rect(surf, CYAN, box2, width=2, border_radius=10)
    surf.blit(font_body.render("LEVEL HORIZON", True, WHITE), (30, 198))
    surf.blit(font_small.render("P: %.1f   R: %.1f" % (state["pitch"], state["roll"]), True, YELLOW), (30, 248))

    pygame.draw.line(surf, (60, 60, 60), (0, 300), (W, 300), 1)
    box3 = pygame.Rect(10, 308, W - 20, 94)
    pygame.draw.rect(surf, (20, 20, 60), box3, border_radius=10)
    pygame.draw.rect(surf, CYAN, box3, width=2, border_radius=10)
    surf.blit(font_body.render("ZERO HEADING", True, WHITE), (30, 318))
    surf.blit(font_small.render("HDG: %.0f  %s" % (state["heading"], "(MAG)" if state["mag_ok"] else "(GYRO)"), True, YELLOW), (30, 366))

    draw_menu_button(surf, font_body, W, H)


def handle_tap(x, y, W, H):
    global current_screen, menu_active

    if menu_active:
        row_h = H * 0.14
        top = H * 0.09
        for i in range(len(menu_items)):
            ry = top + i * row_h
            if ry <= y < ry + row_h:
                current_screen = i
                menu_active = False
                return
        return

    # back button (non-horizon screens)
    if current_screen != 0 and x >= W - W * 0.13 - 10 and y <= 10 + H * 0.08:
        current_screen = 0
        return

    # menu button (non-horizon screens)
    if current_screen != 0 and x < W * 0.13 and y > H - H * 0.09:
        menu_active = True
        return

    # CAL shortcut on horizon screen
    if current_screen == 0 and x < W * 0.13 and y > H - H * 0.11:
        current_screen = 3
        return

    if current_screen == 3:
        if 88 <= y < 162:
            delta = -1.0 if x < W / 2 else 1.0
            if sensor_reader is not None:
                sensor_reader.adjust_qnh(delta)
            else:
                state["qnh"] += delta
        elif 188 <= y < 282:
            if sensor_reader is not None:
                sensor_reader.zero_horizon()
        elif 308 <= y < 402:
            if sensor_reader is not None:
                sensor_reader.zero_heading()


def main():
    global sensor_reader

    if Sensors is not None:
        try:
            sensor_reader = Sensors()
            sensor_reader.calibrate_gyro()
            threading.Thread(target=sensor_loop, args=(sensor_reader,), daemon=True).start()
        except Exception as e:
            print(f"Sensor init failed, using simulated data: {e}", file=sys.stderr)
            sensor_reader = None

    pygame.init()
    pygame.mouse.set_visible(False)
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    W, H = screen.get_size()
    pygame.display.set_caption("Flight Folder HUD")

    font_sm = pygame.font.SysFont("dejavusans", int(H * 0.038))
    font_body = pygame.font.SysFont("dejavusans", int(H * 0.058), bold=True)
    font_big = pygame.font.SysFont("dejavusans", int(H * 0.065), bold=True)
    font_title = pygame.font.SysFont("dejavusans", int(H * 0.09), bold=True)
    font_menu = pygame.font.SysFont("dejavusans", int(H * 0.075), bold=True)

    clock = pygame.time.Clock()
    start = time.time()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                handle_tap(event.pos[0], event.pos[1], W, H)
            elif event.type == pygame.FINGERDOWN:
                handle_tap(event.x * W, event.y * H, W, H)

        if sensor_reader is None:
            t = time.time() - start
            update_simulated_data(t)

        if menu_active:
            draw_menu(screen, font_menu, W, H)
        elif current_screen == 0:
            draw_horizon(screen, font_sm, W, H)
            draw_roll_tape(screen, font_sm, font_big, W, H)
            draw_altitude_tape(screen, font_sm, font_big, W, H)
            draw_heading_tape(screen, font_sm, font_big, W, H)
            draw_overlay(screen, font_big, font_sm, W, H)
        elif current_screen == 1:
            draw_gps_screen(screen, font_title, font_body, W, H)
        elif current_screen == 2:
            draw_info_screen(screen, font_title, font_body, font_sm, W, H)
        elif current_screen == 3:
            draw_calibrate_screen(screen, font_title, font_body, font_sm, W, H)

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
