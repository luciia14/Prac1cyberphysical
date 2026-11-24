"""Follow Line (Unibotics, Simple Circuit): PID + feedforward + velocidad adaptativa.
Resultados finales solo por terminal."""
import WebGUI
import HAL
import Frequency
import cv2
import numpy as np
import time
import math
import threading

LAPS_TARGET = 2
FEEDFORWARD = True  # anticipa la curva con la franja lejana
K_FF = 2.437

ADAPTIVE_V = True  # frena en curvas, acelera en rectas
V_MAX = 28.043
V_MIN = 14.027
V_FIXED = 7.018
ACCEL = 12.034

KP = 2.613
KI = 0.318
KD = 0.2573
I_LIMIT = 0.427
D_FILTER = 0.629

W_MAX = 6.052
SPEED_SCALING = True  # W ~ v * curvatura
V_REF = 6.071
SIGN = -1  # línea a la derecha -> W negativo
SHOW_EVERY = 2

MAX_LOST_S = 3.0
TIMEOUT_S = 240.0

LAP_FAR_DIST = 15.0
LAP_NEAR_DIST = 3.0


# franjas de cerca (abajo) a lejos (arriba)
BANDS = [(0.88, 0.94), (0.76, 0.82), (0.66, 0.71), (0.58, 0.62), (0.525, 0.555)]
W_NEAR = [0.45, 0.35, 0.20]

RED_LO1, RED_HI1 = (0, 90, 50), (10, 255, 255)
RED_LO2, RED_HI2 = (165, 90, 50), (180, 255, 255)
KERNEL = np.ones((3, 3), np.uint8)


def red_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, RED_LO1, RED_HI1) | cv2.inRange(hsv, RED_LO2, RED_HI2)
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, KERNEL)


def band_centroid(mask_band, prev_x):
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask_band, 8)
    best, best_score = None, -1e9
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 15:
            continue
        cx = cents[i][0]
        score = area - (0.5 * abs(cx - prev_x) if prev_x is not None else 0)
        if score > best_score:
            best, best_score = cx, score
    return best


def perceive(img, prev_xs):
    h = img.shape[0]
    mask = red_mask(img)
    xs, ys = [], []
    for k, (a, b) in enumerate(BANDS):
        y0, y1 = int(a * h), int(b * h)
        xs.append(band_centroid(mask[y0:y1, :], prev_xs[k]))
        ys.append((y0 + y1) // 2)
    return mask, xs, ys


def norm(x, w):
    return (x - w / 2.0) / (w / 2.0)


def estimate_error(xs, w):
    near = [(norm(x, w), wt) for x, wt in zip(xs[:3], W_NEAR) if x is not None]
    if not near:
        return None
    far = [norm(x, w) for x in xs[3:] if x is not None]

    e = sum(v * wt for v, wt in near) / sum(wt for _, wt in near)
    e_far = far[-1] if far else e
    return e, e_far - e


class Controlador:
    NOMBRES = ("P", "I", "D")

    def __init__(self):
        self.i = 0.0
        self.d = 0.0
        self.prev = None

    def step(self, e, dt):
        self.i = max(-I_LIMIT, min(I_LIMIT, self.i + e * dt))
        if self.prev is not None:
            d_raw = (e - self.prev) / dt
            self.d = D_FILTER * self.d + (1 - D_FILTER) * d_raw
        self.prev = e
        p, i, d = KP * e, KI * self.i, KD * self.d
        return p + i + d, (p, i, d)

    def gains_text(self):
        return "kp=%.4f  ki=%.4f  kd=%.4f" % (KP, KI, KD)


def target_speed(e, curv):
    if not ADAPTIVE_V:
        return V_FIXED
    severity = min(1.0, 1.6 * abs(curv) + 0.8 * abs(e))
    return V_MAX - (V_MAX - V_MIN) * severity


class Odometry:

    def __init__(self):
        self.xy = None
        self.topic = None

    def start(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            from nav_msgs.msg import Odometry as OdometryMsg

            node = Node("follow_line_lap_timer")
            time.sleep(0.8)
            topics = [n for n, t in node.get_topic_names_and_types()
                      if "nav_msgs/msg/Odometry" in t]
            if not topics:
                print("[LAP] No hay topic de odometría: no se pueden cronometrar vueltas automáticamente")
                return
            topic = sorted(topics, key=lambda s: ("f1" not in s, s))[0]

            def on_msg(msg):
                p = msg.pose.pose.position
                self.xy = (p.x, p.y)

            node.create_subscription(OdometryMsg, topic, on_msg, 10)
            ex = SingleThreadedExecutor()
            ex.add_node(node)
            threading.Thread(target=ex.spin, daemon=True).start()
            self.topic = topic
            print("[LAP] Cronometrando con", topic)
        except Exception as exc:
            print("[LAP] Odometría no disponible:", exc)


RUNNING, DONE, FAILED = "running", "done", "failed"


class RunTracker:
    def __init__(self):
        self.t_start = None
        self.lap_t0 = None
        self.start_xy = None
        self.far_away = False
        self.laps = []
        self.lost_since = None
        self.fail_reason = None
        self.t_end = None
        self.n = 0
        self.abs_e = 0.0
        self.sq_e = 0.0
        self.max_e = 0.0
        self.abs_w = 0.0
        self.lost = 0
        self.crossings = 0
        self.last_sign = 0
        self.abs_dw = 0.0
        self.prev_w = None

    def update(self, now, v_cmd, e, w_cmd, lost, xy):
        if self.t_start is None:
            if v_cmd > 0.5:
                self.t_start = self.lap_t0 = now
            else:
                return RUNNING

        self.n += 1
        self.abs_e += abs(e)
        self.sq_e += e * e
        self.max_e = max(self.max_e, abs(e))
        self.abs_w += abs(w_cmd)
        self.lost += int(lost)

        s = 1 if e > 0.02 else (-1 if e < -0.02 else 0)
        if s != 0:
            if self.last_sign != 0 and s != self.last_sign:
                self.crossings += 1
            self.last_sign = s
        if self.prev_w is not None:
            self.abs_dw += abs(w_cmd - self.prev_w)
        self.prev_w = w_cmd

        self.lost_since = (self.lost_since or now) if lost else None
        if self.lost_since is not None and now - self.lost_since > MAX_LOST_S:
            self.fail_reason = "SE SALIO (linea perdida > %.0f s)" % MAX_LOST_S
        elif now - self.t_start > TIMEOUT_S:
            self.fail_reason = "TIMEOUT"
        if self.fail_reason:
            self.t_end = now
            return FAILED

        if xy is None:
            return RUNNING
        if self.start_xy is None:
            self.start_xy = xy
        dist = math.hypot(xy[0] - self.start_xy[0], xy[1] - self.start_xy[1])
        if dist > LAP_FAR_DIST:
            self.far_away = True
        elif self.far_away and dist < LAP_NEAR_DIST:
            self.laps.append(now - self.lap_t0)
            self.lap_t0 = now
            self.far_away = False
            print("[LAP] Vuelta %d: %.2f s" % (len(self.laps), self.laps[-1]))
            if len(self.laps) >= LAPS_TARGET:
                self.t_end = now
                return DONE
        return RUNNING

    def print_summary(self, tag, gains_text):
        line = "=" * 56
        print(line)
        print(" RESULTADO  %s" % tag)
        print(line)
        print(" Ganancias : %s  K_FF=%.3f" % (gains_text, K_FF))
        print(" Velocidad : V_MAX=%.3f  V_MIN=%.3f  V_FIXED=%.3f" % (V_MAX, V_MIN, V_FIXED))
        print("-" * 56)
        if self.fail_reason:
            print(" ESTADO    : FALLO -> %s" % self.fail_reason)
            print(" Vueltas completadas: %d" % len(self.laps))
            for i, t in enumerate(self.laps, 1):
                print("   Vuelta %d : %.2f s" % (i, t))
            print(" Tiempo transcurrido: %.1f s" % (self.t_end - self.t_start))
        else:
            n = max(1, self.n)
            for i, t in enumerate(self.laps, 1):
                print(" Vuelta %d  : %.2f s" % (i, t))
            print(" TOTAL     : %.2f s" % sum(self.laps))
            print("-" * 56)
            print(" MAE       : %.3f" % (self.abs_e / n))
            print(" RMSE      : %.3f" % math.sqrt(self.sq_e / n))
            print(" max|e|    : %.3f" % self.max_e)
            print(" |W| medio : %.3f" % (self.abs_w / n))
            print(" Linea perdida: %.1f%% del tiempo" % (100.0 * self.lost / n))
            print("-" * 56)
            print(" Oscilación")
            print(" Cruces de e por cero : %d" % self.crossings)
            print(" |dW| medio por ciclo : %.3f" % (self.abs_dw / n))
        print(line)


def put_text(img, text, org):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)


def draw_debug(img, mask, xs, ys, lines):
    h, w = img.shape[:2]
    dbg = img.copy()
    dbg[mask > 0] = (0, 255, 0)
    cv2.line(dbg, (w // 2, int(BANDS[-1][0] * h)), (w // 2, h), (255, 255, 255), 1)

    pts = []
    for (a, b), x, y in zip(BANDS, xs, ys):
        cv2.rectangle(dbg, (0, int(a * h)), (w - 1, int(b * h)), (255, 255, 0), 1)
        if x is not None:
            cv2.circle(dbg, (int(x), y), 6, (0, 0, 255), -1)
            pts.append((int(x), y))
    for p1, p2 in zip(pts, pts[1:]):
        cv2.line(dbg, p1, p2, (0, 255, 255), 2)

    for k, s in enumerate(lines):
        put_text(dbg, s, (8, 22 + 22 * k))
    WebGUI.showImage(dbg)


def main():
    odom = Odometry()
    odom.start()

    ctrl = Controlador()
    tracker = RunTracker()

    tag = "PID%s%s" % ("+FF" if FEEDFORWARD else "", "+Vadapt" if ADAPTIVE_V else "+Vfija")
    print("[RUN] Configuración:", tag, ctrl.gains_text(), "K_FF=%.3f" % K_FF if FEEDFORWARD else "")

    prev_xs = [None] * len(BANDS)
    last_e = 0.0
    v_cmd = 0.0
    t_prev = time.time()
    frame = 0
    finished = False

    while True:
        Frequency.tick()
        img = HAL.getImage()
        if img is None:
            continue

        now = time.time()
        dt = max(1e-3, min(0.2, now - t_prev))
        t_prev = now
        frame += 1
        w = img.shape[1]

        if finished:
            HAL.setV(0)
            HAL.setW(0)
            continue

        mask, xs, ys = perceive(img, prev_xs)
        prev_xs = [x if x is not None else p for x, p in zip(xs, prev_xs)]
        est = estimate_error(xs, w)

        if est is not None:
            e, curv = est
            u, terms = ctrl.step(e, dt)
            ff = K_FF * curv if FEEDFORWARD else 0.0
            k_v = max(1.0, v_cmd / V_REF) if SPEED_SCALING else 1.0
            w_cmd = max(-W_MAX, min(W_MAX, SIGN * k_v * (u + ff)))
            v_target = target_speed(e, curv)
            last_e = e
            lost = False
        else:
            # sin línea: gira hacia el último lado visto y frena
            e, curv, ff = last_e, 0.0, 0.0
            terms = (0.0,) * len(Controlador.NOMBRES)
            w_cmd = SIGN * W_MAX * (1 if last_e >= 0 else -1) * 0.8
            v_target = min(V_MIN, V_FIXED) * 0.5
            lost = True

        v_cmd = v_target if v_target < v_cmd else min(v_target, v_cmd + ACCEL * dt)
        HAL.setV(v_cmd)
        HAL.setW(w_cmd)

        status = tracker.update(now, v_cmd, e, w_cmd, lost, odom.xy)
        if status != RUNNING:
            tracker.print_summary(tag, ctrl.gains_text())
            finished = True

        if frame % SHOW_EVERY == 0:
            elapsed = (now - tracker.t_start) if tracker.t_start else 0.0
            terms_txt = " ".join("%s=%+.2f" % (n, v) for n, v in zip(Controlador.NOMBRES, terms))
            lines = [
                tag,
                "v=%.1f w=%+.2f e=%+.2f curv=%+.2f" % (v_cmd, w_cmd, e, curv),
                "%s FF=%+.2f" % (terms_txt, ff),
                "t=%.1fs vueltas=%s" % (elapsed, " ".join("%.1f" % l for l in tracker.laps) or "-"),
            ]
            if lost:
                lines.append("LINEA PERDIDA")
            draw_debug(img, mask, xs, ys, lines)


main()
