"""差動二輪ロボット 搬送タスク動画
記事 (qiita.com/N622/items/9d89f77d85d9da0af29e) のロボット仕様を再現:
  DiffDrive: wheel_radius 0.033, wheel_separation 0.19, topic cmd_vel
  teleop_node: w/s -> linear.x=±1, a/d -> angular.z=±1, その他キー -> 停止
  キーを離しても最後の指令が保持される (記事の注意書きどおり)
"""
import math, sys, os, subprocess, pickle
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H, SS = 1280, 720, 2
SW, SH = W * SS, H * SS
FPS = 30
R, SEP, TAU = 0.033, 0.19, 0.12
KEYS = {'w': (1.0, 0.0), 's': (-1.0, 0.0), 'a': (0.0, 1.0), 'd': (0.0, -1.0), ' ': (0.0, 0.0)}

def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi

OBST = [(-0.7, -0.4, 0.7, 0.7, 0.35, (160, 118, 74)),
        (0.9, 0.6, 0.4, 1.2, 0.30, (118, 124, 136)),
        (-1.5, 0.6, 1.0, 0.3, 0.25, (160, 118, 74)),
        (0.6, -1.4, 0.3, 0.3, 0.45, (118, 124, 136))]
CUBES0 = [(-1.9, 1.9), (0.2, 2.0), (2.0, 1.0)]
CUBE_COL = [(236, 122, 40), (52, 132, 226), (168, 86, 200)]
NAMES = ['A', 'B', 'C']
SLOTS = [(1.95, -1.95), (2.1, -1.55), (1.55, -2.1)]
ZONE = (1.8, -1.8, 1.0, 1.0)
START = (-2.0, -2.0, 0.0)
CUBE_S, DECK_Z = 0.10, 0.095
CARGO_L = (0.05, 0.0, DECK_Z + CUBE_S / 2)
ARENA = 2.5

# ---------------------------------------------------------------- simulation
class Sim:
    def __init__(s):
        s.x, s.y, s.th = START
        s.v = s.w = s.vc = s.wc = 0.0
        s.key, s.keyt = None, -9
        s.t, s.nf, s.aL, s.aR, s.dist = 0.0, 0, 0.0, 0.0, 0.0
        s.cubes = [{'mode': 'floor', 'p': (cx, cy, CUBE_S / 2), 'yaw': 0.0} for cx, cy in CUBES0]
        s.caption, s.log, s.done = '', [], [False] * 6
        s.frames, s.trail = [], []

    def l2w(s, lx, ly, lz):
        c, sn = math.cos(s.th), math.sin(s.th)
        return (s.x + lx * c - ly * sn, s.y + lx * sn + ly * c, lz)

    def cube_state(s, i):
        c = s.cubes[i]
        if c['mode'] == 'floor': return c['p'], c['yaw']
        if c['mode'] == 'carried': return s.l2w(*CARGO_L), s.th
        u = min(1.0, max(0.0, (s.t - c['t0']) / c['dur']))
        e = u * u * (3 - 2 * u)
        a, b = np.array(c['a']), np.array(c['b'])
        p = a + (b - a) * e
        p[2] += 0.07 * math.sin(math.pi * e)
        return tuple(p), c['ya'] + wrap(c['yb'] - c['ya']) * e

    def record(s):
        s.trail.append((s.x, s.y))
        s.frames.append(dict(t=s.t, x=s.x, y=s.y, th=s.th, v=s.v, w=s.w, vc=s.vc, wc=s.wc,
                             key=s.key, keyt=s.keyt, aL=s.aL, aR=s.aR,
                             wl=(s.v - s.w * SEP / 2) / R, wr=(s.v + s.w * SEP / 2) / R,
                             cubes=[s.cube_state(i) for i in range(3)], caption=s.caption,
                             log=list(s.log[-7:]), done=list(s.done), dist=s.dist, tr=len(s.trail)))

    def run(s, T):
        end = s.t + T
        while s.t < end - 1e-12:
            nft = s.nf / FPS
            if s.t >= nft - 1e-12:
                s.record(); s.nf += 1; continue
            h = min(1 / 480, end - s.t, nft - s.t)
            k = 1 - math.exp(-h / TAU)
            s.v += (s.vc - s.v) * k
            s.w += (s.wc - s.w) * k
            thm = s.th + s.w * h / 2
            s.x += s.v * math.cos(thm) * h
            s.y += s.v * math.sin(thm) * h
            s.th = wrap(s.th + s.w * h)
            s.dist += abs(s.v) * h
            s.aL += (s.v - s.w * SEP / 2) / R * h
            s.aR += (s.v + s.w * SEP / 2) / R * h
            s.t += h

    def say(s, msg): s.log.append(f"{s.t:6.2f} {msg}")

    def press(s, k):
        s.key, s.keyt = k, s.t
        s.vc, s.wc = KEYS[k]
        s.say(f"key={'SPC' if k == ' ' else k}  lin.x={s.vc:+.1f} ang.z={s.wc:+.1f}")

    def turn_to(s, hd):
        d = wrap(hd - s.th)
        if abs(d) < 0.015: return
        s.press('a' if d > 0 else 'd'); s.run(abs(d)); s.press(' '); s.run(0.45)

    def goto(s, px, py):
        s.turn_to(math.atan2(py - s.y, px - s.x))
        d = math.hypot(px - s.x, py - s.y)
        s.press('w'); s.run(d); s.press(' '); s.run(0.6)

    def approach(s, T, off):
        dx, dy = T[0] - s.x, T[1] - s.y
        n = math.hypot(dx, dy)
        s.goto(T[0] - off * dx / n, T[1] - off * dy / n)
        s.turn_to(math.atan2(T[1] - s.y, T[0] - s.x))

    def reverse(s, d):
        s.press('s'); s.run(d); s.press(' '); s.run(0.6)

    def anim(s, i, a, b, ya, yb, dur, final):
        s.cubes[i] = {'mode': 'anim', 'a': a, 'b': b, 'ya': ya, 'yb': yb, 't0': s.t, 'dur': dur}
        s.run(dur + 0.3)
        s.cubes[i] = final

    def load(s, i):
        s.caption = f'荷物{NAMES[i]}を積み込み'
        p, y = s.cube_state(i)
        s.anim(i, p, s.l2w(*CARGO_L), y, s.th, 2.2, {'mode': 'carried'})
        s.done[2 * i] = True; s.say(f"[task] cargo {NAMES[i]} loaded")

    def unload(s, i):
        s.caption = f'荷物{NAMES[i]}を降ろす'
        b = s.l2w(0.28, 0, CUBE_S / 2)
        s.anim(i, s.l2w(*CARGO_L), b, s.th, s.th, 2.2, {'mode': 'floor', 'p': b, 'yaw': s.th})
        s.done[2 * i + 1] = True; s.say(f"[task] cargo {NAMES[i]} delivered")


def simulate():
    s = Sim()
    s.log = ["$ ros2 launch nav_dev nav_teleop.launch.py",
             "[ign gazebo] world.sdf loaded",
             "[bridge] /cmd_vel ROS2 -> ignition.msgs.Twist",
             "[teleop_node] waiting for key input..."]
    s.caption = '準備完了'
    s.run(6.0)
    # --- 荷物A
    s.caption = '荷物Aへ移動'
    s.goto(-2.25, -1.6); s.goto(-2.25, 1.2); s.approach(CUBES0[0], 0.25)
    s.load(0)
    s.caption = '後退して向きを変える'; s.reverse(0.35)
    s.caption = '荷物Aを倉庫エリアへ搬送'
    s.goto(-2.25, 1.2); s.goto(-2.25, -1.0); s.goto(0.0, -1.0); s.goto(1.2, -1.0); s.goto(1.4, -1.4)
    s.approach(SLOTS[0], 0.28); s.unload(0); s.reverse(0.35)
    # --- 荷物B
    s.caption = '荷物Bへ移動'
    s.goto(1.6, -0.3); s.goto(1.6, 1.55); s.goto(0.6, 1.55); s.approach(CUBES0[1], 0.25)
    s.load(1)
    s.caption = '後退して向きを変える'; s.reverse(0.35)
    s.caption = '荷物Bを倉庫エリアへ搬送'
    s.goto(1.6, 1.55); s.goto(1.6, -0.3); s.goto(1.4, -1.3)
    s.approach(SLOTS[1], 0.28); s.unload(1); s.reverse(0.35)
    # --- 荷物C
    s.caption = '荷物Cへ移動'
    s.goto(1.5, -0.3); s.goto(1.5, 0.3); s.approach(CUBES0[2], 0.25)
    s.load(2)
    s.caption = '後退して向きを変える'; s.reverse(0.35)
    s.caption = '荷物Cを倉庫エリアへ搬送'
    s.goto(1.5, -0.3); s.goto(1.3, -1.4)
    s.approach(SLOTS[2], 0.28); s.unload(2); s.reverse(0.35)
    # --- 帰還
    s.caption = 'ホームへ帰還'
    s.goto(1.2, -1.0); s.goto(0.0, -1.0); s.goto(-2.0, -1.0); s.goto(-2.0, -2.0); s.turn_to(0.0)
    s.caption = 'タスク完了'
    s.say("[task] all cargo delivered. returned home")
    tend = s.t
    s.run(7.0)
    return s, tend


def check_collisions(s):
    worst = 9
    for f in s.frames:
        c, sn = math.cos(f['th']), math.sin(f['th'])
        pts = [(f['x'] + lx * c - ly * sn, f['y'] + lx * sn + ly * c)
               for lx in (-0.09, 0.0, 0.09, 0.185) for ly in (-0.105, 0, 0.105)]
        rects = [(o[0], o[1], o[2] / 2, o[3] / 2) for o in OBST]
        for (p, y) in f['cubes']:
            if p[2] < 0.06: rects.append((p[0], p[1], 0.05, 0.05))
        for (px, py) in pts:
            for (cx, cy, hx, hy) in rects:
                d = max(abs(px - cx) - hx, abs(py - cy) - hy)
                worst = min(worst, d)
            worst = min(worst, ARENA - 0.025 - max(abs(px), abs(py)))
    return worst

# ---------------------------------------------------------------- geometry
L = np.array([-0.45, -0.35, 1.0]); L /= np.linalg.norm(L)

def face(pts, col, center, bias=0.0, cull=True):
    pts = np.array(pts, float)
    n = np.cross(pts[1] - pts[0], pts[2] - pts[0])
    nn = np.linalg.norm(n)
    if nn < 1e-12: return None
    n /= nn
    if np.dot(n, pts.mean(0) - center) < 0: n = -n
    return ('poly', pts, col, n, bias, cull)

def box(c, s, yaw, col):
    c = np.array(c, float); hx, hy, hz = s[0] / 2, s[1] / 2, s[2] / 2
    cs, sn = math.cos(yaw), math.sin(yaw)
    def P(a, b, z): return c + np.array([a * cs - b * sn, a * sn + b * cs, z])
    v = [P(a, b, z) for a in (-hx, hx) for b in (-hy, hy) for z in (-hz, hz)]
    idx = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [f for f in (face([v[i] for i in q], col, c) for q in idx) if f]

def prism(c, ax, e1, e2, r, hl, n, phase, col, capcol=None):
    c, ax, e1, e2 = map(lambda a: np.array(a, float), (c, ax, e1, e2))
    ang = [phase + 2 * math.pi * k / n for k in range(n)]
    ring = [c + r * (math.cos(a) * e1 + math.sin(a) * e2) for a in ang]
    out = []
    for k in range(n):
        a, b = ring[k], ring[(k + 1) % n]
        f = face([a - hl * ax, b - hl * ax, b + hl * ax, a + hl * ax], col, c)
        if f: out.append(f)
    for sgn in (-1, 1):
        f = face([p + sgn * hl * ax for p in ring], capcol or col, c)
        if f: out.append(f)
    return out

def robot_items(x, y, th, aL, aR):
    cs, sn = math.cos(th), math.sin(th)
    f = np.array([cs, sn, 0]); l = np.array([-sn, cs, 0]); z = np.array([0, 0, 1.0])
    def W_(lx, ly, lz): return np.array([x, y, 0]) + lx * f + ly * l + lz * z
    it = []
    it += box(W_(0.045, 0, 0.055), (0.27, 0.16, 0.05), th, (72, 88, 110))
    it += box(W_(0.045, 0, 0.088), (0.25, 0.145, 0.014), th, (44, 48, 58))
    it += box(W_(0.183, 0, 0.05), (0.012, 0.15, 0.03), th, (240, 190, 40))
    it += prism(W_(-0.045, 0, 0.11), z, f, l, 0.032, 0.018, 12, 0, (34, 34, 40), (70, 72, 82))
    for side, ang in ((1, aL), (-1, aR)):
        c = W_(0, side * 0.095, 0.033)
        it += prism(c, l, f, z, 0.033, 0.009, 16, 0, (205, 38, 38), (170, 30, 30))
        oc = c + side * 0.0095 * l
        a = -ang  # 前進で前方下向きに回る見た目
        d1 = math.cos(a) * f + math.sin(a) * z
        d2 = -math.sin(a) * f + math.cos(a) * z
        q = [oc - 0.006 * d2, oc + 0.028 * d1 - 0.006 * d2, oc + 0.028 * d1 + 0.006 * d2, oc + 0.006 * d2]
        fc = face(q, (245, 245, 245), c, bias=-0.004)
        if fc: it.append(fc)
    it.append(('sphere', W_(0.15, 0, 0.014), 0.014, (40, 200, 70)))
    return it

def hull(pts):
    pts = sorted(set(pts))
    if len(pts) < 3: return pts
    def cr(o, a, b): return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cr(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cr(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]

def shadow_of(items):
    pts = []
    for it in items:
        P = it[1] if it[0] == 'poly' else np.array([it[1]])
        for p in P:
            q = p - L * (p[2] / L[2])
            pts.append((round(q[0], 4), round(q[1], 4)))
    return hull(pts)

# static scene
STATIC = []
STATIC_SH = []
STATIC_G, STATIC_C = [], []
for (cx, cy, sx, sy, h, col) in OBST:
    b = box((cx, cy, h / 2), (sx, sy, h), 0, col); STATIC += b; STATIC_SH.append(shadow_of(b))
    STATIC_G.append(b); STATIC_C.append((cx, cy, h / 2))
WALL = (196, 200, 208)
for k in range(10):
    a = -ARENA + 0.5 * k + 0.25
    for (c, s) in (((a, ARENA, 0.1), (0.5, 0.05, 0.2)), ((a, -ARENA, 0.1), (0.5, 0.05, 0.2)),
                   ((ARENA, a, 0.1), (0.05, 0.5, 0.2)), ((-ARENA, a, 0.1), (0.05, 0.5, 0.2))):
        b = box(c, s, 0, WALL); STATIC += b; STATIC_SH.append(shadow_of(b))
        STATIC_G.append(b); STATIC_C.append(c)

# ---------------------------------------------------------------- camera / render
class Cam:
    def __init__(s, eye, tgt, fov=58):
        s.eye = np.array(eye, float)
        f = np.array(tgt, float) - s.eye; f /= np.linalg.norm(f)
        r = np.cross(f, [0, 0, 1]); r /= np.linalg.norm(r)
        u = np.cross(r, f)
        s.M = np.stack([r, u, f])
        s.F = (SH / 2) / math.tan(math.radians(fov) / 2)
    def tc(s, P): return (np.asarray(P, float) - s.eye) @ s.M.T
    def proj(s, C): return [(SW / 2 + s.F * p[0] / p[2], SH / 2 - s.F * p[1] / p[2]) for p in C]

NEAR = 0.05
def clip(C):
    out = []
    n = len(C)
    for i in range(n):
        a, b = C[i], C[(i + 1) % n]
        ia, ib = a[2] >= NEAR, b[2] >= NEAR
        if ia: out.append(a)
        if ia != ib:
            t = (NEAR - a[2]) / (b[2] - a[2]); out.append(a + (b - a) * t)
    return out

def poly3(dr, cam, P, col, outline=None, width=1):
    C = clip(list(cam.tc(P)))
    if len(C) >= 3:
        dr.polygon(cam.proj(C), fill=col, outline=outline, width=width)

def line3(dr, cam, a, b, col, w):
    A, B = cam.tc([a, b])
    if A[2] < NEAR and B[2] < NEAR: return
    if A[2] < NEAR: A = A + (B - A) * (NEAR - A[2]) / (B[2] - A[2])
    elif B[2] < NEAR: B = B + (A - B) * (NEAR - B[2]) / (A[2] - B[2])
    dr.line(cam.proj([A, B]), fill=col, width=w)

def shade(col, n):
    k = 0.5 + 0.5 * max(0.0, float(np.dot(n, L)))
    return tuple(int(min(255, c * k + 18)) for c in col)

SKY = None
def sky():
    global SKY
    if SKY is None:
        g = np.linspace(0, 1, SH)[:, None]
        top, bot = np.array([150, 166, 186]), np.array([214, 220, 228])
        arr = (top * (1 - g) + bot * g).astype(np.uint8)
        SKY = Image.fromarray(np.repeat(arr[:, None, :], SW, 1).reshape(SH, SW, 3), 'RGB')
    return SKY.copy()

def render3d(cam, fr, trail):
    img = sky(); dr = ImageDraw.Draw(img)
    G = 30
    poly3(dr, cam, [(-G, -G, 0), (G, -G, 0), (G, G, 0), (-G, G, 0)], (176, 179, 184))
    for k in range(-12, 13):
        line3(dr, cam, (k, -12, 0), (k, 12, 0), (162, 165, 171), 2)
        line3(dr, cam, (-12, k, 0), (12, k, 0), (162, 165, 171), 2)
    A = ARENA
    poly3(dr, cam, [(-A, -A, 0.0005), (A, -A, 0.0005), (A, A, 0.0005), (-A, A, 0.0005)], (208, 210, 214))
    for k in range(11):
        a = -A + 0.5 * k
        line3(dr, cam, (a, -A, 0.001), (a, A, 0.001), (186, 189, 195), 2)
        line3(dr, cam, (-A, a, 0.001), (A, a, 0.001), (186, 189, 195), 2)
    zx, zy, zw, zh = ZONE
    poly3(dr, cam, [(zx - zw / 2, zy - zh / 2, .002), (zx + zw / 2, zy - zh / 2, .002),
                    (zx + zw / 2, zy + zh / 2, .002), (zx - zw / 2, zy + zh / 2, .002)], (118, 196, 132),
          outline=(60, 140, 80), width=4)
    for (cx, cy) in CUBES0:
        poly3(dr, cam, [(cx - .14, cy - .14, .002), (cx + .14, cy - .14, .002), (cx + .14, cy + .14, .002),
                        (cx - .14, cy + .14, .002)], (242, 206, 92), outline=(196, 150, 40), width=3)
    # trail
    tr = trail[max(0, fr['tr'] - 2400):fr['tr']:2]
    if len(tr) > 1:
        P = np.array([(p[0], p[1], 0.003) for p in tr])
        C = cam.tc(P)
        seg = []
        for i in range(len(C)):
            if C[i][2] > NEAR: seg.append(C[i])
            else:
                if len(seg) > 1: dr.line(cam.proj(seg), fill=(60, 120, 230), width=5)
                seg = []
        if len(seg) > 1: dr.line(cam.proj(seg), fill=(60, 120, 230), width=5, joint='curve')
    # dynamic items
    rob = robot_items(fr['x'], fr['y'], fr['th'], fr['aL'], fr['aR'])
    cubes = []
    for i, (p, yaw) in enumerate(fr['cubes']):
        cubes.append(box(p, (CUBE_S,) * 3, yaw, CUBE_COL[i]))
    # shadows
    mask = Image.new('L', (SW, SH), 0); md = ImageDraw.Draw(mask)
    for sh in STATIC_SH + [shadow_of(rob)] + [shadow_of(c) for c in cubes]:
        if len(sh) >= 3: poly3(md, cam, [(a, b, 0.0025) for a, b in sh], 85)
    img.paste((20, 24, 34), (0, 0, SW, SH), mask)
    # painter's: object-level order, faces sorted inside each object
    rp = np.array([fr['x'], fr['y'], 0.05])
    groups = [(np.array(STATIC_C[k]), STATIC_G[k]) for k in range(len(STATIC_G))]
    rg = list(rob)
    for i, c in enumerate(cubes):
        p = np.array(fr['cubes'][i][0])
        if np.linalg.norm(p[:2] - rp[:2]) < 0.4 and p[2] > 0.051: rg += c
        else: groups.append((p, c))
    groups.append((rp, rg))
    drawlist = []
    for ctr0, its in groups:
        od = np.linalg.norm(ctr0 - cam.eye)
        sub = []
        for it in its:
            if it[0] == 'poly':
                _, P, col, n, bias, cull = it
                ctr = P.mean(0)
                if cull and np.dot(n, ctr - cam.eye) >= 0: continue
                sub.append((np.linalg.norm(ctr - cam.eye) + bias, it))
            else:
                sub.append((np.linalg.norm(it[1] - cam.eye) - it[2], it))
        sub.sort(key=lambda d: -d[0])
        drawlist.append((od, sub))
    drawlist.sort(key=lambda d: -d[0])
    drawlist = [x for _, sub in drawlist for x in sub]
    for _, it in drawlist:
        if it[0] == 'poly':
            _, P, col, n, *_r = it
            c = shade(col, n)
            poly3(dr, cam, P, c, outline=tuple(int(v * 0.72) for v in c), width=2)
        else:
            C = cam.tc([it[1]])[0]
            if C[2] < NEAR: continue
            (sx, sy), = cam.proj([C]); rr = cam.F * it[2] / C[2]
            dr.ellipse((sx - rr, sy - rr, sx + rr, sy + rr), fill=it[3], outline=(20, 110, 40), width=2)
            dr.ellipse((sx - rr * .45, sy - rr * .6, sx - rr * .05, sy - rr * .2), fill=(170, 250, 180))
    img = img.reduce(SS)
    # label anchors (1x coordinates)
    labels = []
    for i, (cx, cy) in enumerate(CUBES0):
        if not fr['done'][2 * i]: labels.append((f'荷物{NAMES[i]}', (cx, cy + 0.22, 0.0)))
    labels.append(('倉庫エリア', (zx, zy + zh / 2 + 0.08, 0.0)))
    lab = []
    for txt, p in labels:
        C = cam.tc([p])[0]
        if C[2] > 0.9:
            (sx, sy), = cam.proj([C]); lab.append((txt, sx / SS, sy / SS, C[2]))
    return img, lab

# ---------------------------------------------------------------- HUD
FD = '/usr/share/fonts/opentype/noto/'
def F(w, s): return ImageFont.truetype(FD + f'NotoSansCJK-{w}.ttc', s, index=0)
FM = lambda s: ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', s)
fB30, fB22, fB18, fB15, fR14, fR13, fB48, fR20 = F('Bold', 30), F('Bold', 22), F('Bold', 18), F('Bold', 15), F('Regular', 14), F('Regular', 13), F('Bold', 48), F('Regular', 20)
fM12, fM13 = FM(12), FM(13)
PANEL = (18, 22, 32, 196)
ACC = (88, 166, 255)

def rr(d, box, fill, r=10, outline=None, w=1): d.rounded_rectangle(box, r, fill=fill, outline=outline, width=w)

MINI = 232
def mm(x, y, ox, oy):
    s = MINI / (2 * ARENA + 0.2)
    return ox + MINI / 2 + x * s, oy + MINI / 2 - y * s

def hud(img, fr, trail, lab, gt, alpha_ui):
    ov = Image.new('RGBA', (W, H), (0, 0, 0, 0)); d = ImageDraw.Draw(ov)
    a = lambda c: c[:3] + (int((c[3] if len(c) > 3 else 255) * alpha_ui),)
    # 3D labels
    for txt, sx, sy, z in lab:
        tw = d.textlength(txt, font=fB15)
        rr(d, (sx - tw / 2 - 8, sy - 13, sx + tw / 2 + 8, sy + 13), a((18, 22, 32, 170)), 8)
        d.text((sx, sy), txt, font=fB15, fill=a((255, 255, 255, 255)), anchor='mm')
    # title panel
    rr(d, (16, 16, 420, 168), a(PANEL), 12)
    d.text((32, 26), '搬送タスク：荷物3個を倉庫エリアへ', font=fB18, fill=a((255, 255, 255, 255)))
    d.text((32, 52), 'teleop_node → /cmd_vel → bridge → DiffDrive', font=fR13, fill=a((170, 182, 200, 255)))
    d.text((404, 28), f'{gt:5.1f} s', font=fB15, fill=a(ACC + (255,)), anchor='ra')
    for i in range(3):
        y0 = 80 + i * 28
        d.ellipse((32, y0 + 2, 50, y0 + 20), fill=a(CUBE_COL[i] + (255,)))
        d.text((41, y0 + 11), NAMES[i], font=fB15 if False else fR13, fill=a((255, 255, 255, 255)), anchor='mm')
        for j, lbl in enumerate(('回収', '搬入')):
            ok = fr['done'][2 * i + j]
            bx = 64 + j * 120
            rr(d, (bx, y0, bx + 108, y0 + 22), a((52, 160, 96, 235) if ok else (48, 54, 68, 220)), 6)
            if ok:
                d.line((bx + 9, y0 + 11, bx + 14, y0 + 16, bx + 23, y0 + 6), fill=a((255, 255, 255, 255)), width=3)
            d.text((bx + 64, y0 + 11), lbl, font=fR13, fill=a((255, 255, 255, 255) if ok else (150, 160, 176, 255)), anchor='mm')
        if fr['done'][2 * i + 1]:
            d.text((318, y0 + 11), '完了', font=fB15, fill=a((120, 220, 150, 255)), anchor='lm')
    # caption
    cap = fr['caption']
    tw = d.textlength(cap, font=fB22)
    cx = 640 + (420 - (W - 252)) / 2 + (W - 252 - 420) / 2 - 0  # center of free area
    cx = (420 + W - 252) / 2
    rr(d, (cx - tw / 2 - 22, 18, cx + tw / 2 + 22, 62), a((18, 22, 32, 210)), 22)
    d.text((cx, 40), cap, font=fB22, fill=a((255, 255, 255, 255)), anchor='mm')
    # minimap
    ox, oy = W - 16 - MINI, 16
    rr(d, (ox - 8, oy - 8, ox + MINI + 8, oy + MINI + 34), a(PANEL), 12)
    p0 = mm(-ARENA, ARENA, ox, oy); p1 = mm(ARENA, -ARENA, ox, oy)
    d.rectangle((*p0, *p1), fill=a((60, 66, 80, 255)), outline=a((150, 158, 172, 255)), width=2)
    zx, zy, zw, zh = ZONE
    d.rectangle((*mm(zx - zw / 2, zy + zh / 2, ox, oy), *mm(zx + zw / 2, zy - zh / 2, ox, oy)), fill=a((70, 150, 92, 255)))
    for (cx_, cy_) in CUBES0:
        d.rectangle((*mm(cx_ - .14, cy_ + .14, ox, oy), *mm(cx_ + .14, cy_ - .14, ox, oy)), outline=a((230, 190, 70, 255)), width=1)
    for (cx_, cy_, sx, sy, h, col) in OBST:
        d.rectangle((*mm(cx_ - sx / 2, cy_ + sy / 2, ox, oy), *mm(cx_ + sx / 2, cy_ - sy / 2, ox, oy)), fill=a(col + (255,)))
    tr = trail[:fr['tr']:3]
    if len(tr) > 1: d.line([mm(p[0], p[1], ox, oy) for p in tr], fill=a((96, 160, 255, 255)), width=2)
    for i, (p, y) in enumerate(fr['cubes']):
        q = mm(p[0], p[1], ox, oy)
        d.rectangle((q[0] - 4, q[1] - 4, q[0] + 4, q[1] + 4), fill=a(CUBE_COL[i] + (255,)), outline=a((255, 255, 255, 255)))
    rx, ry = mm(fr['x'], fr['y'], ox, oy); th = fr['th']; s = 11
    tri = [(rx + s * math.cos(th), ry - s * math.sin(th)),
           (rx + s * .7 * math.cos(th + 2.5), ry - s * .7 * math.sin(th + 2.5)),
           (rx + s * .7 * math.cos(th - 2.5), ry - s * .7 * math.sin(th - 2.5))]
    d.polygon(tri, fill=a((255, 255, 255, 255)), outline=a((30, 30, 30, 255)))
    d.text((ox + 4, oy + MINI + 6), f'走行距離 {fr["dist"]:5.1f} m', font=fR13, fill=a((200, 210, 224, 255)))
    # keys + cmd
    bx, by = 16, H - 200
    rr(d, (bx, by, bx + 404, H - 16), a(PANEL), 12)
    d.text((bx + 16, by + 10), 'キー入力（最後の指令を保持）', font=fR13, fill=a((170, 182, 200, 255)))
    kk = fr['key']; fresh = fr['t'] - fr['keyt'] < 0.25
    def key(k, lbl, x, y, w=46):
        on = kk == k
        col = (ACC + (255,)) if on else (50, 56, 72, 235)
        if on and fresh: col = (170, 215, 255, 255)
        rr(d, (x, y, x + w, y + 40), a(col), 7, outline=a((120, 130, 150, 255)), w=1)
        d.text((x + w / 2, y + 20), lbl, font=fB18 if len(lbl) == 1 else fR13, fill=a((255, 255, 255, 255) if not on else (10, 20, 40, 255)), anchor='mm')
    kx, ky = bx + 18, by + 36
    key('w', 'W', kx + 52, ky); key('a', 'A', kx, ky + 46); key('s', 'S', kx + 52, ky + 46); key('d', 'D', kx + 104, ky + 46)
    key(' ', 'SPACE', kx, ky + 92, 150)
    tx = bx + 190
    d.text((tx, by + 34), '/cmd_vel', font=fB15, fill=a((255, 255, 255, 255)))
    def bar(y, lbl, val, cmd, unit):
        d.text((tx, y), lbl, font=fM12, fill=a((170, 182, 200, 255)))
        d.text((tx + 196, y), f'{val:+.2f} {unit}', font=fM12, fill=a((255, 255, 255, 255)), anchor='ra')
        x0, x1, yy = tx, tx + 196, y + 20
        d.rectangle((x0, yy, x1, yy + 6), fill=a((48, 54, 68, 255)))
        m = (x0 + x1) / 2
        d.rectangle((min(m, m + val * 98), yy, max(m, m + val * 98), yy + 6), fill=a(ACC + (255,)))
        cx_ = m + cmd * 98
        d.line((cx_, yy - 3, cx_, yy + 9), fill=a((255, 210, 80, 255)), width=2)
    bar(by + 60, 'linear.x', fr['v'], fr['vc'], 'm/s')
    bar(by + 98, 'angular.z', fr['w'], fr['wc'], 'rad/s')
    d.text((tx, by + 136), f"wheel L {fr['wl']:+6.1f}", font=fM12, fill=a((170, 182, 200, 255)))
    d.text((tx, by + 154), f"wheel R {fr['wr']:+6.1f} rad/s", font=fM12, fill=a((170, 182, 200, 255)))
    # log
    lx, ly = W - 476, H - 172
    rr(d, (lx, ly, W - 16, H - 16), a((10, 12, 18, 214)), 12)
    d.text((lx + 14, ly + 8), 'ros2 log', font=fR13, fill=a((130, 140, 160, 255)))
    for i, line in enumerate(fr['log']):
        c = (130, 220, 150) if '[task]' in line else (200, 208, 220)
        if line.startswith('$'): c = (255, 210, 120)
        d.text((lx + 14, ly + 30 + i * 18), line[:62], font=fM12, fill=a(c + (255,)))
    img = img.convert('RGBA'); img.alpha_composite(ov)
    return img

def card(img, title, lines, alpha):
    if alpha <= 0: return img
    ov = Image.new('RGBA', (W, H), (8, 10, 16, int(150 * alpha))); d = ImageDraw.Draw(ov)
    al = int(255 * alpha)
    d.text((W / 2, H / 2 - 50), title, font=fB48, fill=(255, 255, 255, al), anchor='mm')
    for i, l in enumerate(lines):
        d.text((W / 2, H / 2 + 20 + i * 34), l, font=fR20, fill=(200, 212, 230, al), anchor='mm')
    img.alpha_composite(ov); return img

# ---------------------------------------------------------------- frame driver
def ease(u): u = min(1, max(0, u)); return u * u * (3 - 2 * u)

def cameras(frames, tend):
    out = []; cy = frames[0]['th']; k = 1 - math.exp(-1 / FPS / 0.45)
    for f in frames:
        cy = wrap(cy + wrap(f['th'] - cy) * k)
        t = f['t']
        dx, dy = math.cos(cy), math.sin(cy)
        ce = np.array([f['x'] - 1.25 * dx, f['y'] - 1.25 * dy, 0.72])
        ct = np.array([f['x'] + 0.4 * dx, f['y'] + 0.4 * dy, 0.04])
        ang = math.radians(-118 + 6 * t / 10)
        we = np.array([5.6 * math.cos(ang), 5.6 * math.sin(ang), 4.6]); wt = np.array([0.0, 0.2, 0.0])
        wb = 1 - ease((t - 4.0) / 2.2)
        wb = max(wb, ease((t - tend - 1.2) / 2.5))
        out.append(Cam(ce * (1 - wb) + we * wb, ct * (1 - wb) + wt * wb, 58 + 0 * wb))
    return out

def render_range(a, b, path, data):
    frames, trail, tend = data
    cams = cameras(frames, tend)
    p = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                          '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-preset', 'medium',
                          '-crf', '20', '-pix_fmt', 'yuv420p', path], stdin=subprocess.PIPE)
    total = frames[-1]['t']
    for i in range(a, b):
        fr = frames[i]; t = fr['t']
        img, lab = render3d(cams[i], fr, trail)
        ui = ease((t - 4.2) / 1.2)
        img = hud(img, fr, trail, lab, t, ui)
        intro = 1 - ease((t - 3.6) / 0.8)
        img = card(img, '差動二輪ロボット 搬送タスク',
                   ['車輪半径 0.033 m ／ 車輪間隔 0.19 m（記事の DiffDrive 設定）',
                    'teleop_node のキー指令（linear.x ±1.0 ／ angular.z ±1.0）で走行'], intro)
        endc = ease((t - tend - 2.6) / 1.0)
        img = card(img, 'タスク完了',
                   [f'荷物 3 / 3 搬入 ・ 走行距離 {frames[-1]["dist"]:.1f} m ・ 所要時間 {tend - 6.0:.1f} 秒'], endc)
        p.stdin.write(img.convert('RGB').tobytes())
    p.stdin.close(); p.wait()

if __name__ == '__main__':
    mode = sys.argv[1]
    if mode == 'sim':
        s, tend = simulate()
        print('frames', len(s.frames), 'dur', s.t, 'tend', tend, 'dist', s.dist, 'clear', check_collisions(s))
        pickle.dump((s.frames, s.trail, tend), open(sys.argv[2], 'wb'))
    elif mode == 'still':
        data = pickle.load(open(sys.argv[2], 'rb'))
        frames, trail, tend = data
        cams = cameras(frames, tend)
        for i in map(int, sys.argv[4:]):
            img, lab = render3d(cams[i], frames[i], trail)
            img = hud(img, frames[i], trail, lab, frames[i]['t'], ease((frames[i]['t'] - 4.2) / 1.2))
            img.convert('RGB').save(f'{sys.argv[3]}/still_{i}.png')
    elif mode == 'render':
        data = pickle.load(open(sys.argv[2], 'rb'))
        render_range(int(sys.argv[3]), int(sys.argv[4]), sys.argv[5], data)
