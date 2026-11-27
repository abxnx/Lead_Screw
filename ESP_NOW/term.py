#!/usr/bin/env python3
"""
term.py — ใช้แทน Serial Monitor: ส่ง/รับที่ serial port เดียวกัน + บันทึก CSV + พล็อตสด

  python term.py                       ไม่ใส่ port = ลิสต์ให้เลือก
  python term.py COM5                  เปิด terminal + กราฟ
  python term.py COM5 --no-plot        terminal อย่างเดียว
  python term.py /dev/ttyACM0 --show-data   พิมพ์บรรทัด D ด้วย (เยอะมาก 100 บรรทัด/วินาที)

พิมพ์คำสั่งใน terminal แล้ว Enter: G 25, M -5, V 10, A 50, H, X, Z, S
  /quit  ออก
  /log   เริ่มไฟล์ CSV ใหม่ (ใช้แยกไฟล์ต่อการทดสอบ เช่น T3 แต่ละจุด)

ต้องรันใน Terminal จริง (ไม่ใช่ปุ่ม Run/Code Runner เพราะพิมพ์โต้ตอบไม่ได้)
ต้องปิด Serial Monitor ของ Arduino ก่อน (port เปิดได้ทีละโปรแกรม)
pip install pyserial matplotlib
"""
import argparse
import collections
import csv
import sys
import threading
import time

import serial

FL_RUNNING, FL_HOMED = 0x01, 0x02

ap = argparse.ArgumentParser()
ap.add_argument('port', nargs='?', help='ไม่ใส่ = ลิสต์ port ให้เลือก')
ap.add_argument('--baud', type=int, default=115200)
ap.add_argument('--no-plot', action='store_true')
ap.add_argument('--show-data', action='store_true', help='พิมพ์บรรทัด telemetry ออกจอด้วย')
ap.add_argument('--window', type=float, default=10.0, help='ช่วงเวลาที่โชว์บนกราฟ (s)')
args = ap.parse_args()

def choose_port():
    from serial.tools import list_ports
    ports = sorted(list_ports.comports(), key=lambda p: p.device)
    # Mac: ใช้ cu.* (tty.* จะค้างรอสัญญาณ DCD) และตัด bluetooth/debug ทิ้ง
    ports = [p for p in ports if not any(k in p.device for k in ('Bluetooth', 'debug-console', 'tty.'))]
    if not ports:
        sys.exit('ไม่พบ serial port — เสียบ master แล้วหรือยัง?')
    if len(ports) == 1:
        print(f'[term] ใช้ {ports[0].device} ({ports[0].description})')
        return ports[0].device
    for i, p in enumerate(ports):
        print(f'  [{i}] {p.device}  {p.description}')
    while True:
        try:
            return ports[int(input('เลือกเลข port ของ master: '))].device
        except (ValueError, IndexError):
            print('เลขไม่ถูกต้อง')


port = args.port or choose_port()
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, args.baud, 0.05
ser.dtr = False          # FT231X + วงจร auto-reset: อย่าให้การเปิด port ไป reset / ดันบอร์ดเข้าโหมด download
ser.rts = False
ser.open()
t_open = time.time()
rx_bytes = 0
lock = threading.Lock()          # กัน thread อ่าน/วาดกราฟชนกันตอนแตะ buffer
running = True

N = int(args.window * 100 * 1.2)
T, TGT, CMD, ENC = (collections.deque(maxlen=N) for _ in range(4))
st = {'t0': None, 'last_seq': None, 'rx': 0, 'lost': 0, 'flags': 0, 'link': '-'}

log_file = log = None


def new_log():
    global log_file, log
    if log_file:
        log_file.close()
    name = time.strftime('log_%Y%m%d_%H%M%S.csv')
    log_file = open(name, 'w', newline='')
    log = csv.writer(log_file)
    log.writerow(['t_ms', 'seq', 'target_mm', 'cmd_mm', 'enc_mm', 'flags'])
    print(f'[term] logging to {name}')


def handle_line(text):
    p = text.split(',')
    if p[0] != 'D' or len(p) != 7:
        if text:
            print(text)                       # A / E / I / ข้อความอื่น
            if text.startswith('E,LINK_DOWN'):
                st['link'] = 'DOWN'
            elif text.startswith('I,link up'):
                st['link'] = 'up'
        return
    try:
        t_ms, seq = int(p[1]), int(p[2])
        tgt, cmd, enc = float(p[3]), float(p[4]), float(p[5])
        flags = int(p[6])
    except ValueError:
        return
    if args.show_data:
        print(text)
    with lock:
        last = st['last_seq']
        if last is not None:
            gap = (seq - last - 1) & 0xFFFFFFFF
            if gap > 1000:                    # slave รีบูต
                print('[term] slave restarted — reset')
                st['t0'] = None
                for d in (T, TGT, CMD, ENC):
                    d.clear()
            else:
                st['lost'] += gap
        st['last_seq'] = seq
        if st['t0'] is None:
            st['t0'] = t_ms
        st['rx'] += 1
        st['flags'] = flags
        st['link'] = 'up'
        T.append((t_ms - st['t0']) / 1000.0)
        TGT.append(tgt)
        CMD.append(cmd)
        ENC.append(enc)
    log.writerow(p[1:])


def reader():
    """อ่าน serial ตลอดเวลา (thread แยก ข้อมูลไม่หายแม้กราฟวาดช้า)"""
    global rx_bytes
    buf = b''
    last_flush = time.time()
    warned = False
    while running:
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException as e:
            print(f'[term] serial error: {e}')
            break
        if chunk:
            rx_bytes += len(chunk)
            buf += chunk
            *lines, buf = buf.split(b'\n')
            for ln in lines:
                handle_line(ln.decode(errors='replace').strip())
        if not rx_bytes and not warned and time.time() - t_open > 3:
            warned = True
            print('[term] 3 วินาทีแล้วยังไม่ได้ข้อมูลอะไรจาก port เลย → กดปุ่ม EN/RESET บนบอร์ด master '
                  'ควรเห็น "I,master MAC ..."  (ถ้ายังเงียบ: เช็ค baud ให้ตรง master = 921600 / เช็คว่าเป็น port ของ master)')
        if time.time() - last_flush > 0.5:
            log_file.flush()
            last_flush = time.time()


def writer():
    """อ่านคีย์บอร์ดแล้วส่งลง serial"""
    global running
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        if line == '/quit':
            break
        if line == '/log':
            new_log()
            continue
        ser.write((line + '\n').encode())
    running = False


new_log()
threading.Thread(target=reader, daemon=True).start()
print('[term] พิมพ์คำสั่งแล้ว Enter (G 25, H, X, S, ...)  /log = ไฟล์ใหม่  /quit = ออก')

try:
    if args.no_plot:
        writer()
    else:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        threading.Thread(target=writer, daemon=True).start()
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6),
                                       gridspec_kw={'height_ratios': [3, 1]})
        l_tgt, = ax1.plot([], [], '--', color='gray', lw=1, label='target')
        l_cmd, = ax1.plot([], [], lw=1.5, label='commanded (steps)')
        l_enc, = ax1.plot([], [], lw=1.5, label='actual (encoder)')
        ax1.set_ylabel('position (mm)')
        ax1.grid(alpha=0.3)
        ax1.legend(loc='upper left')
        l_err, = ax2.plot([], [], color='tab:red', lw=1)
        ax2.set_ylabel('enc − cmd (mm)')
        ax2.set_xlabel('time (s)')
        ax2.grid(alpha=0.3)

        def update(_):
            if not running:
                plt.close(fig)
                return ()
            with lock:
                t, tg, c, e = list(T), list(TGT), list(CMD), list(ENC)
                rx, lost, flags, link = st['rx'], st['lost'], st['flags'], st['link']
            if t:
                err = [b - a for a, b in zip(c, e)]
                l_tgt.set_data(t, tg)
                l_cmd.set_data(t, c)
                l_enc.set_data(t, e)
                l_err.set_data(t, err)
                lo = max(0.0, t[-1] - args.window)
                ax1.set_xlim(lo, lo + args.window)
                ax1.relim()
                ax1.autoscale_view(scalex=False)
                lim = max(0.1, max(abs(x) for x in err) * 1.2)
                ax2.set_ylim(-lim, lim)
            total = rx + lost
            pct = 100.0 * lost / total if total else 0.0
            fig.suptitle(f"link={link} | {'RUNNING' if flags & FL_RUNNING else 'stopped'} | "
                         f"{'homed' if flags & FL_HOMED else 'NOT homed'} | "
                         f"rx={rx} lost={lost} ({pct:.2f}%)")
            return ()

        ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
        plt.show()
finally:
    running = False
    time.sleep(0.1)
    log_file.close()
    ser.close()
    print('[term] bye')