#!/usr/bin/env python3
import sys, math, argparse, threading, queue, time
import numpy as np
import tkinter as tk
from pyproj import Transformer

WGS84_TO_ECEF = Transformer.from_crs("epsg:4979", "epsg:4978", always_xy=True)
ECEF_TO_WGS84 = Transformer.from_crs("epsg:4978", "epsg:4979", always_xy=True)

def geodetic_to_ecef(lat, lon, h):
    x,y,z = WGS84_TO_ECEF.transform(lon, lat, h)
    return np.array([x,y,z], float)

def ecef_to_geodetic(x,y,z):
    lon,lat,h = ECEF_TO_WGS84.transform(x,y,z)
    return lat,lon,h

class SimpleDR:
    def __init__(self, lat0, lon0, alt0):
        self.lat0, self.lon0, self.alt0 = lat0, lon0, alt0
        self.pos_ecef = geodetic_to_ecef(lat0, lon0, alt0)
        self.vel_ecef = np.zeros(3)
        self.att = np.eye(3)
        self.t_prev = None
        self.path = [(lat0, lon0)]

    def step(self, t, acc, gyro):
        if self.t_prev is None:
            self.t_prev = t
            return self.path[-1]

        dt = t - self.t_prev
        if dt <= 0 or dt > 0.5:
            dt = 0.02
        self.t_prev = t

        # orientation update (small-angle)
        gx,gy,gz = gyro
        wx = np.array([[0, -gz, gy],[gz, 0, -gx],[-gy, gx, 0]])
        self.att = self.att @ (np.eye(3) + wx*dt)

        # accel transform
        a_e = self.att @ np.array(acc)

        # gravity approx
        r = np.linalg.norm(self.pos_ecef)
        g_vec = -(self.pos_ecef/r)*9.80665

        a_lin = a_e + g_vec

        self.vel_ecef += a_lin * dt
        self.pos_ecef += self.vel_ecef * dt

        lat,lon,_ = ecef_to_geodetic(*self.pos_ecef)
        self.path.append((lat,lon))
        return lat,lon

class GUI:
    def __init__(self, dr, q):
        self.dr = dr
        self.q = q
        self.root = tk.Tk()
        self.root.title("Dead Reckoning – Live Path")
        self.canvas = tk.Canvas(self.root, width=800, height=600, bg='white')
        self.canvas.pack()
        self.origin = None
        self.root.after(30, self.loop)
        self.root.mainloop()

    def to_canvas(self, lat, lon):
        if self.origin is None:
            w = 800; h = 600
            self.origin = (w//2, h//2)

        R = 6378137
        dx = math.radians(lon - self.dr.lon0) * R * math.cos(math.radians(lat))
        dy = math.radians(lat - self.dr.lat0) * R
        x = self.origin[0] + dx/5
        y = self.origin[1] - dy/5
        return x,y

    def redraw(self):
        self.canvas.delete("all")
        pts = self.dr.path
        if len(pts) < 2: return

        coords = [self.to_canvas(lat,lon) for lat,lon in pts]

        for i in range(len(coords)-1):
            self.canvas.create_line(coords[i][0], coords[i][1],
                                    coords[i+1][0], coords[i+1][1],
                                    fill='blue', width=2)
        # last point
        x,y = coords[-1]
        self.canvas.create_oval(x-5,y-5,x+5,y+5,fill='red')

    def loop(self):
        updated = False
        while not self.q.empty():
            line = self.q.get()
            parts = line.split(',')
            if len(parts) != 7: continue
            t = float(parts[0])
            ax,ay,az = float(parts[1]), float(parts[2]), float(parts[3])
            gx,gy,gz = float(parts[4]), float(parts[5]), float(parts[6])
            self.dr.step(t, (ax,ay,az), (gx,gy,gz))
            updated = True

        if updated:
            self.redraw()

        self.root.after(30, self.loop)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lat", type=float, required=True)
    parser.add_argument("--lon", type=float, required=True)
    args = parser.parse_args()

    dr = SimpleDR(args.lat, args.lon, 0)
    q = queue.Queue()

    def read_stdin():
        for line in sys.stdin:
            q.put(line.strip())

    threading.Thread(target=read_stdin, daemon=True).start()

    GUI(dr, q)

if __name__ == "__main__":
    main()
