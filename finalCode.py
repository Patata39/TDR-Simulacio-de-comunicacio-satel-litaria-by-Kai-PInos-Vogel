"""
Servidor Python — ESP32 Control Remot
======================================
"""

import tkinter as tk
from tkinter import ttk
from tkinter import filedialog

import csv
import json as _json_mod
import logging
import socket
import sys
import re
import threading
import time
from datetime import datetime
from json import loads, dumps
from pathlib import Path
from typing import Optional
import http.server
from urllib.parse import urlparse, unquote

import cv2
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image, ImageTk

# ─────────────────────────────────────────────────────────────────
# CONFIGURACIÓ DE RUTES I LOGS
# ─────────────────────────────────────────────────────────────────
SERVER_HOST  = "0.0.0.0"
SERVER_PORT  = 5000
if getattr(sys, "frozen", False):
    RUTA_BASE = Path(sys.executable).parent
else:
    RUTA_BASE = Path(__file__).parent
RUTA_SAVE    = RUTA_BASE / "missions"
RUTA_LOG_FPS = RUTA_BASE / "output.log"
CARPETA_MISSIONS = RUTA_BASE / "missions" 
CAM_DISCONNECT_GRACE_MS = 8000   
CARPETA_WEB = RUTA_BASE / "web"

HUSKY_ALGO_COLORS = {
    "FACE_RECOGNITION":     "#FF6B6B",
    "OBJECT_TRACKING":      "#FFD93D",
    "OBJECT_RECOGNITION":   "#6BCB77",
    "LINE_TRACKING":        "#4D96FF",
    "COLOR_RECOGNITION":    "#C77DFF",
    "TAG_RECOGNITION":      "#FF9F1C",
    "OBJECT_CLASSIFICATION":"#00C9A7",
    "UNKNOWN":              "#AAAAAA",
}

# ═════════════════════════════════════════════════════════════════
# GESTOR DE MISSIONS
# ═════════════════════════════════════════════════════════════════
class GestorMisiones:
    """
    Gestiona la creació, escriptura i tancament de carpetes de missió.

    Estructura generada:
        missions/
        └── mision_20260613_101500/
            ├── telemetria.json
            ├── huskylens.csv
            └── eventos.log
    """

    def __init__(self, carpeta_raiz: Path = CARPETA_MISSIONS):
        self.raiz = Path(carpeta_raiz)
        self.raiz.mkdir(parents=True, exist_ok=True)

        self.mision_activa: Optional[Path]     = None
        self.inicio_mision: Optional[datetime] = None

        self._f_telemetria   = None
        self._f_husky        = None
        self._csv_writer     = None
        self._logger: Optional[logging.Logger] = None
        self._primer_registro = True
        self._buffer_telemetria = []
        self._FLUSH_CADA = 10
        self._nombre_base_pendent: Optional[str] = None

    def preparar_versio_de(self, nombre_carpeta_origen: str):
        """Marca que la PRÒXIMA missió iniciada s'ha de guardar com a
        versió (_v2, _v3...) del nom base de `nombre_carpeta_origen`,
        en lloc de generar un timestamp completament nou. Es consumeix
        (es reseteja a None) en el moment de cridar iniciar_mision()."""
        self._nombre_base_pendent = self._extreure_base_nom(nombre_carpeta_origen)

    @staticmethod
    def _extreure_base_nom(nombre: str) -> str:
        """Treu qualsevol sufix _vN existent, quedant-nos amb el
        timestamp base (mision_YYYYMMDD_HHMMSS)."""
        m = re.match(r"^(mision_\d{8}_\d{6})(?:_v\d+)?$", nombre)
        return m.group(1) if m else nombre

    def iniciar_mision(self) -> Path:
        if self.mision_activa:
            self.finalizar_mision()

        ts   = datetime.now()
        base = self._nombre_base_pendent or f"mision_{ts.strftime('%Y%m%d_%H%M%S')}"
        self._nombre_base_pendent = None  # es consumeix: només afecta aquesta missió

        nombre = base
        sufijo = 1
        # Si ja existeix una carpeta amb aquest nom base (perquè s'ha
        # carregat una missió prèvia i toca versionar-la, o perquè dues
        # missions s'han iniciat dins del mateix segon), afegim
        # _v2, _v3... fins trobar un nom lliure.
        while (self.raiz / nombre).exists():
            sufijo += 1
            nombre = f"{base}_v{sufijo}"

        self.mision_activa = self.raiz / nombre
        self.mision_activa.mkdir(parents=True, exist_ok=False)
        self.inicio_mision = ts

        ruta_t = self.mision_activa / "telemetria.json"
        self._f_telemetria = open(ruta_t, "w", encoding="utf-8")
        self._f_telemetria.write("[\n")
        self._primer_registro = True

        ruta_h = self.mision_activa / "huskylens.csv"
        self._f_husky    = open(ruta_h, "w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._f_husky)
        self._csv_writer.writerow(["time_ms","algo","id","x","y","w","h","area","learned"])

        self._logger = self._crear_logger(self.mision_activa / "eventos.log")
        self._logger.info(f"Missió iniciada: {nombre}")
        return self.mision_activa

    def finalizar_mision(self):
        if not self.mision_activa:
            return
        self._flush_telemetria(forzar=True)
        if self._f_telemetria and not self._f_telemetria.closed:
            self._f_telemetria.write("\n]")
            self._f_telemetria.close()
        if self._f_husky and not self._f_husky.closed:
            self._f_husky.close()
        if self._logger:
            dur = datetime.now() - self.inicio_mision
            self._logger.info(f"Missió finalitzada. Duració: {str(dur).split('.')[0]}")
            for h in self._logger.handlers[:]:
                h.close(); self._logger.removeHandler(h)
        self.mision_activa = None; self.inicio_mision = None
        self._f_telemetria = None; self._f_husky = None
        self._csv_writer = None; self._logger = None
        self._buffer_telemetria = []

    # ── Escriptura ─────────────────────────────────────────────
    def guardar_telemetria(self, temperatura, humedad, presion,
                           roll, pitch, yaw,
                           mag_x=0.0, mag_y=0.0, mag_z=0.0,
                           rssi=None):
        if not self.mision_activa or not self._f_telemetria:
            return
        reg = {
            "ts":          datetime.now().isoformat(timespec="milliseconds"),
            "ts_ms":       self._ms_desde_inicio(),
            "temperatura": round(float(temperatura), 2),
            "humedad":     round(float(humedad),     2),
            "presion":     round(float(presion),     2),
            "roll":        round(float(roll),        2),
            "pitch":       round(float(pitch),       2),
            "yaw":         round(float(yaw),         2),
            "mag_x":       round(float(mag_x),       3),
            "mag_y":       round(float(mag_y),       3),
            "mag_z":       round(float(mag_z),       3),
        }
        if rssi is not None:
            reg["rssi"] = rssi
        self._buffer_telemetria.append(reg)
        self._flush_telemetria()

    def guardar_huskylens(self, algo: str, objetos: list):
        if not self.mision_activa or not self._csv_writer:
            return
        ts_ms = self._ms_desde_inicio()
        if not objetos:
            self._csv_writer.writerow([ts_ms, algo, "","","","","","",""])
        else:
            for obj in objetos:
                area = obj.get("w", 0) * obj.get("h", 0)
                self._csv_writer.writerow([
                    ts_ms, algo, obj.get("id",0),
                    obj.get("x",0), obj.get("y",0),
                    obj.get("w",0), obj.get("h",0),
                    area, 1 if obj.get("learned") else 0,
                ])
        if self._f_husky:
            self._f_husky.flush()

    def log_evento(self, nivel: str, mensaje: str):
        if not self._logger:
            return
        getattr(self._logger, nivel.lower(), self._logger.info)(mensaje)

    # ── Consulta ───────────────────────────────────────────────
    def listar_misiones(self) -> list:
        result = []
        for carpeta in sorted(self.raiz.iterdir(), reverse=True):
            if not carpeta.is_dir() or not carpeta.name.startswith("mision_"):
                continue
            # Extrau la part de data (primers 20 caràcters: mision_YYYYMMDD_HHMMSS)
            # i qualsevol sufix de versió (_v2, _v3, etc.) que hi hagi després
            nom = carpeta.name
            parts_nom = nom.split("_")  # ['mision', 'YYYYMMDD', 'HHMMSS', 'v2'?, ...]
            if len(parts_nom) < 3:
                continue
            data_str = f"mision_{parts_nom[1]}_{parts_nom[2]}"
            versio    = f"_{parts_nom[3]}" if len(parts_nom) > 3 else ""
            try:
                fecha = datetime.strptime(data_str, "mision_%Y%m%d_%H%M%S")
            except ValueError:
                continue
            try:
                etiqueta = fecha.strftime("%-d/%m/%Y a les %H:%Mh") + versio
            except ValueError:
                etiqueta = fecha.strftime("%d/%m/%Y a les %H:%Mh") + versio
            result.append({
                "nombre":   nom,
                "ruta":     carpeta,
                "fecha":    fecha,
                "etiqueta": etiqueta,
                "duracion": self._leer_duracion_log(carpeta),
                "n_reg":    self._contar_filas_csv(carpeta / "huskylens.csv"),
            })
        return result

    def cargar_mision(self, ruta: Path) -> dict:
        res = {"telemetria": [], "huskylens": [], "eventos": [], "duracion_ms": 0}
        f_t = ruta / "telemetria.json"
        if f_t.exists():
            try:
                with open(f_t, "r", encoding="utf-8") as f:
                    res["telemetria"] = _json_mod.load(f)
            except Exception:
                pass
        f_h = ruta / "huskylens.csv"
        if f_h.exists():
            with open(f_h, "r", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        row["time_ms"] = int(row["time_ms"] or 0)
                    except Exception:
                        row["time_ms"] = 0
                    res["huskylens"].append(row)
        f_e = ruta / "eventos.log"
        if f_e.exists():
            with open(f_e, "r", encoding="utf-8") as f:
                res["eventos"] = f.read().splitlines()
        if res["telemetria"]:
            res["duracion_ms"] = res["telemetria"][-1].get("ts_ms", 0)
        return res

    # ── Helpers ────────────────────────────────────────────────
    def _ms_desde_inicio(self) -> int:
        if not self.inicio_mision:
            return 0
        return int((datetime.now() - self.inicio_mision).total_seconds() * 1000)

    def _flush_telemetria(self, forzar=False):
        if not self._buffer_telemetria:
            return
        if not forzar and len(self._buffer_telemetria) < self._FLUSH_CADA:
            return
        for reg in self._buffer_telemetria:
            sep = "" if self._primer_registro else ",\n"
            self._f_telemetria.write(sep + _json_mod.dumps(reg, ensure_ascii=False))
            self._primer_registro = False
        self._f_telemetria.flush()
        self._buffer_telemetria.clear()

    def _crear_logger(self, ruta_log: Path) -> logging.Logger:
        nom = f"mision_{ruta_log.parent.name}"
        logger = logging.getLogger(nom)
        logger.setLevel(logging.DEBUG)
        logger.propagate = False
        h = logging.FileHandler(ruta_log, encoding="utf-8")
        h.setFormatter(logging.Formatter(
            "%(asctime)s [%(levelname)s] %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
        ))
        logger.addHandler(h)
        return logger

    def _leer_duracion_log(self, carpeta: Path) -> str:
        f = carpeta / "eventos.log"
        if not f.exists(): return "—"
        with open(f, "r", encoding="utf-8") as fh:
            for ln in fh:
                if "Duració:" in ln:
                    return ln.split("Duració:")[-1].strip()
        return "—"

    def _contar_filas_csv(self, ruta: Path) -> int:
        if not ruta.exists(): return 0
        with open(ruta, "r", encoding="utf-8") as f:
            return max(0, sum(1 for _ in f) - 1)


# ── Instància global ──────────────────────────────────────────────
misiones = GestorMisiones()

# ─────────────────────────────────────────────────────────────────
# METRIQUES I RENDIMENT
# ─────────────────────────────────────────────────────────────────
temps_inici_app       = time.time()
contador_frames_cam   = 0
contador_refresc_cubo = 0
anomalies_tcp         = 0

def generar_reporte_rendimiento_final():
    temps_total = time.time() - temps_inici_app
    if temps_total <= 0: temps_total = 0.1
    fps_mitja_cam  = contador_frames_cam / temps_total
    fps_mitja_cubo = contador_refresc_cubo / temps_total
    with husky_lock:
        total_deteccions = len(husky_data["history"])
    logging.basicConfig(
        filename=RUTA_LOG_FPS, filemode='a',
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S', level=logging.INFO, encoding='utf-8'
    )
    logging.info("="*60)
    logging.info("INICI DEL REPORT DE RENDIMENT DEL SISTEMA")
    logging.info("="*60)
    logging.info(f"Temps total d'activitat: {temps_total:.2f} segons")
    logging.info(f"Rendiment Stream Vídeo: {fps_mitja_cam:.2f} FPS de mitjana")
    logging.info(f"Total de fotogrames processats: {contador_frames_cam}")
    logging.info(f"Rendiment Renderitzat 3D (Matplotlib): {fps_mitja_cubo:.2f} FPS de refresc mitjà")
    logging.info(f"Visió Artificial (HuskyLens): {total_deteccions} objectes a l'historial")
    logging.info(f"Xarxa TCP: S'han registrat {anomalies_tcp} anomalies")
    logging.info("="*60)
    logging.info("FI DEL REPORT\n")
    print(f"\n[LOG] mètriques guardades a: {RUTA_LOG_FPS.name}")

# ─────────────────────────────────────────────────────────────────
# ESTAT GLOBAL
# ─────────────────────────────────────────────────────────────────
sensor_lock = threading.Lock()
sensor_data = {
    "bme_t": None, "bme_h": None, "bme_p": None,
    "roll":  None, "pitch": None, "yaw":   None,
    "mag_x": None, "mag_y": None, "mag_z": None,
    "s_rssi": None,
    "connected": False,
}

health_lock = threading.Lock()
health_data = {
    "sensor_ip":        "0.0.0.0",
    "sensor_temp":      0.0,
    "sensor_rssi":      -100,
    "cam_ip":           "0.0.0.0",
    "cam_temp":         0.0,
    "cam_rssi":         -100,
    "history_cam_rssi": []
}

cam_lock  = threading.Lock()
cam_state = {"connected": False, "client": None}

sensor_client_socket = None
sensor_client_lock   = threading.Lock()

pending_bme_request = False
pending_imu_request = False

husky_lock = threading.Lock()
husky_data = {
    "objects":   [],
    "algo":      "UNKNOWN",
    "frame_w":   320,
    "frame_h":   240,
    "history":   [],
    "connected": False,
}
HUSKY_HISTORY_MAX = 200
HUSKY_ID_LABELS   = {1: "Persona", 2: "Ampolla"}

# ─────────────────────────────────────────────────────────────────
# ESTILS CONSOLA
# ─────────────────────────────────────────────────────────────────
ESTILS = {
    "normal":     {"foreground": "white",   "font": ("Consolas", 11, "normal"), "justify": "left"},
    "error":      {"foreground": "red",     "font": ("Consolas", 11, "normal"), "justify": "left"},
    "error_bold": {"foreground": "red",     "font": ("Consolas", 11, "bold"),   "justify": "left"},
    "warning":    {"foreground": "orange",  "font": ("Consolas", 11, "italic"), "justify": "left"},
    "success":    {"foreground": "#00aa44", "font": ("Consolas", 11, "normal"), "justify": "left"},
    "center":     {"foreground": "blue",    "font": ("Consolas", 11, "bold"),   "justify": "center"},
    "info":       {"foreground": "#0066cc", "font": ("Consolas", 11, "normal"), "justify": "left"},
}

def configurar_estils(consola):
    for nom, estil in ESTILS.items():
        consola.tag_config(nom, **estil)

def log(text, estil="normal", subtext="", subestil="normal", ts=True):
    def _do():
        consola.config(state=tk.NORMAL)
        prefix = f"[{datetime.now().strftime('%H:%M:%S')}] " if ts else ""
        consola.insert(tk.END, prefix + text, estil)
        if subtext:
            consola.insert(tk.END, subtext + "\n", subestil)
        else:
            consola.insert(tk.END, "\n")
        consola.see(tk.END)
        consola.config(state=tk.DISABLED)
    root.after(0, _do)

# ─────────────────────────────────────────────────────────────────
# SERVIDOR TCP
# ─────────────────────────────────────────────────────────────────
def handle_sensor_client(conn, addr):
    global sensor_client_socket, anomalies_tcp
    log(f"Sensor ESP32 connectat: {addr[0]}:{addr[1]}", "success")
    misiones.log_evento("info", f"Sensor connectat: {addr[0]}:{addr[1]}")
    with sensor_client_lock:
        sensor_client_socket = conn
    with sensor_lock:
        sensor_data["connected"] = True
    with husky_lock:
        husky_data["connected"] = True
    log("HuskyLens disponible (sensor connectat).", "info")
    misiones.log_evento("info", "HuskyLens disponible")

    try:
        buf = ""
        conn.settimeout(5.0)
        while True:
            try:
                chunk = conn.recv(512).decode("utf-8", errors="replace")
            except socket.timeout:
                continue
            if not chunk:
                break
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.replace("\r", "").strip()
                if not line:
                    continue
                _process_sensor_line(line, conn)
    except Exception as e:
        anomalies_tcp += 1
        log(f"Sensor desconnectat ({e})", "warning")
        misiones.log_evento("error", f"Sensor desconnectat inesperadament: {e}")
    finally:
        with sensor_lock:  sensor_data["connected"] = False
        with husky_lock:   husky_data["connected"]  = False
        with sensor_client_lock: sensor_client_socket = None
        conn.close()
        log("Sensor ESP32 desconnectat.", "warning")
        log("HuskyLens desconnectat (pèrdua de sensor).", "warning")
        misiones.log_evento("warning", f"Sensor desconnectat: {addr[0]}:{addr[1]}")
        misiones.log_evento("warning", "HuskyLens desconnectat per pèrdua del sensor")
        if misiones.mision_activa:
            try:
                misiones.finalizar_mision()
                log("Missió finalitzada automàticament per desconnexió del sensor.", "warning")
                misiones.log_evento("warning", "Missió finalitzada automàticament per desconnexió")
            except Exception as e:
                log(f"Error finalitzant missió: {e}", "warning")

def _process_sensor_line(line, conn):
    global pending_bme_request, pending_imu_request

    if line.startswith("DATA:"):
        try:
            parts = line[5:].split(",")
            if len(parts) < 15:
                return

            bme_t  = float(parts[0])
            bme_h  = float(parts[1])
            bme_p  = float(parts[2])
            roll   = float(parts[3])
            pitch  = float(parts[4])
            yaw    = float(parts[5])
            mag_x  = float(parts[6])
            mag_y  = float(parts[7])
            mag_z  = float(parts[8])
            c_rssi = int(parts[9])
            c_ip   = parts[10].strip()
            c_temp = float(parts[11])
            s_rssi = int(parts[12])
            s_ip   = parts[13].strip()
            s_temp = float(parts[14])

            with sensor_lock:
                sensor_data["bme_t"]  = bme_t
                sensor_data["bme_h"]  = bme_h
                sensor_data["bme_p"]  = bme_p
                sensor_data["roll"]   = roll
                sensor_data["pitch"]  = pitch
                sensor_data["yaw"]    = yaw
                sensor_data["mag_x"]  = mag_x
                sensor_data["mag_y"]  = mag_y
                sensor_data["mag_z"]  = mag_z
                sensor_data["s_rssi"] = s_rssi

            with health_lock:
                health_data["cam_rssi"]    = c_rssi
                health_data["cam_ip"]      = c_ip
                health_data["cam_temp"]    = c_temp
                health_data["sensor_rssi"] = s_rssi
                health_data["sensor_ip"]   = s_ip
                health_data["sensor_temp"] = s_temp
                health_data["history_cam_rssi"].append(c_rssi)
                if len(health_data["history_cam_rssi"]) > 30:
                    health_data["history_cam_rssi"].pop(0)

            if pending_bme_request:
                pending_bme_request = False
                log(f"BME → T:{bme_t:.2f}°C  H:{bme_h:.2f}%  P:{bme_p:.2f}hPa", "success")
            if pending_imu_request:
                pending_imu_request = False
                log(f"IMU → Roll:{roll:.2f}°  Pitch:{pitch:.2f}°  Yaw:{yaw:.2f}°", "success")

            # ── NOVA: guardar telemetria a la missió ──────────
            misiones.guardar_telemetria(
                bme_t, bme_h, bme_p,
                roll, pitch, yaw,
                mag_x, mag_y, mag_z,
                rssi=s_rssi
            )

        except Exception as e:
            log(f"Error parsejant DATA: {e}", "error")
            misiones.log_evento("error", f"Error parsejant DATA: {e}")
        return

    if line.startswith("HUSKY:"):
        try:
            parts  = line[6:].split(",")
            algo   = parts[0]
            n_objs = int(parts[1])
            objects = []
            now    = datetime.now()
            idx    = 2
            for _ in range(n_objs):
                if idx + 5 > len(parts): break
                obj = {
                    "id":      int(parts[idx]),
                    "x":       int(parts[idx+1]),
                    "y":       int(parts[idx+2]),
                    "w":       int(parts[idx+3]),
                    "h":       int(parts[idx+4]),
                    "learned": parts[idx+5] == "1" if idx+5 < len(parts) else False,
                    "algo":    algo,
                    "ts":      now,
                }
                objects.append(obj)
                idx += 6
            with husky_lock:
                husky_data["algo"]    = algo
                husky_data["objects"] = objects
                for obj in objects:
                    husky_data["history"].append(obj.copy())
                if len(husky_data["history"]) > HUSKY_HISTORY_MAX:
                    husky_data["history"] = husky_data["history"][-HUSKY_HISTORY_MAX:]

            # ── NOVA: guardar HuskyLens a la missió ──────────
            misiones.guardar_huskylens(algo, objects)

        except Exception:
            pass
        return

# ─────────────────────────────────────────────────────────────────
# CÀMERA
# ─────────────────────────────────────────────────────────────────
latest_cam_frame    = None
cam_frame_lock      = threading.Lock()
latest_decoded_pil  = None
decoded_pil_lock    = threading.Lock()
_decoded_frame_id   = 0
_displayed_frame_id = -1

def _cam_decode_loop():
    global latest_cam_frame, latest_decoded_pil, _decoded_frame_id, contador_frames_cam
    ALGO_COLOR_BGR = {
        "FACE_RECOGNITION":     (100, 107, 255),
        "OBJECT_TRACKING":      ( 61, 217, 255),
        "OBJECT_RECOGNITION":   (119, 203, 107),
        "LINE_TRACKING":        (255, 150,  77),
        "COLOR_RECOGNITION":    (255, 125, 199),
        "TAG_RECOGNITION":      ( 28, 159, 255),
        "OBJECT_CLASSIFICATION":(167, 201,   0),
        "UNKNOWN":              (170, 170, 170),
    }
    CAM_W, CAM_H     = 480, 360
    HUSKY_W, HUSKY_H = 320, 240

    while True:
        jpg = None
        with cam_frame_lock:
            if latest_cam_frame:
                jpg = latest_cam_frame
                latest_cam_frame = None
        if jpg:
            try:
                np_arr    = np.frombuffer(jpg, dtype=np.uint8)
                frame_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                gpu_frame = cv2.UMat(frame_bgr)
                frame_res = cv2.resize(gpu_frame, (CAM_W, CAM_H), interpolation=cv2.INTER_LINEAR)
                frame_cpu = frame_res.get()
                frame_cpu = cv2.flip(frame_cpu, 1)

                with husky_lock:
                    objects = list(husky_data["objects"])
                    algo    = husky_data["algo"]

                color_bgr = ALGO_COLOR_BGR.get(algo, (0, 212, 255))
                sx = CAM_W / HUSKY_W
                sy = CAM_H / HUSKY_H

                for obj in objects:
                    cx = int((HUSKY_W - obj["x"]) * sx)
                    cy = int(obj["y"] * sy)
                    bw = int(obj["w"] * sx / 2)
                    bh = int(obj["h"] * sy / 2)
                    x0, y0 = cx - bw, cy - bh
                    x1, y1 = cx + bw, cy + bh
                    cv2.rectangle(frame_cpu, (x0, y0), (x1, y1), color_bgr, thickness=2)
                    cv2.circle(frame_cpu, (cx, cy), 4, color_bgr, -1)
                    label = HUSKY_ID_LABELS.get(obj["id"], f"ID:{obj['id']}")
                    if obj.get("learned"): label += " ✓"
                    (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                    ty = max(y0 - 4, th + 4)
                    cv2.rectangle(frame_cpu, (x0, ty - th - baseline - 2), (x0 + tw + 4, ty + 2), (0, 0, 0), -1)
                    cv2.putText(frame_cpu, label, (x0 + 2, ty - baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color_bgr, 1, cv2.LINE_AA)

                frame_rgb = cv2.cvtColor(frame_cpu, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                with decoded_pil_lock:
                    latest_decoded_pil = img
                    _decoded_frame_id += 1
                contador_frames_cam += 1
            except Exception:
                pass
        else:
            time.sleep(0.004)

def handle_cam_direct_client(conn, addr):
    global latest_cam_frame, anomalies_tcp
    log(f"ESP32-CAM connectada: {addr[0]}", "success")
    misiones.log_evento("info", f"Càmera connectada: {addr[0]}")
    with cam_lock:
        cam_state["connected"] = True
        cam_state["client"]    = conn
    root.after(0, lambda: [cam_status_var.set("● STREAM BINARI ACTIU"), cam_status_label.config(fg="#00aa44")])
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    conn.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131072)
    conn.settimeout(15.0)

    def recv_exact(n):
        data = bytearray()
        while len(data) < n:
            try:
                packet = conn.recv(n - len(data))
                if not packet: return None
                data.extend(packet)
            except socket.timeout: continue
            except OSError: return None
        return bytes(data)

    try:
        while True:
            magic = recv_exact(4)
            if not magic: break
            if magic != b"MCAM": continue
            header = recv_exact(4)
            if not header: break
            tamano = int.from_bytes(header, "big")
            if tamano <= 0 or tamano > 120000: continue
            jpg_bytes = recv_exact(tamano)
            if not jpg_bytes: break
            try: conn.sendall(b'\x01')
            except Exception: break
            with cam_frame_lock:
                latest_cam_frame = jpg_bytes
    except Exception as e:
        anomalies_tcp += 1
        misiones.log_evento("error", f"Error en stream de càmera: {e}")
    finally:
        with cam_lock:
            cam_state["connected"] = False
            cam_state["client"]    = None
        conn.close()
        misiones.log_evento("warning", f"Càmera desconnectada (connexió tancada): {addr[0]}")
        _addr_cap = addr[0]
        def _check_cam_still_down():
            with cam_lock:
                still_down = not cam_state["connected"]
            if still_down:
                log(f"ESP32-CAM desconnectada de veritat (>{CAM_DISCONNECT_GRACE_MS//1000}s sense reconnectar): {_addr_cap}", "warning")
                aturar_stream_video()
        root.after(CAM_DISCONNECT_GRACE_MS, _check_cam_still_down)

def refresh_cam_ui():
    global _displayed_frame_id
    with decoded_pil_lock:
        fid = _decoded_frame_id
        img = latest_decoded_pil
    if img is not None and fid != _displayed_frame_id:
        _displayed_frame_id = fid
        try:
            cw = cam_canvas.winfo_width()
            ch = cam_canvas.winfo_height()
            if cw < 10 or ch < 10: cw, ch = 480, 360
            img_w, img_h = img.size
            scale = min(cw / img_w, ch / img_h)
            new_w, new_h = int(img_w * scale), int(img_h * scale)
            img_fit = img.resize((new_w, new_h), Image.BILINEAR)
            bg = Image.new("RGB", (cw, ch), (17, 17, 34))
            bg.paste(img_fit, ((cw - new_w) // 2, (ch - new_h) // 2))
            photo = ImageTk.PhotoImage(bg)
            cam_canvas.delete("placeholder")
            cam_canvas.create_image(0, 0, anchor=tk.NW, image=photo, tags="frame")
            cam_canvas.image = photo
        except Exception:
            pass
    root.after(30, refresh_cam_ui)

def tcp_server_loop():
    global anomalies_tcp
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((SERVER_HOST, SERVER_PORT))
    srv.listen(5)
    log("Servidor TCP iniciat. Esperant ESP32...", "normal")
    while True:
        try:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(5.0)
            hello_bytes = b""
            while not hello_bytes.endswith(b"\n"):
                b = conn.recv(1)
                if not b: raise ConnectionError
                hello_bytes += b
            hello = hello_bytes.decode("utf-8", errors="ignore").strip()
            conn.settimeout(None)
            if hello.startswith("HELLO:sensor"):
                threading.Thread(target=handle_sensor_client, args=(conn, addr), daemon=True).start()
            elif hello.startswith("HELLO:camera_direct"):
                threading.Thread(target=handle_cam_direct_client, args=(conn, addr), daemon=True).start()
            else:
                conn.close()
        except Exception as e:
            anomalies_tcp += 1
            misiones.log_evento("error", f"Anomalia TCP al acceptar connexió: {e}")

def enviar_comanda_sensor(cmd: str):
    with sensor_client_lock:
        s = sensor_client_socket
    if s:
        try: s.sendall((cmd + "\n").encode("utf-8"))
        except Exception: pass

def aturar_stream_video():
    with cam_lock:
        if cam_state["client"]:
            try: cam_state["client"].close()
            except: pass
            cam_state["connected"] = False
            cam_state["client"]    = None
    cam_status_var.set("○ Sense càmera")
    cam_status_label.config(fg="#999999")
    cam_canvas.delete("all")
    cam_canvas.create_text(240, 180, text="Esperant ESP32-CAM...", fill="#666666", font=("Consolas", 12), tags="placeholder")

# ─────────────────────────────────────────────────────────────────
# CUB 3D — MATPLOTLIB
# ─────────────────────────────────────────────────────────────────
save_data      = []
save_data_cube = []
data           = [[0] for _ in range(4)]
lineas         = []
after_id       = None
running        = True
finestra_te_focus = True  
imu_angles     = [0.0, 0.0, 0.0]
ciclos_graficos_2d = 0
contador_refresc_cubo = 0

fig_cube    = None
ax_cube     = None
canvas_cube = None

def actualitzar_cub_3d(roll_deg, pitch_deg, yaw_deg, ax_target=None, canvas_target=None):
    """Renderitza el cub 3D. Si s'especifica ax/canvas, dibuixa en ells (per al reproductor)."""
    global contador_refresc_cubo
    ax     = ax_target     if ax_target     is not None else ax_cube
    canvas = canvas_target if canvas_target is not None else canvas_cube
    if ax is None:
        return
    ax.cla()
    ax.set_title(f"Roll: {roll_deg:.1f}°  Pitch: {pitch_deg:.1f}°  Yaw: {yaw_deg:.1f}°", fontsize=9)
    for ax_method, lim in [(ax.set_xlim, [-1.5, 1.5]), (ax.set_ylim, [-1.5, 1.5]), (ax.set_zlim, [-1.5, 1.5])]:
        ax_method(lim)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")

    r = 0.5
    verts_base = np.array([
        [-r,-r,-r],[r,-r,-r],[r,r,-r],[-r,r,-r],
        [-r,-r, r],[r,-r, r],[r,r, r],[-r,r, r]
    ])
    R = (
        np.array([[np.cos(np.deg2rad(yaw_deg)), -np.sin(np.deg2rad(yaw_deg)), 0],
                  [np.sin(np.deg2rad(yaw_deg)),  np.cos(np.deg2rad(yaw_deg)), 0],
                  [0, 0, 1]])
        @
        np.array([[np.cos(np.deg2rad(pitch_deg)), 0, np.sin(np.deg2rad(pitch_deg))],
                  [0, 1, 0],
                  [-np.sin(np.deg2rad(pitch_deg)), 0, np.cos(np.deg2rad(pitch_deg))]])
        @
        np.array([[1, 0, 0],
                  [0, np.cos(np.deg2rad(roll_deg)), -np.sin(np.deg2rad(roll_deg))],
                  [0, np.sin(np.deg2rad(roll_deg)),  np.cos(np.deg2rad(roll_deg))]])
    )
    verts = (R @ verts_base.T).T

    cares_idx = [[0,1,2,3],[4,5,6,7],[0,1,5,4],[2,3,7,6],[0,3,7,4],[1,2,6,5]]
    colors    = ['#4444FF','#8888FF','#44FF44','#88FF88','#FF4444','#FF8888']
    polys     = [[verts[idx] for idx in cara] for cara in cares_idx]
    col = Poly3DCollection(polys, facecolors=colors, edgecolors='white', linewidths=0.8, alpha=0.85)
    ax.add_collection3d(col)

    axis_len = 1.2
    for vec, color in [(R[:,0], 'red'), (R[:,1], 'green'), (R[:,2], 'blue')]:
        ax.quiver(0, 0, 0, vec[0]*axis_len, vec[1]*axis_len, vec[2]*axis_len,
                  color=color, linewidth=3)

    arrow_len = 0.85; label_off = 0.22
    arrow_defs = [
        (R[:,0],             '#FF4444', f'Roll\n{roll_deg:.1f}°'),
        (R[:,1],             '#33CC33', f'Pitch\n{pitch_deg:.1f}°'),
        (np.array([0.,0.,1.]), '#4488FF', f'Yaw\n{yaw_deg:.1f}°'),
    ]
    for uv_raw, color, label in arrow_defs:
        uv = uv_raw / (np.linalg.norm(uv_raw) + 1e-9)
        ax.quiver(0, 0, 0, uv[0]*arrow_len, uv[1]*arrow_len, uv[2]*arrow_len,
                  color=color, linewidth=2.2, arrow_length_ratio=0.22)
        tip = uv * (arrow_len + label_off)
        ax.text(tip[0], tip[1], tip[2], label,
                color=color, fontsize=7, ha='center', va='center', fontweight='bold')
    try:
        canvas.draw_idle()
    except Exception:
        pass
    if ax_target is None:
        contador_refresc_cubo += 1

def crear_grafic_3d_matplotlib(parent_frame):
    global fig_cube, ax_cube, canvas_cube
    fig_cube = plt.figure(figsize=(4, 4), facecolor="#16213e")
    ax_cube  = fig_cube.add_subplot(111, projection='3d')
    ax_cube.set_title("Orientació IMU (Roll / Pitch / Yaw)", fontsize=9)
    for ax_method, lim in [(ax_cube.set_xlim, [-1.5, 1.5]), (ax_cube.set_ylim, [-1.5, 1.5]), (ax_cube.set_zlim, [-1.5, 1.5])]:
        ax_method(lim)
    ax_cube.set_xlabel("X"); ax_cube.set_ylabel("Y"); ax_cube.set_zlabel("Z")
    canvas_cube = FigureCanvasTkAgg(fig_cube, master=parent_frame)
    canvas_cube.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    actualitzar_cub_3d(0.0, 0.0, 0.0)

# ─────────────────────────────────────────────────────────────────
# GRÀFICS 2D SENSORS
# ─────────────────────────────────────────────────────────────────
def actualitzar_grafics():
    global after_id, running, ciclos_graficos_2d
    if not running: return

    with sensor_lock:
        if not sensor_data["connected"]:
            after_id = root.after(500, actualitzar_grafics)
            return
        snap = dict(sensor_data)

    nous_valors = [snap.get("bme_t"), snap.get("bme_h"), snap.get("bme_p"), snap.get("s_rssi")]
    nous_angles = [snap.get("roll"), snap.get("pitch"), snap.get("yaw")]

    raw_roll  = nous_angles[0] if nous_angles[0] is not None else imu_angles[0]
    raw_pitch = nous_angles[1] if nous_angles[1] is not None else imu_angles[1]
    raw_yaw   = nous_angles[2] if nous_angles[2] is not None else imu_angles[2]

    roll  = -raw_pitch
    pitch =  raw_roll
    yaw   =  raw_yaw

    save_data_cube.extend([roll, pitch, yaw])
    imu_angles[0] = roll
    imu_angles[1] = pitch
    imu_angles[2] = yaw

    # Cubo 3D: actualitza cada 5 cicles (~0.5s), però NOMÉS si la finestra té el focus
    # (si la finestra "dorm" ens estalviem el redibuixat car de matplotlib)
    ciclos_graficos_2d += 1
    if finestra_te_focus and ciclos_graficos_2d % 5 == 0:
        actualitzar_cub_3d(roll, pitch, yaw)

    if ciclos_graficos_2d >= 20:
        ciclos_graficos_2d = 0
        for i in range(4):
            try:
                if nous_valors[i] is not None:
                    nou_valor = float(nous_valors[i])
                else:
                    nou_valor = data[i][-1] if data[i] else 0
                # Sempre acumulem dades, finestra dormida o no — no es perd res
                # (la telemetria de la missió es guarda a banda, a _process_sensor_line)
                save_data.append(nou_valor)
                data[i].append(nou_valor)
                if len(data[i]) > 20: data[i].pop(0)
                # Però només toquem matplotlib si la finestra té el focus
                if finestra_te_focus:
                    lineas[i].set_data(range(len(data[i])), data[i])
                    axs.flat[i].relim()
                    axs.flat[i].autoscale_view()
            except Exception:
                pass
        if finestra_te_focus:
            try: canvas.draw_idle()
            except Exception: pass

    after_id = root.after(100, actualitzar_grafics)

def _redibuixar_grafics_temps_real():
    """Es crida quan la finestra recupera el focus: redibuixa de cop
    els gràfics 2D i el cub 3D amb les dades ja acumulades mentre
    la finestra estava 'dormida' (últims 20 punts de cada sèrie)."""
    for i, line in enumerate(lineas):
        try:
            line.set_data(range(len(data[i])), data[i])
            axs.flat[i].relim()
            axs.flat[i].autoscale_view()
        except Exception:
            pass
    try: canvas.draw_idle()
    except Exception: pass
    actualitzar_cub_3d(imu_angles[0], imu_angles[1], imu_angles[2])

def _on_focus_out(event):
    """La finestra principal perd el focus → entrem en mode 'dormit'."""
    global finestra_te_focus
    if event.widget != root:
        return
    finestra_te_focus = False

def _on_focus_in(event):
    """La finestra principal recupera el focus → redibuixem de cop
    amb les dades acumulades i tornem al mode normal."""
    global finestra_te_focus
    if event.widget != root:
        return
    if not finestra_te_focus:
        finestra_te_focus = True
        log("Finestra recuperada — regenerant gràfics en temps real...", "info")
        _redibuixar_grafics_temps_real()

def reset_imu():
    global imu_angles
    imu_angles = [0.0, 0.0, 0.0]
    actualitzar_cub_3d(0.0, 0.0, 0.0)

def guardar_grafics():
    try:
        with open(RUTA_SAVE, "w") as f:
            f.write(dumps(save_data) + "\n" + dumps(save_data_cube))
        log("Gràfics guardats.", "success")
    except Exception: pass

def guardar_captura_camara():
    with decoded_pil_lock: img_a_guardar = latest_decoded_pil
    if img_a_guardar is None:
        log("No hi ha cap imatge disponible de la càmera per guardar.", "error")
        return
    try:
        carpeta = RUTA_BASE / "FOTOS_SATELLITALS"
        carpeta.mkdir(parents=True, exist_ok=True)
        ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
        ruta = carpeta / f"foto_{ts}.jpg"
        img_a_guardar.save(ruta, "JPEG")
        log(f"Imatge guardada: FOTOS_SATELLITALS/foto_{ts}.jpg", "success")
    except Exception as e:
        log(f"Error al guardar la imatge: {e}", "error")

def carregar_grafics():
    global running, imu_angles
    try:
        directorio_seleccionado = filedialog.askdirectory(
            initialdir=CARPETA_MISSIONS if 'CARPETA_MISSIONS' in globals() else RUTA_BASE,
            title="Selecciona la carpeta de la missió a carregar"
        )
        
        if not directorio_seleccionado:
            return
            
        ruta_carpeta = Path(directorio_seleccionado)
        f_t = ruta_carpeta / "telemetria.json"
        
        if not f_t.exists():
            log(f"Error: No s'ha trobat 'telemetria.json' a la carpeta seleccionada.", "error")
            return
            
        running = False  
        
        # Leemos el archivo JSON de telemetría de esa misión
        with open(f_t, "r", encoding="utf-8") as f:
            telemetria_data = _json_mod.load(f)
            
        if not telemetria_data:
            log("La telemetria seleccionada està buida.", "warning")
            return
            
        temp_list = []
        hum_list = []
        pres_list = []
        
        for reg in telemetria_data:
            temp_list.append(reg.get("temperatura", 0.0))
            hum_list.append(reg.get("humedad", 0.0))
            pres_list.append(reg.get("presion", 0.0))
            
        data[0] = temp_list[-20:] if len(temp_list) >= 20 else temp_list
        data[1] = hum_list[-20:] if len(hum_list) >= 20 else hum_list
        data[2] = pres_list[-20:] if len(pres_list) >= 20 else pres_list
        data[3] = [0] * len(data[0]) if data[0] else [0]
        
        ultimo_reg = telemetria_data[-1]
        roll = ultimo_reg.get("roll", 0.0)
        pitch = ultimo_reg.get("pitch", 0.0)
        yaw = ultimo_reg.get("yaw", 0.0)
        
        imu_angles = [roll, pitch, yaw]
        actualitzar_cub_3d(roll, pitch, yaw)
        
        for i, line in enumerate(lineas):
            if i < len(data) and data[i]:
                line.set_data(range(len(data[i])), data[i])
                axs.flat[i].relim()
                axs.flat[i].autoscale_view()
                
                canvas.draw_idle()
        misiones.preparar_versio_de(ruta_carpeta.name)   # ← nueva línea
        log(f"Gràfics carregats correctament des de: {ruta_carpeta.name}", "success")
        log(f"La pròxima missió que s'iniciï es guardarà com a versió de {ruta_carpeta.name}.", "info")  # ← nueva línea
        
    except Exception as e:
        log(f"Error al carregar els gràfics de la carpeta: {e}", "error")

# ═════════════════════════════════════════════════════════════════
# PESTANYA ANÀLISI POST-MISSIÓ
# ═════════════════════════════════════════════════════════════════
def crear_pestanya_analisi(notebook):
    """Crea i retorna el frame de la pestanya d'anàlisi post-missió."""

    # ── Estat intern del reproductor ─────────────────────────
    estat = {
        "dades":         None,   # dict retornat per cargar_mision()
        "pos_ms":        0,      # posició actual en ms
        "reproduint":    False,
        "velocitat":     1,      # x1 / x2 / x5
        "after_id":      None,
        "linies_verticals": [],  # línia vermella als gràfics 2D
    }

    frame = tk.Frame(notebook, bg="#1a1a2e")

    # ════════════════════════════════════════════════════
    # ZONA A — Selector de missions (esquerra)
    # ════════════════════════════════════════════════════
    frame_selector = tk.Frame(frame, bg="#0d0d1a", width=240)
    frame_selector.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 4))
    frame_selector.pack_propagate(False)

    # Crea una etiqueta buida de text sota el selector on escriurem les dades
    lbl_detalls_missio = tk.Label(
        frame_selector, 
        text="Selecciona una missió...", 
        bg="#0d0d1a", 
        fg="#a0a0b0", 
        font=("Consolas", 9), 
        justify=tk.LEFT,
        wraplength=220
    )
    lbl_detalls_missio.pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=8)
    
    tk.Label(frame_selector, text="📂  MISSIONS GUARDADES",bg="#0d0d1a", fg="#00d4ff", font=("Consolas", 10, "bold")).pack(pady=(10, 4))
    style_missions = ttk.Style()
    style_missions.configure("Missions.Treeview",
        background="#111122", foreground="white",
        fieldbackground="#111122", rowheight=28,
        font=("Consolas", 9))
    style_missions.configure("Missions.Treeview.Heading",
        background="#1a1a3e", foreground="#00d4ff",
        font=("Consolas", 9, "bold"))
    style_missions.map("Missions.Treeview", background=[("selected", "#2244aa")])

    btn_carregar = tk.Button(frame_selector, text="▶  Carregar Missió",
              bg="#004488", fg="white", font=("Consolas", 10, "bold"),
              relief=tk.FLAT, padx=6)
    btn_refresh = tk.Button(frame_selector, text="🔄 Actualitzar llista",
              bg="#112233", fg="#00d4ff", font=("Consolas", 9, "bold"),
              relief=tk.FLAT, padx=6)

    btn_carregar.pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=(0, 8))
    btn_refresh.pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=(0, 2))

    # Ara sí, el Treeview ocupa tot l'espai que quedi.
    tree_missions = ttk.Treeview(frame_selector, columns=("missio",),
                                 show="headings", style="Missions.Treeview")
    tree_missions.heading("missio", text="Missió")
    tree_missions.column("missio", width=220, anchor=tk.W)
    sb_m = ttk.Scrollbar(frame_selector, orient=tk.VERTICAL, command=tree_missions.yview)
    tree_missions.configure(yscrollcommand=sb_m.set)
    tree_missions.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(6, 0), pady=4)
    frame_detalls = tk.LabelFrame(
    frame_selector, 
    text="Detalls de la Missió", 
    bg="#1a1a2e", 
    fg="#00d4ff", 
    font=("Consolas", 10, "bold"),
    padx=10, 
    pady=10
)
    frame_detalls.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=10)

    lbl_detalls_missio = tk.Label(
        frame_detalls, 
        text="Selecciona una missió de la llista per veure els detalls.", 
        bg="#0d0d1a", 
        fg="#a0a0b0", 
        font=("Consolas", 9), 
        justify=tk.LEFT,
        anchor=tk.W,
        wraplength=220,
        bd=4,
        relief=tk.FLAT
    )
    lbl_detalls_missio.pack(fill=tk.BOTH, expand=True)
    sb_m.pack(side=tk.LEFT, fill=tk.Y, pady=4)

    def recarregar_llista():
        for item in tree_missions.get_children():
            tree_missions.delete(item)
        _missio_map.clear()
        for m in misiones.listar_misiones():
            iid = tree_missions.insert("", tk.END, values=(m["etiqueta"],))
            _missio_map[iid] = m

    _missio_map = {}

    def on_seleccio(event=None):
        sel = tree_missions.selection()
        if not sel: return
        m = _missio_map.get(sel[0])
        if not m: return

        # Generem el text informatiu
        txt = f"📅 {m['etiqueta']}\n⏱ Duració: {m['duracion']}\n📊 Deteccions: {m['n_reg']}"

        # <--- AQUESTA LÍNIA ÉS LA QUE FALTA PERQUÈ APAREGUI:
        lbl_detalls_missio.config(text=txt, fg="#00d4ff")

        tree_missions.bind("<<TreeviewSelect>>", on_seleccio)

    def carregar_missio_seleccionada():
        sel = tree_missions.selection()
        if not sel:
            return
        m = _missio_map.get(sel[0])
        if not m: return

        _aturar_reproductor(estat)
        dades = misiones.cargar_mision(m["ruta"])
        estat["dades"]  = dades
        estat["pos_ms"] = 0

        dur_s = dades["duracion_ms"] / 1000.0
        scale_tl.config(to=max(dur_s, 1.0))
        scale_tl.set(0)
        lbl_temps.config(text="0:00 / " + _fmt_temps(dades["duracion_ms"]))

        # Dibuixar gràfics 2D complets
        _dibuixar_grafics_missio(dades, axs_r, canvas_r, estat)

        # Actualitzar log
        txt_log.config(state=tk.NORMAL)
        txt_log.delete("1.0", tk.END)
        for ln in dades["eventos"]:
            tag = "warn" if "[WARNING]" in ln else ("err" if "[ERROR]" in ln else "normal")
            txt_log.insert(tk.END, ln + "\n", tag)
        txt_log.config(state=tk.DISABLED)

        # Frame inicial
        _actualitzar_frame(estat, axs_r, canvas_r, ax_r3d, canvas_r3d, tree_husky, lbl_temps, scale_tl, _tira_vars)

        misiones.preparar_versio_de(m["nombre"])   # ← nueva línea
        log(f"Missió carregada: {m['etiqueta']}", "info")
        log(f"La pròxima missió que s'iniciï es guardarà com a versió de {m['nombre']}.", "info")  # ← nueva línea
    
    btn_refresh.config(command=recarregar_llista)
    btn_carregar.config(command=carregar_missio_seleccionada)

    # ════════════════════════════════════════════════════
    # ZONA B + C — Dreta (gràfics + reproductor)
    # ════════════════════════════════════════════════════
    frame_dreta = tk.Frame(frame, bg="#1a1a2e")
    frame_dreta.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # ── Zona B superior: cubo 3D + gràfics 2D ─────────
    frame_zona_b = tk.Frame(frame_dreta, bg="#1a1a2e")
    frame_zona_b.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    # Cubo 3D (dreta)
    frame_cub_r = tk.Frame(frame_zona_b, bg="#1a1a2e")
    frame_cub_r.pack(side=tk.RIGHT, fill=tk.BOTH)
    fig_r3d  = plt.figure(figsize=(3.5, 3.5), facecolor="#16213e")
    ax_r3d   = fig_r3d.add_subplot(111, projection='3d')
    ax_r3d.set_title("Orientació IMU", fontsize=8)
    canvas_r3d = FigureCanvasTkAgg(fig_r3d, master=frame_cub_r)
    canvas_r3d.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    actualitzar_cub_3d(0, 0, 0, ax_target=ax_r3d, canvas_target=canvas_r3d)

    # Gràfics 2D (esquerra)
    frame_grafics_r = tk.Frame(frame_zona_b, bg="#1a1a2e")
    frame_grafics_r.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # ── Tira de dades absolutes ────────────────────────────────────
    frame_tira = tk.Frame(frame_grafics_r, bg="#0d0d1a", pady=3)
    frame_tira.pack(fill=tk.X, padx=2, pady=(2, 0))

    _tira_vars = {k: tk.StringVar(value="—") for k in
                  ("temp", "hum", "pres", "rssi", "roll", "pitch", "yaw")}
    _tira_defs = [
        ("🌡 Temp",  "temp",  "°C",   "#FF6B6B"),
        ("💧 Hum",   "hum",   "%",    "#4D96FF"),
        ("🔵 Pres",  "pres",  " hPa", "#6BCB77"),
        ("📶 RSSI",  "rssi",  " dBm", "#FFD93D"),
        ("↔ Roll",   "roll",  "°",    "#FF9F1C"),
        ("↕ Pitch",  "pitch", "°",    "#C77DFF"),
        ("↻ Yaw",    "yaw",   "°",    "#00C9A7"),
    ]
    for label, key, unit, color in _tira_defs:
        cell = tk.Frame(frame_tira, bg="#0d0d1a")
        cell.pack(side=tk.LEFT, expand=True, padx=4)
        tk.Label(cell, text=label, bg="#0d0d1a", fg="#888888",
                 font=("Consolas", 7)).pack()
        tk.Label(cell, textvariable=_tira_vars[key], bg="#0d0d1a", fg=color,
                 font=("Consolas", 9, "bold")).pack()
        tk.Label(cell, text=unit, bg="#0d0d1a", fg="#555555",
                 font=("Consolas", 7)).pack()
    # ──────────────────────────────────────────────────────────────

    fig_r, axs_r = plt.subplots(2, 2, figsize=(6, 3.5))
    fig_r.tight_layout(pad=1.8)
    titols_r = ["Temperatura (°C)", "Humitat (%)", "Pressió (hPa)", "RSSI (dBm)"]
    for i, ax in enumerate(axs_r.flat):
        ax.set_title(titols_r[i], fontsize=8)
        ax.set_facecolor("#0f3460")
    fig_r.patch.set_facecolor("#16213e")
    canvas_r = FigureCanvasTkAgg(fig_r, master=frame_grafics_r)
    canvas_r.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ── Zona C inferior: reproductor + HuskyLens + log ─
    frame_zona_c = tk.Frame(frame_dreta, bg="#0d0d1a", bd=1, relief=tk.GROOVE)
    frame_zona_c.pack(side=tk.BOTTOM, fill=tk.X, padx=2, pady=(4, 2))

    # Barra de temps
    frame_tl = tk.Frame(frame_zona_c, bg="#0d0d1a")
    frame_tl.pack(fill=tk.X, padx=8, pady=(6, 2))

    lbl_temps = tk.Label(frame_tl, text="0:00 / 0:00",
                         bg="#0d0d1a", fg="#00d4ff", font=("Consolas", 10, "bold"))
    lbl_temps.pack(side=tk.LEFT, padx=(0, 8))

    scale_tl = ttk.Scale(frame_tl, from_=0, to=100, orient=tk.HORIZONTAL)
    scale_tl.pack(side=tk.LEFT, fill=tk.X, expand=True)

    _dragging = [False]

    def on_scale_press(event):
        _dragging[0] = True
        _aturar_reproductor(estat)

    def on_scale_release(event):
        _dragging[0] = False
        estat["pos_ms"] = int(scale_tl.get() * 1000)
        _actualitzar_frame(estat, axs_r, canvas_r, ax_r3d, canvas_r3d,
                           tree_husky, lbl_temps, scale_tl, _tira_vars)

    scale_tl.bind("<ButtonPress-1>",   on_scale_press)
    scale_tl.bind("<ButtonRelease-1>", on_scale_release)

    # Botons del reproductor
    frame_btns = tk.Frame(frame_zona_c, bg="#0d0d1a")
    frame_btns.pack(fill=tk.X, padx=8, pady=(2, 4))

    def btn_play():
        if not estat["dades"]: return
        estat["reproduint"] = True
        _cicle_reproductor(estat, axs_r, canvas_r, ax_r3d, canvas_r3d,
                           tree_husky, lbl_temps, scale_tl, frame_zona_c, _tira_vars)

    def btn_pause():
        estat["reproduint"] = False
        if estat["after_id"]:
            frame_zona_c.after_cancel(estat["after_id"])
            estat["after_id"] = None

    def btn_stop():
        _aturar_reproductor(estat)
        estat["pos_ms"] = 0
        scale_tl.set(0)
        if estat["dades"]:
            lbl_temps.config(text="0:00 / " + _fmt_temps(estat["dades"]["duracion_ms"]))
        _actualitzar_frame(estat, axs_r, canvas_r, ax_r3d, canvas_r3d,
                           tree_husky, lbl_temps, scale_tl, _tira_vars)

    vel_var = tk.StringVar(value="x1")
    def canviar_vel(v):
        estat["velocitat"] = int(v[1:])

    btn_style = {"bg": "#1a2a4a", "fg": "white", "font": ("Consolas", 11, "bold"),
                 "relief": tk.FLAT, "padx": 10, "pady": 2}

    tk.Button(frame_btns, text="⏮", command=btn_stop,   **btn_style).pack(side=tk.LEFT, padx=2)
    tk.Button(frame_btns, text="▶", command=btn_play,   **btn_style).pack(side=tk.LEFT, padx=2)
    tk.Button(frame_btns, text="⏸", command=btn_pause,  **btn_style).pack(side=tk.LEFT, padx=2)

    tk.Label(frame_btns, text="  Velocitat:", bg="#0d0d1a", fg="#aaaaaa",
             font=("Consolas", 9)).pack(side=tk.LEFT, padx=(12, 4))
    for v in ("x1", "x2", "x5"):
        tk.Radiobutton(frame_btns, text=v, variable=vel_var, value=v,
                       command=lambda vv=v: canviar_vel(vv),
                       bg="#0d0d1a", fg="#00d4ff", selectcolor="#112244",
                       activebackground="#0d0d1a", activeforeground="#00d4ff",
                       font=("Consolas", 9, "bold")).pack(side=tk.LEFT, padx=2)

    # Taula HuskyLens + log d'esdeveniments (paral·lels)
    frame_inferior_c = tk.Frame(frame_zona_c, bg="#0d0d1a")
    frame_inferior_c.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 6))

    # Taula HuskyLens
    frame_husky_t = tk.LabelFrame(frame_inferior_c,
                                   text=" 👁 HuskyLens — Frame actual ",
                                   bg="#0d0d1a", fg="#6BCB77",
                                   font=("Consolas", 9, "bold"), bd=2, relief=tk.GROOVE)
    frame_husky_t.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 4))

    cols_h = ("ID", "X", "Y", "W", "H", "Àrea", "Aprés")
    style_rh = ttk.Style()
    style_rh.configure("ReproH.Treeview",
        background="#111122", foreground="white",
        fieldbackground="#111122", rowheight=18,
        font=("Consolas", 9))
    style_rh.configure("ReproH.Treeview.Heading",
        background="#1a1a3e", foreground="#6BCB77",
        font=("Consolas", 9, "bold"))

    tree_husky = ttk.Treeview(frame_husky_t, columns=cols_h, show="headings",
                               style="ReproH.Treeview", height=4)
    w_h = {"ID": 30, "X": 40, "Y": 40, "W": 40, "H": 40, "Àrea": 55, "Aprés": 40}
    for c in cols_h:
        tree_husky.heading(c, text=c)
        tree_husky.column(c, width=w_h.get(c, 50), anchor=tk.CENTER)
    tree_husky.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

    # Log d'esdeveniments
    frame_log = tk.LabelFrame(frame_inferior_c,
                               text=" 📋 Log d'Esdeveniments ",
                               bg="#0d0d1a", fg="#FFD93D",
                               font=("Consolas", 9, "bold"), bd=2, relief=tk.GROOVE)
    frame_log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    txt_log = tk.Text(frame_log, bg="#0a0a14", fg="#cccccc",
                      font=("Consolas", 8), state=tk.DISABLED,
                      relief=tk.FLAT, height=5)
    txt_log.tag_config("warn", foreground="orange")
    txt_log.tag_config("err",  foreground="#FF6B6B")
    txt_log.tag_config("normal", foreground="#aaaaaa")
    sb_log = ttk.Scrollbar(frame_log, orient=tk.VERTICAL, command=txt_log.yview)
    txt_log.configure(yscrollcommand=sb_log.set)
    txt_log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4, pady=4)
    sb_log.pack(side=tk.RIGHT, fill=tk.Y)

    # Càrrega inicial de la llista
    recarregar_llista()

    return frame


# ── Funcions auxiliars del reproductor ───────────────────────────

def _fmt_temps(ms: int) -> str:
    s = ms // 1000
    return f"{s//60}:{s%60:02d}"

def _aturar_reproductor(estat: dict):
    estat["reproduint"] = False
    if estat["after_id"]:
        try: root.after_cancel(estat["after_id"])
        except Exception: pass
        estat["after_id"] = None

def _dibuixar_grafics_missio(dades: dict, axs_r, canvas_r, estat: dict):
    """Dibuixa la telemetria completa als gràfics 2D del reproductor."""
    telem = dades["telemetria"]
    if not telem:
        return

    ts_arr  = np.array([r.get("ts_ms", 0) / 1000.0 for r in telem])
    temp_arr = np.array([r.get("temperatura", 0) for r in telem])
    hum_arr  = np.array([r.get("humedad",     0) for r in telem])
    pres_arr = np.array([r.get("presion",      0) for r in telem])
    rssi_arr = np.array([r.get("rssi",         0) for r in telem])

    dades_plot = [temp_arr, hum_arr, pres_arr, rssi_arr]
    titols     = ["Temperatura (°C)", "Humitat (%)", "Pressió (hPa)", "RSSI (dBm)"]
    colors_p   = ["#FF6B6B", "#4D96FF", "#6BCB77", "#FFD93D"]

    estat["linies_verticals"] = []
    for i, ax in enumerate(axs_r.flat):
        ax.cla()
        ax.set_title(titols[i], fontsize=8)
        ax.set_facecolor("#0f3460")
        if len(ts_arr) > 0:
            ax.plot(ts_arr, dades_plot[i], color=colors_p[i], linewidth=1.2)
            ax.set_xlim(ts_arr[0], max(ts_arr[-1], 1.0))
            ax.autoscale_view(scalex=False)
        vl = ax.axvline(x=0, color="red", linewidth=1.5, linestyle="--")
        estat["linies_verticals"].append(vl)
        ax.tick_params(colors="white", labelsize=7)

    try: canvas_r.draw_idle()
    except Exception: pass

def _trobar_index_mes_proper(llista: list, ts_ms: int, clau="ts_ms") -> int:
    """Retorna l'índex de l'element amb ts_ms més proper a ts_ms."""
    if not llista: return 0
    lo, hi = 0, len(llista) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if llista[mid][clau] < ts_ms:
            lo = mid + 1
        else:
            hi = mid
    return lo

def _actualitzar_frame(estat, axs_r, canvas_r, ax_r3d, canvas_r3d,
                        tree_husky, lbl_temps, scale_tl, tira_vars=None):
    """Actualitza el cubo, la línia temporal i la taula HuskyLens a pos_ms actual."""
    dades  = estat["dades"]
    pos_ms = estat["pos_ms"]
    if not dades:
        return

    # Cubo 3D
    telem = dades["telemetria"]
    if telem:
        idx = _trobar_index_mes_proper(telem, pos_ms)
        r   = telem[idx]
        actualitzar_cub_3d(r.get("roll",0), r.get("pitch",0), r.get("yaw",0),
                           ax_target=ax_r3d, canvas_target=canvas_r3d)

        # Tira de dades absolutes
        if tira_vars:
            def _fmt(v, dec=1):
                try: return f"{float(v):.{dec}f}"
                except: return "—"
            tira_vars["temp"].set(_fmt(r.get("temperatura", "—")))
            tira_vars["hum"].set(_fmt(r.get("humedad",     "—")))
            tira_vars["pres"].set(_fmt(r.get("presion",    "—"), dec=1))
            tira_vars["rssi"].set(_fmt(r.get("rssi",       "—"), dec=0))
            tira_vars["roll"].set(_fmt(r.get("roll",       "—")))
            tira_vars["pitch"].set(_fmt(r.get("pitch",     "—")))
            tira_vars["yaw"].set(_fmt(r.get("yaw",         "—")))

    # Línia temporal vermella
    pos_s = pos_ms / 1000.0
    for vl in estat.get("linies_verticals", []):
        try: vl.set_xdata([pos_s, pos_s])
        except Exception: pass
    try: canvas_r.draw_idle()
    except Exception: pass

    # Taula HuskyLens
    for item in tree_husky.get_children():
        tree_husky.delete(item)
    husky_rows = dades["huskylens"]
    VENTANA_MS = 500
    for row in husky_rows:
        t = row.get("time_ms", 0)
        if abs(t - pos_ms) <= VENTANA_MS and row.get("id", "") != "":
            apres = "Sí" if str(row.get("learned","0")) == "1" else "No"
            tree_husky.insert("", tk.END, values=(
                row.get("id",""), row.get("x",""), row.get("y",""),
                row.get("w",""), row.get("h",""), row.get("area",""), apres
            ))

    # Etiqueta de temps + scale
    dur_ms = dades["duracion_ms"]
    lbl_temps.config(text=f"{_fmt_temps(pos_ms)} / {_fmt_temps(dur_ms)}")
    try:
        scale_tl.set(pos_ms / 1000.0)
    except Exception:
        pass

def _cicle_reproductor(estat, axs_r, canvas_r, ax_r3d, canvas_r3d,
                        tree_husky, lbl_temps, scale_tl, parent, tira_vars=None):
    """Bucle de reproducció basat en .after()."""
    if not estat["reproduint"] or not estat["dades"]:
        return

    INTERVAL_MS = 100
    estat["pos_ms"] += INTERVAL_MS * estat["velocitat"]

    dur_ms = estat["dades"]["duracion_ms"]
    if estat["pos_ms"] >= dur_ms:
        estat["pos_ms"]    = dur_ms
        estat["reproduint"] = False
        _actualitzar_frame(estat, axs_r, canvas_r, ax_r3d, canvas_r3d,
                           tree_husky, lbl_temps, scale_tl, tira_vars)
        return

    _actualitzar_frame(estat, axs_r, canvas_r, ax_r3d, canvas_r3d,
                       tree_husky, lbl_temps, scale_tl, tira_vars)

    estat["after_id"] = parent.after(
        INTERVAL_MS,
        lambda: _cicle_reproductor(estat, axs_r, canvas_r, ax_r3d, canvas_r3d,
                                   tree_husky, lbl_temps, scale_tl, parent, tira_vars)
    )

# ═════════════════════════════════════════════════════════════════
# FINESTRES SECUNDÀRIES (sense canvis)
# ═════════════════════════════════════════════════════════════════
_husky_window = None
def abrir_ventana_huskylens():
    global _husky_window
    if _husky_window is not None:
        try:
            _husky_window.deiconify(); _husky_window.lift(); _husky_window.focus_force()
            return
        except tk.TclError: _husky_window = None

    win = tk.Toplevel(root)
    win.title("HuskyLens — Detector d'Objectes")
    win.configure(bg="#0d0d1a")
    win.geometry("900x620")
    win.resizable(True, True)
    _husky_window = win

    frame_header = tk.Frame(win, bg="#0d0d1a")
    frame_header.pack(fill=tk.X, padx=10, pady=(8, 0))

    husky_status_var = tk.StringVar(value="○ Desconnectat")
    husky_algo_var   = tk.StringVar(value="Algoritme: —")
    husky_count_var  = tk.StringVar(value="Objectes: 0")

    lbl_status = tk.Label(frame_header, textvariable=husky_status_var, bg="#0d0d1a", fg="#999999", font=("Consolas", 11, "bold"))
    lbl_status.pack(side=tk.LEFT, padx=(0, 20))
    tk.Label(frame_header, textvariable=husky_algo_var,  bg="#0d0d1a", fg="#FFD93D", font=("Consolas", 11)).pack(side=tk.LEFT, padx=(0, 20))
    tk.Label(frame_header, textvariable=husky_count_var, bg="#0d0d1a", fg="#6BCB77", font=("Consolas", 11)).pack(side=tk.LEFT)

    frame_body = tk.Frame(win, bg="#0d0d1a")
    frame_body.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

    CANVAS_W, CANVAS_H = 400, 320
    SCALE_X = CANVAS_W / 320
    SCALE_Y = CANVAS_H / 240

    frame_canvas = tk.LabelFrame(frame_body, text=" Vista de Camp (320×240 px) ", bg="#0d0d1a", fg="#00d4ff", font=("Consolas", 10, "bold"), bd=2, relief=tk.GROOVE)
    frame_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=(0, 8))

    husky_canvas = tk.Canvas(frame_canvas, width=CANVAS_W, height=CANVAS_H, bg="#111122", highlightthickness=0)
    husky_canvas.pack(padx=6, pady=6)
    husky_canvas.create_line(CANVAS_W//2, 0, CANVAS_W//2, CANVAS_H, fill="#333355", width=1, dash=(4,4))
    husky_canvas.create_line(0, CANVAS_H//2, CANVAS_W, CANVAS_H//2, fill="#333355", width=1, dash=(4,4))

    frame_tabla = tk.LabelFrame(frame_body, text=" Historial de Deteccions ", bg="#0d0d1a", fg="#00d4ff", font=("Consolas", 10, "bold"), bd=2, relief=tk.GROOVE)
    frame_tabla.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    cols = ("Hora", "ID", "Algo", "X", "Y", "W", "H", "Àrea", "Aprés")
    style = ttk.Style()
    style.theme_use("default")
    style.configure("Husky.Treeview", background="#111122", foreground="white", fieldbackground="#111122", rowheight=20, font=("Consolas", 9))
    style.configure("Husky.Treeview.Heading", background="#1a1a3e", foreground="#00d4ff", font=("Consolas", 9, "bold"))
    style.map("Husky.Treeview", background=[("selected", "#2244aa")])

    tree = ttk.Treeview(frame_tabla, columns=cols, show="headings", style="Husky.Treeview", height=18)
    col_widths = {"Hora": 70, "ID": 35, "Algo": 120, "X": 45, "Y": 45, "W": 45, "H": 45, "Àrea": 55, "Aprés": 45}
    for c in cols:
        tree.heading(c, text=c)
        tree.column(c, width=col_widths.get(c, 60), anchor=tk.CENTER)
    sb = ttk.Scrollbar(frame_tabla, orient=tk.VERTICAL, command=tree.yview)
    tree.configure(yscrollcommand=sb.set)
    tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    sb.pack(side=tk.RIGHT, fill=tk.Y)

    frame_btns = tk.Frame(win, bg="#0d0d1a")
    frame_btns.pack(fill=tk.X, padx=10, pady=(0, 8))

    def limpiar_historial():
        with husky_lock: husky_data["history"].clear()
        for item in tree.get_children(): tree.delete(item)
        log("Historial HuskyLens netejat.", "info")

    def exportar_historial():
        with husky_lock: hist = list(husky_data["history"])
        if not hist:
            log("No hi ha dades HuskyLens per exportar.", "warning"); return
        try:
            folder = RUTA_BASE / f"husky_export_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            folder.mkdir(parents=True, exist_ok=True)
            ruta = folder / "historial.csv"
            with open(ruta, "w") as f:
                f.write("Hora,ID,Algo,X,Y,W,H,Area,Apres\n")
                for obj in hist:
                    area = obj["w"] * obj["h"]
                    f.write(f"{obj['ts'].strftime('%H:%M:%S')},{obj['id']},{obj['algo']},{obj['x']},{obj['y']},{obj['w']},{obj['h']},{area},{1 if obj['learned'] else 0}\n")
            log(f"Exportat: {folder.name}/historial.csv", "success")
        except Exception as e: log(f"Error exportant: {e}", "error")

    tk.Button(frame_btns, text="Netejar Historial", command=limpiar_historial, bg="#441111", fg="white", font=("Consolas", 9, "bold"), relief=tk.FLAT, padx=10).pack(side=tk.LEFT, padx=(0, 8))
    tk.Button(frame_btns, text="Exportar CSV", command=exportar_historial, bg="#114411", fg="white", font=("Consolas", 9, "bold"), relief=tk.FLAT, padx=10).pack(side=tk.LEFT)

    _last_hist_len = [0]
    def _refresh_husky():
        if not win.winfo_exists(): return
        with husky_lock:
            objects   = list(husky_data["objects"])
            algo      = husky_data["algo"]
            connected = husky_data["connected"]
            history   = list(husky_data["history"])

        husky_status_var.set("● Connectat" if connected else "○ Desconnectat")
        lbl_status.config(fg="#00aa44" if connected else "#999999")
        husky_algo_var.set(f"Algoritme: {algo}")
        husky_count_var.set(f"Objectes: {len(objects)}")
        husky_canvas.delete("obj")
        color = HUSKY_ALGO_COLORS.get(algo, "#AAAAAA")

        for obj in objects:
            cx = obj["x"] * SCALE_X; cy = obj["y"] * SCALE_Y
            bw = obj["w"] * SCALE_X / 2; bh = obj["h"] * SCALE_Y / 2
            x0, y0 = cx - bw, cy - bh; x1, y1 = cx + bw, cy + bh
            husky_canvas.create_rectangle(x0, y0, x1, y1, outline=color, width=2, tags="obj")
            r = 4
            husky_canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill=color, outline="", tags="obj")
            nombre_obj = HUSKY_ID_LABELS.get(obj["id"], f"ID:{obj['id']}")
            label_txt = f"{nombre_obj}  ({obj['x']},{obj['y']})\n{obj['w']}×{obj['h']}px"
            husky_canvas.create_text(cx, y0 - 16, text=label_txt, fill=color, font=("Consolas", 7, "bold"), tags="obj", anchor=tk.CENTER)
            if obj["learned"]:
                husky_canvas.create_text(x1 - 2, y0 + 2, text="✓", fill="#00FF88", font=("Consolas", 8, "bold"), tags="obj", anchor=tk.NE)

        new_items = history[_last_hist_len[0]:]
        _last_hist_len[0] = len(history)
        for obj in new_items:
            area = obj["w"] * obj["h"]
            row  = (obj["ts"].strftime("%H:%M:%S"), obj["id"], obj["algo"], obj["x"], obj["y"], obj["w"], obj["h"], area, "Sí" if obj["learned"] else "No")
            iid  = tree.insert("", tk.END, values=row)
            if obj["learned"]: tree.item(iid, tags=("learned",))
        tree.tag_configure("learned", foreground="#00FF88")
        if new_items:
            children = tree.get_children()
            if children: tree.see(children[-1])
        win.after(100, _refresh_husky)

    _refresh_husky()

_health_window = None
def abrir_ventana_salud():
    global _health_window
    if _health_window is not None:
        try:
            _health_window.deiconify(); _health_window.lift(); _health_window.focus_force()
            return
        except tk.TclError: _health_window = None

    win = tk.Toplevel(root)
    win.title("Salut de microcontroladors i estat de la xarxa")
    win.configure(bg="#0d0d1a")
    win.geometry("850x600")
    _health_window = win

    frame_txt = tk.Frame(win, bg="#0d0d1a")
    frame_txt.pack(fill=tk.X, padx=15, pady=10)

    lbl_s_ip    = tk.Label(frame_txt, text="IP ESP32 Sensor: --",    bg="#0d0d1a", fg="white",   font=("Consolas", 11))
    lbl_s_ip.grid(row=0, column=0, sticky="w", padx=15, pady=2)
    lbl_c_ip    = tk.Label(frame_txt, text="IP ESP32 CAM: --",       bg="#0d0d1a", fg="white",   font=("Consolas", 11))
    lbl_c_ip.grid(row=0, column=1, sticky="w", padx=15, pady=2)
    lbl_s_temp  = tk.Label(frame_txt, text="Temp. Sensor: --",       bg="#0d0d1a", fg="#00d4ff", font=("Consolas", 11, "bold"))
    lbl_s_temp.grid(row=1, column=0, sticky="w", padx=15, pady=2)
    lbl_c_temp  = tk.Label(frame_txt, text="Temp. CAM: --",          bg="#0d0d1a", fg="#FF6B6B", font=("Consolas", 11, "bold"))
    lbl_c_temp.grid(row=1, column=1, sticky="w", padx=15, pady=2)
    lbl_s_rssi  = tk.Label(frame_txt, text="RSSI Sensor: -- dBm",    bg="#0d0d1a", fg="white",   font=("Consolas", 11))
    lbl_s_rssi.grid(row=2, column=0, sticky="w", padx=15, pady=2)
    lbl_rssi_now = tk.Label(frame_txt, text="RSSI CAM: -- dBm",      bg="#0d0d1a", fg="white",   font=("Consolas", 12, "bold"))
    lbl_rssi_now.grid(row=2, column=1, sticky="w", padx=15, pady=8)

    fig_h, (ax_t, ax_r) = plt.subplots(1, 2, figsize=(8, 4))
    fig_h.tight_layout(pad=3.5)
    canvas_h = FigureCanvasTkAgg(fig_h, master=win)
    canvas_h.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=15, pady=10)

    def refresh_health_ui():
        if not win.winfo_exists(): return
        with health_lock:
            s_ip         = health_data["sensor_ip"]
            s_temp       = health_data["sensor_temp"]
            s_rssi       = health_data["sensor_rssi"]
            c_ip         = health_data["cam_ip"]
            c_temp       = health_data["cam_temp"]
            c_rssi       = health_data["cam_rssi"]
            history_rssi = list(health_data["history_cam_rssi"])

        lbl_s_ip.config(text=f"IP ESP32 Sensor: {s_ip}")
        lbl_c_ip.config(text=f"IP ESP32 CAM: {c_ip}")
        lbl_s_temp.config(text=f"Temp. Interna Sensor: {s_temp:.1f} °C")
        lbl_c_temp.config(text=f"Temp. Interna CAM: {c_temp:.1f} °C")

        def rssi_quality(v):
            if v >= -60: return "#00aa44", "Excelente"
            if v >= -75: return "#FF9F1C", "Aceptable"
            return "#FF6B6B", "Deficiente"

        sc, sq = rssi_quality(s_rssi)
        cc, cq = rssi_quality(c_rssi)
        lbl_s_rssi.config(text=f"RSSI Sensor: {s_rssi} dBm ({sq})", fg=sc)
        lbl_rssi_now.config(text=f"RSSI CAM: {c_rssi} dBm ({cq})", fg=cc)

        ax_t.cla()
        ax_t.set_title("Temperatura Interna dels Xips", fontsize=10, color="white", fontweight="bold")
        bars = ax_t.bar(["Sensor", "ESP-CAM"], [s_temp, c_temp], color=["#00d4ff", "#FF6B6B"], width=0.5)
        ax_t.set_ylabel("°C", color="white"); ax_t.set_ylim(0, 100)
        ax_t.grid(True, axis='y', linestyle=':', alpha=0.3)
        for b in bars:
            h = b.get_height()
            ax_t.text(b.get_x() + b.get_width()/2.0, h + 2, f"{h:.1f}°C", ha='center', va='bottom', color='white', fontsize=9)

        ax_r.cla()
        ax_r.set_title("Historial RSSI CAM", fontsize=10, color="white", fontweight="bold")
        if history_rssi:
            ax_r.plot(range(len(history_rssi)), history_rssi, marker='o', color=cc, linewidth=2)
        ax_r.set_ylabel("dBm", color="white"); ax_r.set_ylim(-100, -30)
        ax_r.grid(True, linestyle='--', alpha=0.4)

        fig_h.patch.set_facecolor("#16213e")
        for ax in (ax_t, ax_r):
            ax.set_facecolor("#0f3460")
            ax.tick_params(colors='white')
        try: canvas_h.draw_idle()
        except Exception: pass
        win.after(1000, refresh_health_ui)

    refresh_health_ui()

# ═════════════════════════════════════════════════════════════════
# FINESTRA — CONTROL COTXE ARDUINO
# ═════════════════════════════════════════════════════════════════
# Comandes enviades a l'ESP32 sensor (que les reenvia/interpreta cap a l'Arduino):
#   F = endavant   B = enrere   L = esquerra   R = dreta
#   Q = servo (ultrasò) cap a l'esquerra   E = servo cap a la dreta
#   S = stop total
_COTXE_COMANDES = {
    "F": "Endavant", "B": "Enrere", "L": "Esquerra", "R": "Dreta",
    "Q": "Servo ←",  "E": "Servo →", "S": "Stop",
}
_COTXE_MOVIMENT = {"F", "B", "L", "R"}   # comandes que s'auto-aturen en soltar

_coche_window = None
def abrir_ventana_coche():
    global _coche_window
    if _coche_window is not None:
        try:
            _coche_window.deiconify(); _coche_window.lift(); _coche_window.focus_force()
            return
        except tk.TclError: _coche_window = None

    win = tk.Toplevel(root)
    win.title("Control Cotxe Arduino")
    win.configure(bg="#0d0d1a")
    win.geometry("480x520")
    win.resizable(False, False)
    _coche_window = win

    estat_coche = {"ultima": None, "tecles_actives": set()}

    # ── Capçalera d'estat ──────────────────────────────────────
    frame_status = tk.Frame(win, bg="#0d0d1a")
    frame_status.pack(fill=tk.X, padx=12, pady=(12, 4))

    conn_var = tk.StringVar(value="○ Sensor desconnectat")
    tk.Label(frame_status, textvariable=conn_var, bg="#0d0d1a",
             fg="#999999", font=("Consolas", 10, "bold")).pack(side=tk.LEFT)

    cmd_var = tk.StringVar(value="Última comanda: —")
    lbl_cmd = tk.Label(win, textvariable=cmd_var, bg="#0d0d1a",
                        fg="#00d4ff", font=("Consolas", 12, "bold"))
    lbl_cmd.pack(pady=(0, 8))

    def enviar(cmd: str, label: str = None):
        enviar_comanda_sensor(cmd)
        estat_coche["ultima"] = cmd
        cmd_var.set(f"Última comanda: {label or _COTXE_COMANDES.get(cmd, cmd)} ({cmd})")
        misiones.log_evento("info", f"Comanda cotxe enviada: {cmd}")

    def aturar(_=None):
        enviar("S", "Stop")

    # ── Botonera principal de moviment (creueta) ────────────────
    frame_mov = tk.Frame(win, bg="#0d0d1a")
    frame_mov.pack(pady=8)

    def _btn_mov(parent, text, cmd, row, col):
        b = tk.Button(parent, text=text, width=6, height=3,
                      bg="#1a2a4a", fg="white", font=("Consolas", 16, "bold"),
                      relief=tk.RAISED, activebackground="#2244aa")
        b.grid(row=row, column=col, padx=6, pady=6)
        # Mentre es manté premut envia la comanda; en soltar, atura
        b.bind("<ButtonPress-1>",   lambda e, c=cmd: enviar(c))
        b.bind("<ButtonRelease-1>", aturar)
        return b

    _btn_mov(frame_mov, "▲\nF", "F", 0, 1)
    _btn_mov(frame_mov, "◀\nL", "L", 1, 0)
    _btn_mov(frame_mov, "■\nS", "S", 1, 1).config(bg="#661111")
    _btn_mov(frame_mov, "▶\nR", "R", 1, 2)
    _btn_mov(frame_mov, "▼\nB", "B", 2, 1)

    # ── Servo de l'ultrasò ───────────────────────────────────────
    frame_servo = tk.LabelFrame(win, text=" Servo Ultrasò ", bg="#0d0d1a",
                                 fg="#FFD93D", font=("Consolas", 10, "bold"),
                                 bd=2, relief=tk.GROOVE)
    frame_servo.pack(fill=tk.X, padx=20, pady=(16, 8))

    frame_servo_btns = tk.Frame(frame_servo, bg="#0d0d1a")
    frame_servo_btns.pack(pady=8)

    btn_servo_q = tk.Button(frame_servo_btns, text="◀  Q", width=10,
              bg="#443300", fg="white", font=("Consolas", 11, "bold"),
              relief=tk.FLAT, command=lambda: enviar("Q", "Servo ←"))
    btn_servo_q.pack(side=tk.LEFT, padx=6)

    btn_servo_e = tk.Button(frame_servo_btns, text="E  ▶", width=10,
              bg="#443300", fg="white", font=("Consolas", 11, "bold"),
              relief=tk.FLAT, command=lambda: enviar("E", "Servo →"))
    btn_servo_e.pack(side=tk.LEFT, padx=6)

    # ── Stop d'emergència ────────────────────────────────────────
    tk.Button(win, text="⏹  STOP  (Espai)", command=aturar,
              bg="#aa1111", fg="white", font=("Consolas", 13, "bold"),
              relief=tk.FLAT, pady=10).pack(fill=tk.X, padx=20, pady=(8, 4))

    # ── Ajuda de teclat ──────────────────────────────────────────
    tk.Label(win,
             text="Teclat: ↑/W endavant · ↓/S enrere · ←/A esquerra · →/D dreta\n"
                  "Q servo esquerra · E servo dreta · Espai = STOP",
             bg="#0d0d1a", fg="#888888", font=("Consolas", 8),
             justify=tk.CENTER).pack(pady=(4, 10))

    # ── Control per teclat (amb auto-stop en soltar tecla) ──────
    _KEY_MAP = {
        "Up": "F", "w": "F", "W": "F",
        "Down": "B", "s": "B", "S": "B",
        "Left": "L", "a": "L", "A": "L",
        "Right": "R", "d": "R", "D": "R",
        "q": "Q", "Q": "Q",
        "e": "E", "E": "E",
    }

    def on_key_press(event):
        cmd = _KEY_MAP.get(event.keysym)
        if cmd is None:
            return
        if cmd in estat_coche["tecles_actives"]:
            return  # evita repetir auto-repeat del SO
        estat_coche["tecles_actives"].add(cmd)
        enviar(cmd)

    def on_key_release(event):
        cmd = _KEY_MAP.get(event.keysym)
        if cmd is None:
            return
        estat_coche["tecles_actives"].discard(cmd)
        if cmd in _COTXE_MOVIMENT:
            aturar()

    def on_space(event):
        aturar()

    win.bind("<KeyPress>",   on_key_press)
    win.bind("<KeyRelease>", on_key_release)
    win.bind("<space>",      on_space)
    win.focus_set()

    # ── Refresc d'estat de connexió ──────────────────────────────
    def _refresh_status():
        if not win.winfo_exists(): return
        with sensor_lock:
            connectat = sensor_data["connected"]
        conn_var.set("● Sensor connectat" if connectat else "○ Sensor desconnectat")
        win.after(500, _refresh_status)

    def _on_close_coche():
        global _coche_window
        aturar()  # per seguretat, atura el cotxe en tancar la finestra
        _coche_window = None
        win.destroy()

    win.protocol("WM_DELETE_WINDOW", _on_close_coche)
    _refresh_status()

# ─────────────────────────────────────────────────────────────────
# CONSOLA — COMANDES
# ─────────────────────────────────────────────────────────────────
def comunicacio_desde_consola():
    global running, pending_bme_request, pending_imu_request
    cmd = entry_comands.get().strip().upper()
    entry_comands.delete(0, tk.END)

    match cmd:
        case "": log("Cap comanda.", "normal")
        case "CONSOLE TEST": log("CONSOLE TEST OK", "success")
        case "GRAPH STOP":
            running = False
            log("Actualització aturada.", "success")
        case "GRAPH START":
            if not running:
                running = True
                actualitzar_grafics()
                log("Actualització reiniciada.", "success")
            else: log("Ja s'està actualitzant.", "warning")
        case "GRAPH CLEAR":
            for i in range(4):
                data[i] = [0]
                lineas[i].set_data(range(1), [0])
                axs.flat[i].relim(); axs.flat[i].autoscale_view()
            canvas.draw_idle()
            log("Gràfics nets.", "success")
        case "REQUEST BME":
            pending_bme_request = True
            log("Sol·licitant BME280...", "info")
            enviar_comanda_sensor("REQUEST_BME")
        case "REQUEST IMU":
            pending_imu_request = True
            log("Sol·licitant IMU...", "info")
            enviar_comanda_sensor("REQUEST_IMU")
        case "RESET IMU":
            log("Recalibrant IMU...", "warning")
            enviar_comanda_sensor("RESET_IMU")
            reset_imu()
        case "CAM STOP":
            aturar_stream_video()
            log("Stream de càmera demanat aturar.", "warning")
        case "HUSKY":
            abrir_ventana_huskylens()
            log("Finestra HuskyLens oberta.", "info")
        case "HEALTH":
            abrir_ventana_salud()
            log("Finestra de salut oberta.", "info")
        case "COTXE":
            abrir_ventana_coche()
            log("Finestra de control del cotxe oberta.", "info")
        case "STATUS":
            with sensor_lock: sc = sensor_data["connected"]
            with cam_lock:    cc = cam_state["connected"]
            with husky_lock:  hc = husky_data["connected"]
            log(f"Sensor: {'✓' if sc else '✗'} | Càmera: {'✓' if cc else '✗'} | HuskyLens: {'✓' if hc else '✗'}", "info")
        case "HELP":
            cmds = [
                "CONSOLE TEST  — prova la consola",
                "GRAPH STOP/START/CLEAR — control gràfics",
                "REQUEST BME   — llegir BME280",
                "REQUEST IMU   — llegir MPU9250 (9-Eixos)",
                "RESET IMU     — recalibrar IMU i Magnetòmetre",
                "CAM STOP      — aturar stream",
                "HUSKY         — obrir finestra HuskyLens",
                "HEALTH        — obrir finestra de diagnòstic de salut",
                "COTXE         — obrir finestra de control del cotxe",
                "STATUS        — estat connexions",
            ]
            for c in cmds: log(c, "info", ts=False)
        case _: log(f"Comanda desconeguda: {cmd}", "error")

def on_close():
    global after_id, running
    running = False
    aturar_stream_video()
    try: misiones.finalizar_mision()
    except Exception: pass
    try: generar_reporte_rendimiento_final()
    except Exception as e: print(f"Error escribint el log final: {e}")
    if after_id:
        try: root.after_cancel(after_id)
        except: pass
    root.quit(); sys.exit()

_id_tree_last_ids = set()

def refresh_id_tree():
    with husky_lock:
        objects = list(husky_data["objects"])
    ids_actuales = {obj["id"] for obj in objects}
    global _id_tree_last_ids
    if ids_actuales != _id_tree_last_ids:
        for item in id_tree.get_children():
            id_tree.delete(item)
        for obj in objects:
            nombre = HUSKY_ID_LABELS.get(obj["id"], f"ID:{obj['id']}")
            id_tree.insert("", tk.END, values=(obj["id"], nombre))
        _id_tree_last_ids = ids_actuales
    root.after(200, refresh_id_tree)

# ═════════════════════════════════════════════════════════════════
# INTEGRACIÓ WEB
# ═════════════════════════════════════════════════════════════════

_RE_NOM_MISSIO = re.compile(r"^mision_\d{8}_\d{6}(?:_v\d+)?$")
_FITXERS_DESCARREGABLES = {"telemetria.json", "huskylens.csv", "eventos.log"}
_FITXERS_ESTATICS = {"/": "index.html", "/index.html": "index.html",
                     "/style.css": "style.css", "/app.js": "app.js",
                     "/AppICO.ico": "AppICO.ico"}

def _web_llegir_telemetria(ruta: Path) -> list:
    """Llegeix telemetria.json tolerant fitxers sense tancar (missió activa)."""
    try:
        txt = ruta.read_text(encoding="utf-8").strip()
    except Exception:
        return []
    if not txt:
        return []
    if not txt.endswith("]"):
        txt = txt.rstrip(",\n ") + "\n]"
    try:
        return _json_mod.loads(txt)
    except Exception:
        return []

def _web_stats(vals: list):
    v = [float(x) for x in vals if x is not None]
    if not v:
        return None
    return {"min": round(min(v), 2), "max": round(max(v), 2),
            "avg": round(sum(v) / len(v), 2)}

#Porque no comento esto???
#Ya no se que hace, pero lo dejo por si acaso
def _web_pics(telem: list, camps: dict, llindar: float = 3.0, maxim: int = 30) -> dict:
    pics = {}
    for c, k in camps.items():
        pics[k] = []
        if k == "yaw":   
            continue
        punts = [(int(r.get("ts_ms", 0)), float(r[c])) for r in telem if r.get(c) is not None]
        if len(punts) < 10:
            continue
        vals  = [p[1] for p in punts]
        mitja = sum(vals) / len(vals)
        sigma = (sum((v - mitja) ** 2 for v in vals) / len(vals)) ** 0.5
        if sigma < 1e-6:
            continue
        trobats = [{"ms": ms, "v": round(v, 2), "z": round((v - mitja) / sigma, 1)}
                   for ms, v in punts if abs(v - mitja) > llindar * sigma]
        trobats.sort(key=lambda p: -abs(p["z"]))                   
        pics[k] = sorted(trobats[:maxim], key=lambda p: p["ms"])    
    return pics

def _web_resum_mision(nom: str):
    if not _RE_NOM_MISSIO.match(nom):
        return None
    ruta = CARPETA_MISSIONS / nom
    if not ruta.is_dir():
        return None

    telem = _web_llegir_telemetria(ruta / "telemetria.json")

    husky_rows = []
    f_h = ruta / "huskylens.csv"
    if f_h.exists():
        try:
            with open(f_h, "r", encoding="utf-8") as f:
                husky_rows = list(csv.DictReader(f))
        except Exception:
            pass

    eventos = []
    f_e = ruta / "eventos.log"
    if f_e.exists():
        try:
            eventos = f_e.read_text(encoding="utf-8").splitlines()
        except Exception:
            pass

    activa = bool(misiones.mision_activa and misiones.mision_activa.name == nom)

    # ── Durada
    dur_ms = telem[-1].get("ts_ms", 0) if telem else 0
    if activa:
        dur_ms = max(dur_ms, misiones._ms_desde_inicio())
    for r in husky_rows:
        try: dur_ms = max(dur_ms, int(r.get("time_ms") or 0))
        except Exception: pass

    # ── Estadístiques per variable
    camps = {"temperatura": "temp", "humedad": "hum", "presion": "pres",
             "rssi": "rssi", "roll": "roll", "pitch": "pitch", "yaw": "yaw"}
    stats = {k: _web_stats([r.get(c) for r in telem]) for c, k in camps.items()}
    pics = _web_pics(telem, camps)

    # ── Qualitat RSSI (mateixos llindars que la finestra de salut)
    rssi_v = [r["rssi"] for r in telem if r.get("rssi") is not None]
    qualitat = None
    if rssi_v:
        n = len(rssi_v)
        qualitat = {
            "excelent":   round(100 * sum(1 for v in rssi_v if v >= -60) / n, 1),
            "acceptable": round(100 * sum(1 for v in rssi_v if -75 <= v < -60) / n, 1),
            "deficient":  round(100 * sum(1 for v in rssi_v if v < -75) / n, 1),
        }

    # ── Talls de dades (buits > 2 s entre mostres)
    talls, max_tall = 0, 0
    for a, b in zip(telem, telem[1:]):
        d = b.get("ts_ms", 0) - a.get("ts_ms", 0)
        if d > 2000:
            talls += 1
            max_tall = max(max_tall, d)
    freq = round(len(telem) / (dur_ms / 1000.0), 2) if dur_ms > 0 and telem else 0

    # ── HuskyLens: resum per ID + cronologia
    per_id, sense_det, algos = {}, 0, set()
    NB = 20
    cronologia = [0] * NB
    for r in husky_rows:
        if r.get("algo"): algos.add(r["algo"])
        if r.get("id", "") == "":
            sense_det += 1
            continue
        try:
            i = int(r["id"]); t = int(r.get("time_ms") or 0); a = int(r.get("area") or 0)
        except Exception:
            continue
        d = per_id.setdefault(i, {
            "id": i, "nom": HUSKY_ID_LABELS.get(i, f"ID:{i}"),
            "n": 0, "suma_area": 0, "max_area": 0,
            "primer_ms": t, "ultim_ms": t, "apres": 0})
        d["n"] += 1; d["suma_area"] += a
        d["max_area"] = max(d["max_area"], a)
        d["primer_ms"] = min(d["primer_ms"], t)
        d["ultim_ms"]  = max(d["ultim_ms"], t)
        if str(r.get("learned", "0")) == "1": d["apres"] += 1
        if dur_ms > 0:
            cronologia[min(NB - 1, int(t / dur_ms * NB))] += 1
    husky_llista = []
    for d in sorted(per_id.values(), key=lambda x: -x["n"]):
        d["area_mitja"] = round(d.pop("suma_area") / d["n"]) if d["n"] else 0
        husky_llista.append(d)

    # ── Log: comptadors per nivell
    nivells = {"INFO": 0, "WARNING": 0, "ERROR": 0}
    for ln in eventos:
        for n in nivells:
            if f"[{n}]" in ln:
                nivells[n] += 1
                break

    # ── Sèries reduïdes (màx. 300 punts) per als gràfics
    pas = max(1, len(telem) // 300)
    series = {"t": [], "temp": [], "hum": [], "pres": [], "rssi": [],
              "roll": [], "pitch": [], "yaw": []}
    for r in telem[::pas]:
        series["t"].append(r.get("ts_ms", 0))
        for c, k in camps.items():
            series[k].append(r.get(c))

    fitxers = {}
    for f in _FITXERS_DESCARREGABLES:
        p = ruta / f
        fitxers[f] = p.stat().st_size if p.exists() else 0

    return {
        "nom": nom, "activa": activa, "duracio_ms": dur_ms,
        "n_telemetria": len(telem), "freq_hz": freq,
        "talls": talls, "max_tall_ms": max_tall,
        "stats": stats, "pics": pics, "qualitat_rssi": qualitat, "series": series,
        "husky": {"total": sum(d["n"] for d in husky_llista),
                  "sense_deteccio": sense_det, "algos": sorted(algos),
                  "per_id": husky_llista, "cronologia": cronologia},
        "nivells": nivells, "eventos": eventos, "fitxers": fitxers,
    }

def _web_llista_missions() -> list:
    res = []
    for m in misiones.listar_misiones():
        res.append({
            "nom": m["nombre"], "etiqueta": m["etiqueta"],
            "duracio": m["duracion"], "n_reg": m["n_reg"],
            "activa": bool(misiones.mision_activa and misiones.mision_activa.name == m["nombre"]),
        })
    return res

def _web_estat_viu() -> dict:
    with sensor_lock:
        sc = sensor_data["connected"]
    with cam_lock:
        cc = cam_state["connected"]
    with husky_lock:
        hc = husky_data["connected"]
        n_obj = len(husky_data["objects"])
        algo = husky_data["algo"]
    with health_lock:
        h = dict(health_data)
        h.pop("history_cam_rssi", None)
    temps = time.time() - temps_inici_app
    return {
        "sensor": sc, "cam": cc, "husky": hc,
        "husky_objectes": n_obj, "husky_algo": algo,
        "mision_activa": misiones.mision_activa.name if misiones.mision_activa else None,
        "mision_ms": misiones._ms_desde_inicio() if misiones.mision_activa else 0,
        "anomalies_tcp": anomalies_tcp,
        "uptime_s": int(temps),
        "fps_cam": round(contador_frames_cam / temps, 2) if temps > 0 else 0,
        "salut": h,
    }

class _WebHandler(http.server.SimpleHTTPRequestHandler):

    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map,
                  ".js":   "application/javascript",
                  ".css":  "text/css",
                  ".html": "text/html",
                  ".jpg":  "image/x-icon"}
       
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(CARPETA_WEB), **kwargs)
        

    def log_message(self, *args):   # silenci a la consola de Tkinter
        pass

    def _json(self, obj, codi=200):
        cos = _json_mod.dumps(obj, ensure_ascii=False, default=str).encode("utf-8")
        self.send_response(codi)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(cos)))
        self.end_headers()
        self.wfile.write(cos)

    def _descarregar(self, nom: str, fitxer: str):
        if not _RE_NOM_MISSIO.match(nom) or fitxer not in _FITXERS_DESCARREGABLES:
            return self.send_error(404)
        p = CARPETA_MISSIONS / nom / fitxer
        if not p.is_file():
            return self.send_error(404)
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Disposition", f'attachment; filename="{nom}_{fitxer}"')
        self.send_header("Content-Length", str(p.stat().st_size))
        self.end_headers()
        with open(p, "rb") as f:
            self.copyfile(f, self.wfile)

    def do_GET(self):
        ruta = urlparse(self.path).path
        try:
            if ruta == "/api/live":
                return self._json(_web_estat_viu())
            if ruta == "/api/missions":
                return self._json(_web_llista_missions())
            m = re.match(r"^/api/mission/([^/]+)$", ruta)
            if m:
                res = _web_resum_mision(unquote(m.group(1)))
                return self._json(res if res else {"error": "no trobada"}, 200 if res else 404)
            m = re.match(r"^/download/([^/]+)/([^/]+)$", ruta)
            if m:
                return self._descarregar(unquote(m.group(1)), unquote(m.group(2)))
        except Exception as e:
            return self._json({"error": str(e)}, 500)
        fitxer = _FITXERS_ESTATICS.get(ruta)
        if fitxer:
            self.path = "/" + fitxer   
            return super().do_GET()
        self.send_error(404)

def generar_web_informe():
    """Comprova que els fitxers del panell (index.html, style.css, app.js) existeixen a /web."""
    CARPETA_WEB.mkdir(parents=True, exist_ok=True)
    for nom in ("index.html", "style.css", "app.js", "AppICO.ico"):
        if not (CARPETA_WEB / nom).is_file():
            print(f"[WEB] AVÍS: falta web/{nom} — el panell no es veurà correctament.")
    print("[WEB] Fitxers del panell verificats.")

def arrancar_servidor_web():
    PORT = 8080
    try:
        generar_web_informe()
        # ThreadingHTTPServer: una petició lenta no bloqueja les altres
        http.server.ThreadingHTTPServer.allow_reuse_address = True
        with http.server.ThreadingHTTPServer(("", PORT), _WebHandler) as httpd:
            print(f"[WEB] Servidor web actiu a http://localhost:{PORT}")
            httpd.serve_forever()
    except Exception as e:
        print(f"[WEB] Error iniciant servidor web: {e}")

# ═════════════════════════════════════════════════════════════════
# INTERFÍCIE UI
# ═════════════════════════════════════════════════════════════════
root = tk.Tk()
root.title("ESP32 Control Remot — Servidor")
root.protocol("WM_DELETE_WINDOW", on_close)
root.configure(bg="#1a1a2e")

menu_principal = tk.Menu(root)
root.config(menu=menu_principal)
menu_conn = tk.Menu(menu_principal, tearoff=0)
menu_principal.add_cascade(label="Arxiu", menu=menu_conn)
menu_conn.add_command(label="Carregar Missió", command=carregar_grafics)
menu_conn.add_separator()
menu_conn.add_command(label="HuskyLens", command=abrir_ventana_huskylens)

menu_eines = tk.Menu(menu_principal, tearoff=0)
menu_principal.add_cascade(label="Eines", menu=menu_eines)
menu_eines.add_command(label="🔍  HuskyLens — Detector d'Objectes", command=abrir_ventana_huskylens)
menu_eines.add_command(label="📊  Salut dels Controladors", command=abrir_ventana_salud)
menu_eines.add_command(label="🚗  Control Cotxe Arduino", command=abrir_ventana_coche)

# ── Notebook principal amb dues pestanyes ─────────────────────────
notebook_principal = ttk.Notebook(root)
notebook_principal.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

style_nb = ttk.Style()
style_nb.theme_use("default")
style_nb.configure("TNotebook",       background="#1a1a2e", borderwidth=0)
style_nb.configure("TNotebook.Tab",
    background="#0d0d1a", foreground="#888888",
    font=("Consolas", 10, "bold"), padding=[12, 4])
style_nb.map("TNotebook.Tab",
    background=[("selected", "#1a1a2e")],
    foreground=[("selected", "#00d4ff")])

# ── PESTANYA 1: Monitor en temps real ────────────────────────────
tab_monitor = tk.Frame(notebook_principal, bg="#1a1a2e")
notebook_principal.add(tab_monitor, text="📡  Monitor en Temps Real")

frame_main = tk.Frame(tab_monitor, bg="#1a1a2e")
frame_main.pack(fill=tk.BOTH, expand=True)

frame_esquerra = tk.Frame(frame_main, bg="#1a1a2e")
frame_esquerra.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

frame_top = tk.Frame(frame_esquerra, bg="#1a1a2e")
frame_top.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

frame_grafics = tk.Frame(frame_top, bg="#1a1a2e")
frame_grafics.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

mpl.rcParams.update({
    "figure.facecolor": "#16213e", "axes.facecolor": "#0f3460",
    "axes.labelcolor": "white", "xtick.color": "white",
    "ytick.color": "white", "text.color": "white",
    "axes.edgecolor": "#4444aa", "grid.color": "#334477"
})

fig, axs = plt.subplots(2, 2, figsize=(6, 4))
fig.tight_layout(pad=2.0)
titols = ["Temperatura (°C)", "Humitat (%)", "Pressió (hPa)", "RSSI Sensor (dBm)"]
for i, ax in enumerate(axs.flat):
    line, = ax.plot([], [], color="#00d4ff", linewidth=1.5)
    ax.set_title(titols[i], fontsize=9)
    lineas.append(line)
canvas = FigureCanvasTkAgg(fig, master=frame_grafics)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

frame_cubo = tk.Frame(frame_top, bg="#1a1a2e")
frame_cubo.pack(side=tk.LEFT, fill=tk.BOTH)
crear_grafic_3d_matplotlib(frame_cubo)

frame_inferior = tk.Frame(frame_esquerra, bg="#1a1a2e")
frame_inferior.pack(side=tk.BOTTOM, fill=tk.BOTH)

frame_cam = tk.Frame(frame_inferior, bg="#0d0d1a", bd=2, relief=tk.SUNKEN)
frame_cam.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
tk.Label(frame_cam, text="ESP32-CAM", bg="#0d0d1a", fg="#00d4ff", font=("Consolas", 13, "bold")).pack(pady=(8, 2))
cam_status_var = tk.StringVar(value="○ Sense càmera")
cam_status_label = tk.Label(frame_cam, textvariable=cam_status_var, bg="#0d0d1a", fg="#999999", font=("Consolas", 10))
cam_status_label.pack()
cam_canvas = tk.Canvas(frame_cam, width=480, height=360, bg="#111122", highlightthickness=0)
cam_canvas.pack(padx=8, pady=8)
cam_canvas.create_text(240, 180, text="Esperant ESP32-CAM...", fill="#666666", font=("Consolas", 12), tags="placeholder")
tk.Button(frame_cam, text="Guardar Captura", command=guardar_captura_camara, bg="#0066cc", fg="white", font=("Consolas", 10, "bold"), relief=tk.FLAT, activebackground="#0055aa", activeforeground="white").pack(pady=(0, 8))

frame_id_table = tk.LabelFrame(frame_inferior, text=" Objectes Detectats ", bg="#0d0d1a", fg="#00d4ff", font=("Consolas", 10, "bold"), bd=2, relief=tk.GROOVE)
frame_id_table.pack(side=tk.LEFT, fill=tk.BOTH, padx=(6, 0))

_style_mini = ttk.Style()
_style_mini.configure("Mini.Treeview", background="#111122", foreground="white", fieldbackground="#111122", rowheight=22, font=("Consolas", 10))
_style_mini.configure("Mini.Treeview.Heading", background="#1a1a3e", foreground="#00d4ff", font=("Consolas", 10, "bold"))
_style_mini.map("Mini.Treeview", background=[("selected", "#2244aa")])

id_tree = ttk.Treeview(frame_id_table, columns=("ID", "Objecte"), show="headings", style="Mini.Treeview", height=8)
id_tree.heading("ID", text="ID"); id_tree.heading("Objecte", text="Objecte")
id_tree.column("ID", width=45, anchor=tk.CENTER); id_tree.column("Objecte", width=110, anchor=tk.W)
id_tree.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

frame_dreta = tk.Frame(frame_main, bg="#1a1a2e")
frame_dreta.pack(side=tk.RIGHT, fill=tk.Y)

# ── Botó Iniciar / Parar Missió ──────────────────────────────────
frame_mision = tk.Frame(frame_dreta, bg="#1a1a2e")
frame_mision.pack(fill=tk.X, padx=(0, 4), pady=(0, 4))

_btn_mision_var   = tk.StringVar(value="▶  Iniciar Missió")
_lbl_mision_var   = tk.StringVar(value="○  Cap missió activa")

def _toggle_mision_monitor():
    if misiones.mision_activa:
        try:
            misiones.log_evento("info", "Missió finalitzada manualment per l'usuari")
            misiones.finalizar_mision()
            log("Missió finalitzada i guardada.", "success")
            _btn_mision_var.set("▶  Iniciar Missió")
            _lbl_mision_var.set("○  Cap missió activa")
            btn_mision_monitor.config(bg="#114411")
            lbl_mision_monitor.config(fg="#666666")
        except Exception as e:
            log(f"Error finalitzant missió: {e}", "warning")
    else:
        try:
            ruta = misiones.iniciar_mision()
            misiones.log_evento("info", "Missió iniciada manualment per l'usuari")
            log(f"Missió iniciada: {ruta.name}", "success")
            _btn_mision_var.set("⏹  Parar Missió")
            _lbl_mision_var.set(f"● {ruta.name}")
            btn_mision_monitor.config(bg="#661111")
            lbl_mision_monitor.config(fg="#00aa44")
        except Exception as e:
            log(f"Error iniciant missió: {e}", "warning")

def _sync_mision_btn():
    """Sincronitza l'estat del botó si la missió s'ha finalitzat externament."""
    if misiones.mision_activa:
        _btn_mision_var.set("⏹  Parar Missió")
        _lbl_mision_var.set(f"● {misiones.mision_activa.name}")
        btn_mision_monitor.config(bg="#661111")
        lbl_mision_monitor.config(fg="#00aa44")
    else:
        _btn_mision_var.set("▶  Iniciar Missió")
        _lbl_mision_var.set("○  Cap missió activa")
        btn_mision_monitor.config(bg="#114411")
        lbl_mision_monitor.config(fg="#666666")
    frame_dreta.after(1000, _sync_mision_btn)

btn_mision_monitor = tk.Button(
    frame_mision, textvariable=_btn_mision_var,
    bg="#114411", fg="white", font=("Consolas", 10, "bold"),
    relief=tk.FLAT, padx=8, pady=4, command=_toggle_mision_monitor)
btn_mision_monitor.pack(fill=tk.X)

lbl_mision_monitor = tk.Label(
    frame_mision, textvariable=_lbl_mision_var,
    bg="#1a1a2e", fg="#666666", font=("Consolas", 9))
lbl_mision_monitor.pack(fill=tk.X, pady=(2, 0))

consola = tk.Text(frame_dreta, height=28, width=52, bg="#0d0d1a", fg="#e0e0e0", font=("Consolas", 10), state=tk.DISABLED, insertbackground="white", relief=tk.FLAT, bd=0)
configurar_estils(consola)
consola.pack(fill=tk.BOTH, expand=True, padx=(0, 4))

frame_input = tk.Frame(frame_dreta, bg="#1a1a2e")
frame_input.pack(fill=tk.X, pady=(4, 0))
entry_comands = tk.Entry(frame_input, bg="#0d0d1a", fg="#e0e0e0", font=("Consolas", 11), relief=tk.FLAT, bd=4, insertbackground="white")
entry_comands.pack(side=tk.LEFT, fill=tk.X, expand=True)
entry_comands.bind("<Return>", lambda _: comunicacio_desde_consola())
tk.Button(frame_input, text="▶", command=comunicacio_desde_consola, bg="#0066cc", fg="white", font=("Consolas", 11, "bold"), relief=tk.FLAT, padx=8).pack(side=tk.LEFT, padx=(4, 0))

# ── PESTANYA 2: Anàlisi Post-Missió ──────────────────────────────
tab_analisi = crear_pestanya_analisi(notebook_principal)
notebook_principal.add(tab_analisi, text="📂  Anàlisi Post-Missió")

# ── Arrencada ─────────────────────────────────────────────────────
actualitzar_cub_3d(0.0, 0.0, 0.0)
threading.Thread(target=tcp_server_loop, daemon=True).start()
threading.Thread(target=_cam_decode_loop, daemon=True).start()
threading.Thread(target=arrancar_servidor_web, daemon=True).start()
actualitzar_grafics()
root.after(50, refresh_cam_ui)
root.after(200, refresh_id_tree)
root.after(1000, _sync_mision_btn)
root.bind("<FocusOut>", _on_focus_out)
root.bind("<FocusIn>",  _on_focus_in)

root.mainloop()