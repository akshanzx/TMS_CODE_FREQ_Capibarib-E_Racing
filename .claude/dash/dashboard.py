#!/usr/bin/env python3
"""
TMS Dashboard — Segmento 0
Lê dados seriais do firmware STM32 e exibe tensão, frequência e temperatura
de cada uma das 16 células do segmento 0 em tempo real.

Protocolo UART (115200 8N1):
  Uma linha por célula após cada varredura completa:
  S<seg>,C<cell>,<freq_Hz>,<volt_V>,<temp_C>\r\n
  Exemplo: S0,C03,2480.50,1.840,36.12
"""

import re
import queue
import threading
import tkinter as tk
from tkinter import ttk

import serial
import serial.tools.list_ports

NUM_CELLS = 16
LINE_RE   = re.compile(r"^S(\d+),C(\d+),([\d.]+),([\d.]+),(-?[\d.]+)")

# ── Paleta ──────────────────────────────────────────────────────────────────
BG        = "#0d1117"
PANEL     = "#161b22"
ACCENT    = "#e94560"
TEXT      = "#c9d1d9"
MUTED     = "#6e7681"
GOOD      = "#3fb950"   # temperatura ok  (< 45 °C)
WARN      = "#d29922"   # alerta          (45–59 °C)
DANGER    = "#f85149"   # crítico         (≥ 60 °C)
FONT_MONO = ("Courier New", 10)
FONT_HEAD = ("Helvetica", 10, "bold")
FONT_TITL = ("Helvetica", 14, "bold")


def temp_color(temp_str: str) -> str:
    try:
        t = float(temp_str)
        if t >= 60:
            return DANGER
        if t >= 45:
            return WARN
        return GOOD
    except ValueError:
        return TEXT


class TMSDashboard:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("TMS Dashboard — Segmento 0")
        self.root.configure(bg=BG)
        self.root.resizable(True, True)

        self.ser:     serial.Serial | None = None
        self.running: bool                 = False
        self.q:       queue.Queue[str]     = queue.Queue()

        self._build_styles()
        self._build_ui()
        self.root.after(200, self._poll_queue)

    # ── estilos ttk ─────────────────────────────────────────────────────────
    def _build_styles(self) -> None:
        s = ttk.Style()
        s.theme_use("clam")
        s.configure("TButton",     background=PANEL,  foreground=TEXT,
                    relief="flat", padding=6)
        s.map("TButton",           background=[("active", ACCENT)])
        s.configure("TCombobox",   fieldbackground=PANEL, background=PANEL,
                    foreground=TEXT, selectbackground=ACCENT)
        s.configure("TLabel",      background=BG,     foreground=TEXT)
        s.configure("Treeview",
                    background=PANEL, fieldbackground=PANEL,
                    foreground=TEXT,  rowheight=30,
                    font=FONT_MONO)
        s.configure("Treeview.Heading",
                    background=PANEL, foreground=ACCENT,
                    font=FONT_HEAD,   relief="flat")
        s.map("Treeview", background=[("selected", "#21262d")])

    # ── interface ────────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        # ── barra superior ──
        top = tk.Frame(self.root, bg=PANEL, pady=10)
        top.pack(fill="x")

        tk.Label(top, text="TMS Dashboard", bg=PANEL, fg=ACCENT,
                 font=FONT_TITL).pack(side="left", padx=16)

        ctrl = tk.Frame(top, bg=PANEL)
        ctrl.pack(side="right", padx=16)

        tk.Label(ctrl, text="Porta COM:", bg=PANEL, fg=MUTED,
                 font=("Helvetica", 9)).pack(side="left", padx=(0, 4))

        self.port_var   = tk.StringVar()
        self.port_combo = ttk.Combobox(ctrl, textvariable=self.port_var,
                                        width=10, state="readonly")
        self.port_combo.pack(side="left")

        ttk.Button(ctrl, text="↻", width=3,
                   command=self._refresh_ports).pack(side="left", padx=(4, 2))

        self.conn_btn = ttk.Button(ctrl, text="Conectar",
                                    command=self._toggle_connect)
        self.conn_btn.pack(side="left", padx=(2, 0))

        self._refresh_ports()

        # ── tabela ──
        body = tk.Frame(self.root, bg=BG, padx=20, pady=14)
        body.pack(fill="both", expand=True)

        tk.Label(body, text="Segmento 0 — 16 Células",
                 bg=BG, fg=TEXT, font=FONT_HEAD).pack(anchor="w", pady=(0, 8))

        cols = ("célula", "frequência (Hz)", "tensão (V)", "temperatura (°C)")
        self.tree = ttk.Treeview(body, columns=cols, show="headings", height=16)

        col_widths = (90, 170, 120, 180)
        for col, w in zip(cols, col_widths):
            self.tree.heading(col, text=col.title())
            self.tree.column(col, anchor="center", width=w, minwidth=60)

        for i in range(NUM_CELLS):
            self.tree.insert("", "end", iid=str(i),
                             values=(f"Célula {i:02d}", "—", "—", "—"))

        scroll = ttk.Scrollbar(body, orient="vertical",
                                command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

        # ── barra de status ──
        self.status_var = tk.StringVar(value="Desconectado")
        tk.Label(self.root, textvariable=self.status_var,
                 bg=PANEL, fg=MUTED, font=("Helvetica", 9),
                 anchor="w", padx=12, pady=4).pack(fill="x", side="bottom")

    # ── portas seriais ───────────────────────────────────────────────────────
    def _refresh_ports(self) -> None:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    # ── conexão ─────────────────────────────────────────────────────────────
    def _toggle_connect(self) -> None:
        if self.running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self.port_var.get()
        if not port:
            self.status_var.set("Selecione uma porta COM")
            return
        try:
            self.ser     = serial.Serial(port, 115200, timeout=1)
            self.running = True
            self.conn_btn.configure(text="Desconectar")
            self.status_var.set(f"Conectado → {port}  |  115200 8N1  |  aguardando dados…")
            threading.Thread(target=self._read_loop, daemon=True).start()
        except Exception as exc:
            self.status_var.set(f"Erro ao abrir {port}: {exc}")

    def _disconnect(self) -> None:
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.conn_btn.configure(text="Conectar")
        self.status_var.set("Desconectado")

    # ── leitura serial (thread separada) ────────────────────────────────────
    def _read_loop(self) -> None:
        while self.running:
            try:
                raw = self.ser.readline()
                if raw:
                    self.q.put(raw.decode("ascii", errors="ignore").strip())
            except Exception:
                break
        self.running = False
        self.root.after(0, lambda: self.conn_btn.configure(text="Conectar"))
        self.root.after(0, lambda: self.status_var.set("Conexão encerrada"))

    # ── atualização da tabela (main thread) ─────────────────────────────────
    def _poll_queue(self) -> None:
        try:
            while True:
                line = self.q.get_nowait()
                m    = LINE_RE.match(line)
                if m:
                    seg  = int(m.group(1))
                    cell = int(m.group(2))
                    if seg == 0 and 0 <= cell < NUM_CELLS:
                        freq = m.group(3)
                        volt = m.group(4)
                        temp = m.group(5)
                        self.tree.item(str(cell), values=(
                            f"Célula {cell:02d}", freq, volt, temp))
                        # colorir linha pela temperatura
                        self.tree.tag_configure(
                            f"t{cell}", foreground=temp_color(temp))
                        self.tree.item(str(cell), tags=(f"t{cell}",))
        except queue.Empty:
            pass
        self.root.after(200, self._poll_queue)


# ── entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    root.minsize(560, 560)
    TMSDashboard(root)
    root.mainloop()
