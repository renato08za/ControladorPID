import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import time
import csv
from datetime import datetime
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from collections import deque
import queue

# ============================================================================
# CLASSES AUXILIARES
# ============================================================================

class ScrollableFrame(ttk.Frame):
    def __init__(self, container, width=None, height=None, *args, **kwargs):
        super().__init__(container, *args, **kwargs)
        self.canvas = tk.Canvas(self, borderwidth=0, highlightthickness=0, background='#f0f2f5')
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = ttk.Frame(self.canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )

        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        if width:
            self.canvas.config(width=width)
        if height:
            self.canvas.config(height=height)

        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)

    def _on_mousewheel(self, event):
        try:
            self.canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        except Exception:
            pass


# ============================================================================
# TELA DE CONEXÃO
# ============================================================================

class ConnectionScreen:
    def __init__(self, root, show_main_callback):
        self.root = root
        self.show_main_callback = show_main_callback

        self.root.title("Plataforma Didática - Controle de Motor CC")
        self.root.geometry("900x700")
        self.root.configure(bg='#f0f2f5')

        self.center_window()
        self.setup_styles()
        self.create_connection_screen()

    def center_window(self):
        self.root.update_idletasks()
        width, height = 900, 700
        x = (self.root.winfo_screenwidth() // 2) - (width // 2)
        y = (self.root.winfo_screenheight() // 2) - (height // 2)
        self.root.geometry(f'{width}x{height}+{x}+{y}')

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Main.TFrame', background='#f0f2f5')
        style.configure('Title.TLabel', font=('Segoe UI', 28, 'bold'),
                        background='#f0f2f5', foreground='#2c3e50')
        style.configure('Subtitle.TLabel', font=('Segoe UI', 14),
                        background='#f0f2f5', foreground='#7f8c8d')
        style.configure('Card.TLabelframe', background='white', relief='solid',
                        borderwidth=2, padding=15)
        style.configure('Card.TLabelframe.Label', font=('Segoe UI', 10, 'bold'),
                        background='white', foreground='#2c3e50')
        for name, color, active in [
            ('Primary',   '#3498db', '#2980b9'),
            ('Secondary', '#95a5a6', '#7f8c8d'),
            ('Success',   '#2ecc71', '#27ae60'),
            ('Warning',   '#f39c12', '#e67e22'),
            ('Danger',    '#e74c3c', '#c0392b'),
        ]:
            style.configure(f'{name}.TButton', font=('Segoe UI', 10, 'bold'), padding=8,
                            background=color)
            style.map(f'{name}.TButton',
                      background=[('active', active), ('disabled', '#bdc3c7')])

    def create_connection_screen(self):
        main_frame = ttk.Frame(self.root, padding="30")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        main_frame.configure(style='Main.TFrame')
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)

        ttk.Label(main_frame, text="Plataforma Didática - Controle de Motor CC",
                  style='Title.TLabel').grid(row=0, column=0, pady=(0, 5))
        ttk.Label(main_frame, text="Sistema Didático com Arduino — Velocidade & Posição",
                  style='Subtitle.TLabel').grid(row=1, column=0, pady=(0, 20))

        conn_card = ttk.LabelFrame(main_frame, text="CONEXÃO COM ARDUINO",
                                   style='Card.TLabelframe')
        conn_card.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 15))
        conn_card.columnconfigure(1, weight=1)

        port_frame = ttk.Frame(conn_card)
        port_frame.grid(row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=8)
        port_frame.columnconfigure(1, weight=1)

        ttk.Label(port_frame, text="🔌 Porta COM:",
                  font=('Segoe UI', 10, 'bold')).grid(row=0, column=0, sticky=tk.W, padx=5)
        self.port_combo = ttk.Combobox(port_frame, font=('Segoe UI', 10),
                                       height=6, state='readonly')
        self.port_combo.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=10, pady=5)
        ttk.Button(port_frame, text="↻ Atualizar", command=self.refresh_ports,
                   style='Secondary.TButton', width=10).grid(row=0, column=2, padx=5)

        baud_frame = ttk.Frame(conn_card)
        baud_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=8)
        baud_frame.columnconfigure(1, weight=1)

        ttk.Label(baud_frame, text="📶 Baud Rate:",
                  font=('Segoe UI', 10, 'bold')).grid(row=0, column=0, sticky=tk.W, padx=5)
        self.baud_combo = ttk.Combobox(baud_frame,
                                       values=['9600', '115200', '250000', '38400', '19200'],
                                       font=('Segoe UI', 10), state='readonly', height=5)
        self.baud_combo.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=10, pady=5)
        self.baud_combo.set('115200')

        status_card = ttk.LabelFrame(main_frame, text="STATUS DO SISTEMA",
                                     style='Card.TLabelframe')
        status_card.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=(0, 15))

        si_frame = ttk.Frame(status_card)
        si_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=8)

        self.status_indicator = tk.Canvas(si_frame, width=16, height=16,
                                          bg='white', highlightthickness=0)
        self.status_indicator.grid(row=0, column=0, padx=(0, 8))
        self.status_indicator.create_oval(2, 2, 14, 14, fill='red', outline='')

        self.status_label = ttk.Label(si_frame, text="DESCONECTADO",
                                      font=('Segoe UI', 12, 'bold'), foreground='#e74c3c')
        self.status_label.grid(row=0, column=1, sticky=tk.W)

        self.status_message = ttk.Label(status_card,
                                        text="Conecte-se ao Arduino para iniciar o sistema",
                                        font=('Segoe UI', 10), foreground='#7f8c8d')
        self.status_message.grid(row=1, column=0, sticky=tk.W, pady=(0, 8))

        btn_card = ttk.LabelFrame(main_frame, text="CONTROLES PRINCIPAIS",
                                  style='Card.TLabelframe')
        btn_card.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=(0, 15))

        btn_frame = ttk.Frame(btn_card)
        btn_frame.grid(row=0, column=0, pady=8)

        self.connect_btn = ttk.Button(btn_frame, text="🔗 CONECTAR",
                                      command=self.connect_to_arduino,
                                      style='Success.TButton', width=12)
        self.connect_btn.grid(row=0, column=0, padx=4)

        self.disconnect_btn = ttk.Button(btn_frame, text="✖ DESCONECTAR",
                                         command=self.disconnect_arduino,
                                         style='Secondary.TButton',
                                         state=tk.DISABLED, width=12)
        self.disconnect_btn.grid(row=0, column=1, padx=4)

        self.vel_btn = ttk.Button(btn_frame, text="⚡ VELOCIDADE",
                                  command=lambda: self.go_to_mode("velocity"),
                                  style='Primary.TButton',
                                  state=tk.DISABLED, width=14)
        self.vel_btn.grid(row=0, column=2, padx=4)

        self.pos_btn = ttk.Button(btn_frame, text="📐 POSIÇÃO",
                                  command=lambda: self.go_to_mode("position"),
                                  style='Warning.TButton',
                                  state=tk.DISABLED, width=14)
        self.pos_btn.grid(row=0, column=3, padx=4)

        log_card = ttk.LabelFrame(main_frame, text="📝 LOG DO SISTEMA",
                                  style='Card.TLabelframe')
        log_card.grid(row=5, column=0, sticky=(tk.W, tk.E))
        log_card.columnconfigure(0, weight=1)

        log_container = ttk.Frame(log_card)
        log_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        log_container.columnconfigure(0, weight=1)

        self.log_text = tk.Text(log_container, height=6, font=('Consolas', 9),
                                bg='#2c3e50', fg='#ecf0f1',
                                insertbackground='white', borderwidth=0, relief='flat')
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        scrollbar = ttk.Scrollbar(log_container, orient="vertical",
                                  command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.log_text.configure(yscrollcommand=scrollbar.set)

        self.serial_port = None
        self.connected   = False
        self.refresh_ports()

    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_names = [p.device for p in ports]
        self.port_combo['values'] = port_names
        if port_names:
            self.port_combo.set(port_names[0])
            self.log_message("✅ Portas COM atualizadas", "success")
        else:
            self.log_message("⚠️ Nenhuma porta COM encontrada", "warning")

    def connect_to_arduino(self):
        port = self.port_combo.get()
        baud = int(self.baud_combo.get())
        if not port:
            messagebox.showerror("Erro", "Selecione uma porta COM!")
            return
        try:
            self.serial_port  = serial.Serial(port, baud, timeout=1)
            time.sleep(2)
            self.connected    = True
            self.arduino_mode = None

            self.log_message(f"✅ Conectado à {port} @ {baud}", "success")

            time.sleep(0.5)
            while self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.log_message(f"📩 {line}", "info")

            # Pergunta ao Arduino qual modo está rodando
            self.serial_port.write(b"GETMODE\n")
            time.sleep(0.5)
            arduino_mode = None
            while self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.log_message(f"📩 {line}", "info")
                if line.startswith("MODE:"):
                    arduino_mode = line.split(":", 1)[1].strip().lower()

            self.arduino_mode = arduino_mode
            self.update_connection_status(True)

        except Exception as e:
            messagebox.showerror("Erro de Conexão", f"Não foi possível conectar:\n{str(e)}")
            self.log_message(f"❌ {str(e)}", "error")

    def disconnect_arduino(self):
        if self.connected and self.serial_port:
            try:
                self.serial_port.close()
                self.serial_port = None
                self.connected   = False
                self.update_connection_status(False)
                self.log_message("ℹ️ Desconectado", "info")
            except Exception as e:
                self.log_message(f"⚠️ {str(e)}", "warning")

    def update_connection_status(self, connected):
        if connected:
            self.status_indicator.delete("all")
            self.status_indicator.create_oval(2, 2, 14, 14, fill='#2ecc71', outline='')
            self.status_label.config(text="CONECTADO", foreground='#27ae60')
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)

            mode = getattr(self, 'arduino_mode', None)
            if mode == "velocity":
                self.vel_btn.config(state=tk.NORMAL)
                self.pos_btn.config(state=tk.DISABLED)
                self.status_message.config(
                    text="Modo detectado: ⚡ VELOCIDADE  —  botão Posição bloqueado")
            elif mode == "position":
                self.vel_btn.config(state=tk.DISABLED)
                self.pos_btn.config(state=tk.NORMAL)
                self.status_message.config(
                    text="Modo detectado: 📐 POSIÇÃO  —  botão Velocidade bloqueado")
            else:
                self.vel_btn.config(state=tk.NORMAL)
                self.pos_btn.config(state=tk.NORMAL)
                self.status_message.config(
                    text="Modo não detectado — ambos os modos disponíveis")
        else:
            self.status_indicator.delete("all")
            self.status_indicator.create_oval(2, 2, 14, 14, fill='#e74c3c', outline='')
            self.status_label.config(text="DESCONECTADO", foreground='#e74c3c')
            self.status_message.config(text="Conecte-se ao Arduino para iniciar o sistema")
            self.connect_btn.config(state=tk.NORMAL)
            self.disconnect_btn.config(state=tk.DISABLED)
            self.vel_btn.config(state=tk.DISABLED)
            self.pos_btn.config(state=tk.DISABLED)

    def go_to_mode(self, mode):
        if self.connected and self.serial_port:
            self.show_main_callback(self.serial_port, mode)
        else:
            messagebox.showwarning("Aviso", "Conecte-se ao Arduino primeiro!")

    def log_message(self, message, msg_type="info"):
        try:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log_text.insert(tk.END, f"[{timestamp}] ", "timestamp")
            self.log_text.insert(tk.END, f"{message}\n", msg_type)
            self.log_text.see(tk.END)
            self.log_text.tag_config("timestamp", foreground="#95a5a6")
            self.log_text.tag_config("success",   foreground="#2ecc71")
            self.log_text.tag_config("error",     foreground="#e74c3c")
            self.log_text.tag_config("warning",   foreground="#f39c12")
            self.log_text.tag_config("info",      foreground="#3498db")
            if int(self.log_text.index('end-1c').split('.')[0]) > 50:
                self.log_text.delete('1.0', '2.0')
        except Exception:
            pass


# ============================================================================
# TELA PRINCIPAL — BASE COMUM
# ============================================================================

class BasePIDGUI:
    """Classe base com toda a lógica comum de velocidade e posição."""

    def __init__(self, root, serial_port, mode):
        self.root        = root
        self.serial_port = serial_port
        self.mode        = mode
        self.connected   = True
        self.running     = False
        self.data_logging = False
        self.csv_file    = None
        self.csv_writer  = None

        self.serial_queue = queue.Queue()
        self.data_queue   = queue.Queue()

        self.max_points = 500
        self.time_data      = deque(maxlen=self.max_points)
        self.primary_data   = deque(maxlen=self.max_points)
        self.setpoint_data  = deque(maxlen=self.max_points)
        self.pwm_data       = deque(maxlen=self.max_points)
        self.voltage_data   = deque(maxlen=self.max_points)
        self.error_data     = deque(maxlen=self.max_points)

        self.data_lock   = threading.Lock()
        self.serial_lock = threading.Lock()

        self.last_manual_send     = 0
        self.manual_send_interval = 200
        self.GRAPH_INTERVAL_MS    = 100
        self._graph_pending       = False
        self.slider_moving        = False

        # Limites Y mantidos entre updates para não encolher
        self._y1_min = None
        self._y1_max = None

        if mode == "velocity":
            self.current_params = {
                'kp': 0.06, 'ki': 0.00011, 'kd': 0.10,
                'setpoint': 100,
                'primary': 0, 'pwm': 0, 'voltage': 0, 'error': 0
            }
            self.root.title("⚡ Controlador PID — Velocidade")
        else:
            self.current_params = {
                'kp': 0.90, 'ki': 0.00011, 'kd': 1.0,
                'setpoint': 0.0,
                'primary': 0, 'pwm': 0, 'voltage': 0, 'error': 0
            }
            self.root.title("📐 Controlador PID — Posição")

        self.root.geometry("1400x850")
        self.root.configure(bg='#f0f2f5')
        self.setup_styles()
        self.setup_gui()
        self.start_threads()
        self.root.after(1000, self.request_parameters)

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Main.TFrame', background='#f0f2f5')
        style.configure('Card.TLabelframe', background='white', relief='solid',
                        borderwidth=1, padding=10)
        style.configure('Card.TLabelframe.Label', font=('Segoe UI', 9, 'bold'),
                        background='white', foreground='#2c3e50')
        style.configure('Apply.TButton', font=('Segoe UI', 8, 'bold'),
                        padding=3, background='#3498db')
        style.map('Apply.TButton',
                  background=[('active', '#2980b9'), ('disabled', '#bdc3c7')])
        for name, color, active in [
            ('Accent',    '#3498db', '#2980b9'),
            ('Danger',    '#e74c3c', '#c0392b'),
            ('Success',   '#2ecc71', '#27ae60'),
            ('Warning',   '#f39c12', '#e67e22'),
            ('Secondary', '#95a5a6', '#7f8c8d'),
        ]:
            style.configure(f'{name}.TButton', font=('Segoe UI', 9, 'bold'),
                            padding=6, background=color)
            style.map(f'{name}.TButton',
                      background=[('active', active), ('disabled', '#bdc3c7')])

    def setup_gui(self):
        main_frame = ttk.Frame(self.root, padding="15")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        main_frame.configure(style='Main.TFrame')
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=2)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)

        header_frame = ttk.Frame(main_frame)
        header_frame.grid(row=0, column=0, columnspan=2,
                          sticky=(tk.W, tk.E), pady=(0, 10))
        header_frame.columnconfigure(1, weight=1)

        mode_label = "⚡ VELOCIDADE — TEMPO REAL" if self.mode == "velocity" \
                     else "📐 POSIÇÃO — TEMPO REAL"
        mode_color = '#2980b9' if self.mode == "velocity" else '#e67e22'

        ttk.Label(header_frame, text=mode_label,
                  font=('Segoe UI', 16, 'bold'),
                  foreground=mode_color).grid(row=0, column=0, sticky=tk.W)

        ctrl_frame = ttk.Frame(header_frame)
        ctrl_frame.grid(row=0, column=1, sticky=tk.E)

        ttk.Button(ctrl_frame, text="← VOLTAR",
                   command=self.return_to_connection,
                   style='Warning.TButton', width=8).grid(row=0, column=0, padx=2)

        self.status_indicator = tk.Canvas(ctrl_frame, width=12, height=12,
                                          bg='white', highlightthickness=0)
        self.status_indicator.grid(row=0, column=1, padx=(10, 5))
        self.status_indicator.create_oval(1, 1, 11, 11, fill='#2ecc71', outline='')

        ttk.Label(ctrl_frame, text="CONECTADO",
                  font=('Segoe UI', 10, 'bold'),
                  foreground='#27ae60').grid(row=0, column=2)

        graph_card = ttk.LabelFrame(main_frame, text="GRÁFICOS",
                                    style='Card.TLabelframe', padding="10")
        graph_card.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        graph_card.columnconfigure(0, weight=1)
        graph_card.rowconfigure(0, weight=1)
        self.create_graphs(graph_card)

        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=1, column=1, sticky="nsew")
        control_frame.columnconfigure(0, weight=1)

        self.control_scroll = ScrollableFrame(control_frame, height=700)
        self.control_scroll.grid(row=0, column=0, sticky="nsew")
        cc = self.control_scroll.scrollable_frame
        cc.columnconfigure(0, weight=1)

        control_card = ttk.LabelFrame(cc, text="CONTROLE",
                                      style='Card.TLabelframe', padding="8")
        control_card.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 8))

        ctrl_btns = ttk.Frame(control_card)
        ctrl_btns.grid(row=0, column=0, sticky=(tk.W, tk.E))

        self.start_btn = ttk.Button(ctrl_btns, text="▶ INICIAR",
                                    command=self.start_control,
                                    style='Success.TButton', width=10)
        self.start_btn.grid(row=0, column=0, padx=2)

        self.stop_btn = ttk.Button(ctrl_btns, text="⏹ PARAR",
                                   command=self.stop_control,
                                   style='Danger.TButton',
                                   state=tk.DISABLED, width=10)
        self.stop_btn.grid(row=0, column=1, padx=2)

        pid_card = ttk.LabelFrame(cc, text="PARÂMETROS PID",
                                  style='Card.TLabelframe', padding="8")
        pid_card.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 8))
        pid_card.columnconfigure(0, weight=1)

        ttk.Label(pid_card, text="💡 Digite o valor e pressione Enter para aplicar",
                  font=('Segoe UI', 8), foreground='#7f8c8d').grid(
                  row=0, column=0, sticky=tk.W, pady=(0, 6))

        kp_def = self.current_params['kp']
        ki_def = self.current_params['ki']
        kd_def = self.current_params['kd']
        sp_def = self.current_params['setpoint']

        self._make_pid_row(pid_card, "Kp", 1, "kp",
                           0.0, 100.0, 0.00000001, kp_def, "{:.8f}")
        self._make_pid_row(pid_card, "Ki", 5, "ki",
                           0.0, 120.0, 0.00000001, ki_def, "{:.8f}")
        self._make_pid_row(pid_card, "Kd", 9, "kd",
                           0.0, 100.0, 0.00000001, kd_def, "{:.8f}")

        if self.mode == "velocity":
            self._make_pid_row(pid_card, "Setpoint (RPM)", 13, "setpoint",
                               0, 230, 1, sp_def, "{:.1f}")
        else:
            self._make_pid_row(pid_card, "Alvo (graus)", 13, "setpoint",
                               -3600, 3600, 1, sp_def, "{:.1f}")
            self._make_incremental_row(pid_card, row=18)

        if self.mode == "position":
            reset_card = ttk.LabelFrame(cc, text="POSIÇÃO",
                                        style='Card.TLabelframe', padding="8")
            reset_card.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 8))
            ttk.Button(reset_card, text="🔄 ZERAR POSIÇÃO",
                       command=self.reset_position,
                       style='Warning.TButton').grid(row=0, column=0, sticky=(tk.W, tk.E))

        readings_row = 3 if self.mode == "position" else 2
        readings_card = ttk.LabelFrame(cc, text="LEITURAS ATUAIS",
                                       style='Card.TLabelframe', padding="8")
        readings_card.grid(row=readings_row, column=0,
                           sticky=(tk.W, tk.E), pady=(0, 8))

        rg = ttk.Frame(readings_card)
        rg.grid(row=0, column=0, sticky=(tk.W, tk.E))

        primary_label = "RPM:" if self.mode == "velocity" else "Posição (°):"
        primary_color = '#3498db' if self.mode == "velocity" else '#e67e22'

        ttk.Label(rg, text=primary_label,
                  font=('Segoe UI', 10)).grid(row=0, column=0, sticky=tk.W, pady=3)
        self.primary_display = ttk.Label(rg, text="0.00",
                                         font=('Segoe UI', 16, 'bold'),
                                         foreground=primary_color)
        self.primary_display.grid(row=0, column=1, sticky=tk.E, pady=3, padx=(10, 0))

        sp_label = "Setpoint:" if self.mode == "velocity" else "Alvo (°):"
        ttk.Label(rg, text=sp_label,
                  font=('Segoe UI', 10)).grid(row=1, column=0, sticky=tk.W, pady=3)
        self.setpoint_display = ttk.Label(rg, text="0.00",
                                          font=('Segoe UI', 14, 'bold'),
                                          foreground='#e74c3c')
        self.setpoint_display.grid(row=1, column=1, sticky=tk.E, pady=3, padx=(10, 0))

        ttk.Label(rg, text="Erro:",
                  font=('Segoe UI', 10)).grid(row=2, column=0, sticky=tk.W, pady=3)
        self.error_display = ttk.Label(rg, text="0.00",
                                       font=('Segoe UI', 12, 'bold'),
                                       foreground='#f39c12')
        self.error_display.grid(row=2, column=1, sticky=tk.E, pady=3, padx=(10, 0))

        ttk.Label(rg, text="PWM:",
                  font=('Segoe UI', 10)).grid(row=3, column=0, sticky=tk.W, pady=3)
        self.pwm_display = ttk.Label(rg, text="0",
                                     font=('Segoe UI', 14, 'bold'),
                                     foreground='#2ecc71')
        self.pwm_display.grid(row=3, column=1, sticky=tk.E, pady=3, padx=(10, 0))

        ttk.Label(rg, text="Tensão:",
                  font=('Segoe UI', 10)).grid(row=4, column=0, sticky=tk.W, pady=3)
        self.voltage_display = ttk.Label(rg, text="0.00 V",
                                         font=('Segoe UI', 12, 'bold'),
                                         foreground='#9b59b6')
        self.voltage_display.grid(row=4, column=1, sticky=tk.E, pady=3, padx=(10, 0))

        log_row = readings_row + 2
        log_btns = ttk.Frame(cc)
        log_btns.grid(row=log_row - 1, column=0, sticky=(tk.W, tk.E), pady=(0, 8))

        self.log_start_btn = ttk.Button(log_btns, text="📝 INICIAR LOG",
                                        command=self.start_logging,
                                        style='Accent.TButton', width=12)
        self.log_start_btn.grid(row=0, column=0, padx=2, pady=2)

        self.log_stop_btn = ttk.Button(log_btns, text="🛑 PARAR LOG",
                                       command=self.stop_logging,
                                       style='Danger.TButton',
                                       state=tk.DISABLED, width=12)
        self.log_stop_btn.grid(row=0, column=1, padx=2, pady=2)

        log_card = ttk.LabelFrame(cc, text="LOG DO SISTEMA",
                                  style='Card.TLabelframe', padding="6")
        log_card.grid(row=log_row, column=0, sticky=(tk.W, tk.E))
        log_card.columnconfigure(0, weight=1)

        self.log_text = tk.Text(log_card, height=4, font=('Consolas', 8),
                                bg='#2c3e50', fg='#ecf0f1',
                                insertbackground='white',
                                borderwidth=0, relief='flat')
        self.log_text.grid(row=0, column=0,
                           sticky=(tk.W, tk.E, tk.N, tk.S), padx=3, pady=3)
        ls = ttk.Scrollbar(log_card, orient="vertical", command=self.log_text.yview)
        ls.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.log_text.configure(yscrollcommand=ls.set)

        self.update_gui_state()

        for attr in ['kp', 'ki', 'kd', 'setpoint']:
            scale = getattr(self, f"{attr}_scale")
            scale.bind('<ButtonPress-1>',
                       lambda e: setattr(self, 'slider_moving', True))
            scale.bind('<ButtonRelease-1>',
                       lambda e: [setattr(self, 'slider_moving', False),
                                  self.send_manual_parameters()])

    def _make_incremental_row(self, parent, row):
        ttk.Separator(parent, orient='horizontal').grid(
            row=row, column=0, sticky=(tk.W, tk.E), pady=4)

        inc_frame = ttk.Frame(parent)
        inc_frame.grid(row=row + 1, column=0, sticky=(tk.W, tk.E), pady=(0, 4))
        inc_frame.columnconfigure(0, weight=1)

        ttk.Label(inc_frame, text="Incremento (°):",
                  font=('Segoe UI', 9, 'bold')).grid(row=0, column=0,
                                                     columnspan=2, sticky=tk.W)

        self.increment_entry = ttk.Entry(inc_frame, font=('Consolas', 9), width=12)
        self.increment_entry.grid(row=1, column=0, sticky=(tk.W, tk.E), padx=(0, 4))
        self.increment_entry.insert(0, "90.0")
        self.increment_entry.bind('<Return>', lambda e: self.send_increment())

        ttk.Button(inc_frame, text="➤ Enviar",
                   command=self.send_increment,
                   style='Apply.TButton', width=8).grid(row=1, column=1)

    def send_increment(self):
        try:
            val = float(self.increment_entry.get().strip().replace(',', '.'))
            novo_alvo = self.setpoint_scale.get() + val
            self.setpoint_scale.set(novo_alvo)
            self.setpoint_value.config(text=f"{novo_alvo:.1f}")
            entry = self.setpoint_entry
            entry.delete(0, tk.END)
            entry.insert(0, f"{novo_alvo:.1f}")
            self.send_command(f"SETPOS:{val:.2f}")
            self.log_message(f"Incremento: {val:+.1f}° → Alvo: {novo_alvo:.1f}°", "success")
        except ValueError:
            self.log_message("⚠️ Valor de incremento inválido", "error")

    def reset_position(self):
        self.send_command("RESETPOS")
        self.setpoint_scale.set(0)
        self.setpoint_value.config(text="0.0")
        entry = self.setpoint_entry
        entry.delete(0, tk.END)
        entry.insert(0, "0.0")
        with self.data_lock:
            self.time_data.clear()
            self.primary_data.clear()
            self.setpoint_data.clear()
            self.pwm_data.clear()
            self.voltage_data.clear()
            self.error_data.clear()
        # Reseta limites acumulados do eixo Y
        self._y1_min = None
        self._y1_max = None
        self.log_message("Posição zerada", "info")

    def _make_pid_row(self, parent, label, row_start, attr_prefix,
                      scale_from, scale_to, resolution, default_val, fmt):
        header_row = ttk.Frame(parent)
        header_row.grid(row=row_start, column=0,
                        sticky=(tk.W, tk.E), pady=(6, 0))
        header_row.columnconfigure(1, weight=1)

        ttk.Label(header_row, text=f"{label}:",
                  font=('Segoe UI', 9, 'bold')).grid(row=0, column=0, sticky=tk.W)

        value_label = ttk.Label(header_row, text=fmt.format(default_val),
                                font=('Segoe UI', 9, 'bold'), foreground='#2980b9')
        value_label.grid(row=0, column=1, sticky=tk.E)
        setattr(self, f"{attr_prefix}_value", value_label)

        entry_row = ttk.Frame(parent)
        entry_row.grid(row=row_start + 1, column=0,
                       sticky=(tk.W, tk.E), pady=(2, 0))
        entry_row.columnconfigure(0, weight=1)

        entry = ttk.Entry(entry_row, font=('Consolas', 9), width=18)
        entry.grid(row=0, column=0, sticky=(tk.W, tk.E), padx=(0, 4))
        entry.insert(0, fmt.format(default_val))
        entry.bind('<Return>',   lambda e, p=attr_prefix: self.apply_entry_value(p))
        entry.bind('<FocusOut>', lambda e, p=attr_prefix: self.apply_entry_value(p))
        setattr(self, f"{attr_prefix}_entry", entry)

        ttk.Button(entry_row, text="✔ Aplicar",
                   command=lambda p=attr_prefix: self.apply_entry_value(p),
                   style='Apply.TButton', width=8).grid(row=0, column=1)

        scale = tk.Scale(parent, from_=scale_from, to=scale_to,
                         resolution=resolution, orient=tk.HORIZONTAL, length=210,
                         command=lambda val, p=attr_prefix: self._on_scale_moved(p, val),
                         bg='white', troughcolor='#dfe6e9',
                         activebackground='#3498db', highlightthickness=0,
                         font=('Segoe UI', 7))
        scale.grid(row=row_start + 2, column=0, sticky=tk.W, pady=(0, 4))
        scale.set(default_val)
        setattr(self, f"{attr_prefix}_scale", scale)

        ttk.Separator(parent, orient='horizontal').grid(
            row=row_start + 3, column=0, sticky=(tk.W, tk.E), pady=4)

    def _on_scale_moved(self, attr_prefix, val):
        try:
            value = getattr(self, f"{attr_prefix}_scale").get()
            fmt   = "{:.1f}" if attr_prefix == "setpoint" else "{:.8f}"
            getattr(self, f"{attr_prefix}_value").config(text=fmt.format(value))
            entry = getattr(self, f"{attr_prefix}_entry")
            entry.delete(0, tk.END)
            entry.insert(0, fmt.format(value))
            if self.slider_moving:
                current_time = time.time() * 1000
                if current_time - self.last_manual_send > self.manual_send_interval:
                    self.send_manual_parameters()
                    self.last_manual_send = current_time
        except Exception:
            pass

    def apply_entry_value(self, attr_prefix):
        try:
            entry = getattr(self, f"{attr_prefix}_entry")
            scale = getattr(self, f"{attr_prefix}_scale")
            value_label = getattr(self, f"{attr_prefix}_value")

            raw = entry.get().strip().replace(',', '.')
            if not raw:
                return

            value = float(raw)
            value = max(scale.cget('from'), min(scale.cget('to'), value))
            fmt   = "{:.1f}" if attr_prefix == "setpoint" else "{:.8f}"

            scale.set(value)
            value_label.config(text=fmt.format(value))
            entry.delete(0, tk.END)
            entry.insert(0, fmt.format(value))

            self.send_manual_parameters()
            self.log_message(f"{attr_prefix.upper()} → {fmt.format(value)}", "success")
        except ValueError:
            self.log_message(f"⚠️ Valor inválido para {attr_prefix.upper()}", "error")
        except Exception as e:
            self.log_message(f"Erro: {str(e)}", "error")

    def create_graphs(self, parent):
        self.fig = Figure(figsize=(10, 7), dpi=100)
        self.fig.patch.set_facecolor('#f8f9fa')

        primary_title  = "VELOCIDADE DO MOTOR" if self.mode == "velocity" else "POSIÇÃO DO MOTOR"
        primary_ylabel = "RPM"                 if self.mode == "velocity" else "Graus (°)"
        primary_color  = '#3498db'             if self.mode == "velocity" else '#e67e22'
        primary_legend = "RPM Real"            if self.mode == "velocity" else "Posição Real"
        sp_legend      = "Setpoint"            if self.mode == "velocity" else "Alvo"

        self.ax1 = self.fig.add_subplot(211)
        self.ax1.set_title(primary_title, fontsize=12, fontweight='bold', color='#2c3e50')
        self.ax1.set_xlabel('Tempo (s)', fontsize=10, color='#7f8c8d')
        self.ax1.set_ylabel(primary_ylabel, fontsize=10, color='#7f8c8d')
        self.ax1.grid(True, alpha=0.2, linestyle='--', color='#bdc3c7')
        self.ax1.set_facecolor('#ffffff')
        self.line_primary,  = self.ax1.plot([], [], color=primary_color,
                                            linewidth=2.0, label=primary_legend)
        self.line_setpoint, = self.ax1.plot([], [], color='#e74c3c',
                                            linewidth=1.5, linestyle='--', label=sp_legend)
        self.ax1.legend(loc='upper right', fontsize=9, framealpha=0.9)
        self.ax1.spines['top'].set_visible(False)
        self.ax1.spines['right'].set_visible(False)

        self.ax2 = self.fig.add_subplot(212)
        self.ax2.set_title('SINAL PWM', fontsize=12, fontweight='bold', color='#2c3e50')
        self.ax2.set_xlabel('Tempo (s)', fontsize=10, color='#7f8c8d')
        self.ax2.set_ylabel('PWM (0-255)', fontsize=10, color='#7f8c8d')
        self.ax2.set_ylim(-5, 260)
        self.ax2.grid(True, alpha=0.2, linestyle='--', color='#bdc3c7')
        self.ax2.set_facecolor('#ffffff')
        self.line_pwm, = self.ax2.plot([], [], color='#2ecc71',
                                       linewidth=2.0, label='PWM')
        self.ax2.legend(loc='upper right', fontsize=9, framealpha=0.9)
        self.ax2.spines['top'].set_visible(False)
        self.ax2.spines['right'].set_visible(False)

        self.fig.tight_layout(pad=2.5)

        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0,
                                         sticky=(tk.W, tk.E, tk.N, tk.S))

    def start_threads(self):
        threading.Thread(target=self.read_serial,  daemon=True).start()
        threading.Thread(target=self.process_data, daemon=True).start()
        self.log_message("Threads iniciadas", "success")

    def read_serial(self):
        while self.connected:
            try:
                if self.serial_port and self.serial_port.is_open:
                    with self.serial_lock:
                        if self.serial_port.in_waiting > 0:
                            data = self.serial_port.readline().decode(
                                'utf-8', errors='ignore').strip()
                        else:
                            time.sleep(0.01)
                            continue
                else:
                    time.sleep(0.1)
                    continue

                if data:
                    if any(data.startswith(p) for p in
                           ["ARDUINO:", "STATUS:", "COMANDO:", "FORMATO:"]):
                        self.root.after(0, lambda d=data: self.log_message(d, "info"))
                    elif data.startswith("PARAMS:"):
                        self.process_parameters(data)
                    elif ',' in data:
                        self.data_queue.put(data)
            except (serial.SerialException, OSError) as e:
                if self.connected:
                    self.root.after(0, lambda: self.log_message(
                        f"Erro serial: {str(e)}", "error"))
                break
            except Exception as e:
                if self.connected:
                    self.root.after(0, lambda: self.log_message(
                        f"Erro inesperado: {str(e)}", "error"))
                time.sleep(0.1)

    def process_parameters(self, data):
        try:
            params = data[7:].split(',')
            if len(params) >= 4:
                kp, ki, kd = float(params[0]), float(params[1]), float(params[2])
                sp = float(params[3])
                self.current_params.update(
                    {'kp': kp, 'ki': ki, 'kd': kd, 'setpoint': sp})
                self.root.after(0, self.update_parameter_displays)
        except Exception as e:
            self.root.after(0, lambda: self.log_message(
                f"Erro params: {str(e)}", "error"))

    def update_parameter_displays(self):
        for attr, fmt in [('kp', "{:.8f}"), ('ki', "{:.8f}"),
                          ('kd', "{:.8f}"), ('setpoint', "{:.1f}")]:
            val = self.current_params[attr]
            getattr(self, f"{attr}_value").config(text=fmt.format(val))
            try:
                getattr(self, f"{attr}_scale").set(val)
                entry = getattr(self, f"{attr}_entry")
                entry.delete(0, tk.END)
                entry.insert(0, fmt.format(val))
            except Exception:
                pass

    def process_data(self):
        while self.connected:
            try:
                data  = self.data_queue.get(timeout=0.1)
                parts = data.split(',')

                if len(parts) >= 7:
                    try:
                        tempo_ms = float(parts[0])
                        primary  = float(parts[1])
                        setpoint = float(parts[2])
                        voltage  = float(parts[3])
                        pwm      = int(parts[4])
                        error    = float(parts[5])

                        self.current_params.update({
                            'primary': primary, 'setpoint': setpoint,
                            'voltage': voltage, 'pwm': pwm, 'error': error
                        })

                        current_time = tempo_ms / 1000.0

                        with self.data_lock:
                            self.time_data.append(current_time)
                            self.primary_data.append(primary)
                            self.setpoint_data.append(setpoint)
                            self.pwm_data.append(pwm)
                            self.voltage_data.append(voltage)
                            self.error_data.append(error)

                        self.root.after(0, self.update_readings)

                        if not self._graph_pending:
                            self._graph_pending = True
                            self.root.after(self.GRAPH_INTERVAL_MS,
                                            self._throttled_graph_update)

                        if self.data_logging and self.csv_writer:
                            timestamp = datetime.now().strftime(
                                "%Y-%m-%d %H:%M:%S.%f")[:-3]
                            try:
                                self.csv_writer.writerow([
                                    timestamp, current_time,
                                    setpoint, primary, voltage, pwm, error,
                                    self.current_params['kp'],
                                    self.current_params['ki'],
                                    self.current_params['kd']
                                ])
                                self.csv_file.flush()
                            except Exception:
                                pass
                    except ValueError:
                        pass

            except queue.Empty:
                continue
            except Exception as e:
                if self.connected:
                    self.root.after(0, lambda: self.log_message(
                        f"Erro processamento: {str(e)}", "error"))
                time.sleep(0.1)

    def _throttled_graph_update(self):
        self._graph_pending = False
        self.update_graphs()

    def update_readings(self):
        try:
            self.primary_display.config(
                text=f"{self.current_params['primary']:.2f}")
            self.setpoint_display.config(
                text=f"{self.current_params['setpoint']:.2f}")
            self.error_display.config(
                text=f"{self.current_params['error']:.2f}")
            self.pwm_display.config(
                text=f"{self.current_params['pwm']}")
            self.voltage_display.config(
                text=f"{self.current_params['voltage']:.3f} V")
        except Exception:
            pass

    def update_graphs(self):
        try:
            with self.data_lock:
                if not self.time_data:
                    return
                t_arr  = np.array(self.time_data,     dtype=float)
                p_arr  = np.array(self.primary_data,  dtype=float)
                sp_arr = np.array(self.setpoint_data, dtype=float)
                pw_arr = np.array(self.pwm_data,      dtype=float)

            self.line_primary.set_data(t_arr, p_arr)
            self.line_setpoint.set_data(t_arr, sp_arr)

            # ── Eixo X: autoscale normal ──────────────────────────────────
            self.ax1.relim()
            self.ax1.autoscale_view()

            # ── Eixo Y: acumula o maior range já visto — nunca encolhe ───
            # 1. Calcula o range necessário para os dados atuais + margem 10%
            raw_min = min(p_arr.min(), sp_arr.min())
            raw_max = max(p_arr.max(), sp_arr.max())
            span    = raw_max - raw_min
            margin  = max(span * 0.10, 5.0)   # mínimo 5 unidades de margem
            needed_min = raw_min - margin
            needed_max = raw_max + margin

            # 2. Só expande — nunca encolhe
            if self._y1_min is None:
                self._y1_min = needed_min
                self._y1_max = needed_max
            else:
                self._y1_min = min(self._y1_min, needed_min)
                self._y1_max = max(self._y1_max, needed_max)

            self.ax1.set_ylim(self._y1_min, self._y1_max)

            # ── PWM: autoscale + mínimo de 50 unidades ───────────────────
            self.line_pwm.set_data(t_arr, pw_arr)
            self.ax2.relim()
            self.ax2.autoscale_view()
            pw_low, pw_high = self.ax2.get_ylim()
            if (pw_high - pw_low) < 50.0:
                mid = (pw_high + pw_low) / 2.0
                self.ax2.set_ylim(max(-10, mid - 25.0), min(270, mid + 25.0))

            self.canvas.draw_idle()
        except Exception:
            pass

    def start_control(self):
        if self.connected:
            self.send_manual_parameters()
            time.sleep(0.1)
            self.send_command("START")
            self.running = True
            self.update_gui_state()
            self.log_message("Controle iniciado", "success")

    def stop_control(self):
        if self.connected and self.running:
            self.send_command("STOP")
            self.running = False
            self.update_gui_state()
            self.log_message("Controle parado", "info")

    def send_manual_parameters(self):
        if self.connected:
            try:
                kp = self.kp_scale.get()
                ki = self.ki_scale.get()
                kd = self.kd_scale.get()
                sp = self.setpoint_scale.get()
                self.send_command(
                    f"SETPID:{kp:.8f},{ki:.8f},{kd:.8f},{sp:.2f}")
            except Exception:
                pass

    def send_command(self, command):
        if self.connected and self.serial_port and self.serial_port.is_open:
            try:
                with self.serial_lock:
                    self.serial_port.write(f"{command}\n".encode())
                self.log_message(f"→ {command}", "info")
            except Exception as e:
                self.log_message(f"Erro envio: {str(e)}", "error")

    def request_parameters(self):
        if self.connected:
            self.send_command("GETPARAMS")

    def start_logging(self):
        if not self.data_logging:
            mode_str = "velocidade" if self.mode == "velocity" else "posicao"
            filename = filedialog.asksaveasfilename(
                defaultextension=".csv",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
                initialfile=f"pid_{mode_str}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            )
            if filename:
                try:
                    self.csv_file   = open(filename, 'w', newline='')
                    self.csv_writer = csv.writer(self.csv_file)
                    primary_col = "RPM" if self.mode == "velocity" else "Theta(deg)"
                    sp_col      = "Setpoint(RPM)" if self.mode == "velocity" \
                                  else "Alvo(deg)"
                    self.csv_writer.writerow([
                        'Timestamp', 'Time(s)', sp_col, primary_col,
                        'Voltage(V)', 'PWM', 'Error', 'Kp', 'Ki', 'Kd'
                    ])
                    self.data_logging = True
                    self.update_gui_state()
                    self.log_message(f"Log: {filename}", "success")
                except Exception as e:
                    messagebox.showerror("Erro", f"Não foi possível criar arquivo:\n{str(e)}")

    def stop_logging(self):
        if self.data_logging:
            self.data_logging = False
            if self.csv_file:
                self.csv_file.close()
                self.csv_file   = None
                self.csv_writer = None
            self.update_gui_state()
            self.log_message("Log parado", "info")

    def update_gui_state(self):
        try:
            self.start_btn.config(
                state=tk.NORMAL if not self.running else tk.DISABLED)
            self.stop_btn.config(
                state=tk.NORMAL if self.running else tk.DISABLED)
            self.log_start_btn.config(
                state=tk.NORMAL if not self.data_logging else tk.DISABLED)
            self.log_stop_btn.config(
                state=tk.NORMAL if self.data_logging else tk.DISABLED)
        except Exception:
            pass

    def return_to_connection(self):
        self.stop_control()
        self.connected = False
        time.sleep(0.2)

        while not self.data_queue.empty():
            try:
                self.data_queue.get_nowait()
            except Exception:
                break

        if self.data_logging and self.csv_file:
            try:
                self.csv_file.close()
                self.csv_file   = None
                self.csv_writer = None
            except Exception:
                pass

        if messagebox.askyesno("Voltar", "Deseja voltar para a tela de conexão?"):
            sp = self.serial_port
            self.serial_port = None
            time.sleep(0.3)
            try:
                if sp and sp.is_open:
                    sp.close()
            except Exception:
                pass

            for widget in self.root.winfo_children():
                widget.destroy()

            ConnectionScreen(
                self.root,
                lambda port, mode: BasePIDGUI(self.root, port, mode)
            )

    def log_message(self, message, msg_type="info"):
        try:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log_text.insert(tk.END, f"[{timestamp}] ", "timestamp")
            self.log_text.insert(tk.END, f"{message}\n", msg_type)
            self.log_text.see(tk.END)
            self.log_text.tag_config("timestamp", foreground="#95a5a6")
            self.log_text.tag_config("success",   foreground="#2ecc71")
            self.log_text.tag_config("error",     foreground="#e74c3c")
            self.log_text.tag_config("warning",   foreground="#f39c12")
            self.log_text.tag_config("info",      foreground="#3498db")
            if int(self.log_text.index('end-1c').split('.')[0]) > 50:
                self.log_text.delete('1.0', '2.0')
        except Exception:
            pass


# ============================================================================
# FUNÇÃO PRINCIPAL
# ============================================================================

def main():
    root = tk.Tk()

    def show_main_screen(serial_port, mode):
        for widget in root.winfo_children():
            widget.destroy()
        BasePIDGUI(root, serial_port, mode)

    connection_screen = ConnectionScreen(root, show_main_screen)

    def on_closing():
        try:
            if connection_screen and connection_screen.serial_port:
                connection_screen.serial_port.close()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()