 # ============================================================================
# SISTEMA DE MONITORAMENTO CLÍNICO IOT
# ============================================================================
import machine, dht, time, network, ujson, asyncio, gc

MAX30102_ADDRESS = 0x57
REG_INT_ENABLE   = 0x02
REG_FIFO_WR_PTR  = 0x04
REG_FIFO_OVF_CNT = 0x05
REG_FIFO_RD_PTR  = 0x06
REG_FIFO_DATA    = 0x07
REG_FIFO_CONFIG  = 0x08
REG_MODE_CONFIG  = 0x09
REG_SPO2_CONFIG  = 0x0A
REG_LED1_PA      = 0x0C
REG_LED2_PA      = 0x0D

PIN_DHT11        = 4
PIN_HX711_DT     = 32
PIN_HX711_SCK    = 33
PIN_MQ_ADC       = 34
PIN_LED_VERDE    = 18
PIN_LED_AMARELO  = 19
PIN_LED_VERMELHO = 23
PIN_I2C_SDA      = 21
PIN_I2C_SCL      = 22
PIN_BOTAO        = 13
PIN_SENSOR_IR    = 15
PIN_ALCOOL       = 25
PIN_OXIGENIO     = 26
PIN_COOLER       = 27
PIN_BUZZER       = 14
PIN_LAMPADA      = 5

# ============================================================================
# FILTROS
# ============================================================================
class DCBlocker:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.w = 0
    def filter(self, x):
        w_next = x + self.alpha * self.w
        out = w_next - self.w
        self.w = w_next
        return out

class EWMAFilter:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.val = None
    def filter(self, x):
        if self.val is None:
            self.val = x
        else:
            self.val = self.alpha * x + (1.0 - self.alpha) * self.val
        return self.val

class MedianFilter:
    def __init__(self, n=5):
        self.buf = []
        self.n = n
    def filter(self, x):
        self.buf.append(x)
        if len(self.buf) > self.n:
            self.buf.pop(0)
        s = sorted(self.buf)
        return s[len(s) // 2]

# ============================================================================
# DETECTOR DE BPM — intervalo inter-batida com média aparada (trimmed mean)
# ============================================================================
class BPMDetector:
    def __init__(self):
        self.prev       = 0.0
        self.env        = EWMAFilter(alpha=0.005)
        self.beat_times = []
        self.last_beat  = 0
        self.bpm        = 0

    def update(self, ac_val):
        self.env.filter(abs(ac_val))
        envelope  = max(self.env.val, 1.0) if self.env.val else 1.0
        threshold = envelope * 0.5

        is_beat = False
        if self.prev <= threshold < ac_val:
            now = time.ticks_ms()
            if time.ticks_diff(now, self.last_beat) > 333:
                self.last_beat = now
                self.beat_times.append(now)
                if len(self.beat_times) > 12:
                    self.beat_times.pop(0)
                is_beat = True

        self.prev = ac_val

        # Necessita de pelo menos 5 batidas para calcular intervalos fiáveis
        if len(self.beat_times) >= 5:
            ivs = [time.ticks_diff(self.beat_times[i+1], self.beat_times[i])
                   for i in range(len(self.beat_times) - 1)]
            ivs.sort()
            n   = len(ivs)
            cut = max(1, n // 4)
            trimmed = ivs[cut: n - cut]
            if trimmed:
                avg_iv = sum(trimmed) / len(trimmed)
                inst   = int(60000 / avg_iv) if avg_iv > 0 else 0
                if 40 <= inst <= 180:
                    if self.bpm == 0:
                        self.bpm = inst
                    else:
                        self.bpm = int(self.bpm * 0.90 + inst * 0.10)

        return is_beat

# ============================================================================
# DRIVER MAX30102 — Stop-Start (clone-safe)
# ============================================================================
class MAX30102:
    def __init__(self, scl_pin=22, sda_pin=21, addr=MAX30102_ADDRESS):
        self.i2c  = machine.SoftI2C(scl=machine.Pin(scl_pin),
                                     sda=machine.Pin(sda_pin), freq=100000)
        self.addr = addr

        self.ir_dc   = DCBlocker(0.98)
        self.red_dc  = DCBlocker(0.98)
        self.ir_lp   = EWMAFilter(0.3)
        self.red_lp  = EWMAFilter(0.3)
        self.ir_med  = MedianFilter(5)
        self.red_med = MedianFilter(5)

        self.bpm_det     = BPMDetector()
        self.ir_win      = []
        self.red_win     = []
        self.ir_raw_win  = []
        self.red_raw_win = []
        self.win_size    = 100

        self.heart_rate  = 0
        self.spo2        = 0
        self.last_raw_ir = 0
        self.last_raw_red= 0
        self.signal_quality = "AUSENTE"

        time.sleep_ms(250)
        self.i2c.scan()
        self.init_sensor()

    def _rd(self, reg, n=1):
        self.i2c.writeto(self.addr, bytes([reg]))
        return self.i2c.readfrom(self.addr, n)

    def _wr(self, reg, val):
        if isinstance(val, int):
            self.i2c.writeto(self.addr, bytes([reg, val]))
        else:
            self.i2c.writeto(self.addr, bytes([reg]) + val)

    def _reset_filters(self):
        self.ir_dc   = DCBlocker(0.98)
        self.red_dc  = DCBlocker(0.98)
        self.ir_lp   = EWMAFilter(0.3)
        self.red_lp  = EWMAFilter(0.3)
        self.ir_med  = MedianFilter(5)
        self.red_med = MedianFilter(5)
        self.bpm_det = BPMDetector()
        self.ir_win.clear()
        self.red_win.clear()
        self.ir_raw_win.clear()
        self.red_raw_win.clear()

    def init_sensor(self):
        try:
            self._wr(REG_MODE_CONFIG,  0x40)   # reset
            time.sleep_ms(200)
            self._wr(REG_INT_ENABLE,   0xC0)
            self._wr(REG_FIFO_CONFIG,  0x0F)
            # ---------------------------------------------------------------
            # LED1 (RED) e LED2 (IR): 0x3F ≈ 12 mA
            # Aumentado de 0x1F (6 mA) para melhorar perfusão em clones.
            # Valores mais altos (0x5F, 0x7F) melhoram sinal mas geram calor.
            # ---------------------------------------------------------------
            self._wr(REG_LED1_PA,      0x3F)
            self._wr(REG_LED2_PA,      0x3F)
            # SPO2_CONFIG: ADC 4096 nA, SR 100 Hz, PW 411 µs (18-bit)
            self._wr(REG_SPO2_CONFIG,  0x27)
            self._wr(REG_MODE_CONFIG,  0x03)   # SpO2 mode (RED + IR)
            self._wr(REG_FIFO_OVF_CNT, 0x00)
            self._wr(REG_FIFO_RD_PTR,  0x00)
            self._reset_filters()
            print("✓ MAX30102 configurado")
            return True
        except Exception as e:
            print(f"MAX30102 init erro: {e}")
            return False

    def update(self):
        try:
            w = self._rd(REG_FIFO_WR_PTR)[0]
            r = self._rd(REG_FIFO_RD_PTR)[0]
            o = self._rd(REG_FIFO_OVF_CNT)[0]

            n = (w - r) & 0x1F
            if o > 0:
                n = 16
                self._wr(REG_FIFO_OVF_CNT, 0x00)
            if n == 0:
                return False

            raw = self._rd(REG_FIFO_DATA, n * 6)
            self._wr(REG_FIFO_RD_PTR, (r + n) & 0x1F)

            for i in range(n):
                b = raw[i*6:(i+1)*6]
                red_v = ((b[0] & 0x03) << 16) | (b[1] << 8) | b[2]
                ir_v  = ((b[3] & 0x03) << 16) | (b[4] << 8) | b[5]

                self.last_raw_ir  = ir_v
                self.last_raw_red = red_v

                # Dedo ausente: IR < 20 000 (fundo de escala sem perfusão)
                if ir_v < 20000:
                    self.signal_quality = "AUSENTE"
                    self.heart_rate = 0
                    self.spo2 = 0
                    self._reset_filters()
                    continue

                # Janela raw para DC (média da janela inteira — estável)
                self.ir_raw_win.append(ir_v)
                self.red_raw_win.append(red_v)
                if len(self.ir_raw_win) > self.win_size:
                    self.ir_raw_win.pop(0)
                    self.red_raw_win.pop(0)

                # Pipeline de filtragem AC
                ir_ac  = self.ir_dc.filter(ir_v)
                red_ac = self.red_dc.filter(red_v)
                ir_s   = self.ir_med.filter(self.ir_lp.filter(ir_ac))
                red_s  = self.red_med.filter(self.red_lp.filter(red_ac))

                self.bpm_det.update(ir_s)
                if self.bpm_det.bpm > 0:
                    self.heart_rate = self.bpm_det.bpm

                self.ir_win.append(ir_s)
                self.red_win.append(red_s)
                if len(self.ir_win) > self.win_size:
                    self.ir_win.pop(0)
                    self.red_win.pop(0)

                if len(self.ir_win) == self.win_size:
                    self._calc_spo2()

            return True
        except Exception as e:
            print(f"MAX30102 update erro: {e}")
            return False

    def _calc_spo2(self):
        vpp_ir  = max(self.ir_win)  - min(self.ir_win)
        vpp_red = max(self.red_win) - min(self.red_win)

        if vpp_ir < 50:
            self.signal_quality = "FRACO"
            return
        self.signal_quality = "EXCELENTE" if vpp_ir > 200 else "OK"

        dc_ir  = sum(self.ir_raw_win)  / len(self.ir_raw_win)
        dc_red = sum(self.red_raw_win) / len(self.red_raw_win)

        if dc_ir > 1000 and dc_red > 1000 and vpp_ir > 0 and vpp_red > 0:
            r = (vpp_red / dc_red) / (vpp_ir / dc_ir)
            if 0.3 <= r <= 2.0:
                inst = int(110 - 25 * r)
                # Piso clínico a 75 % — abaixo disto o sensor não é fiável
                inst = max(75, min(100, inst))
                if self.spo2 == 0:
                    self.spo2 = inst
                else:
                    # -------------------------------------------------------
                    # Rejeição de outliers: ignora se diferença > 12 %
                    # (era 6 % — demasiado apertado para clones).
                    # EWMA: nova leitura vale 15 % (era 7 % — muito lento).
                    # -------------------------------------------------------
                    if abs(inst - self.spo2) <= 12:
                        self.spo2 = int(self.spo2 * 0.85 + inst * 0.15)

# ============================================================================
# HX711 — driver síncrono + wrapper async seguro
# ============================================================================
class HX711:
    def __init__(self, dout, pd_sck, gain=128):
        self.p_dout = machine.Pin(dout, machine.Pin.IN)
        self.p_sck  = machine.Pin(pd_sck, machine.Pin.OUT, value=0)
        self.gain   = gain
        self.OFFSET = 0
        self.SCALE  = 1
        self._ewma  = None
        self.set_gain(gain)

    def set_gain(self, gain):
        if gain == 128:   self.gain_bits = 1
        elif gain == 64:  self.gain_bits = 3
        elif gain == 32:  self.gain_bits = 2
        else:             self.gain_bits = 1

    def is_ready(self):
        return self.p_dout.value() == 0

    def read(self):
        while not self.is_ready():
            machine.idle()
        raw = 0
        # Desactiva IRQ durante toda a sequência de relógio.
        # O ESP32 @ 240 MHz é rápido demais para o HX711 sem delay;
        # interrupções do asyncio a meio dos 25 pulsos corrompem os bits
        # e causam leituras espúrias crescentes.
        irq = machine.disable_irq()
        try:
            for _ in range(24):
                self.p_sck.value(1)
                time.sleep_us(1)          # t_high mín. HX711 = 0.2 µs
                raw = (raw << 1) | self.p_dout.value()
                self.p_sck.value(0)
                time.sleep_us(1)          # t_low  mín. HX711 = 0.2 µs
            for _ in range(self.gain_bits):
                self.p_sck.value(1)
                time.sleep_us(1)
                self.p_sck.value(0)
                time.sleep_us(1)
        finally:
            machine.enable_irq(irq)
        if raw & 0x800000:
            raw -= 0x1000000
        return raw

    def read_average(self, times=10):
        total = 0
        for _ in range(times):
            total += self.read()
        return total / times

    def tare(self, times=15):
        self.OFFSET = self.read_average(times)

    def get_value(self, times=1):
        return self.read_average(times) - self.OFFSET

    def get_units(self, times=1):
        return self.get_value(times) / self.SCALE

    def set_scale(self, scale):
        self.SCALE = scale

    async def get_units_async(self, times=10):
        """Wrapper async — cede ao event loop entre amostras."""
        vals = []
        for _ in range(times):
            deadline = time.ticks_add(time.ticks_ms(), 500)
            while not self.is_ready():
                if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                    break
                await asyncio.sleep_ms(5)
            if self.is_ready():
                vals.append(self.read())
            await asyncio.sleep_ms(10)
        if len(vals) < 3:
            return None
        vals.sort()
        trimmed = vals[1:-1]
        avg   = sum(trimmed) / len(trimmed)
        raw_g = (avg - self.OFFSET) / self.SCALE
        if self._ewma is None:
            self._ewma = raw_g
        else:
            self._ewma = self._ewma * 0.7 + raw_g * 0.3
        return self._ewma

# ============================================================================
# ESTADO GLOBAL
# ============================================================================
estado_sistema = {
    "temperatura": 0.0, "humidade": 0.0, "gas_bruto": 0,
    "soro_percentagem": 0.0, "soro_gramas": 0.0,
    "status_oxigenio": 0, "status_cooler": 0, "status_alcool": 0,
    "status_lampada": 0, "status_botao": "SOLTO", "status_ir": "SEM SINAL",
    "alarme_ativo": 0, "max_ir": 0, "max_red": 0,
    "bpm": 0, "oxigenacao": 0, "paciente_status": "AUSENTE"
}

last_remote_sensor_time = 0
REMOTE_SENSOR_STALE_MS  = 3000

# Conjunto das condições que estavam activas quando o utilizador silenciou.
# O alarme só permanece mudo enquanto não aparecer uma condição NOVA.
_condicoes_silenciadas = set()

def _get_condicoes(estado):
    """Devolve o conjunto de condições críticas activas no momento."""
    c = set()
    if estado["temperatura"] > 35.0:           c.add("temp")
    if 0 < estado["soro_percentagem"] < 15.0:  c.add("soro")
    if 0 < estado["oxigenacao"] < 92:          c.add("spo2")
    if estado["bpm"] > 120:                    c.add("bpm")
    return c

out_oxigenio = machine.Pin(PIN_OXIGENIO,  machine.Pin.OUT, value=0)
out_cooler   = machine.Pin(PIN_COOLER,    machine.Pin.OUT, value=0)
out_buzzer   = machine.Pin(PIN_BUZZER,    machine.Pin.OUT, value=0)
out_alcool   = machine.Pin(PIN_ALCOOL,    machine.Pin.OUT, value=0)
out_lampada  = machine.Pin(PIN_LAMPADA,   machine.Pin.OUT, value=0)
led_verde    = machine.Pin(PIN_LED_VERDE,    machine.Pin.OUT, value=0)
led_amarelo  = machine.Pin(PIN_LED_AMARELO,  machine.Pin.OUT, value=0)
led_vermelho = machine.Pin(PIN_LED_VERMELHO, machine.Pin.OUT, value=0)
in_botao     = machine.Pin(PIN_BOTAO,     machine.Pin.IN, machine.Pin.PULL_UP)
in_ir        = machine.Pin(PIN_SENSOR_IR, machine.Pin.IN, machine.Pin.PULL_UP)

sensor_max = None
try:
    sensor_max = MAX30102(scl_pin=PIN_I2C_SCL, sda_pin=PIN_I2C_SDA)
except Exception as e:
    print(f"MAX30102 falhou: {e}")

sensor_dht = None
try:
    sensor_dht = dht.DHT11(machine.Pin(PIN_DHT11))
    print("✓ DHT11 OK")
except Exception:
    print("⚠ DHT11 indisponível")

mq_adc = None
try:
    mq_adc = machine.ADC(machine.Pin(PIN_MQ_ADC))
    mq_adc.atten(machine.ADC.ATTN_11DB)
    print("✓ MQ OK")
except Exception:
    print("⚠ MQ indisponível")

hx = None
try:
    hx = HX711(dout=PIN_HX711_DT, pd_sck=PIN_HX711_SCK)
    hx.set_scale(-435.0)
    hx.tare(20)
    print("✓ HX711 OK")
except Exception as e:
    print(f"⚠ HX711 falhou: {e}")

ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid="Hospital-IoT", password="hospital123", authmode=3)
print("✓ WiFi AP Hospital-IoT activo")

# ============================================================================
# HTML — armazenado como string e codificado uma vez para bytes
# ============================================================================
_HTML_STR = """<!DOCTYPE html>
<html lang="pt">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Painel de Monitoramento Clínico IoT Avançado</title>
    <style>
        :root {
            --bg-hospital: #f4f7fa; --surface: #ffffff; --text-primary: #1e293b;
            --text-secondary: #64748b; --primary: #0284c7; --primary-hover: #0369a1;
            --success: #0d9488; --danger: #e11d48; --warning: #d97706; --border: #e2e8f0;
        }
        * { margin: 0; padding: 0; box-sizing: border-box; font-family: 'Inter', sans-serif; }
        body { background-color: var(--bg-hospital); color: var(--text-primary); padding: 24px; }
        .container { max-width: 1280px; margin: 0 auto; }
        header { display: flex; justify-content: space-between; align-items: center; padding: 16px 24px; background: var(--surface); border-radius: 16px; border: 1px solid var(--border); margin-bottom: 24px; box-shadow: 0 1px 3px rgba(0,0,0,0.05); }
        h1 { font-size: 20px; font-weight: 700; }
        .status-badge { display: flex; align-items: center; gap: 8px; padding: 6px 14px; border-radius: 9999px; font-size: 13px; font-weight: 600; background: #f1f5f9; }
        .grid-cards { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 24px; margin-bottom: 24px; }
        .card { background: var(--surface); border-radius: 16px; padding: 24px; border: 1px solid var(--border); position: relative; transition: all 0.3s ease; }
        .card-header-icon { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }
        .card-title { font-size: 13px; font-weight: 600; color: var(--text-secondary); text-transform: uppercase; letter-spacing: 0.5px; }
        .card-value { font-size: 32px; font-weight: 700; color: var(--text-primary); display: flex; align-items: baseline; gap: 4px; }
        .card-unit { font-size: 16px; font-weight: 500; color: var(--text-secondary); }
        .progress-container { width: 100%; height: 6px; background: #e2e8f0; border-radius: 9999px; margin-top: 10px; overflow: hidden; }
        .progress-bar { height: 100%; width: 0%; background: var(--primary); transition: width 0.5s ease; }
        .card-critico { border-color: var(--danger) !important; background: #fff5f5; animation: pulse 2s infinite; }
        .card-critico .card-title { color: var(--danger); }
        .card-paciente-ativo { border-color: var(--success) !important; background: #f0fdf4; }
        .section-title { font-size: 16px; font-weight: 700; color: var(--text-secondary); margin-bottom: 16px; margin-top: 12px; }
        .grid-controls { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 24px; }
        .card-control { display: flex; align-items: center; justify-content: space-between; padding: 20px 24px; }
        .control-info { display: flex; align-items: center; gap: 16px; }
        .switch-btn { padding: 10px 20px; border-radius: 10px; border: none; font-weight: 600; font-size: 13px; cursor: pointer; min-width: 110px; text-align: center; }
        .btn-off { background: #f1f5f9; color: var(--text-primary); border: 1px solid var(--border); }
        .btn-on { background: var(--danger); color: white; }
        .btn-normal-on { background: var(--primary); color: white; }
        .btn-disabled { background: #f8fafc; color: #cbd5e1; cursor: not-allowed; border: 1px solid #e2e8f0; }
        .icon-svg { width: 24px; height: 24px; fill: none; stroke: currentColor; stroke-width: 2; stroke-linecap: round; stroke-linejoin: round; color: var(--text-secondary); }
        @keyframes pulse { 0% { box-shadow: 0 0 0 0 rgba(225,29,72,0.4); } 70% { box-shadow: 0 0 0 12px rgba(225,29,72,0); } 100% { box-shadow: 0 0 0 0 rgba(225,29,72,0); } }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>Painel Clínico de Monitoramento & Biossegurança</h1>
            <div id="sistema-status" class="status-badge" style="color: var(--success);">AP OPERACIONAL</div>
        </header>

        <div class="section-title">Dados Biométricos do Paciente</div>
        <div class="grid-cards">
            <div class="card" id="card-paciente">
                <div class="card-header-icon">
                    <div class="card-title">Status do Paciente</div>
                    <svg class="icon-svg" viewBox="0 0 24 24"><path d="M19 14c1.49-1.46 3-3.21 3-5.5A5.5 5.5 0 0 0 16.5 3c-1.76 0-3 .5-4.5 2-1.5-1.5-2.74-2-4.5-2A5.5 5.5 0 0 0 2 8.5c0 2.3 1.5 4.05 3 5.5l7 7Z"/></svg>
                </div>
                <div class="card-value" id="val-paciente" style="font-size: 26px;">MONITORANDO...</div>
                <div style="font-size: 12px; color: var(--text-secondary); margin-top: 8px;">Deteção por Limiar IR</div>
            </div>
            <div class="card" id="card-bpm">
                <div class="card-header-icon">
                    <div class="card-title">Frequência Cardíaca</div>
                    <svg class="icon-svg" style="color: var(--danger);" viewBox="0 0 24 24"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg>
                </div>
                <div class="card-value"><span id="val-bpm">0</span><span class="card-unit">BPM</span></div>
                <div class="progress-container"><div id="prog-bpm" class="progress-bar" style="background:var(--danger);"></div></div>
            </div>
            <div class="card" id="card-spo2">
                <div class="card-header-icon">
                    <div class="card-title">Saturação de O₂</div>
                    <svg class="icon-svg" style="color: var(--primary);" viewBox="0 0 24 24"><path d="M12 2v20M17 5H7M19 9H5"/></svg>
                </div>
                <div class="card-value"><span id="val-spo2">0</span><span class="card-unit">% SpO₂</span></div>
                <div class="progress-container"><div id="prog-spo2" class="progress-bar" style="background:var(--primary);"></div></div>
            </div>
            <div class="card">
                <div class="card-header-icon">
                    <div class="card-title">Sinal Infravermelho (IR)</div>
                    <svg class="icon-svg" viewBox="0 0 24 24"><path d="M2 12h3l2 5 4-10 3 8 2-3h3"/></svg>
                </div>
                <div class="card-value"><span id="val-max-ir">0</span><span class="card-unit">raw</span></div>
                <div class="progress-container"><div id="prog-max-ir" class="progress-bar" style="background:#0284c7;"></div></div>
            </div>
            <div class="card">
                <div class="card-header-icon">
                    <div class="card-title">Canal Vermelho (RED)</div>
                    <svg class="icon-svg" viewBox="0 0 24 24"><circle cx="12" cy="12" r="10"/><path d="m12 8 4 4-4 4-4-4Z"/></svg>
                </div>
                <div class="card-value"><span id="val-max-red">0</span><span class="card-unit">raw</span></div>
                <div class="progress-container"><div id="prog-max-red" class="progress-bar" style="background:#e11d48;"></div></div>
            </div>
        </div>

        <div class="section-title">Telemetria de Ambiente e Insumos</div>
        <div class="grid-cards">
            <div class="card" id="card-temp">
                <div class="card-header-icon">
                    <div class="card-title">Temperatura</div>
                    <svg class="icon-svg" viewBox="0 0 24 24"><path d="M14 4v10.54a4 4 0 1 1-4 0V4a2 2 0 0 1 4 0Z"/></svg>
                </div>
                <div class="card-value"><span id="val-temp">--.-</span><span class="card-unit">°C</span></div>
                <div class="progress-container"><div id="prog-temp" class="progress-bar"></div></div>
            </div>
            <div class="card" id="card-hum">
                <div class="card-header-icon">
                    <div class="card-title">Humidade</div>
                    <svg class="icon-svg" viewBox="0 0 24 24"><path d="M12 22a7 7 0 0 0 7-7c0-4.3-7-13-7-13S5 10.7 5 15a7 7 0 0 0 7 7Z"/></svg>
                </div>
                <div class="card-value"><span id="val-hum">--.-</span><span class="card-unit">%</span></div>
                <div class="progress-container"><div id="prog-hum" class="progress-bar" style="background: #0d9488;"></div></div>
            </div>
            <div class="card" id="card-gas">
                <div class="card-header-icon">
                    <div class="card-title">Qualidade do Ar</div>
                    <svg class="icon-svg" viewBox="0 0 24 24"><path d="M8.5 14.5A2.5 2.5 0 0 0 11 12c0-1.38-.5-2-1-3-1.072-2.143-.224-4.054 2-6 .5 2.5 2 4.9 4 6.5 2 1.6 3 3.5 3 5.5a7 7 0 1 1-14 0c0-1.153.433-2.294 1-3a2.5 2.5 0 0 0 2.5 2.5Z"/></svg>
                </div>
                <div class="card-value"><span id="val-gas">----</span><span class="card-unit">ADC</span></div>
                <div class="progress-container"><div id="prog-gas" class="progress-bar" style="background: var(--warning);"></div></div>
            </div>
            <div class="card" id="card-soro">
                <div class="card-header-icon">
                    <div class="card-title">Nível Soro</div>
                    <svg class="icon-svg" viewBox="0 0 24 24"><path d="M10 22h4M12 2v20M12 2a4 4 0 0 1 4 4v3a4 4 0 0 1-4 4 4 4 0 0 1-4-4V6a4 4 0 0 1 4-4Z"/></svg>
                </div>
                <div class="card-value"><span id="val-soro">---</span><span class="card-unit">%</span></div>
                <div style="font-size: 12px; color: var(--text-secondary);">Massa: <span id="val-soro-g">---.-</span>g</div>
                <div class="progress-container"><div id="prog-soro" class="progress-bar" style="background: #a855f7;"></div></div>
            </div>
        </div>

        <div class="section-title">Status dos Dispositivos de Entrada Locais</div>
        <div class="grid-cards" style="grid-template-columns: repeat(auto-fit, minmax(450px, 1fr));">
            <div class="card" style="padding: 16px 24px; display: flex; justify-content: space-between; align-items: center;">
                <div>
                    <div class="card-title">Botão de Chamada</div>
                    <div style="font-size: 20px; font-weight:700; margin-top:4px;" id="val-botao">SOLTO</div>
                </div>
            </div>
            <div class="card" style="padding: 16px 24px; display: flex; justify-content: space-between; align-items: center;">
                <div>
                    <div class="card-title">Presença IR Dispensador</div>
                    <div style="font-size: 20px; font-weight:700; margin-top:4px;" id="val-ir">SEM SINAL</div>
                </div>
            </div>
        </div>

        <div class="section-title">Controle Ativo de Atuadores do Leito</div>
        <div class="grid-controls">
            <div class="card card-control">
                <div class="control-info">
                    <svg class="icon-svg" viewBox="0 0 24 24" style="color:var(--primary);"><path d="M12 2v20M17 5H7M19 9H5M21 13H3"/></svg>
                    <div><div class="control-label">Válvula de Oxigénio</div><div class="control-desc"></div></div>
                </div>
                <button id="btn-oxigenio" class="switch-btn btn-off" onclick="enviarComando('oxigenio')">LIGAR</button>
            </div>
            <div class="card card-control">
                <div class="control-info">
                    <svg class="icon-svg" viewBox="0 0 24 24" style="color:var(--primary);"><path d="M12 12m-9 0a9 9 0 1 0 18 0a9 9 0 1 0 -18 0M12 12L12 3M12 12L3 12"/></svg>
                    <div><div class="control-label">Ventilador</div><div class="control-desc"></div></div>
                </div>
                <button id="btn-cooler" class="switch-btn btn-off" onclick="enviarComando('cooler')">LIGAR</button>
            </div>
            <div class="card card-control">
                <div class="control-info">
                    <svg class="icon-svg" viewBox="0 0 24 24" style="color:var(--success);"><path d="M12 22a7 7 0 0 0 7-7c0-4.3-7-13-7-13S5 10.7 5 15a7 7 0 0 0 7 7Z"/></svg>
                    <div><div class="control-label">Dispensar Álcool Gel</div><div class="control-desc"></div></div>
                </div>
                <button id="btn-alcool" class="switch-btn btn-off" onclick="enviarComando('alcool')">PULSAR</button>
            </div>
            <div class="card card-control">
                <div class="control-info">
                    <svg class="icon-svg" viewBox="0 0 24 24" style="color:var(--warning);"><path d="M15 14c.2-1 .7-1.7 1.5-2.5 1-.9 1.5-2.2 1.5-3.5A6 6 0 0 0 6 8c0 1 .5 2.2 1.5 3.1.7.7 1.3 1.5 1.5 2.5h6Z"/><path d="M9 18h6M10 22h4"/></svg>
                    <div><div class="control-label">Lâmpada do Quarto</div><div class="control-desc"></div></div>
                </div>
                <button id="btn-lampada" class="switch-btn btn-off" onclick="enviarComando('lampada')">LIGAR</button>
            </div>
            <div class="card card-control" id="card-alarme">
                <div class="control-info">
                    <svg class="icon-svg" viewBox="0 0 24 24" style="color:var(--danger);"><path d="M10.3 21a1.94 1.94 0 0 0 3.4 0M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9"/></svg>
                    <div><div class="control-label">Alarme de Emergência</div><div class="control-desc"></div></div>
                </div>
                <button id="btn-alarme" class="switch-btn btn-disabled" onclick="enviarComando('alarme')" disabled>SEGURO</button>
            </div>
        </div>
    </div>

    <script>
        async function buscarDados() {
            try {
                const response = await fetch('/api/data');
                const data = await response.json();
                document.getElementById('val-max-ir').innerText = data.max_ir;
                document.getElementById('val-max-red').innerText = data.max_red;
                document.getElementById('val-paciente').innerText = data.paciente_status;
                document.getElementById('val-bpm').innerText = data.bpm;
                document.getElementById('val-spo2').innerText = data.oxigenacao;
                document.getElementById('prog-bpm').style.width = Math.min((data.bpm / 200) * 100, 100) + '%';
                document.getElementById('prog-spo2').style.width = data.oxigenacao + '%';
                document.getElementById('prog-max-ir').style.width = Math.min((data.max_ir / 120000) * 100, 100) + '%';
                document.getElementById('prog-max-red').style.width = Math.min((data.max_red / 120000) * 100, 100) + '%';
                const cardPac = document.getElementById('card-paciente');
                if(data.paciente_status === 'EXCELENTE' || data.paciente_status === 'OK') {
                    cardPac.className = 'card card-paciente-ativo';
                } else {
                    cardPac.className = 'card';
                }
                document.getElementById('card-bpm').className = (data.bpm > 120 || (data.bpm < 45 && data.bpm > 0)) ? 'card card-critico' : 'card';
                document.getElementById('card-spo2').className = (data.oxigenacao < 92 && data.oxigenacao > 0) ? 'card card-critico' : 'card';
                document.getElementById('val-temp').innerText = data.temperatura.toFixed(1);
                document.getElementById('val-hum').innerText = data.humidade.toFixed(1);
                document.getElementById('val-gas').innerText = data.gas_bruto;
                document.getElementById('val-soro').innerText = data.soro_percentagem.toFixed(0);
                document.getElementById('val-soro-g').innerText = data.soro_gramas.toFixed(0);
                document.getElementById('val-botao').innerText = data.status_botao;
                document.getElementById('val-ir').innerText = data.status_ir;
                document.getElementById('prog-temp').style.width = Math.min((data.temperatura / 50) * 100, 100) + '%';
                document.getElementById('prog-hum').style.width = data.humidade + '%';
                document.getElementById('prog-gas').style.width = Math.min((data.gas_bruto / 4095) * 100, 100) + '%';
                document.getElementById('prog-soro').style.width = data.soro_percentagem + '%';
                document.getElementById('card-temp').className = data.temperatura > 35.0 ? 'card card-critico' : 'card';
                document.getElementById('card-soro').className = (data.soro_percentagem < 15.0 && data.soro_percentagem > 0) ? 'card card-critico' : 'card';
                if(data.alarme_ativo) {
                    document.getElementById('card-alarme').classList.add('card-critico');
                    const btn = document.getElementById('btn-alarme');
                    btn.className = 'switch-btn btn-on'; btn.innerText = 'DESATIVAR'; btn.disabled = false;
                } else {
                    document.getElementById('card-alarme').classList.remove('card-critico');
                    const btn = document.getElementById('btn-alarme');
                    btn.className = 'switch-btn btn-disabled'; btn.innerText = 'SEGURO'; btn.disabled = true;
                }
                atualizarBotao('btn-oxigenio', data.status_oxigenio);
                atualizarBotao('btn-cooler', data.status_cooler);
                atualizarBotao('btn-lampada', data.status_lampada);
                const btnAlcool = document.getElementById('btn-alcool');
                if(data.status_alcool) { btnAlcool.className = 'switch-btn btn-on'; btnAlcool.innerText = 'DISPENSANDO...'; }
                else { btnAlcool.className = 'switch-btn btn-normal-on'; btnAlcool.innerText = 'PULSAR'; }
                document.getElementById('sistema-status').style.color = 'var(--success)';
            } catch (error) {
                document.getElementById('sistema-status').style.color = 'var(--danger)';
            }
        }
        function atualizarBotao(id, estadoAtivo) {
            const btn = document.getElementById(id);
            if(estadoAtivo) { btn.className = 'switch-btn btn-on'; btn.innerText = 'DESLIGAR'; }
            else { btn.className = 'switch-btn btn-normal-on'; btn.innerText = 'LIGAR'; }
        }
        async function enviarComando(atuador) {
            let estado = 0;
            const btn = document.getElementById('btn-' + atuador);
            if (atuador !== 'alarme' && atuador !== 'alcool') {
                estado = btn.innerText === 'LIGAR' ? 1 : 0;
                btn.innerText = 'Aguarde...';
            }
            try {
                await fetch('/comando?atuador=' + atuador + '&estado=' + estado, { method: 'POST' });
            } catch (error) {}
            buscarDados();
        }
        setInterval(buscarDados, 2000);
        window.onload = buscarDados;
    </script>
</body>
</html>"""

HTML_BYTES  = _HTML_STR.encode('utf-8')
del _HTML_STR
HTML_CHUNKS = [HTML_BYTES[i:i+512] for i in range(0, len(HTML_BYTES), 512)]
LEN_HTML    = len(HTML_BYTES)

# ============================================================================
# TAREFAS ASYNC
# ============================================================================
async def task_max30102():
    global estado_sistema
    if sensor_max is None:
        return
    last_ok = time.ticks_ms()
    while True:
        try:
            if sensor_max.update():
                last_ok = time.ticks_ms()
            else:
                if time.ticks_diff(time.ticks_ms(), last_ok) > 2000:
                    print("MAX30102 watchdog — reiniciando...")
                    sensor_max.init_sensor()
                    last_ok = time.ticks_ms()
            estado_sistema["max_ir"]          = sensor_max.last_raw_ir
            estado_sistema["max_red"]         = sensor_max.last_raw_red
            estado_sistema["paciente_status"] = sensor_max.signal_quality
            estado_sistema["bpm"]             = sensor_max.heart_rate
            estado_sistema["oxigenacao"]      = sensor_max.spo2
        except Exception as e:
            print(f"task_max30102: {e}")
        await asyncio.sleep_ms(10)

async def task_sensores():
    global estado_sistema, _condicoes_silenciadas
    while True:
        # DHT11
        try:
            if sensor_dht:
                sensor_dht.measure()
                estado_sistema["temperatura"] = sensor_dht.temperature()
                estado_sistema["humidade"]    = sensor_dht.humidity()
        except Exception:
            pass

        # MQ — LEDs de qualidade do ar (verde < 1200, amarelo 1200-2500, vermelho >= 2500)
        try:
            if mq_adc:
                g = mq_adc.read()
                estado_sistema["gas_bruto"] = g
                led_verde.value(g < 1200)
                led_amarelo.value(1200 <= g < 2500)
                led_vermelho.value(g >= 2500)
        except Exception:
            pass

        # HX711 — 100 % == 550 g de líquido (frasco cheio)
        if hx:
            try:
                peso = await hx.get_units_async(times=10)
                if peso is not None:
                    liq = max(0.0, peso - 30.0)          # desconta tara do recipiente (30 g)
                    estado_sistema["soro_gramas"]      = round(liq, 1)
                    # 550 g → 100 %;  0 g → 0 %
                    estado_sistema["soro_percentagem"] = max(0.0, min(100.0, (liq / 550.0) * 100.0))
                    print(f"[HX711] {peso:.1f}g -> soro {estado_sistema['soro_percentagem']:.0f}%")
                else:
                    print("[HX711] None — sem leitura")
            except Exception as e:
                print(f"[HX711] erro: {e}")

        # Auto-oxigénio: liga quando SpO2 < 92 %; desliga com histerese em 96 %
        spo2_val = estado_sistema["oxigenacao"]
        if 0 < spo2_val < 92 and not estado_sistema["status_oxigenio"]:
            estado_sistema["status_oxigenio"] = 1
            out_oxigenio.value(1)
            print("[AUTO] Oxigenio ligado — SpO2 baixo")
        elif spo2_val >= 96 and estado_sistema["status_oxigenio"]:
            estado_sistema["status_oxigenio"] = 0
            out_oxigenio.value(0)
            print("[AUTO] Oxigenio desligado — SpO2 recuperado")

        # Alarme: activa se houver condições críticas que NÃO foram silenciadas
        condicoes_atuais = _get_condicoes(estado_sistema)
        novas = condicoes_atuais - _condicoes_silenciadas
        if novas:
            estado_sistema["alarme_ativo"] = 1
            out_buzzer.on()
        else:
            # Quando todas as condições desapareceram, limpa o registo de silêncio
            if not condicoes_atuais:
                _condicoes_silenciadas.clear()
            estado_sistema["alarme_ativo"] = 0
            out_buzzer.off()

        await asyncio.sleep(2)

async def task_entradas():
    global estado_sistema
    lt_btn = lt_ir = 0
    deb = 150   # janela anti-bounce em ms
    while True:
        now = time.ticks_ms()

        # Botão activo LOW (PULL_UP)
        if in_botao.value() == 0:
            if time.ticks_diff(now, lt_btn) > deb:
                estado_sistema["status_botao"] = "PRESSIONADO"
                lt_btn = now
        else:
            if time.ticks_diff(now, lt_btn) > deb:
                estado_sistema["status_botao"] = "SOLTO"
                lt_btn = now     # FIX: actualiza timestamp também na libertação

        # IR activo LOW (PULL_UP)
        if in_ir.value() == 0:
            if time.ticks_diff(now, lt_ir) > deb:
                estado_sistema["status_ir"] = "PROXIMIDADE DETECTADA"
                lt_ir = now
                if estado_sistema["status_alcool"] == 0:
                    asyncio.create_task(pulso_alcool())
        else:
            if time.ticks_diff(now, lt_ir) > deb:
                estado_sistema["status_ir"] = "SEM SINAL"
                lt_ir = now     # FIX: actualiza timestamp também na ausência

        await asyncio.sleep_ms(20)

async def pulso_alcool():
    global estado_sistema
    try:
        estado_sistema["status_alcool"] = 1
        out_alcool.on()
        await asyncio.sleep_ms(150)
    finally:
        out_alcool.off()
        estado_sistema["status_alcool"] = 0

# ============================================================================
# SERVIDOR HTTP
# ============================================================================
async def _drain_headers(reader):
    for _ in range(30):
        try:
            h = await asyncio.wait_for(reader.readline(), timeout=2)
        except asyncio.TimeoutError:
            break
        if h in (b'\r\n', b'\n', b''):
            break

async def _send(writer, data):
    try:
        writer.write(data)
        await writer.drain()
    except OSError:
        pass

async def handle(reader, writer):
    global estado_sistema, _condicoes_silenciadas, last_remote_sensor_time
    try:
        try:
            line = await asyncio.wait_for(reader.readline(), timeout=3)
        except asyncio.TimeoutError:
            return
        if not line:
            return
        req = line.decode()

        await _drain_headers(reader)

        if 'GET / ' in req:
            hdr = (f"HTTP/1.1 200 OK\r\nContent-Type:text/html;charset=utf-8\r\n"
                   f"Content-Length:{LEN_HTML}\r\nConnection:close\r\n\r\n").encode()
            await _send(writer, hdr)
            for chunk in HTML_CHUNKS:
                await _send(writer, chunk)

        elif '/api/data' in req:
            js = ujson.dumps(estado_sistema).encode()
            hdr = (f"HTTP/1.1 200 OK\r\nContent-Type:application/json\r\n"
                   f"Content-Length:{len(js)}\r\nConnection:close\r\n\r\n").encode()
            await _send(writer, hdr)
            await _send(writer, js)

        elif '/sensor_update' in req:
            try:
                qs = req.split('?', 1)[1].split(' ')[0] if '?' in req else ''
                p  = dict(x.split('=') for x in qs.split('&') if '=' in x)
                for k, dk in [('ir','max_ir'), ('red','max_red'),
                               ('bpm','bpm'), ('spo2','oxigenacao')]:
                    if k in p:
                        try: estado_sistema[dk] = int(p[k])
                        except: pass
                estado_sistema['paciente_status'] = p.get('status', 'AUSENTE')
                last_remote_sensor_time = time.ticks_ms()
            except Exception:
                pass
            await _send(writer,
                b"HTTP/1.1 200 OK\r\nContent-Length:0\r\nConnection:close\r\n\r\n")

        elif '/comando' in req:
            qs = req.split('?', 1)[1].split(' ')[0] if '?' in req else ''
            p  = dict(x.split('=') for x in qs.split('&') if '=' in x)
            atu = p.get('atuador', '')
            est = int(p.get('estado', 0))
            if atu == 'oxigenio':
                estado_sistema['status_oxigenio'] = est; out_oxigenio.value(est)
            elif atu == 'cooler':
                estado_sistema['status_cooler'] = est; out_cooler.value(est)
            elif atu == 'alcool':
                if not estado_sistema['status_alcool']:
                    asyncio.create_task(pulso_alcool())
            elif atu == 'lampada':
                estado_sistema['status_lampada'] = est; out_lampada.value(est)
            elif atu == 'alarme':
                _condicoes_silenciadas.update(_get_condicoes(estado_sistema))
                estado_sistema['alarme_ativo'] = 0; out_buzzer.value(0)
            await _send(writer,
                b"HTTP/1.1 200 OK\r\nContent-Length:0\r\nConnection:close\r\n\r\n")

        else:
            await _send(writer,
                b"HTTP/1.1 404 Not Found\r\nContent-Length:0\r\nConnection:close\r\n\r\n")

    except Exception as e:
        print(f"HTTP erro: {e}")
    finally:
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass
        gc.collect()

# ============================================================================
# MAIN
# ============================================================================
async def main():
    asyncio.create_task(task_max30102())
    asyncio.create_task(task_sensores())
    asyncio.create_task(task_entradas())
    srv = await asyncio.start_server(handle, "0.0.0.0", 80, backlog=2)
    print("✓ Servidor activo na porta 80 — 192.168.4.1")
    while True:
        await asyncio.sleep(5)
        gc.collect()

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Encerrado.")
