#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define SDA_PIN 20
#define SCL_PIN 19

#define XSHUT_1 14
#define XSHUT_2 22
#define XSHUT_3 23

#define UART_RX 6
#define UART_TX 7

const char* WIFI_SSID = "Arana-Robot";
const char* WIFI_PASSWORD = "Percepcion2026";

IPAddress localIP(10, 42, 0, 50);
IPAddress gateway(10, 42, 0, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

unsigned long ultimoIntentoWiFi = 0;
const unsigned long INTERVALO_WIFI = 10000;

bool servidorIniciado = false;


const uint16_t DISTANCIA_FRENTE = 300;
const uint16_t DISTANCIA_LATERAL = 500;
const uint16_t SIN_OBJETO = 8190;

const uint8_t LECTURAS_CONFIRMACION = 3;

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

struct Lectura {
  uint16_t distancia;
  bool timeout;
};

Lectura lecturaIzquierda = {8190, false};
Lectura lecturaFrontal = {8190, false};
Lectura lecturaDerecha = {8190, false};

unsigned long ultimaLecturaSensores = 0;
const unsigned long INTERVALO_SENSORES = 100;


const int SERVO_MIN = 150;
const int SERVO_MAX = 600;

Adafruit_PWMServoDriver pwm(0x40);


// LOCOMOCION CI

const float FRECUENCIA_PWM = 50.0;

const float A = 86.0;
const float B = 136.0;
const float L = 49.26;

const float OFFSET_FEMUR = -25.0;
const float OFFSET_TIBIA = 40.0;

const float COXA_MIN = 0.0;
const float COXA_MAX = 155.0;
const float FEMUR_MIN = 20.0;
const float FEMUR_MAX = 180.0;
const float TIBIA_MIN = 0.0;
const float TIBIA_MAX = 180.0;

const float X_BASE = 0.0;
const float Y_BASE = 140.0;
const float Z_BASE = -78.0;

const float LONGITUD_ZANCADA = 80.0;
const float ALTURA_LEVANTAMIENTO = 35.0;
const unsigned long CICLO_MS = 2000;
const unsigned long INTERVALO_MS = 20;
const float FRACCION_BALANCEO = 0.25;

const byte CICLOS_GIRO_LATERAL = 5;
const byte CICLOS_GIRO_180 = 10;

const byte CANALES[4][3] = {
  {3, 4, 5},       // FL
  {0, 1, 2},       // FR
  {6, 7, 8},       // RL
  {9, 10, 11}      // RR
};

const float DESFASE[4] = {
  0.00,   // FL
  0.50,   // FR
  0.75,   // RL
  0.25    // RR
};

const float SIGNO_AVANCE[4] = {
  -1.0,   // FL
   1.0,   // FR
  -1.0,   // RL
   1.0    // RR
};

const float NEUTRAL[4][3] = {
  {90, 105, 165},
  {90,  92, 145},
  {90, 102, 145},
  {90,  90, 145}
};

float correccion[4][3];
bool calibracionCorrecta = false;

float actualCanal[12] = {
  90, 90, 90, 90, 90, 90,
  90, 90, 90, 90, 90, 90
};

enum ModoLocomocion {
  LOC_DETENIDA,
  LOC_AVANCE,
  LOC_GIRO
};

ModoLocomocion modoLocomocion = LOC_DETENIDA;

float signoRotacion = 1.0;

unsigned long inicioMovimiento = 0;
unsigned long ultimoUpdateLocomocion = 0;
byte ciclosRestantes = 0;


enum Estado {
  AVANZANDO,
  ESPERANDO_VISION,
  ROTANDO_INSPECCION_IZQ,
  ROTANDO_INSPECCION_DER,
  ROTANDO_RUTA_IZQ,
  ROTANDO_RUTA_DER,
  ROTANDO_180,
  ESPERANDO_FRENTE_LIBRE,
  ERROR_VISION,
  TERMINADO
};

enum TipoBusqueda {
  BUSQUEDA_FRONTAL,
  INSPECCION_IZQUIERDA,
  INSPECCION_DERECHA
};

Estado estado = AVANZANDO;

TipoBusqueda tipoBusqueda = BUSQUEDA_FRONTAL;

uint8_t contadorFrente = 0;
uint8_t contadorLibre = 0;

bool izquierdaInvestigada = false;
bool derechaInvestigada = false;
bool preferirIzquierda = true;


String mensajeUART = "";
String comandoUSB = "";

String ultimoUARTEnviado = "Sin mensajes";
String ultimoUARTRecibido = "Sin mensajes";

String historialUART = "";

const unsigned int MAX_HISTORIAL = 3000;


unsigned long ultimaTelemetria = 0;

const unsigned long INTERVALO_WEB = 500;


// navegacion y locomocion
void detenerMovimiento();
void iniciarMarchaAdelante();
void iniciarGiro(byte nuevoSentido, byte ciclos);
void solicitarBusqueda(TipoBusqueda tipo);
void decidirDireccion();
void actualizarLocomocion();
void completarCicloRotacion();
void procesarMensajeUART(String respuesta);
void leerUART();
void reiniciarNavegacion();

void configurarServidor();
void atenderWiFi();
void actualizarServidor();


void agregarHistorial(String mensaje) {

  String hora = String(millis() / 1000) + "s ";

  historialUART += hora + mensaje + "\n";

  if (historialUART.length() > MAX_HISTORIAL) {

    int corte = historialUART.indexOf('\n');

    if (corte >= 0) {
      historialUART.remove(0, corte + 1);
    }
  }
}


String escaparJSON(String texto) {

  String resultado = "";

  for (unsigned int i = 0; i < texto.length(); i++) {

    char c = texto[i];

    if (c == '"') {
      resultado += "\\\"";
    }
    else if (c == '\\') {
      resultado += "\\\\";
    }
    else if (c == '\n') {
      resultado += "\\n";
    }
    else if (c == '\r') {
      resultado += "\\r";
    }
    else if (c == '\t') {
      resultado += "\\t";
    }
    else if ((uint8_t)c >= 32) {
      resultado += c;
    }
  }

  return resultado;
}


const char* obtenerNombreEstado() {

  switch (estado) {

    case AVANZANDO:
      return "AVANZANDO";

    case ESPERANDO_VISION:
      return "ESPERANDO VISION";

    case ROTANDO_INSPECCION_IZQ:
      return "INSPECCION IZQUIERDA";

    case ROTANDO_INSPECCION_DER:
      return "INSPECCION DERECHA";

    case ROTANDO_RUTA_IZQ:
      return "RUTA IZQUIERDA";

    case ROTANDO_RUTA_DER:
      return "RUTA DERECHA";

    case ROTANDO_180:
      return "ROTACION 180";

    case ESPERANDO_FRENTE_LIBRE:
      return "COMPROBANDO FRENTE";

    case ERROR_VISION:
      return "ERROR VISION";

    case TERMINADO:
      return "MISION COMPLETADA";

    default:
      return "DESCONOCIDO";
  }
}

String distanciaJSON(Lectura r) {

  if (r.timeout) {
    return "\"TIMEOUT\"";
  }

  if (r.distancia == SIN_OBJETO) {
    return "\"SIN OBJETO\"";
  }

  return String(r.distancia);
}


void enviarTelemetria() {

  String json = "{";

  json += "\"estado\":\"";
  json += obtenerNombreEstado();
  json += "\",";

  json += "\"wifi\":";
  json += WiFi.status() == WL_CONNECTED ? "true" : "false";
  json += ",";

  json += "\"ip\":\"";
  json += WiFi.localIP().toString();
  json += "\",";

  json += "\"izquierda\":";
  json += distanciaJSON(lecturaIzquierda);
  json += ",";

  json += "\"frontal\":";
  json += distanciaJSON(lecturaFrontal);
  json += ",";

  json += "\"derecha\":";
  json += distanciaJSON(lecturaDerecha);
  json += ",";

  json += "\"uart_tx\":\"";
  json += escaparJSON(ultimoUARTEnviado);
  json += "\",";

  json += "\"uart_rx\":\"";
  json += escaparJSON(ultimoUARTRecibido);
  json += "\",";

  json += "\"historial\":\"";
  json += escaparJSON(historialUART);
  json += "\",";

  json += "\"tiempo\":";
  json += String(millis() / 1000);

  json += "}";

  server.send(200, "application/json", json);
}


const char PAGINA_WEB[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">

<head>

<meta charset="UTF-8">

<meta name="viewport"
content="width=device-width, initial-scale=1">

<title>Monitor Araña 12 GDL</title>

<style>

* {
  box-sizing: border-box;
}

body {
  margin: 0;
  padding: 18px;
  background: #10151d;
  color: white;
  font-family: Arial, sans-serif;
}

.container {
  max-width: 900px;
  margin: auto;
}

h1 {
  text-align: center;
  color: #48d597;
}

.card {
  background: #1c2532;
  padding: 18px;
  margin: 12px 0;
  border-radius: 12px;
}

.grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 10px;
}

.sensor {
  background: #293747;
  padding: 15px 5px;
  border-radius: 10px;
  text-align: center;
}

.valor {
  font-size: 25px;
  color: #48d597;
  font-weight: bold;
}

.estado {
  font-size: 19px;
  color: #48d597;
  font-weight: bold;
  overflow-wrap: anywhere;
}

button {
  width: 100%;
  padding: 16px;
  margin-top: 10px;
  border: none;
  border-radius: 9px;
  font-size: 16px;
  font-weight: bold;
  cursor: pointer;
}

button:disabled {
  opacity: 0.5;
  cursor: wait;
}

.iniciar {
  background: #168c60;
  color: white;
}

.reiniciar {
  background: #b94747;
  color: white;
}

pre {
  background: #10151d;
  padding: 12px;
  border-radius: 8px;
  height: 190px;
  overflow-y: auto;
  white-space: pre-wrap;
  overflow-wrap: anywhere;
  font-size: 12px;
  color: #a9e8c9;
}

.etiqueta {
  color: #a9b7c7;
  font-size: 13px;
}

@media(max-width:500px) {

  .grid {
    grid-template-columns: 1fr;
  }

}

</style>

</head>

<body>

<div class="container">

<h1>🕷️ ARAÑA ROBOTICA 12 GDL</h1>

<div class="card">

<h3>Conexion Wi-Fi</h3>

<p>Estado:
<span id="wifi">--</span>
</p>

<p>Direccion IP:
<span id="ip">--</span>
</p>

<p>Tiempo activo:
<span id="tiempo">--</span>
</p>

</div>

<div class="card">

<h3>Sensores de distancia</h3>

<div class="grid">

<div class="sensor">

<p>IZQUIERDA</p>

<div class="valor" id="izquierda">--</div>

<small>mm</small>

</div>

<div class="sensor">

<p>FRENTE</p>

<div class="valor" id="frontal">--</div>

<small>mm</small>

</div>

<div class="sensor">

<p>DERECHA</p>

<div class="valor" id="derecha">--</div>

<small>mm</small>

</div>

</div>

</div>

<div class="card">

<h3>Estado de navegacion</h3>

<div class="estado" id="estado">--</div>

</div>

<div class="card">

<h3>Comunicacion UART</h3>

<p class="etiqueta">
Ultimo mensaje ESP32 → Raspberry
</p>

<div id="tx">--</div>

<p class="etiqueta">
Ultimo mensaje Raspberry → ESP32
</p>

<div id="rx">--</div>

<h4>Historial de mensajes</h4>

<pre id="historial">Esperando mensajes...</pre>

</div>

<div class="card">

<h3>Control manual</h3>

<button class="iniciar"
id="btnIniciar"
onclick="control('/iniciar')">

▶ INICIAR BUSQUEDA

</button>

<button class="reiniciar"
id="btnReiniciar"
onclick="control('/reiniciar')">

↻ REINICIAR NAVEGACION

</button>

<p id="respuesta"></p>

</div>

</div>

<script>

async function actualizar() {

  try {

    const r = await fetch('/api');

    if (!r.ok) throw new Error('Error HTTP');

    const d = await r.json();

    document.getElementById('wifi').textContent =
      d.wifi ? 'CONECTADO' : 'DESCONECTADO';

    document.getElementById('ip').textContent =
      d.ip;

    document.getElementById('tiempo').textContent =
      d.tiempo + ' segundos';

    document.getElementById('izquierda').textContent =
      d.izquierda;

    document.getElementById('frontal').textContent =
      d.frontal;

    document.getElementById('derecha').textContent =
      d.derecha;

    document.getElementById('estado').textContent =
      d.estado;

    document.getElementById('tx').textContent =
      d.uart_tx;

    document.getElementById('rx').textContent =
      d.uart_rx;

    const h = document.getElementById('historial');

    const estabaAbajo =
      h.scrollTop + h.clientHeight >= h.scrollHeight - 10;

    h.textContent = d.historial;

    if (estabaAbajo) {
      h.scrollTop = h.scrollHeight;
    }

  } catch(e) {

    document.getElementById('wifi').textContent =
      'SIN COMUNICACION';

  }

}

async function control(ruta) {

  const respuesta = document.getElementById('respuesta');

  const b1 = document.getElementById('btnIniciar');
  const b2 = document.getElementById('btnReiniciar');

  b1.disabled = true;
  b2.disabled = true;

  respuesta.textContent = 'Enviando solicitud...';

  try {

    const r = await fetch(ruta, {
      method: 'POST'
    });

    const d = await r.json();

    respuesta.textContent = d.mensaje;

    if (!r.ok) {
      respuesta.textContent = 'Aviso: ' + d.mensaje;
    }

    actualizar();

  } catch(e) {

    respuesta.textContent = 'Error de comunicacion';

  }

  b1.disabled = false;
  b2.disabled = false;

}

setInterval(actualizar, 500);

actualizar();

</script>

</body>
</html>
)rawliteral";


void iniciarWiFi() {

  WiFi.mode(WIFI_STA);

  WiFi.setSleep(false);

  if (!WiFi.config(localIP, gateway, subnet)) {
    Serial.println("Error configurando IP fija.");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Conectando al Wi-Fi del Raspberry...");

  ultimoIntentoWiFi = millis();
}

void atenderWiFi() {

  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  if (millis() - ultimoIntentoWiFi >= INTERVALO_WIFI) {

    ultimoIntentoWiFi = millis();

    Serial.println("Reintentando conexion Wi-Fi...");

    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}


void reiniciarNavegacion() {

  Serial.println();
  Serial.println("================================");
  Serial.println("REINICIANDO NAVEGACION");
  Serial.println("================================");


  detenerMovimiento();

  contadorFrente = 0;
  contadorLibre = 0;

  izquierdaInvestigada = false;
  derechaInvestigada = false;

  preferirIzquierda = true;

  tipoBusqueda = BUSQUEDA_FRONTAL;

  mensajeUART = "";


  while (Serial1.available()) {
    Serial1.read();
  }


  estado = AVANZANDO;

  iniciarMarchaAdelante();

  Serial.println("Navegacion restablecida.");
  Serial.println("Iniciando marcha autonoma.");
}


void configurarServidor() {

  server.on("/", HTTP_GET, []() {

    server.send_P(
      200,
      "text/html",
      PAGINA_WEB
    );

  });

  server.on("/api", HTTP_GET, []() {

    enviarTelemetria();

  });


  server.on("/iniciar", HTTP_POST, []() {

    if (estado != TERMINADO && estado != ERROR_VISION) {

      server.send(
        409,
        "application/json",
        "{\"mensaje\":\"La navegacion ya esta activa.\"}"
      );

      return;
    }

    contadorFrente = 0;
    contadorLibre = 0;

    izquierdaInvestigada = false;
    derechaInvestigada = false;

    preferirIzquierda = true;

    tipoBusqueda = BUSQUEDA_FRONTAL;

    estado = AVANZANDO;

    iniciarMarchaAdelante();

    Serial.println("Busqueda iniciada desde pagina web.");

    server.send(
      200,
      "application/json",
      "{\"mensaje\":\"Busqueda iniciada correctamente.\"}"
    );

  });


  server.on("/reiniciar", HTTP_POST, []() {

    reiniciarNavegacion();

    server.send(
      200,
      "application/json",
      "{\"mensaje\":\"Navegacion reiniciada desde el principio.\"}"
    );

  });

  server.onNotFound([]() {

    server.send(
      404,
      "text/plain",
      "Pagina no encontrada"
    );

  });

  server.begin();

  servidorIniciado = true;

  Serial.println("Servidor web iniciado.");
  Serial.print("Pagina: http://");
  Serial.println(localIP);

}


void actualizarServidor() {

  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  server.handleClient();
}


Lectura leerSensor(VL53L0X &sensor) {

  Lectura r;

  r.distancia = sensor.readRangeContinuousMillimeters();

  r.timeout = sensor.timeoutOccurred();

  return r;
}

bool estaLibre(Lectura r, uint16_t umbral) {

  if (r.timeout) return false;

  if (r.distancia == SIN_OBJETO) return true;

  return r.distancia >= umbral;
}

bool hayObstaculo(Lectura r, uint16_t umbral) {

  if (r.timeout) return true;

  if (r.distancia == SIN_OBJETO) return false;

  return r.distancia < umbral;
}

bool objetoCercanoValido(Lectura r) {

  if (r.timeout || r.distancia == SIN_OBJETO) {
    return false;
  }

  return r.distancia <= DISTANCIA_LATERAL;
}

void imprimirLectura(const char *nombre, Lectura r) {

  Serial.print(nombre);
  Serial.print(": ");

  if (r.timeout) {

    Serial.println("TIMEOUT");

  } else if (r.distancia == SIN_OBJETO) {

    Serial.println("SIN OBJETO EN RANGO");

  } else {

    Serial.print(r.distancia);
    Serial.println(" mm");

  }
}


int anguloAPulso(float angulo) {

  angulo = constrain(angulo, 0.0f, 180.0f);

  return SERVO_MIN +
    (int)((SERVO_MAX - SERVO_MIN) * angulo / 180.0f);
}

void escribirCanal(byte canal, float angulo) {

  angulo = constrain(angulo, 0.0f, 180.0f);

  pwm.setPWM(canal, 0, anguloAPulso(angulo));

  actualCanal[canal] = angulo;
}

// =====================================================
// CINEMATICA INVERSA
// =====================================================

float limitarCoseno(float valor) {

  if (valor > 1.0f) return 1.0f;
  if (valor < -1.0f) return -1.0f;

  return valor;
}

bool calcularCinematica(
  float x,
  float y,
  float z,
  float angulos[3]
) {

  float d = sqrt(x * x + y * y);
  float r = d - L;
  float c = sqrt(r * r + z * z);

  if (
    c < fabs(A - B) ||
    c > (A + B) ||
    c < 0.001f
  ) {
    return false;
  }

  float theta1 = atan2(y, x) * 180.0f / PI;

  float arg2 =
    (A * A + c * c - B * B) /
    (2.0f * A * c);

  arg2 = limitarCoseno(arg2);

  float theta2 =
    atan2(r, -z) * 180.0f / PI +
    acos(arg2) * 180.0f / PI;

  float arg3 =
    (A * A + B * B - c * c) /
    (2.0f * A * B);

  arg3 = limitarCoseno(arg3);

  float theta3 =
    180.0f - acos(arg3) * 180.0f / PI;

  angulos[0] = theta1;
  angulos[1] = theta2 + OFFSET_FEMUR;
  angulos[2] = theta3 + OFFSET_TIBIA;

  return true;
}


void calibrarPatas() {

  float referencia[3];

  if (!calcularCinematica(
        X_BASE,
        Y_BASE,
        Z_BASE,
        referencia
      )) {

    calibracionCorrecta = false;
    return;
  }

  for (byte pata = 0; pata < 4; pata++) {

    for (byte articulacion = 0; articulacion < 3; articulacion++) {

      correccion[pata][articulacion] =
        NEUTRAL[pata][articulacion] -
        referencia[articulacion];
    }
  }

  calibracionCorrecta = true;
}


bool calcularAngulosPata(
  byte pata,
  float x,
  float y,
  float z,
  float angulos[3]
) {

  float calculados[3];

  if (!calcularCinematica(x, y, z, calculados)) {
    return false;
  }

  for (byte articulacion = 0; articulacion < 3; articulacion++) {

    angulos[articulacion] =
      calculados[articulacion] +
      correccion[pata][articulacion];
  }

  if (
    angulos[0] < COXA_MIN || angulos[0] > COXA_MAX ||
    angulos[1] < FEMUR_MIN || angulos[1] > FEMUR_MAX ||
    angulos[2] < TIBIA_MIN || angulos[2] > TIBIA_MAX
  ) {
    return false;
  }

  return true;
}


void calcularTrayectoria(
  byte pata,
  float fase,
  float signoX,
  float &x,
  float &y,
  float &z
) {

  float medioPaso = LONGITUD_ZANCADA / 2.0f;
  float xLocal;

  if (fase < FRACCION_BALANCEO) {

    // Recuperacion: el pie se levanta y avanza.
    float t = fase / FRACCION_BALANCEO;

    xLocal =
      -medioPaso +
      LONGITUD_ZANCADA * t;

    z =
      Z_BASE +
      ALTURA_LEVANTAMIENTO * sin(PI * t);

  } else {

    // Apoyo: el pie retrocede respecto al cuerpo.
    float t =
      (fase - FRACCION_BALANCEO) /
      (1.0f - FRACCION_BALANCEO);

    xLocal =
      medioPaso -
      LONGITUD_ZANCADA * t;

    z = Z_BASE;
  }

  x = X_BASE + signoX * xLocal;
  y = Y_BASE;
}


void actualizarPata(
  byte pata,
  float fase,
  float signoX
) {

  float x, y, z;
  float angulos[3];

  calcularTrayectoria(
    pata,
    fase,
    signoX,
    x,
    y,
    z
  );

  if (!calcularAngulosPata(
        pata,
        x,
        y,
        z,
        angulos
      )) {

    return;
  }

  for (byte articulacion = 0; articulacion < 3; articulacion++) {

    escribirCanal(
      CANALES[pata][articulacion],
      angulos[articulacion]
    );
  }
}


void detenerMovimiento() {

  modoLocomocion = LOC_DETENIDA;
  ciclosRestantes = 0;
}

void iniciarMarchaAdelante() {

  modoLocomocion = LOC_AVANCE;
  inicioMovimiento = millis();
  ultimoUpdateLocomocion = inicioMovimiento - INTERVALO_MS;
  ciclosRestantes = 0;
}

void iniciarGiro(byte nuevoSentido, byte ciclos) {

  signoRotacion = (nuevoSentido == 0) ? 1.0f : -1.0f;

  ciclosRestantes = (ciclos == 0) ? 1 : ciclos;

  modoLocomocion = LOC_GIRO;
  inicioMovimiento = millis();
  ultimoUpdateLocomocion = inicioMovimiento - INTERVALO_MS;
}

void actualizarLocomocion() {

  if (modoLocomocion == LOC_DETENIDA) {
    return;
  }

  unsigned long ahora = millis();

  if (ahora - ultimoUpdateLocomocion < INTERVALO_MS) {
    return;
  }

  ultimoUpdateLocomocion = ahora;

  unsigned long transcurrido = ahora - inicioMovimiento;

  bool terminarGiro = false;

  if (modoLocomocion == LOC_GIRO) {

    unsigned long duracionGiro =
      (unsigned long)ciclosRestantes * CICLO_MS;

    if (transcurrido >= duracionGiro) {
      transcurrido = duracionGiro;
      terminarGiro = true;
    }
  }

  float ciclo =
    (float)transcurrido / (float)CICLO_MS;

  ciclo -= floor(ciclo);

  for (byte pata = 0; pata < 4; pata++) {

    float fase = ciclo + DESFASE[pata];
    fase -= floor(fase);

    float signoX;

    if (modoLocomocion == LOC_AVANCE) {
      signoX = SIGNO_AVANCE[pata];
    } else {
      signoX = signoRotacion;
    }

    actualizarPata(
      pata,
      fase,
      signoX
    );
  }

  if (terminarGiro) {

    detenerMovimiento();

    completarCicloRotacion();
  }
}


void completarCicloRotacion() {

  switch (estado) {

    case ROTANDO_INSPECCION_IZQ:

      solicitarBusqueda(INSPECCION_IZQUIERDA);
      break;

    case ROTANDO_INSPECCION_DER:

      solicitarBusqueda(INSPECCION_DERECHA);
      break;

    case ROTANDO_RUTA_IZQ:
    case ROTANDO_RUTA_DER:
    case ROTANDO_180:

      contadorLibre = 0;
      estado = ESPERANDO_FRENTE_LIBRE;

      Serial.println(
        "Giro terminado; comprobando el frente."
      );
      break;

    default:
      break;
  }
}


void solicitarBusqueda(TipoBusqueda tipo) {

  tipoBusqueda = tipo;

  detenerMovimiento();

  estado = ESPERANDO_VISION;

  mensajeUART = "";

  ultimoUARTEnviado = "BUSCAR_MANZANA";

  agregarHistorial(
    "[ESP32 -> RASPBERRY] BUSCAR_MANZANA"
  );

  Serial.println();

  Serial.println(
    "Solicitando busqueda a Raspberry Pi..."
  );

  Serial1.println("BUSCAR_MANZANA");
}


void procesarMensajeUART(String respuesta) {

  respuesta.trim();

  respuesta.toUpperCase();

  if (respuesta.length() == 0) return;

  ultimoUARTRecibido = respuesta;

  agregarHistorial(
    "[RASPBERRY -> ESP32] " + respuesta
  );

  Serial.print("Raspberry -> ESP32: ");

  Serial.println(respuesta);

  if (estado != ESPERANDO_VISION) {

    Serial.println(
      "Respuesta ignorada: no hay busqueda pendiente."
    );

    return;
  }

  if (respuesta == "ENCONTRADA") {

    detenerMovimiento();

    estado = TERMINADO;

    Serial.println(
      "MANZANA ENCONTRADA. MISION COMPLETADA."
    );

  }

  else if (respuesta == "NO_ENCONTRADA") {

    decidirDireccion();

  }

  else if (respuesta == "ERROR_VISION") {

    detenerMovimiento();

    estado = ERROR_VISION;

    Serial.println(
      "Error de vision. Escribe r para reintentar."
    );

  }
}


void leerUART() {

  while (Serial1.available()) {

    char c = (char)Serial1.read();

    if (c == '\n') {

      procesarMensajeUART(mensajeUART);

      mensajeUART = "";

    }

    else if (c != '\r') {

      if (mensajeUART.length() < 100) {

        mensajeUART += c;

      } else {

        mensajeUART = "";

      }
    }
  }
}


void leerMonitorSerial() {

  while (Serial.available()) {

    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {

      comandoUSB.trim();

      comandoUSB.toLowerCase();

      if (
        comandoUSB == "r" &&
        estado == ERROR_VISION
      ) {

        solicitarBusqueda(tipoBusqueda);

      }

      comandoUSB = "";

    }

    else if (comandoUSB.length() < 30) {

      comandoUSB += c;

    }

    else {

      comandoUSB = "";

    }
  }
}


void decidirDireccion() {

  Lectura izq = leerSensor(sensor1);

  Lectura der = leerSensor(sensor3);

  imprimirLectura("IZQUIERDA", izq);

  imprimirLectura("DERECHA", der);

  bool objetoIzq = objetoCercanoValido(izq);

  bool objetoDer = objetoCercanoValido(der);

  bool libreIzq = estaLibre(izq, DISTANCIA_LATERAL);

  bool libreDer = estaLibre(der, DISTANCIA_LATERAL);

  bool revisarIzq = objetoIzq && !izquierdaInvestigada;

  bool revisarDer = objetoDer && !derechaInvestigada;

  if (revisarIzq && revisarDer) {

    if (izq.distancia < der.distancia) {

      revisarDer = false;

    } else if (der.distancia < izq.distancia) {

      revisarIzq = false;

    } else {

      revisarIzq = preferirIzquierda;

      revisarDer = !preferirIzquierda;

    }
  }

  detenerMovimiento();

  if (revisarIzq) {

    izquierdaInvestigada = true;

    estado = ROTANDO_INSPECCION_IZQ;

    Serial.println(
      "Girando a la izquierda para inspeccionar."
    );

    iniciarGiro(0, CICLOS_GIRO_LATERAL);

    return;
  }

  if (revisarDer) {

    derechaInvestigada = true;

    estado = ROTANDO_INSPECCION_DER;

    Serial.println(
      "Girando a la derecha para inspeccionar."
    );

    iniciarGiro(1, CICLOS_GIRO_LATERAL);

    return;
  }

  if (libreIzq && libreDer) {

    if (preferirIzquierda) {

      estado = ROTANDO_RUTA_IZQ;

      iniciarGiro(0, CICLOS_GIRO_LATERAL);

    } else {

      estado = ROTANDO_RUTA_DER;

      iniciarGiro(1, CICLOS_GIRO_LATERAL);

    }

    preferirIzquierda = !preferirIzquierda;

    return;
  }

  if (libreIzq) {

    estado = ROTANDO_RUTA_IZQ;

    iniciarGiro(0, CICLOS_GIRO_LATERAL);

    return;
  }

  if (libreDer) {

    estado = ROTANDO_RUTA_DER;

    iniciarGiro(1, CICLOS_GIRO_LATERAL);

    return;
  }

  Serial.println(
    "Sin ruta lateral libre. Rotando 180 grados."
  );

  estado = ROTANDO_180;

  iniciarGiro(0, CICLOS_GIRO_180);
}


void setup() {

  Serial.begin(115200);

  Serial1.begin(
    115200,
    SERIAL_8N1,
    UART_RX,
    UART_TX
  );


  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(300);


  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);

  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);

  delay(300);


  digitalWrite(XSHUT_1, HIGH);
  delay(100);
  if (!sensor1.init()) {

    Serial.println("ERROR: sensor izquierdo.");
    detenerMovimiento();
    while (true) delay(1000);

  }

  sensor1.setAddress(0x30);
  sensor1.setTimeout(100);
  sensor1.startContinuous(50);


  digitalWrite(XSHUT_2, HIGH);
  delay(100);
  if (!sensor2.init()) {

    Serial.println("ERROR: sensor frontal.");
    detenerMovimiento();
    while (true) delay(1000);

  }

  sensor2.setAddress(0x31);
  sensor2.setTimeout(100);
  sensor2.startContinuous(50);


  digitalWrite(XSHUT_3, HIGH);
  delay(100);
  if (!sensor3.init()) {

    Serial.println("ERROR: sensor derecho.");
    detenerMovimiento();
    while (true) delay(1000);

  }

  sensor3.setAddress(0x32);
  sensor3.setTimeout(100);
  sensor3.startContinuous(50);


  calibrarPatas();

  if (!calibracionCorrecta) {
    Serial.println("ERROR: referencia XYZ fuera del alcance de la cinematica.");
    while (true) {
      delay(1000);
    }
  }

  for (byte p = 0; p < 4; p++) {

    for (byte j = 0; j < 3; j++) {

      escribirCanal(
        CANALES[p][j],
        NEUTRAL[p][j]
      );
    }
  }

  delay(1000);
  Serial.println("Araña 12 GDL lista.");
  Serial.println("I2C: SDA 20 / SCL 19");
  Serial.println("UART: RX 6 / TX 7");
  Serial.println("Sensores VL53L0X inicializados.");


  iniciarWiFi();


  configurarServidor();


  Serial.println("Iniciando marcha autonoma.");

  estado = AVANZANDO;

  iniciarMarchaAdelante();
}


void loop() {

  atenderWiFi();
  actualizarServidor();
  leerUART();
  leerMonitorSerial();
  actualizarLocomocion();

  unsigned long ahora = millis();

  if (
    ahora - ultimaLecturaSensores <
    INTERVALO_SENSORES
  ) {

    return;
  }

  ultimaLecturaSensores = ahora;

  lecturaIzquierda = leerSensor(sensor1);

  lecturaFrontal = leerSensor(sensor2);

  lecturaDerecha = leerSensor(sensor3);

  Lectura izq = lecturaIzquierda;

  Lectura frente = lecturaFrontal;

  Lectura der = lecturaDerecha;


  if (estado == ESPERANDO_VISION) {
    return;
  }


  if (
    estado == TERMINADO ||
    estado == ERROR_VISION
  ) {

    return;
  }


  if (estado == AVANZANDO) {

    if (
      hayObstaculo(
        frente,
        DISTANCIA_FRENTE
      )
    ) {

      contadorFrente++;

    } else {

      contadorFrente = 0;

    }

    if (
      contadorFrente >= LECTURAS_CONFIRMACION
    ) {

      Serial.println(
        "OBSTACULO FRONTAL DETECTADO."
      );

      detenerMovimiento();

      izquierdaInvestigada = false;

      derechaInvestigada = false;

      contadorFrente = 0;

      solicitarBusqueda(BUSQUEDA_FRONTAL);
    }
  }


  else if (estado == ESPERANDO_FRENTE_LIBRE) {

    if (
      estaLibre(
        frente,
        DISTANCIA_FRENTE
      )
    ) {

      contadorLibre++;

    } else {

      contadorLibre = 0;

    }

    if (
      contadorLibre >= LECTURAS_CONFIRMACION
    ) {

      Serial.println(
        "FRENTE LIBRE. Reanudando marcha."
      );

      contadorLibre = 0;

      contadorFrente = 0;

      izquierdaInvestigada = false;

      derechaInvestigada = false;

      estado = AVANZANDO;

      iniciarMarchaAdelante();
    }
  }
}