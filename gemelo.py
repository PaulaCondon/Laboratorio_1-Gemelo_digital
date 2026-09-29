import serial
import time
import threading

from collections import deque

from dash import Dash, dcc, html, Input, Output, ctx

import plotly.graph_objects as go


# ==================================================
# PUERTO SERIAL
# ==================================================

PUERTO = "COM6"
BAUDRATE = 115200


ser = serial.Serial(
    PUERTO,
    BAUDRATE,
    timeout=0.2
)

time.sleep(2)


serial_lock = threading.Lock()


# ==================================================
# VARIABLES ACTUALES DEL SISTEMA
# ==================================================

rpm_actual = 0.0
setpoint_actual = 0.0
distancia_actual = -1.0
pwm_actual = 0.0
estado_actual = 0

ultima_trama = "Esperando datos del Arduino..."


# ==================================================
# DIVERGENCIA
# ==================================================

divergencia_activa = False

inicio_divergencia = None

ultimo_setpoint_divergencia = 0.0


# El error debe mantenerse durante 2 segundos
TIEMPO_DIVERGENCIA = 2.0


# Permitimos 10 % de diferencia
TOLERANCIA_PORCENTAJE = 0.10


# Pero como mínimo permitimos 0.75 RPM
TOLERANCIA_MINIMA = 0.75


# ==================================================
# HISTORIAL PARA GRÁFICAS
# ==================================================

MAX_PUNTOS = 100

tiempos = deque(maxlen=MAX_PUNTOS)

rpm_hist = deque(maxlen=MAX_PUNTOS)

setpoint_hist = deque(maxlen=MAX_PUNTOS)


# ==================================================
# LEER DATOS DEL ARDUINO
# ==================================================

def leer_serial():

    global rpm_actual
    global setpoint_actual
    global distancia_actual
    global pwm_actual
    global estado_actual
    global ultima_trama

    global divergencia_activa
    global inicio_divergencia
    global ultimo_setpoint_divergencia


    while True:

        try:

            if ser.in_waiting > 0:

                linea = (
                    ser.readline()
                    .decode(
                        "utf-8",
                        errors="ignore"
                    )
                    .strip()
                )


                # Guardar la trama recibida
                ultima_trama = linea


                datos = linea.split(",")


                # RPM,SETPOINT,DISTANCIA,PWM,ESTADO

                if len(datos) != 5:
                    continue


                rpm_actual = float(datos[0])

                setpoint_actual = float(datos[1])

                distancia_actual = float(datos[2])

                pwm_actual = float(datos[3])

                estado_actual = int(datos[4])


                # ==================================================
                # GUARDAR DATOS HISTÓRICOS
                # ==================================================

                tiempos.append(
                    time.time()
                )

                rpm_hist.append(
                    abs(rpm_actual)
                )

                setpoint_hist.append(
                    setpoint_actual
                )


                # ==================================================
                # DETECCIÓN DE DIVERGENCIA
                # ==================================================

                ahora = time.time()

                if abs(
                    setpoint_actual
                    -
                    ultimo_setpoint_divergencia
                ) > 0.01:

                    ultimo_setpoint_divergencia = (
                        setpoint_actual
                    )

                    inicio_divergencia = None

                    divergencia_activa = False


                if (
                    setpoint_actual > 0
                    and
                    estado_actual != 0
                ):


                    error = abs(
                        abs(rpm_actual)
                        -
                        setpoint_actual
                    )


                    # Tolerancia dinámica:
                    #
                    # 10 % del setpoint

                    tolerancia = max(

                        TOLERANCIA_MINIMA,

                        setpoint_actual
                        *
                        TOLERANCIA_PORCENTAJE
                    )


                    # ==========================================
                    # FUERA DE TOLERANCIA
                    # ==========================================

                    if error > tolerancia:


                        if inicio_divergencia is None:

                            inicio_divergencia = ahora


                        elif (
                            ahora
                            -
                            inicio_divergencia
                            >=
                            TIEMPO_DIVERGENCIA
                        ):

                            divergencia_activa = True


                    # ==========================================
                    # VOLVIÓ A LA NORMALIDAD
                    # ==========================================

                    else:

                        inicio_divergencia = None

                        divergencia_activa = False


                # ==================================================
                # MOTOR DETENIDO
                # ==================================================

                else:

                    inicio_divergencia = None

                    divergencia_activa = False


        except Exception as e:

            print(
                "Error leyendo Arduino:",
                e
            )


        time.sleep(0.02)


# ==================================================
# HILO PARA LECTURA SERIAL
# ==================================================

hilo = threading.Thread(
    target=leer_serial,
    daemon=True
)

hilo.start()


# ==================================================
# ENVIAR COMANDOS AL ARDUINO
# ==================================================

def enviar(comando):

    with serial_lock:

        ser.write(
            (comando + "\n").encode()
        )


# ==================================================
# CREAR APLICACIÓN DASH
# ==================================================

app = Dash(__name__)


# ==================================================
# DISEÑO DE LA PÁGINA
# ==================================================

app.layout = html.Div(

    style={

        "fontFamily": "Arial",

        "maxWidth": "1100px",

        "margin": "auto",

        "padding": "20px"
    },

    children=[


        # ==========================================
        # TÍTULO
        # ==========================================

        html.H1(

            "Gemelo Digital - Banda Transportadora",

            style={
                "textAlign": "center"
            }
        ),


        # ==========================================
        # CONTROL
        # ==========================================

        html.H2(
            "Control"
        ),


        html.Label(
            "Velocidad deseada (%)"
        ),


        dcc.Slider(

            id="velocidad",

            min=0,

            max=100,

            value=30,

            step=1,

            marks={

                0: "0%",

                25: "25%",

                50: "50%",

                75: "75%",

                100: "100%"

            }
        ),


        html.Br(),


        # ==========================================
        # BOTONES
        # ==========================================

        html.Button(

            "Adelante",

            id="adelante",

            n_clicks=0
        ),


        html.Button(

            "Reversa",

            id="reversa",

            n_clicks=0,

            style={
                "marginLeft": "10px"
            }
        ),


        html.Button(

            "STOP",

            id="stop",

            n_clicks=0,

            style={
                "marginLeft": "10px"
            }
        ),


        html.Div(

            id="mensaje",

            style={
                "marginTop": "15px"
            }
        ),


        html.Hr(),


        # ==========================================
        # INDICADORES
        # ==========================================

        html.Div(

            style={

                "display": "flex",

                "justifyContent": "space-around",

                "textAlign": "center"
            },

            children=[


                html.Div([

                    html.H3(
                        "RPM"
                    ),

                    html.H2(
                        id="rpm-texto"
                    )

                ]),


                html.Div([

                    html.H3(
                        "Setpoint"
                    ),

                    html.H2(
                        id="setpoint-texto"
                    )

                ]),


                html.Div([

                    html.H3(
                        "PWM"
                    ),

                    html.H2(
                        id="pwm-texto"
                    )

                ]),


                html.Div([

                    html.H3(
                        "Distancia"
                    ),

                    html.H2(
                        id="distancia-texto"
                    )

                ]),

            ]
        ),


        # ==========================================
        # ESTADO
        # ==========================================

        html.H2(
            "Estado del sistema"
        ),


        html.Div(

            id="estado-texto",

            style={

                "fontSize": "25px",

                "fontWeight": "bold",

                "marginBottom": "20px"
            }
        ),


        # ==========================================
        # DIVERGENCIA
        # ==========================================

        html.H2(
            "Divergencia"
        ),


        html.Div(

            id="divergencia-texto",

            style={

                "fontSize": "20px",

                "fontWeight": "bold",

                "marginBottom": "25px"
            }
        ),


        # ==========================================
        # DATOS SERIALES
        # ==========================================

        html.H2(
            "Datos seriales"
        ),


        html.Div(

            id="datos-seriales",

            style={

                "backgroundColor": "#eeeeee",

                "padding": "15px",

                "borderRadius": "8px",

                "fontFamily": "monospace",

                "fontSize": "17px",

                "whiteSpace": "pre-line",

                "marginBottom": "30px"
            }
        ),


        # ==========================================
        # GRÁFICA
        # ==========================================

        html.H2(
            "Velocidad"
        ),


        dcc.Graph(
            id="grafica"
        ),


        # ==========================================
        # ACTUALIZACIÓN DE PÁGINA
        # ==========================================

        dcc.Interval(

            id="actualizar",

            interval=200,

            n_intervals=0
        )

    ]
)


# ==================================================
# CONTROL DESDE LA PÁGINA
# ==================================================

@app.callback(

    Output(
        "mensaje",
        "children"
    ),

    Input(
        "velocidad",
        "value"
    ),

    Input(
        "adelante",
        "n_clicks"
    ),

    Input(
        "reversa",
        "n_clicks"
    ),

    Input(
        "stop",
        "n_clicks"
    ),

    prevent_initial_call=True
)

def controlar(
    velocidad,
    adelante,
    reversa,
    stop
):

    boton = ctx.triggered_id


    # ==========================================
    # CAMBIAR VELOCIDAD
    # ==========================================

    if boton == "velocidad":

        enviar(
            f"V{velocidad}"
        )

        return (
            f"Velocidad seleccionada: "
            f"{velocidad}%"
        )


    # ==========================================
    # ADELANTE
    # ==========================================

    if boton == "adelante":

        enviar("F")

        return "Motor hacia adelante"


    # ==========================================
    # REVERSA
    # ==========================================

    if boton == "reversa":

        enviar("R")

        return "Motor en reversa"


    # ==========================================
    # STOP
    # ==========================================

    if boton == "stop":

        enviar("S")

        return "Motor detenido"


    return ""


# ==================================================
# ACTUALIZAR INFORMACIÓN DE LA PÁGINA
# ==================================================

@app.callback(

    Output(
        "rpm-texto",
        "children"
    ),

    Output(
        "setpoint-texto",
        "children"
    ),

    Output(
        "pwm-texto",
        "children"
    ),

    Output(
        "distancia-texto",
        "children"
    ),

    Output(
        "estado-texto",
        "children"
    ),

    Output(
        "divergencia-texto",
        "children"
    ),

    Output(
        "divergencia-texto",
        "style"
    ),

    Output(
        "datos-seriales",
        "children"
    ),

    Output(
        "grafica",
        "figure"
    ),

    Input(
        "actualizar",
        "n_intervals"
    )
)

def actualizar_pagina(n):


    # ==========================================
    # ESTADO DEL MOTOR
    # ==========================================

    if estado_actual == 1:

        estado = "ADELANTE"

    elif estado_actual == 2:

        estado = "REVERSA"

    else:

        estado = "PARADO"


    # ==========================================
    # DISTANCIA
    # ==========================================

    if distancia_actual < 0:

        distancia = "Sin lectura"

    else:

        distancia = (
            f"{distancia_actual:.1f} cm"
        )


    # ==========================================
    # DIVERGENCIA
    # ==========================================

    error = abs(
        abs(rpm_actual)
        -
        setpoint_actual
    )


    tolerancia = max(

        TOLERANCIA_MINIMA,

        setpoint_actual
        *
        TOLERANCIA_PORCENTAJE
    )


    # Motor detenido

    if (
        estado_actual == 0
        or
        setpoint_actual <= 0
    ):

        texto_divergencia = (
            "Sin evaluación - motor detenido"
        )

        estilo_divergencia = {

            "fontSize": "20px",

            "fontWeight": "bold",

            "marginBottom": "25px",

            "color": "gray"
        }


    # Divergencia confirmada

    elif divergencia_activa:

        texto_divergencia = (

            "⚠ DIVERGENCIA DE VELOCIDAD  |  "

            f"Error: {error:.2f} RPM"
        )

        estilo_divergencia = {

            "fontSize": "20px",

            "fontWeight": "bold",

            "marginBottom": "25px",

            "color": "red"
        }


    # Todavía está verificando

    elif inicio_divergencia is not None:

        texto_divergencia = (

            "Verificando posible divergencia...  |  "

            f"Error: {error:.2f} RPM"
        )

        estilo_divergencia = {

            "fontSize": "20px",

            "fontWeight": "bold",

            "marginBottom": "25px",

            "color": "orange"
        }


    # Funcionamiento normal

    else:

        texto_divergencia = (

            "✓ SIN DIVERGENCIA  |  "

            f"Error: {error:.2f} RPM"
        )

        estilo_divergencia = {

            "fontSize": "20px",

            "fontWeight": "bold",

            "marginBottom": "25px",

            "color": "green"
        }


    # ==========================================
    # DATOS SERIALES
    # ==========================================

    datos_seriales = (

        f"RPM real:      {rpm_actual:.2f} RPM\n"

        f"Setpoint:      {setpoint_actual:.2f} RPM\n"

        f"Distancia:     {distancia_actual:.2f} cm\n"

        f"PWM:           {pwm_actual:.0f}\n"

        f"Estado:        {estado}\n"

        f"\n"

        f"Trama recibida desde Arduino:\n"

        f"{ultima_trama}"
    )


    # ==========================================
    # CREAR GRÁFICA
    # ==========================================

    figura = go.Figure()


    if len(tiempos) > 0:

        t0 = tiempos[0]


        tiempo_relativo = [

            t - t0

            for t in tiempos
        ]


        # RPM reales

        figura.add_trace(

            go.Scatter(

                x=tiempo_relativo,

                y=list(rpm_hist),

                mode="lines",

                name="RPM reales"
            )
        )


        # Setpoint

        figura.add_trace(

            go.Scatter(

                x=tiempo_relativo,

                y=list(setpoint_hist),

                mode="lines",

                name="Setpoint"
            )
        )


    figura.update_layout(

        title="Respuesta del controlador PID",

        xaxis_title="Tiempo (s)",

        yaxis_title="Velocidad (RPM)"
    )


    # ==========================================
    # ACTUALIZAR COMPONENTES
    # ==========================================

    return (

        f"{abs(rpm_actual):.1f}",

        f"{setpoint_actual:.1f} RPM",

        f"{pwm_actual:.0f}",

        distancia,

        estado,

        texto_divergencia,

        estilo_divergencia,

        datos_seriales,

        figura
    )


# ==================================================
# EJECUTAR SERVIDOR
# ==================================================

if __name__ == "__main__":

    app.run(
        debug=False,
        port=8050
    )