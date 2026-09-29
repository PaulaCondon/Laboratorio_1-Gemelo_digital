#include "PID.h"

// ======================================================
// PINES
// ======================================================

#define ENC_A 2

#define TRIG 9
#define ECHO 10

#define IN1 4
#define IN2 5
#define ENA 6


// ======================================================
// ENCODER
// ======================================================

volatile long encoderCount = 0;

const float PULSOS_POR_VUELTA = 3242.0;


// ======================================================
// VELOCIDAD
// ======================================================

// 100 % = 30 RPM
const double RPM_MAX_REAL = 30.0;

int porcentajeVelocidad = 0;


// ÚNICO SETPOINT
double setpoint = 0.0;


// Entrada y salida PID
double inputPID = 0.0;
double outputPID = 0.0;


// ======================================================
// FILTRO RPM
// ======================================================

double rpmFiltrada = 0.0;

const double ALPHA_RPM = 0.30;


// ======================================================
// CORRECCIÓN DEL ENCODER
// ======================================================

const int PWM_UMBRAL_CORRECCION = 155;

const double PENDIENTE_RPM_PWM = 0.192;
const double OFFSET_RPM_PWM = -11.94;


// ======================================================
// PID
// ======================================================

double Kp = 8.0;
double Ki = 2.2;
double Kd = 0.10;


PID miPID(
  &inputPID,
  &outputPID,
  &setpoint,
  Kp,
  Ki,
  Kd,
  PID::Direction::DIRECT
);


// ======================================================
// PWM
// ======================================================

// Permitimos llegar aproximadamente a 30 RPM
const int PWM_MAX_PID = 220;


// Impulso inicial
const int PWM_ARRANQUE = 210;

const unsigned long TIEMPO_ARRANQUE_MS = 500;

int pwmAplicado = 0;


// ======================================================
// ESTADO MOTOR
// ======================================================

enum EstadoMotor {

  PARADO = 0,

  ADELANTE = 1,

  REVERSA = 2
};


EstadoMotor estadoMotor = PARADO;


// ======================================================
// ARRANQUE
// ======================================================

bool enImpulsoArranque = false;

unsigned long inicioImpulso = 0;


// ======================================================
// MEDICIÓN
// ======================================================

unsigned long ultimoTiempo = 0;

const unsigned long PERIODO_MUESTREO = 100;


// ======================================================
// ULTRASONIDO
// ======================================================

float distancia = -1;


// ======================================================
// INTERRUPCIÓN ENCODER
// ======================================================

void encoderISR() {

  encoderCount++;
}


// ======================================================
// DISTANCIA
// ======================================================

float leerDistancia() {

  digitalWrite(TRIG, LOW);

  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);

  delayMicroseconds(10);

  digitalWrite(TRIG, LOW);


  unsigned long duracion =
    pulseIn(
      ECHO,
      HIGH,
      25000
    );


  if (duracion == 0) {

    return -1;
  }


  return
    duracion
    *
    0.0343
    /
    2.0;
}


// ======================================================
// ARRANQUE
// ======================================================

void iniciarImpulso() {

  if (estadoMotor == PARADO) {

    return;
  }


  if (porcentajeVelocidad <= 0) {

    return;
  }


  miPID.SetMode(
    PID::Mode::MANUAL
  );


  miPID.reset();


  outputPID = 0;


  enImpulsoArranque = true;

  inicioImpulso = millis();


  pwmAplicado =
    PWM_ARRANQUE;


  analogWrite(
    ENA,
    pwmAplicado
  );
}


// ======================================================
// ADELANTE
// ======================================================

void adelante() {

  estadoMotor =
    ADELANTE;


  digitalWrite(
    IN1,
    HIGH
  );


  digitalWrite(
    IN2,
    LOW
  );


  if (porcentajeVelocidad > 0) {

    iniciarImpulso();

  } else {

    analogWrite(
      ENA,
      0
    );
  }
}


// ======================================================
// REVERSA
// ======================================================

void reversa() {

  estadoMotor =
    REVERSA;


  digitalWrite(
    IN1,
    LOW
  );


  digitalWrite(
    IN2,
    HIGH
  );


  if (porcentajeVelocidad > 0) {

    iniciarImpulso();

  } else {

    analogWrite(
      ENA,
      0
    );
  }
}


// ======================================================
// STOP
// ======================================================

void detener() {

  estadoMotor =
    PARADO;


  digitalWrite(
    IN1,
    LOW
  );


  digitalWrite(
    IN2,
    LOW
  );


  analogWrite(
    ENA,
    0
  );


  porcentajeVelocidad = 0;

  setpoint = 0;

  inputPID = 0;

  outputPID = 0;

  rpmFiltrada = 0;

  pwmAplicado = 0;


  enImpulsoArranque =
    false;


  miPID.SetMode(
    PID::Mode::MANUAL
  );


  miPID.reset();
}


// ======================================================
// CAMBIAR VELOCIDAD
// ======================================================

void cambiarVelocidad(
  int porcentaje
) {

  porcentaje =
    constrain(
      porcentaje,
      0,
      100
    );


  int porcentajeAnterior =
    porcentajeVelocidad;


  porcentajeVelocidad =
    porcentaje;

  setpoint =
    (
      porcentajeVelocidad
      /
      100.0
    )
    *
    RPM_MAX_REAL;


  // ==================================================
  // 0 %
  // ==================================================

  if (porcentajeVelocidad == 0) {

    pwmAplicado = 0;

    outputPID = 0;


    analogWrite(
      ENA,
      0
    );


    enImpulsoArranque =
      false;


    miPID.SetMode(
      PID::Mode::MANUAL
    );


    miPID.reset();


    return;
  }


  // ==================================================
  // ARRANQUE DESDE CERO
  // ==================================================

  if (
    porcentajeAnterior == 0
    &&
    estadoMotor != PARADO
  ) {

    iniciarImpulso();

    return;
  }

}


// ======================================================
// SERIAL
// ======================================================

void leerComandosSerial() {

  if (Serial.available() <= 0) {

    return;
  }


  char comando =
    Serial.read();


  switch (comando) {


    case 'F':

      adelante();

      break;


    case 'R':

      reversa();

      break;


    case 'S':

      detener();

      break;


    case 'V': {

      int porcentaje =
        Serial.parseInt();


      cambiarVelocidad(
        porcentaje
      );


      break;
    }
  }
}


// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(
    115200
  );


  Serial.setTimeout(
    50
  );


  // Encoder
  pinMode(
    ENC_A,
    INPUT_PULLUP
  );


  // Ultrasonido
  pinMode(
    TRIG,
    OUTPUT
  );


  pinMode(
    ECHO,
    INPUT
  );


  // Motor
  pinMode(
    IN1,
    OUTPUT
  );


  pinMode(
    IN2,
    OUTPUT
  );


  pinMode(
    ENA,
    OUTPUT
  );


  // Encoder
  attachInterrupt(
    digitalPinToInterrupt(ENC_A),
    encoderISR,
    RISING
  );


  // ==================================================
  // CONFIGURAR PID
  // ==================================================

  miPID.SetOutputLimits(
    0,
    PWM_MAX_PID
  );


  miPID.SetMode(
    PID::Mode::MANUAL
  );


  detener();


  ultimoTiempo =
    millis();
}


// ======================================================
// LOOP
// ======================================================

void loop() {

  leerComandosSerial();


  unsigned long ahora =
    millis();


  // ==================================================
  // IMPULSO DE ARRANQUE
  // ==================================================

  if (enImpulsoArranque) {


    pwmAplicado =
      PWM_ARRANQUE;


    analogWrite(
      ENA,
      pwmAplicado
    );


    if (
      ahora
      -
      inicioImpulso
      >=
      TIEMPO_ARRANQUE_MS
    ) {


      enImpulsoArranque =
        false;


      miPID.SetMode(
        PID::Mode::AUTOMATIC
      );
    }
  }


  // ==================================================
  // MEDICIÓN
  // ==================================================

  if (
    ahora
    -
    ultimoTiempo
    >=
    PERIODO_MUESTREO
  ) {


    // ==================================================
    // TIEMPO
    // ==================================================

    float dt =
      (
        ahora
        -
        ultimoTiempo
      )
      /
      1000.0;


    ultimoTiempo =
      ahora;


    // ==================================================
    // PULSOS
    // ==================================================

    noInterrupts();


    long pulsos =
      encoderCount;


    encoderCount =
      0;


    interrupts();


    // ==================================================
    // RPM DEL ENCODER
    // ==================================================

    double rpmEncoder =
      (
        pulsos
        /
        PULSOS_POR_VUELTA
      )
      *
      (
        60.0
        /
        dt
      );


    // ==================================================
    // CORRECCIÓN DE RPM
    // ==================================================

    double rpmInstantanea =
      rpmEncoder;


    if (
      pwmAplicado
      >
      PWM_UMBRAL_CORRECCION
    ) {


      double rpmEstimadaPWM =
        (
          PENDIENTE_RPM_PWM
          *
          pwmAplicado
        )
        +
        OFFSET_RPM_PWM;

      if (
        rpmEncoder
        >
        3.0
      ) {


        if (
          rpmEstimadaPWM
          >
          rpmEncoder
        ) {

          rpmInstantanea =
            rpmEstimadaPWM;

        } else {

          rpmInstantanea =
            rpmEncoder;
        }
      }
    }


    // ==================================================
    // FILTRAR RPM
    // ==================================================

    if (
      rpmFiltrada
      ==
      0
    ) {

      rpmFiltrada =
        rpmInstantanea;

    } else {

      rpmFiltrada =
        (
          ALPHA_RPM
          *
          rpmInstantanea
        )
        +
        (
          (1.0 - ALPHA_RPM)
          *
          rpmFiltrada
        );
    }


    // ==================================================
    // ENTRADA DEL PID
    // ==================================================

    inputPID =
      rpmFiltrada;


    // ==================================================
    // CONTROL PID
    // ==================================================

    if (
      estadoMotor != PARADO
      &&
      porcentajeVelocidad > 0
      &&
      !enImpulsoArranque
    ) {


      miPID.Compute();


      pwmAplicado =
        constrain(
          (int)outputPID,
          0,
          PWM_MAX_PID
        );


      analogWrite(
        ENA,
        pwmAplicado
      );
    }


    // ==================================================
    // MOTOR PARADO
    // ==================================================

    if (
      estadoMotor == PARADO
      ||
      porcentajeVelocidad == 0
    ) {

      pwmAplicado = 0;


      analogWrite(
        ENA,
        0
      );
    }


    // ==================================================
    // DISTANCIA
    // ==================================================

    distancia =
      leerDistancia();


    // ==================================================
    // SALIDA SERIAL
    //
    // RPM,SETPOINT,DISTANCIA,PWM,ESTADO
    // ==================================================

    Serial.print(
      rpmFiltrada,
      2
    );


    Serial.print(",");


    Serial.print(
      setpoint,
      2
    );


    Serial.print(",");


    Serial.print(
      distancia,
      2
    );


    Serial.print(",");


    Serial.print(
      pwmAplicado
    );


    Serial.print(",");


    Serial.println(
      estadoMotor
    );
  }
}