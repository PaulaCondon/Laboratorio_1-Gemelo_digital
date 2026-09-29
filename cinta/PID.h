#pragma once

#include <Arduino.h>


class PID {

public:

  enum class Mode {

    MANUAL,

    AUTOMATIC
  };


  enum class Direction {

    DIRECT,

    REVERSE
  };


private:

  double* input;

  double* output;

  double* setpoint;


  double kp;

  double ki;

  double kd;


  double integral;

  double errorAnterior;


  double outputMin;

  double outputMax;


  Mode mode;

  Direction direction;


  bool primerCalculo;


  unsigned long lastTimeCalculated;


public:


  PID(
    double* input,
    double* output,
    double* setpoint,
    double kp,
    double ki,
    double kd,
    Direction direction = Direction::DIRECT
  )
    :

      input(input),

      output(output),

      setpoint(setpoint),

      kp(kp),

      ki(ki),

      kd(kd),

      integral(0.0),

      errorAnterior(0.0),

      outputMin(0.0),

      outputMax(255.0),

      mode(Mode::MANUAL),

      direction(direction),

      primerCalculo(true),

      lastTimeCalculated(0)

  {

  }


  // ==================================================
  // COMPUTE
  // ==================================================

  void Compute() {


    if (
      !input
      ||
      !output
      ||
      !setpoint
    ) {

      return;
    }


    if (
      mode
      ==
      Mode::MANUAL
    ) {

      return;
    }


    unsigned long ahora =
      millis();


    // Primera ejecución

    if (
      lastTimeCalculated
      ==
      0
    ) {

      lastTimeCalculated =
        ahora;

      return;
    }


    double dt =
      (
        ahora
        -
        lastTimeCalculated
      )
      /
      1000.0;


    lastTimeCalculated =
      ahora;


    if (
      dt <= 0
    ) {

      return;
    }


    *output =
      calcular(
        *setpoint,
        *input,
        dt
      );
  }


  // ==================================================
  // PID
  // ==================================================

  double calcular(
    double referencia,
    double valorActual,
    double dt
  ) {


    // ==========================================
    // ERROR
    // ==========================================

    double error =
      referencia
      -
      valorActual;


    if (
      direction
      ==
      Direction::REVERSE
    ) {

      error =
        -error;
    }


    // ==========================================
    // PROPORCIONAL
    // ==========================================

    double P =
      kp
      *
      error;


    // ==========================================
    // DERIVATIVO
    // ==========================================

    double derivada =
      0;


    if (
      !primerCalculo
    ) {

      derivada =
        (
          error
          -
          errorAnterior
        )
        /
        dt;
    }


    double D =
      kd
      *
      derivada;


    // ==========================================
    // INTEGRAL
    // ==========================================

    double integralNueva =
      integral
      +
      error
      *
      dt;


    double I =
      ki
      *
      integralNueva;


    // ==========================================
    // SALIDA PROVISIONAL
    // ==========================================

    double salidaProvisional =
      P
      +
      I
      +
      D;


    // ==========================================
    // ANTI WINDUP
    // ==========================================

    if (

      (
        salidaProvisional
        <=
        outputMax

        &&

        salidaProvisional
        >=
        outputMin
      )

      ||

      (
        salidaProvisional
        >
        outputMax

        &&

        error
        <
        0
      )

      ||

      (
        salidaProvisional
        <
        outputMin

        &&

        error
        >
        0
      )

    ) {

      integral =
        integralNueva;
    }


    // Recalcular integral aceptada

    I =
      ki
      *
      integral;


    // ==========================================
    // SALIDA PID
    // ==========================================

    double salida =
      P
      +
      I
      +
      D;


    // Limitar salida PWM

    if (
      salida
      >
      outputMax
    ) {

      salida =
        outputMax;
    }


    if (
      salida
      <
      outputMin
    ) {

      salida =
        outputMin;
    }


    errorAnterior =
      error;


    primerCalculo =
      false;


    return salida;
  }


  // ==================================================
  // RESET
  // ==================================================

  void reset() {

    integral =
      0;

    errorAnterior =
      0;

    primerCalculo =
      true;

    lastTimeCalculated =
      0;
  }


  // ==================================================
  // MODO
  // ==================================================

  void SetMode(
    Mode nuevoModo
  ) {


    if (
      nuevoModo
      ==
      mode
    ) {

      return;
    }


    if (
      nuevoModo
      ==
      Mode::AUTOMATIC
    ) {

      reset();
    }


    mode =
      nuevoModo;
  }


  // ==================================================
  // LÍMITES
  // ==================================================

  void SetOutputLimits(
    double minimo,
    double maximo
  ) {


    if (
      minimo
      >=
      maximo
    ) {

      return;
    }


    outputMin =
      minimo;

    outputMax =
      maximo;
  }


  // ==================================================
  // GANANCIAS
  // ==================================================

  void setKp(
    double valor
  ) {

    kp =
      valor;
  }


  void setKi(
    double valor
  ) {

    ki =
      valor;
  }


  void setKd(
    double valor
  ) {

    kd =
      valor;
  }
};