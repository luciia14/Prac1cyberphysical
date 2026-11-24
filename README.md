# Práctica 1: Follow Line en Unibotics

Asignatura: Cyber-physical and Robotic Intelligent Systems (SJK001)

Seguidor de línea roja para el coche de Fórmula 1 del ejercicio **Follow Line** (circuito *Simple Circuit*) de Unibotics. Percepción con máscara HSV y cinco franjas horizontales, control lateral con feedforward y velocidad adaptativa. Se entregan cuatro variantes que solo se diferencian en el controlador: **P, PI, PD y PID**.

## Contenido

| Fichero | Controlador | Ganancias |
|---|---|---|
| `follow_line_P.py` | Proporcional | `KP = 2.613` |
| `follow_line_PI.py` | Proporcional e integral | `KP = 2.613`, `KI = 0.537` |
| `follow_line_PD.py` | Proporcional y derivativo | `KP = 2.613`, `KD = 0.2573` |
| `follow_line_PID.py` | Proporcional, integral y derivativo | `KP = 2.613`, `KI = 0.318`, `KD = 0.2573` |
| `memoria.pdf` | Memoria de la práctica | |

## Requisitos

- Equipo con **Linux** y **Docker** instalado (con permisos para ejecutar contenedores).
- Navegador web con acceso a [Unibotics](https://unibotics.org).

## Puesta en marcha

### 1. Lanzar el backend en local

En una terminal de Linux, ejecutar:

```bash
docker run --rm -it \
  -p 6080-6090:6080-6090 -p 7163:7163 \
  jderobot/robotics-backend:latest
```

Se lanza en local porque el equipo no tiene tarjeta gráfica con la que acelerar la simulación: Gazebo y los nodos del ejercicio corren en la propia máquina y Unibotics solo se conecta a ellos. La terminal debe quedarse abierta durante toda la sesión.

### 2. Conectar Unibotics con el backend

1. Abrir en el navegador el ejercicio **Follow Line** (*Simple Circuit*) en Unibotics.
2. Pulsar el botón **Click to connect**.
3. Esperar a que cargue el simulador con el coche en la línea de salida.

### 3. Subir el código y ejecutarlo

1. Copiar el contenido de la variante que se quiere probar en el editor de Unibotics (o subir el fichero a la plataforma).
2. Pulsar **Run**.
3. Ver la imagen de depuración en la interfaz y el coche en el simulador.
4. Esperar a que el coche termine las dos vueltas: se detiene solo y el resumen aparece en la terminal.

## Ajustes

Al principio de cada fichero hay un bloque de configuración. Los más útiles:

| Parámetro | Valor | Efecto |
|---|---|---|
| `FEEDFORWARD` | `True` | Anticipa la curva con las franjas lejanas |
| `ADAPTIVE_V` | `True` | Frena en curvas y acelera en rectas |
| `V_MAX` / `V_MIN` | `28.043` / `14.027` | Velocidad en recta y en la curva más cerrada |
| `LAPS_TARGET` | `2` | Vueltas que se cronometran antes de parar |

Para repetir la comparativa, basta con cambiar `FEEDFORWARD` y `ADAPTIVE_V`. Todas las pruebas de la memoria se hicieron con ambos activados.

## Salida por terminal

Durante la ejecución se imprime la configuración (por ejemplo `PID+FF+Vadapt`) y un aviso `[LAP]` por cada vuelta completada. Al terminar se imprime el resumen final: tiempo de cada vuelta, tiempo total, MAE, RMSE, error máximo, `|W|` medio, porcentaje de línea perdida y dos medidas de oscilación. Los resultados salen solo por terminal.

## Resultados

Dos vueltas en *Simple Circuit*, todas con feedforward y velocidad adaptativa:

| Configuración | Tiempo total (s) | MAE | W medio | Línea perdida (%) |
|---|---|---|---|---|
| PID + FF | 70.10 | 0.465 | 1.240 | 1.5 |
| PD + FF | 71.93 | 0.489 | 1.310 | 1.8 |
| PI + FF | 74.45 | 0.524 | 1.420 | 2.3 |
| P + FF | 76.80 | 0.562 | 1.535 | 2.9 |

El PID mejora un 8.7 % el tiempo total respecto al P. El análisis completo está en la memoria.

