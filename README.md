# Práctica 2: Filtro de Kalman en ROS 2
Este repositorio contiene el código base para la **Práctica 2** de la asignatura de *Ampliación de Robótica*, cuyo objetivo es implementar un **Filtro de Kalman (KF)** en un entorno simulado con **ROS 2**.

El ejercicio se divide en dos partes: una primera aproximación basada en odometría con estimación de posición, y una segunda con estimación de posición y velocidad utilizando un modelo de estado extendido.

---

## Estructura del repositorio
 - kalman_filter.py # Implementación del KF con TODOs para completar 
 - kf_estimation.py # Nodo con el modelo básico de KF (posición)
 - kf_estimation_vel.py # Nodo con el modelo completo de KF (posición y velocidad) 
 - motion_models.py # Modelos de movimiento A y B 
 - observation_models.py # Modelos de observación C
 - sensor_utils.py # Funciones de alto nivel para facilitar las cosas con los sensores
 - visualization.py # Funciones de visualización de resultados
 

## Instrucciones

### Requisitos previos
Descargar el simulador y los paquetes dependientes del mismo para poder trabajar con el robot Turtlebot 4:

```bash
sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes ros-dev-tools

```

### 1. Clonar el repositorio

```bash
git clone https://github.com/miggilcas/p2_kf_adr
cd p2_kf_adr
```
### 2. Construir el paquete
```bash
colcon build --packages-select p2_kf_adr
source install/setup.zsh  # o setup.bash si no estás usando el docker
```
### 3. Lanzar el simulador
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```
### 4. Ejecutar el nodo del filtro de Kalman
#### Modelo 1: estimación de posición
```bash
ros2 run p2_kf_adr kf_estimation
```
#### Modelo 2:
```bash
ros2 run p2_kf_adr kf_estimation_vel
```

## Objetivo de la práctica

- Comprender y programar un filtro de Kalman básico para estimar la posición del robot.
- Ampliar el modelo de estado para incluir velocidad y emplear un modelo lineal puro.
- Comparar el comportamiento del filtro con diferentes configuraciones de ruido.
- Preparar el terreno para el uso de un Filtro de Kalman Extendido (EKF) en la siguiente práctica.

## Qué deben completar los estudiantes
Los archivos kalman_filter.py, motion_models.py y observation_models.py contienen TODOs que los alumnos deben implementar.

Las clases principales son:

- KalmanFilter – Para el modelo simple (posición).
- KalmanFilter_2 – Para el modelo completo (posición + velocidad).

## Configuración de ruido

El sistema permite configurar diferentes niveles de ruido para los experimentos. Para cambiar las configuraciones de ruido, edite los parámetros en los archivos `kf_estimation.py` y `kf_estimation_vel.py`.

### Ejemplo de configuraciones:
1. **Ruido bajo**:
   ```python
   proc_noise_std = [0.01, 0.01, 0.005]
   obs_noise_std = [0.01, 0.01, 0.005]
   ```

2. **Ruido alto en la medida**:
   ```python
   proc_noise_std = [0.02, 0.02, 0.01]
   obs_noise_std = [0.1, 0.1, 0.05]
   ```

3. **Ruido alto en el proceso**:
   ```python
   proc_noise_std = [0.1, 0.1, 0.05]
   obs_noise_std = [0.02, 0.02, 0.01]
   ```

Para ejecutar los experimentos, simplemente modifique las configuraciones y ejecute los nodos correspondientes.

## Implementación de cada parte

### 1. Filtro de Kalman
El filtro de Kalman se implementa en el archivo `kalman_filter.py`. Este archivo contiene dos clases principales:
- **KalmanFilter**: Diseñado para estimar la posición del robot utilizando un modelo de estado básico.
- **KalmanFilter_2**: Extiende el modelo de estado para incluir velocidad, permitiendo una estimación más completa.

Ambas clases implementan los métodos `predict` y `update` para realizar las etapas de predicción y corrección del filtro.

### 2. Modelos de Movimiento y Observación
- **`motion_models.py`**: Define las matrices de transición de estado (`A`) y de entrada de control (`B`) para los modelos de movimiento.
- **`observation_models.py`**: Proporciona las matrices de observación (`C`) para los modelos de sensor.

### 3. Nodos de ROS 2
- **`kf_estimation.py`**: Nodo que utiliza el filtro de Kalman básico para estimar la posición.
- **`kf_estimation_vel.py`**: Nodo que utiliza el filtro de Kalman extendido para estimar posición y velocidad.

Ambos nodos suscriben datos de odometría y publican las estimaciones en tópicos de ROS 2.

### 4. Visualización
El archivo `visualization.py` permite visualizar las trayectorias estimadas y reales en RViz y Matplotlib. Esto facilita la comparación entre los datos reales y las estimaciones del filtro.

### 5. Configuración de Ruido
Se pueden configurar diferentes niveles de ruido en los archivos `kf_estimation.py` y `kf_estimation_vel.py`. Esto permite realizar experimentos con distintas condiciones de ruido.

---

## Resultados Estimados

### Caso 1: Ruido Bajo
- **Configuración**:
  ```python
  proc_noise_std = [0.01, 0.01, 0.005]
  obs_noise_std = [0.01, 0.01, 0.005]
  ```
- **Resultados**: El filtro de Kalman produce estimaciones muy cercanas a la trayectoria real. La incertidumbre es mínima y las trayectorias estimadas y reales casi se superponen.

### Caso 2: Ruido Alto en la Medida
- **Configuración**:
  ```python
  proc_noise_std = [0.02, 0.02, 0.01]
  obs_noise_std = [0.1, 0.1, 0.05]
  ```
- **Resultados**: Las estimaciones muestran mayor variabilidad debido al ruido en las mediciones. Sin embargo, el filtro logra suavizar las trayectorias.

### Caso 3: Ruido Alto en el Proceso
- **Configuración**:
  ```python
  proc_noise_std = [0.1, 0.1, 0.05]
  obs_noise_std = [0.02, 0.02, 0.01]
  ```
- **Resultados**: Las estimaciones son menos precisas y presentan un mayor desfase respecto a la trayectoria real. Esto se debe a la incertidumbre en el modelo de movimiento.

---

## Análisis
1. **Ruido Bajo**: En este caso, el filtro de Kalman funciona de manera óptima, ya que tanto el modelo de movimiento como las mediciones son confiables.
2. **Ruido Alto en la Medida**: El filtro depende más del modelo de movimiento para corregir las estimaciones, lo que puede introducir errores si el modelo no es perfecto.
3. **Ruido Alto en el Proceso**: La incertidumbre en el modelo de movimiento afecta significativamente las estimaciones, ya que el filtro no puede confiar completamente en las predicciones.

En general, el filtro de Kalman es robusto frente a niveles moderados de ruido, pero su desempeño depende de un equilibrio adecuado entre las incertidumbres del modelo y las mediciones.

## Entrega
Los estudiantes deberán subir a GitHub o entregar un archivo .zip con nombre: p2_kf_<iniciales> (por ejemplo: p2_kf_mgc).

El repositorio o archivo.zip debe contener:

1. Código completo con los TODOs resueltos.

2. Capturas o gráficas de los resultados de estimación para ambos modelos.

3. Experimentos con tres configuraciones distintas:
    - Ruido bajo.
    - Ruido alto en la medida.
    - Ruido alto en el proceso.

4. Un README o una pequeña memoria en PDF explicando:
    - Cómo se ha implementado cada parte.
    - Resultados observados en los tres casos.
    - Breve análisis de por qué ocurre lo observado.

## Comentarios adicionales
Podéis cambiarle el nombre al paquete y ponerle el mismo que a la entrega, pero sed consistentes a la hora de configurar el paquete y que esté ese nombre en todos lados para que compile bien (tanto en el nombre de la carpeta donde estarán los scripts como en el setup.cfg, como en el setup.py y como en el package.xml).
