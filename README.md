# 🤖 Optimización de Producción con Brazos Robóticos

¡Bienvenido a este emocionante proyecto de optimización de procesos industriales utilizando brazos robóticos y algoritmos genéticos! Este repositorio contiene un potente código en Python que busca optimizar la eficiencia de los brazos robóticos, reduciendo el tiempo de ciclo y mejorando la productividad en entornos de manufactura.

---

## 📋 Descripción del Proyecto

Este proyecto utiliza **algoritmos genéticos** para optimizar las trayectorias y configuraciones de un brazo robótico con múltiples grados de libertad (DOFs). Al tener en cuenta diversos factores como la configuración de cinemática inversa (IKM), la ubicación, la orientación y la longitud de los enlaces del brazo, nuestro algoritmo busca minimizar el tiempo de ciclo y maximizar la eficiencia operativa.

### 🛠️ Características Principales

- **Optimización de Rutas**: Determina la ruta más eficiente para el brazo robótico en un entorno de trabajo.
- **Configuración Adaptativa**: Ajusta dinámicamente la configuración de cinemática inversa para cada tarea, adaptándose a las necesidades del entorno.
- **Simulación Visual 3D**: Proporciona una visualización detallada del movimiento del brazo robótico en un espacio 3D, mejorando la comprensión del movimiento y las trayectorias.
- **Análisis de Desempeño**: Grafica la evolución de la eficiencia del algoritmo a lo largo de múltiples generaciones, mostrando la mejora continua en el tiempo de ciclo.

### 🌟 Tecnologías Utilizadas

- **Python**: Lenguaje principal utilizado para el desarrollo del algoritmo.
- **Matplotlib**: Utilizada para visualizaciones y gráficos que muestran el rendimiento del algoritmo y las trayectorias del brazo robótico.
- **NumPy**: Para cálculos numéricos avanzados, proporcionando eficiencia y velocidad en las operaciones matemáticas.

---

## 🚀 Instrucciones para Empezar

### Requisitos

Antes de comenzar, asegúrate de tener instaladas las siguientes bibliotecas de Python:

```bash
pip install numpy matplotlib deap
git clone https://github.com/tu_usuario/optimiza-robotica.git
cd optimiza-robotica
python robot_optimization.py
```
### ⚙️ Parámetros del Código
- **NUM_POINTS:** Define el número de puntos de tarea que el brazo debe completar. Un mayor número de puntos aumenta la complejidad del problema.
- **DOFs:** Grados de libertad del brazo robótico, determinando la flexibilidad y el rango de movimiento del mismo.
- **IKM_CONFIGS:** Configuraciones posibles para la cinemática inversa del brazo, afectando las posibles posturas y trayectorias.
- **PLACEMENT_NODES:** Posibles ubicaciones del manipulador en el entorno de trabajo, representando la diversidad en posiciones iniciales.
- **ORIENTATION_NODES:** Posibles orientaciones del manipulador, incrementando la capacidad de ajuste fino del brazo.
- **LINK_LENGTHS:** Longitudes de los enlaces del brazo robótico, cruciales para calcular las trayectorias utilizando cinemática directa.

### 🧩 Estructura del Algoritmo
- **evaluate(individual):** Función de evaluación que calcula el tiempo de ciclo basado en la configuración actual del cromosoma. Evalúa la eficiencia de cada solución propuesta.
- **init_individual(icls):** Inicializa un nuevo individuo para la población de soluciones, generando una combinación única de tareas, configuraciones, ubicaciones y orientaciones.
- **main():** Controla el flujo principal del algoritmo genético, incluyendo la evolución de la población y la evaluación de fitness.
- **forward_kinematics(joint_angles, link_lengths):** Calcula la posición final del end-effector del brazo robótico utilizando cinemática directa.
- **plot_robot_arm(joint_angles, link_lengths):** Función para graficar el brazo robótico en un espacio 3D, mostrando su configuración actual.
- **animate_robot_arm(joint_angles_list, link_lengths):** Función para animar el movimiento del brazo robótico, proporcionando una visualización dinámica de su trayectoria.
- **simulate_robot_arm_movement(best_individual, link_lengths):** Simula el movimiento del brazo robótico basado en la mejor trayectoria encontrada por el algoritmo genético.

### 📊 Visualización de Resultados
- **Gráficas de Evolución:** El script genera gráficos que muestran la evolución del mejor tiempo de ciclo, el tiempo promedio de ciclo, y la diversidad de la población a lo largo de las generaciones.
- **Distribución de Tiempos de Ciclo:** Visualiza la distribución de los tiempos de ciclo en la última generación, proporcionando una idea de la convergencia del algoritmo.
- **Simulación 3D:** Utiliza animaciones para mostrar el movimiento óptimo del brazo robótico, facilitando la comprensión de los resultados.

### 📈 Ejemplos de Gráficas
- Evolución del Mejor Tiempo de Ciclo
- Diversidad de la Población a lo Largo de las Generaciones
- Simulación del Movimiento del Brazo Robótico

### 📜 Licencia
Este proyecto está bajo la licencia MIT. Siéntete libre de usar, modificar y distribuir el código según tus necesidades.

### 👥 Contribuciones
¡Las contribuciones son bienvenidas! Si deseas mejorar el algoritmo o agregar nuevas características, no dudes en crear un pull request o abrir un issue.

### 📞 Contacto
Para cualquier pregunta o comentario, por favor contacta a DCR7JUILLAND@duck.com.

python
Copiar código

Este código en Markdown cubre todos los aspectos de un proyecto típico, desde la descripción del proyecto hasta las instrucciones para comenzar, características principales, tecnologías utilizadas, estructura del algoritmo, visualización de resultados, ejemplos de gráficos, licencia, contribuciones y contacto.







