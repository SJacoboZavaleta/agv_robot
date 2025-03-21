# moose_controller.py
# Controlador del robot Moose usando Pure Pursuit y PID
# Author: Sergio Jacobo Zavaleta
# Date: 25/02/2025
# Version: 3.4
# Curso: Robots de campo

from controller import Robot, Keyboard
import math, csv, random

# Constantes
TIME_STEP = 16  # Paso de tiempo en milisegundos
MAX_SPEED = 26.0 * 0.5  # Velocidad máxima en rad/s
WHEEL_RADIUS = 0.57  # Radio de las ruedas (m)
v_base = 5.0  # Velocidad base ajustada (m/s)
L = 2.4  # Distancia entre ejes (m)
L_d = 1.5  # Distancia de lookahead (m)
Kp = 80.0  # Ganancia proporcional del PID
Ki = 0.01  # Ganancia integral del PID
Kd = 5.0  # Ganancia derivativa del PID
# https://github.com/cyberbotics/webots/blob/released/projects/robots/clearpath/moose/protos/Moose.proto

# Inicialización del robot
robot = Robot()

# Dispositivos
motors = []
motor_names = ["left motor 1", "left motor 2", "left motor 3", "left motor 4", 
              "right motor 1", "right motor 2", "right motor 3", "right motor 4"]
for i in range(8):
    motor = robot.getDevice(motor_names[i])
    motor.setPosition(float('inf'))
    motors.append(motor)

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

led = robot.getDevice("robot_led")
led.set(0)
led_counter = 0

inertial_unit = robot.getDevice("inertial unit")
inertial_unit.enable(TIME_STEP)

keyboard = Keyboard()
keyboard.enable(TIME_STEP)

camera = robot.getDevice("camera")
camera.enable(TIME_STEP)

# Encender LED indicador
def update_led():
    # Resetear ambos LEDs
    # led.set(0)
    print("Actualizando LED...")
    led.set(0)
    if PID:
        led.set(0x00FF00)
    else:
        led.set(0xFFFF00)# Verde

# Función para leer la trayectoria desde un archivo CSV
def read_path_from_csv(filename):
    """Lee una ruta desde un archivo CSV y la devuelve como una lista de tuplas (x, y, z)."""
    path = []
    with open(filename, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader)  # Omitir la fila de encabezados
        for row in csvreader:
            x, y, z = map(float, row)  # Convertir las coordenadas a float
            path.append((x, y, z))
    return path

# Cargar la trayectoria desde un archivo CSV
cost_function = 3
slope_method = 3
if cost_function == 1: # Funcion de costo: pendiente
    if slope_method == 1: # Metodo pendiente zevenbergen_thorne
        path_csv = 'path_slope_purchena_z.csv'
    elif slope_method == 2: # Metodo pendiente Sobel
        path_csv = 'path_slope_purchena_s.csv'
    elif slope_method == 3: # Metodo pendiente Horn
        path_csv = 'path_slope_purchena_h.csv'
elif cost_function == 2: # Funcion de costo: energía potencial
    if slope_method == 1: # Metodo pendiente zevenbergen_thorne
        path_csv = 'path_potential_energy_purchena_z.csv'
    elif slope_method == 2: # Metodo pendiente Sobel
        path_csv = 'path_potential_energy_purchena_s.csv'
    elif slope_method == 3: # Metodo pendiente Horn
        path_csv = 'path_potential_energy_purchena_h.csv'
else: # Funcion de costo: pendiente + distancia
    if slope_method == 1: # Metodo pendiente zevenbergen_thorne
        path_csv = 'path_combined_purchena_z.csv'
    elif slope_method == 2: # Metodo pendiente Sobel
        path_csv = 'path_combined_purchena_s.csv'
    elif slope_method == 3: # Metodo pendiente Horn
        path_csv = 'path_combined_purchena_h.csv'
waypoints = read_path_from_csv(path_csv)

# Variables de estado
current_target_index = 0
integral_error = 0
previous_error = 0

# Variables auxiliares
omega = 0
gamma = 0
PID = True  # Para activar el control PID con teclado
start_time = robot.getTime()  # Tiempo inicial
total_alpha = 0  # Acumulador de errores (alpha)
iteration_count = 0  # Contador de iteraciones
aux = ""

# Función para calcular la distancia entre dos puntos
def calculate_distance(point1, point2):
    """Calcula la distancia euclidiana entre dos puntos (x, y, z)."""
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)

# Función para encontrar el punto de referencia (lookahead point) con distancia 3D
def find_lookahead_point(robot_pos, waypoints, L_d, current_idx):
    """
    Encuentra el punto de referencia más cercano a una distancia L_d en un espacio 3D.
    Si no se encuentra un punto dentro de L_d, devuelve el waypoint más cercano.
    """
    closest_point = None
    closest_distance = float('inf')
    closest_index = current_idx

    for i in range(current_idx, len(waypoints)):
        distance = calculate_distance(robot_pos, waypoints[i])
        if distance >= L_d:
            return waypoints[i], i
        if distance < closest_distance:
            closest_point = waypoints[i]
            closest_distance = distance
            closest_index = i

    # Si no se encuentra un punto dentro de L_d, devolver el más cercano
    return closest_point, closest_index

# Función para calcular el ángulo de dirección
def calculate_steering_angle(robot_pos, target_point, robot_yaw):
    """Calcula el ángulo de dirección y la curvatura necesaria para seguir el punto de referencia."""
    dx = target_point[0] - robot_pos[0]
    dy = target_point[1] - robot_pos[1]
    target_angle = math.atan2(dy, dx)
    alpha = target_angle - robot_yaw  # Error angular
    gamma = 2 * math.sin(alpha) / L_d  # Curvatura
    return gamma, alpha

# Controlador PID
def pid_control(error, dt, Kp, Ki, Kd):
    """Controlador PID para ajustar la velocidad angular."""
    global integral_error, previous_error
    proportional = Kp * error
    integral_error += error * dt
    integral = Ki * integral_error
    derivative = Kd * (error - previous_error) / dt
    previous_error = error
    return proportional + integral + derivative

# Función para establecer la velocidad de los motores
def robot_set_speed(left, right):
    """Establece la velocidad de los motores izquierdos y derechos."""
    for i in range(4):
        motors[i].setVelocity(left)
        motors[i + 4].setVelocity(right)

# Bucle principal
while robot.step(TIME_STEP) != -1:
    # Obtener posición y orientación del robot
    position_3d = gps.getValues()
    orientation = inertial_unit.getRollPitchYaw()  # Obtener roll, pitch, yaw
    robot_yaw = orientation[2]  # Usamos el ángulo de yaw para la orientación
    robot_pos = (position_3d[0], position_3d[1], position_3d[2])

    # Controlar LED indicador
    if led_counter == 0:
        random_color = random.randint(0, 3)
        led.set(random_color)
        led_counter = random.randint(4, 7)
    else:
        led_counter -= 1

    # Obtener comandos del teclado
    keys = keyboard.getKey()
    if keys == Keyboard.ALT + ord('P'):
        print('Pure Pursuit + PID activado !!!')
        PID = True
        update_led()
    if keys == Keyboard.ALT + ord('L'):
        print('Solo Pure Pursuit activado !!!')
        PID = False
        update_led()

    # Verificar si se llegó al objetivo final
    if calculate_distance(robot_pos, waypoints[-1]) < 0.1:
        robot_set_speed(0, 0)
        print("¡Trayectoria completada!")
        
        # Calcular tiempo total y error promedio
        end_time = robot.getTime()
        total_time = end_time - start_time
        average_alpha = total_alpha / iteration_count if iteration_count > 0 else 0

        # Guardar resultados en un archivo
        with open(f'simulation_results_{cost_function}_{slope_method}.txt', 'w') as f:
            f.write(f"Tiempo total de simulación: {total_time:.2f} segundos\n")
            f.write(f"Error promedio (alpha): {average_alpha:.4f} radianes\n")

        print(f"Tiempo total: {total_time:.2f} segundos")
        print(f"Error promedio (alpha): {average_alpha:.4f} radianes")
        break

    # Encontrar el punto de referencia
    lookahead_point, current_target_index = find_lookahead_point(robot_pos, waypoints, L_d, current_target_index)

    # Calcular el ángulo de dirección y la curvatura
    gamma, alpha = calculate_steering_angle(robot_pos, lookahead_point, robot_yaw)

    # Acumular el error (alpha) y contar iteraciones
    total_alpha += abs(alpha)
    iteration_count += 1

    # Calcular la pendiente usando el ángulo de pitch del IMU
    pitch = orientation[1]  # Ángulo de pitch en radianes
    slope = math.tan(pitch)  # Pendiente como tangente del ángulo de pitch
    # tan 30 = 0.577

    # Ajustar ganancias del PID en función de la pendiente
    if abs(slope) > 0.1:
        Kp_adjusted = Kp * 1.5  # Aumentar Kp en pendientes
    else:
        Kp_adjusted = Kp

    # Controlador PID para ajustar la velocidad angular
    if PID:
        omega = pid_control(alpha, TIME_STEP / 1000.0, Kp_adjusted, Ki, Kd)
    else:
        omega = gamma * v_base

    # Calcular velocidades de las ruedas (m/s)
    v_left = v_base - omega * (L / 2)
    v_right = v_base + omega * (L / 2)

    # Ajustar velocidad en pendientes
    if slope > 0.1:  # Subiendo
        v_left *= 1.5  # Aumentar velocidad al subir
        v_right *= 1.5
        aux = "Subiendo..."
    elif slope < -0.05:  # Bajando
        # Considerar una pendiente más pronunciada para reducir la velocidad en mapas con pendientes pronunciadas (<-0.1)
        v_left *= 0.8  # Reducir velocidad al bajar
        v_right *= 0.8
        aux = "Bajando..."
    else:
        aux = "En plano..."

    # Limitar las velocidades al máximo permitido (Convirtiendo a rad/s)
    v_left = max(min(v_left / WHEEL_RADIUS, MAX_SPEED), -MAX_SPEED)
    v_right = max(min(v_right / WHEEL_RADIUS, MAX_SPEED), -MAX_SPEED)

    # Establecer velocidades de los motores
    robot_set_speed(v_left, v_right)

    # Salidas en terminal
    if iteration_count % 50 == 0:
        print(f"Modo: {'PID' if PID else 'Pure Pursuit'}, Pos: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}, {position_3d[2]:.2f}), " +
              f"Look: ({lookahead_point[0]:.2f}, {lookahead_point[1]:.2f}, {lookahead_point[2]:.2f}), " +
              f"Err: {alpha:.2f}, V_L: {v_left:.2f}, V_R: {v_right:.2f}, " +
              f"Slope: {slope:.2f}, Kp: {Kp_adjusted:.2f}, estado: {aux}")