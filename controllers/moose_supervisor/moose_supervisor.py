# supervisor.py
# Supervisador para el controlador del robot Moose usando Pure Pursuit y PID
# Permite la visualizacion de la trayectoria del robot mediante la creacion de esferas en el mapa
# Author: Sergio Jacobo Zavaleta
# Date: 25/02/2025
# Version: 3.0
# Curso: Robots de campo

from controller import Supervisor
import struct
import numpy as np
from scipy.interpolate import CubicSpline
import csv

class TrajectoryVisualizer(Supervisor):
    def __init__(self):
        super().__init__()
        
        # Waypoints originales obtenidos mediante el algoritmo A*
        # (x, y) en metros 
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

        self.waypoints = self.read_path_from_csv(path_csv)
        
        # Calcular los puntos interpolados
        #self.trajectory_points = self.interpolate_waypoints(self.waypoints)
        self.trajectory_points = self.waypoints

        # Visualizar los puntos
        self.visualize_points()
    
    def read_path_from_csv(self, filename):
        """Función para leer una ruta desde un archivo CSV"""
        path = []
        with open(filename, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            next(csvreader)  # Omitimos los encabezados
            for row in csvreader:
                x, y, z = map(float, row)  # Convertimos las coordenadas a float
                path.append((x, y, z))
        return path

    def interpolate_waypoints(self, waypoints, num_points=10):
        waypoints = np.array(waypoints)
        x = waypoints[:, 0]
        y = waypoints[:, 1]
        t = np.linspace(0, 1, len(x))
        
        spline_x = CubicSpline(t, x)
        spline_y = CubicSpline(t, y)
        
        t_fine = np.linspace(0, 1, len(x) * num_points)
        interpolated_points = [(spline_x(ti), spline_y(ti)) for ti in t_fine]
        return interpolated_points
    
    # diffuseColor 0 0.75 0.9 : celeste
    def create_point_node(self, x, y, z):
        return f"""
            Transform {{
                translation {x} {y} {z}
                children [
                    Shape {{
                        appearance Appearance {{
                            material Material {{
                                diffuseColor 1 0 0
                                transparency 0.2
                            }}
                        }}
                        geometry Sphere {{
                            radius 1
                            subdivision 2
                        }}
                    }}
                ]
            }}
        """
    
    def visualize_points(self):
        root = self.getRoot()
        children_field = root.getField('children')
        
        # Crear el nodo Transform principal
        node_string = "DEF TRAJECTORY Group { children ["
        
        # Altura constante para la visualización
        #height = 1290
        
        # Añadir cada punto como un nodo hijo
        for point in self.trajectory_points:
            node_string += self.create_point_node(point[0], point[1], point[2]+1)
        
        # Cerrar el nodo Group
        node_string += "]}"
        
        # Importar el nodo completo al mundo
        children_field.importMFNodeFromString(-1, node_string)
    
    def run(self):
        timestep = int(self.getBasicTimeStep())
        while self.step(timestep) != -1:
            pass

# Crear y ejecutar el controlador del supervisador
controller = TrajectoryVisualizer()
controller.run()