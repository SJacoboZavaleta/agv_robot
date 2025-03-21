# Simulación de un robot UGV en un terreno irregular utilizando Webots
## Trabajo para el curso de Robots de campo
### Máster en Robótica y Automatización
#### Universidad Complutense de Madrid
>> Autor: Sergio Jacobo Zavaleta
>> Date: 21/03/2025
>> 100514566@alumnos.uc3m.es

## Proceso de instalación

### Paso 0: Clonar el repositorio

```bash
git https://github.com/SJacoboZavaleta/agv_robot.git
cd land_robot
```

### Paso 1: Crear un Entorno Virtual

Puedes usar `venv` (integrado en Python) o `conda` (de Anaconda/Miniconda).

#### Opción 1: Usar `venv` (Entorno Virtual de Python)

1. **Crear el entorno virtual:**
   Abre una terminal en la carpeta de tu proyecto y ejecuta:
   ```bash
   python -m venv venv
   ```
   Esto creará una carpeta llamada `venv` que contiene el entorno virtual.

2. **Activar el entorno virtual:**
   - En Windows:
     ```bash
     venv\Scripts\activate
     ```
   - En Linux/Mac:
     ```bash
     source venv/bin/activate
     ```

3. **Instalar las dependencias:**
   Si tienes un archivo `requirements.txt` con las dependencias, ejecuta:
   ```bash
   pip install -r requirements.txt
   ```
   Si no tienes un `requirements.txt`, puedes crearlo manualmente con las librerías necesarias.

#### Opción 2: Usar `conda` (Entorno Virtual de Anaconda)

1. **Crear el entorno virtual:**
   Abre una terminal y ejecuta:
   ```bash
   conda create --name moose_env python=3.8
   ```
   (Cambia `moose_env` por el nombre que desees y `python=3.8` por la versión de Python que uses).

2. **Activar el entorno virtual:**
   ```bash
   conda activate moose_env
   ```

3. **Instalar las dependencias:**
   Si tienes un archivo `requirements.txt`, ejecuta:
   ```bash
   pip install -r requirements.txt
   ```
   O instala las dependencias manualmente:
   ```bash
   pip install numpy  # Ejemplo de dependencia
   ```

---

### Paso 2: Verificar e Instalar las Librerías de Webots

Webots requiere algunas librerías específicas para interactuar con Python. Asegúrate de que estén instaladas en tu entorno virtual.

1. **Instalar las librerías necesarias:**
   Ejecuta el siguiente comando en tu entorno virtual:
   ```bash
   pip install numpy
   ```
   (Webots ya incluye su propia API de Python, pero si usas otras librerías como `numpy`, asegúrate de instalarlas).

2. **Verificar la API de Webots:**
   Webots incluye su propia API de Python, que se encuentra en la instalación de Webots. No necesitas instalarla manualmente, pero debes asegurarte de que Webots esté correctamente configurado para usar tu entorno virtual.

---

### Paso 3: Configurar Webots para Usar tu Entorno Virtual

Webots necesita saber qué intérprete de Python usar. Aquí te explico cómo configurarlo.

1. **Abrir Webots:**
   Inicia Webots y abre tu proyecto.

2. **Configurar el intérprete de Python:**
   - Ve a `Tools > Preferences > General > Python command`.
   - Especifica la ruta al intérprete de Python de tu entorno virtual.
     - Si usas `venv`, la ruta será algo como:
       - Windows: `ruta\al\proyecto\venv\Scripts\python.exe`
       - Linux/Mac: `ruta/al/proyecto/venv/bin/python`
     - Si usas `conda`, la ruta será algo como:
       - Windows: `C:\Users\tu_usuario\Anaconda3\envs\moose_env\python.exe`
       - Linux/Mac: `/home/tu_usuario/anaconda3/envs/moose_env/bin/python`

3. **Guardar la configuración:**
   Guarda los cambios y reinicia Webots si es necesario.

---

### Paso 4: Estructurar el Proyecto

Para que tu proyecto sea portable, es importante organizar los archivos de manera clara. Aquí tienes una estructura sugerida:

```
agv_robot/
├── controllers/
│   ├── moose_controller/
│   │   ├── moose_controller.py
│   │   └── paths/
│   │       ├── path_combined_purchena_h.csv
│   │       ├── path_combined_purchena_s.csv
│   │       ├── path_combined_purchena_z.csv
│   │       ├── path_potential_energy_purchena_h.csv
│   │       ├── path_potential_energy_purchena_s.csv
│   │       ├── path_potential_energy_purchena_z.csv
│   │       ├── path_slope_purchena_h.csv
│   │       ├── path_slope_purchena_s.csv
│   │       └── path_slope_purchena_z.csv
│   │       └── ...                   # Otros archivos de trayectorias
│   │
│   └── moose_supervisor/
│       ├── moose_supervisor.py
│       └── paths/
│           ├── path_combined_purchena_h.csv
│           ├── path_combined_purchena_s.csv
│           ├── path_combined_purchena_z.csv
│           ├── path_potential_energy_purchena_h.csv
│           ├── path_potential_energy_purchena_s.csv
│           ├── path_potential_energy_purchena_z.csv
│           ├── path_slope_purchena_h.csv
│           ├── path_slope_purchena_s.csv
│           └── path_slope_purchena_z.csv
│           └── ...                   # Otros archivos de trayectorias
│
├── worlds/
│   ├── land.wbt
│   ├── .land.jpg
│   └── uc3m_logo.png
│   └── ...                           # Otros mundos o recursos
│
├── results/
│   ├── simulation_results_1_1.txt
│   ├── simulation_results_2_2.txt
│   └── simulation_results_3_3.txt
│   └── ...                           # Otros archivos de resultados
│
├── requirements.txt
│
└── README.md
```

#### Paso 5: Ejecutar el mundo en Webots:

- Abrir `worlds/moose_world.wbt`. Presionar pausa y luego retroceso inicial de la simulación.
- Ejecutar la simulación.  
- Abrir `moose_controller.py` y `moose_supervisor.py` y definir la ruta del controlador y supervisor respectivamente (ambos por igual).  
   - Escoger `cost_function` entre 1 (ruta 1), 2 (ruta 2) y 3 (ruta 3).
   - Escoger `slope_method` entre 1 (pendiente tipo Sobel), 2 (pendiente tipo Horn) y 3 (pendiente tipo Zevenbergen). 
- Ejecutar la simulación.


## Requisitos

- Webots (> versión 2023b)
- Python 3.8+
- Dependencias: `numpy` y `scipy`