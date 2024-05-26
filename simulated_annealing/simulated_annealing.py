import math
import random
import urllib.request
import folium 
import webbrowser
import time

class sim_anneal(object):
  def __init__(self, coords, T=5e3, alpha=0.999, stop_t = 1e-3, max_convergence_iters=1000):
    """ Inicializa el algoritmo de recocido simulado con los parámetros de temperatura, enfriamiento y umbral de parada 
    que pueden ser definidos por el usuario o ser los valores por defecto"""
    self.coords = coords
    self.N = len(coords)
    self.T = T
    self.alpha = alpha
    self.stop_t = stop_t
    self.iteration = 0
    self.nodes = [i for i in range(self.N)] # Lista de nodos
    self.best_solution = None
    self.best_distance = float("Inf")
    self.max_convergence_iters = max_convergence_iters
    self.accepted_sol = None
    self.accepted_distance = float("Inf")

  def init_solution(self):
    """ Genera una solución aleatoria inicial para el problema del agente viajero"""
    self.generate_distance_matrix() # Genera la matriz de distancias
    self.accepted_sol = [0] #0 es el nodo orígen
    free_nodes = set(self.nodes) # Crea un conjunto, algunas operaciones son más rápidas que con listas
    free_nodes.remove(0) 

    while (free_nodes):
      # Selecciona un nodo aleatorio de los nodos libres
      next_node = random.choice(list(free_nodes))
      free_nodes.remove(next_node)
      self.accepted_sol.append(next_node)

    self.accepted_distance = self.total_distance(self.accepted_sol) # Calcula la distancia total de la solución
    self.best_solution, self.best_distance = list(self.accepted_sol), self.accepted_distance
    return self.accepted_distance, self.accepted_distance

  def generate_distance_matrix(self):
    """ Genera una matriz de distancias entre los nodos """
    self.dist_matrix = [[self.distance(i, j) for j in range(self.N)] for i in range(self.N)]

  def get_tour(self):
    """ Devuelve la mejor solución encontrada """
    return self.best_solution
  
  def distance(self, node_0, node_1):
    """ Calcula la distancia euclidiana entre dos nodos """
    coord_0, coord_1 = self.coords[node_0], self.coords[node_1]
    return math.sqrt((coord_0[0] - coord_1[0])**2 + (coord_0[1] - coord_1[1])**2)
  
  def total_distance(self, solution):
    """ Calcula la distancia total de una solución a partir de la matriz de distancias """
    distance = 0
    for i in range(self.N - 1):
        distance += self.dist_matrix[solution[i]][solution[i+1]]
    distance += self.dist_matrix[solution[self.N - 1]][solution[0]]
    return distance
  
  def probability_acceptance(self, candidate_distance):
    """ Calcula la probabilidad de aceptar un candidato """
    return math.exp(-abs(candidate_distance - self.accepted_distance) / self.T)
  
  def accept(self, candidate):
    """ Toma como entrada una solución candidata y la acepta inmediatamente si es mejor que la mejor solución actual 
    o con una probabilidad dada por la función de probabilidad de aceptación """
    candidate_distance = self.total_distance(candidate)

    if (candidate_distance - self.accepted_distance) < 0: # Si la solución candidata es mejor que la actual siempre se acepta
        self.accepted_distance, self.accepted_sol  = candidate_distance, candidate
        if self.accepted_distance < self.best_distance:
          self.best_distance, self.best_solution = candidate_distance, candidate
          return self.validate_best_solution()

    elif (candidate_distance - self.accepted_distance) > 0: # Si la solución candidata es peor que la actual se acepta con una probabilidad dada
      if random.random() <= self.probability_acceptance(candidate_distance):
          self.accepted_distance, self.accepted_sol = candidate_distance, candidate
          return self.validate_best_solution()
    return False
          

  def validate_best_solution(self):
    """ Valida si se debe actualizar la mejor solución encontrada """
    if self.accepted_distance < self.best_distance:
        self.best_distance, self.best_solution = self.accepted_distance, self.accepted_sol
        return True
    return False
  

  def do_annealing(self, temp=-1, alpha=-1, stop_t=-1, stop_iters=-1, max_convergence_iters = -1, initial_solution=None):
    # Se permite cambiar los parámetros de temperatura, enfriamiento y umbral de parada
    if temp > 0:
        self.T = temp
    if alpha > 0:
        self.alpha = alpha
    if stop_t > 0:
        self.stop_t = stop_t
    if stop_iters > 0:
        self.stop_iters = stop_iters
    if max_convergence_iters > 0:
        self.max_convergence_iters = max_convergence_iters
    self.__annealing_temperature(initial_solution)

  def __annealing_temperature(self, initial_solution=None):
    """ Ejecuta el algoritmo de recocido simulado con una temperatura inicial y una solucion inicial que puede ser
    definida por el usuario o generada aleatoriamente"""
    
    if initial_solution is None:
        self.best_solution, self.best_distance = self.init_solution()
    else:
        self.best_solution, self.best_distance = initial_solution, self.total_distance(initial_solution)
        self.accepted_sol, self.accepted_distance = self.best_solution, self.best_distance

    T = self.T
    while ((self.T >= self.stop_t)):
        for i in range(int(self.max_convergence_iters)): # Condicion de convergencia a una temperatura
          candidate_solution = list(self.accepted_sol) # Copia la solución actual
          self.__swap(candidate_solution) # Genera una solución candidata
          if self.accept(candidate_solution): # Acepta o no la solución candidata
             break # Si se acepta la solución candidata y es mejor que la mejor solución se asume convergencia
        self.T *= self.alpha # Disminuye la temperatura  
    self.T = T

  def __swap(self, candidate_solution):
    """ Intercambia dos nodos aleatorios en una solución, excepto el nodo inicial """
    i = random.randint(1, self.N - 1)
    j = random.randint(1, self.N - 1)
    aux = candidate_solution[i]
    candidate_solution[i] = candidate_solution[j]
    candidate_solution[j] = aux
    return candidate_solution
  
def read_cities(url):
    try:
        # Descargar el contenido del archivo desde la URL
        response = urllib.request.urlopen(url)
        data = response.read().decode('utf-8')
        
        # Inicializa un diccionario vacío
        cities_dict = {}
        
        # Divide el contenido en líneas
        lines = data.strip().split('\n')
        
        # Procesa cada línea
        for line in lines:
            parts = line.split(',')
            if len(parts) == 3:
                city = parts[0].strip()
                lat = float(parts[1].strip())
                long = float(parts[2].strip())
                cities_dict[city] = (lat, long)
        
        return cities_dict
    except Exception as e:
        print(f"Error al leer el archivo desde la URL: {e}")
        return None


def do_overheating(sa, t=None, alphas=None, coverg_iters=None):
   """ Encuentra una solución inicial y hace un recalentamiento para tratar de encontrar una mejor solución 
   Asume que el número de repeticiones, la temperatura y el enfriamiento son los mismos
   los parametros repeat, t, alphas, se envian en listas"""

   if len(coverg_iters) == len(t) == len(alphas) and t is not None:
       sa.do_annealing(temp=t[0], alpha=alphas[0], max_convergence_iters=coverg_iters[0])
       for i in range(len(t) - 1):
           sa.do_annealing(temp=t[i+1], alpha=alphas[i+1], max_convergence_iters=coverg_iters[i+1], initial_solution=sa.best_solution)
   else:
       print("Los parámetros de covergencia, temperatura y enfriamiento deben tener la misma longitud")
   


if __name__ == '__main__':
    url = 'https://raw.githubusercontent.com/CerealKilleer/tsp/main/ciudades/tsp50.txt'
    coordinates_cities = read_cities(url)
    cities = list(coordinates_cities.keys())
    coordinates = list(coordinates_cities.values())
    start = time.time()
    sa = sim_anneal(coordinates)
    do_overheating(sa, t=[10e3, 10e3], alphas=[0.99, 0.99], coverg_iters=[1000, 1000])
    end = time.time()
    tour = sa.get_tour()
    map = folium.Map(location=[-15,-60], zoom_start = 4)

    points = []
    for city in tour:
      points.append(coordinates[city])
    points.append(points[0])  

    folium.PolyLine(points, color='red').add_to(map)
    print("Distancia total: ", sa.best_distance)
    print("Tiempo del recocido simulado: ", end-start)
    map.save('map.html')
    webbrowser.open('map.html')
