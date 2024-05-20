import math
import random
import urllib.request
import folium 
import webbrowser

class sim_anneal(object):
  def __init__(self, coords, T=5e3, alpha=0.999, stop_t = 1e-3, stop_iters=10e3):
    self.coords = coords
    self.N = len(coords)
    self.T = T
    self.alpha = alpha
    self.stop_t = stop_t
    self.stop_iters = 1e-3
    self.iteration = 1
    self.nodes = [i for i in range(self.N)]
    self.best_solution = None
    self.best_distance = float("Inf")

  def init_solution(self):
    """ Genera una solución aleatoria inicial para el problema del agente viajero"""

    self.best_solution = [0] #0 es el nodo orígen
    free_nodes = set(self.nodes) # Crea un conjunto, algunas operaciones son más rápidas que con listas
    free_nodes.remove(0) 

    while (free_nodes):
      # Selecciona un nodo aleatorio de los nodos libres
      next_node = random.choice(list(free_nodes))
      free_nodes.remove(next_node)
      self.best_solution.append(next_node)

    self.best_distance = self.total_distance(self.best_solution) # Calcula la distancia total de la solución
    return self.best_solution, self.best_distance

  def get_tour(self):
    """ Devuelve la mejor solución encontrada """
    return self.best_solution
  
  def distance(self, node_0, node_1):
    """ Calcula la distancia euclidiana entre dos nodos """
    coord_0, coord_1 = self.coords[node_0], self.coords[node_1]
    return math.sqrt((coord_0[0] - coord_1[0])**2 + (coord_0[1] - coord_1[1])**2)
  
  def total_distance(self, solution):
    """ Calcula la distancia total de una solución """
    return sum([self.distance(solution[i % self.N], solution[(i+1) % self.N]) for i in range(self.N)])
  
  def probability_acceptance(self, candidate_distance):
    """ Calcula la probabilidad de aceptar un candidato """
    return math.exp(-abs(candidate_distance - self.best_distance) / self.T)
  
  def accept(self, candidate):
    """ Toma como entrada una solución candidata y la acepta inmediatamente si es mejor que la mejor solución actual 
    o con una probabilidad dada por la función de probabilidad de aceptación """
    candidate_distance = self.total_distance(candidate)

    if candidate_distance < self.best_distance:
        self.best_distance, self.best_solution = candidate_distance, candidate
    elif random.random() <= self.probability_acceptance(candidate_distance):
        self.best_distance, self.best_solution = candidate_distance, candidate
    
  def do_annealing(self, temp=-1, alpha=-1, stop_t=-1, stop_iters=-1, initial_solution=None,do_iters=False):
    # Se permite cambiar los parámetros de temperatura, enfriamiento y umbral de parada
    if temp != -1:
        self.T = temp
    if alpha != -1:
        self.alpha = alpha
    if stop_t != -1:
        self.stop_t = stop_t
    if stop_iters != -1:
        self.stop_iters = stop_iters

    if not do_iters:
        """ Se permite cambiar los parámetros de temperatura, enfriamiento y umbral de parada
          si se quiere hacer el recocido con parada por temperatura
        """
        self.__annealing_temperature(initial_solution)
    else:
        
        self.__anneling_iters(initial_solution)


  def __anneling_iters(self, initial_solution=None):
    """ Ejecuta el algoritmo de recocido simulado con un número fijo de iteraciones """
    
    if initial_solution is None:
        self.best_solution, self.best_distance = self.init_solution()
    else:
        self.best_solution, self.best_distance = initial_solution, self.total_distance(initial_solution)

    self.iteration = 0
    while self.iteration < self.stop_iters:
        candidate_solution = list(self.best_solution)
        self.__swap(candidate_solution)
        self.accept(candidate_solution)
        self.T *= self.alpha # Disminuye la temperatura
        self.iteration += 1

    print("Iteraciones con condicion de parada en iteraciones: ",  self.iteration)

  def __annealing_temperature(self, initial_solution=None):
    """ Ejecuta el algoritmo de recocido simulado con una temperatura inicial y una solucion inicial que pueden ser dadas"""
    
    if initial_solution is None:
        self.best_solution, self.best_distance = self.init_solution()
    else:
        self.best_solution, self.best_distance = initial_solution, self.total_distance(initial_solution)

    self.iteration = 0
    while self.T >= self.stop_t: # Mientras la temperatura sea mayor que el umbral de parada
        candidate_solution = list(self.best_solution) # Copia la solución actual
        self.__swap(candidate_solution) # Genera una solución candidata
        self.accept(candidate_solution) # Acepta o no la solución candidata
        self.T *= self.alpha # Disminuye la temperatura
        self.iteration += 1
    
    print("Iteraciones con condición de parada en temperatura: ", self.iteration)

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

if __name__ == '__main__':
    url = 'https://raw.githubusercontent.com/CerealKilleer/tsp/main/ciudades/tsp15.txt'
    coordinates_cities = read_cities(url)
    cities = list(coordinates_cities.keys())
    coordinates = list(coordinates_cities.values())
    sa = sim_anneal(coordinates, T=100e3, alpha=0.99)
    sa.do_annealing(initial_solution=None, do_iters=False)
   
    tour = sa.get_tour()
    map = folium.Map(location=[-15,-60], zoom_start = 4)

    points = []
    for city in tour:
      points.append(coordinates[city])
    points.append(points[0])

    folium.PolyLine(points, color='red').add_to(map)
    print("Distancia total: ", sa.total_distance(tour))
    map.save('map.html')
    webbrowser.open('map.html')
