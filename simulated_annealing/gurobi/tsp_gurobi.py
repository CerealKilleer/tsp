"""Esta es una adaptación del código de la solución del TSP disponible en gurobi"""
import urllib.request
import math
from itertools import combinations
import gurobipy as gp
from gurobipy import GRB
import webbrowser
import folium

url = 'https://raw.githubusercontent.com/CerealKilleer/tsp/main/ciudades/tsp10.txt'

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


def distance(city1, city2):
    """Calcula la distancia euclidiana entre dos ciudades y las almacena en un diccionario"""
    c1 = coordinates[city1]
    c2 = coordinates[city2]
    diff = (c1[0]-c2[0], c1[1]-c2[1])
    return math.sqrt(diff[0]*diff[0]+diff[1]*diff[1])


# Callback - usa lazy constraints para eliminar subciclos

def subtourelim(model, where):
    if where == GRB.Callback.MIPSOL:
        # make a list of edges selected in the solution
        vals = model.cbGetSolution(model._vars)
        selected = gp.tuplelist((i, j) for i, j in model._vars.keys()
                             if vals[i, j] > 0.5)
        # find the shortest cycle in the selected edge list
        tour = subtour(selected)
        if len(tour) < len(cities):
            # add subtour elimination constr. for every pair of cities in subtour
            model.cbLazy(gp.quicksum(model._vars[i, j] for i, j in combinations(tour, 2))
                         <= len(tour)-1)

# Dada una lista de aristas, determina la longitud del ciclo más corto

def subtour(edges):
    unvisited = cities[:]
    cycle = cities[:] # Dummy - guaranteed to be replaced
    while unvisited:  # true if list is non-empty
        thiscycle = []
        neighbors = unvisited
        while neighbors:
            current = neighbors[0]
            thiscycle.append(current)
            unvisited.remove(current)
            neighbors = [j for i, j in edges.select(current, '*')
                         if j in unvisited]
        if len(thiscycle) <= len(cycle):
            cycle = thiscycle # New shortest subtour
    return cycle


# Llama a la función para leer las ciudades y las almacena  en un diccionario
coordinates = read_cities(url)
cities = list(coordinates.keys())
# Crea un diccionario con las distancias entre las ciudades
dist = {(c1, c2): distance(c1, c2) for c1, c2 in combinations(cities, 2)}


# Crea el modelo
m = gp.Model()

# Variables: es la ciudad 'i' adjacente a la ciudad 'j' en el tour?
vars = m.addVars(dist.keys(), obj=dist, vtype=GRB.BINARY, name='x')

# Direccion simétrica: usa dict.update para agregar una tupla con las mismas claves
vars.update({(j,i):vars[i,j] for i,j in vars.keys()})

# Restricciones: cada ciudad debe ser adyacente a exactamente dos ciudades
cons = m.addConstrs(vars.sum(c, '*') == 2 for c in cities)

m._vars = vars
m.Params.lazyConstraints = 1
m.optimize(subtourelim)
print("Tiempo de optimización: ", m.Runtime)

# Recupera la solución y la imprime
vals = m.getAttr('x', vars)
selected = gp.tuplelist((i, j) for i, j in vals.keys() if vals[i, j] > 0.5)
tour = subtour(selected)
assert len(tour) == len(cities)
map = folium.Map(location=[-15,-60], zoom_start = 4)

points = []
for city in tour:
  points.append(coordinates[city])
points.append(points[0])

folium.PolyLine(points).add_to(map)
map.save('map_gurobi.html')
webbrowser.open('map_gurobi.html')
