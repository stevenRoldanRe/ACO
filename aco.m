% Algoritmo de Optimización por Colonia de Hormigas (ACO)
% para encontrar el camino más corto en un grafo

clc; clear; close all;

% Definir los nodos y las conexiones del grafo
nodos = {'A', 'B', 'C', 'D', 'E', 'F'};
aristas = [
    1 2 2;
    1 4 4;
    2 3 3;
    2 5 7;
    3 5 6;
    4 6 5;
    5 6 2
];

% Crear la matriz de adyacencia dinámicamente
numNodos = length(nodos);
G = zeros(numNodos);
for i = 1:size(aristas, 1)
    G(aristas(i, 1), aristas(i, 2)) = aristas(i, 3);
    G(aristas(i, 2), aristas(i, 1)) = aristas(i, 3);
end

% Graficar el grafo
graphPlot = graph(G, nodos);
figure;
plot(graphPlot, 'EdgeLabel', graphPlot.Edges.Weight);
title('Grafo para la búsqueda del camino más corto');

numHormigas = 10;
numIteraciones = 100;
alfa = 1;  % Importancia de las feromonas
beta = 2;  % Importancia de la heurística
evaporacion = 0.5;
Q = 100;  % Cantidad de feromonas depositadas

% Inicialización de feromonas
feromonas = 0.1 * ones(numNodos);

% Ejecutar algoritmo ACO
for iter = 1:numIteraciones
    rutas = zeros(numHormigas, numNodos);
    costos = inf(numHormigas, 1);
    
    for h = 1:numHormigas
        % Cada hormiga empieza en A (nodo 1)
        ruta = [1];
        visitados = false(1, numNodos);
        visitados(1) = true;
        
        while ruta(end) ~= 6  % Hasta llegar a F (nodo 6)
            nodoActual = ruta(end);
            nodosPosibles = find(G(nodoActual, :) > 0 & ~visitados);
            
            if isempty(nodosPosibles)
                break;
            end
            
            % Probabilidades de transición
            tau = feromonas(nodoActual, nodosPosibles).^alfa;
            eta = (1 ./ G(nodoActual, nodosPosibles)).^beta;
            P = tau .* eta;
            P = P / sum(P);
            
            % Seleccionar nodo basado en probabilidad
            siguienteNodo = nodosPosibles(roulette_wheel(P));
            ruta = [ruta, siguienteNodo];
            visitados(siguienteNodo) = true;
        end
        
        if ruta(end) == 6
            rutas(h, 1:length(ruta)) = ruta;
            costos(h) = calcular_costo(ruta, G);
        end
    end
    
    % Actualizar feromonas
    feromonas = (1 - evaporacion) * feromonas;
    for h = 1:numHormigas
        if costos(h) < inf
            for j = 1:length(rutas(h, :)) - 1
                if rutas(h, j+1) == 0
                    break;
                end
                i = rutas(h, j);
                j = rutas(h, j+1);
                feromonas(i, j) = feromonas(i, j) + Q / costos(h);
                feromonas(j, i) = feromonas(i, j);
            end
        end
    end
end

% Mostrar mejor resultado
[mejorCosto, idx] = min(costos);
mejorRuta = rutas(idx, rutas(idx, :) > 0);
if isempty(mejorRuta)
    disp('No se encontró un camino válido entre A y F');
else
    mejorRutaNombres = nodos(mejorRuta);
    disp('Mejor ruta encontrada (por nodos):');
    disp(strjoin(mejorRutaNombres, ' -> '));
    disp(['Costo mínimo: ', num2str(mejorCosto)]);
end

% Función para calcular el costo de una ruta
function costo = calcular_costo(ruta, G)
    costo = 0;
    for i = 1:length(ruta) - 1
        costo = costo + G(ruta(i), ruta(i+1));
    end
end

% Función para selección por ruleta
function idx = roulette_wheel(P)
    r = rand;
    C = cumsum(P);
    idx = find(r <= C, 1, 'first');
end
