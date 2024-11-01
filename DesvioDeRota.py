import heapq
import sys

def dijkstra(n, graph, start, end, route_service_end):
    # Inicializar as distâncias como infinito
    distances = [float('inf')] * n
    distances[start] = 0
    queue = [(0, start)]  # fila de prioridade com (distância, cidade)

    while queue:
        current_distance, u = heapq.heappop(queue)

        # Ignorar se a distância atual não é a mínima conhecida
        if current_distance > distances[u]:
            continue

        # Verificar todas as cidades vizinhas
        for v, weight in graph[u]:
            # Checar se estamos dentro da rota de serviço
            if u < route_service_end:
                if v == u + 1:  # Seguir para a próxima cidade na rota
                    new_distance = current_distance + weight
                    if new_distance < distances[v]:
                        distances[v] = new_distance
                        heapq.heappush(queue, (new_distance, v))
            else:
                # Fora da rota de serviço, todas as cidades vizinhas são válidas
                new_distance = current_distance + weight
                if new_distance < distances[v]:
                    distances[v] = new_distance
                    heapq.heappush(queue, (new_distance, v))

    return distances[end]

def main():
    while True:
        # Ler a entrada
        n, m, c, k = map(int, input().split())
        if n == 0 and m == 0 and c == 0 and k == 0:
            break
        
        # Construir o grafo
        graph = [[] for _ in range(n)]
        for _ in range(m):
            u, v, p = map(int, input().split())
            graph[u].append((v, p))
            graph[v].append((u, p))

        # Obter o menor custo para chegar ao destino da rota de serviço
        result = dijkstra(n, graph, k, c - 1, c - 1)
        print(result)

# Executar o programa
if __name__ == "__main__":
    main()
