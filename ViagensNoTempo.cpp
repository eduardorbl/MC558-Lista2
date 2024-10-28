#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <set>
#include <limits>
using namespace std;

/*
    Esse código utiliza o algoritmo de Stoer-Wagner para encontrar o corte mínimo de um grafo.
    Isso permite que nós atinjamos uma complexidade de O(V^3) para encontrar o corte mínimo de um grafo.
    
    Encontrando o corte mínimo de um grafo atingimos o objetivo do problema: "Viagens no Tempo" 2082 do beecrowd.
*/

/**
 * 
 * @brief Classe que representa um grafo não direcionado utilizando lista de adjacência.
 * 
 * Utiliza um unordered_map para representar o grafo, onde a chave é o vértice e o valor é um outro unordered_map
 * onde as chaves são os vértices adjacentes e o valor é o custo da aresta.
 * 
 * Portanto adj[u][v] = c significa que existe uma aresta entre u e v com custo c.
 * 
 */
class Graph {
public:
    /**
     * @brief Lista de adjacência do grafo.
     * 
     * O mapa externo é uma chave para outro mapa interno.
     * O mapa interno é um mapeamento de vértices adjacentes para o custo da aresta.
     */
    unordered_map<int, unordered_map<int, int>> adj;

    /**
     * @brief Adiciona uma aresta entre os vértices u e v com custo c.
     * 
     * @param u um dos vértices da aresta
     * @param v outro vértice da aresta
     * @param c custo da aresta
     * 
     * @return void
     */
    void add_edge(int u, int v, int c) {
        adj[u][v] = c; // Adiciona a aresta u -> v
        adj[v][u] = c; // Adiciona a aresta v -> u
    }

    /**
     * @brief Contrai o vértice `u` com o vértice `v`. 
     * 
     * Transfere todas as arestas de `u` para `v` e remove o vértice `u` do grafo.
     * 
     * Caso u e v tenham o mesmo vértice adjacente w, a aresta entre u e w terá seu custo somado ao custo da aresta entre v e w.
     * 
     * @param u vértice a ser contraído
     * @param v vértice a ser mantido
     * 
     * @return void
     * 
     */
    void contract(int u, int v) {
        for (auto& [w, cost] : adj[u]) {
            if (w != v) {
                if (adj[v].count(w)) {
                    adj[v][w] += cost;  // Soma o custo existente com o novo custo
                    adj[w][v] += cost;
                } else {
                    adj[v][w] = cost;
                    adj[w][v] = cost;
                }
            }
            adj[w].erase(u);  // Remove a referência de `w` para `u`
        }
        adj.erase(u);  // Remove completamente o vértice `u`
    }

};

/**
 * @brief Classe que representa um heap máximo.
 * 
 * Utiliza um priority_queue para manter os elementos ordenados de acordo com o peso.
 * Também utiliza um unordered_map para manter o peso de cada elemento.
 */
class MaxHeap {
public:
    /**
     * @brief Priority queue que mantém os elementos ordenados de acordo com o peso.
     * 
     * A prioridade é o peso do elemento e os elementos são pares (peso, elemento).
     */
    priority_queue<pair<int, int>> pq;
    /**
     * @brief unordered_map que mantém o peso de cada elemento.
     * 
     * Auxilia na atualização do peso de um elemento.
     */
    unordered_map<int, int> weights;

    /**
     * @brief Insere um elemento no heap.
     * 
     * @param weight peso do elemento
     * @param node elemento a ser inserido
     * 
     * @return void
     */
    void push(int weight, int node) {
        weights[node] = weight;
        pq.push({weight, node});
    }

    /**
     * @brief Extrai o elemento de maior peso do heap.
     * 
     * @return pair<int, int> par (peso, elemento) do elemento de maior peso ou (-1, -1) caso o heap esteja vazio.
     */
    pair<int, int> extract_max() {
       while (!pq.empty()) {
            int node = pq.top().second;
            int weight = pq.top().first;
            pq.pop();
            if (weights[node] == weight) {
                weights.erase(node);
                return {weight, node};
            }
        }
        return {-1, -1}; // Apenas para evitar retorno vazio
    }

    /**
     * @brief Atualiza o peso de um elemento.
     * 
     * @param node elemento a ser atualizado
     * @param increase incremento do peso
     * 
     * @return void
     */
    void increase_value(int node, int increase) {
        if (weights.count(node)) {
            int new_weight = weights[node] + increase;
            weights[node] = new_weight;
            pq.push({new_weight, node});
        }
    }
};

/**
 * @brief Classe que representa um conjunto disjunto.
 * 
 * Utiliza um set para manter os vértices do conjunto e um MaxHeap para manter os vértices adjacentes ao set
 */
class UnionFindSet {
public:
    /**
     * @brief Set que mantém os vértices do conjunto.
     */
    set<int> vertices;
    /**
     * @brief Set que mantém os vértices adjacentes ao conjunto.
     */
    set<int> adj;
    /**
     * @brief MaxHeap que mantém os vértices adjacentes ao conjunto ordenados de acordo com o peso.
     * 
     * Auxilia na escolha do vértice de maior peso para ser adicionado ao conjunto (algoritmo de Stoer-Wagner).
     */
    MaxHeap adj_heap;

    /**
     * @brief Adiciona um vértice ao conjunto.
     * 
     * Adiciona o vértice ao set de vértices e adiciona os vértices adjacentes ao MaxHeap.
     * 
     * @param u vértice a ser adicionado
     * @param graph grafo que contém a lista de vértices adjacentes
     * 
     * @return void
     */
    void add_vertex(int u, Graph& graph) {
        vertices.insert(u);
        for (auto& [v, cost] : graph.adj[u]) {
            if (!vertices.count(v)) {
                if (adj.count(v)) {
                    adj_heap.increase_value(v, cost);
                } else {
                    adj.insert(v);
                    adj_heap.push(cost, v);
                }
            }
        }
    }

    /**
     * @brief Extrai o vértice de maior peso do conjunto.
     * 
     * @return pair<int, int> par (peso, vértice) do vértice de maior peso.
     */
    pair<int, int> extract_max() {
        auto max_pair = adj_heap.extract_max();
        adj.erase(max_pair.second);
        return max_pair;
    }
};

/**
 * @brief Classe que realiza o algoritmo de Stoer-Wagner.
 * 
 * Utiliza um grafo e o algoritmo de Stoer-Wagner para encontrar o corte mínimo do grafo.
 */
class StoerWagner {
public:
    /**
     * @brief Grafo que será utilizado para encontrar o corte mínimo.
     */
    Graph graph;

    /**
     * @brief Realiza uma iteração do algoritmo de Stoer-Wagner.
     * 
     * @return int corte mínimo da fase
     */
    int min_cut_phase() {
        vector<int> vertices;
        for (auto& [u, _] : graph.adj) vertices.push_back(u);

        UnionFindSet union_set;
        union_set.add_vertex(vertices[0], graph); // Adiciona o primeiro vértice ao conjunto

        int last_vertex = vertices[0];
        int min_cut_cost = 0;
        int len_v = vertices.size() - 1;
        int second_last_vertex;

        // Adiciona os vértices fortemente ligados ao conjunto e registra o corte mínimo 
        for (int i = 0; i < len_v; i++) {
            auto [max_weight, max_vertex] = union_set.extract_max();
            if (i == len_v - 1) {
                min_cut_cost = max_weight;
                second_last_vertex = last_vertex;
            }
            union_set.add_vertex(max_vertex, graph);
            last_vertex = max_vertex;
        }

        // Contrai os dois últimos vértices da iteração (Algoritmo de Stoer-Wagner)
        graph.contract(last_vertex, second_last_vertex);
        return min_cut_cost;
    }

    /**
     * @brief Encontra o corte mínimo do grafo utilizando o algoritmo de Stoer-Wagner.
     * 
     * @return int corte mínimo do grafo
     */
    int min_cut() {
        int min_cut = numeric_limits<int>::max();
        // Realiza a iteração e salva o mínimo valor de corte que será o corte mínimo do grafo (algoritmo de Stoer-Wagner)
        while (graph.adj.size() > 1) {
            min_cut = min(min_cut, min_cut_phase());
        }
        return min_cut;
    }
};

/**
 * @brief Função principal.
 * 
 * Lê o número de casos de teste e para cada caso de teste lê o número de vértices e arestas.
 * Lê as arestas e seus custos e cria um grafo com essas informações.
 * 
 * Cria um objeto da classe StoerWagner e encontra o corte mínimo do grafo.
 * 
 * @return int 0
 */
int main() {
    int T;
    cin >> T;

    for (int i = 0; i < T; i++) {
        int N, M;
        cin >> N >> M;

        if (M == 0) {
            std::cout << 0 << std::endl;
            continue;
        }

        // Salvar as entradas u, v e c em uma lista para printar depois
        vector<tuple<int, int, int>> edges;

        Graph graph;
        for (int j = 0; j < M; j++) {
            int u, v, c;
            cin >> u >> v >> c;
            edges.push_back({u, v, c});
            graph.add_edge(u, v, c);
        }

        StoerWagner stoer_wagner;
        stoer_wagner.graph = graph;
        int min_cut = stoer_wagner.min_cut();
        cout << min_cut << endl;

    }

    return 0;
}