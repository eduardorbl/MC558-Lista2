# include <vector>
# include <iostream>
# include <algorithm>

/**
 * Classe UnionFind
 * @brief Implementa a estrutura de dados Union-Find
 * 
 * @param parent Vetor de pais que indica o pai de cada elemento
 * @param rank Vetor de rank que indica a "altura" de cada conjunto
 */
class UnionFind {
private:
    /**
     * @brief Vetor de pais que indica o pai de cada elemento
     */
    std::vector<int> parent; // parent[i] é o pai de i
    /**
     * @brief Vetor de rank que indica a "altura" de cada conjunto
     */
    std::vector<int> rank;   // rank[i] é o rank (altura) de cada conjunto
public:
    /**
     * @brief Construtor da classe UnionFind
     * 
     * @param n Número de elementos do conjunto
     */
    UnionFind(int n) {
        parent.resize(n);
        rank.resize(n);
        for (int i = 0; i < n; i++) {
            parent[i] = i; // Cada elemento é seu próprio pai no início
            rank[i] = 0; // Cada conjunto tem altura 0 no início
        }
    }

    /**
     * @brief Função find que retorna o pai de um elemento
     * 
     * @param i Elemento cujo pai será retornado
     * @return Pai do elemento i
     */
    int find(int i) {
        if (parent[i] != i) {
            parent[i] = find(parent[i]);
        }
        return parent[i];
    }

    /**
     * @brief Função union_set que une dois conjuntos disjuntos
     * 
     * @param u Elemento do primeiro conjunto
     * @param v Elemento do segundo conjunto
     */
    void union_set(int u, int v) {
        int rootU = find(u);
        int rootV = find(v);
        if (rootU != rootV) {
            // União por rank
            if (rank[rootU] > rank[rootV]) {
                parent[rootV] = rootU;
            } else if (rank[rootU] < rank[rootV]) {
                parent[rootU] = rootV;
            } else {
                parent[rootV] = rootU;
                rank[rootU]++;
            }
        }
    }
};

/**
 * Classe Graph
 * @brief Implementa um grafo com arestas ponderadas
 * 
 * @param edges Vetor de arestas do grafo
 * @param V Número de vértices do grafo
 */
class Graph {
private:
    /**
     * @brief Vetor de arestas do grafo
     */
    std::vector<std::tuple<int, int, int>> edges;
    /**
     * @brief Número de vértices do grafo
     */
    int V;

public:
    /**
     * @brief Construtor da classe Graph
     * 
     * @param V Número de vértices do grafo
     */
    Graph(int V) : V(V) {}

    /**
     * @brief Função add_edge que adiciona uma aresta ao grafo
     * 
     * @param u Vértice de origem da aresta
     * @param v Vértice de destino da aresta
     * @param c Peso da aresta
     */
    void add_edge(int u, int v, int c) {
        edges.push_back({c, u, v});
    }

    /**
     * @brief Função krushkal que calcula o peso total da MST
     * 
     * @return Peso total da MST
     */
    int krushkal() {
        std::sort(edges.begin(), edges.end());
        UnionFind uf(V);
        int cost = 0;
        for (auto [c, u, v] : edges) {
            if (uf.find(u) != uf.find(v)) {
                uf.union_set(u, v);
                cost += c;
            }
        }
        return cost;
    }
};

/**
 * @brief Função principal
 * 
 * @return 0
 */
int main() {
    int M, N;
    while (true) {
        std::cin >> M >> N;

        // Condição de parada
        if (M == 0 && N == 0) break;

        // Criação do grafo com M vértices
        Graph g(M);
    
        // Leitura das N arestas
        for (int i = 0; i < N; ++i) {
            int X, Y, Z;
            std::cin >> X >> Y >> Z;
            g.add_edge(X, Y, Z);
        }

        // Calcula o peso total da MST
        int mstWeight = g.krushkal();

        std::cout << mstWeight << std::endl;
    }

    return 0;
}
