// NOT DONE YET DO NOT JUDGE
#ifndef SPARSE_GRAPH_H_
#define SPARSE_GRAPH_H_

#include <vector>
#include <unordered_map>
#include <utility>
#include <algorithm>

namespace dsa {

template <typename T>
class SparseGraph {
  struct Vertex;
 public:
  SparseGraph() = default;
  SparseGraph(const SparseGraph&) = default;
  SparseGraph(SparseGraph&&);
  ~SparseGraph() = default;

  void AddVertex(T& val); //O(1)
  bool VertexExists(T& val); // O(1)
  void AddNeighbor(T& source, T& destination); // O(1)
  void RemoveNeighbor(T& source, T& destination); // O(E/V)
  std::vector<T> GetNeighbors(T& val); // O(E/V)
  bool EdgeExists(T& source, T& destination); // O(E/V)
 private:
  std::vector<Vertex> graph_;
  std::unordered_map<T, Vertex> map_;
  struct Vertex {
      Vertex(const T& val): value_(val) {};
      T value_;
      std::vector<size_t> neighbors_;
   };
};

template <typename T>
void SparseGraph<T>::AddVertex(T& val) {
   map_.insert(std::make_pair(val, std::size(graph_)));
   graph_.emplace_back(val);
}

template <typename T>
bool SparseGraph<T>::VertexExists(T& val) {
   return map_.find(val) != std::end(map_);
}

template <typename T>
void SparseGraph<T>::AddNeighbor(T &source, T &destination) {
   size_t src_idx = map_[source], dest_idx = map_[destination];
   graph_[src_idx].push_back(dest_idx);
}

template <typename T>
void SparseGraph<T>::RemoveNeighbor(T &source, T &destination) {
  auto& neighbors = graph_[map_[source]].neighbors;
  neighbors.erase(std::find(std::begin(neighbors), 
                  std::end(neighbors), map_[destination]));
}

template <typename T>
std::vector<T> SparseGraph<T>::GetNeighbors(T& val){
   auto& neighbors = graph_[map_[val]].neighbors;
   std::vector<T> tmp;
   tmp.reserve(std::size(neighbors));
   for (auto& neighbor: neighbors) {
      tmp.push_back(neighbor.value_);
   };
   return tmp;
}

template <typename T>
bool SparseGraph<T>::EdgeExists(T& source, T& destination){
   auto& neighbors = graph_[map_[source]].neighbors;
   return std::find(std::begin(neighbors), 
                    std::end(neighbors), map_[destination]) 
                    != std::end(neighbors);
}

}  // namespace dsa



#endif  // SPARSE_GRAPH_H