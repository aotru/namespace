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
  struct Vertex;  // forward declaration
 public:
  using index = size_t;
  SparseGraph() = default;
  SparseGraph(const SparseGraph&) = default;
  SparseGraph(SparseGraph&&);  // TODO(implement)
  SparseGraph& operator=(const SparseGraph&) = default;
  SparseGraph& operator=(SparseGraph&&); // TODO(implement)
  ~SparseGraph() = default;

  void AddVertex(const T& val); // O(1)
  bool VertexExists(const T& val) const ; // O(1)
  void AddNeighbor(const T& source, const T& destination); // O(1)
  void AddNeighbor(const std::pair<T, T>& src_and_dest); // TODO(implement)
  void RemoveNeighbor(const T& source, const T& destination); // O(E/V)
  void RemoveNeighbor(const std::pair<T, T>& src_and_dest); // TODO(implement)
  std::vector<T> GetNeighbors(const T& val) const; // O(E/V)
  bool EdgeExists(const T& source, const T& destination) const; // O(E/V)
 private:
  std::vector<Vertex> graph_;
  std::unordered_map<T, index> map_;
  struct Vertex {
      friend SparseGraph; // not entirely sure if this is needed
      Vertex() = default;
      Vertex(const T& val): value_(val) {};
      T value_;
      std::vector<index> neighbors_;
   };
};

template <typename T>
inline void SparseGraph<T>::AddVertex(const T& val) {
   map_.insert(std::make_pair(val, std::size(graph_)));
   graph_.push_back(val);
}

template <typename T>
inline bool SparseGraph<T>::VertexExists(const T& val) const {
   return map_.find(val) != std::end(map_);
}

template <typename T>
inline void SparseGraph<T>::AddNeighbor(const T &source, const T &destination) {
   index src_idx = map_[source];
   index dest_idx = map_[destination];
   graph_[src_idx].push_back(dest_idx);
}

template <typename T>
inline void SparseGraph<T>::RemoveNeighbor(const T &source, 
                                           const T &destination) {
  auto& neighbors = graph_[map_[source]].neighbors;
  neighbors.erase(std::find(std::begin(neighbors), 
                  std::end(neighbors), map_[destination]));
}

template <typename T>
inline std::vector<T> SparseGraph<T>::GetNeighbors(const T& val) const {
   const auto& neighbors = graph_[map_[val]].neighbors;
   std::vector<T> tmp;
   tmp.reserve(std::size(neighbors));
   for (auto& neighbor: neighbors) {
      tmp.push_back(neighbor.value_);
   };
   return tmp;
}

template <typename T>
inline bool SparseGraph<T>::EdgeExists(const T& source, 
                                       const T& destination) const {
   const auto& neighbors = graph_[map_[source]].neighbors;
   return std::find(std::begin(neighbors), 
                    std::end(neighbors), map_[destination]) 
                    != std::end(neighbors);
}

}  // namespace dsa

#endif  // SPARSE_GRAPH_H_