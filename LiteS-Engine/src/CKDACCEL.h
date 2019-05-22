#include <atomic>
#include <memory>
#include <numeric>
#include <stack>
#include <thread>
#include <vector>
#include "CMesh.h"

namespace ACCEL {

const size_t NAI = std::numeric_limits<size_t>::max();

class KDTree {
 public:
  std::vector<Vertex> vertices;
  struct Node {
    size_t ID;
    size_t vertexID;
    size_t splitAxis;
    union {
      size_t first;
      size_t left;
    };
    union {
      size_t last;
      size_t right;
    };
  };

  size_t numNodes;
  std::vector<Node> nodes;
  size_t createNode(size_t d, size_t first, size_t last) {
    size_t nodeID = numNodes++;
    Node& node = nodes[nodeID];
    node.first = first;
    node.last = last;
    node.vertexID = std::numeric_limits<size_t>::max();
    node.splitAxis = d;
    return nodeID;
  }

  std::pair<size_t, size_t> ssplit(size_t vNodeID,
                                   std::vector<size_t>* indices) {
    Node& node = nodes[vNodeID];
    size_t mid = (node.last + node.first) / 2;
    size_t d = node.splitAxis;
    std::nth_element(indices->begin() + node.first, indices->begin() + mid,
                     indices->begin() + node.last,
                     [this, d](size_t a, size_t b) -> bool {
                       return vertices[a].Position[d] < vertices[b].Position[d];
                     });
    d = (d + 1) % 3;
    node.vertexID = indices->at(mid);
    if (mid - node.first > 0) {
      node.left = createNode(d, node.first, mid);
    } else {
      node.left = NAI;
    }
    if (node.last - (mid + 1) > 0) {
      node.right = createNode(d, mid + 1, node.last);
    } else {
      node.right = NAI;
    }
    return std::make_pair(node.left, node.right);
  }

  void split(size_t vNodeID, std::vector<size_t>* indices,
             std::atomic<int>* num_threads) {
    size_t left, right;
    if ((*num_threads -= 1) >= 1) {
      std::tie(left, right) = ssplit(vNodeID, indices);
      if (left != NAI && right != NAI) {
        std::thread other(&KDTree::split, this, left, indices, num_threads);
        split(right, indices, num_threads);
        other.join();
      } else {
        if (left != NAI) split(left, indices, num_threads);
        if (right != NAI) split(right, indices, num_threads);
      }
    } else {
      std::deque<size_t> queue;
      queue.push_back(vNodeID);
      while (!queue.empty()) {
        size_t node_id = queue.front();
        queue.pop_front();
        std::tie(left, right) = ssplit(node_id, indices);
        if (left != NAI) queue.push_back(left);
        if (right != NAI) queue.push_back(right);
      }
    }
    *num_threads += 1;
  }

 public:
  KDTree(std::vector<Vertex> const& vVertices,
         int max_threads = std::thread::hardware_concurrency())
      : vertices(vVertices), numNodes(0) {
    std::size_t numVertices = vVertices.size();
    nodes.resize(numVertices);

    std::vector<size_t> indices(numVertices);
    std::iota(indices.begin(), indices.end(), 0);

    std::atomic<int> numThreads(max_threads);
    split(createNode(0, 0, numVertices), &indices, &numThreads);
  }

  bool find_nn(Vertex point, std::pair<size_t, float>& nn,
               float max_dist = std::numeric_limits<float>::infinity()) const {
    std::vector<std::pair<size_t, float> > nns;
    if (!find_nns(point, 1, nns, max_dist)) return false;

    nn = nns[0];
    return true;
  }

  bool find_nns(Vertex point, std::size_t n,
                std::vector<std::pair<size_t, float> >& nns,
                float max_dist = std::numeric_limits<float>::infinity()) const {
    nns.reserve(n);

    // TODO use square distances

    size_t node_id = 0;
    std::stack<size_t> s;
    bool down = true;

    while (true) {
      Node const& node = nodes[node_id];

      float diff = point.Position[node.splitAxis] -
                   vertices[node.vertexID].Position[node.splitAxis];
      if (down) {
        float dist =
            glm::length(point.Position - vertices[node.vertexID].Position);
        if (dist <= max_dist) {
          if (nns.size() < n) {
            nns.emplace_back(node.vertexID, dist);
          } else {
            std::vector<std::pair<size_t, float> >::iterator it;
            it = std::max_element(nns.begin(), nns.end(), f_nodeCompare);
            *it = std::make_pair(node.vertexID, dist);
          }

          if (nns.size() == n) {
            std::vector<std::pair<size_t, float> >::iterator it;
            it = std::max_element(nns.begin(), nns.end(), f_nodeCompare);
            max_dist = it->second;
          }
        }

        if (node.left != NAI || node.right != NAI) {
          /* Inner node - traverse further down. */
          down = true;

          if (node.left != NAI && node.right != NAI) {
            s.push(node_id);
          }

          float diff = point.Position[node.splitAxis] -
                       vertices[node.vertexID].Position[node.splitAxis];

          size_t next = (diff < 0.0f) ? node.left : node.right;
          size_t other = (diff < 0.0f) ? node.right : node.left;

          node_id = (next != NAI) ? next : other;
        } else {
          /* Leaf - traverse up and search for next node. */
          down = false;
          node_id = NAI;
        }
      } else {
        if (std::abs(diff) < max_dist) {
          down = true;
          node_id = (diff < 0.0f) ? node.right : node.left;
        } else {
          down = false;
          node_id = NAI;
        }
      }

      if (node_id == NAI) {
        if (s.empty()) break;
        node_id = s.top();
        s.pop();
      }
    }

    std::sort(nns.begin(), nns.end(), f_nodeCompare);

    bool success = nns.size() == n;

    return success;
  }
};

static bool f_nodeCompare(std::pair<size_t, float> l, std::pair<size_t, float> r) {
  return l.second < r.second;
}

}  // namespace ACCEL
