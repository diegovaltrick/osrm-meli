#ifndef MANY_TO_MANY_ROUTING_HPP
#define MANY_TO_MANY_ROUTING_HPP

#include "engine/algorithm.hpp"
#include "engine/datafacade.hpp"
#include "engine/search_engine_data.hpp"

#include "util/typedefs.hpp"

#include <vector>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{
namespace
{
struct NodeBucket
{
    NodeID middle_node;
    NodeID parent_node;
    unsigned column_index : 31; // a column in the weight/duration matrix
    unsigned from_clique_arc : 1;
    EdgeWeight weight;
    EdgeDuration duration;
    EdgeDistance distance;
    /**
     * @brief Adicionado campo para acumular a quantidade de cruzamentos para o nó
     * 
     */
    int crosses;

    /**
     * @brief Todos os construtores foram alterados para inicializar 
     * o campo crosses com valor 0
     * 
     * @param middle_node 
     * @param parent_node 
     * @param from_clique_arc 
     * @param column_index 
     * @param weight 
     * @param duration 
     * @param distance 
     */
    NodeBucket(NodeID middle_node,
               NodeID parent_node,
               bool from_clique_arc,
               unsigned column_index,
               EdgeWeight weight,
               EdgeDuration duration,
               EdgeDistance distance)
        : middle_node(middle_node), parent_node(parent_node), column_index(column_index),
          from_clique_arc(from_clique_arc), weight(weight), duration(duration), distance(distance),
          crosses(0)
    {
    }

    NodeBucket(NodeID middle_node,
               NodeID parent_node,
               unsigned column_index,
               EdgeWeight weight,
               EdgeDuration duration,
               EdgeDistance distance)
        : middle_node(middle_node), parent_node(parent_node), column_index(column_index),
          from_clique_arc(false), weight(weight), duration(duration), distance(distance), crosses(0)
    {
    }

    NodeBucket(NodeID middle_node,
               NodeID parent_node,
               bool from_clique_arc,
               unsigned column_index,
               EdgeWeight weight,
               EdgeDuration duration,
               EdgeDistance distance,
               int crosses)
        : middle_node(middle_node), parent_node(parent_node), column_index(column_index),
          from_clique_arc(from_clique_arc), weight(weight), duration(duration), distance(distance),
          crosses(crosses)
    {
    }

    NodeBucket(NodeID middle_node,
               NodeID parent_node,
               unsigned column_index,
               EdgeWeight weight,
               EdgeDuration duration,
               EdgeDistance distance,
               int crosses)
        : middle_node(middle_node), parent_node(parent_node), column_index(column_index),
          from_clique_arc(false), weight(weight), duration(duration), distance(distance),
          crosses(crosses)
    {
    }

    // partial order comparison
    bool operator<(const NodeBucket &rhs) const
    {
        return std::tie(middle_node, column_index) < std::tie(rhs.middle_node, rhs.column_index);
    }

    // functor for equal_range
    struct Compare
    {
        bool operator()(const NodeBucket &lhs, const NodeID &rhs) const
        {
            return lhs.middle_node < rhs;
        }

        bool operator()(const NodeID &lhs, const NodeBucket &rhs) const
        {
            return lhs < rhs.middle_node;
        }
    };

    // functor for equal_range
    struct ColumnCompare
    {
        unsigned column_idx;

        ColumnCompare(unsigned column_idx) : column_idx(column_idx){};

        bool operator()(const NodeBucket &lhs, const NodeID &rhs) const // lowerbound
        {
            return std::tie(lhs.middle_node, lhs.column_index) < std::tie(rhs, column_idx);
        }

        bool operator()(const NodeID &lhs, const NodeBucket &rhs) const // upperbound
        {
            return std::tie(lhs, column_idx) < std::tie(rhs.middle_node, rhs.column_index);
        }
    };
};
} // namespace

/**
 * @brief Alterado o tipo de retorno do método que sera sobrescrito para permitir retornar a lista de cruzamentos
 * 
 * @tparam Algorithm 
 * @param engine_working_data 
 * @param facade 
 * @param phantom_nodes 
 * @param source_indices 
 * @param target_indices 
 * @param calculate_distance 
 * @return std::pair<std::pair<std::vector<EdgeDuration>, std::vector<EdgeDistance>>, std::vector<int>> 
 */
template <typename Algorithm>
std::pair<std::pair<std::vector<EdgeDuration>, std::vector<EdgeDistance>>, std::vector<int>>
manyToManySearch(SearchEngineData<Algorithm> &engine_working_data,
                 const DataFacade<Algorithm> &facade,
                 const std::vector<PhantomNode> &phantom_nodes,
                 const std::vector<std::size_t> &source_indices,
                 const std::vector<std::size_t> &target_indices,
                 const bool calculate_distance);

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm

#endif
