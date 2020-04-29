/*
 *
 * NGraph++ : a simple graph library
 *
 * Mathematical and Computational Sciences Division
 * National Institute of Technology,
 * Gaithersburg, MD USA
 *
 *
 * This software was developed at the National Institute of Standards and
 * Technology (NIST) by employees of the Federal Government in the course
 * of their official duties. Pursuant to title 17 Section 105 of the
 * United States Code, this software is not subject to copyright protection
 * and is in the public domain. NIST assumes no responsibility whatsoever for
 * its use by other parties, and makes no guarantees, expressed or implied,
 * about its quality, reliability, or any other characteristic.
 *
 */

#ifndef NGRAPH_H_
#define NGRAPH_H_

// version 4.2


#include <iostream>
#include <set>
#include <map>
#include <list>
#include <utility>      // for std::pair
#include <iterator>     // for inserter()
#include <vector>       // for exporting edge_list
#include <string>
#include <algorithm>
#include <sstream>      // for I/O << and >> operators
#include <functional>
#include "set_ops.hpp"


// TEMPLATE DIRECTED GRAPH (with in-out adjacency list)
//
//
// T is the vertex type
//
//  An adjacency graph format lists for each vertex, a set of neighbors
//  the it connects to (outlinks) and optionally another set of neighbors
//  which connect to it.  (The second set provides a quick way to find
//  out who is pointing to you.)
//
//     vertex  {out-neighbors}  {in-neighbors}
//

/**
 
 @brief A mathematical graph object: a simple, directed, connected graph,
 where nodes are of arbitrary type (colores, cities, names, etc.)
 Operations for adding and removing edges and vertices, together with
 functions for finding neighbors, subgraphs, and other properties are
 included.
 
 Example:
 
 
 <pre>
 enum color {blue, green, red, yellow, pink, black};
 tGraph<color> A;
 
 A.insert_edge(blue, red);
 A.insert_edge(yellow, blue);
 A.insert_edge(blue, black);
 
 tGraph<color> B(A), S=A.subgraph(blue);
 
 </pre>
 
 
 */
namespace NGraph
{

template <typename T, typename W = T, typename X = T>
class tGraph
{
    
public:
    
    typedef T vertex;
    typedef T value_type;
    typedef W vertex_data;
    typedef std::pair<vertex,vertex> edge;
    typedef X edge_data;
    //typedef struct{ vertex from; vertex to;} edge;
    typedef std::set<vertex> vertex_set;
    typedef std::set<edge> edge_set;
    typedef std::pair<vertex_set, vertex_set> in_out_edge_sets;
    typedef std::map<vertex, in_out_edge_sets>  adj_graph;
    typedef std::map<vertex, vertex_data> vertex_data_list;
    typedef std::map<edge, edge_data> edge_data_list;
    
    typedef typename edge_set::iterator edge_iterator;
    typedef typename edge_set::const_iterator const_edge_iterator;
    
    typedef typename vertex_set::iterator vertex_iterator;
    typedef typename vertex_set::const_iterator const_vertex_iterator;
    
    enum line_type { VERTEX, EDGE, COMMENT, EMPTY};
    
    typedef typename vertex_set::iterator vertex_neighbor_iterator;
    typedef typename vertex_set::const_iterator vertex_neighbor_const_iterator;
    
private:
    
    adj_graph G_;
    unsigned int num_edges_;
    bool undirected_;
    vertex_data_list VDL_;
    edge_data_list EDL_;
    
public:
    
    //
    //
    
    /**
     tGraph::iterator refers to pair<const vertex, in_out_edge_sets> .
     
     <pre>
     tGraph::const_iterator p = G.begin();
     
     tGraph::vertex a = node(p);
     tGraph::vertex_set  &in_edges = in_neighbors(p);
     tGraph::vertex_set  &out_edges = out_neighbors(p);
     
     </pre>
     */
    
    typedef typename adj_graph::iterator iterator;
    typedef typename adj_graph::const_iterator const_iterator;
    
    typedef iterator node_iterator;
    typedef const_iterator const_node_iterator;
    
    typedef typename vertex_data_list::iterator vdl_iterator;
    typedef typename vertex_data_list::const_iterator const_vdl_iterator;
    
    unsigned long num_vertices() const { return G_.size(); }
    unsigned int num_nodes() const { return G_.size(); }
    unsigned int num_edges() const { return num_edges_; }
    
    iterator begin() { return G_.begin(); }
    const_iterator begin() const { return G_.begin(); }
    iterator end() { return G_.end(); }
    const_iterator end() const { return G_.end(); }
    
    void clear()
    {
        G_.clear();
        num_edges_ = 0;
        undirected_ = false;
        VDL_.clear();
        EDL_.clear();
    }
    
    vertex_neighbor_iterator out_neighbors_begin(const vertex &a)
    {
        return out_neighbors(a).begin();
    }
    
    vertex_neighbor_const_iterator out_neighbors_begin(const vertex &a) const
    {
        return out_neighbors(a).begin();
    }
    
    vertex_neighbor_iterator out_neighbors_end(const vertex &a)
    {
        return out_neighbors(a).end();
    }
    
    vertex_neighbor_const_iterator out_neighbors_end(const vertex &a) const
    {
        return out_neighbors(a).end();
    }
    
    
    tGraph(): G_(), num_edges_(0), undirected_(false), VDL_(), EDL_(){}
    tGraph(std::istream &s): G_(), num_edges_(0), undirected_(false)
    {
        s >> *this;
    }
    
    tGraph(const tGraph &B) : G_(B.G_), num_edges_(B.num_edges_),
    undirected_(B.undirected_), VDL_(B.VDL_), EDL_(B.EDL_){}
    tGraph(const edge_set &E)
    {
        num_edges_ = 0;
        undirected_ = false;
        for (typename edge_set::const_iterator p = E.begin();
             p != E.end(); p++) {
//            std::cout << "Edge: [" << p->first << ", " << p->second << "]" << std::endl;
            insert_edge(*p);
        }
        
            
    }
    
    tGraph(const edge_set &E, const vertex_data_list& vdl, const edge_data_list& edl)
        : G_(), VDL_(), EDL_()
    {
        num_edges_ = 0;
        undirected_ = false;
        for (typename edge_set::const_iterator p = E.begin();
             p != E.end(); p++) {
//            std::cout << "Edge: [" << p->first << ", " << p->second << "]" << std::endl;
            insert_edge({p->first, vdl.at(p->first)}, {p->second, vdl.at(p->second)}, edl.at(*p));
        }
        
            
    }
    
    bool is_undirected() const
    {
        return undirected_;
    }
    
    
    bool is_directed() const
    {
        return !undirected_;
    }
    
    void set_undirected()
    {
        undirected_ = true;
    }
    
    iterator find(const vertex &a)
    {
        return G_.find(a);
    }
    
    const_iterator find(const vertex  &a) const
    {
        return G_.find(a);
    }
    
    const vertex_set &in_neighbors(const vertex &a) const
    { return (find(a))->second.first;}
    vertex_set &in_neighbors(const vertex &a)
    { return (G_[a]).first; }
    
    const vertex_set &out_neighbors(const vertex &a)const
    {return find(a)->second.second;}
    vertex_set &out_neighbors(const vertex &a)
    {return G_[a].second; }
    
    
    unsigned int in_degree(const vertex &a) const
    {
        return in_neighbors(a).size();
    }
    
    unsigned int out_degree(const vertex &a) const
    {
        return out_neighbors(a).size();
    }
    
    unsigned int degree(const vertex &a) const
    {
        const_iterator p = find(a);
        if (p == end())
            return 0;
        else
            return degree(p);
    }
    
    
    bool isolated(const vertex &a) const
    {
        const_iterator p = find(a);
        if ( p != end() )
            return  isolated(p) ;
        else
            return false;
    }
    
    void insert_vertex(const vertex &a)
    {
        G_[a];
    }
    
    void insert_vertex(std::pair<const vertex&, const vertex_data&> a)
    {
        G_[a.first];
        VDL_[a.first] = a.second;
    }
    
    void insert_new_vertex_inout_list(const vertex &a, const vertex_set &IN,
                                      const vertex_set &OUT)
    {
        typename adj_graph::iterator p = find(a);
        
        // return if "a" already in graph
        if (p != G_.end())
        {
            // remove the old number of OUT vertices
            num_edges_ -= p->second.second.size();
        }
        
        G_[a] = make_pair(IN, OUT);
        num_edges_ += OUT.size();
    }
    
    
    
    void insert_edge_noloop(iterator pa, iterator pb)
    {
        if (pa==pb) return;
        insert_edge(pa, pb);
    }
    
    void insert_edge(iterator pa, iterator pb)
    {
        vertex a = node(pa);
        vertex b = node(pb);
        
        if (is_undirected())
        {
            vertex smallest = ( a < b ? a : b);
            //vertex biggest =  ( a < b ? b : a);
            
            if (smallest == b)
            {
                std::swap(a,b);
                std::swap(pa, pb);
            }
        }
        unsigned long old_size = out_neighbors(pa).size();
        
        out_neighbors(pa).insert(b);
        in_neighbors(pb).insert(a);
        
        unsigned long new_size = out_neighbors(pa).size();
        if (new_size > old_size)
        {
            num_edges_++;
        }
        
    }
    
    void insert_edge(iterator pa, iterator pb, const edge_data& ed)
    {
        vertex a = node(pa);
        vertex b = node(pb);
        
        if (is_undirected())
        {
            vertex smallest = ( a < b ? a : b);
            //vertex biggest =  ( a < b ? b : a);
            
            if (smallest == b)
            {
                std::swap(a,b);
                std::swap(pa, pb);
            }
        }
        unsigned long old_size = out_neighbors(pa).size();
        
        out_neighbors(pa).insert(b);
        in_neighbors(pb).insert(a);
        
        // insert edge data
        EDL_[{a, b}] = ed;
        
        
        unsigned long new_size = out_neighbors(pa).size();
        if (new_size > old_size)
        {
            num_edges_++;
        }
        
    }
    
    void insert_edge_noloop(const vertex &a, const vertex &b)
    {
        if (a==b) return;
        insert_edge(a, b);
    }
    
    
    void insert_edge(const vertex &a, const vertex& b)
    {
//        std::cout << "hej" << std::endl;
        iterator pa = find(a);
        if (pa == G_.end())
        {
            insert_vertex(a);
            pa = find(a);
        }
        
        iterator pb = find(b);
        if (pb == G_.end())
        {
            insert_vertex(b);
            pb = find(b);
        }
        
        insert_edge( pa, pb );
    }
    
    void insert_edge(std::pair<const vertex&, const vertex_data&> a, std::pair<const vertex&, const vertex_data&> b) {
        iterator pa = find(a.first);
        if (pa == G_.end())
        {
            insert_vertex(a);
            pa = find(a.first);
        }
        
        iterator pb = find(b.first);
        if (pb == G_.end())
        {
            insert_vertex(b);
            pb = find(b.first);
        }
        
        insert_edge( pa, pb );
    }
    
    void insert_edge(std::pair<const vertex&, const vertex_data&> a, std::pair<const vertex&, const vertex_data&> b, const edge_data& ed) {
        iterator pa = find(a.first);
        if (pa == G_.end())
        {
            insert_vertex(a);
            pa = find(a.first);
        }
        
        iterator pb = find(b.first);
        if (pb == G_.end())
        {
            insert_vertex(b);
            pb = find(b.first);
        }
        
        insert_edge( pa, pb, ed);
    }
    
    void insert_undirected_edge(const vertex &a, const vertex &b)
    {
        (a < b ) ?  insert_edge(a,b) : insert_edge(b,a);
    }
    
    
    void insert_edge(const edge &E)
    {
        insert_edge(E.first, E.second);
    }
    
    void insert_undirected_edge(const edge &E)
    {
        insert_undirected_edge(E.first, E.second);
    }
    
    
    
    bool remove_edge(iterator pa, iterator pb)
    {
        if (pa == end() || pb == end())
            return false;
        
        vertex a = node(pa);
        vertex b = node(pb);
        
        if ( is_undirected())
        {
            if (a > b)
            {
                std::swap(b,a);
                std::swap(pb, pa);
            }
            
        }
        unsigned long old_size = out_neighbors(pa).size();
        out_neighbors(pa).erase(b);
        in_neighbors(pb).erase(a);
        
        // remove edge data
        EDL_.erase({a, b});
        
        if (out_neighbors(pa).size() < old_size)
            num_edges_ --;
        
        return true;
    }
    
    void remove_edge(const vertex &a, const vertex& b)
    {
        iterator pa = find(a);
        if (pa == end())
            return;
        iterator pb = find(b);
        if (pb == end())
            return;
        remove_edge( pa, pb );
    }
    
    void remove_edge(const edge &E)
    {
        remove_edge(E.first, E.second);
    }
    
    void remove_undirected_edge(const vertex &a, const vertex& b)
    {
        (a < b) ? remove_edge(a,b) : remove_edge(b,a);
    }
    
    void remove_undirected_edge(const edge &e)
    {
        remove_undirected_edge(e.first, e.second);
    }
    
    void remove_vertex(iterator pa)
    {
        vertex_set  out_edges = out_neighbors(pa);
        vertex_set  in_edges =  in_neighbors(pa);
        
        //vertex_set & out_edges = out_neighbors(pa);
        //vertex_set & in_edges =  in_neighbors(pa);
        
        // remove out-going edges
        for (typename vertex_set::iterator p = out_edges.begin();
             p!=out_edges.end(); p++)
        {
            remove_edge(pa, find(*p));
        }
        
        
        // remove in-coming edges
        for (typename vertex_set::iterator p = in_edges.begin();
             p!=in_edges.end(); p++)
        {
            remove_edge(find(*p), pa);
        }
        
        // remove from adj_graph and vertex data list
        G_.erase(node(pa));
        VDL_.erase(node(pa));
    }
    
    
    void remove_vertex_set(const vertex_set &V)
    {
        for (typename vertex_set::const_iterator p=V.begin(); p!=V.end(); p++)
            remove_vertex(*p);
    }
    
    
    void remove_vertex(const vertex &a)
    {
        iterator pa = find(a);
        if (pa != G_.end())
            remove_vertex( pa);
        
    }
    
    /**
     Is vertex 'a' included in graph?
     
     @return true, if vertex a is present; false, otherwise.
     */
    bool includes_vertex(const vertex &a) const
    {
        return  (find(a) != G_.end());
    }
    
    
    /**
     Is edge (a,b) included in graph?
     
     @return true, if edge is present; false, otherwise.
     
     */
    bool includes_edge(const vertex &a, const vertex &b) const
    {
        return (includes_vertex(a)) ?
        includes_elm(out_neighbors(a),b): false;
        
        //const vertex_set &out = out_neighbors(a);
        // return ( out.find(b) != out.end());
        
    }
    
    
    bool includes_edge(const edge& e) const
    {
        return includes_edge(e.first, e.second);
    }
    
    
    /*
     vertex data functions
     */
    const vertex_data& get_vertex_data(const vertex& v) const {
        return VDL_.at(v);
    }
    
    void set_vertex_data(const vertex& v, const vertex_data& vd) {
        VDL_[v] = vd;
    }
    
    void set_vertex_data(std::pair<const vertex&, const vertex_data&> v_pair) {
        VDL_[v_pair.first] = v_pair.second;
    }
    
    /*
     edge data functions
     */
    const edge_data& get_edge_data(const edge& e) const {
        //std::cout << EDL_.size() << std::endl;
        return EDL_.at(e);
        //return 0;
    }
    
    const edge_data& get_edge_data(const vertex& a, const vertex& b) const {
        return EDL_.at({a, b});
    }
    
    void set_edge_data(const edge& e, const edge_data& ed) {
        EDL_[e] = ed;
    }
    void set_edge_data(std::pair<const edge&, const edge_data&> e_pair) {
        EDL_[e_pair.first] = e_pair.second;
    }
    
    /*
     breadth first traversal and search
     */
    
    void BFT(const vertex& start_vertex) {
        // visited
        std::vector<bool> visited(num_vertices(), false);
        //std::cout << visited[1] << std::endl;
        
        // queue used to traverse graph
        std::list<vertex> queue;
        
        visited[start_vertex] = true;
        queue.push_back(start_vertex);
        
        while(!queue.empty()) {
            vertex cur = queue.front();
            std::cout << cur << std::endl;
            queue.pop_front();
            
            for (vertex_iterator p = out_neighbors_begin(cur); p != out_neighbors_end(cur); ++p) {
                //std::cout << "gaa" << std::endl;
                if (!visited[*p]) {
                    visited[*p] = true;
                    queue.push_back(*p);
                }
            }
        }
    }
    
    // define a function type which returns a bool and takes
    // vertex_data
    // using condition_function = bool (*) (const vertex_data& vd);
    std::function<bool(const vertex_data&)> search_condition_vertex;
    
    void BFS_vertex_util(const vertex& start_vertex, std::vector<bool>& visited, vertex_set& search_set) {
        
        // queue used to traverse graph
        std::list<vertex> queue;
        
        visited[start_vertex] = true;
        queue.push_back(start_vertex);
        
        // Go through queue until empty
        while(!queue.empty()) {
            vertex cur = queue.front();
            //std::cout << cur << std::endl;
            queue.pop_front();
            
            //std::cout  << "haha: " << search_condition_vertex(VDL_[cur]) << std::endl;
            
            if (search_condition_vertex(VDL_[cur])) {
                search_set.insert(cur);
                //std::cout  << "haha: " << search_condition_vertex(VDL_[cur]) << std::endl;
            }
            
            for (vertex_iterator p = out_neighbors_begin(cur); p != out_neighbors_end(cur); ++p) {
                //std::cout << "gaa" << std::endl;
                if (!visited[*p]) {
                    visited[*p] = true;
                    queue.push_back(*p);
                }
            }
        }
        
    }
    
    tGraph BFS_vertex() {
        // visited
        std::vector<bool> visited(num_vertices(), false);
        //std::cout << visited[1] << std::endl;
        
        // set used to save vertices which fulfill the search condition
        vertex_set search_set;
        
        for (unsigned long i = 0; i < num_vertices(); i++) {
            if (visited[i] == false) {
                BFS_vertex_util(i, visited, search_set);
            }
        }
        
        return subgraph(search_set);
    }
    
    std::function<bool(const edge_data&)> search_condition_edge;
     
     void BFS_edge_util(const vertex& start_vertex, std::vector<bool>& visited, edge_set& search_set) {
         
         // queue used to traverse graph
         std::list<vertex> queue;
         
         visited[start_vertex] = true;
         queue.push_back(start_vertex);
         
         // Go through queue until empty
         while(!queue.empty()) {
             vertex cur = queue.front();
//             std::cout << cur << std::endl;
             queue.pop_front();

             // go through all outgoing edges
             for (vertex_iterator p = out_neighbors_begin(cur); p != out_neighbors_end(cur); ++p) {

                 edge e{cur, *p};
//                 std::cout << "Edge: [" << e.first << ", " << e.second << "]" << std::endl;
                 // if edge fulfills condition, add to set
                 if (search_condition_edge(EDL_.at(e))) {
                     search_set.insert(e);
                 }
                 
                 // if reachable vertex has not been visited
                 if (!visited[*p]) {
                     visited[*p] = true;
                     queue.push_back(*p);
                 }
             }
         }
         
     }
    
    tGraph BFS_edge() {
        // visited
        std::vector<bool> visited(num_vertices(), false);
        //std::cout << visited[1] << std::endl;
        
        // set used to save edges which fulfill the search condition
        edge_set search_set{};
        
        for (unsigned long i = 0; i < num_vertices(); i++) {
            if (visited[i] == false) {
                BFS_edge_util(i, visited, search_set);
            }
        }
//        std::cout << search_set.size() << std::endl;
//
        return tGraph(search_set, VDL_, EDL_);
    }
    
    
    /*
     return graph as list of edges in vertex format
     */
    std::vector<vertex> get_graph_as_vertex_list() {
        std::vector<vertex> output;
        
        for (const_iterator p=G_.begin();
             p != G_.end(); p++)
        {
            const vertex_set   &out =  out_neighbors(p);
            
            for (typename vertex_set::const_iterator q=out.begin();
                 q!=out.end(); q++) {
                //std::cerr << p->first << "  -->  " << *q << "\n";
                output.push_back(p->first);
                output.push_back(*q);
            }
                
        }
        return output;
    }
    
    
    // convert to a simple edge list for exporting
    //
    /**
     Create a new representation of graph as a list
     of vertex pairs (a,b).
     
     @return std::vector<edge> a vector (list) of vertex pairs
     */
    std::vector<edge> edge_list() const;
    
    
    /*
     graph unions
     */
    
    tGraph & plus_eq(const tGraph &B)
    {
        
        for (const_iterator p=B.begin(); p != B.end(); p++)
        {
            const vertex &this_node = node(p);
            insert_vertex(this_node);
            const vertex_set &out = out_neighbors(p);
            for (typename vertex_set::const_iterator q= out.begin(); q != out.end(); q++)
            {
                insert_edge(this_node, *q);
            }
        }
        return *this;
    }
    
    tGraph intersect(const tGraph &B) const
    {
        tGraph G;
        for (const_iterator p=begin(); p != end(); p++)
        {
            const vertex &this_node = node(p);
            if (B.includes_vertex(this_node))
                G.insert_vertex({this_node, VDL_.at(this_node)});
            {
                const vertex_set &out = out_neighbors(p);
                for (typename vertex_set::const_iterator q= out.begin();
                     q != out.end(); q++)
                {
                    if (B.includes_edge(this_node, *q))
                        G.insert_edge({this_node, VDL_.at(this_node)}, {*q, VDL_.at(*q)}, EDL_.at({this_node, *q}));
                }
            }
        }
        return G;
    }
    
    tGraph operator*(const tGraph &B) const
    {
        return intersect(B);
    }
    
    
    tGraph minus(const tGraph &B) const
    {
        tGraph G;
        for (const_iterator p=begin(); p != end(); p++)
        {
            const vertex &this_vertex = node(p);
            if (isolated(p))
            {
                if (! B.isolated(this_vertex))
                    G.insert_vertex(this_vertex);\
            }
            else
            {
                const vertex_set &out = out_neighbors(p);
                for (typename vertex_set::const_iterator q= out.begin();
                     q != out.end(); q++)
                {
                    if (!B.includes_edge(this_vertex, *q))
                        G.insert_edge(this_vertex, *q);
                }
            }
        }
        return G;
    }
    
    tGraph operator-(const tGraph &B) const
    {
        return minus(B);
    }
    
    
    tGraph plus(const tGraph &B) const
    {
        tGraph U(*this);
        
        U.plus_eq(B);
        return U;
    }
    
    tGraph operator+(const tGraph &B) const
    {
        return plus(B);
    }
    
    tGraph & operator+=(const tGraph &B)
    {
        return plus_eq(B);
    }
    
    
    
#if 0
    /*
     union of two graphs
     */
    tGraph Union(const tGraph &B) const
    {
        tGraph U(B);
        typedef std::vector<edge> VE;
        
        VE b = B.edge_list();
        for (typename VE::cosnt_iterator e = b.begin(); e != b.end(); e++)
        {
            U.insert_edge(*e);
        }
        return U;
    }
    
    /*
     intersection of two graphs
     */
    tGraph intersect(const tGraph &B) const
    {
        tGraph I;
        typedef std::vector<tGraph::edge> VE;
        
        VE b = B.edge_list();
        for (typename VE::const_iterator e = b.begin(); e != b.end(); e++)
        {
            if (includes_edge(*e))
            {
                I.insert_edge(*e);
            }
        }
        
        for (typename tGraph::const_iterator p = B.begin(); p!=B.end();p++)
        {
            if (includes_vertex( node(p) ))
                I.insert_vertex( node(p) );
        }
        return I;
    }
    
#endif
    
    
    
    
    /**
     @param A vertex set of nodes (subset of G)
     @return a new subgraph containing all nodes of A
     */
    tGraph subgraph(const vertex_set &A) const
    {
        tGraph G;
        
        for (typename vertex_set::const_iterator p = A.begin(); p!=A.end(); p++)
        {
            const_iterator t = find(*p);
            if (t != end())
            {
                vertex_set new_in =  (A * in_neighbors(t));
                vertex_set new_out = (A * out_neighbors(t));
                
                G.insert_new_vertex_inout_list(*p, new_in, new_out);
                
                // get and save edge data
                for (const_vertex_iterator i = new_in.begin(); i != new_in.end(); ++i) {
                    edge e = {node(i), node(t)};
                    G.set_edge_data(e, EDL_.at(e));
                }
                
                for (const_vertex_iterator i = new_out.begin(); i != new_out.end(); ++i) {
                    edge e = {node(t), node(i)};
                    G.set_edge_data(e, EDL_.at(e));
                }
                
                // set vertex data in new graph
                G.set_vertex_data(node(t), VDL_.at(node(t)));
            }
        }
        return G;
    }
    
    
    unsigned int subgraph_size(const vertex_set &A) const
    {
        unsigned int num_edges = 0;
        for (typename vertex_set::const_iterator p = A.begin(); p!=A.end(); p++)
        {
            const_iterator pG = find(*p);
            if (pG != this->end())
            {
                num_edges +=  intersection_size(A, out_neighbors(pG) );
            }
        }
        return num_edges;
    }
    
    
    // we don't need to divide by two since  we are only
    // counting out-edges in subgraph_size()
    //
    double subgraph_sparsity(const vertex_set &A) const
    {
        double N  = A.size();
        
        double sparsity =  (A.size() ==1 ? 0.0 : subgraph_size(A)/(N * (N-1)));
        if (is_undirected())
        {
            sparsity *= 2.0;
        }
        return sparsity;
    }
    
    void print() const;
    
    
    /* tGraph iterator methods */
    
    /**
     @param p tGraph::const_iterator
     */
    static const vertex &node(const_iterator p)
    {
        return p->first;
    }
    
    static const vertex &node(iterator p)
    {
        return p->first;
    }
    
    static const vertex &node( const_vertex_iterator p)
    {
        return *p;
    }
    
    
    static const vertex_set & in_neighbors(const_iterator p)
    { return (p->second).first; }
    
    static    vertex_set & in_neighbors(iterator p)
    { return (p->second).first; }
    
    static const_vertex_iterator in_begin(const_iterator p)
    {
        return in_neighbors(p).begin();
    }
    
    
    static const_vertex_iterator in_end(const_iterator p)
    {
        return in_neighbors(p).end();
    }
    
    
    static const vertex_set& out_neighbors(const_iterator p)
    { return (p->second).second; }
    
    static const_vertex_iterator out_begin(const_iterator p)
    {
        return out_neighbors(p).begin();
    }
    
    
    static const_vertex_iterator out_end(const_iterator p)
    {
        return out_neighbors(p).end();
    }
    
    
    static     vertex_set& out_neighbors(iterator p)
    { return (p->second).second; }
    
    static vertex_iterator out_begin(iterator p)
    {
        return out_neighbors(p).begin();
    }
    
    
    static  unsigned int num_edges(const_iterator p)
    {
        return out_neighbors(p).size();
    }
    
    inline static  unsigned int num_edges(iterator p)
    {
        return out_neighbors(p).size();
    }
    
    /**
     @param p tGraph::const_iterator
     @return number of edges going out (out-degree) at node pointed to by p.
     */
    inline static  unsigned int out_degree(const_iterator p)
    {
        return (p->second).second.size();
    }
    
    inline static  unsigned int out_degree(iterator p)
    {
        return (p->second).second.size();
    }
    
    /**
     @param p tGraph::const_iterator
     @return number of edges going out (out-degree) at node pointed to by p.
     */
    inline static  unsigned int in_degree(const_iterator p)
    {
        return (p->second).first.size();
    }
    
    inline static  unsigned int in_degree(iterator p)
    {
        return (p->second).first.size();
    }
    
    inline static  unsigned int degree(const_iterator p)
    {
        return in_neighbors(p).size() + out_neighbors(p).size();
    }
    
    inline static  unsigned int degree(iterator p)
    {
        return in_neighbors(p).size + out_neighbors(p).size();
    }
    
    
    inline static bool isolated(const_iterator p)
    {
        return (in_degree(p) == 0 && out_degree(p) == 0 );
    }
    
    static bool isolated(iterator p)
    {
        return (in_degree(p) == 0 && out_degree(p) == 0 );
    }
    
    
    
    /**
     abosrb(a,b):   'a' absorbs 'b', b gets removed from graph
     */
    void absorb(iterator pa, iterator pb)
    {
        
        //std::cerr << "Graph::absorb(" << node(pa) << ", " << node(pb) << ")\n";
        
        if (pa == pb)
            return;
        
        
        // first remove edge (a,b) to avoid self-loops
        remove_edge(pa, pb);
        
        // chnage edges (b,i) to a(i,j)
        //
        {
            vertex_set b_out = out_neighbors(pb);
            for (typename vertex_set::iterator p = b_out.begin();
                 p!=b_out.end(); p++)
            {
                iterator pi = find(*p);
                remove_edge(pb, pi);
                //std::cerr<<"\t remove_edge("<<node(pb)<< ", " << node(pi) <<")\n";
                insert_edge(pa, pi);
                //std::cerr<<"\t insert_edge("<<node(pa)<< ", " << node(pi) <<")\n";
            }
        }
        
        // change edges (i,b) to (i,a)
        {
            vertex_set b_in = in_neighbors(pb);
            for (typename vertex_set::iterator p = b_in.begin();
                 p!=b_in.end(); p++)
            {
                iterator pi = find(*p);
                remove_edge(pi, pb);
                //std::cerr<<"\t remove_edge("<<node(pi)<< ", " << node(pb) <<")\n";
                insert_edge(pi, pa);
                //std::cerr<<"\t insert_edge("<<node(pi)<< ", " << node(pa) <<")\n";
            }
        }
        
        
        //std::cout<<"\t about to remove vertex: "<< node(pb) << "\n";
        
        remove_vertex(pb);
        //G_.erase( node(pb) );
        
        //std::cout<<"\t removed_vertex.\n";
        
        
    }
    
    /**
     c smart_abosrb(a,b):   'a' absorbs 'b', or b aborbs a, depending on
     whichever causes the least amount of graph updates
     */
    iterator smart_absorb(iterator pa, iterator pb)
    {
        if (degree(pa) >= degree(pb))
        {
            absorb(pa, pb);
            return pb;
        }
        else
        {
            absorb(pb, pa);
            return pb;
        }
    }
    
    
    vertex smart_absorb(vertex a, vertex b)
    {
        iterator pa = find(a);
        if (pa == end())
        {
            return b;
        }
        
        iterator pb = find(b);
        if (pb == end())
        {
            return a;
        }
        
        iterator pc = smart_absorb(pa, pb);
        return node(pc);
    }
    
    
    void absorb(vertex a, vertex b)
    {
        if (a == b)
            return ;
        
        iterator pa = find(a);
        if (pa == end())
            return;
        
        iterator pb = find(b);
        if (pb == end())
            return;
        
        absorb( pa, pb );
    }
    
    
    static std::istream &read_line(std::istream &s, T &v1, T &v2,
                                   std::string &line , line_type &t )
    {
        
        // next non-empty text line, if possible
        
        while (getline(s, line) && line.size() < 1);
        
        // if at end-of-file, return empty line
        if (s.eof())
        {
            t = EMPTY;
            return s;
        }
        
        // otherwise, check for a comment
        if  (line[0] == '%' || line[0] == '#')
        {
            t = COMMENT;
            return s;
        }
        
        
        // if we are here, then we have a valid graph input line:
        // either a single vertex or an edge
        //
        std::istringstream L(line);
        L >> v1;
        if (L.eof())
        {
            t = VERTEX;
        }
        else
        {
            L >> v2;
            t = EDGE;
        }
        
        return s;
    }
    
};
// end tGraph<T>

// global functions
//


typedef tGraph<unsigned int> Graph;
typedef tGraph<int> iGraph;
typedef tGraph<std::string> sGraph;


template <class T, class W, class X>
std::vector<typename tGraph<T, W, X>::edge> tGraph<T, W, X>::edge_list() const
{
    //std::vector<tGraph::edge> E(num_edges());
    std::vector<typename tGraph<T>::edge> E;
    
    for (typename tGraph::const_iterator p = begin(); p!=end(); p++)
    {
        const vertex &a = tGraph::node(p);
        const vertex_set &out = tGraph::out_neighbors(p);
        for (typename vertex_set::const_iterator t = out.begin();
             t != out.end(); t++)
        {
            E.push_back( edge(a, *t));
        }
    }
    return E;
}




template <typename T>
std::istream & operator>>(std::istream &s, tGraph<T> &G)
{
    std::string line;
    T v1, v2;
    typename tGraph<T>::line_type t;
    
    while (tGraph<T>::read_line(s, v1, v2, line, t))
    {
        if (t == tGraph<T>::VERTEX)
        {
            G.insert_vertex(v1);
        }
        else if (t == tGraph<T>::EDGE)
        {
            G.insert_edge(v1, v2);
        }
    }
    return s;
}

template <typename T>
std::ostream & operator<<(std::ostream &s, const tGraph<T> &G)
{
    for (typename tGraph<T>::const_node_iterator p=G.begin(); p != G.end(); p++)
    {
        const typename tGraph<T>::vertex_set &out = tGraph<T>::out_neighbors(p);
        typename tGraph<T>::vertex v = p->first;
        if (out.size() == 0 && tGraph<T>::in_neighbors(p).size() == 0)
        {
            // v is an isolated node
            s << v << "\n";
        }
        else
        {
            for ( typename tGraph<T>::vertex_set::const_iterator q=out.begin();
                 q!=out.end(); q++)
                s << v << " " << *q << "\n";
        }
    }
    return s;
}


template <typename T, typename W, typename X>
void tGraph<T, W, X>::print() const
{
    
    std::cerr << "# vertices: " <<  num_vertices()  << "\n";
    std::cerr << "# edges:    " <<  num_edges()  << "\n";
    
    for (const_iterator p=G_.begin();
         p != G_.end(); p++)
    {
        const vertex_set   &out =  out_neighbors(p);
        
        for (typename vertex_set::const_iterator q=out.begin();
             q!=out.end(); q++)
            std::cerr << p->first << "  -->  " << *q << "\n"
            << "Edge data: " <<  EDL_.at({p->first, *q}) << "\n";
    }
    std::cerr << std::endl;
    
}

}
// namespace NGraph



#endif
// NGRAPH_H_
