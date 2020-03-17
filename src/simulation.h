#ifndef SIMULATION_H
#define SIMULATION_H

#include "graphics/shape.h"
#include <unordered_set>
#include <set>
#include <unordered_map>

//template<typename T>
//struct hash_on_sum
//: private std::hash<typename T>
//{
//  typedef T count_type;
//  typedef std::hash<count_type> base;
//  std::size_t operator()(T const&obj) const
//  {
//    return base::operator()(std::accumulate(obj.begin(),obj.end(),count_type()));
//  }
//};

//typedef std::set<int> inner_type;
//typedef std::unordered_set<inner_type, hash_on_sum<inner_type>> set_of_unique_sets;
using namespace std;
struct hash_tuple {
    template <class T1, class T2, class T3>
    size_t operator()(const tuple<T1, T2, T3>& p) const
    {
        auto hash1 = hash<T1>{}(std::get<0>(p));
        auto hash2 = hash<T2>{}(std::get<1>(p));
        auto hash3 = hash<T3>{}(std::get<2>(p));
        return hash1 ^ hash2 ^ hash3;
    }
};

class Shader;

class Simulation
{
public:
    Simulation();

    void init();

    void update(float seconds);

    void draw(Shader *shader);

    void toggleWire();
    Eigen::Vector3i turnClockwise(Eigen::Vector3i &face, int f, std::vector<Eigen::Vector3f> const &vertices);
private:
    Shape m_shape;

    Shape m_ground;
    void initGround();
//    std::unordered_set<std::set<unsigned>> surface;
//    set_of_unique_sets surface;
    std::unordered_map<std::string, int> m_surface;
    std::unordered_map<std::tuple<int, int, int>, int, hash_tuple> m_surface2;
};

#endif // SIMULATION_H
