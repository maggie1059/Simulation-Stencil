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

class Shader;

class Simulation
{
public:
    Simulation();

    void init();

    void update(float seconds);

    void draw(Shader *shader);

    void toggleWire();
    void turnClockwise(Eigen::Vector3i &face);
private:
    Shape m_shape;

    Shape m_ground;
    void initGround();
//    std::unordered_set<std::set<unsigned>> surface;
//    set_of_unique_sets surface;
    std::unordered_map<std::string, int> m_surface;
    std::unordered_map<std::string, int> m_fourths;
};

#endif // SIMULATION_H
