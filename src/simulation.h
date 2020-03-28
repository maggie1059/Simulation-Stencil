#ifndef SIMULATION_H
#define SIMULATION_H

#include "graphics/shape.h"
#include <unordered_set>
#include <set>
#include <unordered_map>
#include <memory>
#include "Node.h"
#include "Element.h"

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

struct sphereInfo {
    bool intersect;
    Eigen::Vector3f normal;
    float depth;
};

class Shader;
class Node;
class Element;

class Simulation
{
public:
    Simulation();

    void init();
    void update(float seconds);
    void updateRK4(float seconds);
    void draw(Shader *shader);
    void toggleWire();
    Eigen::Vector3i turnClockwise(Eigen::Vector3i &face, int f, std::vector<Eigen::Vector3f> const &vertices);
    void updateForces();

    Eigen::Vector3f reflect(Eigen::Vector3f in, Eigen::Vector3f n);
    sphereInfo checkSphereCollision(Eigen::Vector3f position);

private:
    Shape m_shape;
    Shape m_sphere;
    Shape m_ground;
    float drag_surface;
    void initGround();
    void initSphere();
    Eigen::Vector3f getAvgVelocity();
    std::vector<shared_ptr<Node>> m_nodes;
    std::vector<shared_ptr<Element>> m_elements;
//    std::unordered_set<std::set<unsigned>> surface;
//    set_of_unique_sets surface;
//    std::unordered_map<std::string, int> m_surface;
    std::unordered_map<std::tuple<int, int, int>, int, hash_tuple> m_surface2;
    std::unordered_map<std::tuple<int, int, int>, int, hash_tuple> fourths;
};

#endif // SIMULATION_H
