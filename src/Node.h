#ifndef NODE_H
#define NODE_H
#include "simulation.h"

class Node
{
public:
    Node(Eigen::Vector3f pos);
    virtual ~Node();
    Eigen::Vector3f m_position;
    Eigen::Vector3f m_velocity;
    Eigen::Vector3f m_force;
    float m_mass;
    void setMass(float mass);
};

#endif // NODE_H
