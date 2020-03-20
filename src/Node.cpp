#include "Node.h"
#include "simulation.h"

using namespace Eigen;

Node::Node(Vector3f pos)
    : m_position(pos), m_velocity(Vector3f(0,0,0)), m_force(Vector3f(0,0,0)), m_mass(0.f)
{

}

Node::~Node(){

}

void Node::setMass(float mass){
    m_mass += mass;
}
