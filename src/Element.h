#ifndef ELEMENT_H
#define ELEMENT_H

#include <memory>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "Node.h"

class Node;

class Element
{
public:
    Element(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, std::shared_ptr<Node> n3, std::shared_ptr<Node> n4);
    virtual ~Element();

    std::shared_ptr<Node> m_n1;
    std::shared_ptr<Node> m_n2;
    std::shared_ptr<Node> m_n3;
    std::shared_ptr<Node> m_n4;
    Eigen::Matrix3f m_beta;
    Eigen::Vector3f m_f1normal;
    Eigen::Vector3f m_f2normal;
    Eigen::Vector3f m_f3normal;
    Eigen::Vector3f m_f4normal;

    float f1_area;
    float f2_area;
    float f3_area;
    float f4_area;

    void setMasses();
    void setBeta();
    Eigen::Vector3f setNormal(std::shared_ptr<Node> n1, std::vector<std::shared_ptr<Node>> nodes);
    float setArea(std::vector<std::shared_ptr<Node>> nodes);
    void updateForces();
};

#endif // ELEMENT_H
