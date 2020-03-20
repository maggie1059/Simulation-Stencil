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

    void setMasses();
    void setBeta();
//    Eigen::Matrix3f getStress();
//    void distributeForces();
    void updateForces();
    Eigen::Vector3f addForces(Eigen::Matrix3f dx_du, Eigen::Matrix3f stress, std::shared_ptr<Node> n1, std::vector<std::shared_ptr<Node>> nodes);
};

#endif // ELEMENT_H
