#include <behaviortree_cpp/bt_factory.h>
#include "Move1m.h"
#include "TurnCW.h"
#include <iostream>

int main() {

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<Move1m>("Move1m");
    factory.registerNodeType<TurnCW>("TurnCW");

    auto tree = factory.createTreeFromFile("circle.xml");
    BT::NodeStatus status = tree.tickWhileRunning();

    std::cout << status;

    return 0;
}
