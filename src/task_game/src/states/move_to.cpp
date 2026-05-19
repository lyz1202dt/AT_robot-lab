#include "actions/move_to.hpp"
#include <thread>

using namespace std::chrono_literals;

MoveToAction::MoveToAction()  : BaseAction<Robot>("move_to_action")
{

}

std::string MoveToAction::process(Robot* context, const std::string &last_status)
{
    (void)context;
    (void)last_status;
    //TODO:执行一些动作
    std::this_thread::sleep_for(1s);
    int a;
    push_msg<int>(10);  //用于状态间消息传递
    pop_msg<int>(a);
    return "move_to_action";
}
