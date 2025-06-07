from okmr_msgs.action import Movement
import time


def handle_freeze(goal_handle):
    pass

def test_handle_freeze(goal_handle):
    start_time = time.time()
        
    for i in range(20):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Test freeze command was canceled'
            return result
            
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = time.time() - start_time
        feedback_msg.completion_percentage = (i / 20.0) * 100.0
            
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(0.1)
        
    goal_handle.succeed()
    result = Movement.Result()
    result.completion_time = time.time() - start_time
    result.debug_info = 'Test freeze command completed successfully'
    return result

    
