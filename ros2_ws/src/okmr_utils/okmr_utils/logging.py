def make_green_log(message: str) -> str:
    """
    Wrap a message with ANSI color codes to display it in green.
    
    Args:
        message (str): The message to colorize
        
    Returns:
        str: The message wrapped with green ANSI color codes
    """
    return f"\033[32m{message}\033[0m"