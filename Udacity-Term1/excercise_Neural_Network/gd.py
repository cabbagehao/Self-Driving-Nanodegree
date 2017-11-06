def gradient_descent_update(x, gradx, learning_rate):
    """
    Performs a gradient descent update.
    """
    # Return the new value for x
    
    x -= learning_rate * gradx
    
    return x
