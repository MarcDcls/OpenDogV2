class JoinNotRecognizedAsHip(Exception):
    """
    Raised when the input is not a hip id
    """
    pass

class JoinNotRecognizedAsKnee(Exception):
    """
    Raised when the input is not a knee id
    """
    pass

class JoinNotRecognizedAsAnkle(Exception):
    """
    Raised when the input is not an ankle id
    """
    pass

class OutOfRange(Exception):
    """
    Raised when the input position is not in range
    """
    pass

class InputNotRecognizedAsLeg(Exception): 
    """
    Raised when the input is not a leg
    """
    pass

class InputNotRecognizedAsJoint(Exception): 
    """
    Raised when the input is not a joint
    """
    pass