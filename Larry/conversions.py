import math

conversionFactor = 2048 / 360 # 1 revolution of the falcon 500 is 2048 units

def convertDegreesToTalonFXUnits(num: float) -> float:
    num *= conversionFactor
    return num

def talonGetRevolutions(pos) -> int:
    return int(pos/2048)

def convertTalonFXUnitsToDegrees(num: float) -> float:
    num *= (360 / 2048)
    #num = (num % 360) if sign(num) == 1 or sign(num) == 0 else -(math.fabs(num) % 360) 
    return num

def getRevolutions(num: float) -> int:
    num =  (num - num % 360) if sign(num) == 1 or sign(num) == 0 else (num + math.fabs(num) % 360)
    return num/360

def roundToNearestRev(num):
    val = num % 360
    if val >= 180:
        val = num - (val) + 360
    else:
        val = num - (val)
    return val

def giveRevCompensation(currentAngle, direction):
    """
    currentAngle is the true angle, all the revolutions
    direction is the relative angle that we want to go to
    """
    originalAngle = currentAngle
    curRev = getRevolutions(currentAngle) * 360
    revCompensation = getRevolutions(currentAngle)
    currentAngle %= 360 ## now it is the angle relative to the revolution

    if direction == 0:
        
        revCompensation = roundToNearestRev(originalAngle)
    
    ## step down
    elif math.fabs(360 + currentAngle - direction) <= math.fabs(direction): ## (36)1 => 350

        revCompensation = curRev - 360

    ## if it is closer not to add revCompensation
    elif math.fabs(currentAngle - direction) < math.fabs(curRev - (direction + curRev)):

        revCompensation = curRev

    elif math.fabs(curRev - (direction + curRev)) <= math.fabs(curRev - direction):

        revCompensation = (revCompensation + 1) * 360

    else:
        revCompensation = curRev

    return revCompensation

def getOptimizedAngleAndMagnitude(currentAngle, direction, magnitude):
    """
    Gives us the optmized angle to travel to and correctly adjusted magnitude for a swerve module.
    """

    # This is the value that will be added to the angle to make a coterminal angle to limit distance
    # (coterminal = angle +/- 360)
    rev = giveRevCompensation(currentAngle, direction)

    # If direction is clockwise
    if direction < 0:
        opposAngle = direction + 180
        negAngle = 360 + direction # coterminal angle

    # If direction is counterclockwise
    elif direction > 0:
        opposAngle = direction - 180
        negAngle = direction - 360 # coterminal angle
        
    # If either direction works (aka angle == 0)
    else:
        if sign(direction) == -1:
            opposAngle = -180
            negAngle = 0
        else:
            opposAngle = 180
            negAngle = 0

    # If the negative angle (angle - 360) is shorter, we use the negAngle and add rev compensation
    if math.fabs(currentAngle - direction) >= math.fabs(currentAngle - negAngle):
        return (negAngle + rev), magnitude

    # If direction is closer than the opposite angle (the +/- 180 deg), we choose direction and add the rev comp
    elif math.fabs(currentAngle - direction) <= math.fabs(currentAngle - opposAngle):
        return (direction + rev), magnitude
    
    # Else, the opposite angle is the closest, then we choose the opposAngle and flip the magnitude
    else:
        return (opposAngle + rev), -magnitude

def sign(num) -> int:
    if num > 0:
        # positive
        return 1
    elif num < 0:
        # negative
        return -1
    else:
        # zero
        return 0

def convertJoystickInputToDegrees(x: float,
                                  y: float):  # this allows us to use the x and y values on the joystick and convert it into degrees
    if sign(x) == -1:
        return float(
            math.degrees(math.atan2(x, -y)) + 360.0)  # this will make sure that it gives us a number between 0 and 360
    else:
        if float(math.degrees(
                math.atan2(x, -y))) == 360.0:  # This makes sure that if we get 360.0 degrees, it will be zero
            return 0.0
        else:
            return float(math.degrees(
                math.atan2(x, -y)))  # the degrees, the joystick up is zero and the values increase clock-wise

def deadband(value: float, band: float):
    # this makes sure that joystick drifting is not an issue.
    # It takes the small values and forces it to be zero if smaller than the band value
    if math.fabs(value) <= band:
        return 0
    else:
        return value