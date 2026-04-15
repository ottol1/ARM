import math

def inverse_kinematics(x: float, y: float, z: float) -> list[float]:
    """
    Solves for inverse kinematics of ARM
    
    Inputs: target position (x, y, z) in mm
    Outputs: list of joint angles in RADIANS [theta1, theta2, theta3, theta4, theta5]
    
    If the target is outside the workspace (|D| > 1), ARM points to the object
    """
    # Link lengths
    d1 = 57.48
    a2 = 140.05
    d2 = 2.0
    a3 = 143.19
    d5 = 161.74

    # Desired wrist angle thetad (radians) in world frame
    radial = math.sqrt(x**2 + y**2)
    thetad = math.atan2(z - d1, radial)

    # Effective radial distance after base offset d2
    xy_dist2 = x**2 + y**2
    a = math.sqrt(xy_dist2 - d2**2)
    theta1 = math.atan2(y, x) - math.atan2(d2, a)

    # variables describing shoulder and elbow positions
    r = z - d1 - (d5) * math.sin(thetad)
    s = a - (d5) * math.cos(math.fabs(thetad))
    D = (s**2 + r**2 - a2**2 - a3**2) / (2 * a3 * a2)

    # Check for reachable solution
    if abs(D) > 1.0:
        # Outside workspace
        theta3 = 0.0
        theta2 = math.atan2(z-d1, radial)
        theta4 = 0.0
    else:
        # Inverse kinematics
        sqrt_term = math.sqrt(1.0 - D**2)
        theta3 = math.atan2(-sqrt_term, D)
        
        theta2_temp = math.atan2(r, s)
        atan_term = math.atan2(
            a3 * math.sin(theta3),
            a2 + a3 * math.cos(theta3)
        )
        
        theta2 = theta2_temp - atan_term
        
        theta4 = -theta2 - theta3 + thetad

    # theta5 is fixed at 0
    theta5 = 0.0
    
    theta2 = math.pi - theta2
    
    return [round(theta1, 4), 
            round(theta2, 4), 
            round(theta3, 4), 
            round(theta4, 4), 
            round(theta5, 4)]

