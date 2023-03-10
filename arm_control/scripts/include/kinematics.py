import math

class ur10_Kin:
  def __init__(self):
    #measurements in radians
    self.base_home = 0
    self.shoulder_home = 0
    self.elbow_home = 0
    self.wrist_tilt_home = -1.57
    self.wrist_rot_home = 0
    self.gripper_tilt_home = 1.57/2

    #measurements in mm
    self.base_height = 128
    self.forearm = 572
    self.upperarm = 612
    self.shoulder_offset = 176
    self.pipe_length = 0
    self.grip_ang_offset = math.atan(268/1300)
    self.end_effector_width = 92 # a
    self.end_effector_length = 116 
    self.end_effector_offset = 164 # d1

    self.prevX = 0
    self.prevY = 0
    self.prevZ = 0

    
  def hypotenuse(self, a, b):
    return math.sqrt((a*a)+(b*b))
  
  def cosineRule(self, a, b, c):
    return math.acos(((b*b) + (c*c) - (a*a))/(2*b*c))

  def solve_kinematics(self, x, y, z, theta5_offset, theta4_offset, debug_level = 0 ):
    self.prevX = x
    self.prevY = y
    self.prevZ = z
    alpha1 = math.atan2(y, x)
    end_effect_horizontal_offset = self.end_effector_width * math.cos(theta5_offset) #d3
    end_effect_vertical_offset = self.end_effector_width * math.sin(theta5_offset) #d4
    d2 = self.end_effector_offset + end_effect_horizontal_offset

    r1 = self.hypotenuse(x,y)
    r2 = math.sqrt((r1*r1)-(d2*d2))
    alpha1 = math.atan2(y, x)

    theta5 = -((math.pi/2 - (alpha1))- math.atan2(d2,r2))
    th5_off_vert = -116*math.tan(theta5)

    alpha2 = math.asin(d2/r1)
    theta1 = -(math.pi/2 - (alpha1 + alpha2))

    end_eff_offset_hypot = self.hypotenuse(self.end_effector_length,end_effect_vertical_offset) #a1
    beta1 = theta4_offset - math.atan2(end_effect_vertical_offset, self.end_effector_length)

    z_off = end_eff_offset_hypot*math.sin(beta1)
    xy_off = end_eff_offset_hypot*math.cos(beta1)
    r3 = r2-xy_off - th5_off_vert #r2'
    z1 = z-z_off #z'
    r5 = math.sqrt((r3*r3) + (z1*z1))


    omega1 = math.atan2(z1, r3)
    omega2 = self.cosineRule(self.forearm, self.upperarm, r5)
    theta2 = omega1 + omega2
    theta3 = self.cosineRule(r5, self.forearm, self.upperarm)
    phi = self.cosineRule(self.upperarm, r5, self.forearm)
    theta4_min = phi - omega1
    theta4 = theta4_offset + theta4_min
    
    if debug_level != 0:
      print("ee_hor: ", end_effect_horizontal_offset)
      print("ee_vert: ", end_effect_vertical_offset)
      print("eef_hyp: ", end_eff_offset_hypot)
      print("z1: ", z1)
      print("r3: ", r3)
      print("d2: ", d2)
      print("r2: ", r2)
      print("zoff: ", z_off)
      
      print("beta 1: ", (beta1/math.pi)*180)
      print("theta 4_min: ", (theta4_min/math.pi)*180)
      
      print("theta 1: ", (theta1/math.pi)*180)
      print("theta 2: ", (theta2/math.pi)*180)
      print("theta 3: ", (theta3/math.pi)*180)
      print("theta 4: ", (theta4/math.pi)*180)

    
    return theta1, theta2, theta3, theta4, theta5


