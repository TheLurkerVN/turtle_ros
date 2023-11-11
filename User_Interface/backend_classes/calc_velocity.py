BURGER_MAX_LIN_VEL = 0.20
BURGER_MAX_ANG_VEL = 2.80

#LIN_VEL_STEP_SIZE = 0.01
#ANG_VEL_STEP_SIZE = 0.1

def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))
    
def make_simple_profile(output, input, slop):
    output = input

    return output

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel

def check_linear_limit_velocity(velocity):
    return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

class CalcVel():
    def __init__(self):
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0

    def controlVel(self, dir, percentage):
        match(dir):
            case "up":
                self.target_linear_velocity =\
                BURGER_MAX_LIN_VEL * percentage
                #check_linear_limit_velocity(self.target_linear_velocity + LIN_VEL_STEP_SIZE)
                print_vels(self.target_linear_velocity, self.target_angular_velocity)
            case "down":
                self.target_linear_velocity =\
                BURGER_MAX_LIN_VEL * percentage * -1.0
                #check_linear_limit_velocity(self.target_linear_velocity - LIN_VEL_STEP_SIZE)
                print_vels(self.target_linear_velocity, self.target_angular_velocity)
            case "left":
                self.target_angular_velocity =\
                BURGER_MAX_ANG_VEL * percentage
                #check_angular_limit_velocity(self.target_angular_velocity + ANG_VEL_STEP_SIZE)
                print_vels(self.target_linear_velocity, self.target_angular_velocity)
            case "right":
                self.target_angular_velocity =\
                BURGER_MAX_ANG_VEL * percentage * -1.0
                #check_angular_limit_velocity(self.target_angular_velocity - ANG_VEL_STEP_SIZE)
                print_vels(self.target_linear_velocity, self.target_angular_velocity)
            case "stop":
                self.target_linear_velocity = 0.0
                self.control_linear_velocity = 0.0
                self.target_angular_velocity = 0.0
                self.control_angular_velocity = 0.0
                print("stop")
        
        #self.control_linear_velocity = make_simple_profile(
        #    self.control_linear_velocity,
        #    self.target_linear_velocity,
        #    (LIN_VEL_STEP_SIZE / 2.0))

        #self.control_angular_velocity = make_simple_profile(
        #    self.control_angular_velocity,
        #    self.target_angular_velocity,
        #    (ANG_VEL_STEP_SIZE / 2.0))
            
        control_param = (self.target_linear_velocity, self.target_angular_velocity)
        return control_param
