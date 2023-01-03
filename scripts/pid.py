class PID:
    def __init__(self):
        self.kp = 0
        self.ki = 0
        self.kd = 0

        self.prev_e = 0.0
        self.prev_i = 0.0
        
        self.step = 0.02

    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_step(self, step):
        self.step = step

    def reset_controller(self):
        self.prev_e = 0.0
        self.prev_i = 0.0 

    def control(self, e, x_dot = None, bias = 0.0):
        
        # e =  x_desired - x               #Error between the real and desired value
        P = self.kp * e                           #Proportional controller 
        if x_dot == None : 
            D = self.kd * ((e - self.prev_e)/ self.step)
        else : 
            D = self.kd *x_dot

        I = self.prev_i +  e * self.step                                                 #Integral controller
        control_effort = P + self.ki * I + D + bias                     #Output of the PID controller 
        self.prev_e = e
        self.prev_i = I                                                             #Update the initial value of integral controller 

        return control_effort 
        